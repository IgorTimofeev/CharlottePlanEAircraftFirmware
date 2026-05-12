#include "systems/ADIRS/I2CADIRS.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "utilities/math.h"
#include "aircraft.h"
#include "config.h"

namespace pizda {
	void I2CADIRS::setup() {
		const auto& ac = Aircraft::getInstance();

		if (!setupBus())
			return;

		if (!setupIMUs())
			return;

		// Updating IMU biases from settings
		for (size_t ADIRUIndex = 0; ADIRUIndex < config::adirs::unitCount; ++ADIRUIndex) {
			auto& IMU = _IMUs[ADIRUIndex].unit;
			auto& settingsUnit = ac.settings.adirs.units[ADIRUIndex];

			IMU.setAccelBias(settingsUnit.accelBias);
			IMU.setGyroBias(settingsUnit.gyroBias);
			IMU.setMagBias(settingsUnit.magBias);
		}

		if (!setupBMPs())
			return;

		ADIRS::setup();
	}

	void I2CADIRS::onCalibrateAccelAndGyro() {
		ESP_LOGI(_logTag, "Accel & gyro calibration started");

		auto& ac = Aircraft::getInstance();

		constexpr static uint16_t iterations = 5'000;

		for (size_t ADIRUIndex = 0; ADIRUIndex < config::adirs::unitCount; ++ADIRUIndex) {
			auto& IMU = _IMUs[ADIRUIndex].unit;

			// Setting calibration attenuation
			IMU.setCalibrationMode();

			Vector3F aSum {};
			Vector3F gSum {};

			for (uint16_t i = 0; i < iterations; ++i) {
				// Accumulating samples
				aSum += IMU.getRawAccelData();
				gSum += IMU.getRawGyroData();

				// Reporting progress
				ac.aircraftData.calibration.progress = static_cast<uint8_t>(static_cast<uint32_t>(i) * 0xFF / iterations);
				ac.transceiver.enqueueAuxiliary(AircraftAuxiliaryPacketType::calibration);

				vTaskDelay(pdMS_TO_TICKS(std::max(IMU::MPUSampleIntervalHz / 1000, portTICK_PERIOD_MS)));
			}

			aSum /= iterations;
			// Z axis - 1G
			aSum.setZ(aSum.getZ() - 1);

			gSum /= iterations;

			auto& settingsUnit = ac.settings.adirs.units[ADIRUIndex];
			settingsUnit.accelBias = aSum;
			settingsUnit.gyroBias = gSum;
			ac.settings.adirs.scheduleWrite();

			IMU.setAccelBias(settingsUnit.accelBias);
			IMU.setGyroBias(settingsUnit.gyroBias);

			// Restoring attenuation to operational
			IMU.setOperationalMode();
		}

		// Reporting progress once more
		ac.aircraftData.calibration.progress = 0xFF;
		ac.transceiver.enqueueAuxiliary(AircraftAuxiliaryPacketType::calibration);

		ESP_LOGI(_logTag, "accel & gyro calibration finished");
	}

	void I2CADIRS::onCalibrateMag() {
		ESP_LOGI(_logTag, "mag calibration started");

		auto& ac = Aircraft::getInstance();

		constexpr static uint16_t iterations = 2'000;

		for (size_t ADIRUIndex = 0; ADIRUIndex < config::adirs::unitCount; ++ADIRUIndex) {
			auto& IMU = _IMUs[ADIRUIndex].unit;

			// Setting calibration attenuation
			IMU.setCalibrationMode();

			Vector3F min {};
			Vector3F max {};

			for (uint16_t i = 0; i < iterations; ++i) {
				const auto magData = IMU.getRawMagData();

				min = min.min(magData);
				max = max.max(magData);

				// Reporting progress
				ac.aircraftData.calibration.progress = static_cast<uint8_t>(static_cast<uint32_t>(i) * 0xFF / iterations);
				ac.transceiver.enqueueAuxiliary(AircraftAuxiliaryPacketType::calibration);

				vTaskDelay(pdMS_TO_TICKS(std::max(IMU::magTickIntervalUs / 1000, portTICK_PERIOD_MS)));
			}

			auto& settingsUnit = ac.settings.adirs.units[ADIRUIndex];
			settingsUnit.magBias = min + (max - min) / 2;
			ac.settings.adirs.scheduleWrite();

			IMU.setMagBias(settingsUnit.magBias);

			// Restoring attenuation to operational
			IMU.setOperationalMode();
		}

		// Reporting progress once more
		ac.aircraftData.calibration.progress = 0xFF;
		ac.transceiver.enqueueAuxiliary(AircraftAuxiliaryPacketType::calibration);

		ESP_LOGI(_logTag, "mag calibration finished");
	}

	void I2CADIRS::onTick() {
		updateIMUs();
		updateBMPs();

		vTaskDelay(pdMS_TO_TICKS(std::max(IMU::commonTickIntervalUs / 1000, portTICK_PERIOD_MS)));
	}

	bool I2CADIRS::setupBus() {
		i2c_master_bus_config_t i2c_mst_config {};
		i2c_mst_config.clk_source = I2C_CLK_SRC_DEFAULT;
		i2c_mst_config.i2c_port = I2C_NUM_0;
		i2c_mst_config.scl_io_num = config::I2C::SCL;
		i2c_mst_config.sda_io_num = config::I2C::SDA;
		i2c_mst_config.glitch_ignore_cnt = 7;
		i2c_mst_config.flags.enable_internal_pullup = true;

		const auto state = i2c_new_master_bus(&i2c_mst_config, &_I2CBusHandle);

		if (state != ESP_OK && state != ESP_ERR_INVALID_STATE) {
			ESP_ERROR_CHECK(state);
			return false;
		}

		return true;
	}

	bool I2CADIRS::setupIMUs() {
		for (uint8_t i = 0; i < static_cast<uint8_t>(_IMUs.size()); ++i) {
			auto& IMU = _IMUs[i];

			IMU.hal.setup(_I2CBusHandle, IMU.address, 400'000);

			if (!IMU.unit.setup(&IMU.hal)) {
				ESP_LOGE(_logTag, "IMU %d initialization failed", i);
				return false;
			}
		}

		return true;
	}

	void I2CADIRS::updateIMUs() {
		const auto& ac = Aircraft::getInstance();

		float rollRadSum = 0;
		float pitchRadSum = 0;
		float yawRadSum = 0;
		Vector3F accelerationGSum {};
		Vector3F integratedVelocityMsSum {};
		Vector3F integratedPositionMSum {};

		for (auto& IMU : _IMUs) {
			IMU.unit.tick();

			rollRadSum += IMU.unit.getRollRad();
			pitchRadSum += IMU.unit.getPitchRad();
			yawRadSum += IMU.unit.getYawRad();
			accelerationGSum += IMU.unit.getAccelerationG();
			integratedVelocityMsSum += IMU.unit.getIntegratedVelocityMs();
			integratedPositionMSum += IMU.unit.getIntegratedPositionM();
		}

		// Roll / pitch / yaw
		setRollRad(rollRadSum / _IMUs.size());
		setPitchRad(pitchRadSum / _IMUs.size());
		setYawRad(yawRadSum / _IMUs.size() + toRadians(ac.settings.adirs.magneticDeclinationDeg));
		updateHeadingFromYaw();

		// Velocity
		integratedVelocityMsSum /= _IMUs.size();
		setIntegratedVelocityMPS(integratedVelocityMsSum);

		// Airspeed
		setAirspeedMPS(std::max<float>(integratedVelocityMsSum.getY(), 0));

		// Coordinates
		integratedPositionMSum /= _IMUs.size();

		// Slip & skid
		const auto accelerationG = accelerationGSum / _IMUs.size();
		updateSlipAndSkidFactor(accelerationG.getX(), IMU::accelOperationalRangeG);
	}

	bool I2CADIRS::setupBMPs() {
		for (uint8_t i = 0; i < static_cast<uint8_t>(_BMPs.size()); ++i) {
			auto& BMP = _BMPs[i];

			BMP.hal.setup(_I2CBusHandle, BMP.address, 1'000'000);

			if (!BMP.unit.setup(
				&BMP.hal,

				BMP280Mode::normal,
				BMP280Oversampling::x16,
				BMP280Oversampling::x2,
				BMP280Filter::x4,
				BMP280StandbyDuration::ms125
			)) {
				ESP_LOGI(_logTag, "BMP %d initialization failed", i);
				return false;
			}
		}

		return true;
	}

	void I2CADIRS::updateBMPs() {
		float pressureSum = 0;
		float temperatureSum = 0;

		float pressure;
		float temperature;

		for (auto& BMP : _BMPs) {
			BMP.unit.readPressureAndTemperature(pressure, temperature);

			pressureSum += pressure;
			temperatureSum += temperature;
		}

		setPressurePa(pressureSum / _BMPs.size());
		setTemperatureC(temperatureSum / _BMPs.size());
		updateAltitudeFromPressureTemperatureAndReferenceValue();

		//				ESP_LOGI(_logTag, "Avg press: %f, temp: %f, alt: %f", _pressure, _temperature, _altitude);
	}
}
