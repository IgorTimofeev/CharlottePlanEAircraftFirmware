#include "systems/ADIRS/I2CADIRS.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "utilities/math.h"
#include "aircraft.h"
#include "config.h"

namespace pizda {
	void I2CADIRS::setup() {
		const auto& ac = Aircraft::getInstance();

		if (!setupIMUs())
			return;

		// Updating IMU biases from settings
		for (size_t ADIRUIndex = 0; ADIRUIndex < config::ADIRS::unitCount; ++ADIRUIndex) {
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

	void I2CADIRS::setHomeCoordinates(const GeoCoordinates& homeCoordinates) {
		ADIRS::setHomeCoordinates(homeCoordinates);

		for (auto& IMU : _IMUs) {
			IMU.unit.resetIntegratedCoordinates();
		}
	}

	void I2CADIRS::onCalibrateAccelAndGyro() {
		ESP_LOGI(_logTag, "Accel & gyro calibration started");

		auto& ac = Aircraft::getInstance();

		constexpr static uint16_t iterations = 5'000;

		for (size_t ADIRUIndex = 0; ADIRUIndex < config::ADIRS::unitCount; ++ADIRUIndex) {
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
				ac.transceiver.enqueueSystemPacket(AircraftSystemPacketType::calibration);

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
		ac.transceiver.enqueueSystemPacket(AircraftSystemPacketType::calibration);

		ESP_LOGI(_logTag, "accel & gyro calibration finished");
	}

	void I2CADIRS::onCalibrateMag() {
		ESP_LOGI(_logTag, "mag calibration started");

		auto& ac = Aircraft::getInstance();

		constexpr static uint16_t iterations = 2'000;

		for (size_t ADIRUIndex = 0; ADIRUIndex < config::ADIRS::unitCount; ++ADIRUIndex) {
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
				ac.transceiver.enqueueSystemPacket(AircraftSystemPacketType::calibration);

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
		ac.transceiver.enqueueSystemPacket(AircraftSystemPacketType::calibration);

		ESP_LOGI(_logTag, "mag calibration finished");
	}

	void I2CADIRS::onTick() {
		updateIMUs();
		updateBMPs();

		vTaskDelay(pdMS_TO_TICKS(std::max(IMU::commonTickIntervalUs / 1000, portTICK_PERIOD_MS)));
	}

	bool I2CADIRS::setupIMUs() {
		const auto& ac = Aircraft::getInstance();

		for (uint8_t i = 0; i < static_cast<uint8_t>(_IMUs.size()); ++i) {
			auto& IMU = _IMUs[i];

			IMU.hal.setup(ac.I2CMasterBusHandle, IMU.address, 400'000);

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
		float integratedVelocityMsSum = 0;
		Vector3F accelerationGSum {};
		float integratedLatitudeRadSum = 0;
		float integratedLongitudeRadSum = 0;

		for (auto& IMU : _IMUs) {
			IMU.unit.tick();

			rollRadSum += IMU.unit.getRollRad();
			pitchRadSum += IMU.unit.getPitchRad();
			yawRadSum += IMU.unit.getYawRad();

			integratedVelocityMsSum += IMU.unit.getIntegratedVelocityMs();
			accelerationGSum += IMU.unit.getAccelerationG();

			integratedLatitudeRadSum += IMU.unit.getIntegratedLatitudeRad();
			integratedLongitudeRadSum += IMU.unit.getIntegratedLongitudeRad();
		}

		// Roll / pitch / yaw
		setRollRad(rollRadSum / _IMUs.size());
		setPitchRad(pitchRadSum / _IMUs.size());
		setYawRad(yawRadSum / _IMUs.size() + toRadians(ac.settings.adirs.magneticDeclinationDeg));
		updateHeadingFromYaw();

		// Velocity
		integratedVelocityMsSum /= _IMUs.size();
		setAirspeedMs(std::abs(integratedVelocityMsSum));

		// ESP_LOGI("aefa","vel: %f", integratedVelocityMsSum);

		// Coordinates
		integratedLatitudeRadSum /= _IMUs.size();
		integratedLongitudeRadSum /= _IMUs.size();

		const auto& homeCoordinates = getHomeCoordinates();
		setLatitude(homeCoordinates.getLatitude() + integratedLatitudeRadSum);
		setLongitude(homeCoordinates.getLongitude() + integratedLongitudeRadSum);

		// Slip & skid
		const auto accelerationG = accelerationGSum / _IMUs.size();
		updateSlipAndSkidFactor(accelerationG.getX(), IMU::accelOperationalRangeG);
	}

	bool I2CADIRS::setupBMPs() {
		const auto& ac = Aircraft::getInstance();

		for (uint8_t i = 0; i < static_cast<uint8_t>(_BMPs.size()); ++i) {
			auto& BMP = _BMPs[i];

			BMP.hal.setup(ac.I2CMasterBusHandle, BMP.address, 1'000'000);

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
