#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_timer.h>

#include <EMAFilter.hpp>

#include "Systems/ADIRS/I2CADIRS.hpp"
#include "Utilities/Math.hpp"
#include "Aircraft.hpp"
#include "Config.hpp"

namespace pizda {
	void I2CADIRS::setup() {
		xTaskCreatePinnedToCore(
			[](void* arg) {
				static_cast<I2CADIRS*>(arg)->onIMUStart();
			},
			"I2CADIRSIMU",
			4 * 1024,
			this,
			10,
			nullptr,
			0
		);

		xTaskCreatePinnedToCore(
			[](void* arg) {
				static_cast<I2CADIRS*>(arg)->onBarometerStart();
			},
			"I2CADIRSIMUBarometer",
			4 * 1024,
			this,
			16,
			nullptr,
			0
		);

		xTaskCreatePinnedToCore(
			[](void* arg) {
				static_cast<I2CADIRS*>(arg)->onAirspeedStart();
			},
			"I2CADIRSAirspeed",
			4 * 1024,
			this,
			16,
			nullptr,
			0
		);
	}

	void I2CADIRS::setHomeCoordinates(const float latitude, const float longitude, const float altitude) {
		ADIRS::setHomeCoordinates(latitude, longitude, altitude);

		_IMU.unit.resetIntegratedCoordinates();
	}

	void I2CADIRS::onCalibrateAccelAndGyro() {
		ESP_LOGI(_logTag, "Accel & gyro calibration started");

		auto& ac = Aircraft::getInstance();

		constexpr static uint16_t iterations = 1'000;

		// Setting calibration attenuation
		_IMU.unit.setCalibrationMode();

		Vector3F aSum {};
		Vector3F gSum {};
		uint32_t XCVRPacketTime = 0;

		for (uint16_t i = 0; i < iterations; ++i) {
			// Accumulating samples
			aSum += _IMU.unit.getRawAccelData();
			gSum += _IMU.unit.getRawGyroData();

			// Reporting progress
			if (esp_timer_get_time() >= XCVRPacketTime) {
				ac.aircraftData.calibration.setProgress(static_cast<uint8_t>(static_cast<uint32_t>(i) * 0xFF / iterations));
				ac.transceiver.enqueueSystemPacket(AircraftSystemPacketType::calibration);

				XCVRPacketTime = esp_timer_get_time() + _calibrationXCVRPacketIntervalUs;
			}

			vTaskDelay(pdMS_TO_TICKS(std::max(IMU::MPUSampleIntervalHz / 1000, portTICK_PERIOD_MS)));
		}

		aSum /= iterations;
		// Z axis - 1G
		aSum.setZ(aSum.getZ() - 1);

		gSum /= iterations;

		// Updating settings
		ac.settings.ADIRS.setAccelBias(aSum);
		ac.settings.ADIRS.setGyroBias(gSum);
		ac.settings.ADIRS.writeLater();

		// Updating unit
		_IMU.unit.setAccelBias(aSum);
		_IMU.unit.setGyroBias(gSum);

		// Restoring attenuation to operational
		_IMU.unit.setOperationalMode();

		// Reporting progress once more
		ac.aircraftData.calibration.setProgress(0xFF);
		ac.transceiver.enqueueSystemPacket(AircraftSystemPacketType::calibration);

		ESP_LOGI(_logTag, "accel & gyro calibration finished");
	}

	void I2CADIRS::onCalibrateMag() {
		ESP_LOGI(_logTag, "mag calibration started");

		auto& ac = Aircraft::getInstance();

		constexpr static uint16_t iterations = 2'000;

		// Setting calibration attenuation
		_IMU.unit.setCalibrationMode();

		Vector3F min {};
		Vector3F max {};
		uint32_t XCVRPacketTime = 0;

		for (uint16_t i = 0; i < iterations; ++i) {
			const auto magData = _IMU.unit.getRawMagData();

			min = min.min(magData);
			max = max.max(magData);

			// Reporting progress
			if (esp_timer_get_time() >= XCVRPacketTime) {
				ac.aircraftData.calibration.setProgress(static_cast<uint8_t>(static_cast<uint32_t>(i) * 0xFF / iterations));
				ac.transceiver.enqueueSystemPacket(AircraftSystemPacketType::calibration);

				XCVRPacketTime = esp_timer_get_time() + _calibrationXCVRPacketIntervalUs;
			}

			vTaskDelay(pdMS_TO_TICKS(std::max(IMU::magTickIntervalUs / 1000, portTICK_PERIOD_MS)));
		}

		const auto bias = min + (max - min) / 2;

		// Updating settings
		ac.settings.ADIRS.setMagBias(bias);
		ac.settings.ADIRS.writeLater();

		// Updating unit
		_IMU.unit.setMagBias(bias);

		// Restoring attenuation to operational
		_IMU.unit.setOperationalMode();

		// Reporting progress once more
		ac.aircraftData.calibration.setProgress(0xFF);
		ac.transceiver.enqueueSystemPacket(AircraftSystemPacketType::calibration);

		ESP_LOGI(_logTag, "mag calibration finished");
	}

	void I2CADIRS::onIMUTick() {
		_IMU.unit.tick();

		// Roll / pitch / yaw
		setRollRad(_IMU.unit.getRollRad());
		setPitchRad(_IMU.unit.getPitchRad());
		setYawRad( _IMU.unit.getYawRad());
		updateHeadingFromYaw();

		// Velocity
		// setAirspeedMS(std::abs(_IMU.unit.getIntegratedVelocityMPS()));

		// ESP_LOGI("aefa","vel: %f", integratedVelocityMsSum);

		// Coordinates
		setLatitude(getHomeLatitude() + _IMU.unit.getIntegratedLatitudeRad());
		setLongitude(getHomeLongitude() + _IMU.unit.getIntegratedLongitudeRad());

		// Slip & skid
		updateSlipAndSkidFactor(_IMU.unit.getAccelerationG().getX(), IMU::accelOperationalRangeG);
	}

	void I2CADIRS::onIMUStart() {
		auto& ac = Aircraft::getInstance();
		const auto system = ac.aircraftData.calibration.getSystem();

		_IMU.hal.setup(
			ac.I2CMasterBusHandle,
			_IMU.address,
			_IMU.frequencyHz
		);

		if (!_IMU.unit.setup(&_IMU.hal)) {
			ESP_LOGE(_logTag, "IMU initialization failed");
			return;
		}

		// Updating biases from settings
		_IMU.unit.setAccelBias(ac.settings.ADIRS.getAccelBias());
		_IMU.unit.setGyroBias(ac.settings.ADIRS.getGyroBias());
		_IMU.unit.setMagBias(ac.settings.ADIRS.getMagBias());

		while (true) {
			// Calibration mode
			if (
				ac.aircraftData.calibration.isCalibrating()
				&& (
					system == AircraftCalibrationSystem::accelAndGyro
					|| system == AircraftCalibrationSystem::mag
				)
			) {
				if (system == AircraftCalibrationSystem::accelAndGyro) {
					onCalibrateAccelAndGyro();
				}
				else {
					onCalibrateMag();
				}

				ac.aircraftData.calibration.setCalibrating(false);
			}
			// Normal mode
			else {
				onIMUTick();

				vTaskDelay(_IMUTickIntervalTicks);
			}
		}
	}

	void I2CADIRS::onBarometerTick() {
		float pressure, temperature;
		_barometer.unit.readPressureAndTemperature(pressure, temperature);

		setPressurePa(pressure);
		setTemperatureC(temperature);
		updateAltitudeFromPressureTemperatureAndReferenceValue();

		//				ESP_LOGI(_logTag, "Avg press: %f, temp: %f, alt: %f", _pressure, _temperature, _altitude);

	}

	void I2CADIRS::onBarometerStart() {
		const auto& ac = Aircraft::getInstance();

		_barometer.hal.setup(
			ac.I2CMasterBusHandle,
			_barometer.address,
			_barometer.frequencyHz
		);

		if (!_barometer.unit.setup(
			&_barometer.hal,

			BMP280Mode::normal,
			BMP280Oversampling::x4,
			BMP280Oversampling::x2,
			BMP280Filter::x4,
			BMP280StandbyDuration::ms63
		)) {
			ESP_LOGI(_logTag, "barometer initialization failed");
			return;
		}

		while (true) {
			onBarometerTick();

			vTaskDelay(_barometerTickIntervalTicks);
		}
	}

	void I2CADIRS::onAirspeedTick() {
		// Reading diff pressure & temperature
		float differentialPressurePSI, temperatureC;
		const auto error = _airspeedSensor.readDifferentialPressureAndTemperature(differentialPressurePSI, temperatureC);
		checkMS4525Error("failed to read MS4525 diff pressure & temperature", error);

		// Computing IAS using diff pressure
		auto IASMPS = MS4525::computeIndicatedAirspeedMPS(differentialPressurePSI);

		// Rejecting too low values
		if (IASMPS < config::ADIRS::MS4525::minValidAirspeedThresholdMPS)
			IASMPS = 0;

		// Applying EMA filter
		setAirspeedMPS(EMAFilter::apply(getAirspeedMPS(), IASMPS, config::ADIRS::MS4525::airspeedEMAFilterFactor));
	}

	void I2CADIRS::onAirspeedStart() {
		const auto& ac = Aircraft::getInstance();

		// Initializing
		auto MS4525Error = _airspeedSensor.setup(
			ac.I2CMasterBusHandle,
			MS4525::defaultI2CAddress,
			config::ADIRS::MS4525::I2CFrequencyHz,

			MS4525OutputType::a,
			-1,
			1
		);

		if (!checkMS4525Error("MS4525 initialization failed", MS4525Error))
			return;

		// Computing meadian differential pressure bias
		float diffPressureBias = 0;
		MS4525Error = _airspeedSensor.computeMedianDifferentialPressureBias<200, MS4525::defaultSampleRateHz>(diffPressureBias);

		if (!checkMS4525Error("failed to compute MS4525 median diff pressure bias", MS4525Error))
			return;

		// Setting computed bias
		_airspeedSensor.setDifferentialPressureBias(diffPressureBias);

		// ESP_LOGI(_logTag, "MS4525 differential pressure bias: %f", diffPressureBias);

		while (true) {
			// Normal mode
			if (!ac.aircraftData.calibration.isCalibrating()) {
				onAirspeedTick();
			}

			vTaskDelay(_airspeedTickIntervalTicks);
		}
	}

	bool I2CADIRS::checkMS4525Error(const char* prefix, const MS4525Error error) {
		if (error == MS4525Error::none)
			return true;

		ESP_LOGE(_logTag, "%s: %s", prefix, MS4525::errorToString(error));

		return false;
	}
}
