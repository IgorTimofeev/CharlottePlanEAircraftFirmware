#include "systems/ADIRS/I2CADIRS.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_timer.h>

#include <EMAFilter.h>

#include "utilities/math.h"
#include "aircraft.h"
#include "config.h"

namespace pizda {
	void I2CADIRS::setup() {
		const auto& ac = Aircraft::getInstance();

		// IMU
		{
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
			_IMU.unit.setAccelBias(ac.settings.adirs.getAccelBias());
			_IMU.unit.setGyroBias(ac.settings.adirs.getGyroBias());
			_IMU.unit.setMagBias(ac.settings.adirs.getMagBias());
		}

		// BMP280
		{
			_BMP280.hal.setup(
				ac.I2CMasterBusHandle,
				_BMP280.address,
				_BMP280.frequencyHz
			);

			if (!_BMP280.unit.setup(
				&_BMP280.hal,

				BMP280Mode::normal,
				BMP280Oversampling::x16,
				BMP280Oversampling::x2,
				BMP280Filter::x4,
				BMP280StandbyDuration::ms125
			)) {
				ESP_LOGI(_logTag, "BMP280 initialization failed");
				return;
			}
		}

		// Airspeed sensor
		{
			// Initializing
			auto MS4525Error = _MS4525.setup(
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
			MS4525Error = _MS4525.computeMedianDifferentialPressureBias<20, MS4525::defaultSampleRateHz>(diffPressureBias);

			if (!checkMS4525Error("failed to compute MS4525 median diff pressure bias", MS4525Error))
				return;

			// Setting computed bias
			_MS4525.setDifferentialPressureBias(diffPressureBias);

			// ESP_LOGI(_logTag, "MS4525 differential pressure bias: %f", diffPressureBias);
		}

		ADIRS::setup();
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
		ac.settings.adirs.setAccelBias(aSum);
		ac.settings.adirs.setGyroBias(gSum);
		ac.settings.adirs.writeLater();

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
		ac.settings.adirs.setMagBias(bias);
		ac.settings.adirs.writeLater();

		// Updating unit
		_IMU.unit.setMagBias(bias);

		// Restoring attenuation to operational
		_IMU.unit.setOperationalMode();

		// Reporting progress once more
		ac.aircraftData.calibration.setProgress(0xFF);
		ac.transceiver.enqueueSystemPacket(AircraftSystemPacketType::calibration);

		ESP_LOGI(_logTag, "mag calibration finished");
	}

	void I2CADIRS::onTick() {
		IMUTick();
		BMP280Tick();
		MS4525Tick();

		vTaskDelay(_minTickIntervalTicks);
	}

	void I2CADIRS::IMUTick() {
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

	void I2CADIRS::BMP280Tick() {
		float pressure, temperature;
		_BMP280.unit.readPressureAndTemperature(pressure, temperature);

		setPressurePa(pressure);
		setTemperatureC(temperature);
		updateAltitudeFromPressureTemperatureAndReferenceValue();

		//				ESP_LOGI(_logTag, "Avg press: %f, temp: %f, alt: %f", _pressure, _temperature, _altitude);
	}

	bool I2CADIRS::checkMS4525Error(const char* prefix, const MS4525Error error) {
		if (error == MS4525Error::none)
			return true;

		ESP_LOGE(_logTag, "%s: %s", prefix, MS4525::errorToString(error));

		return false;
	}

	void I2CADIRS::MS4525Tick() {
		if (esp_timer_get_time() < _MS4525TickTimeUs)
			return;

		_MS4525TickTimeUs = esp_timer_get_time() + _MS4525TickIntervalUs;

		// Reading diff pressure & temperature
		float differentialPressurePSI, temperatureC;
		const auto error = _MS4525.readDifferentialPressureAndTemperature(differentialPressurePSI, temperatureC);
		checkMS4525Error("failed to read MS4525 diff pressure & temperature", error);

		// Computing IAS using diff pressure
		auto IASMPS = MS4525::computeIndicatedAirspeedMPS(differentialPressurePSI);

		// Rejecting too low values
		if (IASMPS < config::ADIRS::MS4525::minValidAirspeedThresholdMPS)
			IASMPS = 0;

		// Applying EMA filter
		setAirspeedMPS(EMAFilter::apply(getAirspeedMPS(), IASMPS, config::ADIRS::MS4525::airspeedEMAFilterFactor));
	}
}
