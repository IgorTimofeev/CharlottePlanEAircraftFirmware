#include "IMU.h"

#include <cmath>
#include <algorithm>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_timer.h>

#include <MPU9250.h>
#include <lowPassFilter.h>
#include <units.h>

#include "aircraft.h"
#include "geoCoordinates.h"
#include "systems/ADIRS/adaptiveComplimentaryFilter.h"
#include "utilities/math.h"

namespace pizda {
    bool IMU::setup(BusHAL* bus) {
        if (!_MPU.setup(bus))
            return false;

        // SRD
        _MPU.setSRD(MPUSampleRateDivider);

        setOperationalMode();

        return true;
    }

    void IMU::tick()  {
        magTick();
        FIFOTick();
    }

    float IMU::getRollRad() const {
        return _rollRad;
    }

	float IMU::getPitchRad() const {
		return _pitchRad;
	}

	float IMU::getYawRad() const {
		return _yawRad;
	}

	const Vector3F& IMU::getAccelerationG() const {
		return _accelerationG;
	}

	float IMU::getIntegratedLatitudeRad() const {
		return _integratedLatitudeRad;
	}

	float IMU::getIntegratedLongitudeRad() const {
    	return _integratedLongitudeRad;
    }

	float IMU::getIntegratedVelocityMPS() const {
		return _integratedVelocityMPS;
	}

	void IMU::resetIntegratedCoordinates() {
    	_integratedTiltCompensatedVelocityMPS = {};
		_integratedAlignedPositionM = {};

    	_integratedVelocityMPS = 0;
    	_integratedLongitudeRad = 0;
    	_integratedLongitudeRad = 0;
	}

	void IMU::setCalibrationMode() {
		_MPU.setGyroRange(MPU9250_GYRO_RANGE_250);
		_MPU.setAccelRange(MPU9250_ACC_RANGE_2G);

		_MPU.setAccelDLPF(MPU9250_DLPF_6);
		_MPU.enableAccelDLPF();

		_MPU.setGyroDLPF(MPU9250_DLPF_6);
		_MPU.enableGyroDLPF();

		vTaskDelay(pdMS_TO_TICKS(100));

		_MPU.disableFIFO();
	}

	void IMU::setOperationalMode() {
		// Range
		_MPU.setAccelRange(accelOperationalRange);
		_MPU.setGyroRange(gyroOperationalRange);

		// LPF
		_MPU.setAccelDLPF(accelDLPF);
		_MPU.enableAccelDLPF();

		_MPU.setGyroDLPF(gyroDLPF);
		_MPU.enableGyroDLPF();

		vTaskDelay(pdMS_TO_TICKS(100));

		// FIFO
		_MPU.setFIFOMode(MPU9250_STOP_WHEN_FULL);
		_MPU.enableFIFO();

		// In some cases a delay after enabling FIFO makes sense
		vTaskDelay(pdMS_TO_TICKS(100));

		_MPU.setFIFODataSource(FIFODataSource);
	}

	Vector3F IMU::getRawAccelData() const {
		float x, y, z;
		_MPU.readAccelData(x, y, z);

		return { x, y, z };
	}

	Vector3F IMU::getRawGyroData() const {
		float x, y, z;
		_MPU.readGyroData(x, y, z);

		return { x, y, z };
	}

	Vector3F IMU::getRawMagData() const {
		float x, y, z;
		_MPU.readMagData(x, y, z);

		return { x, y, z };
	}

	const Vector3F& IMU::getAccelBias() const {
		return _accelBias;
	}

	void IMU::setAccelBias(const Vector3F& value) {
		_accelBias = value;

		ESP_LOGI(_logTag, "acc bias: %f x %f x %f", _accelBias.getX(), _accelBias.getY(), _accelBias.getZ());
	}

	const Vector3F& IMU::getGyroBias() const {
		return _gyroBias;
	}

	void IMU::setGyroBias(const Vector3F& value) {
		_gyroBias = value;

		ESP_LOGI(_logTag, "gyro bias: %f x %f x %f", _gyroBias.getX(), _gyroBias.getY(), _gyroBias.getZ());
	}

	const Vector3F& IMU::getMagBias() const {
		return _magBias;
	}

	void IMU::setMagBias(const Vector3F& value) {
		_magBias = value;

		ESP_LOGI(_logTag, "mag bias: %f x %f x %f", _magBias.getX(), _magBias.getY(), _magBias.getZ());
	}

	void IMU::magTick() {
		const auto deltaTime = esp_timer_get_time() - _magTickTimeUs;

		if (deltaTime < magTickIntervalUs)
			return;

		_magTickTimeUs = esp_timer_get_time();

		float x, y, z;
		_MPU.readMagData(x, y, z);

		//					ESP_LOGI(_logTag, "mag raw: %f x %f x %f", magData.getX(), magData.getY(), magData.getZ());

		// Axis swap, fuck MPU
		// Also applying LPF because mag is noisy as shit
		constexpr static float magLPFFactorPerSecond = 2.f;
		const auto magLPFFactor = magLPFFactorPerSecond * static_cast<float>(deltaTime) / 1'000'000.f;
		_magDataFiltered.setX(LowPassFilter::apply(_magDataFiltered.getX(), y - _magBias.getY(), magLPFFactor));
		_magDataFiltered.setY(LowPassFilter::apply(_magDataFiltered.getY(), x - _magBias.getX(), magLPFFactor));
		_magDataFiltered.setZ(LowPassFilter::apply(_magDataFiltered.getZ(), -(z - _magBias.getZ()), magLPFFactor));

		// _magDataFiltered.setX(y - _magBias.getY());
		// _magDataFiltered.setY(x - _magBias.getX());
		// _magDataFiltered.setZ(-(z - _magBias.getZ()));

		// const auto magYaw = std::atan2(_magDataFiltered.getX(), _magDataFiltered.getY());
		// ESP_LOGI(_logTag, "Mag yaw: %f", toDegrees(magYaw));
	}

	void IMU::FIFOTick() {
		if (esp_timer_get_time() < _FIFOTickTimeUs)
			return;

		const auto sampleCount = _MPU.getFIFOCount() / FIFOSampleLength;

		// ESP_LOGI(_logTag, "FIFO sample count %d", sampleCount);

		if (sampleCount < 2) {
			ESP_LOGW(_logTag, "FIFO sample count %d is not enough, skipping for more data", sampleCount);
			return;
		}
		else if (sampleCount > FIFOMaxSampleCount) {
			ESP_LOGW(_logTag, "FIFO sample count %d exceeds max sample count %d, data was permanently lost", sampleCount, FIFOMaxSampleCount);
		}

		_MPU.setFIFODataSource(MPU9250_FIFO_DATA_SOURCE_NONE);

		// ESP_LOGI(_logTag, "FIFO sample count: %d", sampleCount);

		uint8_t sample[FIFOSampleLength] {};
		Vector3F accelerationGSum {};
		float x, y, z;

		for (uint32_t i = 0; i < sampleCount; i++) {
			// -------------------------------- FIFO data --------------------------------

			// Reading sample
			_MPU.readFIFOData(sample, FIFOSampleLength);

			// Extracting accel vector
			_MPU.getAccelData(sample, x, y, z);
			const auto accelData = Vector3F(x, y, z) - _accelBias;

			// Extracting gyr vector
			_MPU.getGyroData(sample + FIFOSampleDataTypeLength, x, y, z);
			const auto gyroData = Vector3F(x, y, z) - _gyroBias;

			// -------------------------------- RPY angles --------------------------------

			// Performing sensor fusion & obtaining aircraft attitude
			// (mag vector is being processed outside of FIFO loop)
			AdaptiveComplimentaryFiler::apply(
				accelData,
				gyroData,
				_magDataFiltered,

				FIFOSampleIntervalS,

				0.8f,
				0.95,

				0.95f,

				_rollRad,
				_pitchRad,
				_yawRad
			);

			// ESP_LOGI(_logTag, "RPY deg: %f x %f x %f", toDegrees(_rollRad),  toDegrees(_pitchRad),  toDegrees(_yawRad));

			// Applying tilt compensation to accel vector using obtained RP angles
			// (will be less accurate than raw accel data, because of FUCKING TOKYO DRIFT)
			auto tiltCompensatedAccelData = AdaptiveComplimentaryFiler::applyTiltCompensation(accelData, _rollRad, _pitchRad);
			// Subtracting 1G, because we need "pure" aircraft data without Mother Earth affection
			tiltCompensatedAccelData.setZ(tiltCompensatedAccelData.getZ() - 1);

			// -------------------------------- Acceleration --------------------------------

			accelerationGSum += accelData;

			// -------------------------------- Velocity --------------------------------

			// Computing integrated velocity
			// accelerationMPS = accelerationG * ~9.8
			// velocityMPS = accelerationMPS * deltaTime
			_integratedVelocityMPS += (-tiltCompensatedAccelData.getY()) * Units::earthGMPS2 * FIFOSampleIntervalS;

			// -------------------------------- Position --------------------------------

			// Computing integrated velocity again, but now using tilt compensation
			_integratedTiltCompensatedVelocityMPS += tiltCompensatedAccelData * Units::earthGMPS2 * FIFOSampleIntervalS;

			// Computing integrated position relative to start point
			auto integratedMovementM = _integratedTiltCompensatedVelocityMPS * FIFOSampleIntervalS;
			// Aligning movement vector to Earth axis (Y should point to north)
			integratedMovementM = integratedMovementM.rotateAroundYAxis(_yawRad);
			// Accumulating relative position
			_integratedAlignedPositionM += integratedMovementM;

			// ESP_LOGI(_logTag, "Acc vel: %f x %f x %f", _integratedTiltCompensatedVelocityMPS.getX(), _integratedTiltCompensatedVelocityMPS.getY(), _integratedTiltCompensatedVelocityMPS.getZ());
			// ESP_LOGI(_logTag, "Acc pos: %f x %f x %f", _integratedPositionM.getX(), _integratedPositionM.getY(), _integratedPositionM.getZ());
		}

		_MPU.resetFIFO();
		_MPU.setFIFODataSource(FIFODataSource);
		_MPU.readAndClearInterruptStatus();

    	// Computing acceleration
		accelerationGSum /= sampleCount;
		_accelerationG = accelerationGSum;

    	// Computing geographic coordinates using aligned relative position
    	GeoCoordinates::distanceToLatitudeAndLongitude(
    		_integratedAlignedPositionM.getY(),
    		_integratedAlignedPositionM.getX(),

    		_integratedLatitudeRad,
    		_integratedLongitudeRad
		);

		_FIFOTickTimeUs = esp_timer_get_time() + FIFOTickIntervalUs;
	}
}
