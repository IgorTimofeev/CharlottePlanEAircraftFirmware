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

	const Vector3F& IMU::getIntegratedPositionM() const {
		return _integratedPositionM;
	}

	const Vector3F& IMU::getIntegratedVelocityMs() const {
		return _integratedVelocityMs;
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
		_MPU.getAccelData(x, y, z);

		return { x, y, z };
	}

	Vector3F IMU::getRawGyroData() const {
		float x, y, z;
		_MPU.getGyroData(x, y, z);

		return { x, y, z };
	}

	Vector3F IMU::getRawMagData() const {
		float x, y, z;
		_MPU.getMagData(x, y, z);

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
		_MPU.getMagData(x, y, z);

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

		//				ESP_LOGI(_logTag, "FIFO sample count %d", sampleCount);

		if (sampleCount < 2) {
			ESP_LOGW(_logTag, "FIFO sample count %d is not enough, skipping for more data", sampleCount);
			return;
		}
		else if (sampleCount > FIFOMaxSampleCount) {
			ESP_LOGW(_logTag, "FIFO sample count %d exceeds max sample count %d, data was permanently lost", sampleCount, FIFOMaxSampleCount);
		}

		_MPU.setFIFODataSource(MPU9250_FIFO_DATA_SOURCE_NONE);

		//				ESP_LOGI(_logTag, "FIFO sample count: %d", sampleCount);

		uint8_t sample[FIFOSampleLength] {};
		Vector3F accelerationGSum {};
		float x, y, z;

		for (uint32_t i = 0; i < sampleCount; i++) {
			// FIFO
			_MPU.getFIFOData(sample, FIFOSampleLength);

			// Accel
			_MPU.getAccelData(sample, x, y, z);
			const auto accelData = Vector3F(x, y, z) - _accelBias;
			accelerationGSum += accelData;

			// Gyro
			_MPU.getGyroData(sample + FIFOSampleDataTypeLength, x, y, z);
			const auto gyroData = Vector3F(x, y, z) - _gyroBias;

			// Applying adaptive complimentary filter
			AdaptiveComplimentaryFiler::apply(
				accelData,
				gyroData,
				_magDataFiltered,

				FIFOSampleIntervalS,

				0.8,
				0.95,

				0.95,

				_rollRad,
				_pitchRad,
				_yawRad
			);

			// Position
			auto accelTilt = AdaptiveComplimentaryFiler::applyTiltCompensation(accelData, _rollRad, _pitchRad);
			// Subtracting 1G
			accelTilt.setZ(accelTilt.getZ() - 1);

			auto accelerationMs2 = accelTilt * Units::earthGMs2;
			_integratedVelocityMs += accelerationMs2 * FIFOSampleIntervalS;

			auto accelPositionOffsetM = _integratedVelocityMs * FIFOSampleIntervalS;
			_integratedPositionM += accelPositionOffsetM;

			// ESP_LOGI(_logTag, "Acc vel: %f x %f x %f", _integratedVelocityMs.getX(), _integratedVelocityMs.getY(), _integratedVelocityMs.getZ());
			// ESP_LOGI(_logTag, "Acc pos: %f x %f x %f", _integratedPositionM.getX(), _integratedPositionM.getY(), _integratedPositionM.getZ());
		}

		_MPU.resetFIFO();
		_MPU.setFIFODataSource(FIFODataSource);
		_MPU.readAndClearInterruptStatus();

		accelerationGSum /= sampleCount;
		_accelerationG = accelerationGSum;

		_FIFOTickTimeUs = esp_timer_get_time() + FIFOTickIntervalUs;
	}
}
