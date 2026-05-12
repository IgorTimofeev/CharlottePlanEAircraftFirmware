#pragma once

#include <cmath>
#include <algorithm>

#include <esp_log.h>
#include <MPU9250.h>
#include <vector3.h>

namespace pizda {
	using namespace YOBA;

	class IMU {
		public:
			// -------------------------------- MPU --------------------------------

			// 500 Hz is a good balance between sample noise and system response speed
			constexpr static uint8_t MPUSampleRateDivider = 1;
			constexpr static uint16_t MPUSampleRateHz = 1000 / (1 + MPUSampleRateDivider);
			constexpr static uint32_t MPUSampleIntervalHz = 1'000'000 / MPUSampleRateHz;

			// -------------------------------- Accel --------------------------------

			// The narrower the operating range, the more accurate the result will be.
			// However, it should be never exceeded in "real" conditions to avoid data loss
			constexpr static uint8_t accelOperationalRangeG = 2;
			constexpr static MPU9250_accRange accelOperationalRange = MPU9250_ACC_RANGE_2G;
			constexpr static MPU9250_dlpf accelDLPF = MPU9250_DLPF_1;

			// -------------------------------- Gyro --------------------------------

			// Same shit
			constexpr static MPU9250_gyroRange gyroOperationalRange = MPU9250_GYRO_RANGE_250;
			constexpr static MPU9250_dlpf gyroDLPF = MPU9250_DLPF_1;

			// -------------------------------- Mag --------------------------------

			// 100 hz is absolute maximum for AK8963
			constexpr static uint32_t magSampleRateHz = 100;
			constexpr static uint32_t magTickIntervalUs = 1'000'000 / magSampleRateHz;

			// -------------------------------- FIFO --------------------------------

			// Total buffer length
			constexpr static uint16_t FIFOLength = 512;
			// 1 sample = 2 bytes (uint16) * 3 axis (x, y, z) *
			constexpr static uint8_t FIFOSampleDataTypeLength = 2 * 3;

			// Accel + gyro
			constexpr static uint8_t FIFOSampleDataTypeCount = 2;
			constexpr static MPU9250_fifo_data_source FIFODataSource = MPU9250_FIFO_DATA_SOURCE_ACCEL_GYRO;

			constexpr static uint8_t FIFOSampleLength = FIFOSampleDataTypeLength * FIFOSampleDataTypeCount;
			constexpr static uint16_t FIFOMaxSampleCount = FIFOLength / FIFOSampleLength;

			constexpr static float FIFOSampleIntervalS = 1.0f / MPUSampleRateHz;
			constexpr static uint32_t FIFOBytesPerSecond = FIFOSampleLength * MPUSampleRateHz;
			constexpr static uint32_t FIFOMinimumTickIntervalUs = FIFOLength * 1'000'000 / FIFOBytesPerSecond;
			constexpr static uint32_t FIFOMinimumTickRateHz = 1'000'000 / FIFOMinimumTickIntervalUs + 1;

			// Lower tick rate means lesser MCU/SPI overhead and more accurate time intervals for accel/gyro integrating.
			// Therefore, from a performance perspective, FIFOTickRateHz should be equal to FIFOMinimumTickRateHz.
			//
			// On the other hand, if raw data accumulates for a long time, roll/pitch/yaw calculations are also performed
			// rarely. Therefore, a balance is needed here. I think a frequency of around 50-100 Hz should be sufficient
			constexpr static uint32_t FIFOTickRateHz = 50;
			constexpr static uint32_t FIFOTickIntervalUs = 1'000'000 / FIFOTickRateHz;

			// -------------------------------- Computed --------------------------------

			constexpr static uint32_t commonTickIntervalUs = std::min(FIFOTickIntervalUs, magTickIntervalUs);

			bool setup(BusHAL* bus);
			void tick();

			float getRollRad() const;
			float getPitchRad() const;
			float getYawRad() const;

			const Vector3F& getAccelerationG() const;
			const Vector3F& getIntegratedPositionM() const;
			const Vector3F& getIntegratedVelocityMs() const;

			void setCalibrationMode();
			void setOperationalMode();

			Vector3F getRawAccelData() const;
			Vector3F getRawGyroData() const;
			Vector3F getRawMagData() const;

			const Vector3F& getAccelBias() const;
			void setAccelBias(const Vector3F& value);

			const Vector3F& getGyroBias() const;
			void setGyroBias(const Vector3F& value);

			const Vector3F& getMagBias() const;
			void setMagBias(const Vector3F& value);

		private:
			constexpr static auto _logTag = "IMU";
			
			// -------------------------------- MPU --------------------------------

			MPU9250 _MPU {};

			// -------------------------------- Accel --------------------------------

			Vector3F _accelBias {};
			Vector3F _accelerationG {};
			
			// -------------------------------- Gyro --------------------------------
			
			Vector3F _gyroBias {};
			
			// -------------------------------- Mag --------------------------------

			Vector3F _magBias {};
			Vector3F _magDataFiltered {};
			uint32_t _magTickTimeUs = 0;

			void magTick();

			// -------------------------------- FIFO --------------------------------

			uint32_t _FIFOTickTimeUs = 0;
			
			void FIFOTick();

			// -------------------------------- Computed --------------------------------
			
			float _rollRad = 0;
			float _pitchRad = 0;
			float _yawRad = 0;
			
			Vector3F _integratedVelocityMs {};
			Vector3F _integratedPositionM {};
	};
}