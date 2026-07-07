#pragma once

#include <array>

#include <BMP280.hpp>
#include <MPU9250.hpp>
#include <MS4525.hpp>

#include "Systems/ADIRS/ADIRS.hpp"
#include "Systems/ADIRS/IMU.hpp"
#include "Config.hpp"

namespace pizda {
	template<typename TUnit>
	class I2CADIRSUnit {
		public:
			explicit I2CADIRSUnit(const uint8_t address, const uint32_t frequencyHz) : address(address), frequencyHz(frequencyHz) {

			}

			TUnit unit {};
			I2CBusHAL hal {};
			uint8_t address;
			uint32_t frequencyHz;
	};

	class I2CADIRS : public ADIRS {
		// -------------------------------- Main --------------------------------

		public:
			void setup() override;
			void setHomeCoordinates(float latitude, float longitude, float altitude) override;

		protected:
			void onCalibrateAccelAndGyro() override;
			void onCalibrateMag() override;
			void onTick() override;

		private:
			constexpr static uint32_t _calibrationXCVRPacketIntervalUs = 1'000'000 / 5;

		// -------------------------------- IMU --------------------------------

		private:
			I2CADIRSUnit<IMU> _IMU {
				config::ADIRS::MPU9250::I2CAddress,
				config::ADIRS::MPU9250::I2CFrequencyHz
			};

			void IMUTick();

		// -------------------------------- BMP280 --------------------------------

		private:
			I2CADIRSUnit<BMP280> _BMP280 {
				config::ADIRS::BMP280::I2CAddress,
				config::ADIRS::BMP280::I2CFrequencyHz
			};

			void BMP280Tick();

		// -------------------------------- MS4525 --------------------------------

		private:
			MS4525 _MS4525 {};

			constexpr static uint32_t _MS4525TickIntervalUs = 1'000'000 / MS4525::defaultSampleRateHz;
			int64_t _MS4525TickTimeUs = 0;

			void MS4525Tick();

			static bool checkMS4525Error(const char* prefix, const MS4525Error error);

		// -------------------------------- Summary --------------------------------

		private:
			constexpr static uint32_t _minTickIntervalUs = std::min(IMU::minTickIntervalUs, _MS4525TickIntervalUs);
			constexpr static uint32_t _minTickIntervalTicks = pdMS_TO_TICKS(std::max(_minTickIntervalUs / 1000, portTICK_PERIOD_MS));
	};
}