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
			void setup();
			void setHomeCoordinates(float latitude, float longitude, float altitude) override;

		protected:

		private:
			constexpr static uint32_t _calibrationXCVRPacketIntervalUs = 1'000'000 / 5;

		// -------------------------------- IMU --------------------------------

		private:
			constexpr static uint32_t _IMUTickIntervalTicks =
				pdMS_TO_TICKS(std::max<uint32_t>(IMU::minTickIntervalUs / 1000, portTICK_PERIOD_MS));

			I2CADIRSUnit<IMU> _IMU {
				config::ADIRS::MPU9250::I2CAddress,
				config::ADIRS::MPU9250::I2CFrequencyHz
			};

			void onCalibrateAccelAndGyro();
			void onCalibrateMag();
			void onIMUTick();
			void onIMUStart();

		// -------------------------------- Barometer --------------------------------

		private:
			constexpr static uint16_t _barometerTickRateHz = 12;

			constexpr static uint32_t _barometerTickIntervalTicks =
				pdMS_TO_TICKS(std::max<uint32_t>(1'000'000 / _barometerTickRateHz / 1000, portTICK_PERIOD_MS));

			I2CADIRSUnit<BMP280> _barometer {
				config::ADIRS::BMP280::I2CAddress,
				config::ADIRS::BMP280::I2CFrequencyHz
			};

			void onBarometerTick();
			void onBarometerStart();

			// -------------------------------- Airspeed --------------------------------

		private:
			MS4525 _airspeedSensor {};

			constexpr static uint32_t _airspeedTickIntervalTicks =
				pdMS_TO_TICKS(std::max<uint32_t>(1'000'000 / MS4525::defaultSampleRateHz / 1000, portTICK_PERIOD_MS));

			static bool checkMS4525Error(const char* prefix, const MS4525Error error);

			void onAirspeedTick();
			void onAirspeedStart();

		// -------------------------------- Summary --------------------------------

		private:
	};
}