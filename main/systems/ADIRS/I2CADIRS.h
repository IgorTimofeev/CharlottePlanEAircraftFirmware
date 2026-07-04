#pragma once

#include "systems/ADIRS/ADIRS.h"

#include <array>

#include <BMP280.h>
#include <MPU9250.h>

#include "config.h"
#include "systems/ADIRS/IMU.h"

namespace pizda {
	template<typename TUnit>
	class I2CADIRSUnit {
		public:
			explicit I2CADIRSUnit(const uint8_t address) : address(address) {

			}

			TUnit unit {};
			I2CBusHAL hal {};
			uint8_t address;
	};

	class I2CADIRS : public ADIRS {
		public:
			void setup() override;
			void setHomeCoordinates(float latitude, float longitude, float altitude) override;

		protected:
			void onCalibrateAccelAndGyro() override;
			void onCalibrateMag() override;
			void onTick() override;

		private:
			constexpr static uint32_t _calibrationXCVRPacketIntervalUs = 1'000'000 / 5;

			I2CADIRSUnit<IMU> _IMU {
				config::ADIRS::MPU9250I2CAddress
			};

			I2CADIRSUnit<BMP280> _BMP280 {
				config::ADIRS::BMP280I2CAddress
			};

			void IMUTick();

			void BMP280Tick();
	};
}