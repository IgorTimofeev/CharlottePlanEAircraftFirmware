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

		protected:
			void onCalibrateAccelAndGyro() override;

			void onCalibrateMag() override;

			void onTick() override;

		private:
			std::array<I2CADIRSUnit<IMU>, 1> _IMUs {
				I2CADIRSUnit<IMU> {
					config::adirs::adiru0::mpu9250I2CAddress
				}
			};

			std::array<I2CADIRSUnit<BMP280>, 1> _BMPs {
				I2CADIRSUnit<BMP280> {
					config::adirs::adiru0::bmp280I2CAddress
				}
			};

			bool setupIMUs();

			void updateIMUs();

			bool setupBMPs();

			void updateBMPs();
	};
}