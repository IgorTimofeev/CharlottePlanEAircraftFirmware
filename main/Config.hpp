#pragma once

#include <cstdint>

#include <driver/ledc.h>
#include <driver/uart.h>
#include <driver/ledc.h>
#include <driver/gpio.h>
#include <esp_adc/adc_oneshot.h>
#include <driver/spi_master.h>
#include <driver/i2c_master.h>

#include <Units.hpp>
#include <SX1262.hpp>
#include <PCA9685.hpp>
#include <MS4525.hpp>
#include <MPU9250.hpp>
#include <BMP280.hpp>

#include "Utilities/Math.hpp"
#include "Types/Generic.hpp"

namespace pizda {
	using namespace YOBA;
	
	class config {
		public:
			class common {
				public:
					constexpr static gpio_num_t RST = GPIO_NUM_1;
			};

			class I2C {
				public:
					constexpr static i2c_port_t port = I2C_NUM_0;

					constexpr static gpio_num_t SCL = GPIO_NUM_2;
					constexpr static gpio_num_t SDA = GPIO_NUM_3;
			};

			class SPI {
				public:
					constexpr static spi_host_device_t device = SPI2_HOST;

					constexpr static gpio_num_t SCK = GPIO_NUM_4;
					constexpr static gpio_num_t MISO = GPIO_NUM_5;
					constexpr static gpio_num_t MOSI = GPIO_NUM_6;
			};

			class battery {
				public:
					constexpr static gpio_num_t pin = GPIO_NUM_18;

					constexpr static uint32_t voltageMin = 4 * 3'000;
					constexpr static uint32_t voltageMax = 4 * 4'200;

					constexpr static uint32_t voltageDividerR1 = 220'000;
					constexpr static uint32_t voltageDividerR2 = 51'000;
			};

			class lights {
				public:
					class cabin {
						public:
							constexpr static gpio_num_t pin = GPIO_NUM_48;
					};

					class wingLeft {
						public:
							constexpr static gpio_num_t pin = GPIO_NUM_12;
							constexpr static uint8_t length = 6;
					};

					class wingRight {
						public:
							constexpr static gpio_num_t pin = GPIO_NUM_11;
							constexpr static uint8_t length = 6;
					};

					class tail {
						public:
							constexpr static gpio_num_t pin = GPIO_NUM_10;
							constexpr static uint8_t length = 3;
					};
			};

			class PWMU {
				public:
					constexpr static uint8_t I2CAddress =  PCA9685::I2CBaseAddress | 0b0000'0000;
					constexpr static uint32_t I2CFrequencyHz = PCA9685::I2CDefaultFrequency;
			};

			class ADIRS {
				public:
					class MPU9250 {
						public:
							constexpr static uint8_t I2CAddress = YOBA::MPU9250::defaultI2CAddress;
							constexpr static uint32_t I2CFrequencyHz = 400'000;
					};

					class BMP280 {
						public:
							constexpr static uint8_t I2CAddress = YOBA::BMP280::defaultI2CAddress;
							constexpr static uint32_t I2CFrequencyHz = 1'000'000;
					};

					class MS4525 {
						public:
							constexpr static uint8_t I2CAddress = YOBA::MS4525::defaultI2CAddress;

							// For some reason, MS4525 is really sensitive to I2C wiring issues - even 400 kHz
							// may produce communication problems. Lowering this to 100-200 kHz is a good idea
							constexpr static uint32_t I2CFrequencyHz = 200'000;

							// Sensor may be too noisy on low diff pressure values - so we can consider any IAS
							// lower than given threshold as 0 m/s
							constexpr static uint16_t minValidAirspeedThresholdMPS = 3;

							// Prevents IAS jittering and excessive noise
							constexpr static float airspeedEMAFilterFactor = 0.1f;
					};
			};

			class XCVR {
				public:
					constexpr static gpio_num_t SS = GPIO_NUM_7;
					constexpr static gpio_num_t busy = GPIO_NUM_8;
					constexpr static gpio_num_t DIO1 = GPIO_NUM_9;
					constexpr static gpio_num_t RST = common::RST;

					// SX1262 supports up to 16 MHz, but with long wires (10+ cm) there will be troubles, so
					constexpr static uint32_t SPIFrequencyHz = 10'000'000;

					// Default values, can be changed and stored in NVS
					constexpr static TransceiverCommunicationSettings communicationSettings {
						915'000'000,
						SX1262LoRaBandwidth::bw500_0,
						7,
						SX1262LoRaCodingRate::cr4_5,
						0x34,
						8,

						60,
						22,

						6000,
						6000
					};
			};

			class GNSS {
				public:
					constexpr static gpio_num_t rx = GPIO_NUM_NC;
					constexpr static gpio_num_t tx = GPIO_NUM_NC;
			};

			class camera {
				public:
					constexpr static int16_t servoAngularRangeDeg = 180;
					constexpr static int16_t servoMaxDeg = servoAngularRangeDeg / 2;
					constexpr static int16_t servoMinDeg = -servoMaxDeg;

					constexpr static int16_t pitchMinDeg = servoMinDeg;
					constexpr static int16_t pitchMaxDeg = 10;

					constexpr static int16_t yawMinDeg = servoMinDeg;
					constexpr static int16_t yawMaxDeg = servoMaxDeg;

					constexpr static int16_t pitchCorrectionYawThresholdMinDeg = 40;
					constexpr static int16_t pitchCorrectionYawThresholdMaxDeg = std::min<int16_t>(yawMaxDeg, 90);
					constexpr static int16_t pitchCorrectionByMaxDeg = 30;

					static void clamp(int16_t& pitch, int16_t& yaw) {
						pitch = std::clamp<int16_t>(
							std::clamp<int16_t>(
								pitch,
								pitchMinDeg,
								pitchMaxDeg
							),
							servoMinDeg,
							servoMaxDeg
						);

						yaw = std::clamp<int16_t>(
							std::clamp<int16_t>(
								yaw,
								yawMinDeg,
								yawMaxDeg
							),
							servoMinDeg,
							servoMaxDeg
						);
					}

					static void correctPitchForYaw(int16_t& pitch, int16_t yaw) {
						// Yaw abs
						if (yaw < 0)
							yaw = -yaw;

						// Nothing to do
						if (yaw <= pitchCorrectionYawThresholdMinDeg)
							return;

						// Changing pitch by difference between yaw and min/max threshold
						pitch -=
							static_cast<int16_t>(pitchCorrectionByMaxDeg)
							* (yaw - pitchCorrectionYawThresholdMinDeg)
							/ (pitchCorrectionYawThresholdMaxDeg - pitchCorrectionYawThresholdMinDeg);
					}
			};

			class FBW {
				public:
					// How many meters between selected and indicated altitude are required to switch from FLC mode to ALTS mode
					constexpr static float altitudeDeltaForFLCToALTSSwitchM = Units::convertDistance(50.f, DistanceUnit::foot, DistanceUnit::meter);
			};
	};
}
