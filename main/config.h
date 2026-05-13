#pragma once

#include <cstdint>

#include <driver/ledc.h>
#include <driver/uart.h>
#include <driver/ledc.h>
#include <driver/gpio.h>
#include <esp_adc/adc_oneshot.h>
#include <driver/spi_master.h>
#include <driver/i2c_master.h>

#include <units.h>
#include <SX1262.h>
#include <PCA9685.h>

#include "utilities/math.h"
#include "types/generic.h"

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
					constexpr static adc_unit_t unit = ADC_UNIT_2;
					constexpr static adc_channel_t channel = ADC_CHANNEL_7;

					constexpr static uint32_t voltageMin = 4 * 3'000;
					constexpr static uint32_t voltageMax = 4 * 4'200;

					constexpr static uint32_t voltageDividerR1 = 220'000;
					constexpr static uint32_t voltageDividerR2 = 51'000;
			};

			class motors {
				public:
					constexpr static uint8_t cameraHorizontal = 0;
					constexpr static uint8_t cameraVertical = 1;

					constexpr static uint8_t throttle = 2;
					constexpr static uint8_t noseWheel = 3;
					constexpr static uint8_t reverse = 4;

					constexpr static uint8_t flapLeft = 5;
					constexpr static uint8_t aileronLeft = 6;

					constexpr static uint8_t flapRight = 7;
					constexpr static uint8_t aileronRight = 8;

					constexpr static uint8_t tailLeft = 9;
					constexpr static uint8_t tailRight = 10;
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

			class pwmu {
				public:
					constexpr static uint8_t I2CAddress =  PCA9685::I2CBaseAddress | 0b0000'0000;
					constexpr static uint32_t I2CFrequencyHz = PCA9685::I2CDefaultFrequency;
			};

			class adirs {
				public:
					constexpr static uint8_t unitCount = 1;

					class adiru0 {
						public:
							constexpr static uint8_t mpu9250I2CAddress = 0x68;
							constexpr static uint8_t bmp280I2CAddress = 0x76;

							// constexpr static gpio_num_t mpu9250ss = GPIO_NUM_17;
							// constexpr static gpio_num_t bmp280ss = GPIO_NUM_18;
					};
			};

			class transceiver {
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
						SX1262::LoRaBandwidth::bw500_0,
						7,
						SX1262::LoRaCodingRate::cr4_5,
						0x34,
						8,

						60,
						22
					};
			};

			class GNSS {
				public:
					constexpr static gpio_num_t rx = GPIO_NUM_NC;
					constexpr static gpio_num_t tx = GPIO_NUM_NC;
			};
			
			class flyByWire {
				public:
					// How many meters between selected and indicated altitude are required to switch from FLC mode to ALTS mode
					constexpr static float altitudeDeltaForFLCToALTSSwitchM = Units::convertDistance(50.f, DistanceUnit::foot, DistanceUnit::meter);
			};
	};
}
