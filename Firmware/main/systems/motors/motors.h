#pragma once

#include <array>
#include <atomic>

#include "config.h"
#include "systems/motors/motor.h"
#include "systems/PWM/LEDCPulseWidthModulator.h"

namespace pizda {
	enum class MotorType : uint8_t {
		throttle,
		noseWheel,
		
		flapLeft,
		aileronLeft,
		
		flapRight,
		aileronRight,
		
		tailLeft,
		tailRight
	};
	
	class Motors {
		public:
			void setup();
			Motor* getMotor(uint8_t index);
			Motor* getMotor(MotorType type);
			void updateConfigurationsFromSettings();
		
		private:
			constexpr static auto _logTag = "Motors";

			std::array<LEDCPulseWidthModulator, 8> _LEDCPWMs {
				LEDCPulseWidthModulator { config::motors::throttle, LEDC_CHANNEL_0 },
				LEDCPulseWidthModulator { config::motors::noseWheel, LEDC_CHANNEL_1 },

				LEDCPulseWidthModulator { config::motors::flapLeft, LEDC_CHANNEL_2 },
				LEDCPulseWidthModulator { config::motors::aileronLeft, LEDC_CHANNEL_3 },

				LEDCPulseWidthModulator { config::motors::flapRight, LEDC_CHANNEL_4 },
				LEDCPulseWidthModulator { config::motors::aileronRight, LEDC_CHANNEL_5 },

				LEDCPulseWidthModulator { config::motors::tailLeft, LEDC_CHANNEL_6 },
				LEDCPulseWidthModulator { config::motors::tailRight, LEDC_CHANNEL_7 },
			};

			std::array<Motor, 8> _motors {
				Motor { &_LEDCPWMs[0] },
				Motor { &_LEDCPWMs[1] },
				
				Motor { &_LEDCPWMs[2] },
				Motor { &_LEDCPWMs[3] },
				
				Motor { &_LEDCPWMs[4] },
				Motor { &_LEDCPWMs[5] },
				
				Motor { &_LEDCPWMs[6] },
				Motor { &_LEDCPWMs[7] },
			};
	};
}
