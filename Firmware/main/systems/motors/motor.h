#pragma once

#include "types/generic.h"
#include "systems/PWM/pulseWidthModulator.h"

namespace pizda {
	class Motor {
		public:
			Motor(PulseWidthModulator* PWM);
			
			constexpr static auto _logTag = "Motor";
			
			constexpr static uint16_t powerMax = 0xFFFF;
			
			constexpr static uint8_t tickFrequencyHz = 50;
			constexpr static uint32_t tickDurationUs = 1'000'000 / tickFrequencyHz;
			
			constexpr static uint8_t dutyLengthBits = 13;
			constexpr static uint32_t dutyMax = (1 << dutyLengthBits) - 1;

			uint16_t getPower() const;
			float getPowerF() const;
			
			void setPower(uint16_t value);
			void setPowerF(float value);
			void setSettings(const MotorSettings& settings);
		
		private:
			PulseWidthModulator* _PWM;

			MotorSettings _settings {};
			uint16_t _power = 0;
		
	};
}