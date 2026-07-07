#pragma once

#include "Types/Generic.hpp"

namespace pizda {
	class Motor {
		public:
			constexpr static auto _logTag = "Motor";

			uint16_t getRawPower() const;
			uint16_t getPower() const;
			uint16_t getPulseWidthUs() const;
			uint16_t getDuty() const;
			float getRawPowerF() const;

			void setPower(uint16_t value);
			void setPowerF(float value);
			void setSettings(const MotorSettings* settings);
		
		private:
			MotorSettings _settings {};
			uint16_t _power = 0;
	};
}