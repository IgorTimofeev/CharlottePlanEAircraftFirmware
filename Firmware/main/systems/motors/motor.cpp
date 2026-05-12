#include "systems/motors/motor.h"

#include "aircraft.h"

namespace pizda {
	Motor::Motor(PulseWidthModulator* PWM) : _PWM(PWM) {
	
	}
	
	uint16_t Motor::getPower() const {
		return _power;
	}
	
	float Motor::getPowerF() const {
		return static_cast<float>(getPower()) / static_cast<float>(powerMax);
	}
	
	// Value range is [0; 0xFFFF]
	void Motor::setPower(const uint16_t value) {
		_power = value;
		
		auto pulseWidthUs = _settings.min + (_settings.max - _settings.min) * _power / powerMax;
		
		if (_settings.reverse)
			pulseWidthUs = _settings.min + _settings.max - pulseWidthUs;
		
		pulseWidthUs = std::clamp<int32_t>(pulseWidthUs, _settings.min, _settings.max);

//		ESP_LOGI("ppizda", "pulse: %d", pulseWidthUs);

		_PWM->setPulseWidth(pulseWidthUs);
	}
	
	void Motor::setPowerF(const float value) {
		setPower(std::round(value * static_cast<float>(powerMax)));
	}

	void Motor::setSettings(const MotorSettings& settings) {
		_settings = settings;
	}
}