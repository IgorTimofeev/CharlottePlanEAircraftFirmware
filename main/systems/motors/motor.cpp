#include "systems/motors/motor.h"

#include "aircraft.h"

namespace pizda {
	uint16_t Motor::getRawPower() const {
		return _power;
	}

	float Motor::getRawPowerF() const {
		return static_cast<float>(getRawPower()) / static_cast<float>(MotorSettings::powerMax);
	}

	uint16_t Motor::getPower() const {
		return _settings.reverse ? MotorSettings::powerMax - _power : _power;
	}

	uint16_t Motor::getPulseWidthUs() const {
		return _settings.min + (_settings.max - _settings.min) * getPower() / MotorSettings::powerMax;
	}

	uint16_t Motor::getDuty() const {
		return MotorSettings::pulseWidthUsToDuty(getPulseWidthUs());
	}

	void Motor::setPower(const uint16_t value) {
		_power = value;
	}
	
	void Motor::setPowerF(const float value) {
		setPower(std::round(value * static_cast<float>(MotorSettings::powerMax)));
	}

	void Motor::setSettings(const MotorSettings* settings) {
		_settings = *settings;
	}
}
