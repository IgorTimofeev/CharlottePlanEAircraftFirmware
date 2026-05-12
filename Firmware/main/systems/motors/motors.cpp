#include "systems/motors/motors.h"

#include "aircraft.h"

namespace pizda {
	void Motors::setup() {
		for (auto& pwm : _LEDCPWMs)
			pwm.setup();

		updateConfigurationsFromSettings();
	}
	
	Motor* Motors::getMotor(const uint8_t index) {
		if (index >= _motors.size()) {
			ESP_LOGI(_logTag, "index %d >= motors count %d", index, _motors.size());
			return nullptr;
		}
		
		return &_motors[index];
	}
	
	Motor* Motors::getMotor(const MotorType type) {
		return getMotor(std::to_underlying(type));
	}
	
	void Motors::updateConfigurationsFromSettings() {
		const auto& ac = Aircraft::getInstance();
		
		getMotor(MotorType::throttle)->setSettings(ac.settings.motors.throttle);
		getMotor(MotorType::noseWheel)->setSettings(ac.settings.motors.noseWheel);
		
		getMotor(MotorType::flapLeft)->setSettings(ac.settings.motors.flapLeft);
		getMotor(MotorType::aileronLeft)->setSettings(ac.settings.motors.aileronLeft);
		
		getMotor(MotorType::flapRight)->setSettings(ac.settings.motors.flapRight);
		getMotor(MotorType::aileronRight)->setSettings(ac.settings.motors.aileronRight);
		
		getMotor(MotorType::tailLeft)->setSettings(ac.settings.motors.tailLeft);
		getMotor(MotorType::tailRight)->setSettings(ac.settings.motors.tailRight);
	}
}