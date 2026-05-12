#include "systems/motors/motors.h"

#include "aircraft.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace pizda {
	void Motors::setup() {
		const auto& ac = Aircraft::getInstance();

		checkPCA9685Error(_PCA9685.setup(
			ac.I2CMasterBusHandle,
			config::pwmc::I2CAddress,
			config::pwmc::I2CFrequencyHz,

			config::pwmc::PWMFrequencyHz,
			PCA9685OutputDriverMode::totemPole,
			PCA9685OutputChangeMode::stop,
			PCA9685OutputDisabledMode::low,
			false,
			true
		));

		updateConfigurationsFromSettings();

		xTaskCreate(
			[](void* arg) {
				static_cast<Motors*>(arg)->onStart();
			},
			"Motors",
			2 * 1024,
			this,
			20,
			nullptr
		);
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
		getMotor(MotorType::reverse)->setSettings(ac.settings.motors.throttle);
		getMotor(MotorType::noseWheel)->setSettings(ac.settings.motors.noseWheel);

		getMotor(MotorType::flapLeft)->setSettings(ac.settings.motors.flapLeft);
		getMotor(MotorType::aileronLeft)->setSettings(ac.settings.motors.aileronLeft);
		
		getMotor(MotorType::flapRight)->setSettings(ac.settings.motors.flapRight);
		getMotor(MotorType::aileronRight)->setSettings(ac.settings.motors.aileronRight);
		
		getMotor(MotorType::tailLeft)->setSettings(ac.settings.motors.tailLeft);
		getMotor(MotorType::tailRight)->setSettings(ac.settings.motors.tailRight);

		getMotor(MotorType::cameraPitch)->setSettings(ac.settings.motors.cameraPitch);
		getMotor(MotorType::cameraYaw)->setSettings(ac.settings.motors.cameraYaw);
	}

	bool Motors::checkPCA9685Error(const PCA9685Error error) {
		if (error == PCA9685Error::none)
			return true;

		constexpr static uint8_t errorBufferLength = 255;
		char errorBuffer[errorBufferLength];

		PCA9685::errorToString(error, { errorBuffer, errorBufferLength });

		ESP_LOGE(_logTag, "PCA9685E error: %s", errorBuffer);

		return false;
	}

	void Motors::onStart() {
		while (true) {
			std::array<uint16_t, _motorCount> duties {};

			for (uint8_t i = 0; i < _motorCount; ++i) {
				const auto pulseWidthUs = _motors[i].getPowerPulseWithUs();

				// Pulse width -> duty cycle conversion
				constexpr uint32_t tickDurationUs = 1'000'000 / config::pwmc::PWMFrequencyHz;
				constexpr uint32_t dutyMax = (1 << config::pwmc::PWMResolutionBits) - 1;

				duties[i] = pulseWidthUs * dutyMax / tickDurationUs;
			}

			checkPCA9685Error(_PCA9685.setDuties<0, _motorCount>(duties));

			vTaskDelay(pdMS_TO_TICKS(1'000 / 30));
		}
	}
}
