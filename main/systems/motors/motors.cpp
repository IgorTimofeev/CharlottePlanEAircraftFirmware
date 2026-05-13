#include "systems/motors/motors.h"

#include "aircraft.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <sys/stat.h>

namespace pizda {
	void Motors::setup() {
		auto& ac = Aircraft::getInstance();

		checkPCA9685Error(_PCA9685.setup(
			ac.I2CMasterBusHandle,
			config::pwmu::I2CAddress,
			config::pwmu::I2CFrequencyHz,

			MotorSettings::dutyFrequencyHz,
			PCA9685OutputDriverMode::totemPole,
			PCA9685OutputChangeMode::stop,
			PCA9685OutputDisabledMode::low,
			false,
			true
		));

		for (uint8_t mt = 0; mt < static_cast<uint8_t>(MotorType::maxValue); ++mt) {
			const auto motorType = static_cast<MotorType>(mt);

			const auto motor = getByType(motorType);
			const auto settings = ac.settings.motors.getByType(motorType);

			if (motor && settings)
				motor->setSettings(settings);
		}

		xTaskCreate(
			[](void* arg) {
				static_cast<Motors*>(arg)->onStart();
			},
			"Motors",
			4 * 1024,
			this,
			20,
			nullptr
		);
	}

	Motor* Motors::get(const uint8_t index) {
		if (index >= _motors.size()) {
			ESP_LOGI(_logTag, "index %d >= motors count %d", index, _motors.size());
			return nullptr;
		}
		
		return &_motors[index];
	}
	
	Motor* Motors::getByType(const MotorType type) {
		return get(std::to_underlying(type));
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
				// if (i == 5) {
				// 	ESP_LOGI("motr", "flap power: %d, pulse width: %d, duty: %d", _motors[i].getPower(), _motors[i].getPulseWidthUs(), _motors[i].getDuty());
				// }

				duties[i] = _motors[i].getDuty();
			}

			checkPCA9685Error(_PCA9685.setDuties<0, _motorCount>(duties));

			vTaskDelay(pdMS_TO_TICKS(1'000 / 30));
		}
	}
}
