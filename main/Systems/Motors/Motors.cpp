#include <utility>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <sys/stat.h>

#include "Systems/Motors/Motors.hpp"
#include "Aircraft.hpp"

namespace pizda {
	void Motors::setup() {
		auto& ac = Aircraft::getInstance();

		checkPCA9685Error(_PCA9685.setup(
			ac.I2CMasterBusHandle,
			config::PWMU::I2CAddress,
			config::PWMU::I2CFrequencyHz,

			MotorSettings::dutyFrequencyHz,
			PCA9685OutputDriverMode::totemPole,
			PCA9685OutputChangeMode::stop,
			PCA9685OutputDisabledMode::low,
			false,
			true
		));

		for (uint8_t mt = 0; mt < static_cast<uint8_t>(MotorType::maxValue) + 1; ++mt) {
			const auto motorType = static_cast<MotorType>(mt);
			const auto settings = ac.settings.motors.getByType(motorType);

			getByType(motorType)->setSettings(settings);

				// ESP_LOGI(_logTag, "loaded type: %d, min: %d, max: %d, reverse: %d", static_cast<uint8_t>(motorType), settings->min, settings->max, settings->reverse);
		}

		xTaskCreatePinnedToCore(
			[](void* arg) {
				static_cast<Motors*>(arg)->onStart();
			},
			"Motors",
			4 * 1024,
			this,
			20,
			nullptr,
			0
		);
	}

	Motor* Motors::getByType(const MotorType type) {
		switch (type) {
			case MotorType::throttleLeft: return &_throttleLeft;
			case MotorType::throttleRight: return &_throttleRight;
			case MotorType::noseWheel: return &_noseWheel;

			case MotorType::flapLeft: return &_flapLeft;
			case MotorType::aileronLeft: return &_aileronLeft;

			case MotorType::flapRight: return &_flapRight;
			case MotorType::aileronRight: return &_aileronRight;

			case MotorType::tailLeft: return &_tailLeft;
			case MotorType::tailRight: return &_tailRight;

			case MotorType::cameraPitch: return &_cameraPitch;
			case MotorType::cameraYaw: return &_cameraYaw;

			default: {
				char pizda[32];
				std::snprintf(pizda, sizeof(pizda), "unsupported motor type: %d", std::to_underlying(type));

				esp_system_abort(pizda);
			}
		}
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
		std::array<uint16_t, static_cast<uint8_t>(MotorType::maxValue) + 1> duties {};

		while (true) {
			for (uint8_t i = 0; i < duties.size(); ++i) {
				// if (i == 5) {
				// 	ESP_LOGI("motr", "flap power: %d, pulse width: %d, duty: %d", _motors[i].getPower(), _motors[i].getPulseWidthUs(), _motors[i].getDuty());
				// }

				duties[i] = getByType(static_cast<MotorType>(i))->getDuty();
			}

			checkPCA9685Error(_PCA9685.setDuties<0, duties.size()>(duties));

			vTaskDelay(pdMS_TO_TICKS(1'000 / 30));
		}
	}
}
