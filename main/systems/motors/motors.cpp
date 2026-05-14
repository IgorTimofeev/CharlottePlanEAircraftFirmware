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
			config::PWMU::I2CAddress,
			config::PWMU::I2CFrequencyHz,

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

	Motor* Motors::getByType(const MotorType type) {
		switch (type) {
			case MotorType::throttle: return &_throttle;
			case MotorType::noseWheel: return &_noseWheel;

			case MotorType::flapLeft: return &_flapLeft;
			case MotorType::aileronLeft: return &_aileronLeft;

			case MotorType::flapRight: return &_flapRight;
			case MotorType::aileronRight: return &_aileronRight;

			case MotorType::tailLeft: return &_tailLeft;
			case MotorType::tailRight: return &_tailRight;

			case MotorType::cameraPitch: return &_cameraPitch;
			case MotorType::cameraYaw: return &_cameraPitch;

			default: return nullptr;
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
		while (true) {
			std::array<uint16_t, static_cast<uint8_t>(MotorType::maxValue) + 1> duties {};

			for (uint8_t i = 0; i < duties.size(); ++i) {
				// if (i == 5) {
				// 	ESP_LOGI("motr", "flap power: %d, pulse width: %d, duty: %d", _motors[i].getPower(), _motors[i].getPulseWidthUs(), _motors[i].getDuty());
				// }

				const auto motor = getByType(static_cast<MotorType>(i));

				duties[i] = motor ? motor->getDuty() : 0;
			}

			checkPCA9685Error(_PCA9685.setDuties<0, duties.size()>(duties));

			vTaskDelay(pdMS_TO_TICKS(1'000 / 30));
		}
	}
}
