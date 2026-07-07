#include "Systems/Lights/Lights.hpp"

#include <esp_timer.h>
#include <driver/gpio.h>

#include "Aircraft.hpp"

namespace pizda {
	void Lights::setup() {
		xTaskCreatePinnedToCore(
			[](void* arg) {
				static_cast<Lights*>(arg)->onStart();
			},
			"lights",
			4096,
			this,
			tskIDLE_PRIORITY,
			&taskHandle,
			0
		);
	}

	void Lights::setCabinEnabled(const bool value) const {
		auto& ac = Aircraft::getInstance();
		
		if (value == ac.settings.lights.getCabin())
			return;
		
		ac.settings.lights.setCabin(value);
		ac.settings.lights.writeLater();
		
		wake();
	}

	void Lights::setNavigationEnabled(const bool value) const {
		auto& ac = Aircraft::getInstance();
		
		if (value == ac.settings.lights.getNav())
			return;
		
		ac.settings.lights.setNav(value);
		ac.settings.lights.writeLater();
		
		wake();
	}

	void Lights::setStrobeEnabled(const bool value) const {
		auto& ac = Aircraft::getInstance();
		
		if (value == ac.settings.lights.getStrobe())
			return;
		
		ac.settings.lights.setStrobe(value);
		ac.settings.lights.writeLater();
		
		wake();
	}

	void Lights::setLandingEnabled(const bool value) const {
		auto& ac = Aircraft::getInstance();
		
		if (value == ac.settings.lights.getLanding())
			return;
		
		ac.settings.lights.setLanding(value);
		ac.settings.lights.writeLater();
		
		wake();
	}

	void Lights::setEmergencyEnabled(const bool value) {
		if (value == _emergency)
			return;

		_emergency = value;
		
		wake();
	}
	
	// -------------------------------- Processing --------------------------------
	
	bool Lights::delay(const uint32_t ms) {
		return ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(ms)) > 0;
	}
	
	void Lights::wake() const {
		xTaskNotifyGive(taskHandle);
	}
	
	void Lights::updateNavOrLanding(const Light& light, const uint8_t r, const uint8_t g, const uint8_t b) {
		const auto& ac = Aircraft::getInstance();
		
		// Navigation
		if (ac.settings.lights.getNav()) {
			light.fill(r, g, b);
		}
		else {
			light.fill(0x00);
		}

		// Landing
		if (ac.settings.lights.getLanding())
			light.fill(0, light.getLength() / 2, 0xFF, 0xFF, 0xFF);

		light.flush();
	}

	void Lights::updateWingStrobe(const Light& light, const uint8_t r, const uint8_t g, const uint8_t b) {
		const auto& ac = Aircraft::getInstance();
		
		if (ac.settings.lights.getStrobe()) {
			light.fill(0xFF);
			light.flush();
		}
		else {
			updateNavOrLanding(light, r, g, b);
		}
	}

	void Lights::updateTailStrobe(const bool active) const {
		const auto& ac = Aircraft::getInstance();

		_tail.fill(
			ac.settings.lights.getStrobe() && active
			? 0xFF
			: (
				ac.settings.lights.getNav()
				? tailDimmedValue
				: 0
			)
		);

		_tail.flush();
	}

	void Lights::setCabin(const bool state) {
		gpio_set_level(config::lights::cabin::pin, !state);
	}

	[[noreturn]] void Lights::onStart() const {
		const auto& ac = Aircraft::getInstance();

		// GPIO
		{
			gpio_config_t g = {};
			g.pin_bit_mask = 1ULL << config::lights::cabin::pin;
			g.mode = GPIO_MODE_OUTPUT;
			g.pull_up_en = GPIO_PULLUP_DISABLE;
			g.pull_down_en = GPIO_PULLDOWN_DISABLE;
			g.intr_type = GPIO_INTR_DISABLE;
			gpio_config(&g);
		}

		while (true) {
			if (_emergency) {
				constexpr static uint16_t stageDurationMs = 250;

				// Left wing
				_leftWing.fill(0xFF, 0x00, 0x00);
				_leftWing.flush();
				
				// Right wing
				_rightWing.fill(0xFF, 0x00, 0x00);
				_rightWing.flush();
				
				// Tail
				_tail.fill(0xFF, 0x00, 0x00);
				_tail.flush();
				
				// Cabin
				setCabin(true);

				if (delay(stageDurationMs))
					continue;
				
				// Left wing
				_leftWing.fill(0x00);
				_leftWing.flush();
				
				// Right wing
				_rightWing.fill(0x00);
				_rightWing.flush();
				
				// Tail
				_tail.fill(0x00);
				_tail.flush();
				
				// Cabin
				setCabin(false);
				
				if (delay(stageDurationMs))
					continue;
			}
			else {
				// Stages:     12345 ---------------
				// Left wing:  WRWRR RRRRRRRRRRRRRRR
				// Right wing: WGWGG GGGGGGGGGGGGGGG
				// Tail:       DDDDW DDDDDDDDDDDDDDD

				constexpr static uint8_t stageDurationMs = 40;
				constexpr static uint16_t totalDurationMs = 1000;

				// -------------------------------- Stage 1 --------------------------------

				// Cabin
				setCabin(ac.settings.lights.getCabin());

				// Left wing (strobe 1 or red)
				updateWingStrobe(_leftWing, 0xFF, 0x00, 0x00);

				// Right wing (strobe 1 or green)
				updateWingStrobe(_rightWing, 0x00, 0xFF, 0x00);

				// Tail (dimmed)
				updateTailStrobe(false);

				if (delay(stageDurationMs))
					continue;

				// -------------------------------- Stage 2 --------------------------------

				// Left wing (red)
				updateNavOrLanding(_leftWing, 0xFF, 0x00, 0x00);

				// Right wing (green)
				updateNavOrLanding(_rightWing, 0x00, 0xFF, 0x00);

				if (delay(stageDurationMs))
					continue;

				// -------------------------------- Stage 3 --------------------------------

				// Left wing (strobe 2 or red)
				updateWingStrobe(_leftWing, 0xFF, 0x00, 0x00);

				// Right wing (strobe 2 or green)
				updateWingStrobe(_rightWing, 0x00, 0xFF, 0x00);

				if (delay(stageDurationMs * 2))
					continue;

				// -------------------------------- Stage 4 --------------------------------

				// Left wing (red)
				updateNavOrLanding(_leftWing, 0xFF, 0x00, 0x00);

				// Right wing (green)
				updateNavOrLanding(_rightWing, 0x00, 0xFF, 0x00);

				// Tail (strobe)
				updateTailStrobe(true);

				// Delay 5
				if (delay(stageDurationMs))
					continue;

				// -------------------------------- Stage 5 --------------------------------

				// Tail (dimmed)
				updateTailStrobe(false);

				// 5 delays (not stages, DELAYS) so far, subtracting them from total time & delaying for what remains
				delay(totalDurationMs - stageDurationMs * 5);
			}
		}
	}
}