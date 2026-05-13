#include "systems/lights/lights.h"

#include <esp_timer.h>
#include <driver/gpio.h>

#include "aircraft.h"

namespace pizda {
	void Lights::setup() {
		xTaskCreate(
			[](void* arg) {
				static_cast<Lights*>(arg)->onStart();
			},
			"lights",
			4096,
			this,
			tskIDLE_PRIORITY,
			&taskHandle
		);
	}

	void Lights::setCabinEnabled(const bool value) const {
		auto& ac = Aircraft::getInstance();
		
		if (value == ac.settings.lights.cabin)
			return;
		
		ac.settings.lights.cabin = value;
		ac.settings.lights.scheduleWrite();
		
		wake();
	}

	void Lights::setNavigationEnabled(const bool value) const {
		auto& ac = Aircraft::getInstance();
		
		if (value == ac.settings.lights.nav)
			return;
		
		ac.settings.lights.nav = value;
		ac.settings.lights.scheduleWrite();
		
		wake();
	}

	void Lights::setStrobeEnabled(const bool value) const {
		auto& ac = Aircraft::getInstance();
		
		if (value == ac.settings.lights.strobe)
			return;
		
		ac.settings.lights.strobe = value;
		ac.settings.lights.scheduleWrite();
		
		wake();
	}

	void Lights::setLandingEnabled(const bool value) const {
		auto& ac = Aircraft::getInstance();
		
		if (value == ac.settings.lights.landing)
			return;
		
		ac.settings.lights.landing = value;
		ac.settings.lights.scheduleWrite();
		
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
		if (ac.settings.lights.nav) {
			light.fill(r, g, b);
		}
		else {
			light.fill(0x00);
		}

		// Landing
		if (ac.settings.lights.landing)
			light.fill(0, light.getLength() / 2, 0xFF, 0xFF, 0xFF);

		light.flush();
	}

	void Lights::updateWingStrobe(const Light& light, const uint8_t r, const uint8_t g, const uint8_t b) {
		const auto& ac = Aircraft::getInstance();
		
		if (ac.settings.lights.strobe) {
			light.fill(0xFF);
			light.flush();
		}
		else {
			updateNavOrLanding(light, r, g, b);
		}
	}

	void Lights::updateTailStrobe(const bool active) const {
		const auto& ac = Aircraft::getInstance();

		_tail.fill(ac.settings.lights.strobe && active ? 0xFF : tailDimmedValue);
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

		//             0       500       1000 ms
		//             +--------+---------+
		// Left wing:  WRWRRRRRRRRRRRRRRRRR
		// Right wing: WGWGGGGGGGGGGGGGGGGG
		// Tail:       DDDDWDDDDDDDDDDDDDDD

		while (true) {
			if (_emergency) {
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

				if (delay(500))
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
				
				if (delay(500))
					continue;
			}
			else {
				// Cabin
				setCabin(ac.settings.lights.cabin);
				
				// Left wing (strobe 1 or red)
				updateWingStrobe(_leftWing, 0xFF, 0x00, 0x00);
				
				// Right wing (strobe 1 or green)
				updateWingStrobe(_rightWing, 0x00, 0xFF, 0x00);

				// Tail (dimmed)
				updateTailStrobe(false);

				if (delay(50))
					continue;

				// Left wing (red)
				updateNavOrLanding(_leftWing, 0xFF, 0x00, 0x00);

				// Right wing (green)
				updateNavOrLanding(_rightWing, 0x00, 0xFF, 0x00);
				
				if (delay(50))
					continue;

				// Left wing (strobe 2 or red)
				updateWingStrobe(_leftWing, 0xFF, 0x00, 0x00);
				
				// Right wing (strobe 2 or green)
				updateWingStrobe(_rightWing, 0x00, 0xFF, 0x00);
				
				if (delay(50 * 2))
					continue;

				// Left wing (red)
				updateNavOrLanding(_leftWing, 0xFF, 0x00, 0x00);
				
				// Right wing (green)
				updateNavOrLanding(_rightWing, 0x00, 0xFF, 0x00);
				
				// Tail (strobe)
				updateTailStrobe(true);

				if (delay(50))
					continue;

				// Tail (dimmed)
				updateTailStrobe(false);

				delay(50 * 15);
			}
		}
	}
}