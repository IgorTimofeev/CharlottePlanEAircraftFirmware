#pragma once

#include <led_strip.h>

namespace pizda {
	class Light {
		public:
			Light(const gpio_num_t pin, const uint8_t length) : _length(length) {
				led_strip_config_t strip_config {};
				strip_config.strip_gpio_num = pin;
				strip_config.max_leds = length;
				strip_config.color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB;
				strip_config.led_model = LED_MODEL_WS2812;
				strip_config.flags.invert_out = false;

				led_strip_rmt_config_t rmt_config {};
				rmt_config.clk_src = RMT_CLK_SRC_DEFAULT; // different clock source can lead to different power consumption
				rmt_config.resolution_hz = 10000000;
				rmt_config.mem_block_symbols = 0;
				rmt_config.flags.with_dma = false;

				ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &_stripHandle));
			}

			uint8_t getLength() const {
				return _length;
			}

			void set(const uint8_t index, const uint8_t r, const uint8_t g, const uint8_t b) const {
				led_strip_set_pixel(_stripHandle, index, r, g, b);
			}

			void fill(const uint8_t index, const uint8_t count, const uint8_t r, const uint8_t g, const uint8_t b) const {
				for (uint8_t i = index; i < index + count; i++)
					set(i, r, g, b);
			}

			void fill(const uint8_t r, const uint8_t g, const uint8_t b) const {
				fill(0, _length, r, g, b);
			}

			void fill(const uint8_t value) const {
				fill(value, value, value);
			}

			void flush() const {
				led_strip_refresh(_stripHandle);
			}

		private:
			constexpr static auto _logTag = "Light";

			uint8_t _length;
			led_strip_handle_t _stripHandle {};
	};
}