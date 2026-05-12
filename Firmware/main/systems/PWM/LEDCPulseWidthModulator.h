#pragma once

#include <cstdint>
#include <driver/ledc.h>

#include "systems/PWM/pulseWidthModulator.h"

namespace pizda {
	class LEDCPulseWidthModulator : public PulseWidthModulator {
		public:
			LEDCPulseWidthModulator(
				const gpio_num_t pin,
				const ledc_channel_t ledcChannel,

				const ledc_timer_t ledcTimer = LEDC_TIMER_0,
				const int16_t tickFrequencyHz = 50,
				const uint8_t dutyLengthBits = 13
			) :
				_pin(pin),
				_ledcChannel(ledcChannel),

				_ledcTimer(ledcTimer),
				_tickFrequencyHz(tickFrequencyHz),
				_dutyLengthBits(dutyLengthBits)
			{

			}

			void setup() {
				ledc_timer_config_t timerConfig {};
				timerConfig.speed_mode = LEDC_LOW_SPEED_MODE;
				timerConfig.duty_resolution = static_cast<ledc_timer_bit_t>(_dutyLengthBits);
				timerConfig.timer_num = _ledcTimer;
				timerConfig.freq_hz = _tickFrequencyHz;
				timerConfig.clk_cfg = LEDC_AUTO_CLK;
				ESP_ERROR_CHECK(ledc_timer_config(&timerConfig));

				ledc_channel_config_t channelConfig {};
				channelConfig.speed_mode = LEDC_LOW_SPEED_MODE;
				channelConfig.channel = _ledcChannel;
				channelConfig.timer_sel = LEDC_TIMER_0;
				channelConfig.gpio_num = _pin;
				channelConfig.duty = 0;
				channelConfig.hpoint = 0;
				ESP_ERROR_CHECK(ledc_channel_config(&channelConfig));
			}

			void setPulseWidth(const uint32_t pulseWidthUs) override {
				// Pulse width -> duty cycle conversion
				const uint32_t tickDurationUs = 1'000'000 / _tickFrequencyHz;
				const uint32_t dutyMax = (1 << _dutyLengthBits) - 1;

				const auto duty = pulseWidthUs * dutyMax / tickDurationUs;

				ESP_ERROR_CHECK_WITHOUT_ABORT(ledc_set_duty(LEDC_LOW_SPEED_MODE, _ledcChannel, duty));
				ESP_ERROR_CHECK_WITHOUT_ABORT(ledc_update_duty(LEDC_LOW_SPEED_MODE, _ledcChannel));
			}

		private:
			gpio_num_t _pin;
			ledc_channel_t _ledcChannel;
			ledc_timer_t _ledcTimer;
			int16_t _tickFrequencyHz;
			uint8_t _dutyLengthBits;
	};
}
