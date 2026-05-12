#pragma once

#include <cmath>
#include <numbers>
#include <algorithm>

namespace YOBA {
	class LowPassFilter {
		public:
			static float getDeltaTimeSFactor(const float factorPerSecond, const float deltaTimeS) {
				return factorPerSecond * deltaTimeS;
			}

			static float getDeltaTimeUsFactor(const float factorPerSecond, const uint32_t deltaTimeUs) {
				return factorPerSecond * static_cast<float>(deltaTimeUs) / 1'000'000.f;
			}

			static float apply(const float oldValue, const float newValue, float factor) {
				// Just in case
				factor = std::clamp(factor, 0.f, 1.f);
				
				return oldValue * (1.f - factor) + newValue * factor;
			}

			static float apply(const uint16_t oldValue, const uint16_t newValue, const uint16_t factor) {
				return oldValue * (0xFFFF - factor) / 0xFFFF + newValue * factor / 0xFFFF;
			}
			
			static float applyToAngle(float oldValue, const float newValue, const float factor) {
				auto delta = newValue - oldValue;
				
				// -170 to 170
				// delta = 170 - -170 = 340
				if (delta > std::numbers::pi_v<float>) {
					// delta = 340 - 360 = -20
					delta -= 2.0f * std::numbers::pi_v<float>;
				}
				else if (delta < -std::numbers::pi_v<float>) {
					delta += 2.0f * std::numbers::pi_v<float>;
				}
				
				// oldValue = 170 - -20 = 190;
				oldValue = newValue - delta;
				oldValue = apply(oldValue, newValue, factor);
				
				if (oldValue > std::numbers::pi_v<float>) {
					oldValue -= 2.0f * std::numbers::pi_v<float>;
				}
				else if (oldValue < -std::numbers::pi_v<float>) {
					oldValue += 2.0f * std::numbers::pi_v<float>;
				}
				
				return oldValue;
			}
	};
}