#pragma once

#include <array>
#include <atomic>

#include "config.h"
#include "systems/motors/motor.h"

#include <PCA9685.h>

namespace pizda {
	using namespace YOBA;

	enum class MotorType : uint8_t {
		cameraPitch,
		cameraYaw,

		throttle,
		noseWheel,
		reverse,

		flapLeft,
		aileronLeft,
		
		flapRight,
		aileronRight,
		
		tailLeft,
		tailRight,

		maxValue = tailRight
	};
	
	class Motors {
		public:
			void setup();
			Motor* getMotor(uint8_t index);
			Motor* getMotor(MotorType type);
			void updateConfigurationsFromSettings();

		private:
			constexpr static auto _logTag = "Motors";

			PCA9685 _PCA9685 {};

			constexpr static uint8_t _motorCount = static_cast<uint8_t>(MotorType::maxValue) + 1;

			std::array<Motor, _motorCount> _motors {
				// Camera
				Motor {},
				Motor {},

				// Throttle / nose wheel / reverse
				Motor {},
				Motor {},
				Motor {},
				
				Motor {},
				Motor {},
				
				Motor {},
				Motor {},
				
				Motor {},
				Motor {},
			};

			static bool checkPCA9685Error(const PCA9685Error error);
			void onStart();
	};
}
