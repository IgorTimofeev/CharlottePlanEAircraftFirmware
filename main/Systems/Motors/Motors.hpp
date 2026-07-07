#pragma once

#include <array>
#include <atomic>

#include <PCA9685.hpp>

#include "Types/Generic.hpp"
#include "Systems/Motors/Motor.hpp"
#include "Config.hpp"

namespace pizda {
	using namespace YOBA;
	
	class Motors {
		public:
			void setup();
			Motor* getByType(MotorType type);

		private:
			constexpr static auto _logTag = "Motors";

			PCA9685 _PCA9685 {};

			Motor _cameraPitch {};
			Motor _cameraYaw {};

			Motor _throttleLeft {};
			Motor _throttleRight {};
			Motor _noseWheel {};

			Motor _flapLeft {};
			Motor _aileronLeft {};

			Motor _flapRight {};
			Motor _aileronRight {};

			Motor _tailLeft {};
			Motor _tailRight {};

			static bool checkPCA9685Error(const PCA9685Error error);
			[[noreturn]] void onStart();
	};
}
