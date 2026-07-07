#pragma once

#include "Systems/ADIRS/ADIRS.hpp"

namespace pizda {
	class SimADIRS : public ADIRS {
		protected:
			void setup();
			
		private:
			static void simulateCalibration();
	};
}