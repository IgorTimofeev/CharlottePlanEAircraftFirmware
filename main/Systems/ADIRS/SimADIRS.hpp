#pragma once

#include "Systems/ADIRS/ADIRS.hpp"

namespace pizda {
	class SimADIRS : public ADIRS {
		protected:
			void onTick() override;
			void onCalibrateAccelAndGyro() override;
			void onCalibrateMag() override;
			
		private:
			static void simulateCalibration();
	};
}