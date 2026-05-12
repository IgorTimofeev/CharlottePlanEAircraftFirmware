#pragma once

#include "systems/ADIRS/ADIRS.h"

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