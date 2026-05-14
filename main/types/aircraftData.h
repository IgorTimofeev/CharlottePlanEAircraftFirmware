#pragma once

#include <cmath>

#include "types/generic.h"

namespace pizda {
	class AircraftDataCalibration {
		public:
			bool calibrating = false;
			AircraftCalibrationSystem system = AircraftCalibrationSystem::accelAndGyro;
			uint8_t progress = 0xFF;
	};

	class AircraftDataCamera {
		public:
			int16_t pitchDeg = 0;
			int16_t yawDeg = 0;
	};

	class AircraftData {
		public:
			AircraftDataCalibration calibration {};
			AircraftDataCamera camera {};
	};
}