#pragma once

#include <cmath>

#include "types/generic.h"

namespace pizda {
	class RemoteDataRawControls {
		public:
			// Factor in [0.0; 1.0] range
			float throttle = 0;
			float ailerons = 0.5f;
			float elevator = 0.5f;
			float rudder = 0.5f;
			float flaps = 0;
	};

	class RemoteDataRawCamera {
		public:
			// [0.0; 1.0]
			float pitchFactor01 = 0.5f;
			float yawFactor01 = 0.5f;
	};
	
	class RemoteDataRaw {
		public:
			RemoteDataRawControls controls {};
			RemoteDataRawCamera camera {};
	};
	
	class RemoteDataComputed {
		public:
		
	};
	
	class RemoteData {
		public:
			RemoteDataRaw raw {};
			RemoteDataComputed computed {};
	};
}