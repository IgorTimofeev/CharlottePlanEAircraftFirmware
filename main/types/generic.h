#pragma once

#include <cstdint>
#include <algorithm>

#include <SX1262.h>

namespace pizda {
	enum class ConnectionState : uint8_t {
		initial,
		connected,
		disconnected
	};

	enum class AircraftCalibrationSystem : uint8_t {
		accelAndGyro,
		mag
	};

	enum class AutopilotLateralMode : uint8_t {
		dir,
		stab,
		hdg
	};

	enum class AutopilotVerticalMode : uint8_t {
		dir,
		stab,
		flc,
		alts,
		alt
	};

	class MotorSettings {
		public:
			uint16_t min = 1000;
			uint16_t max = 2000;
			bool reverse = false;

			void sanitize() {
				min = std::clamp<uint16_t>(min, 0, 3000);
				max = std::clamp<uint16_t>(max, 0, 3000);

				if (min > max)
					std::swap(min, max);
			}
	};

	class TransceiverCommunicationSettings {
		public:
			uint32_t frequencyHz = 0;
			SX1262::LoRaBandwidth bandwidth = SX1262::LoRaBandwidth::bw500_0;
			uint8_t spreadingFactor = 0;
			SX1262::LoRaCodingRate codingRate = SX1262::LoRaCodingRate::cr4_5;
			uint8_t syncWord = 0;
			uint16_t preambleLength = 0;

			uint8_t currentLimitMA = 0;
			int8_t powerDBm = 0;

			void sanitize() {
				frequencyHz = std::clamp<uint32_t>(frequencyHz, 120'000'000, 960'000'000);
				spreadingFactor = std::clamp<uint8_t>(spreadingFactor, 5, 12);

				currentLimitMA = std::clamp<uint8_t>(currentLimitMA, 0, 140);
				powerDBm = std::clamp<int8_t>(powerDBm, -17, 22);
			}
	};

	class PIDCoefficients {
		public:
			float p, i, d;
	};
}