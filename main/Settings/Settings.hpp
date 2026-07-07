#pragma once

#include "Settings/MotorsSettings.hpp"
#include "Settings/TrimSettings.hpp"
#include "Settings/LightsSettings.hpp"
#include "Settings/TransceiverSettings.hpp"
#include "Settings/ADIRSSettings.hpp"
#include "Settings/AutopilotConfigurationSettings.hpp"
#include "Settings/FlightModeSelectionSettings.hpp"

namespace pizda {
	class Settings {
		public:
			MotorsSettings motors {};
			LightsSettings lights {};
			TrimSettings trim {};
			TransceiverSettings transceiver {};
			ADIRSSettings ADIRS {};
			AutopilotConfigurationSettings autopilotConfiguration {};
			FlightModeSelectionSettings flightModeSelection {};

			void setup() {
				motors.read();
				trim.read();
				lights.read();
				transceiver.read();
				ADIRS.read();
				autopilotConfiguration.read();
				flightModeSelection.read();
			}
	};
}