#pragma once

#include "Settings/MotorsSettings.hpp"
#include "Settings/ADIRSSettings.hpp"
#include "Settings/TrimSettings.hpp"
#include "Settings/LightsSettings.hpp"
#include "Settings/TransceiverSettings.hpp"
#include "Settings/AutopilotSettings.hpp"

namespace pizda {
	class SettingsAutopilot {
		public:
			AutopilotConfigurationSettings configuration {};
			AutopilotSelectionSettings selection {};
	};

	class Settings {
		public:
			MotorsSettings motors {};
			ADIRSSettings adirs {};
			LightsSettings lights {};
			TrimSettings trim {};
			TransceiverSettings transceiver {};
			SettingsAutopilot autopilot {};

			void setup() {
				motors.read();
				trim.read();
				lights.read();
				adirs.read();
				transceiver.read();

				autopilot.configuration.read();
				autopilot.selection.read();
			}
	};
}