#pragma once

#include "settings/motorsSettings.h"
#include "settings/ADIRSSettings.h"
#include "settings/trimSettings.h"
#include "settings/lightsSettings.h"
#include "settings/transceiverSettings.h"
#include "settings/autopilotSettings.h"

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