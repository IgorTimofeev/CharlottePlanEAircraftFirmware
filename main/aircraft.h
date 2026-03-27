#pragma once

// #define SIM

#include <esp_adc/adc_oneshot.h>

#include <battery.h>

#include "config.h"
#include "settings/settings.h"
#include "systems/lights/lights.h"
#include "systems/motors/motors.h"
#include "systems/transceiver/aircraftTransceiver.h"
#include "types/aircraftData.h"
#include "types/remoteData.h"
#include "systems/flyByWire/flyByWire.h"

#ifdef SIM
	#include "systems/ADIRS/simADIRS.h"
	#include "systems/simLink/simLink.h"
#else
	#include "systems/ADIRS/I2CADIRS.h"
#endif

namespace pizda {
	using namespace YOBA;

	class Aircraft {
		public:
			Settings settings {};

			Lights lights {};
			Motors motors {};

			Battery battery {
				config::battery::unit,
				getAssignedADCOneshotUnit(config::battery::unit),
				config::battery::channel,

				config::battery::voltageMin,
				config::battery::voltageMax,
				config::battery::voltageDividerR1,
				config::battery::voltageDividerR2
			};
			
			AircraftTransceiver transceiver {};
			
			#ifdef SIM
				SimLink simLink {};
				SimADIRS adirs {};
			#else
				I2CADIRS adirs {};
			#endif
			
			FlyByWire fbw {};
			
			AircraftData aircraftData {};
			RemoteData remoteData {};
			
			static Aircraft& getInstance();
			
			[[noreturn]] void start();

		private:
			constexpr static auto _logTag = "Aircraft";
			
			Aircraft() = default;

			adc_oneshot_unit_handle_t _ADCOneshotUnit2 {};

			constexpr adc_oneshot_unit_handle_t* getAssignedADCOneshotUnit(const adc_unit_t ADCUnit) {
				switch (ADCUnit) {
					case ADC_UNIT_2: return &_ADCOneshotUnit2;
					default: startErrorLoop("failed to find assigned ADC oneshot unit");
				}
			}

			[[noreturn]] static void startErrorLoop(const char* error);

			int64_t _batteryTickTime = 0;

			void batteryTick();
	};
}
