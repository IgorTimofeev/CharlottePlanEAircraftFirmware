#pragma once

// #define SIM

#include <esp_adc/adc_oneshot.h>
#include <driver/i2c_master.h>

#include <ADCVoltmeter.h>

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

			i2c_master_bus_handle_t I2CMasterBusHandle {};

			Lights lights {};
			Motors motors {};
			ADCVoltmeter battery {};
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

			[[noreturn]] static void startErrorLoop(const char* error);

			// -------------------------------- ADC --------------------------------

			// This sucks tbh
			adc_oneshot_unit_handle_t _ADCOneshotUnit2 {};

			constexpr adc_oneshot_unit_handle_t getAssignedADCOneshotUnit(const adc_unit_t ADCUnit) const {
				switch (ADCUnit) {
					case ADC_UNIT_2: return _ADCOneshotUnit2;
					default: startErrorLoop("failed to find assigned ADC oneshot unit");
				}
			}

			// -------------------------------- Battery --------------------------------

			int64_t _batteryTickTime = 0;

			void batteryTick();
	};
}
