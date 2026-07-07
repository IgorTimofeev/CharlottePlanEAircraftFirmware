#pragma once

// #define SIM

#include <esp_adc/adc_oneshot.h>
#include <driver/i2c_master.h>

#include <ADCVoltmeter.hpp>

#include "Config.hpp"
#include "Settings/Settings.hpp"
#include "Systems/Lights/Lights.hpp"
#include "Systems/Motors/Motors.hpp"
#include "Systems/transceiver/AircraftTransceiver.hpp"
#include "Types/AircraftData.hpp"
#include "Types/RemoteData.hpp"
#include "Systems/FlyByWire/FlyByWire.hpp"

#ifdef SIM
	#include "Systems/ADIRS/SimADIRS.hpp"
	#include "Systems/SimLink/SimLink.hpp"
#else
	#include "Systems/ADIRS/I2CADIRS.hpp"
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
