#include "simADIRS.h"


#include <cstring>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_timer.h>

#include "systems/ADIRS/ADIRS.h"
#include "systems/simLink/simLink.h"

#include "aircraft.h"

#ifdef SIM

namespace pizda {
	void SimADIRS::onTick() {
		const auto& packet = Aircraft::getInstance().simLink.getLastSimPacket();
		
		updateSlipAndSkidFactor(packet.accelerationX, 2);
		
		setRollRad(packet.rollRad);
		setPitchRad(packet.pitchRad);
		setYawRad(packet.yawRad);
		updateHeadingFromYaw();
		
		setLatitude(packet.latitudeRad);
		setLongitude(packet.longitudeRad);
		
		setAirspeedMPS(packet.speedMPS);
		
		setPressurePa(packet.pressurePA);
		setTemperatureC(packet.temperatureC);
		updateAltitudeFromPressureTemperatureAndReferenceValue();
		
		vTaskDelay(pdMS_TO_TICKS(1'000 / 30));
	}
	
	void SimADIRS::onCalibrateAccelAndGyro() {
		simulateCalibration();
	}
	
	void SimADIRS::onCalibrateMag() {
		simulateCalibration();
	}
	
	void SimADIRS::simulateCalibration() {
		auto& ac = Aircraft::getInstance();
		
		constexpr static uint32_t durationMs = 10'000;
		constexpr static uint32_t percentDurationMs = durationMs / 100;
		
		for (uint8_t i = 0; i < 100; ++i) {
			ac.aircraftData.calibration.progress = static_cast<uint8_t>(static_cast<uint32_t>(i) * 0xFF / 100);
			ac.transceiver.enqueueAuxiliary(AircraftAuxiliaryPacketType::calibration);

			vTaskDelay(pdMS_TO_TICKS(percentDurationMs));
		}

		ac.aircraftData.calibration.progress = 0xFF;
		ac.transceiver.enqueueAuxiliary(AircraftAuxiliaryPacketType::calibration);
	}
}

#endif