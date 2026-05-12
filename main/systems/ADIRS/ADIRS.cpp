#include "ADIRS.h"

#include <cmath>
#include <algorithm>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "aircraft.h"

namespace pizda {
	void ADIRS::setup() {
		const auto& ac = Aircraft::getInstance();

		_referencePressurePa = ac.settings.adirs.referencePressurePa;

		xTaskCreate(
			[](void* arg) {
				static_cast<ADIRS*>(arg)->onStart();
			},
			"ADIRS",
			4 * 1024,
			this,
			10,
			nullptr
		);
	}

	void ADIRS::setupAsync() {
		xTaskCreate(
			[](void* arg) {
				static_cast<ADIRS*>(arg)->setup();
				vTaskDelete(nullptr);
			},
			"ADIRSSetup",
			4 * 1024,
			this,
			10,
			nullptr
		);
	}

	float ADIRS::getRollRad() const {
		return _rollRad;
	}

	float ADIRS::getPitchRad() const {
		return _pitchRad;
	}

	float ADIRS::getYawRad() const {
		return _yawRad;
	}

	float ADIRS::getHeadingDeg() const {
		return _headingDeg;
	}

	float ADIRS::getSlipAndSkidFactor() const {
		return _slipAndSkidFactor;
	}

	const GeoCoordinates& ADIRS::getCoordinates() const {
		return _coordinates;
	}

	const Vector3F& ADIRS::getIntegratedVelocityMPS() const {
		return _integratedVelocityMPS;
	}

	void ADIRS::setReferencePressurePa(const uint32_t value) {
		_referencePressurePa = value;
	}

	float ADIRS::getAirspeedMPS() const {
		return _airspeedMPS;
	}

	void ADIRS::setAirspeedMPS(const float value) {
		_airspeedMPS = value;
	}

	void ADIRS::setRollRad(const float value) {
		_rollRad = value;
	}

	void ADIRS::setPitchRad(const float value) {
		_pitchRad = value;
	}

	void ADIRS::setYawRad(const float value) {
		_yawRad = value;
	}

	void ADIRS::updateHeadingFromYaw() {
		_headingDeg = toDegrees(-_yawRad);
	}

	void ADIRS::setIntegratedVelocityMPS(const Vector3F& value) {
		_integratedVelocityMPS = value;
	}

	float ADIRS::computeAltitude(const float pressurePa, const float temperatureC, const uint32_t referencePressurePa, const float lapseRateKpm) {
		// Physical constants
		constexpr static float g = 9.80665f;       // Gravitational acceleration (m/s²)
		constexpr static float R = 8.314462618f;   // Universal gas constant (J/(mol·K))
		constexpr static float M = 0.0289644f;     // Molar mass of dry air (kg/mol)

		// Convert temperature from Celsius to Kelvin
		const float temperatureK = temperatureC + 273.15f;

		// Avoid division by zero and invalid values
		if (pressurePa <= 0.0f || temperatureK <= 0.0f)
			return 0.0f;

		// Barometric formula with temperature gradient consideration
		// Using International Standard Atmosphere (ISA) model
		// h = (T0 / L) * (1 - (P / P0)^(R * L / (g * M)))

		// If temperature lapse rate is close to zero, use simplified formula
		if (std::abs(lapseRateKpm) < 1e-6f) {
			// Isothermal atmosphere (lapse rate ≈ 0)
			return (R * temperatureK) / (g * M) * std::log(static_cast<float>(referencePressurePa) / pressurePa);
		}

		// Full formula with temperature gradient
		const float exponent = (R * lapseRateKpm) / (g * M);
		const float power = std::pow(pressurePa / static_cast<float>(referencePressurePa), exponent);
		const float altitude = (temperatureK / lapseRateKpm) * (1.0f - power);

		return altitude;
	}

	void ADIRS::updateSlipAndSkidFactor(const float lateralAccelerationG, const float maxG) {
		_slipAndSkidFactor =
			std::clamp<float>(-lateralAccelerationG - std::sin(getRollRad()), -maxG, maxG)
			/ static_cast<float>(maxG);
	}

	void ADIRS::setPressurePa(const float pressurePa) {
		_pressurePa = pressurePa;
	}

	void ADIRS::setTemperatureC(const float temperatureC) {
		_temperatureC = temperatureC;
	}

	void ADIRS::updateAltitudeFromPressureTemperatureAndReferenceValue() {
		_coordinates.setAltitude(computeAltitude(_pressurePa, _temperatureC, _referencePressurePa));
	}

	void ADIRS::setLatitude(const float value) {
		_coordinates.setLatitude(value);
	}

	void ADIRS::setLongitude(const float value) {
		_coordinates.setLongitude(value);
	}

	void ADIRS::onStart() {
		auto& ac = Aircraft::getInstance();

		while (true) {
			if (
				ac.aircraftData.calibration.calibrating
				&& (
					ac.aircraftData.calibration.system == AircraftCalibrationSystem::accelAndGyro
					|| ac.aircraftData.calibration.system == AircraftCalibrationSystem::mag
				)
			) {
				if (ac.aircraftData.calibration.system == AircraftCalibrationSystem::accelAndGyro) {
					onCalibrateAccelAndGyro();
				}
				else {
					onCalibrateMag();
				}
				
				ac.aircraftData.calibration.calibrating = false;
			}
			else {
				onTick();
			}
		}
	}
}