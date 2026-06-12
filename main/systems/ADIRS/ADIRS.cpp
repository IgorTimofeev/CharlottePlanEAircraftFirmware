#include "ADIRS.h"

#include <cmath>
#include <algorithm>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "aircraft.h"

namespace pizda {
	void ADIRS::setup() {
		xTaskCreatePinnedToCore(
			[](void* arg) {
				static_cast<ADIRS*>(arg)->onStart();
			},
			"ADIRS",
			4 * 1024,
			this,
			10,
			nullptr,
			0
		);
	}

	float ADIRS::getRollRad() const {
		return _rollRad.load(std::memory_order_acquire);
	}

	float ADIRS::getPitchRad() const {
		return _pitchRad.load(std::memory_order_acquire);
	}

	float ADIRS::getYawRad() const {
		return _yawRad.load(std::memory_order_acquire);
	}

	float ADIRS::getHeadingDeg() const {
		return _headingDeg.load(std::memory_order_acquire);
	}

	float ADIRS::getSlipAndSkidFactor() const {
		return _slipAndSkidFactor.load(std::memory_order_acquire);
	}

	float ADIRS::getAirspeedMPS() const {
		return _airspeedMPS.load(std::memory_order_acquire);
	}

	float ADIRS::getHomeLatitude() const {
		return _homeLatitude.load(std::memory_order_acquire);
	}

	float ADIRS::getHomeLongitude() const {
		return _homeLongitude.load(std::memory_order_acquire);
	}

	float ADIRS::getHomeAltitude() const {
		return _homeAltitude.load(std::memory_order_acquire);
	}

	float ADIRS::getLatitude() const {
		return _latitude.load(std::memory_order_acquire);
	}

	float ADIRS::getLongitude() const {
		return _longitude.load(std::memory_order_acquire);
	}

	float ADIRS::getAltitude() const {
		return _altitude.load(std::memory_order_acquire);
	}

	void ADIRS::setAirspeedMS(const float value) {
		_airspeedMPS.store(value, std::memory_order_release);
	}

	void ADIRS::setRollRad(const float value) {
		_rollRad.store(value, std::memory_order_release);
	}

	void ADIRS::setPitchRad(const float value) {
		_pitchRad.store(value, std::memory_order_release);
	}

	void ADIRS::setYawRad(const float value) {
		_yawRad.store(value, std::memory_order_release);
	}

	void ADIRS::updateHeadingFromYaw() {
		_headingDeg.store(toDegrees(-_yawRad), std::memory_order_release);
	}

	float ADIRS::computeAltitude(const float pressurePa, const float temperatureC, const uint32_t referencePressurePa, const float lapseRateKPM) {
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
		if (std::abs(lapseRateKPM) < 1e-6f) {
			// Isothermal atmosphere (lapse rate ≈ 0)
			return (R * temperatureK) / (g * M) * std::log(static_cast<float>(referencePressurePa) / pressurePa);
		}

		// Full formula with temperature gradient
		const float exponent = (R * lapseRateKPM) / (g * M);
		const float power = std::pow(pressurePa / static_cast<float>(referencePressurePa), exponent);
		const float altitude = (temperatureK / lapseRateKPM) * (1.0f - power);

		return altitude;
	}

	void ADIRS::updateSlipAndSkidFactor(const float lateralAccelerationG, const float maxG) {
		_slipAndSkidFactor.store(
			std::clamp<float>(-lateralAccelerationG - std::sin(getRollRad()), -maxG, maxG)
				/ static_cast<float>(maxG),
			std::memory_order_release
		);
	}

	void ADIRS::setPressurePa(const float value) {
		_pressurePa.store(value, std::memory_order_release);
	}

	void ADIRS::setTemperatureC(const float value) {
		_temperatureC.store(value, std::memory_order_release);
	}

	void ADIRS::updateAltitudeFromPressureTemperatureAndReferenceValue() {
		setAltitude(computeAltitude(
			_pressurePa.load(std::memory_order_acquire),
			_temperatureC.load(std::memory_order_acquire),
			Aircraft::getInstance().settings.adirs.getReferencePressurePa()
		));
	}

	void ADIRS::setHomeCoordinates(const float latitude, const float longitude, const float altitude) {
		_homeLatitude.store(latitude, std::memory_order_release);
		_homeLongitude.store(longitude, std::memory_order_release);
		_homeAltitude.store(altitude, std::memory_order_release);
	}

	void ADIRS::setLatitude(const float value) {
		_latitude.store(value, std::memory_order_release);
	}

	void ADIRS::setLongitude(const float value) {
		_longitude.store(value, std::memory_order_release);
	}

	void ADIRS::setAltitude(const float value) {
		_altitude.store(value, std::memory_order_release);
	}

	void ADIRS::onStart() {
		auto& ac = Aircraft::getInstance();

		while (true) {
			const auto system = ac.aircraftData.calibration.getSystem();

			if (
				ac.aircraftData.calibration.isCalibrating()
				&& (
					system == AircraftCalibrationSystem::accelAndGyro
					|| system == AircraftCalibrationSystem::mag
				)
			) {
				if (system == AircraftCalibrationSystem::accelAndGyro) {
					onCalibrateAccelAndGyro();
				}
				else {
					onCalibrateMag();
				}
				
				ac.aircraftData.calibration.setCalibrating(false);
			}
			else {
				onTick();
			}
		}
	}
}