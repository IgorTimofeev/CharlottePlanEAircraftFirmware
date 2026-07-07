#include "Systems/FlyByWire/FlyByWire.hpp"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_timer.h>
#include <esp_log.h>
#include <sys/stat.h>

#include <Units.hpp>
#include <EMAFilter.hpp>

#include "Config.hpp"
#include "Aircraft.hpp"
#include "Utilities/Math.hpp"

namespace pizda {
	void FlyByWire::setup() {
		xTaskCreatePinnedToCore(
			[](void* arg) {
				static_cast<FlyByWire*>(arg)->onStart();
			},
			"FlyByWire",
			4 * 1024,
			this,
			20,
			nullptr,
			0
		);
	}

	void FlyByWire::setAutothrottleEnabled(const bool value) {
		_autothrottleEnabled.store(value, std::memory_order_release);
	}

	bool FlyByWire::isAutopilotEngaged() const {
		return _autopilotEngaged.load(std::memory_order_acquire);
	}

	void FlyByWire::setAutopilotEngaged(const bool value) {
		_autopilotEngaged.store(value, std::memory_order_release);
	}

	float FlyByWire::getTargetRollRad() const {
		return _rollTargetRad;
	}
	
	float FlyByWire::getTargetPitchRad() const {
		return _pitchTargetRad;
	}

	float FlyByWire::getThrottleFactor() const {
		return _throttleFactor;
	}

	AutopilotLateralMode FlyByWire::getLateralMode() const {
		return _lateralMode.load(std::memory_order_acquire);
	}

	void FlyByWire::setLateralMode(const AutopilotLateralMode value) {
		_lateralMode.store(value, std::memory_order_release);
	}

	AutopilotVerticalMode FlyByWire::getVerticalMode() const {
		return _verticalMode.load(std::memory_order_acquire);
	}

	void FlyByWire::setVerticalMode(const AutopilotVerticalMode value) {
		_verticalMode.store(value, std::memory_order_release);

		if (value == AutopilotVerticalMode::alt)
			setHoldAltitudeM(Aircraft::getInstance().adirs.getAltitude());
	}

	float FlyByWire::getHoldAltitudeM() const {
		return _holdAltitudeM.load(std::memory_order_acquire);
	}

	void FlyByWire::setHoldAltitudeM(const float value) {
		_holdAltitudeM.store(value, std::memory_order_release);
	}

	bool FlyByWire::isAutothrottleEnabled() const {
		return _autothrottleEnabled.load(std::memory_order_acquire);
	}

	void FlyByWire::setEmergency(const bool state) {
		_emergency = state;
	}

	float FlyByWire::predictValue(const float valueDelta, const float deltaTimeS, const float dueTimeS) {
		// valueDelta - deltaTimeS
		// x          - dueTimeS
		return valueDelta * dueTimeS / deltaTimeS;
	}
	
	void FlyByWire::computeData() {
		auto& ac = Aircraft::getInstance();

		const auto deltaTimeS = static_cast<float>(esp_timer_get_time() - _computationTimeUs) / 1'000'000;
		_computationTimeUs = esp_timer_get_time();
		
		// -------------------------------- Values --------------------------------
		
		const auto airspeedMPS = ac.adirs.getAirspeedMPS();
		const auto altitudeM = ac.adirs.getAltitude();
		const auto rollRad = ac.adirs.getRollRad();
		const auto pitchRad = ac.adirs.getPitchRad();
		const auto yawRad = ac.adirs.getYawRad();

		const auto lateralMode = getLateralMode();
		auto verticalMode = getVerticalMode();

		const auto selectedSpeedMPS = ac.settings.autopilot.selection.getSelectedSpeedMPS();
		const auto selectedHeadingDeg = ac.settings.autopilot.selection.getSelectedHeadingDeg();
		const auto selectedAltitudeM = ac.settings.autopilot.selection.getSelectedAltitudeM();
		auto holdAltitudeM = getHoldAltitudeM();

		// -------------------------------- Lateral --------------------------------

		{
			if (_emergency) {
				_rollTargetRad = EMAFilter::applyToAngle(
					_rollTargetRad,
					0,
					EMAFilter::getDeltaTimeSFactor(ac.settings.autopilot.configuration.rollAngleEMAFilterFactorPerSecond, deltaTimeS)
				);
			}
			else {
				if (lateralMode == AutopilotLateralMode::stab && _autopilotEngaged) {
					_rollTargetRad = std::clamp(
						_rollTargetRad
							+ (ac.remoteData.controls.getAilerons() * 2 - 1)
							* ac.settings.autopilot.configuration.stabilizedModeRollAngleIncrementRadPerSecond
							* deltaTimeS,
						-ac.settings.autopilot.configuration.maxRollAngleRad,
						ac.settings.autopilot.configuration.maxRollAngleRad
					);
				}
				else {
					float rollTargetRad = 0;

					switch (lateralMode) {
						case AutopilotLateralMode::hdg: {
							const auto yawTargetRad = -normalizeAngleRadPi(toRadians(static_cast<float>(selectedHeadingDeg)));
							const auto yawTargetDeltaRad = normalizeAngleRadPi(yawTargetRad - yawRad);

							rollTargetRad = _yawDeltaToRollPID.tick(
								yawTargetDeltaRad,
								0,

								ac.settings.autopilot.configuration.PIDs.yawToRoll.p,
								ac.settings.autopilot.configuration.PIDs.yawToRoll.i,
								ac.settings.autopilot.configuration.PIDs.yawToRoll.d,

								deltaTimeS,

								-ac.settings.autopilot.configuration.maxRollAngleRad,
								ac.settings.autopilot.configuration.maxRollAngleRad
							);

							break;
						}
						default:
							break;
					}

					_rollTargetRad = EMAFilter::applyToAngle(
						_rollTargetRad,
						rollTargetRad,
						EMAFilter::getDeltaTimeSFactor(ac.settings.autopilot.configuration.rollAngleEMAFilterFactorPerSecond, deltaTimeS)
					);
				}
			}
		}

		// -------------------------------- Vertical --------------------------------

		const auto speedTargetDeltaMPS = selectedSpeedMPS - airspeedMPS;
		float altitudeTargetDeltaM = selectedAltitudeM - altitudeM;

		if (verticalMode == AutopilotVerticalMode::flc) {
			// Was in FLC mode and become close enough to selected altitude
			if (std::abs(altitudeTargetDeltaM) <= config::FBW::altitudeDeltaForFLCToALTSSwitchM) {
				// Switching to ALTS mode
				holdAltitudeM = selectedAltitudeM;
				verticalMode = AutopilotVerticalMode::alts;

				setHoldAltitudeM(holdAltitudeM);
				setVerticalMode(verticalMode);

				ac.settings.autopilot.selection.writeLater();
			}
		}

		switch (verticalMode) {
			case AutopilotVerticalMode::alts:
			case AutopilotVerticalMode::alt:
				altitudeTargetDeltaM = holdAltitudeM - altitudeM;

				break;

			default:
				break;
		}

		const auto altitudeLow = altitudeTargetDeltaM > 0;
		const auto speedLow = speedTargetDeltaMPS > 0;

		{
			if (_emergency) {
				_pitchTargetRad = EMAFilter::applyToAngle(
					_pitchTargetRad,
					0,
					EMAFilter::getDeltaTimeSFactor(ac.settings.autopilot.configuration.pitchAngleEMAFilterFactorPerSecond, deltaTimeS)
				);
			}
			else {
				if (verticalMode == AutopilotVerticalMode::stab && _autopilotEngaged) {
					_pitchTargetRad = std::clamp(
						_pitchTargetRad
							- (ac.remoteData.controls.getElevator() * 2 - 1)
							* ac.settings.autopilot.configuration.stabilizedModePitchAngleIncrementRadPerSecond
							* deltaTimeS,
						-ac.settings.autopilot.configuration.maxPitchAngleRad,
						ac.settings.autopilot.configuration.maxPitchAngleRad
					);
				}
				else {
					float pitchTargetRad = 0;

					switch (verticalMode) {
						case AutopilotVerticalMode::alts:
						case AutopilotVerticalMode::alt: {
							// Relying on altitude difference, speed doesn't matter
							pitchTargetRad = _altitudeToPitchPID.tick(
								altitudeTargetDeltaM,
								0,

								ac.settings.autopilot.configuration.PIDs.altitudeToPitch.p,
								ac.settings.autopilot.configuration.PIDs.altitudeToPitch.i,
								ac.settings.autopilot.configuration.PIDs.altitudeToPitch.d,

								deltaTimeS,

								-1.f,
								1.f
							);

							pitchTargetRad = -ac.settings.autopilot.configuration.maxPitchAngleRad * pitchTargetRad;

							break;
						}
						case AutopilotVerticalMode::flc: {
							// Relying on speed difference, altitude doesn't matter
							if ((altitudeLow && !speedLow) || (!altitudeLow && speedLow)) {
								pitchTargetRad = _speedToPitchPID.tick(
									speedTargetDeltaMPS,
									0,

									ac.settings.autopilot.configuration.PIDs.speedToPitch.p,
									ac.settings.autopilot.configuration.PIDs.speedToPitch.i,
									ac.settings.autopilot.configuration.PIDs.speedToPitch.d,

									deltaTimeS,

									-1.f,
									1.f
								);

								pitchTargetRad = ac.settings.autopilot.configuration.maxPitchAngleRad * pitchTargetRad;
							}

							break;
						}
						default: {
							break;
						}
					}

					_pitchTargetRad = EMAFilter::applyToAngle(
						_pitchTargetRad,
						pitchTargetRad,
						EMAFilter::getDeltaTimeSFactor(ac.settings.autopilot.configuration.pitchAngleEMAFilterFactorPerSecond, deltaTimeS)
					);
				}
			}
		}

		// -------------------------------- Ailerons --------------------------------

		if (_emergency || (_autopilotEngaged && lateralMode != AutopilotLateralMode::dir)) {
			const auto rollTargetDeltaRad = normalizeAngleRadPi(_rollTargetRad - rollRad);

			_aileronsFactor = _rollToAileronsPID.tick(
				rollTargetDeltaRad,
				0,

				ac.settings.autopilot.configuration.PIDs.rollToAilerons.p,
				ac.settings.autopilot.configuration.PIDs.rollToAilerons.i,
				ac.settings.autopilot.configuration.PIDs.rollToAilerons.d,

				deltaTimeS,

				-1.f,
				1.f
			);

			// [-1; 1] => [0; 1]
			_aileronsFactor = (0.5f - _aileronsFactor / 2.f) * static_cast<float>(ac.settings.autopilot.configuration.maxAileronsPercent) / 100.f;
		}
		else {
			_aileronsFactor = std::clamp(ac.remoteData.controls.getAilerons() + ac.settings.trim.getAileronsTrim(), 0.f, 1.f);
		}

		// -------------------------------- Elevator --------------------------------

		if (_emergency || (_autopilotEngaged && verticalMode != AutopilotVerticalMode::dir)) {
			const auto pitchTargetDeltaRad = normalizeAngleRadPi(_pitchTargetRad - pitchRad);

			_elevatorFactor = _pitchToElevatorPID.tick(
				pitchTargetDeltaRad,
				0,

				ac.settings.autopilot.configuration.PIDs.pitchToElevator.p,
				ac.settings.autopilot.configuration.PIDs.pitchToElevator.i,
				ac.settings.autopilot.configuration.PIDs.pitchToElevator.d,

				deltaTimeS,

				-1.f,
				1.f
			);

			// [-1; 1] => [0; 1]
			_elevatorFactor = (0.5f + _elevatorFactor / 2.f) * static_cast<float>(ac.settings.autopilot.configuration.maxElevatorPercent) / 100.f;
		}
		else {
			_elevatorFactor = std::clamp(ac.remoteData.controls.getElevator() + ac.settings.trim.getElevatorTrim(), 0.f, 1.f);
		}

		// -------------------------------- Rudder --------------------------------

		_noseWheelFactor = std::clamp(ac.remoteData.controls.getRudder() + ac.settings.trim.getRudderTrim(), 0.f, 1.f);

		_rudderFactor =
			_emergency || (_autopilotEngaged && lateralMode != AutopilotLateralMode::dir)
			? 0.5f
			: _noseWheelFactor;

		// -------------------------------- Throttle --------------------------------

		if (_emergency) {
			_throttleFactor = 0.0f;
		}
		else {
			if (_autothrottleEnabled) {
				// FLC
				if (_autopilotEngaged && verticalMode == AutopilotVerticalMode::flc) {
					_throttleFactor =
						static_cast<float>(
							altitudeLow
							? ac.settings.autopilot.configuration.maxThrottlePercent
							: ac.settings.autopilot.configuration.minThrottlePercent
						)
						/ 100.f;
				}
				// Others
				else {
					_throttleFactor = _speedToThrottlePID.tick(
						-speedTargetDeltaMPS,
						0,

						ac.settings.autopilot.configuration.PIDs.speedToThrottle.p,
						ac.settings.autopilot.configuration.PIDs.speedToThrottle.i,
						ac.settings.autopilot.configuration.PIDs.speedToThrottle.d,

						deltaTimeS,

						-1.f,
						1.f
					);

					// Mapping from [-1; 1] to [0; 1]
					_throttleFactor = (_throttleFactor + 1.f) / 2.f;

					_throttleFactor =
						// Min
						static_cast<float>(ac.settings.autopilot.configuration.minThrottlePercent) / 100.f
						// Factor
						+ _throttleFactor
						// Max - min
						* static_cast<float>(ac.settings.autopilot.configuration.maxThrottlePercent - ac.settings.autopilot.configuration.minThrottlePercent) / 100.f;

				}
			}
			else {
				_throttleFactor = ac.remoteData.controls.getThrottle();
			}
		}
	}

	void FlyByWire::applyData() const {
		auto& ac = Aircraft::getInstance();
		
		// Throttle
		ac.motors.getByType(MotorType::throttleLeft)->setPowerF(_throttleFactor);
		ac.motors.getByType(MotorType::throttleRight)->setPowerF(_throttleFactor);

		// Ailerons
		ac.motors.getByType(MotorType::aileronLeft)->setPowerF(_aileronsFactor);
		ac.motors.getByType(MotorType::aileronRight)->setPowerF(1.f - _aileronsFactor);

		// Elevator & rudder
		{
			// ESP_LOGI(_logTag, "raw rudder: %f, elevator: %f", ac.remoteData.controls.rudder, ac.remoteData.controls.elevator);
			// ESP_LOGI(_logTag, "factors rudder: %f, elevator: %f", _rudderFactor, _elevatorFactor);

			#ifdef SIM
				ac.motors.getByType(MotorType::tailLeft)->setPowerF(_elevatorFactor);
				ac.motors.getByType(MotorType::tailRight)->setPowerF(_rudderFactor);

			#else
				// V-tail mixing
				const auto elevatorPowerShifted = _elevatorFactor * 2 - 1;
				const auto rudderPowerShifted = _rudderFactor * 2 - 1;
				const auto leftPower = (std::clamp(elevatorPowerShifted - rudderPowerShifted, -1.f, 1.f) + 1.f) / 2.f;
				const auto rightPower = (std::clamp(elevatorPowerShifted + rudderPowerShifted, -1.f, 1.f) + 1.f) / 2.f;

				// ESP_LOGI(_logTag, "tail power left: %f, left: %f", leftPower, rightPower);

				ac.motors.getByType(MotorType::tailLeft)->setPowerF(leftPower);
				ac.motors.getByType(MotorType::tailRight)->setPowerF(rightPower);

			#endif
		}

		// Nose wheel
		ac.motors.getByType(MotorType::noseWheel)->setPowerF(_noseWheelFactor);

		// Flaps
		ac.motors.getByType(MotorType::flapLeft)->setPowerF(ac.remoteData.controls.getFlaps());
		ac.motors.getByType(MotorType::flapRight)->setPowerF(ac.remoteData.controls.getFlaps());

		// Camera
		{
			const auto setPower = [&ac](MotorType motorType, const int16_t angleDeg) {
				ac.motors.getByType(motorType)->setPower(
					(static_cast<int32_t>(angleDeg) - config::camera::servoMinDeg)
					* static_cast<int32_t>(MotorSettings::powerMax)
					/ static_cast<int32_t>(config::camera::servoAngularRangeDeg)
				);
			};

			// ESP_LOGI("cam", "pitch: %d, yaw: %d", ac.aircraftData.camera.pitchDeg, ac.aircraftData.camera.yawDeg);

			setPower(MotorType::cameraPitch, ac.aircraftData.camera.getPitchDeg());
			setPower(MotorType::cameraYaw, ac.aircraftData.camera.getYawDeg());
		}
	}

	[[noreturn]] void FlyByWire::onStart() {
		_computationTimeUs = esp_timer_get_time();

		while (true) {
			computeData();
			applyData();
			
			vTaskDelay(pdMS_TO_TICKS(1'000 / _tickFrequencyHz));
			// vTaskDelay(pdMS_TO_TICKS(1'000 / 1));
		}
	}
}