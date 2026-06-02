#include "systems/flyByWire/flyByWire.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_timer.h>
#include <esp_log.h>

#include <units.h>
#include <lowPassFilter.h>
#include <sys/stat.h>

#include "config.h"
#include "aircraft.h"
#include "utilities/math.h"

namespace pizda {
	void FlyByWire::setup() {
		xTaskCreate(
			[](void* arg) {
				static_cast<FlyByWire*>(arg)->onStart();
			},
			"FlyByWire",
			4 * 1024,
			this,
			20,
			nullptr
		);
	}
	
	float FlyByWire::getSelectedSpeedMps() const {
		return _speedSelectedMPS;
	}
	
	void FlyByWire::setSelectedSpeedMps(const float value) {
		_speedSelectedMPS = value;
	}
	
	uint16_t FlyByWire::getSelectedHeadingDeg() const {
		return _headingSelectedDeg;
	}
	
	void FlyByWire::setSelectedHeadingDeg(const uint16_t value) {
		_headingSelectedDeg = value;
	}
	
	float FlyByWire::getSelectedAltitudeM() const {
		return _altitudeSelectedM;
	}
	
	void FlyByWire::setSelectedAltitudeM(const float value) {
		_altitudeSelectedM = value;
	}

	float FlyByWire::getHoldAltitudeM() const {
		return _altitudeHoldM;
	}

	void FlyByWire::setHoldAltitudeM(const float value) {
		_altitudeHoldM = value;
	}
	
	AutopilotLateralMode FlyByWire::getLateralMode() const {
		return _lateralMode;
	}
	
	void FlyByWire::setLateralMode(const AutopilotLateralMode value) {
		if (_lateralMode == value)
			return;

		_lateralMode = value;
	}
	
	AutopilotVerticalMode FlyByWire::getVerticalMode() const {
		return _verticalMode;
	}
	
	void FlyByWire::setVerticalMode(const AutopilotVerticalMode value) {
		if (_verticalMode == value)
			return;

		_verticalMode = value;

		if (_verticalMode == AutopilotVerticalMode::alt)
			_altitudeHoldM = Aircraft::getInstance().adirs.getCoordinates().getAltitude();
	}
	
	bool FlyByWire::isAutothrottleEnabled() const {
		return _autothrottleEnabled;
	}
	
	void FlyByWire::setAutothrottleEnabled(const bool value) {
		_autothrottleEnabled = value;
	}
	
	bool FlyByWire::isAutopilotEngaged() const {
		return _autopilotEngaged;
	}
	
	void FlyByWire::setAutopilotEngaged(const bool value) {
		_autopilotEngaged = value;
	}

	float FlyByWire::getTargetRollRad() const {
		return _rollTargetRad;
	}
	
	float FlyByWire::getTargetPitchRad() const {
		return _pitchTargetRad;
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
		
		const auto speedMPS = ac.adirs.getAirspeedMPS();
		const auto altitudeM = ac.adirs.getCoordinates().getAltitude();
		const auto rollRad = ac.adirs.getRollRad();
		const auto pitchRad = ac.adirs.getPitchRad();
		const auto yawRad = ac.adirs.getYawRad();

		// -------------------------------- Lateral --------------------------------

		{
			if (_emergency) {
				_rollTargetRad = LowPassFilter::applyToAngle(
					_rollTargetRad,
					0,
					LowPassFilter::getDeltaTimeSFactor(ac.settings.autopilot.rollAngleLPFFactorPerSecond, deltaTimeS)
				);
			}
			else {
				if (_lateralMode == AutopilotLateralMode::stab && _autopilotEngaged) {
					_rollTargetRad = std::clamp(
						_rollTargetRad
							+ (ac.remoteData.raw.controls.ailerons * 2 - 1)
							* ac.settings.autopilot.stabilizedModeRollAngleIncrementRadPerSecond
							* deltaTimeS,
						-ac.settings.autopilot.maxRollAngleRad,
						ac.settings.autopilot.maxRollAngleRad
					);
				}
				else {
					float rollTargetRad = 0;

					switch (_lateralMode) {
						case AutopilotLateralMode::hdg: {
							const auto yawTargetRad = -normalizeAngleRadPi(toRadians(static_cast<float>(_headingSelectedDeg)));
							const auto yawTargetDeltaRad = normalizeAngleRadPi(yawTargetRad - yawRad);

							rollTargetRad = _yawDeltaToRollPID.tick(
								yawTargetDeltaRad,
								0,

								ac.settings.autopilot.PIDs.yawToRoll.p,
								ac.settings.autopilot.PIDs.yawToRoll.i,
								ac.settings.autopilot.PIDs.yawToRoll.d,

								deltaTimeS,

								-ac.settings.autopilot.maxRollAngleRad,
								ac.settings.autopilot.maxRollAngleRad
							);

							break;
						}
						default:
							break;
					}

					_rollTargetRad = LowPassFilter::applyToAngle(
						_rollTargetRad,
						rollTargetRad,
						LowPassFilter::getDeltaTimeSFactor(ac.settings.autopilot.rollAngleLPFFactorPerSecond, deltaTimeS)
					);
				}
			}
		}

		// -------------------------------- Vertical --------------------------------

		const auto speedTargetDeltaMPS = _speedSelectedMPS - speedMPS;
		float altitudeTargetDeltaM = _altitudeSelectedM - altitudeM;

		if (_verticalMode == AutopilotVerticalMode::flc) {
			// Was in FLC mode and become close enough to selected altitude
			if (std::abs(altitudeTargetDeltaM) <= config::FBW::altitudeDeltaForFLCToALTSSwitchM) {
				// Switching to ALTS mode
				_altitudeHoldM = _altitudeSelectedM;
				_verticalMode = AutopilotVerticalMode::alts;
			}
		}

		switch (_verticalMode) {
			case AutopilotVerticalMode::alts:
			case AutopilotVerticalMode::alt:
				altitudeTargetDeltaM = _altitudeHoldM - altitudeM;

				break;

			default:
				break;
		}

		const auto altitudeLow = altitudeTargetDeltaM > 0;
		const auto speedLow = speedTargetDeltaMPS > 0;

		{
			if (_emergency) {
				_pitchTargetRad = LowPassFilter::applyToAngle(
					_pitchTargetRad,
					0,
					LowPassFilter::getDeltaTimeSFactor(ac.settings.autopilot.pitchAngleLPFFactorPerSecond, deltaTimeS)
				);
			}
			else {
				if (_verticalMode == AutopilotVerticalMode::stab && _autopilotEngaged) {
					_pitchTargetRad = std::clamp(
						_pitchTargetRad
							- (ac.remoteData.raw.controls.elevator * 2 - 1)
							* ac.settings.autopilot.stabilizedModePitchAngleIncrementRadPerSecond
							* deltaTimeS,
						-ac.settings.autopilot.maxPitchAngleRad,
						ac.settings.autopilot.maxPitchAngleRad
					);
				}
				else {
					float pitchTargetRad = 0;

					switch (_verticalMode) {
						case AutopilotVerticalMode::alts:
						case AutopilotVerticalMode::alt: {
							// Relying on altitude difference, speed doesn't matter
							pitchTargetRad = _altitudeToPitchPID.tick(
								altitudeTargetDeltaM,
								0,

								ac.settings.autopilot.PIDs.altitudeToPitch.p,
								ac.settings.autopilot.PIDs.altitudeToPitch.i,
								ac.settings.autopilot.PIDs.altitudeToPitch.d,

								deltaTimeS,

								-1.f,
								1.f
							);

							pitchTargetRad = -ac.settings.autopilot.maxPitchAngleRad * pitchTargetRad;

							break;
						}
						case AutopilotVerticalMode::flc: {
							// Relying on speed difference, altitude doesn't matter
							if ((altitudeLow && !speedLow) || (!altitudeLow && speedLow)) {
								pitchTargetRad = _speedToPitchPID.tick(
									speedTargetDeltaMPS,
									0,

									ac.settings.autopilot.PIDs.speedToPitch.p,
									ac.settings.autopilot.PIDs.speedToPitch.i,
									ac.settings.autopilot.PIDs.speedToPitch.d,

									deltaTimeS,

									-1.f,
									1.f
								);

								pitchTargetRad = ac.settings.autopilot.maxPitchAngleRad * pitchTargetRad;
							}

							break;
						}
						default: {
							break;
						}
					}

					_pitchTargetRad = LowPassFilter::applyToAngle(
						_pitchTargetRad,
						pitchTargetRad,
						LowPassFilter::getDeltaTimeSFactor(ac.settings.autopilot.pitchAngleLPFFactorPerSecond, deltaTimeS)
					);
				}
			}
		}

		// -------------------------------- Ailerons --------------------------------

		if (_emergency || (_autopilotEngaged && _lateralMode != AutopilotLateralMode::dir)) {
			const auto rollTargetDeltaRad = normalizeAngleRadPi(_rollTargetRad - rollRad);

			_aileronsFactor = _rollToAileronsPID.tick(
				rollTargetDeltaRad,
				0,

				ac.settings.autopilot.PIDs.rollToAilerons.p,
				ac.settings.autopilot.PIDs.rollToAilerons.i,
				ac.settings.autopilot.PIDs.rollToAilerons.d,

				deltaTimeS,

				-1.f,
				1.f
			);

			// [-1; 1] => [0; 1]
			_aileronsFactor = (0.5f - _aileronsFactor / 2.f) * static_cast<float>(ac.settings.autopilot.maxAileronsPercent) / 100.f;
		}
		else {
			_aileronsFactor = std::clamp(ac.remoteData.raw.controls.ailerons + ac.settings.trim.aileronsTrim, 0.f, 1.f);
		}

		// -------------------------------- Elevator --------------------------------

		if (_emergency || (_autopilotEngaged && _verticalMode != AutopilotVerticalMode::dir)) {
			const auto pitchTargetDeltaRad = normalizeAngleRadPi(_pitchTargetRad - pitchRad);

			_elevatorFactor = _pitchToElevatorPID.tick(
				pitchTargetDeltaRad,
				0,

				ac.settings.autopilot.PIDs.pitchToElevator.p,
				ac.settings.autopilot.PIDs.pitchToElevator.i,
				ac.settings.autopilot.PIDs.pitchToElevator.d,

				deltaTimeS,

				-1.f,
				1.f
			);

			// [-1; 1] => [0; 1]
			_elevatorFactor = (0.5f + _elevatorFactor / 2.f) * static_cast<float>(ac.settings.autopilot.maxElevatorPercent) / 100.f;
		}
		else {
			_elevatorFactor = std::clamp(ac.remoteData.raw.controls.elevator + ac.settings.trim.elevatorTrim, 0.f, 1.f);
		}

		// -------------------------------- Rudder --------------------------------

		_rudderFactor =
			_emergency
			? 0.5f
			: std::clamp(ac.remoteData.raw.controls.rudder + ac.settings.trim.rudderTrim, 0.f, 1.f);

		// -------------------------------- Throttle --------------------------------

		if (_emergency) {
			_throttleFactor = 0.0f;
		}
		else {
			if (_autothrottleEnabled) {
				// FLC
				if (_autopilotEngaged && _verticalMode == AutopilotVerticalMode::flc) {
					_throttleFactor =
						static_cast<float>(
							altitudeLow
							? ac.settings.autopilot.maxThrottlePercent
							: ac.settings.autopilot.minThrottlePercent
						)
						/ 100.f;
				}
				// Others
				else {
					_throttleFactor = _speedToThrottlePID.tick(
						-speedTargetDeltaMPS,
						0,

						ac.settings.autopilot.PIDs.speedToThrottle.p,
						ac.settings.autopilot.PIDs.speedToThrottle.i,
						ac.settings.autopilot.PIDs.speedToThrottle.d,

						deltaTimeS,

						-1.f,
						1.f
					);

					// Mapping from [-1; 1] to [0; 1]
					_throttleFactor = (_throttleFactor + 1.f) / 2.f;

					_throttleFactor =
						// Min
						static_cast<float>(ac.settings.autopilot.minThrottlePercent) / 100.f
						// Factor
						+ _throttleFactor
						// Max - min
						* static_cast<float>(ac.settings.autopilot.maxThrottlePercent - ac.settings.autopilot.minThrottlePercent) / 100.f;

				}
			}
			else {
				_throttleFactor = ac.remoteData.raw.controls.throttle;
			}
		}
	}

	void FlyByWire::applyData() const {
		auto& ac = Aircraft::getInstance();
		
		// Throttle
		{
			const auto motor = ac.motors.getByType(MotorType::throttle);
			
			if (motor)
				motor->setPowerF(_throttleFactor);
		}
		
		// Ailerons
		{
			const auto leftAileronMotor = ac.motors.getByType(MotorType::aileronLeft);
			const auto rightAileronMotor = ac.motors.getByType(MotorType::aileronRight);

			leftAileronMotor->setPowerF(_aileronsFactor);
			rightAileronMotor->setPowerF(1.f - _aileronsFactor);
		}
		
		// Elevator & rudder
		{
			const auto leftTailMotor = ac.motors.getByType(MotorType::tailLeft);
			const auto rightTailMotor = ac.motors.getByType(MotorType::tailRight);
			const auto noseWheelMotor = ac.motors.getByType(MotorType::noseWheel);

			// ESP_LOGI(_logTag, "raw rudder: %f, elevator: %f", ac.remoteData.raw.controls.rudder, ac.remoteData.raw.controls.elevator);
			// ESP_LOGI(_logTag, "factors rudder: %f, elevator: %f", _rudderFactor, _elevatorFactor);

			#ifdef SIM
				leftTailMotor->setPowerF(_elevatorFactor);
				rightTailMotor->setPowerF(_rudderFactor);

			#else
				// V-tail mixing
				const auto elevatorPowerShifted = _elevatorFactor * 2 - 1;
				const auto rudderPowerShifted = _rudderFactor * 2 - 1;
				const auto leftPower = (std::clamp(elevatorPowerShifted - rudderPowerShifted, -1.f, 1.f) + 1.f) / 2.f;
				const auto rightPower = (std::clamp(elevatorPowerShifted + rudderPowerShifted, -1.f, 1.f) + 1.f) / 2.f;

				// ESP_LOGI(_logTag, "tail power left: %f, left: %f", leftPower, rightPower);

				leftTailMotor->setPowerF(leftPower);
				rightTailMotor->setPowerF(rightPower);

			#endif

			// Nose wheel
			noseWheelMotor->setPowerF(ac.remoteData.raw.controls.rudder);
		}
		
		// Flaps
		{
			const auto leftFlapMotor = ac.motors.getByType(MotorType::flapLeft);
			const auto rightFlapMotor = ac.motors.getByType(MotorType::flapRight);
			
			if (!leftFlapMotor || !rightFlapMotor)
				return;
			
			leftFlapMotor->setPowerF(ac.remoteData.raw.controls.flaps);
			rightFlapMotor->setPowerF(ac.remoteData.raw.controls.flaps);
		}

		// Camera
		{
			const auto cameraPitchMotor = ac.motors.getByType(MotorType::cameraPitch);
			const auto cameraYawMotor = ac.motors.getByType(MotorType::cameraYaw);

			if (!cameraPitchMotor || !cameraYawMotor)
				return;

			const auto setPower = [](Motor* motor, const int16_t angleDeg) {
				motor->setPower(
					(static_cast<int32_t>(angleDeg) - config::camera::servoMinDeg)
					* static_cast<int32_t>(MotorSettings::powerMax)
					/ static_cast<int32_t>(config::camera::servoAngularRangeDeg)
				);
			};

			// ESP_LOGI("cam", "pitch: %d, yaw: %d", ac.aircraftData.camera.pitchDeg, ac.aircraftData.camera.yawDeg);

			setPower(cameraPitchMotor, ac.aircraftData.camera.pitchDeg);
			setPower(cameraYawMotor, ac.aircraftData.camera.yawDeg);
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