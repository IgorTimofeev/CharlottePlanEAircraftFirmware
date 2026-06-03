#include "systems/transceiver/aircraftTransceiver.h"

#include <utility>

#include "aircraft.h"
#include "systems/motors/motors.h"

namespace pizda {
	// -------------------------------- Generic --------------------------------

	AircraftTransceiver::AircraftTransceiver() : Transceiver({
		PacketSequenceItem(AircraftPacketType::STierTelemetry),
		PacketSequenceItem(AircraftPacketType::STierTelemetry, true),

		PacketSequenceItem(AircraftPacketType::ATierTelemetry),
		PacketSequenceItem(AircraftPacketType::STierTelemetry),

		PacketSequenceItem(AircraftPacketType::ATierTelemetry, true),
		PacketSequenceItem(AircraftPacketType::STierTelemetry, true),

		PacketSequenceItem(AircraftPacketType::BTierTelemetry)
	}) {

	}

	void AircraftTransceiver::onStart() {
		auto& ac = Aircraft::getInstance();
		// Receive -> wait -> transmit

		bool receiveMode = true;
		int64_t transmitTime = 0;

		while (true) {
			// Should schedule communication settings sync check
			if (_communicationSettingsACKTime < 0) {
				setCommunicationSettings(_tmpCommunicationSettings);

				_communicationSettingsACKTime = esp_timer_get_time() + 2'000'000;
			}
			// Should perform communication settings sync check
			else if (_communicationSettingsACKTime > 0 && esp_timer_get_time() >= _communicationSettingsACKTime) {
				// Received and decoded enough packets to consider the connection is stable
				if (getRXPPS() > 5) {
					ESP_LOGI(_logTag, "communication settings synchronized");

					ac.settings.transceiver.communication = _tmpCommunicationSettings;
					ac.settings.transceiver.scheduleWrite();
				}
				// Or not enough...
				else {
					ESP_LOGI(_logTag, "communication settings change timed out, falling back to default");

					// Falling back to default communication settings
					setCommunicationSettings(config::XCVR::communicationSettings);
				}

				_communicationSettingsACKTime = 0;
			}

			if (receiveMode) {
				if (receive(1'000'000)) {
					receiveMode = false;

					transmitTime = esp_timer_get_time() + 8'000;
				}
			}
			else {
				if (esp_timer_get_time() >= transmitTime) {
					transmit(1'000'000);

					receiveMode = true;
				}
				else {
					taskYIELD();
				}
			}

			PPSTick();
		}
	}

	void AircraftTransceiver::onConnectionStateChanged() {
		auto& ac = Aircraft::getInstance();

		// ESP_LOGI(_logTag, "onConnectionStateChanged: %d", (uint8_t) getConnectionState());

		switch (getConnectionState()) {
			case ConnectionState::initial: {

				break;
			}
			case ConnectionState::connected: {

				break;
			}
			case ConnectionState::disconnected: {
				ac.fbw.setEmergency(true);
				ac.lights.setEmergencyEnabled(true);

				break;
			}
			case ConnectionState::reconnected: {
				ac.fbw.setEmergency(false);
				ac.lights.setEmergencyEnabled(false);

				break;
			}
		}
	}
	
	// -------------------------------- Receiving --------------------------------
	
	bool AircraftTransceiver::onReceive(BitStream& stream, const RemotePacketType packetType, const uint8_t payloadLength) {
		switch (packetType) {
			case RemotePacketType::controls:
				return receiveRemoteControlsPacket(stream, payloadLength);
			
			case RemotePacketType::system:
				return receiveRemoteSystemPacket(stream, payloadLength);

			default:
				ESP_LOGE(_logTag, "failed to receive packet: unsupported type %d", std::to_underlying(packetType));
				return false;
		}
		
		return true;
	}

	bool AircraftTransceiver::receiveRemoteControlsPacket(BitStream& stream, const uint8_t payloadLength) {
		auto& ac = Aircraft::getInstance();

		if (!validatePayloadChecksumAndLength(
			stream,
			RemoteControlsPacket::motorLengthBits * 5,
			payloadLength
		))
			return false;

		const auto readMotor = [&stream] {
			return
				static_cast<float>(stream.readUint16(RemoteControlsPacket::motorLengthBits))
				/ static_cast<float>((1 << RemoteControlsPacket::motorLengthBits) - 1);
		};

		ac.remoteData.raw.controls.throttle = readMotor();
		ac.remoteData.raw.controls.ailerons = readMotor();
		ac.remoteData.raw.controls.elevator = readMotor();
		ac.remoteData.raw.controls.rudder = readMotor();
		ac.remoteData.raw.controls.flaps = readMotor();

		return true;
	}

	bool AircraftTransceiver::receiveRemoteSystemPacket(BitStream& stream, const uint8_t payloadLength) {
		const auto type = static_cast<RemoteSystemPacketType>(stream.readUint8(RemoteSystemPacket::typeLengthBits));

		switch (type) {
			case RemoteSystemPacketType::trim:
				return receiveRemoteSystemTrimPacket(stream, payloadLength);

			case RemoteSystemPacketType::lights:
				return receiveRemoteSystemLightsPacket(stream, payloadLength);

			case RemoteSystemPacketType::baro:
				return receiveRemoteSystemBaroPacket(stream, payloadLength);

			case RemoteSystemPacketType::calibrate:
				return receiveRemoteSystemCalibratePacket(stream, payloadLength);

			case RemoteSystemPacketType::autopilot:
				return receiveRemoteSystemAutopilotPacket(stream, payloadLength);

			case RemoteSystemPacketType::camera:
				return receiveRemoteSystemCameraPacket(stream, payloadLength);

			case RemoteSystemPacketType::motors:
				return receiveRemoteSystemMotorsPacket(stream, payloadLength);

			case RemoteSystemPacketType::ADIRS:
				return receiveRemoteSystemADIRSPacket(stream, payloadLength);

			case RemoteSystemPacketType::XCVR:
				return receiveRemoteSystemXCVRPacket(stream, payloadLength);

			default:
				ESP_LOGE(_logTag, "failed to receive packet: unsupported type %d", std::to_underlying(type));
				return false;
		}
	}
	
	bool AircraftTransceiver::receiveRemoteSystemTrimPacket(BitStream& stream, const uint8_t payloadLength) {
		auto& ac = Aircraft::getInstance();
		
		if (!validatePayloadChecksumAndLength(
			stream,
			RemoteSystemPacket::typeLengthBits
				+ RemoteSystemTrimPacket::valueLengthBits * 3,
			payloadLength
		))
			return false;
		
		const auto read = [&stream] {
			return
				// Mapping [0; bits] to [-0.5; 0.5]
				static_cast<float>(stream.readUint16(RemoteSystemTrimPacket::valueLengthBits))
				/ static_cast<float>((1 << RemoteSystemTrimPacket::valueLengthBits) - 1)
				- 0.5f;
		};

		ac.settings.trim.aileronsTrim = read();
		ac.settings.trim.elevatorTrim = read();
		ac.settings.trim.rudderTrim = read();
		ac.settings.trim.scheduleWrite();

		return true;
	}
	
	bool AircraftTransceiver::receiveRemoteSystemLightsPacket(BitStream& stream, const uint8_t payloadLength) {
		const auto& ac = Aircraft::getInstance();
		
		if (!validatePayloadChecksumAndLength(
			stream,
			RemoteSystemPacket::typeLengthBits
				+ 4,
			payloadLength
		))
			return false;
		
		ac.lights.setNavigationEnabled(stream.readBool());
		ac.lights.setStrobeEnabled(stream.readBool());
		ac.lights.setLandingEnabled(stream.readBool());
		ac.lights.setCabinEnabled(stream.readBool());
		
		return true;
	}
	
	bool AircraftTransceiver::receiveRemoteSystemBaroPacket(BitStream& stream, const uint8_t payloadLength) {
		auto& ac = Aircraft::getInstance();
		
		if (!validatePayloadChecksumAndLength(
			stream,
			RemoteSystemPacket::typeLengthBits
				+ RemoteSystemBaroPacket::referencePressureLengthBits,
			payloadLength
		))
			return false;
		
		// Reference pressure
		const auto referencePressureDaPa = stream.readUint16(RemoteSystemBaroPacket::referencePressureLengthBits);

		ac.settings.adirs.referencePressurePa = sanitizeValue<uint32_t>(static_cast<uint32_t>(referencePressureDaPa) * 10, 900'00, 1100'00);
		ac.settings.adirs.scheduleWrite();

		ac.adirs.setReferencePressurePa(ac.settings.adirs.referencePressurePa);
		
		return true;
	}
	
	bool AircraftTransceiver::receiveRemoteSystemAutopilotPacket(BitStream& stream, const uint8_t payloadLength) {
		auto& ac = Aircraft::getInstance();

		const auto type = static_cast<RemoteSystemAutopilotPacketType>(stream.readUint8(RemoteSystemAutopilotPacket::typeLengthBits));

		ESP_LOGI(_logTag, "A/P packet, type: %d", std::to_underlying(type));

		const auto validate = [&stream, payloadLength](const size_t adder) {
			return validatePayloadChecksumAndLength(
				stream,
				RemoteSystemPacket::typeLengthBits
					+ RemoteSystemAutopilotPacket::typeLengthBits
					+ adder,
				payloadLength
			);
		};

		const auto readPID = [&stream, &validate, &ac](PIDCoefficients& coefficients) {
			if (!validate(8 * 4 * 3))
				return false;

			coefficients.p = stream.readFloat();
			coefficients.i = stream.readFloat();
			coefficients.d = stream.readFloat();
			ac.settings.autopilot.scheduleWrite();

			ESP_LOGI(_logTag, "PID values: %f, %f, %f", coefficients.p, coefficients.i, coefficients.d);

			return true;
		};

		switch (type) {
			// Generic
			case RemoteSystemAutopilotPacketType::setAutopilotEngaged: {
				if (!validate(1))
					return false;

				ac.fbw.setAutopilotEngaged(stream.readBool());

				break;
			}

			// Lateral
			case RemoteSystemAutopilotPacketType::setLateralMode: {
				if (!validate(RemoteSystemAutopilotPacket::lateralModeLengthBits))
					return false;

				ac.fbw.setLateralMode(static_cast<AutopilotLateralMode>(stream.readUint8(RemoteSystemAutopilotPacket::lateralModeLengthBits)));

				break;
			}
			case RemoteSystemAutopilotPacketType::setHeading: {
				if (!validate(RemoteSystemAutopilotPacket::headingLengthBits))
					return false;

				ac.fbw.setSelectedHeadingDeg(stream.readUint16(RemoteSystemAutopilotPacket::headingLengthBits));

				break;
			}
			case RemoteSystemAutopilotPacketType::setMaxRollAngleRad: {
				if (!validate(8 * 4))
					return false;

				ac.settings.autopilot.maxRollAngleRad = stream.readFloat();
				ac.settings.autopilot.scheduleWrite();

				break;
			}
			case RemoteSystemAutopilotPacketType::setYawToRollPID: {
				if (!readPID(ac.settings.autopilot.PIDs.yawToRoll))
					return false;

				break;
			}
			case RemoteSystemAutopilotPacketType::setRollToAileronsPID: {
				if (!readPID(ac.settings.autopilot.PIDs.rollToAilerons))
					return false;

				break;
			}
			case RemoteSystemAutopilotPacketType::setStabilizedModeRollAngleIncrementRadPerSecond: {
				if (!validate(8 * 4))
					return false;

				ac.settings.autopilot.stabilizedModeRollAngleIncrementRadPerSecond = stream.readFloat();
				ac.settings.autopilot.scheduleWrite();

				break;
			}
			case RemoteSystemAutopilotPacketType::setRollAngleLPFFactorPerSecond: {
				if (!validate(8 * 4))
					return false;

				ac.settings.autopilot.rollAngleLPFFactorPerSecond = stream.readFloat();
				ac.settings.autopilot.scheduleWrite();

				break;
			}
			case RemoteSystemAutopilotPacketType::setMaxAileronsPercent: {
				if (!validate(RemoteSystemAutopilotPacket::percentLengthBits))
					return false;

				ac.settings.autopilot.maxAileronsPercent = stream.readUint8(RemoteSystemAutopilotPacket::percentLengthBits);
				ac.settings.autopilot.scheduleWrite();

				break;
			}

			// Vertical
			case RemoteSystemAutopilotPacketType::setVerticalMode: {
				if (!validate(RemoteSystemAutopilotPacket::verticalModeLengthBits))
					return false;

				ac.fbw.setVerticalMode(static_cast<AutopilotVerticalMode>(stream.readUint8(RemoteSystemAutopilotPacket::verticalModeLengthBits)));

				break;
			}
			case RemoteSystemAutopilotPacketType::setAltitude: {
				if (!validate(RemoteSystemAutopilotPacket::altitudeLengthBits))
					return false;

				ac.fbw.setSelectedAltitudeM(readAltitude(
					stream,
					RemoteSystemAutopilotPacket::altitudeLengthBits,
					RemoteSystemAutopilotPacket::altitudeMinM,
					RemoteSystemAutopilotPacket::altitudeMaxM
				));

				break;
			}
			case RemoteSystemAutopilotPacketType::setMaxPitchAngleRad: {
				if (!validate(8 * 4))
					return false;

				ac.settings.autopilot.maxPitchAngleRad = stream.readFloat();
				ac.settings.autopilot.scheduleWrite();

				break;
			}
			case RemoteSystemAutopilotPacketType::setSpeedToPitchPID: {
				if (!readPID(ac.settings.autopilot.PIDs.speedToPitch))
					return false;

				break;
			}
			case RemoteSystemAutopilotPacketType::setAltitudeToPitchPID: {
				if (!readPID(ac.settings.autopilot.PIDs.altitudeToPitch))
					return false;

				break;
			}
			case RemoteSystemAutopilotPacketType::setPitchToElevatorPID: {
				if (!readPID(ac.settings.autopilot.PIDs.pitchToElevator))
					return false;

				break;
			}
			case RemoteSystemAutopilotPacketType::setStabilizedModePitchAngleIncrementRadPerSecond: {
				if (!validate(8 * 4))
					return false;

				ac.settings.autopilot.stabilizedModePitchAngleIncrementRadPerSecond = stream.readFloat();
				ac.settings.autopilot.scheduleWrite();

				break;
			}
			case RemoteSystemAutopilotPacketType::setPitchAngleLPFFactorPerSecond: {
				if (!validate(8 * 4))
					return false;

				ac.settings.autopilot.pitchAngleLPFFactorPerSecond = stream.readFloat();
				ac.settings.autopilot.scheduleWrite();

				break;
			}
			case RemoteSystemAutopilotPacketType::setMaxElevatorPercent: {
				if (!validate(RemoteSystemAutopilotPacket::percentLengthBits))
					return false;

				ac.settings.autopilot.maxElevatorPercent = stream.readUint8(RemoteSystemAutopilotPacket::percentLengthBits);
				ac.settings.autopilot.scheduleWrite();

				break;
			}

			// Longitudinal
			case RemoteSystemAutopilotPacketType::setAutothrottleEnabled: {
				if (!validate(1))
					return false;

				ac.fbw.setAutothrottleEnabled(stream.readBool());

				break;
			}
			case RemoteSystemAutopilotPacketType::setSpeed: {
				if (!validate(RemoteSystemAutopilotPacket::speedLengthBits))
					return false;

				const auto speedFactor =
					static_cast<float>(stream.readUint8(RemoteSystemAutopilotPacket::speedLengthBits))
					/ static_cast<float>((1 << RemoteSystemAutopilotPacket::speedLengthBits) - 1);

				ac.fbw.setSelectedSpeedMps(static_cast<float>(RemoteSystemAutopilotPacket::speedMaxMPS) * speedFactor);

				break;
			}
			case RemoteSystemAutopilotPacketType::setSpeedToThrottlePID: {
				if (!readPID(ac.settings.autopilot.PIDs.speedToThrottle))
					return false;

				break;
			}
			case RemoteSystemAutopilotPacketType::setMinThrottlePercent: {
				if (!validate(RemoteSystemAutopilotPacket::percentLengthBits))
					return false;

				ac.settings.autopilot.minThrottlePercent = stream.readUint8(RemoteSystemAutopilotPacket::percentLengthBits);
				ac.settings.autopilot.scheduleWrite();

				break;
			}
			case RemoteSystemAutopilotPacketType::setMaxThrottlePercent: {
				if (!validate(RemoteSystemAutopilotPacket::percentLengthBits))
					return false;

				ac.settings.autopilot.maxThrottlePercent = stream.readUint8(RemoteSystemAutopilotPacket::percentLengthBits);
				ac.settings.autopilot.scheduleWrite();

				break;
			}
		}

		return true;
	}

	bool AircraftTransceiver::receiveRemoteSystemCameraPacket(BitStream& stream, const uint8_t payloadLength) {
		auto& ac = Aircraft::getInstance();

		if (!validatePayloadChecksumAndLength(
			stream,
			RemoteSystemPacket::typeLengthBits
				+ RemoteSystemCameraPacket::pitchLengthBits
				+ RemoteSystemCameraPacket::yawLengthBits,
			payloadLength
		))
			return false;

		ac.aircraftData.camera.pitchDeg = stream.readInt16(RemoteSystemCameraPacket::pitchLengthBits);
		ac.aircraftData.camera.yawDeg = stream.readInt16(RemoteSystemCameraPacket::yawLengthBits);

		config::camera::clamp(ac.aircraftData.camera.pitchDeg, ac.aircraftData.camera.yawDeg);
		config::camera::correctPitchPitchForYaw(ac.aircraftData.camera.pitchDeg, ac.aircraftData.camera.yawDeg);

		return true;
	}

	bool AircraftTransceiver::receiveRemoteSystemCalibratePacket(BitStream& stream, const uint8_t payloadLength) {
		auto& ac = Aircraft::getInstance();
		
		if (!validatePayloadChecksumAndLength(
			stream,
			RemoteSystemPacket::typeLengthBits
				+ RemoteSystemCalibratePacket::systemLengthBits,
			payloadLength
		))
			return false;
		
		ac.aircraftData.calibration.system = static_cast<AircraftCalibrationSystem>(stream.readUint8(RemoteSystemCalibratePacket::systemLengthBits));
		ac.aircraftData.calibration.progress = 0;
		ac.aircraftData.calibration.calibrating = true;

//		ESP_LOGI(_logTag, "Received calibrate packet");
		
		return true;
	}
	
	bool AircraftTransceiver::receiveRemoteSystemMotorsPacket(BitStream& stream, const uint8_t payloadLength) {
		auto& ac = Aircraft::getInstance();
		
		ESP_LOGI(_logTag, "Received motor config packet");
		
		if (!validatePayloadChecksumAndLength(
			stream,
			RemoteSystemPacket::typeLengthBits
				+ RemoteSystemMotorConfigurationPacket::typeLengthBits
				+ RemoteSystemMotorConfigurationPacket::minLengthBits
				+ RemoteSystemMotorConfigurationPacket::maxLengthBits
				+ 1,
			payloadLength
		))
			return false;

		auto motorType = static_cast<MotorType>( stream.readUint8(RemoteSystemMotorConfigurationPacket::typeLengthBits));

		auto motor = ac.motors.getByType(motorType);
		auto settings = ac.settings.motors.getByType(motorType);

		if (motor && settings) {
			// Settings
			settings->min = stream.readUint16(RemoteSystemMotorConfigurationPacket::minLengthBits);
			settings->max = stream.readUint16(RemoteSystemMotorConfigurationPacket::maxLengthBits);
			settings->reverse = stream.readBool();
			settings->sanitize();

			// Motor
			motor->setSettings(settings);
			ac.settings.motors.scheduleWrite();

			ESP_LOGI(_logTag, "type: %d, min: %d, max: %d, reverse: %d", static_cast<uint8_t>(motorType), settings->min, settings->max, settings->reverse);
		}

		return true;
	}

	bool AircraftTransceiver::receiveRemoteSystemADIRSPacket(BitStream& stream, const uint8_t payloadLength) {
		auto& ac = Aircraft::getInstance();

		if (!validatePayloadChecksumAndLength(
			stream,
			RemoteSystemPacket::typeLengthBits
				+ RemoteSystemADIRSPacket::magneticDeclinationLengthBits,
			payloadLength
		))
			return false;

		ac.settings.adirs.magneticDeclinationDeg = stream.readInt16(RemoteSystemADIRSPacket::magneticDeclinationLengthBits);
		ac.settings.adirs.scheduleWrite();

		ESP_LOGI(_logTag, "Magnetic declination: %d", ac.settings.adirs.magneticDeclinationDeg);

		return true;
	}

	bool AircraftTransceiver::receiveRemoteSystemXCVRPacket(BitStream& stream, const uint8_t payloadLength) {
		auto& ac = Aircraft::getInstance();

		if (!validatePayloadChecksumAndLength(
			stream,
			RemoteSystemPacket::typeLengthBits
				+ RemoteSystemXCVRPacket::RFFrequencyLengthBits
				+ RemoteSystemXCVRPacket::bandwidthLengthBits
				+ RemoteSystemXCVRPacket::spreadingFactorLengthBits
				+ RemoteSystemXCVRPacket::codingRateLengthBits
				+ RemoteSystemXCVRPacket::syncWordLengthBits
				+ RemoteSystemXCVRPacket::preambleLengthLengthBits

				+ RemoteSystemXCVRPacket::currentLimitMALengthBits
				+ RemoteSystemXCVRPacket::powerDBmLengthBits,
			payloadLength
		))
			return false;

		_tmpCommunicationSettings.frequencyHz = stream.readUint16(RemoteSystemXCVRPacket::RFFrequencyLengthBits) * 1'000'000;
		_tmpCommunicationSettings.bandwidth = static_cast<SX1262::LoRaBandwidth>(stream.readUint8(RemoteSystemXCVRPacket::bandwidthLengthBits));
		_tmpCommunicationSettings.spreadingFactor = stream.readUint8(RemoteSystemXCVRPacket::spreadingFactorLengthBits);
		_tmpCommunicationSettings.codingRate = static_cast<SX1262::LoRaCodingRate>(stream.readUint8(RemoteSystemXCVRPacket::codingRateLengthBits));
		_tmpCommunicationSettings.syncWord = stream.readUint8(RemoteSystemXCVRPacket::syncWordLengthBits);
		_tmpCommunicationSettings.preambleLength = stream.readUint16(RemoteSystemXCVRPacket::preambleLengthLengthBits);

		_tmpCommunicationSettings.currentLimitMA = stream.readInt8(RemoteSystemXCVRPacket::currentLimitMALengthBits);
		_tmpCommunicationSettings.powerDBm = stream.readInt8(RemoteSystemXCVRPacket::powerDBmLengthBits);

		_tmpCommunicationSettings.sanitize();

		ESP_LOGI(_logTag, "received communication settings");
		ESP_LOGI(_logTag, "RFFrequencyHz: %d", _tmpCommunicationSettings.frequencyHz);
		ESP_LOGI(_logTag, "bandwidth: %d", std::to_underlying(_tmpCommunicationSettings.bandwidth));
		ESP_LOGI(_logTag, "spreadingFactor: %d", _tmpCommunicationSettings.spreadingFactor);
		ESP_LOGI(_logTag, "codingRate: %d",std::to_underlying(_tmpCommunicationSettings.codingRate));
		ESP_LOGI(_logTag, "syncWord: %d", _tmpCommunicationSettings.syncWord);
		ESP_LOGI(_logTag, "preambleLength: %d", _tmpCommunicationSettings.preambleLength);

		ESP_LOGI(_logTag, "currentLimitMA: %d", _tmpCommunicationSettings.currentLimitMA);
		ESP_LOGI(_logTag, "powerDBm: %d", _tmpCommunicationSettings.powerDBm);

		enqueueSystemPacket(AircraftSystemPacketType::communicationSettingsACK);

		return true;
	}
	
	// -------------------------------- Transmitting --------------------------------
	
	void AircraftTransceiver::onTransmit(BitStream& stream, const AircraftPacketType packetType) {
		switch (packetType) {
			case AircraftPacketType::STierTelemetry:
				transmitAircraftSTierTelemetryPacket(stream);
				break;
			
			case AircraftPacketType::ATierTelemetry:
				transmitAircraftATierTelemetryPacket(stream);
				break;

			case AircraftPacketType::BTierTelemetry:
				transmitAircraftBTierTelemetryPacket(stream);
				break;
			
			case AircraftPacketType::system:
				transmitAircraftSystemPacket(stream);
				break;
			
			default:
				ESP_LOGE(_logTag, "failed to write packet: unsupported type %d", packetType);
				break;
		}
	}
	
	void AircraftTransceiver::transmitAircraftSTierTelemetryPacket(BitStream& stream) {
		auto& ac = Aircraft::getInstance();

		// -------------------------------- Roll / pitch / yaw --------------------------------

		writeRadians(stream, ac.adirs.getRollRad(), 2.f * std::numbers::pi_v<float>, AircraftSTierTelemetryPacket::rollLengthBits);
		writeRadians(stream, ac.adirs.getPitchRad(), std::numbers::pi_v<float>, AircraftSTierTelemetryPacket::pitchLengthBits);
		writeRadians(stream, ac.adirs.getYawRad(), 2.f * std::numbers::pi_v<float>, AircraftSTierTelemetryPacket::yawLengthBits);

		// -------------------------------- Slip & skid --------------------------------

		// Slip & skid
		const auto slipAndSkidValue = static_cast<uint8_t>(
			static_cast<float>((1 << AircraftSTierTelemetryPacket::slipAndSkidLengthBits) - 1)
			// Mapping from [-1.0; 1.0] to [0.0; 1.0]
			* (ac.adirs.getSlipAndSkidFactor() + 1.f) / 2.f
		);

		stream.writeUint8(slipAndSkidValue, AircraftSTierTelemetryPacket::slipAndSkidLengthBits);

		// -------------------------------- Speed --------------------------------

		const auto speedFactor =
			std::min<float>(ac.adirs.getAirspeedMs(), AircraftSTierTelemetryPacket::speedMaxMPS)
			/ static_cast<float>(AircraftSTierTelemetryPacket::speedMaxMPS);

		const auto speedMapped = static_cast<float>((1 << AircraftSTierTelemetryPacket::speedLengthBits) - 1) * speedFactor;

		stream.writeUint8(static_cast<uint8_t>(speedMapped), AircraftSTierTelemetryPacket::speedLengthBits);

		// -------------------------------- Altitude --------------------------------

		writeAltitude(
			stream,
			ac.adirs.getCoordinates().getAltitude(),
			AircraftSTierTelemetryPacket::altitudeLengthBits,
			AircraftSTierTelemetryPacket::altitudeMinM,
			AircraftSTierTelemetryPacket::altitudeMaxM
		);

		// -------------------------------- Autopilot target roll / pitch  --------------------------------

		writeRadians(stream, ac.fbw.getTargetRollRad(), AircraftSTierTelemetryPacket::autopilotTargetRollRangeRad, AircraftSTierTelemetryPacket::autopilotTargetRollLengthBits);
		writeRadians(stream, ac.fbw.getTargetPitchRad(), AircraftSTierTelemetryPacket::autopilotTargetPitchRangeRad, AircraftSTierTelemetryPacket::autopilotTargetPitchLengthBits);
	}
	
	void AircraftTransceiver::transmitAircraftATierTelemetryPacket(BitStream& stream) {
		auto& ac = Aircraft::getInstance();

		// -------------------------------- Throttle --------------------------------

		stream.writeUint8(
			static_cast<uint32_t>(ac.motors.getByType(MotorType::throttle)->getRawPower())
				* ((1 << AircraftATierTelemetryPacket::throttleLengthBits) - 1)
				/ MotorSettings::powerMax,
			AircraftATierTelemetryPacket::throttleLengthBits
		);

		// -------------------------------- Latitude & longitude --------------------------------

		const auto& coordinates = ac.adirs.getCoordinates();

		// Lat
		const auto latRad = coordinates.getLatitude();
		// Mapping from [-90; 90] to [0; 180] and then to [0; 1]
		const auto latFactor = (latRad + std::numbers::pi_v<float> / 2.f) / std::numbers::pi_v<float>;
		const auto latValue = static_cast<uint32_t>(static_cast<float>((1 << AircraftATierTelemetryPacket::latLengthBits) - 1) * latFactor);
		
		stream.writeUint32(latValue, AircraftATierTelemetryPacket::latLengthBits);
		
		// Lon
		const auto lonRad = coordinates.getLongitude();
		// Mapping from [0; 360] to [0; 1]
		const auto lonFactor = lonRad / (2 * std::numbers::pi_v<float>);
		const auto lonValue = static_cast<uint32_t>(static_cast<float>((1 << AircraftATierTelemetryPacket::lonLengthBits) - 1) * lonFactor);
		
		stream.writeUint32(lonValue, AircraftATierTelemetryPacket::lonLengthBits);
	}

	void AircraftTransceiver::transmitAircraftBTierTelemetryPacket(BitStream& stream) {
		auto& ac = Aircraft::getInstance();

		// -------------------------------- Autopilot --------------------------------

		// Modes
		stream.writeUint8(std::to_underlying(ac.fbw.getLateralMode()), AircraftBTierTelemetryPacket::autopilotLateralModeLengthBits);
		stream.writeUint8(std::to_underlying(ac.fbw.getVerticalMode()), AircraftBTierTelemetryPacket::autopilotVerticalModeLengthBits);

		// Altitude for ALT/ALTS/VNAV modes
		writeAltitude(
			stream,
			ac.fbw.getVerticalMode() == AutopilotVerticalMode::alt
				? ac.fbw.getHoldAltitudeM()
				: ac.fbw.getSelectedAltitudeM(),
			AircraftBTierTelemetryPacket::autopilotAltitudeLengthBits,
			AircraftBTierTelemetryPacket::autopilotAltitudeMinM,
			AircraftBTierTelemetryPacket::autopilotAltitudeMaxM
		);

		// Autothrottle
		stream.writeBool(ac.fbw.isAutothrottleEnabled());

		// Autopilot
		stream.writeBool(ac.fbw.isAutopilotEngaged());

		// -------------------------------- Lights --------------------------------

		stream.writeBool(ac.settings.lights.nav);
		stream.writeBool(ac.settings.lights.strobe);
		stream.writeBool(ac.settings.lights.landing);
		stream.writeBool(ac.settings.lights.cabin);

		// -------------------------------- Battery --------------------------------

		// Decavolts
		stream.writeUint16(ac.battery.getVoltageMV() / 100, AircraftBTierTelemetryPacket::batteryLengthBits);
	}

	void AircraftTransceiver::transmitAircraftSystemPacket(BitStream& stream) {
		stream.writeUint8(std::to_underlying(getEnqueuedSystemPacketType()), AircraftSystemPacket::typeLengthBits);

		switch (getEnqueuedSystemPacketType()) {
			case AircraftSystemPacketType::calibration:
				transmitAircraftSystemCalibrationPacket(stream);
				break;
			case AircraftSystemPacketType::communicationSettingsACK:
				transmitAircraftSystemCommunicationSettingsACKPacket(stream);
				break;
			default:
				break;
		}
	}

	void AircraftTransceiver::transmitAircraftSystemCalibrationPacket(BitStream& stream) {
		const auto& ac = Aircraft::getInstance();
	
		stream.writeUint8(std::to_underlying(ac.aircraftData.calibration.system), AircraftSystemCalibrationPacket::systemLengthBits);
		stream.writeUint8(static_cast<uint16_t>(ac.aircraftData.calibration.progress) * ((1 << AircraftSystemCalibrationPacket::progressLengthBits) - 1) / 0xFF, AircraftSystemCalibrationPacket::progressLengthBits);
	}

	void AircraftTransceiver::transmitAircraftSystemCommunicationSettingsACKPacket(BitStream& stream) {
		const auto& ac = Aircraft::getInstance();

		_communicationSettingsACKTime = -1;
	}
}