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

	void AircraftTransceiver::onTick() {
		if (_receiveMode) {
			if (receive(1'000'000)) {
				_receiveMode = false;

				_transmitTimeUs = esp_timer_get_time() + 8'000;
			}
		}
		else {
			if (esp_timer_get_time() >= _transmitTimeUs) {
				transmit(1'000'000);

				_receiveMode = true;
			}
			else {
				taskYIELD();
			}
		}
	}

	void AircraftTransceiver::onCommunicationSettingsSyncCheckScheduled() {
		setCommunicationSettings(_receivedCommunicationSettings);
	}

	void AircraftTransceiver::onCommunicationSettingsSyncCheckCompleted() {
		auto& ac = Aircraft::getInstance();
		ac.settings.transceiver.communication = _receivedCommunicationSettings;
		ac.settings.transceiver.writeLater();
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
			case RemotePacketType::controls: {
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

				ac.remoteData.controls.setThrottle(readMotor());
				ac.remoteData.controls.setAilerons(readMotor());
				ac.remoteData.controls.setElevator(readMotor());
				ac.remoteData.controls.setRudder(readMotor());
				ac.remoteData.controls.setFlaps(readMotor());

				return true;
			}
			case RemotePacketType::system: {
				return receiveRemoteSystemPacket(stream, payloadLength);
			}
			default: {
				ESP_LOGE(_logTag, "failed to receive packet: unsupported type %d", std::to_underlying(packetType));

				return false;
			}
		}
	}

	bool AircraftTransceiver::receiveRemoteSystemPacket(BitStream& stream, const uint8_t payloadLength) {
		const auto type = static_cast<RemoteSystemPacketType>(stream.readUint8(RemoteSystemPacket::typeLengthBits));

		const auto readPID = [&stream, payloadLength](PIDCoefficients& coefficients) {
			if (!validatePayloadChecksumAndLength(
				stream,
				RemoteSystemPacket::typeLengthBits
					+ 8 * 4 * 3,
				payloadLength
			))
				return false;

			coefficients.p = stream.readFloat();
			coefficients.i = stream.readFloat();
			coefficients.d = stream.readFloat();
			Aircraft::getInstance().settings.autopilot.configuration.writeLater();

			ESP_LOGI(_logTag, "PID values: %f, %f, %f", coefficients.p, coefficients.i, coefficients.d);

			return true;
		};

		switch (type) {
			case RemoteSystemPacketType::trim: {
				if (!validatePayloadChecksumAndLength(
					stream,
					RemoteSystemPacket::typeLengthBits
						+ RemoteSystemPacket::trimValueLengthBits * 3,
					payloadLength
				))
					return false;

				auto& ac = Aircraft::getInstance();

				const auto read = [&stream] {
					return
						// Mapping [0; bits] to [-0.5; 0.5]
						static_cast<float>(stream.readUint16(RemoteSystemPacket::trimValueLengthBits))
						/ static_cast<float>((1 << RemoteSystemPacket::trimValueLengthBits) - 1)
						- 0.5f;
				};

				ac.settings.trim.setAileronsTrim(read());
				ac.settings.trim.setElevatorTrim(read());
				ac.settings.trim.setRudderTrim(read());
				ac.settings.trim.writeLater();

				return true;
			}
			case RemoteSystemPacketType::lights: {
				if (!validatePayloadChecksumAndLength(
					stream,
					RemoteSystemPacket::typeLengthBits
						+ 4,
					payloadLength
				))
					return false;

				auto& ac = Aircraft::getInstance();

				ac.lights.setNavigationEnabled(stream.readBool());
				ac.lights.setStrobeEnabled(stream.readBool());
				ac.lights.setLandingEnabled(stream.readBool());
				ac.lights.setCabinEnabled(stream.readBool());

				return true;
			}
			case RemoteSystemPacketType::referencePressure: {
				if (!validatePayloadChecksumAndLength(
					stream,
					RemoteSystemPacket::typeLengthBits
						+ RemoteSystemPacket::referencePressureLengthBits,
					payloadLength
				))
					return false;

				auto& ac = Aircraft::getInstance();

				// Reference pressure
				const auto referencePressureDaPa = stream.readUint16(RemoteSystemPacket::referencePressureLengthBits);

				ac.settings.adirs.setReferencePressurePa(sanitizeValue<uint32_t>(static_cast<uint32_t>(referencePressureDaPa) * 10, 900'00, 1100'00));
				ac.settings.adirs.writeLater();

				return true;
			}
			case RemoteSystemPacketType::calibrate: {
				if (!validatePayloadChecksumAndLength(
					stream,
					RemoteSystemPacket::typeLengthBits
						+ RemoteSystemPacket::calibrateSystemLengthBits,
					payloadLength
				))
					return false;

				auto& ac = Aircraft::getInstance();

				ac.aircraftData.calibration.setSystem(static_cast<AircraftCalibrationSystem>(stream.readUint8(RemoteSystemPacket::calibrateSystemLengthBits)));
				ac.aircraftData.calibration.setProgress(0);
				ac.aircraftData.calibration.setCalibrating(true);

				//		ESP_LOGI(_logTag, "Received calibrate packet");

				return true;
			}
			case RemoteSystemPacketType::camera: {
				if (!validatePayloadChecksumAndLength(
					stream,
					RemoteSystemPacket::typeLengthBits
						+ RemoteSystemPacket::cameraPitchLengthBits
						+ RemoteSystemPacket::cameraYawLengthBits,
					payloadLength
				))
					return false;

				auto& ac = Aircraft::getInstance();

				auto pitchDeg = stream.readInt16(RemoteSystemPacket::cameraPitchLengthBits);
				auto yawDeg = stream.readInt16(RemoteSystemPacket::cameraYawLengthBits);

				config::camera::clamp(pitchDeg, yawDeg);
				config::camera::correctPitchPitchForYaw(pitchDeg, yawDeg);

				ac.aircraftData.camera.setPitchDeg(pitchDeg);
				ac.aircraftData.camera.setYawDeg(yawDeg);

				return true;
			}
			case RemoteSystemPacketType::motors: {
				ESP_LOGI(_logTag, "Received motor config packet");

				if (!validatePayloadChecksumAndLength(
					stream,
					RemoteSystemPacket::typeLengthBits
						+ RemoteSystemPacket::motorConfigurationTypeLengthBits
						+ RemoteSystemPacket::motorConfigurationMinLengthBits
						+ RemoteSystemPacket::motorConfigurationMaxLengthBits
						+ 1,
					payloadLength
				))
					return false;

				auto& ac = Aircraft::getInstance();

				auto motorType = static_cast<MotorType>(stream.readUint8(RemoteSystemPacket::motorConfigurationTypeLengthBits));

				auto motor = ac.motors.getByType(motorType);
				auto settings = ac.settings.motors.getByType(motorType);

				if (motor && settings) {
					// Settings
					settings->min = stream.readUint16(RemoteSystemPacket::motorConfigurationMinLengthBits);
					settings->max = stream.readUint16(RemoteSystemPacket::motorConfigurationMaxLengthBits);
					settings->reverse = stream.readBool();
					settings->sanitize();

					// Motor
					motor->setSettings(settings);
					ac.settings.motors.writeLater();

					ESP_LOGI(_logTag, "type: %d, min: %d, max: %d, reverse: %d", static_cast<uint8_t>(motorType), settings->min, settings->max, settings->reverse);
				}

				return true;
			}
			case RemoteSystemPacketType::magneticDeclination: {
				if (!validatePayloadChecksumAndLength(
					stream,
					RemoteSystemPacket::typeLengthBits
						+ RemoteSystemPacket::magneticDeclinationLengthBits,
					payloadLength
				))
					return false;

				auto& ac = Aircraft::getInstance();

				ac.settings.adirs.setMagneticDeclinationDeg(stream.readInt16(RemoteSystemPacket::magneticDeclinationLengthBits));
				ac.settings.adirs.writeLater();

				ESP_LOGI(_logTag, "Magnetic declination: %d", ac.settings.adirs.getMagneticDeclinationDeg());

				return true;
			}
			case RemoteSystemPacketType::communicationSettings: {
				if (!validatePayloadChecksumAndLength(
					stream,
					RemoteSystemPacket::typeLengthBits
						+ RemoteSystemCommunicationSettingsPacket::RFFrequencyLengthBits
						+ RemoteSystemCommunicationSettingsPacket::bandwidthLengthBits
						+ RemoteSystemCommunicationSettingsPacket::spreadingFactorLengthBits
						+ RemoteSystemCommunicationSettingsPacket::codingRateLengthBits
						+ RemoteSystemCommunicationSettingsPacket::syncWordLengthBits
						+ RemoteSystemCommunicationSettingsPacket::preambleLengthLengthBits

						+ RemoteSystemCommunicationSettingsPacket::currentLimitMALengthBits
						+ RemoteSystemCommunicationSettingsPacket::powerDBmLengthBits,
					payloadLength
				))
					return false;

				_receivedCommunicationSettings.frequencyHz = stream.readUint16(RemoteSystemCommunicationSettingsPacket::RFFrequencyLengthBits) * 1'000'000;
				_receivedCommunicationSettings.bandwidth = static_cast<SX1262::LoRaBandwidth>(stream.readUint8(RemoteSystemCommunicationSettingsPacket::bandwidthLengthBits));
				_receivedCommunicationSettings.spreadingFactor = stream.readUint8(RemoteSystemCommunicationSettingsPacket::spreadingFactorLengthBits);
				_receivedCommunicationSettings.codingRate = static_cast<SX1262::LoRaCodingRate>(stream.readUint8(RemoteSystemCommunicationSettingsPacket::codingRateLengthBits));
				_receivedCommunicationSettings.syncWord = stream.readUint8(RemoteSystemCommunicationSettingsPacket::syncWordLengthBits);
				_receivedCommunicationSettings.preambleLength = stream.readUint16(RemoteSystemCommunicationSettingsPacket::preambleLengthLengthBits);

				_receivedCommunicationSettings.currentLimitMA = stream.readInt8(RemoteSystemCommunicationSettingsPacket::currentLimitMALengthBits);
				_receivedCommunicationSettings.powerDBm = stream.readInt8(RemoteSystemCommunicationSettingsPacket::powerDBmLengthBits);

				_receivedCommunicationSettings.sanitize();

				ESP_LOGI(_logTag, "received communication settings");
				ESP_LOGI(_logTag, "RFFrequencyHz: %d", _receivedCommunicationSettings.frequencyHz);
				ESP_LOGI(_logTag, "bandwidth: %d", std::to_underlying(_receivedCommunicationSettings.bandwidth));
				ESP_LOGI(_logTag, "spreadingFactor: %d", _receivedCommunicationSettings.spreadingFactor);
				ESP_LOGI(_logTag, "codingRate: %d",std::to_underlying(_receivedCommunicationSettings.codingRate));
				ESP_LOGI(_logTag, "syncWord: %d", _receivedCommunicationSettings.syncWord);
				ESP_LOGI(_logTag, "preambleLength: %d", _receivedCommunicationSettings.preambleLength);

				ESP_LOGI(_logTag, "currentLimitMA: %d", _receivedCommunicationSettings.currentLimitMA);
				ESP_LOGI(_logTag, "powerDBm: %d", _receivedCommunicationSettings.powerDBm);

				enqueueSystemPacket(AircraftSystemPacketType::communicationSettingsACK);

				return true;
			}
			case RemoteSystemPacketType::homeCoordinates: {
				if (!validatePayloadChecksumAndLength(
					stream,
					RemoteSystemPacket::typeLengthBits
						+ RemoteSystemPacket::homeCoordinatesLatitudeLengthBits
						+ RemoteSystemPacket::homeCoordinatesLongitudeLengthBits,
					payloadLength
				))
					return false;

				auto& ac = Aircraft::getInstance();

				ac.adirs.setHomeCoordinates(
					readLatitude(stream, RemoteSystemPacket::homeCoordinatesLatitudeLengthBits),
					readLongitude(stream, RemoteSystemPacket::homeCoordinatesLongitudeLengthBits),
					0
				);

				return true;
			}

			// -------------------------------- Autopilot --------------------------------

			// Generic
			case RemoteSystemPacketType::autopilotEnabled: {
				if (!validatePayloadChecksumAndLength(
					stream,
					RemoteSystemPacket::typeLengthBits
						+ 1,
					payloadLength
				))
					return false;

				auto& ac = Aircraft::getInstance();

				ac.fbw.setAutopilotEngaged(stream.readBool());

				return true;
			}

			// Lateral
			case RemoteSystemPacketType::autopilotLateralMode: {
				if (!validatePayloadChecksumAndLength(
					stream,
					RemoteSystemPacket::typeLengthBits
						+ RemoteSystemPacket::autopilotLateralModeLengthBits,
					payloadLength
				))
					return false;

				auto& ac = Aircraft::getInstance();

				ac.fbw.setLateralMode(static_cast<AutopilotLateralMode>(stream.readUint8(RemoteSystemPacket::autopilotLateralModeLengthBits)));

				return true;
			}
			case RemoteSystemPacketType::autopilotHeading: {
				if (!validatePayloadChecksumAndLength(
					stream,
					RemoteSystemPacket::typeLengthBits
						+ RemoteSystemPacket::autopilotHeadingLengthBits,
					payloadLength
				))
					return false;

				auto& ac = Aircraft::getInstance();

				ac.settings.autopilot.selection.setSelectedHeadingDeg(stream.readUint16(RemoteSystemPacket::autopilotHeadingLengthBits));
				ac.settings.autopilot.selection.writeLater();

				return true;
			}
			case RemoteSystemPacketType::autopilotMaxRollAngleRad: {
				if (!validatePayloadChecksumAndLength(
					stream,
					RemoteSystemPacket::typeLengthBits
						+ 8 * 4,
					payloadLength
				))
					return false;

				auto& ac = Aircraft::getInstance();

				ac.settings.autopilot.configuration.maxRollAngleRad = stream.readFloat();
				ac.settings.autopilot.configuration.writeLater();

				return true;
			}
			case RemoteSystemPacketType::autopilotYawToRollPID: {
				auto& ac = Aircraft::getInstance();

				if (!readPID(ac.settings.autopilot.configuration.PIDs.yawToRoll))
					return false;

				return true;
			}
			case RemoteSystemPacketType::autopilotRollToAileronsPID: {
				auto& ac = Aircraft::getInstance();

				if (!readPID(ac.settings.autopilot.configuration.PIDs.rollToAilerons))
					return false;

				return true;
			}
			case RemoteSystemPacketType::autopilotStabilizedModeRollAngleIncrementRadPerSecond: {
				if (!validatePayloadChecksumAndLength(
					stream,
					RemoteSystemPacket::typeLengthBits
						+ 8 * 4,
					payloadLength
				))
					return false;

				auto& ac = Aircraft::getInstance();

				ac.settings.autopilot.configuration.stabilizedModeRollAngleIncrementRadPerSecond = stream.readFloat();
				ac.settings.autopilot.configuration.writeLater();

				return true;
			}
			case RemoteSystemPacketType::autopilotRollAngleLPFFactorPerSecond: {
				if (!validatePayloadChecksumAndLength(
					stream,
					RemoteSystemPacket::typeLengthBits
						+ 8 * 4,
					payloadLength
				))
					return false;

				auto& ac = Aircraft::getInstance();

				ac.settings.autopilot.configuration.rollAngleLPFFactorPerSecond = stream.readFloat();
				ac.settings.autopilot.configuration.writeLater();

				return true;
			}
			case RemoteSystemPacketType::autopilotMaxAileronsPercent: {
				if (!validatePayloadChecksumAndLength(
					stream,
					RemoteSystemPacket::typeLengthBits
						+ RemoteSystemPacket::autopilotPercentLengthBits,
					payloadLength
				))
					return false;

				auto& ac = Aircraft::getInstance();

				ac.settings.autopilot.configuration.maxAileronsPercent = stream.readUint8(RemoteSystemPacket::autopilotPercentLengthBits);
				ac.settings.autopilot.configuration.writeLater();

				return true;
			}

			// Vertical
			case RemoteSystemPacketType::autopilotVerticalMode: {
				if (!validatePayloadChecksumAndLength(
					stream,
					RemoteSystemPacket::typeLengthBits
						+ RemoteSystemPacket::autopilotVerticalModeLengthBits,
					payloadLength
				))
					return false;

				auto& ac = Aircraft::getInstance();

				ac.fbw.setVerticalMode(static_cast<AutopilotVerticalMode>(stream.readUint8(RemoteSystemPacket::autopilotVerticalModeLengthBits)));

				return true;
			}
			case RemoteSystemPacketType::autopilotAltitude: {
				if (!validatePayloadChecksumAndLength(
					stream,
					RemoteSystemPacket::typeLengthBits
						+ RemoteSystemPacket::autopilotAltitudeLengthBits,
					payloadLength
				))
					return false;

				auto& ac = Aircraft::getInstance();

				ac.settings.autopilot.selection.setSelectedAltitudeM(readAltitude(
					stream,
					RemoteSystemPacket::autopilotAltitudeLengthBits,
					RemoteSystemPacket::autopilotAltitudeMinM,
					RemoteSystemPacket::autopilotAltitudeMaxM
				));

				ac.settings.autopilot.selection.writeLater();

				return true;
			}
			case RemoteSystemPacketType::autopilotMaxPitchAngleRad: {
				if (!validatePayloadChecksumAndLength(
					stream,
					RemoteSystemPacket::typeLengthBits
						+ 8 * 4,
					payloadLength
				))
					return false;

				auto& ac = Aircraft::getInstance();

				ac.settings.autopilot.configuration.maxPitchAngleRad = stream.readFloat();
				ac.settings.autopilot.configuration.writeLater();

				return true;
			}
			case RemoteSystemPacketType::autopilotSpeedToPitchPID: {
				auto& ac = Aircraft::getInstance();

				if (!readPID(ac.settings.autopilot.configuration.PIDs.speedToPitch))
					return false;

				return true;
			}
			case RemoteSystemPacketType::autopilotAltitudeToPitchPID: {
				auto& ac = Aircraft::getInstance();

				if (!readPID(ac.settings.autopilot.configuration.PIDs.altitudeToPitch))
					return false;

				return true;
			}
			case RemoteSystemPacketType::autopilotPitchToElevatorPID: {
				auto& ac = Aircraft::getInstance();

				if (!readPID(ac.settings.autopilot.configuration.PIDs.pitchToElevator))
					return false;

				return true;
			}
			case RemoteSystemPacketType::autopilotStabilizedModePitchAngleIncrementRadPerSecond: {
				if (!validatePayloadChecksumAndLength(
					stream,
					RemoteSystemPacket::typeLengthBits
						+ 8 * 4,
					payloadLength
				))
					return false;

				auto& ac = Aircraft::getInstance();

				ac.settings.autopilot.configuration.stabilizedModePitchAngleIncrementRadPerSecond = stream.readFloat();
				ac.settings.autopilot.configuration.writeLater();

				return true;
			}
			case RemoteSystemPacketType::autopilotPitchAngleLPFFactorPerSecond: {
				if (!validatePayloadChecksumAndLength(
					stream,
					RemoteSystemPacket::typeLengthBits
						+ 8 * 4,
					payloadLength
				))
					return false;

				auto& ac = Aircraft::getInstance();

				ac.settings.autopilot.configuration.pitchAngleLPFFactorPerSecond = stream.readFloat();
				ac.settings.autopilot.configuration.writeLater();

				return true;
			}
			case RemoteSystemPacketType::autopilotMaxElevatorPercent: {
				if (!validatePayloadChecksumAndLength(
					stream,
					RemoteSystemPacket::typeLengthBits
						+ RemoteSystemPacket::autopilotPercentLengthBits,
					payloadLength
				))
					return false;

				auto& ac = Aircraft::getInstance();

				ac.settings.autopilot.configuration.maxElevatorPercent = stream.readUint8(RemoteSystemPacket::autopilotPercentLengthBits);
				ac.settings.autopilot.configuration.writeLater();

				return true;
			}

			// Longitudinal
			case RemoteSystemPacketType::autopilotAutothrottleEnabled: {
				if (!validatePayloadChecksumAndLength(
					stream,
					RemoteSystemPacket::typeLengthBits
						+ 1,
					payloadLength
				))
					return false;

				auto& ac = Aircraft::getInstance();

				ac.fbw.setAutothrottleEnabled(stream.readBool());

				return true;
			}
			case RemoteSystemPacketType::autopilotSpeed: {
				if (!validatePayloadChecksumAndLength(
					stream,
					RemoteSystemPacket::typeLengthBits
						+ RemoteSystemPacket::autopilotSpeedLengthBits,
					payloadLength
				))
					return false;

				auto& ac = Aircraft::getInstance();

				const auto speedFactor =
					static_cast<float>(stream.readUint8(RemoteSystemPacket::autopilotSpeedLengthBits))
					/ static_cast<float>((1 << RemoteSystemPacket::autopilotSpeedLengthBits) - 1);

				ac.settings.autopilot.selection.setSelectedSpeedMPS(static_cast<float>(RemoteSystemPacket::autopilotSpeedMaxMPS) * speedFactor);
				ac.settings.autopilot.selection.writeLater();

				return true;
			}
			case RemoteSystemPacketType::autopilotSpeedToThrottlePID: {
				auto& ac = Aircraft::getInstance();

				if (!readPID(ac.settings.autopilot.configuration.PIDs.speedToThrottle))
					return false;

				return true;
			}
			case RemoteSystemPacketType::autopilotMinThrottlePercent: {
				if (!validatePayloadChecksumAndLength(
					stream,
					RemoteSystemPacket::typeLengthBits
						+ RemoteSystemPacket::autopilotPercentLengthBits,
					payloadLength
				))
					return false;

				auto& ac = Aircraft::getInstance();

				ac.settings.autopilot.configuration.minThrottlePercent = stream.readUint8(RemoteSystemPacket::autopilotPercentLengthBits);
				ac.settings.autopilot.configuration.writeLater();

				return true;
			}
			case RemoteSystemPacketType::autopilotMaxThrottlePercent: {
				if (!validatePayloadChecksumAndLength(
					stream,
					RemoteSystemPacket::typeLengthBits
						+ RemoteSystemPacket::autopilotPercentLengthBits,
					payloadLength
				))
					return false;

				auto& ac = Aircraft::getInstance();

				ac.settings.autopilot.configuration.maxThrottlePercent = stream.readUint8(RemoteSystemPacket::autopilotPercentLengthBits);
				ac.settings.autopilot.configuration.writeLater();

				return true;
			}

			default: {
				ESP_LOGE(_logTag, "failed to receive packet: unsupported type %d", std::to_underlying(type));

				return false;
			}
		}
	}
	
	// -------------------------------- Transmitting --------------------------------
	
	void AircraftTransceiver::onTransmit(BitStream& stream, const AircraftPacketType packetType) {
		switch (packetType) {
			case AircraftPacketType::STierTelemetry: {
				const auto& ac = Aircraft::getInstance();

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
					std::min<float>(ac.adirs.getAirspeedMPS(), AircraftSTierTelemetryPacket::speedMaxMPS)
					/ static_cast<float>(AircraftSTierTelemetryPacket::speedMaxMPS);

				const auto speedMapped = static_cast<float>((1 << AircraftSTierTelemetryPacket::speedLengthBits) - 1) * speedFactor;

				stream.writeUint8(static_cast<uint8_t>(speedMapped), AircraftSTierTelemetryPacket::speedLengthBits);

				// -------------------------------- Altitude --------------------------------

				writeAltitude(
					stream,
					ac.adirs.getAltitude(),
					AircraftSTierTelemetryPacket::altitudeLengthBits,
					AircraftSTierTelemetryPacket::altitudeMinM,
					AircraftSTierTelemetryPacket::altitudeMaxM
				);

				// -------------------------------- Autopilot target roll / pitch  --------------------------------

				writeRadians(stream, ac.fbw.getTargetRollRad(), AircraftSTierTelemetryPacket::autopilotTargetRollRangeRad, AircraftSTierTelemetryPacket::autopilotTargetRollLengthBits);
				writeRadians(stream, ac.fbw.getTargetPitchRad(), AircraftSTierTelemetryPacket::autopilotTargetPitchRangeRad, AircraftSTierTelemetryPacket::autopilotTargetPitchLengthBits);

				break;
			}
			case AircraftPacketType::ATierTelemetry: {
				const auto& ac = Aircraft::getInstance();

				// -------------------------------- Throttle --------------------------------

				stream.writeUint8(
					static_cast<uint32_t>(
						ac.fbw.getThrottleFactor()
						* static_cast<float>((1 << AircraftATierTelemetryPacket::throttleLengthBits) - 1)
					),
					AircraftATierTelemetryPacket::throttleLengthBits
				);

				// -------------------------------- Latitude & longitude --------------------------------

				writeLatitude(stream, ac.adirs.getLatitude(), AircraftATierTelemetryPacket::latitudeLengthBits);
				writeLongitude(stream, ac.adirs.getLongitude(), AircraftATierTelemetryPacket::longitudeLengthBits);

				break;
			}
			case AircraftPacketType::BTierTelemetry: {
				const auto& ac = Aircraft::getInstance();

				// -------------------------------- Autopilot --------------------------------

				// Modes
				stream.writeUint8(std::to_underlying(ac.fbw.getLateralMode()), AircraftBTierTelemetryPacket::autopilotLateralModeLengthBits);
				stream.writeUint8(std::to_underlying(ac.fbw.getVerticalMode()), AircraftBTierTelemetryPacket::autopilotVerticalModeLengthBits);

				// Altitude for ALT/ALTS/VNAV modes
				writeAltitude(
					stream,
					ac.fbw.getVerticalMode() == AutopilotVerticalMode::alt
						? ac.fbw.getHoldAltitudeM()
						: ac.settings.autopilot.selection.getSelectedAltitudeM(),
					AircraftBTierTelemetryPacket::autopilotAltitudeLengthBits,
					AircraftBTierTelemetryPacket::autopilotAltitudeMinM,
					AircraftBTierTelemetryPacket::autopilotAltitudeMaxM
				);

				// Autothrottle
				stream.writeBool(ac.fbw.isAutothrottleEnabled());

				// Autopilot
				stream.writeBool(ac.fbw.isAutopilotEngaged());

				// -------------------------------- Lights --------------------------------

				stream.writeBool(ac.settings.lights.getNav());
				stream.writeBool(ac.settings.lights.getStrobe());
				stream.writeBool(ac.settings.lights.getLanding());
				stream.writeBool(ac.settings.lights.getCabin());

				// -------------------------------- Battery --------------------------------

				// Decavolts
				stream.writeUint16(ac.battery.getVoltageMV() / 100, AircraftBTierTelemetryPacket::batteryLengthBits);

				break;
			}
			case AircraftPacketType::system:
				transmitAircraftSystemPacket(stream);
				break;
			
			default:
				ESP_LOGE(_logTag, "failed to write packet: unsupported type %d", packetType);
				break;
		}
	}

	void AircraftTransceiver::transmitAircraftSystemPacket(BitStream& stream) {
		stream.writeUint8(std::to_underlying(getEnqueuedSystemPacketType()), AircraftSystemPacket::typeLengthBits);

		switch (getEnqueuedSystemPacketType()) {
			case AircraftSystemPacketType::calibration: {
				const auto& ac = Aircraft::getInstance();

				stream.writeUint8(std::to_underlying(ac.aircraftData.calibration.getSystem()), AircraftSystemCalibrationPacket::systemLengthBits);
				stream.writeUint8(static_cast<uint16_t>(ac.aircraftData.calibration.getProgress()) * ((1 << AircraftSystemCalibrationPacket::progressLengthBits) - 1) / 0xFF, AircraftSystemCalibrationPacket::progressLengthBits);

				break;
			}
			case AircraftSystemPacketType::communicationSettingsACK: {
				requestCommunicationSettingsSyncCheck();

				break;
			}
			default:
				break;
		}
	}
}