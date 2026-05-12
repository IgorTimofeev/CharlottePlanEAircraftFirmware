#pragma once

#include <cstdint>
#include <numbers>

namespace pizda {
	#pragma pack(push, 1)

	class Packet {
		public:
			constexpr static uint8_t checksumLengthBytes = 1;
	};

	// -------------------------------- Remote --------------------------------

	enum class RemotePacketType : uint8_t {
		controls,
		auxiliary,

		maxValue = auxiliary
	};

	class RemotePacket {
		public:
			constexpr static uint8_t typeLengthBits = 1;
	};

	class RemoteControlsPacket {
		public:
			constexpr static uint8_t motorLengthBits = 12;
	};

	enum class RemoteAuxiliaryPacketType : uint8_t {
		trim,
		lights,
		baro,
		autopilot,
		camera,
		motors,
		calibrate,
		ADIRS,
		XCVR,

		maxValue = XCVR
	};

	class RemoteAuxiliaryPacket {
		public:
			constexpr static uint8_t typeLengthBits = 4;
	};

	class RemoteAuxiliaryCameraPacket {
		public:
			constexpr static uint8_t pitchLengthBits = 12;
			constexpr static uint8_t yawLengthBits = pitchLengthBits;
	};

	class RemoteAuxiliaryMotorConfigurationPacket {
		public:
			constexpr static uint8_t minLengthBits = 12;
			constexpr static uint8_t maxLengthBits = minLengthBits;
	};

	class RemoteAuxiliaryCalibratePacket {
		public:
			constexpr static uint8_t systemLengthBits = 2;
	};

	class RemoteAuxiliaryTrimPacket {
		public:
			constexpr static uint8_t valueLengthBits = RemoteControlsPacket::motorLengthBits;
	};

	enum class RemoteAuxiliaryAutopilotPacketType : uint8_t {
		// Generic
		setAutopilotEngaged,

		// Lateral
		setLateralMode,
		setHeading,
		setMaxRollAngleRad,
		setYawToRollPID,
		setRollToAileronsPID,
		setStabilizedModeRollAngleIncrementRadPerSecond,
		setRollAngleLPFFactorPerSecond,
		setMaxAileronsPercent,

		// Vertical
		setVerticalMode,
		setAltitude,
		setMaxPitchAngleRad,
		setSpeedToPitchPID,
		setAltitudeToPitchPID,
		setPitchToElevatorPID,
		setStabilizedModePitchAngleIncrementRadPerSecond,
		setPitchAngleLPFFactorPerSecond,
		setMaxElevatorPercent,

		// Longitudinal
		setAutothrottleEnabled,
		setSpeed,
		setSpeedToThrottlePID,
		setMinThrottlePercent,
		setMaxThrottlePercent,
	};

	class RemoteAuxiliaryAutopilotPacket {
		public:
			constexpr static uint8_t typeLengthBits = 5;

			// Speed
			// The Guinness World Record for the fastest RC jet-powered aircraft is 749.221 km/h, set by Niels Herbrich in 2017.
			// In our case 350 km/h (97.2 m/s) will be enough, because it gives precision of ~0.37 m/s per 1 bit
			constexpr static uint8_t speedLengthBits = 8;
			constexpr static int16_t speedMaxMPS = 97;

			// 360 deg = 9 bits
			constexpr static uint8_t headingLengthBits = 9;

			// Altitude
			// 13 bit = 8191, not enough
			// 14 bit = 16383, more than enough
			// Mapping [-1000; 10000] to [0; 16383]
			constexpr static uint8_t altitudeLengthBits = 14;
			constexpr static int16_t altitudeMinM = -1'000;
			constexpr static int16_t altitudeMaxM = 10'000;

			// Mode
			constexpr static uint8_t lateralModeLengthBits = 2;
			constexpr static uint8_t verticalModeLengthBits = 3;

			// Ailerons, elevator, throttle
			constexpr static uint8_t percentLengthBits = 7;
	};

	class RemoteAuxiliaryBaroPacket {
		public:
			// 1 hectopascal ~= 7.88 meters ~= 30 feet of altitude, which is not good enough for low altitude RC aircraft
			// So we'll be using decapascals. Cool fact: The maximum recorded atmospheric pressure on Earth, adjusted to sea level,
			// is around 1085.7 hPa (32.09 inHg), occurring in Tonsontsengel, Mongolia (2001)
			// So in theory 1100 daPa will be more than enough
			// So 14 bits
			constexpr static uint8_t referencePressureLengthBits = 14;
	};

	class RemoteAuxiliaryADIRSPacket {
		public:
			constexpr static uint8_t magneticDeclinationLengthBits = 9;
	};

	class RemoteAuxiliaryXCVRPacket {
		public:
			constexpr static uint8_t RFFrequencyLengthBits = 10;
			constexpr static uint8_t bandwidthLengthBits = 4;
			constexpr static uint8_t spreadingFactorLengthBits = 4;
			constexpr static uint8_t codingRateLengthBits = 3;
			constexpr static uint8_t syncWordLengthBits = 8;
			constexpr static uint8_t preambleLengthLengthBits = 16;

			constexpr static uint8_t currentLimitMALengthBits = 8;
			constexpr static uint8_t powerDBmLengthBits = 8;
	};

	class RemoteAuxiliaryPIDPacket {
		public:
			constexpr static uint8_t typeLengthBits = 4;
			constexpr static uint8_t coefficientLengthBits = 32;
	};

	// -------------------------------- Aircraft --------------------------------

	enum class AircraftPacketType : uint8_t {
		telemetryPrimary,
		telemetrySecondary,
		auxiliary,

		maxValue = auxiliary
	};

	class AircraftPacket {
		public:
			constexpr static uint8_t typeLengthBits = 2;
	};

	class AircraftTelemetryPrimaryPacket {
		public:
			// Roll / pitch / yaw
			// Precision of 0.25 - 0.5 deg should be enough for any client-side visualization with LPF
			// So 360 * 1 / 0.25 = 1440 ~= 10 bits

			// Roll range is [-180; 180] deg
			constexpr static uint8_t rollLengthBits = 10;
			constexpr static float rollRangeRad = 2 * std::numbers::pi_v<float>;

			// Pitch is [-90; 90] deg, but we can't use fewer bits
			constexpr static uint8_t pitchLengthBits = 10;
			constexpr static float pitchRangeRad = std::numbers::pi_v<float>;

			// Same as roll, [-180; 180] deg
			constexpr static uint8_t yawLengthBits = 10;
			constexpr static float yawRangeRad = 2 * std::numbers::pi_v<float>;

			// Slip & skid
			// 8 bit = [-127; 127]
			constexpr static uint8_t slipAndSkidLengthBits = 8;
			constexpr static uint8_t slipAndSkidMaxG = 2;

			constexpr static uint8_t speedLengthBits = RemoteAuxiliaryAutopilotPacket::speedLengthBits;
			constexpr static int16_t speedMaxMPS = RemoteAuxiliaryAutopilotPacket::speedMaxMPS;

			constexpr static uint8_t altitudeLengthBits = RemoteAuxiliaryAutopilotPacket::altitudeLengthBits;
			constexpr static int16_t altitudeMinM = RemoteAuxiliaryAutopilotPacket::altitudeMinM;
			constexpr static int16_t altitudeMaxM = RemoteAuxiliaryAutopilotPacket::altitudeMaxM;

			constexpr static uint8_t autopilotRollLengthBits = rollLengthBits;
			constexpr static float autopilotRollRangeRad = rollRangeRad;

			constexpr static uint8_t autopilotPitchLengthBits = pitchLengthBits;
			constexpr static float autopilotPitchRangeRad = pitchRangeRad;
	};

	class AircraftTelemetrySecondaryPacket {
		public:
			constexpr static uint8_t throttleLengthBits = 7;

			constexpr static uint8_t cameraPitchLengthBits = 7;
			constexpr static uint8_t cameraYawLengthBits = cameraPitchLengthBits;

			constexpr static uint8_t latLengthBits = 25;
			constexpr static uint8_t lonLengthBits = 26;

			constexpr static uint8_t batteryLengthBits = 9;

			constexpr static uint8_t autopilotLateralModeLengthBits = RemoteAuxiliaryAutopilotPacket::lateralModeLengthBits;
			constexpr static uint8_t autopilotVerticalModeLengthBits = RemoteAuxiliaryAutopilotPacket::verticalModeLengthBits;

			constexpr static uint8_t autopilotAltitudeLengthBits = RemoteAuxiliaryAutopilotPacket::altitudeLengthBits;
			constexpr static int16_t autopilotAltitudeMinM = RemoteAuxiliaryAutopilotPacket::altitudeMinM;
			constexpr static int16_t autopilotAltitudeMaxM = RemoteAuxiliaryAutopilotPacket::altitudeMaxM;
	};

	enum class AircraftAuxiliaryPacketType : uint8_t {
		calibration,
		XCVRACK,

		maxValue = XCVRACK
	};

	class AircraftAuxiliaryPacket {
		public:
			constexpr static uint8_t typeLengthBits = 1;
	};

	class AircraftAuxiliaryCalibrationPacket {
		public:
			constexpr static uint8_t systemLengthBits = RemoteAuxiliaryCalibratePacket::systemLengthBits;
			constexpr static uint8_t progressLengthBits = 7;
	};

	#pragma pack(pop)
}