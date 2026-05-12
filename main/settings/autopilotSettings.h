#pragma once

#include <cstdint>

#include <NVSSettings.h>

#include "types/generic.h"
#include "utilities/math.h"

namespace pizda {
	using namespace YOBA;

	class AutopilotSettingsPIDs {
		public:
			PIDCoefficients yawToRoll {};
			PIDCoefficients altitudeToPitch {};
			PIDCoefficients speedToPitch {};

			PIDCoefficients rollToAilerons {};
			PIDCoefficients pitchToElevator {};

			PIDCoefficients speedToThrottle {};

			static void read(const NVSStream& stream, const char* keyP, const char* keyI, const char* keyD, PIDCoefficients& coefficients) {
				coefficients.p = stream.readFloat(keyP, 1);
				coefficients.i = stream.readFloat(keyI, 0);
				coefficients.d = stream.readFloat(keyD, 0);
			}

			static void write(const NVSStream& stream, const char* keyP, const char* keyI, const char* keyD, const PIDCoefficients& coefficients) {
				stream.writeFloat(keyP, coefficients.p);
				stream.writeFloat(keyI, coefficients.i);
				stream.writeFloat(keyD, coefficients.d);
			}
	};

	class AutopilotSettings : public NVSSettings {
		public:
			// Lateral
			AutopilotLateralMode lateralMode = AutopilotLateralMode::hdg;
			float maxRollAngleRad = 0;
			float stabilizedModeRollAngleIncrementRadPerSecond = 0;
			float rollAngleLPFFactorPerSecond = 0;
			uint8_t maxAileronsPercent = 0;

			// Vertical
			AutopilotVerticalMode verticalMode = AutopilotVerticalMode::flc;
			float maxPitchAngleRad = 0;
			float stabilizedModePitchAngleIncrementRadPerSecond = 0;
			float pitchAngleLPFFactorPerSecond = 0;
			uint8_t maxElevatorPercent = 0;

			// Longitudinal
			uint8_t minThrottlePercent = 0;
			uint8_t maxThrottlePercent = 0;

			// PIDs
			AutopilotSettingsPIDs PIDs {};

		protected:
			const char* getNamespace() override {
				return _namespace;
			}

			void onRead(const NVSStream& stream) override {
				// Lateral
				lateralMode = stream.readEnum<AutopilotLateralMode>(_lateralMode, AutopilotLateralMode::hdg);
				maxRollAngleRad = stream.readFloat(_maxRollAngleRad, toRadians(30));
				stabilizedModeRollAngleIncrementRadPerSecond = stream.readFloat(_stabilizedModeRollAngleIncrementRadPerSecond, toRadians(5));
				rollAngleLPFFactorPerSecond = stream.readFloat(_rollAngleLPFFactorPerSecond, 0.5f);
				maxAileronsPercent = stream.readUint8(_maxAileronsPercent, 100);

				// Vertical
				verticalMode = stream.readEnum<AutopilotVerticalMode>(_verticalMode, AutopilotVerticalMode::flc);
				maxPitchAngleRad = stream.readFloat(_maxPitchAngleRad, toRadians(20));
				stabilizedModePitchAngleIncrementRadPerSecond = stream.readFloat(_stabilizedModePitchAngleIncrementRadPerSecond, toRadians(5));
				pitchAngleLPFFactorPerSecond = stream.readFloat(_pitchAngleLPFFactorPerSecond, 0.5f);
				maxElevatorPercent = stream.readUint8(_maxElevatorPercent, 100);

				// Longitudinal
				minThrottlePercent = stream.readUint8(_minThrottlePercent, 0);
				maxThrottlePercent = stream.readUint8(_maxThrottlePercent, 100);

				// PIDs
				AutopilotSettingsPIDs::read(stream, _yawToRollP, _yawToRollI, _yawToRollD, PIDs.yawToRoll);
				AutopilotSettingsPIDs::read(stream, _altitudeToPitchP, _altitudeToPitchI, _altitudeToPitchD, PIDs.altitudeToPitch);
				AutopilotSettingsPIDs::read(stream, _speedToPitchP, _speedToPitchI, _speedToPitchD, PIDs.speedToPitch);
				AutopilotSettingsPIDs::read(stream, _rollToAileronsP, _rollToAileronsI, _rollToAileronsD, PIDs.rollToAilerons);
				AutopilotSettingsPIDs::read(stream, _pitchToElevatorP, _pitchToElevatorI, _pitchToElevatorD, PIDs.pitchToElevator);
				AutopilotSettingsPIDs::read(stream, _speedToThrottleP, _speedToThrottleI, _speedToThrottleD, PIDs.speedToThrottle);
			}

			void onWrite(const NVSStream& stream) override {
				// Lateral
				stream.writeEnum<AutopilotLateralMode>(_lateralMode, lateralMode);
				stream.writeFloat(_maxRollAngleRad, maxRollAngleRad);
				stream.writeFloat(_stabilizedModeRollAngleIncrementRadPerSecond, stabilizedModeRollAngleIncrementRadPerSecond);
				stream.writeFloat(_rollAngleLPFFactorPerSecond, rollAngleLPFFactorPerSecond);
				stream.writeUint8(_maxAileronsPercent, maxAileronsPercent);

				// Vertical
				stream.writeEnum<AutopilotVerticalMode>(_verticalMode, verticalMode);
				stream.writeFloat(_maxPitchAngleRad, maxPitchAngleRad);
				stream.writeFloat(_stabilizedModePitchAngleIncrementRadPerSecond, stabilizedModePitchAngleIncrementRadPerSecond);
				stream.writeFloat(_pitchAngleLPFFactorPerSecond, pitchAngleLPFFactorPerSecond);
				stream.writeUint8(_maxElevatorPercent, maxElevatorPercent);

				// Longitudinal
				stream.writeUint8(_minThrottlePercent, minThrottlePercent);
				stream.writeUint8(_maxThrottlePercent, maxThrottlePercent);

				// PIDs
				AutopilotSettingsPIDs::write(stream, _yawToRollP, _yawToRollI, _yawToRollD, PIDs.yawToRoll);
				AutopilotSettingsPIDs::write(stream, _altitudeToPitchP, _altitudeToPitchI, _altitudeToPitchD, PIDs.altitudeToPitch);
				AutopilotSettingsPIDs::write(stream, _speedToPitchP, _speedToPitchI, _speedToPitchD, PIDs.speedToPitch);
				AutopilotSettingsPIDs::write(stream, _rollToAileronsP, _rollToAileronsI, _rollToAileronsD, PIDs.rollToAilerons);
				AutopilotSettingsPIDs::write(stream, _pitchToElevatorP, _pitchToElevatorI, _pitchToElevatorD, PIDs.pitchToElevator);
				AutopilotSettingsPIDs::write(stream, _speedToThrottleP, _speedToThrottleI, _speedToThrottleD, PIDs.speedToThrottle);
			}

		private:
			constexpr static auto _namespace = "ap1";

			// Lateral
			constexpr static auto _lateralMode = "ltmd";
			constexpr static auto _maxRollAngleRad = "mrla";
			constexpr static auto _stabilizedModeRollAngleIncrementRadPerSecond = "rair";
			constexpr static auto _rollAngleLPFFactorPerSecond = "ralf";
			constexpr static auto _maxAileronsPercent = "aipe";

			// Vertical
			constexpr static auto _verticalMode = "vtmd";
			constexpr static auto _maxPitchAngleRad = "mpia";
			constexpr static auto _stabilizedModePitchAngleIncrementRadPerSecond = "pair";
			constexpr static auto _pitchAngleLPFFactorPerSecond = "palf";
			constexpr static auto _maxElevatorPercent = "elpe";

			// Longitudinal
			constexpr static auto _minThrottlePercent = "tmip";
			constexpr static auto _maxThrottlePercent = "tmap";

			// PIDs
			constexpr static auto _yawToRollP = "pyrp";
			constexpr static auto _yawToRollI = "pyri";
			constexpr static auto _yawToRollD = "pyrd";

			constexpr static auto _altitudeToPitchP = "papp";
			constexpr static auto _altitudeToPitchI = "papi";
			constexpr static auto _altitudeToPitchD = "papd";

			constexpr static auto _speedToPitchP = "pspp";
			constexpr static auto _speedToPitchI = "pspi";
			constexpr static auto _speedToPitchD = "pspd";

			constexpr static auto _rollToAileronsP = "prap";
			constexpr static auto _rollToAileronsI = "prai";
			constexpr static auto _rollToAileronsD = "prad";

			constexpr static auto _pitchToElevatorP = "ppep";
			constexpr static auto _pitchToElevatorI = "ppei";
			constexpr static auto _pitchToElevatorD = "pped";

			constexpr static auto _speedToThrottleP = "pstp";
			constexpr static auto _speedToThrottleI = "psti";
			constexpr static auto _speedToThrottleD = "pstd";
	};
}
