#pragma once

#include <cstdint>
#include <atomic>

#include <NVSSettings.hpp>

#include "Types/Generic.hpp"
#include "Utilities/Math.hpp"

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

			static void read(const NVSStream& stream, const char* keyP, const char* keyI, const char* keyD, PIDCoefficients& coefficients, const PIDCoefficients& fallbackCoefficients) {
				coefficients.p = stream.readFloat(keyP, fallbackCoefficients.p);
				coefficients.i = stream.readFloat(keyI, fallbackCoefficients.i);
				coefficients.d = stream.readFloat(keyD, fallbackCoefficients.d);
			}

			static void write(const NVSStream& stream, const char* keyP, const char* keyI, const char* keyD, const PIDCoefficients& coefficients) {
				stream.writeFloat(keyP, coefficients.p);
				stream.writeFloat(keyI, coefficients.i);
				stream.writeFloat(keyD, coefficients.d);
			}
	};

	class AutopilotConfigurationSettings : public NVSSettings {
		public:
			// Lateral
			AutopilotLateralMode lateralMode = AutopilotLateralMode::hdg;
			float maxRollAngleRad = 0;
			float stabilizedModeRollAngleIncrementRadPerSecond = 0;
			float rollAngleEMAFilterFactorPerSecond = 0;
			uint8_t maxAileronsPercent = 0;

			// Vertical
			AutopilotVerticalMode verticalMode = AutopilotVerticalMode::flc;
			float maxPitchAngleRad = 0;
			float stabilizedModePitchAngleIncrementRadPerSecond = 0;
			float pitchAngleEMAFilterFactorPerSecond = 0;
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
				rollAngleEMAFilterFactorPerSecond = stream.readFloat(_rollAngleEMAFilterFactorPerSecond, 0.8f);
				maxAileronsPercent = stream.readUint8(_maxAileronsPercent, 100);

				// Vertical
				verticalMode = stream.readEnum<AutopilotVerticalMode>(_verticalMode, AutopilotVerticalMode::flc);
				maxPitchAngleRad = stream.readFloat(_maxPitchAngleRad, toRadians(15));
				stabilizedModePitchAngleIncrementRadPerSecond = stream.readFloat(_stabilizedModePitchAngleIncrementRadPerSecond, toRadians(5));
				pitchAngleEMAFilterFactorPerSecond = stream.readFloat(_pitchAngleEMAFilterFactorPerSecond, 0.8f);
				maxElevatorPercent = stream.readUint8(_maxElevatorPercent, 100);

				// Longitudinal
				minThrottlePercent = stream.readUint8(_minThrottlePercent, 0);
				maxThrottlePercent = stream.readUint8(_maxThrottlePercent, 100);

				// PIDs
				AutopilotSettingsPIDs::read(stream, _yawToRollP, _yawToRollI, _yawToRollD, PIDs.yawToRoll, { 0.8f, 0.1f, 0.3f });
				AutopilotSettingsPIDs::read(stream, _altitudeToPitchP, _altitudeToPitchI, _altitudeToPitchD, PIDs.altitudeToPitch, { 0.04f, 0.01f, 0.01f });
				AutopilotSettingsPIDs::read(stream, _speedToPitchP, _speedToPitchI, _speedToPitchD, PIDs.speedToPitch, { 0.2f, 0.05f, 0.01f });
				AutopilotSettingsPIDs::read(stream, _rollToAileronsP, _rollToAileronsI, _rollToAileronsD, PIDs.rollToAilerons, { 2.5f, 0.01f, 0.2f });
				AutopilotSettingsPIDs::read(stream, _pitchToElevatorP, _pitchToElevatorI, _pitchToElevatorD, PIDs.pitchToElevator, { 3.5f, 0.3f, 0.2f });
				AutopilotSettingsPIDs::read(stream, _speedToThrottleP, _speedToThrottleI, _speedToThrottleD, PIDs.speedToThrottle, { 0.4f, 0.1f, 0.1f });
			}

			void onWrite(const NVSStream& stream) override {
				// Lateral
				stream.writeEnum<AutopilotLateralMode>(_lateralMode, lateralMode);
				stream.writeFloat(_maxRollAngleRad, maxRollAngleRad);
				stream.writeFloat(_stabilizedModeRollAngleIncrementRadPerSecond, stabilizedModeRollAngleIncrementRadPerSecond);
				stream.writeFloat(_rollAngleEMAFilterFactorPerSecond, rollAngleEMAFilterFactorPerSecond);
				stream.writeUint8(_maxAileronsPercent, maxAileronsPercent);

				// Vertical
				stream.writeEnum<AutopilotVerticalMode>(_verticalMode, verticalMode);
				stream.writeFloat(_maxPitchAngleRad, maxPitchAngleRad);
				stream.writeFloat(_stabilizedModePitchAngleIncrementRadPerSecond, stabilizedModePitchAngleIncrementRadPerSecond);
				stream.writeFloat(_pitchAngleEMAFilterFactorPerSecond, pitchAngleEMAFilterFactorPerSecond);
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
			constexpr static auto _rollAngleEMAFilterFactorPerSecond = "raef";
			constexpr static auto _maxAileronsPercent = "aipe";

			// Vertical
			constexpr static auto _verticalMode = "vtmd";
			constexpr static auto _maxPitchAngleRad = "mpia";
			constexpr static auto _stabilizedModePitchAngleIncrementRadPerSecond = "pair";
			constexpr static auto _pitchAngleEMAFilterFactorPerSecond = "paef";
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

	class AutopilotSelectionSettings : public NVSSettings {
		public:
			float getSelectedSpeedMPS() const {
				return _selectedSpeedMPS.load(std::memory_order_acquire);
			}

			void setSelectedSpeedMPS(const float value) {
				_selectedSpeedMPS.store(value, std::memory_order_release);
			}

			uint16_t getSelectedHeadingDeg() const {
				return _selectedHeadingDeg.load(std::memory_order_acquire);
			}

			void setSelectedHeadingDeg(const uint16_t value) {
				_selectedHeadingDeg.store(value, std::memory_order_release);
			}

			float getSelectedAltitudeM() const {
				return _selectedAltitudeM.load(std::memory_order_acquire);
			}

			void setSelectedAltitudeM(const float value) {
				_selectedAltitudeM.store(value, std::memory_order_release);
			}

		protected:
			const char* getNamespace() override {
				return _namespace;
			}

			void onRead(const NVSStream& stream) override {
				setSelectedSpeedMPS(stream.readFloat(_selectedSpeedMPSKey, 0.0f));
				setSelectedHeadingDeg(stream.readUint16(_selectedHeadingDegKey, 0));
				setSelectedAltitudeM(stream.readFloat(_selectedAltitudeMKey, 0.0f));
			}

			void onWrite(const NVSStream& stream) override {
				stream.writeFloat(_selectedSpeedMPSKey, getSelectedSpeedMPS());
				stream.writeUint16(_selectedHeadingDegKey, getSelectedHeadingDeg());
				stream.writeFloat(_selectedAltitudeMKey, getSelectedAltitudeM());
			}

		private:
			constexpr static auto _namespace = "apm";

			constexpr static auto _selectedSpeedMPSKey = "sp";
			constexpr static auto _selectedHeadingDegKey = "hd";
			constexpr static auto _selectedAltitudeMKey = "as";

			std::atomic<float> _selectedSpeedMPS { 0.0f };
			std::atomic<uint16_t> _selectedHeadingDeg { 0 };
			std::atomic<float> _selectedAltitudeM { 0.0f };

	};
}