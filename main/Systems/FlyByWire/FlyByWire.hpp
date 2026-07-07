#pragma once

#include <cstdint>
#include <atomic>

#include <PIDController.hpp>

#include "Types/Generic.hpp"

namespace pizda {
	using namespace YOBA;

	class FlyByWire {
		public:
			void setup();
			
			float getTargetRollRad() const;
			float getTargetPitchRad() const;
			float getThrottleFactor() const;

			AutopilotLateralMode getLateralMode() const;
			void setLateralMode(const AutopilotLateralMode value);

			AutopilotVerticalMode getVerticalMode() const;
			void setVerticalMode(const AutopilotVerticalMode value);

			float getHoldAltitudeM() const;
			void setHoldAltitudeM(const float value);

			bool isAutothrottleEnabled() const;
			void setAutothrottleEnabled(const bool value);

			bool isAutopilotEngaged() const;
			void setAutopilotEngaged(const bool value);

			void setEmergency(bool state);

		private:
			constexpr static auto _logTag = "FlyByWire";
			
			constexpr static uint32_t _tickFrequencyHz = 30;

			int64_t _computationTimeUs = 0;
			PIDController _yawDeltaToRollPID {};
			PIDController _altitudeToPitchPID {};
			PIDController _speedToPitchPID {};
			PIDController _rollToAileronsPID {};
			PIDController _pitchToElevatorPID {};
			PIDController _speedToThrottlePID {};

			std::atomic<AutopilotLateralMode> _lateralMode { AutopilotLateralMode::dir };
			std::atomic<AutopilotVerticalMode> _verticalMode { AutopilotVerticalMode::dir };

			std::atomic<float> _holdAltitudeM { 0.0f };

			std::atomic<bool> _autothrottleEnabled { false };
			std::atomic<bool> _autopilotEngaged { false };

			bool _emergency = false;

			float _throttleFactor = 0.0f;
			float _rollTargetRad = 0.0f;
			float _pitchTargetRad = 0.0f;

			float _aileronsFactor = 0.5f;
			float _elevatorFactor = 0.5f;
			float _rudderFactor = 0.5f;
			float _noseWheelFactor = 0.5f;

			static float predictValue(float valueDelta, float deltaTimeS, float dueTimeS);
			
			void computeData();
			void applyData() const;

			[[noreturn]] void onStart();
			
	};
}