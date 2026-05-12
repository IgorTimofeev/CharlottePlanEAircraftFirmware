#pragma once

#include <cstdint>

#include <PIDController.h>

#include "types/generic.h"

namespace pizda {
	using namespace YOBA;

	class FlyByWire {
		public:
			void setup();
			
			float getSelectedSpeedMps() const;
			void setSelectedSpeedMps(float value);
			
			uint16_t getSelectedHeadingDeg() const;
			void setSelectedHeadingDeg(uint16_t value);
			
			float getSelectedAltitudeM() const;
			void setSelectedAltitudeM(float value);

			float getHoldAltitudeM() const;
			void setHoldAltitudeM(float value);

			AutopilotLateralMode getLateralMode() const;
			void setLateralMode(AutopilotLateralMode value);
			
			AutopilotVerticalMode getVerticalMode() const;
			void setVerticalMode(AutopilotVerticalMode value);
			
			bool isAutothrottleEnabled() const;
			void setAutothrottleEnabled(bool value);
			
			bool isAutopilotEngaged() const;
			void setAutopilotEngaged(bool value);

			float getTargetRollRad() const;
			float getTargetPitchRad() const;

		private:
			constexpr static auto _logTag = "FlyByWire";
			
			constexpr static uint32_t _tickFrequencyHz = 30;

			int64_t _computationTimeUs = 0;

			float _throttleFactor = 0;
			float _rollTargetRad = 0;
			float _pitchTargetRad = 0;
			
			float _aileronsFactor = 0.5;
			float _elevatorFactor = 0.5;
			float _rudderFactor = 0.5;

			float _speedSelectedMPS = 0;
			uint16_t _headingSelectedDeg = 0;
			float _altitudeSelectedM = 0;
			float _altitudeHoldM = 0;

			AutopilotLateralMode _lateralMode = AutopilotLateralMode::dir;
			AutopilotVerticalMode _verticalMode = AutopilotVerticalMode::dir;

			PIDController _yawDeltaToRollPID {};
			PIDController _altitudeToPitchPID {};
			PIDController _speedToPitchPID {};
			PIDController _rollToAileronsPID {};
			PIDController _pitchToElevatorPID {};
			PIDController _speedToThrottlePID {};

			bool _autothrottleEnabled = false;
			bool _autopilotEngaged = false;

			static float predictValue(float valueDelta, float deltaTimeS, float dueTimeS);
			
			void computeData();
			void applyData() const;

			[[noreturn]] void onStart();
			
	};
}