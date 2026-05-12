#pragma once

#include <esp_log.h>

#include <geoCoordinates.h>

#include "utilities/math.h"

namespace pizda {
	using namespace YOBA;

	class ADIRS {
		public:
			virtual ~ADIRS() = default;

			virtual void setup();
			void setupAsync();

			float getRollRad() const;
			float getPitchRad() const;
			float getYawRad() const;
			float getHeadingDeg() const;

			//			int32_t _speedTime = 0;
//			float _speed = 0;
			
			float getSlipAndSkidFactor() const;
			const GeoCoordinates& getCoordinates() const;
			const Vector3F& getIntegratedVelocityMPS() const;
			void setReferencePressurePa(const uint32_t value);
			float getAirspeedMPS() const;

		protected:
			constexpr static auto _logTag = "ADIRS";
			
			virtual void onTick() = 0;
			virtual void onCalibrateAccelAndGyro() = 0;
			virtual void onCalibrateMag() = 0;

			void setRollRad(const float value);
			void setPitchRad(const float value);
			void setYawRad(const float value);
			void updateHeadingFromYaw();
			void setIntegratedVelocityMPS(const Vector3F& value);
			void setAirspeedMPS(const float value);

			static float computeAltitude(
				const float pressurePa,
				const float temperatureC,
				const uint32_t referencePressurePa,
				const float lapseRateKpm = -0.0065f
			);

			void updateSlipAndSkidFactor(const float lateralAccelerationG, const float maxG);
			void setPressurePa(const float pressurePa);
			void setTemperatureC(const float temperatureC);
			void updateAltitudeFromPressureTemperatureAndReferenceValue();
			void setLatitude(const float value);
			void setLongitude(const float value);

		private:
			float _rollRad = 0;
			float _pitchRad = 0;
			
			float _yawRad = 0;
			float _headingDeg = 0;

			float _slipAndSkidFactor = 0;

			Vector3F _integratedVelocityMPS {};
			Vector3F _integratedPositionMPS {};

			float _airspeedMPS = 0;

			float _pressurePa = 0;
			float _temperatureC = 0;

			uint32_t _referencePressurePa = 101325;

			// 60.014002019765776, 29.717151511256816
			// ОПЯТЬ ЖЕНЩИНЫ??? ФЕДЯ СУКА ЭТО ТЫ ЕБЛАН СДЕЛАЛ
			GeoCoordinates _coordinates {
				toRadians(60.014581566191914f),
				toRadians(29.70258579817704f),
				0
			};
			
			[[noreturn]] void onStart();
	};
}