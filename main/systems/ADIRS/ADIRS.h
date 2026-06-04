#pragma once

#include <atomic>

#include <geoCoordinates.h>

#include "utilities/math.h"

namespace pizda {
	using namespace YOBA;

	class ADIRS {
		public:
			virtual ~ADIRS() = default;

			virtual void setup();

			float getRollRad() const;
			float getPitchRad() const;
			float getYawRad() const;
			float getHeadingDeg() const;
			float getSlipAndSkidFactor() const;

			float getAirspeedMPS() const;

			float getHomeLatitude() const;
			float getHomeLongitude() const;
			float getHomeAltitude() const;

			float getLatitude() const;
			float getLongitude() const;
			float getAltitude() const;

		protected:
			constexpr static auto _logTag = "ADIRS";
			
			virtual void onTick() = 0;
			virtual void onCalibrateAccelAndGyro() = 0;
			virtual void onCalibrateMag() = 0;

			void setRollRad(const float value);
			void setPitchRad(const float value);
			void setYawRad(const float value);
			void updateHeadingFromYaw();
			void setAirspeedMS(const float value);

			static float computeAltitude(
				const float pressurePa,
				const float temperatureC,
				const uint32_t referencePressurePa,
				const float lapseRateKPM = -0.0065f
			);

			void updateSlipAndSkidFactor(const float lateralAccelerationG, const float maxG);
			void setPressurePa(const float value);
			void setTemperatureC(const float value);
			void updateAltitudeFromPressureTemperatureAndReferenceValue();

			virtual void setHomeCoordinates(const float latitude, const float longitude, const float altitude);

			void setLatitude(const float value);
			void setLongitude(const float value);
			void setAltitude(const float value);

		private:
			std::atomic<float> _rollRad { 0.0f };
			std::atomic<float> _pitchRad { 0.0f };
			
			std::atomic<float> _yawRad { 0.0f };
			std::atomic<float> _headingDeg { 0.0f };

			std::atomic<float> _slipAndSkidFactor { 0.0f };

			std::atomic<float> _airspeedMPS { 0.0f };

			std::atomic<float> _pressurePa { 0.0f };
			std::atomic<float> _temperatureC { 0.0f };

			// // 60.014002019765776, 29.717151511256816
			// // ОПЯТЬ ЖЕНЩИНЫ??? ФЕДЯ СУКА ЭТО ТЫ ЕБЛАН СДЕЛАЛ
			// GeoCoordinates _coordinates {
			// 	toRadians(60.014581566191914f),
			// 	toRadians(29.70258579817704f),
			// 	0
			// };

			// От греха подальше, а то, блядь, по Кронштадту ебнули сегодня, и нормальным авиамоделистам
			// из-за такой хуйни остается лишь сосать бибу
			std::atomic<float> _homeLatitude { toRadians(59.812414f) };
			std::atomic<float> _homeLongitude { toRadians(30.555595f) };
			std::atomic<float> _homeAltitude { 0.0f };

			std::atomic<float> _latitude { _homeLatitude.load(std::memory_order_acquire) };
			std::atomic<float> _longitude { _homeLongitude.load(std::memory_order_acquire) };
			std::atomic<float> _altitude { _homeAltitude.load(std::memory_order_acquire) };

			[[noreturn]] void onStart();
	};
}