#pragma once

#include <cmath>
#include <algorithm>

#include <esp_log.h>

#include <vector3.h>

#include "utilities/math.h"

namespace pizda {
	using namespace YOBA;

	class AdaptiveComplimentaryFiler {
		public:
			static void apply(
				const Vector3F& accelData,
				const Vector3F& gyroData,
				const Vector3F& magData,

				const float deltaTimeS,

				const float rollAndPitchGyroTrustFactorMin,
				const float rollAndPitchGyroTrustFactorMax,

				const float magGyroTrustFactor,

				float& rollRad,
				float& pitchRad,
				float& yawRad
			) {
				const float accelRoll = -std::atan2(accelData.getX(), accelData.getZ());
				const float accelPitch = std::atan2(accelData.getY(), accelData.getZ());

				const float gyroRoll = rollRad + toRadians(gyroData.getY()) * deltaTimeS;
				const float gyroPitch = pitchRad + toRadians(gyroData.getX()) * deltaTimeS;
				const float gyroYaw = yawRad + toRadians(gyroData.getX()) * deltaTimeS;

				// Filter itself

				// Roll/pitch
				{
					const float accelMagnitude = accelData.getLength();
					// In steady state accel magnitude should ~= 1G
					const float accelMagnitudeError = std::abs(accelMagnitude - 1);
					// Let error threshold also be 1G
					constexpr static float accelMagnitudeErrorThreshold = 1;
					// More error -> more trust to gyro
					const float accelMagnitudeFactor = std::clamp(accelMagnitudeError / accelMagnitudeErrorThreshold, 0.0f, 1.0f);

					const float rollAndPitchGyroTrustFactor =
						rollAndPitchGyroTrustFactorMin
						+ (rollAndPitchGyroTrustFactorMax - rollAndPitchGyroTrustFactorMin)
						* accelMagnitudeFactor;

					rollRad = applyGyroTrustFactor(accelRoll, gyroRoll, rollAndPitchGyroTrustFactor);
					pitchRad = applyGyroTrustFactor(accelPitch, gyroPitch, rollAndPitchGyroTrustFactor);
				}

				// Mag tilt compensation using computed pitch/roll
//				float magYawWithoutTilt = std::atan2(magData.getX(), magData.getY());

				const auto magDataTilt = applyTiltCompensation(magData, rollRad, pitchRad);
				const auto magYawRad = std::atan2(magDataTilt.getX(), magDataTilt.getY());

				// For mag, we're using other gyro trust factor, because mag produces a lot of noise
				yawRad = applyGyroTrustFactor(magYawRad, gyroYaw, magGyroTrustFactor);

//					ESP_LOGI("Compl", "acc roll/pitch: %f x %f", toDegrees(accelRoll), toDegrees(accelPitch));
//					ESP_LOGI("Compl", "acc: %f x %f x %f", accelData.getX(), accelData.getY(), accelData.getZ());
//					ESP_LOGI("Compl", "gyr: %f x %f x %f", gyroData.getX(), gyroData.getY(), gyroData.getZ());
//					ESP_LOGI("Compl", "mag: %f x %f x %f", magData.getX(), magData.getY(), magData.getZ());
//					ESP_LOGI("Compl", "mag cor: %f x %f x %f", magDataTilt.getX(), magDataTilt.getY(), magDataTilt.getZ());
//					ESP_LOGI("Compl", "mag yaw no tilt: %f", toDegrees(magYawWithoutTilt));

					// ESP_LOGI("Compl", "gyr yaw: %f", toDegrees(yawRad));
					// ESP_LOGI("Compl", "mag yaw: %f", toDegrees(magYawRad));
			}

			static Vector3F applyTiltCompensation(const Vector3F& vec, const float rollRad, const float pitchRad) {
				return vec.rotateAroundXAxis(pitchRad).rotateAroundYAxis(rollRad);
			}

		private:
			static float applyGyroTrustFactor(const float nonGyroValue, const float gyroValue, const float gyroTrustFactor) {
				// Normally calculation logic should be as simple as
				// return nonGyroValue * (1.0f - gyroTrustFactor) + gyroValue * gyroTrustFactor;

				// But since we're dealing with angles, we should do some range checks to keep result in [-pi; pi] range
				// Those checks can be done here, but we can simply use LPF, because it behaves the same way
				return LowPassFilter::applyToAngle(nonGyroValue, gyroValue, gyroTrustFactor);
			}
	};

}