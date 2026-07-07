#pragma once

#include <cstdint>

#include <Vector3.hpp>

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
			);

			static Vector3F applyTiltCompensation(const Vector3F& vec, const float rollRad, const float pitchRad);

		private:
			static float applyGyroTrustFactor(const float nonGyroValue, const float gyroValue, const float gyroTrustFactor);
	};

}