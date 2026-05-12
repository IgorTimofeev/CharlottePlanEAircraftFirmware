#pragma once

#include <cstdint>

#include <NVSSettings.h>

#include "types/generic.h"

namespace pizda {
	using namespace YOBA;
	
	class MotorsSettings : public NVSSettings {
		public:
			MotorSettings throttle {};
			MotorSettings noseWheel {};
			
			MotorSettings flapLeft {};
			MotorSettings aileronLeft {};
			
			MotorSettings flapRight {};
			MotorSettings aileronRight {};
			
			MotorSettings tailLeft {};
			MotorSettings tailRight {};
			
			MotorSettings cameraPitch {};
			MotorSettings cameraYaw {};
		
		protected:
			const char* getNamespace() override {
				return _namespace;
			}
			
			void onRead(const NVSStream& stream) override {
				readMotor(
					stream,
					throttle,

					_throttleMin,
					_throttleMax,
					_throttleReverse
				);

				readMotor(
					stream,
					noseWheel,

					_noseWheelMin,
					_noseWheelMax,
					_noseWheelReverse
				);

				readMotor(
					stream,
					flapLeft,

					_flapLeftMin,
					_flapLeftMax,
					_flapLeftReverse
				);

				readMotor(
					stream,
					aileronLeft,

					_aileronLeftMin,
					_aileronLeftMax,
					_aileronLeftReverse
				);

				readMotor(
					stream,
					flapRight,

					_flapRightMin,
					_flapRightMax,
					_flapRightReverse
				);

				readMotor(
					stream,
					aileronRight,

					_aileronRightMin,
					_aileronRightMax,
					_aileronRightReverse
				);

				readMotor(
					stream,
					tailLeft,

					_tailLeftMin,
					_tailLeftMax,
					_tailLeftReverse
				);

				readMotor(
					stream,
					tailRight,

					_tailRightMin,
					_tailRightMax,
					_tailRightReverse
				);
				
				readMotor(
					stream,
					cameraPitch,

					_cameraPitchMin,
					_cameraPitchMax,
					_cameraPitchReverse
				);

				readMotor(
					stream,
					cameraYaw,

					_cameraYawMin,
					_cameraYawMax,
					_cameraYawReverse
				);
			}

			void onWrite(const NVSStream& stream) override {
				writeMotor(
					stream,
					throttle,

					_throttleMin,
					_throttleMax,
					_throttleReverse
				);

				writeMotor(
					stream,
					noseWheel,

					_noseWheelMin,
					_noseWheelMax,
					_noseWheelReverse
				);

				writeMotor(
					stream,
					flapLeft,

					_flapLeftMin,
					_flapLeftMax,
					_flapLeftReverse
				);

				writeMotor(
					stream,
					aileronLeft,

					_aileronLeftMin,
					_aileronLeftMax,
					_aileronLeftReverse
				);

				writeMotor(
					stream,
					flapRight,

					_flapRightMin,
					_flapRightMax,
					_flapRightReverse
				);

				writeMotor(
					stream,
					aileronRight,

					_aileronRightMin,
					_aileronRightMax,
					_aileronRightReverse
				);

				writeMotor(
					stream,
					tailLeft,

					_tailLeftMin,
					_tailLeftMax,
					_tailLeftReverse
				);

				writeMotor(
					stream,
					tailRight,

					_tailRightMin,
					_tailRightMax,
					_tailRightReverse
				);
				
				writeMotor(
					stream,
					cameraPitch,

					_cameraPitchMin,
					_cameraPitchMax,
					_cameraPitchReverse
				);

				writeMotor(
					stream,
					cameraYaw,

					_cameraYawMin,
					_cameraYawMax,
					_cameraYawReverse
				);
			}

		private:
			constexpr static auto _namespace = "mt2";

			constexpr static auto _throttleMin = "thm";
			constexpr static auto _throttleMax = "thx";
			constexpr static auto _throttleReverse = "thr";

			constexpr static auto _noseWheelMin = "nwm";
			constexpr static auto _noseWheelMax = "nwx";
			constexpr static auto _noseWheelReverse = "nwr";

			constexpr static auto _flapLeftMin = "flm";
			constexpr static auto _flapLeftMax = "flx";
			constexpr static auto _flapLeftReverse = "flr";

			constexpr static auto _aileronLeftMin = "alm";
			constexpr static auto _aileronLeftMax = "alx";
			constexpr static auto _aileronLeftReverse = "alr";

			constexpr static auto _flapRightMin = "frm";
			constexpr static auto _flapRightMax = "frx";
			constexpr static auto _flapRightReverse = "frr";

			constexpr static auto _aileronRightMin = "arm";
			constexpr static auto _aileronRightMax = "arx";
			constexpr static auto _aileronRightReverse = "arr";

			constexpr static auto _tailLeftMin = "tlm";
			constexpr static auto _tailLeftMax = "tlx";
			constexpr static auto _tailLeftReverse = "tlr";

			constexpr static auto _tailRightMin = "trm";
			constexpr static auto _tailRightMax = "trx";
			constexpr static auto _tailRightReverse = "trr";

			constexpr static auto _cameraPitchMin = "cpm";
			constexpr static auto _cameraPitchMax = "cpx";
			constexpr static auto _cameraPitchReverse = "cpr";

			constexpr static auto _cameraYawMin = "cym";
			constexpr static auto _cameraYawMax = "cyx";
			constexpr static auto _cameraYawReverse = "cyr";
			
			static void readMotor(
				const NVSStream& stream,
				MotorSettings& motor,
				
				const char* minKey,
				const char* maxKey,
				const char* reverseKey
			) {
				motor.min = stream.readUint16(minKey, 1000);
				motor.max = stream.readUint16(maxKey, 2000);
				motor.reverse = stream.readBool(reverseKey, false);
			}
			
			static void writeMotor(
				const NVSStream& stream,
				const MotorSettings& motor,
				
				const char* minKey,
				const char* maxKey,
				const char* reverseKey
			) {
				stream.writeUint16(minKey, motor.min);
				stream.writeUint16(maxKey, motor.max);
				stream.writeBool(reverseKey, motor.reverse);
			}
	};
}