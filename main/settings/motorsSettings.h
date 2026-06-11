#pragma once

#include <cstdint>

#include <NVSSettings.h>

#include "types/generic.h"

namespace pizda {
	using namespace YOBA;
	
	class MotorsSettings : public NVSSettings {
		public:
			MotorSettings cameraPitch {};
			MotorSettings cameraYaw {};

			MotorSettings throttleLeft {};
			MotorSettings throttleRight {};
			MotorSettings noseWheel {};
			
			MotorSettings flapLeft {};
			MotorSettings aileronLeft {};
			
			MotorSettings flapRight {};
			MotorSettings aileronRight {};
			
			MotorSettings tailLeft {};
			MotorSettings tailRight {};

			MotorSettings* getByType(const MotorType type) {
				switch (type) {
					case MotorType::cameraPitch: return &cameraPitch;
					case MotorType::cameraYaw: return &cameraPitch;

					case MotorType::throttleLeft: return &throttleLeft;
					case MotorType::throttleRight: return &throttleRight;
					case MotorType::noseWheel: return &noseWheel;

					case MotorType::flapLeft: return &flapLeft;
					case MotorType::aileronLeft: return &aileronLeft;

					case MotorType::flapRight: return &flapRight;
					case MotorType::aileronRight: return &aileronRight;

					case MotorType::tailLeft: return &tailLeft;
					case MotorType::tailRight: return &tailRight;

					default: return nullptr;
				}
			}
		
		protected:
			const char* getNamespace() override {
				return _namespace;
			}
			
			void onRead(const NVSStream& stream) override {
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

				readMotor(
					stream,
					throttleLeft,

					_throttleLeftMin,
					_throttleLeftMax,
					_throttleLeftReverse
				);

				readMotor(
					stream,
					throttleRight,

					_throttleRightMin,
					_throttleRightMax,
					_throttleRightReverse
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
			}

			void onWrite(const NVSStream& stream) override {
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

				writeMotor(
					stream,
					throttleLeft,

					_throttleLeftMin,
					_throttleLeftMax,
					_throttleLeftReverse
				);

				writeMotor(
					stream,
					throttleRight,

					_throttleRightMin,
					_throttleRightMax,
					_throttleRightReverse
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
			}

		private:
			constexpr static auto _namespace = "mt2";

			constexpr static auto _cameraPitchMin = "cpm";
			constexpr static auto _cameraPitchMax = "cpx";
			constexpr static auto _cameraPitchReverse = "cpr";

			constexpr static auto _cameraYawMin = "cym";
			constexpr static auto _cameraYawMax = "cyx";
			constexpr static auto _cameraYawReverse = "cyr";

			constexpr static auto _throttleLeftMin = "tlm";
			constexpr static auto _throttleLeftMax = "tlx";
			constexpr static auto _throttleLeftReverse = "tlr";

			constexpr static auto _throttleRightMin = "trm";
			constexpr static auto _throttleRightMax = "trx";
			constexpr static auto _throttleRightReverse = "trr";

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

			constexpr static auto _tailLeftMin = "llm";
			constexpr static auto _tailLeftMax = "llx";
			constexpr static auto _tailLeftReverse = "llr";

			constexpr static auto _tailRightMin = "lrm";
			constexpr static auto _tailRightMax = "lrx";
			constexpr static auto _tailRightReverse = "lrr";
			
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