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
			}
		
		private:
			constexpr static const char* _namespace = "mt2";
			
			constexpr static const char* _throttleMin = "thm";
			constexpr static const char* _throttleMax = "thx";
			constexpr static const char* _throttleReverse = "thr";
			
			constexpr static const char* _noseWheelMin = "nwm";
			constexpr static const char* _noseWheelMax = "nwx";
			constexpr static const char* _noseWheelReverse = "nwr";
			
			constexpr static const char* _flapLeftMin = "flm";
			constexpr static const char* _flapLeftMax = "flx";
			constexpr static const char* _flapLeftReverse = "flr";
			
			constexpr static const char* _aileronLeftMin = "alm";
			constexpr static const char* _aileronLeftMax = "alx";
			constexpr static const char* _aileronLeftReverse = "alr";
			
			constexpr static const char* _flapRightMin = "frm";
			constexpr static const char* _flapRightMax = "frx";
			constexpr static const char* _flapRightReverse = "frr";
			
			constexpr static const char* _aileronRightMin = "arm";
			constexpr static const char* _aileronRightMax = "arx";
			constexpr static const char* _aileronRightReverse = "arr";
			
			constexpr static const char* _tailLeftMin = "tlm";
			constexpr static const char* _tailLeftMax = "tlx";
			constexpr static const char* _tailLeftReverse = "tlr";
			
			constexpr static const char* _tailRightMin = "trm";
			constexpr static const char* _tailRightMax = "trx";
			constexpr static const char* _tailRightReverse = "trr";
			
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