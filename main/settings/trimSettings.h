#pragma once

#include <vector>

#include <NVSSettings.h>
#include <NVSStream.h>

namespace pizda {
	using namespace YOBA;
	
	class TrimSettings : public NVSSettings {
		public:
			// Trim
			// Pre-mapped to [-0.5; 0.5]
			float aileronsTrim = 0;
			float elevatorTrim = 0;
			float rudderTrim = 0;
		
		protected:
			const char* getNamespace() override {
				return "ct1";
			}
			
			void onRead(const NVSStream& stream) override {
				// Trim
				aileronsTrim = stream.readFloat(_aileronsTrim, 0);
				elevatorTrim = stream.readFloat(_elevatorTrim, 0);
				rudderTrim = stream.readFloat(_rudderTrim, 0);
			}
			
			void onWrite(const NVSStream& stream) override {
				
				// Trim
				stream.writeFloat(_aileronsTrim, aileronsTrim);
				stream.writeFloat(_elevatorTrim, elevatorTrim);
				stream.writeFloat(_rudderTrim, rudderTrim);
			}
		
		private:
			constexpr static auto _configurations = "tr1";
			
			constexpr static auto _aileronsTrim = "ta";
			constexpr static auto _elevatorTrim = "te";
			constexpr static auto _rudderTrim = "tr";
	};
}