#pragma once

#include <cstdint>

#include <NVSSettings.h>

#include "types/generic.h"

namespace pizda {
	using namespace YOBA;
	
	class LightsSettings : public NVSSettings {
		public:
			bool nav = false;
			bool strobe = false;
			bool landing = false;
			bool cabin = false;
			
		protected:
			const char* getNamespace() override {
				return _namespace;
			}
			
			void onRead(const NVSStream& stream) override {
				nav = stream.readBool(_nav, false);
				strobe = stream.readBool(_strobe, false);
				landing = stream.readBool(_landing, false);
				cabin = stream.readBool(_cabin, false);
			}
			
			void onWrite(const NVSStream& stream) override {
				stream.writeBool(_nav, nav);
				stream.writeBool(_strobe, strobe);
				stream.writeBool(_landing, landing);
				stream.writeBool(_cabin, cabin);
			}
		
		private:
			constexpr static const char* _namespace = "lt1";
			
			constexpr static const char* _nav = "n";
			constexpr static const char* _strobe = "s";
			constexpr static const char* _landing = "l";
			constexpr static const char* _cabin = "c";
	};
}