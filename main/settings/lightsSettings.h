#pragma once

#include <cstdint>

#include <NVSSettings.h>

#include "types/generic.h"

namespace pizda {
	using namespace YOBA;
	
	class LightsSettings : public NVSSettings {
		public:
			bool getNav() const {
				return _nav.load(std::memory_order_acquire);
			}

			void setNav(const bool value) {
				_nav.store(value, std::memory_order_release);
			}

			bool getStrobe() const {
				return _strobe.load(std::memory_order_acquire);
			}

			void setStrobe(const bool value) {
				_strobe.store(value, std::memory_order_release);
			}

			bool getLanding() const {
				return _landing.load(std::memory_order_acquire);
			}

			void setLanding(const bool value) {
				_landing.store(value, std::memory_order_release);
			}

			bool getCabin() const {
				return _cabin.load(std::memory_order_acquire);
			}

			void setCabin(const bool value) {
				_cabin.store(value, std::memory_order_release);
			}
			
		protected:
			const char* getNamespace() override {
				return _namespace;
			}
			
			void onRead(const NVSStream& stream) override {
				setNav(stream.readBool(_navKey, false));
				setStrobe(stream.readBool(_strobeKey, false));
				setLanding(stream.readBool(_landingKey, false));
				setCabin(stream.readBool(_cabinKey, false));
			}
			
			void onWrite(const NVSStream& stream) override {
				stream.writeBool(_navKey, getNav());
				stream.writeBool(_strobeKey, getStrobe());
				stream.writeBool(_landingKey, getLanding());
				stream.writeBool(_cabinKey, getCabin());
			}
		
		private:
			constexpr static auto _namespace = "lt1";
			
			constexpr static auto _navKey = "n";
			constexpr static auto _strobeKey = "s";
			constexpr static auto _landingKey = "l";
			constexpr static auto _cabinKey = "c";

			std::atomic<bool> _nav { false };
			std::atomic<bool> _strobe { false };
			std::atomic<bool> _landing { false };
			std::atomic<bool> _cabin { false };
	};
}