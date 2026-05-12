#pragma once

#include <cstring>
#include <array>
#include <cmath>

#include <driver/gpio.h>
#include <esp_log.h>

#include "config.h"

namespace pizda {
	#pragma pack(push, 1)
	
	class SimLinkPacket {
		public:
			constexpr static uint32_t header = 0xAABBCCDD;
	};
	
	class SimLinkSimPacket {
		public:
			uint32_t header = SimLinkPacket::header;
			
			float accelerationX;
			
			float rollRad;
			float pitchRad;
			float yawRad;
			
			float latitudeRad;
			float longitudeRad;
			
			float speedMPS;
			
			float pressurePA;
			float temperatureC;
	};
	
	class SimLinkAircraftPacket {
		public:
			uint32_t header = SimLinkPacket::header;
			
			float throttle = 0;
			float ailerons = 0;
			float elevator = 0;
			float rudder = 0;
			float flaps = 0;
			uint8_t lights = 0;
	};
	
	#pragma pack(pop)
	
	class SimLink {
		public:
			void setup();
			
			const SimLinkSimPacket& getLastSimPacket() const;
		
		private:
			constexpr static uint16_t _bufferLength = 512;
			uint8_t _buffer[_bufferLength] {};
			
			SimLinkSimPacket _lastSimPacket {};
			
			[[noreturn]] void taskBody();
	};
}