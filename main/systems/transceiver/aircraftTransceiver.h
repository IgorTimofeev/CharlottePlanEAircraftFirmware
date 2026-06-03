#pragma once

#include <array>

#include "transceiver.h"

#include <bitStream.h>

namespace pizda {
	class AircraftTransceiver : public Transceiver<
		AircraftPacketType,
		AircraftPacket::typeLengthBits,

		7,
		AircraftSystemPacketType,

		RemotePacketType,
		RemotePacket::typeLengthBits,

		0
	> {
		public:
			AircraftTransceiver();

		protected:
			[[noreturn]] void onStart() override;
			void onTransmit(BitStream& stream, AircraftPacketType packetType) override;
			bool onReceive(BitStream& stream, RemotePacketType packetType, uint8_t payloadLength) override;
			bool receiveRemoteSystemPacket(BitStream& stream, uint8_t payloadLength);
			void onConnectionStateChanged() override;

		private:
			constexpr static uint32_t _trendsInterval = 500'000;
			int64_t _trendsTime = 0;
			float _trendsAirspeedPrevMPS = 0;
			float _trendsAltitudePrevM = 0;

			TransceiverCommunicationSettings _tmpCommunicationSettings {};
			int64_t _communicationSettingsACKTime = 0;

			bool receiveRemoteControlsPacket(BitStream& stream, uint8_t payloadLength);
			bool receiveRemoteSystemTrimPacket(BitStream& stream, uint8_t payloadLength);
			bool receiveRemoteSystemLightsPacket(BitStream& stream, uint8_t payloadLength);
			bool receiveRemoteSystemBaroPacket(BitStream& stream, uint8_t payloadLength);
			bool receiveRemoteSystemAutopilotPacket(BitStream& stream, uint8_t payloadLength);
			bool receiveRemoteSystemCameraPacket(BitStream& stream, uint8_t payloadLength);
			bool receiveRemoteSystemMotorsPacket(BitStream& stream, uint8_t payloadLength);
			bool receiveRemoteSystemADIRSPacket(BitStream& stream, uint8_t payloadLength);
			bool receiveRemoteSystemXCVRPacket(BitStream& stream, uint8_t payloadLength);
			bool receiveRemoteSystemCalibratePacket(BitStream& stream, uint8_t payloadLength);

			void transmitAircraftSTierTelemetryPacket(BitStream& stream);
			void transmitAircraftATierTelemetryPacket(BitStream& stream);
			void transmitAircraftBTierTelemetryPacket(BitStream& stream);

			void transmitAircraftSystemPacket(BitStream& stream);
			void transmitAircraftSystemCalibrationPacket(BitStream& stream);
			void transmitAircraftSystemCommunicationSettingsACKPacket(BitStream& stream);
	};
}