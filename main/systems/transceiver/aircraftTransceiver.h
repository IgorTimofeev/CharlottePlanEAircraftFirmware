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
		// -------------------------------- Main --------------------------------

		public:
			AircraftTransceiver();

		protected:
			void onTick() override;
			void onTransmit(BitStream& stream, AircraftPacketType packetType) override;
			bool onReceive(BitStream& stream, RemotePacketType packetType, uint8_t payloadLength) override;
			void onConnectionStateChanged() override;

		private:
			bool _receiveMode = true;
			int64_t _transmitTimeUs = 0;

		// -------------------------------- Communication settings --------------------------------

		private:
			TransceiverCommunicationSettings _receivedCommunicationSettings {};

			void onCommunicationSettingsSyncCheckScheduled() override;

			void onCommunicationSettingsSyncCheckCompleted() override;

			// -------------------------------- Receiving --------------------------------

		private:
			bool receiveRemoteSystemPacket(BitStream& stream, uint8_t payloadLength);


		// -------------------------------- Transmitting --------------------------------

		private:
			void transmitAircraftSystemPacket(BitStream& stream);
	};
}