#pragma once

#include <array>

#include "transceiver.h"

#include <bitStream.h>

namespace pizda {
	class AircraftTransceiver : public Transceiver<
		AircraftPacketType,
		AircraftPacket::typeLengthBits,

		3,
		AircraftAuxiliaryPacketType,

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
			bool receiveRemoteAuxiliaryPacket(BitStream& stream, uint8_t payloadLength);
			void onConnectionStateChanged() override;

		private:
			constexpr static uint32_t _trendsInterval = 500'000;
			int64_t _trendsTime = 0;
			float _trendsAirspeedPrevMPS = 0;
			float _trendsAltitudePrevM = 0;

			TransceiverCommunicationSettings _tmpCommunicationSettings {};
			int64_t _communicationSettingsACKTime = 0;

			bool receiveRemoteControlsPacket(BitStream& stream, uint8_t payloadLength);
			bool receiveRemoteAuxiliaryTrimPacket(BitStream& stream, uint8_t payloadLength);
			bool receiveRemoteAuxiliaryLightsPacket(BitStream& stream, uint8_t payloadLength);
			bool receiveRemoteAuxiliaryBaroPacket(BitStream& stream, uint8_t payloadLength);
			bool receiveRemoteAuxiliaryAutopilotPacket(BitStream& stream, uint8_t payloadLength);
			bool receiveRemoteAuxiliaryMotorsPacket(BitStream& stream, uint8_t payloadLength);
			bool receiveRemoteAuxiliaryADIRSPacket(BitStream& stream, uint8_t payloadLength);
			bool receiveRemoteAuxiliaryXCVRPacket(BitStream& stream, uint8_t payloadLength);
			bool receiveRemoteAuxiliaryCalibratePacket(BitStream& stream, uint8_t payloadLength);

			void transmitAircraftTelemetryPrimaryPacket(BitStream& stream);
			void transmitAircraftTelemetrySecondaryPacket(BitStream& stream);
			void transmitAircraftAuxiliaryPacket(BitStream& stream);
			void transmitAircraftAuxiliaryCalibrationPacket(BitStream& stream);
			void transmitAircraftAuxiliaryXCVRACKPacket(BitStream& stream);
	};
}