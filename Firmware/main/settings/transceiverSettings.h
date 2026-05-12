#pragma once

#include <NVSSettings.h>
#include <SX1262.h>

#include "config.h"
#include "types/generic.h"

namespace pizda {
	using namespace YOBA;

	class TransceiverSettings : public NVSSettings {
		public:
			TransceiverCommunicationSettings communication {};

		protected:
			const char* getNamespace() override {
				return _namespace;
			}

			void onRead(const NVSStream& stream) override {
				communication.frequencyHz = stream.readUint32(_communicationRFFrequencyHz, config::transceiver::communicationSettings.frequencyHz);
				communication.bandwidth = static_cast<SX1262::LoRaBandwidth>(stream.readUint8(_communicationBandwidth, static_cast<uint8_t>(config::transceiver::communicationSettings.bandwidth)));
				communication.spreadingFactor = stream.readUint8(_communicationSpreadingFactor, config::transceiver::communicationSettings.spreadingFactor);
				communication.codingRate = static_cast<SX1262::LoRaCodingRate>(stream.readUint8(_communicationCodingRate, static_cast<uint8_t>(config::transceiver::communicationSettings.codingRate)));
				communication.syncWord = stream.readUint8(_communicationSyncWord, config::transceiver::communicationSettings.syncWord);
				communication.preambleLength = stream.readUint16(_communicationPreambleLength, config::transceiver::communicationSettings.preambleLength);

				communication.currentLimitMA = stream.readInt8(_communicationCurrentLimitMA, config::transceiver::communicationSettings.currentLimitMA);
				communication.powerDBm = stream.readInt8(_communicationPowerDBm, config::transceiver::communicationSettings.powerDBm);
			}

			void onWrite(const NVSStream& stream) override {
				stream.writeUint32(_communicationRFFrequencyHz, communication.frequencyHz);
				stream.writeUint8(_communicationBandwidth, static_cast<uint8_t>(communication.bandwidth));
				stream.writeUint8(_communicationSpreadingFactor, communication.spreadingFactor);
				stream.writeUint8(_communicationCodingRate, static_cast<uint8_t>(communication.codingRate));
				stream.writeUint8(_communicationSyncWord, communication.syncWord);
				stream.writeUint16(_communicationPreambleLength, communication.preambleLength);

				stream.writeInt8(_communicationCurrentLimitMA, communication.currentLimitMA);
				stream.writeInt8(_communicationPowerDBm, communication.powerDBm);
			}

		private:
			constexpr static auto _namespace = "trn";
			
			constexpr static auto _communicationRFFrequencyHz = "cmrf";
			constexpr static auto _communicationBandwidth = "cmbw";
			constexpr static auto _communicationSpreadingFactor = "cmsf";
			constexpr static auto _communicationCodingRate = "cmcr";
			constexpr static auto _communicationSyncWord = "cmsw";
			constexpr static auto _communicationPreambleLength = "cmpl";

			constexpr static auto _communicationCurrentLimitMA = "cmcl";
			constexpr static auto _communicationPowerDBm = "cmpw";
	};
}
