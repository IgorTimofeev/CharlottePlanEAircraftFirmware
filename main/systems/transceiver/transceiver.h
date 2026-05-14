#pragma once

#include <cstdint>
#include <utility>
#include <cmath>
#include <array>
#include <span>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_timer.h>

#include <SX1262.h>
#include <bitStream.h>
#include <circularQueue.h>

#include "utilities/math.h"

#include "config.h"
#include "types/generic.h"
#include "packet.h"

namespace pizda {
	using namespace YOBA;

	template<typename TLocalPacketType>
	class PacketSequenceItem {
		public:
			PacketSequenceItem(const TLocalPacketType type, const uint8_t count, const bool useEnqueued = false) : _type(type), _count(count), _useEnqueued(useEnqueued) {

			}

			TLocalPacketType getType() const {
				return _type;
			}

			uint8_t getCount() const {
				return _count;
			}

			bool useEnqueued() const {
				return _useEnqueued;
			}

		private:
			TLocalPacketType _type;
			uint8_t _count;
			bool _useEnqueued;
	};

	template<
		typename TLocalPacketType,
		uint8_t localPacketTypeLengthBits,

		uint8_t TLocalPacketSequenceLength,
		typename TLocalAuxiliaryPacketType,

		typename TRemotePacketType,
		uint8_t remotePacketTypeLengthBits,

		uint8_t CPUCoreID
	>
	class Transceiver {
		public:
			Transceiver(const std::array<PacketSequenceItem<TLocalPacketType>, TLocalPacketSequenceLength>& packetSequence) : _packetSequence(packetSequence) {
				_packetQueueMutex = xSemaphoreCreateMutex();
			}

			virtual ~Transceiver() = default;

			virtual bool setup() {
				const auto error = _SX.setup(
					config::SPI::device,
					config::XCVR::SPIFrequencyHz,

					config::XCVR::communicationSettings.frequencyHz,
					config::XCVR::communicationSettings.bandwidth,
					config::XCVR::communicationSettings.spreadingFactor,
					config::XCVR::communicationSettings.codingRate,
					config::XCVR::communicationSettings.syncWord,
					config::XCVR::communicationSettings.preambleLength,

					config::XCVR::communicationSettings.currentLimitMA,
					config::XCVR::communicationSettings.powerDBm,
					false
				);

				if (error != SX1262::error::none) {
					ESP_LOGE(_logTag, "SX1262 setup failed with code %d", std::to_underlying(error));
					return false;
				}

				xTaskCreatePinnedToCore(
					[](void* arg) {
						static_cast<Transceiver*>(arg)->onStart();
					},
					"XCVR",
					8 * 1024,
					this,
					configMAX_PRIORITIES - 1,
					nullptr,
					CPUCoreID
				);

				return true;
			}

			bool receive(const uint32_t timeoutUs) {
				uint8_t receivedLength = 0;

				const auto error = _SX.receive(_buffer, receivedLength, timeoutUs);

				if (error == SX1262::error::none) {
					//		ESP_LOGI(_logTag, "read length: %d", length);
					//
					//		for (int i = 0; i < length; ++i) {
					//			ESP_LOGI(_logTag, "read buffer[%d]: %d", i, _buffer[i]);
					//		}

					// Length check
					if (receivedLength >= 1) {
						const uint8_t payloadLength = receivedLength - Packet::checksumLengthBytes;

						BitStream stream { _buffer };

						// Type
						const auto packetType = static_cast<TRemotePacketType>(stream.readUint8(remotePacketTypeLengthBits));

						// Reading
						if (onReceive(stream, packetType, payloadLength)) {
							_connectionLostTimeUs = esp_timer_get_time() + _connectionLostIntervalUs;

							if (_connectionState != ConnectionState::connected)
								setConnectionState(ConnectionState::connected);

							_RXPPSTemp++;

							return true;
						}
					}
					else {
						ESP_LOGE(_logTag, "failed to receive packet: length %d is too small to fit any data", receivedLength);
					}
				}
				else {
					logSXError("receive error", error);
				}

				if (_connectionState == ConnectionState::connected && esp_timer_get_time() >= _connectionLostTimeUs)
					setConnectionState(ConnectionState::disconnected);

				return false;
			}

			bool transmit(const uint32_t timeoutUs) {
				BitStream stream { _buffer };

				const auto packetType = getTransmitPacketType();

				// Type
				stream.writeUint8(static_cast<uint8_t>(packetType), localPacketTypeLengthBits);

				// Body
				onTransmit(stream, packetType);

				// Checksum
				const auto payloadLength = stream.getBytesProcessed();
				const auto checksum = getCRC8({ stream.getBuffer(), payloadLength });
				stream.finishByte();
				stream.writeUint8(checksum, Packet::checksumLengthBytes * 8);

				const auto totalLength = static_cast<uint8_t>(payloadLength + Packet::checksumLengthBytes);

				// Transmitting

				//		ESP_LOGI(_logTag, "write length: %d", length);
				//
				//		for (int i = 0; i < length; ++i) {
				//			ESP_LOGI(_logTag, "write buffer[%d]: %d", i, _buffer[i]);
				//		}

				const auto error = _SX.transmit({ _buffer, totalLength }, timeoutUs);

				if (error != SX1262::error::none) {
					logSXError("transmit error", error);
					return false;
				}

				_TXPPSTemp++;

				return true;
			}

			ConnectionState getConnectionState() const {
				return _connectionState;
			}

			bool isConnected() const {
				return _connectionState == ConnectionState::connected;
			}

			void enqueueAuxiliary(TLocalAuxiliaryPacketType type) {
				xSemaphoreTake(_packetQueueMutex, portMAX_DELAY);

				if (_packetQueue.full()) {
					ESP_LOGW(_logTag, "packet queue is full");
				}
				else {
					_packetQueue.push(type);
				}

				xSemaphoreGive(_packetQueueMutex);
			}

			uint16_t getRXPPS() const {
				return _RXPPS;
			}

			uint16_t getTXPPS() const {
				return _TXPPS;
			}

		protected:
			constexpr static auto _logTag = "XCVR";

			SX1262::Transceiver _SX {
				config::XCVR::SS,
				config::XCVR::busy,
				config::XCVR::DIO1,
				config::XCVR::RST
			};

			constexpr static uint16_t _bufferLength = 255;
			uint8_t _buffer[_bufferLength] {};

			static void logSXError(const char* key, const SX1262::error error) {
				// if (error == SX1262::error::timeout)
				// 	return;

				constexpr static uint8_t errorBufferLength = 255;
				char errorBuffer[errorBufferLength];

				SX1262::errorToString(error, { errorBuffer, errorBufferLength });

				ESP_LOGE(_logTag, "%s: %s", key, errorBuffer);
			}

			static bool checkSetCommunicationSettingsSXError(const SX1262::error error) {
				if (error != SX1262::error::none) {
					logSXError("failed to set communication settings", error);
					return false;
				}

				return true;
			}

			bool setCommunicationSettings(const TransceiverCommunicationSettings& settings) {
				if (!checkSetCommunicationSettingsSXError(
					_SX.setStandby()
				))
					return false;

				if (!checkSetCommunicationSettingsSXError(
					_SX.setLoRaSyncWord(settings.syncWord)
				))
					return false;

				if (!checkSetCommunicationSettingsSXError(
					_SX.setLoRaPreambleLength(settings.preambleLength)
				))
					return false;

				if (!checkSetCommunicationSettingsSXError(
					_SX.setLoRaModulationParams(
						settings.spreadingFactor,
						settings.bandwidth,
						settings.codingRate,
						false
					)
				))
					return false;

				if (!checkSetCommunicationSettingsSXError(
					_SX.setLoRaCADParams()
				))
					return false;

				if (!checkSetCommunicationSettingsSXError(
					_SX.setRFFrequency(settings.frequencyHz)
				))
					return false;

				if (!checkSetCommunicationSettingsSXError(
					_SX.setCurrentLimit(settings.currentLimitMA)
				))
					return false;

				if (!checkSetCommunicationSettingsSXError(
					_SX.setOutputPower(settings.powerDBm)
				))
					return false;

				return true;
			}

			static uint8_t getCRC8(const std::span<uint8_t> data) {
				uint8_t crc = 0xff;

				for (size_t i = 0; i < data.size(); i++) {
					crc ^= data.data()[i];

					for (size_t j = 0; j < 8; j++) {
						if ((crc & 0x80) != 0)
							crc = static_cast<uint8_t>((crc << 1) ^ 0x31);
						else
							crc <<= 1;
					}
				}

				return crc;
			}

			TLocalAuxiliaryPacketType getEnqueuedAuxiliaryPacketType() const {
				return _packetQueueAuxiliaryType;
			}

			static bool validatePayloadChecksumAndLength(BitStream& stream, size_t expectedDataLengthBits, uint8_t payloadLengthBytes) {
				const auto expectedPayloadLengthBytes = static_cast<uint8_t>(divideCeiling<uint16_t>(
					static_cast<uint16_t>(remotePacketTypeLengthBits) + expectedDataLengthBits,
					8
				));

				// Length
				if (payloadLengthBytes != expectedPayloadLengthBytes) {
					ESP_LOGE(_logTag, "payload length mismatch: got %d, expected %d", payloadLengthBytes, expectedPayloadLengthBytes);

					return false;
				}

				const auto checksum = getCRC8({ stream.getBuffer(), expectedPayloadLengthBytes });
				const auto expectedChecksum = *(stream.getBuffer() + expectedPayloadLengthBytes);

				// Checksum
				if (checksum != expectedChecksum) {
					ESP_LOGE(_logTag, "checksum mismatch: got %d, expected %d", checksum, expectedChecksum);

					return false;
				}

				return true;
			}

			void PPSTick() {
				if (esp_timer_get_time() < _PPSTime)
					return;

				_RXPPS = _RXPPSTemp;
				_TXPPS = _TXPPSTemp;

				_RXPPSTemp = 0;
				_TXPPSTemp = 0;

				_PPSTime = esp_timer_get_time() + 1'000'000;
			}

			template<typename T>
			static float sanitizeValue(T value, T min, T max) {
				if (value < min) {
					ESP_LOGW(_logTag, "value %f is out of range [%f, %f]", static_cast<float>(value), static_cast<float>(min), static_cast<float>(max));

					value = min;

				}
				else if (value > max) {
					ESP_LOGW(_logTag, "value %f is out of range [%f, %f]", static_cast<float>(value), static_cast<float>(min), static_cast<float>(max));

					value = max;
				}

				return value;
			}

			static float readRadians(BitStream& stream, const float range, const uint8_t bits) {
				auto value = static_cast<float>(stream.readUint16(bits)) / static_cast<float>((1 << bits) - 1);
				value = value - 0.5f;
				value = value * range;

				return sanitizeValue<float>(value, toRadians(-180), toRadians(180));
			}

			static void writeRadians(BitStream& stream, const float value, const float range, const uint8_t bits) {
				const auto uintValue = static_cast<uint16_t>((value / range + 0.5f) * ((1 << bits) - 1));

				stream.writeUint16(uintValue, bits);
			}

			static int16_t readAltitude(BitStream& stream, const uint8_t lengthBits, const int16_t min, const int16_t max) {
				const auto factor =
					static_cast<float>(stream.readUint16(lengthBits))
					/ static_cast<float>((1 << lengthBits) - 1);

				return min + (max - min) * factor;
			}

			static void writeAltitude(BitStream& stream, const float valueM, const uint8_t lengthBits, const int16_t min, const int16_t max) {
				const auto factor =
					(std::clamp<float>(valueM, min, max) - static_cast<float>(min))
					/ static_cast<float>(max - min);

				stream.writeUint16(
					static_cast<uint16_t>(std::round(factor * static_cast<float>((1 << lengthBits) - 1))),
					lengthBits
				);
			}

			virtual void onStart() = 0;
			virtual bool onReceive(BitStream& stream, TRemotePacketType packetType, uint8_t payloadLength) = 0;

			virtual void onTransmit(BitStream& stream, TLocalPacketType packetType) {}
			virtual void onConnectionStateChanged() = 0;

		private:
			constexpr static uint8_t _desiredTickFrequencyHz = 30;
			constexpr static uint32_t _desiredTickDurationUs = 1'000'000 / _desiredTickFrequencyHz;
			constexpr static uint32_t _safetyMarginDurationUs = 2'000;

			constexpr static uint32_t _desiredTXDurationUs = (_desiredTickDurationUs - _safetyMarginDurationUs) / 2;
			constexpr static uint32_t _desiredRXDurationUs = _desiredTickDurationUs - _desiredTXDurationUs - _safetyMarginDurationUs;

			int64_t _PPSTime = 0;
			uint16_t _RXPPSTemp = 0;
			uint16_t _TXPPSTemp = 0;
			uint16_t _RXPPS = 0;
			uint16_t _TXPPS = 0;

			// ----------------------------- Packet queue -----------------------------

			std::array<PacketSequenceItem<TLocalPacketType>, TLocalPacketSequenceLength> _packetSequence;

			uint8_t _packetSequenceIndex = 0;
			uint8_t _packetSequenceItemCounter = 0;

			CircularQueue<TLocalAuxiliaryPacketType, 255> _packetQueue {};
			SemaphoreHandle_t _packetQueueMutex;
			TLocalAuxiliaryPacketType _packetQueueAuxiliaryType {};

			TLocalPacketType getTransmitPacketType() {
				const auto& item = _packetSequence[_packetSequenceIndex];

				const auto nextSequenceItem = [this, &item] {
					_packetSequenceItemCounter++;

					if (_packetSequenceItemCounter < item.getCount())
						return;

					_packetSequenceItemCounter = 0;

					_packetSequenceIndex++;

					if (_packetSequenceIndex >= _packetSequence.size())
						_packetSequenceIndex = 0;
				};

				// Enqueued
				if (item.useEnqueued() && !_packetQueue.empty()) {
					xSemaphoreTake(_packetQueueMutex, portMAX_DELAY);

					_packetQueueAuxiliaryType = _packetQueue.front();
					_packetQueue.pop();

					xSemaphoreGive(_packetQueueMutex);

					nextSequenceItem();

					return TLocalPacketType::auxiliary;
				}

				// Normal
				const auto packetType = item.getType();

				nextSequenceItem();

				return packetType;
			}

			// ----------------------------- Connection state -----------------------------

			constexpr static uint32_t _connectionLostIntervalUs = 5'000'000;
			int64_t _connectionLostTimeUs = 0;
			ConnectionState _connectionState = ConnectionState::initial;

			void setConnectionState(const ConnectionState state) {
				_connectionState = state;

				onConnectionStateChanged();
			}
	};
}