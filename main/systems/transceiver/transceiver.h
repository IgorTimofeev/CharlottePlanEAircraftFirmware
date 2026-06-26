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

#include "config.h"
#include "utilities/math.h"

#include "config.h"
#include "types/generic.h"
#include "packet.h"

namespace pizda {
	using namespace YOBA;

	template<typename TLocalPacketType>
	class PacketSequenceItem {
		public:
			PacketSequenceItem(const TLocalPacketType type, const bool useEnqueued = false) : _type(type),  _useEnqueued(useEnqueued) {

			}

			TLocalPacketType getType() const {
				return _type;
			}

			bool useEnqueued() const {
				return _useEnqueued;
			}

		private:
			TLocalPacketType _type;
			bool _useEnqueued;
	};

	template<
		typename TLocalPacketType,
		uint8_t localPacketTypeLengthBits,

		uint8_t TLocalPacketSequenceLength,
		typename TLocalSystemPacketType,

		typename TRemotePacketType,
		uint8_t remotePacketTypeLengthBits,

		uint8_t CPUCoreID
	>
	class Transceiver {
		// ----------------------------- Main -----------------------------

		public:
			Transceiver(const std::array<PacketSequenceItem<TLocalPacketType>, TLocalPacketSequenceLength>& packetSequence) : _packetSequence(packetSequence) {
				_packetQueueMutex = xSemaphoreCreateMutex();
			}

			virtual ~Transceiver() = default;

			virtual bool setup() {
				const auto error = _SX.setup(
					config::XCVR::SS,
					config::XCVR::busy,
					config::XCVR::DIO1,
					config::XCVR::RST,

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

				if (error != SX1262Error::none) {
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
					CPUCoreID == 0 ? 20 : configMAX_PRIORITIES - 1,
					nullptr,
					CPUCoreID
				);

				return true;
			}

			[[noreturn]] void onStart() {
				while (true) {
					// Communication settings sync requested
					if (_communicationSettingsCheckTime < 0) {
						scheduleCommunicationSettingsSyncCheck();
					}
					// Communication settings sync check time reached
					else if (_communicationSettingsCheckTime > 0 && esp_timer_get_time() >= _communicationSettingsCheckTime) {
						performCommunicationSettingsSyncCheck();
					}

					// Receive / transmit operations will be performed here
					onTick();

					// Computing PPS
					if (esp_timer_get_time() >= _packetRateUpdateTime) {
						_packetRateRX = _packetRateRXNext;
						_packetRateTX = _packetRateTXNext;

						_packetRateRXNext = 0;
						_packetRateTXNext = 0;

						_packetRateUpdateTime = esp_timer_get_time() + 1'000'000;
					}
				}
			}

			bool receive(const uint32_t timeoutUs) {
				uint8_t receivedLength = 0;

				const auto error = _SX.receive(_SXBuffer, receivedLength, timeoutUs);

				if (error == SX1262Error::none) {
					//		ESP_LOGI(_logTag, "read length: %d", length);
					//
					//		for (int i = 0; i < length; ++i) {
					//			ESP_LOGI(_logTag, "read buffer[%d]: %d", i, _buffer[i]);
					//		}

					// Length check
					if (receivedLength >= 1) {
						const uint8_t payloadLength = receivedLength - Packet::checksumLengthBytes;

						BitStream stream { _SXBuffer };

						// Type
						const auto packetType = static_cast<TRemotePacketType>(stream.readUint8(remotePacketTypeLengthBits));

						// Reading
						if (onReceive(stream, packetType, payloadLength)) {
							_connectionLostTimeUs = esp_timer_get_time() + _connectionLostIntervalUs;

							switch (_connectionState) {
								case ConnectionState::initial: {
									setConnectionState(ConnectionState::connected);
									break;
								}
								case ConnectionState::disconnected: {
									setConnectionState(ConnectionState::reconnected);
									break;
								}
								default:
									break;
							}

							_packetRateRXNext++;

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

				if (
					(_connectionState == ConnectionState::connected || _connectionState == ConnectionState::reconnected)
					&& esp_timer_get_time() >= _connectionLostTimeUs
				) {
					setConnectionState(ConnectionState::disconnected);
				}

				return false;
			}

			bool transmit() {
				BitStream stream { _SXBuffer };

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

				// ESP_LOGI(_logTag, "write length: %d", length);
				//
				// for (int i = 0; i < length; ++i) {
				// 	ESP_LOGI(_logTag, "write buffer[%d]: %d", i, _buffer[i]);
				// }

				const auto error = _SX.transmit({ _SXBuffer, totalLength }, 500'000);

				if (error != SX1262Error::none) {
					logSXError("transmit error", error);
					return false;
				}

				_packetRateTXNext++;

				return true;
			}

			bool isConnected() const {
				return _connectionState == ConnectionState::connected || _connectionState == ConnectionState::reconnected;
			}

			void enqueueSystemPacket(TLocalSystemPacketType type) {
				xSemaphoreTake(_packetQueueMutex, portMAX_DELAY);

				if (_packetQueue.full()) {
					ESP_LOGW(_logTag, "packet queue is full");
				}
				else {
					_packetQueue.push(type);
				}

				xSemaphoreGive(_packetQueueMutex);
			}

		protected:
			virtual void onTick() = 0;
			virtual bool onReceive(BitStream& stream, TRemotePacketType packetType, uint8_t payloadLength) = 0;
			virtual void onTransmit(BitStream& stream, TLocalPacketType packetType) {}

		// ----------------------------- SX -----------------------------

		protected:
			constexpr static auto _logTag = "XCVR";

			SX1262 _SX {};

			constexpr static uint16_t _SXBufferLength = 255;
			uint8_t _SXBuffer[_SXBufferLength] {};

			static void logSXError(const char* key, const SX1262Error error) {
				// if (error == SX1262Error::timeout)
				// 	return;

				ESP_LOGE(_logTag, "%s: %s", key, SX1262::errorToString(error));
			}

		// ----------------------------- Connection state -----------------------------

		public:
			ConnectionState getConnectionState() const {
				return _connectionState;
			}

		protected:
			virtual void onConnectionStateChanged() = 0;

		private:
			constexpr static uint32_t _connectionLostIntervalUs = 5'000'000;
			int64_t _connectionLostTimeUs = 0;
			ConnectionState _connectionState = ConnectionState::initial;

			void setConnectionState(const ConnectionState state) {
				_connectionState = state;

				onConnectionStateChanged();
			}

		// ----------------------------- Communication settings -----------------------------

		protected:
			int32_t _receivingTimeOffsetUs = config::XCVR::communicationSettings.receivingTimeOffsetUs;
			int32_t _transmittingTimeOffsetUs = config::XCVR::communicationSettings.transmittingTimeOffsetUs;

			virtual void onCommunicationSettingsSyncCheckScheduled() = 0;
			virtual void onCommunicationSettingsSyncCheckCompleted() = 0;

			static bool checkSXErrorOnSetCommunicationSettings(const SX1262Error error) {
				if (error != SX1262Error::none) {
					logSXError("failed to set communication settings", error);
					return false;
				}

				return true;
			}

			bool setCommunicationSettings(const TransceiverCommunicationSettings& settings) {
				_receivingTimeOffsetUs = settings.receivingTimeOffsetUs;
				_transmittingTimeOffsetUs = settings.transmittingTimeOffsetUs;

				if (!checkSXErrorOnSetCommunicationSettings(
					_SX.setStandby()
				))
					return false;

				if (!checkSXErrorOnSetCommunicationSettings(
					_SX.setLoRaSyncWord(settings.syncWord)
				))
					return false;

				if (!checkSXErrorOnSetCommunicationSettings(
					_SX.setLoRaPreambleLength(settings.preambleLength)
				))
					return false;

				if (!checkSXErrorOnSetCommunicationSettings(
					_SX.setLoRaModulationParams(
						settings.spreadingFactor,
						settings.bandwidth,
						settings.codingRate,
						false
					)
				))
					return false;

				if (!checkSXErrorOnSetCommunicationSettings(
					_SX.setLoRaCADParams()
				))
					return false;

				if (!checkSXErrorOnSetCommunicationSettings(
					_SX.setRFFrequency(settings.frequencyHz)
				))
					return false;

				if (!checkSXErrorOnSetCommunicationSettings(
					_SX.setCurrentLimit(settings.currentLimitMA)
				))
					return false;

				if (!checkSXErrorOnSetCommunicationSettings(
					_SX.setOutputPower(settings.powerDBm)
				))
					return false;

				return true;
			}

			void requestCommunicationSettingsSyncCheck() {
				_communicationSettingsCheckTime = -1;
			}

		private:
			constexpr static uint8_t _communicationSettingsCheckMinPacketRate = 10;
			constexpr static uint32_t _communicationSettingsCheckDelayUs = 2'000'000;
			int64_t _communicationSettingsCheckTime = 0;

			void scheduleCommunicationSettingsSyncCheck() {
				onCommunicationSettingsSyncCheckScheduled();

				_communicationSettingsCheckTime = esp_timer_get_time() + _communicationSettingsCheckDelayUs;
			}

			void performCommunicationSettingsSyncCheck() {
				// Received and decoded enough packets to consider the connection is stable
				if (getRXPacketRate() >= _communicationSettingsCheckMinPacketRate) {
					ESP_LOGI(_logTag, "communication settings synchronized");

					onCommunicationSettingsSyncCheckCompleted();

					_communicationSettingsCheckTime = esp_timer_get_time() + _communicationSettingsCheckDelayUs;
				}
				// Or not enough...
				else {
					ESP_LOGI(_logTag, "communication settings change timed out with PPS = %d, falling back to default", getRXPacketRate());

					// Falling back to default communication settings
					setCommunicationSettings(config::XCVR::communicationSettings);
				}

				_communicationSettingsCheckTime = 0;
			}

		// ----------------------------- PPS -----------------------------

		public:
			uint16_t getRXPacketRate() const {
				return _packetRateRX;
			}

			uint16_t getTXPacketRate() const {
				return _packetRateTX;
			}

		private:
			int64_t _packetRateUpdateTime = 0;
			uint16_t _packetRateRXNext = 0;
			uint16_t _packetRateTXNext = 0;
			uint16_t _packetRateRX = 0;
			uint16_t _packetRateTX = 0;

		// ----------------------------- Packet queue -----------------------------

		protected:
			virtual TLocalPacketType getTransmitPacketType() {
				const auto& item = _packetSequence[_packetSequenceIndex];

				const auto nextSequenceItem = [this] {
					_packetSequenceIndex++;

					if (_packetSequenceIndex >= _packetSequence.size())
						_packetSequenceIndex = 0;
				};

				// Enqueued
				if (item.useEnqueued() && !_packetQueue.empty()) {
					xSemaphoreTake(_packetQueueMutex, portMAX_DELAY);

					_packetQueueSystemType = _packetQueue.front();
					_packetQueue.pop();

					xSemaphoreGive(_packetQueueMutex);

					nextSequenceItem();

					return TLocalPacketType::system;
				}

				// Normal
				const auto packetType = item.getType();

				nextSequenceItem();

				return packetType;
			}

			virtual TLocalSystemPacketType getTransmitSystemPacketType() {
				return _packetQueueSystemType;
			}

		private:
			std::array<PacketSequenceItem<TLocalPacketType>, TLocalPacketSequenceLength> _packetSequence;

			uint8_t _packetSequenceIndex = 0;

			CircularQueue<TLocalSystemPacketType, 255> _packetQueue {};
			SemaphoreHandle_t _packetQueueMutex;
			TLocalSystemPacketType _packetQueueSystemType {};



		// ----------------------------- Validation -----------------------------

		protected:
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

		// ----------------------------- Reading / writing -----------------------------

		protected:
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

			static float readLatitude(BitStream& stream, const uint8_t lengthBits) {
				// [0.0; 1.0]
				const auto latFactor =
					static_cast<float>(stream.readUint32(lengthBits))
					/ static_cast<float>((1 << lengthBits) - 1);

				// [-pi / 2; pi / 2]
				return latFactor * std::numbers::pi_v<float> - std::numbers::pi_v<float> / 2.f;
			}

			static void writeLatitude(BitStream& stream, const float latRad, const uint8_t lengthBits) {
				// Mapping from [-90; 90] to [0; 180] and then to [0; 1]
				const auto latFactor = (latRad + std::numbers::pi_v<float> / 2.f) / std::numbers::pi_v<float>;
				const auto latValue = static_cast<uint32_t>(static_cast<float>((1 << lengthBits) - 1) * latFactor);

				stream.writeUint32(latValue, lengthBits);
			}

			static float readLongitude(BitStream& stream, const uint8_t lengthBits) {
				// [0.0; 1.0]
				const auto lonFactor =
					static_cast<float>(stream.readUint32(lengthBits))
					/ static_cast<float>((1 << lengthBits) - 1);

				// [-pi; pi]
				return lonFactor * std::numbers::pi_v<float> * 2;
			}

			static void writeLongitude(BitStream& stream, const float lonRad, const uint8_t lengthBits) {
				// Mapping from [0; 360] to [0; 1]
				const auto lonFactor = lonRad / (2 * std::numbers::pi_v<float>);
				const auto lonValue = static_cast<uint32_t>(static_cast<float>((1 << lengthBits) - 1) * lonFactor);

				stream.writeUint32(lonValue, lengthBits);
			}
	};
}