#pragma once

#include <string>
#include <type_traits>
#include <span>

#include <nvs_flash.h>
#include <esp_log.h>
#include <sys/types.h>

namespace YOBA {
	class NVSStream {
		public:
			void openForWriting(const char* key) {
				initialize();

				ESP_ERROR_CHECK(nvs_open(key, NVS_READWRITE, &_handle));
			}

			void openForReading(const char* key) {
				initialize();

				const auto status = nvs_open(key, NVS_READONLY, &_handle);
				assert(status == ESP_OK || status == ESP_ERR_NVS_NOT_FOUND);
			}

			void commit() const {
				ESP_ERROR_CHECK(nvs_commit(_handle));
			}

			void close() const {
				nvs_close(_handle);
			}

			uint8_t readUint8(const char* key, const uint8_t defaultValue = 0) const {
				return readValue<uint8_t, uint8_t, nvs_get_u8>(key, defaultValue);
			}
			
			int8_t readInt8(const char* key, const int8_t defaultValue = 0) const {
				return readValue<int8_t, int8_t, nvs_get_i8>(key, defaultValue);
			}

			void writeUint8(const char* key, const uint8_t value) const {
				writeValue<uint8_t, nvs_set_u8>(key, value);
			}
			
			void writeInt8(const char* key, const int8_t value) const {
				writeValue<int8_t, nvs_set_i8>(key, value);
			}
			
			uint16_t readUint16(const char* key, const uint16_t defaultValue = 0) const {
				return readValue<uint16_t, uint16_t, nvs_get_u16>(key, defaultValue);
			}

			void writeUint16(const char* key, const uint16_t value) const {
				writeValue<uint16_t, nvs_set_u16>(key, value);
			}

			int16_t readInt16(const char* key, const int16_t defaultValue = 0) const {
				return readValue<int16_t, int16_t, nvs_get_i16>(key, defaultValue);
			}

			void writeInt16(const char* key, const int16_t value) const {
				writeValue<int16_t, nvs_set_i16>(key, value);
			}

			uint32_t readUint32(const char* key, const uint32_t defaultValue = 0) const {
				return readValue<uint32_t, uint32_t, nvs_get_u32>(key, defaultValue);
			}

			void writeUint32(const char* key, const uint32_t value) const {
				writeValue<uint32_t, nvs_set_u32>(key, value);
			}

			uint64_t readUint64(const char* key, const uint64_t defaultValue = 0) const {
				return readValue<uint64_t, uint64_t, nvs_get_u64>(key, defaultValue);
			}

			void writeUint64(const char* key, const uint64_t value) const {
				writeValue<uint64_t, nvs_set_u64>(key, value);
			}

			float readFloat(const char* key, const float defaultValue = 0) const {
				const auto size = readBlobSize(key);

				if (size != sizeof(float))
					return defaultValue;

				float result = 0;

				readBlob(
					key,
					std::span {
						reinterpret_cast<uint8_t*>(&result),
						size
					}
				);

				return result;
			}

			void writeFloat(const char* key, const float value) const {
				writeBlob(key, std::span<const uint8_t>(reinterpret_cast<const uint8_t*>(&value), sizeof(float)));
			}

			bool readBool(const char* key, const bool defaultValue = false) const {
				return readValue<bool, uint8_t, nvs_get_u8>(key, defaultValue);
			}

			void writeBool(const char* key, const bool value) const {
				writeValue<bool, nvs_set_u8>(key, value);
			}

			std::string readString(const char* key, const std::string& defaultValue = std::string()) const {
				return readStringT<char>(key, defaultValue);
			}

			void writeString(const char* key, const std::string& value) const {
				writeStringT<char>(key, value);
			}

			size_t readBlobSize(const char* key) const {
				size_t result = 0;
				nvs_get_blob(_handle, key, nullptr, &result);

				return result;
			}

			void readBlob(const char* key, const std::span<uint8_t> data) const {
				size_t sizeCopy = data.size_bytes();
				ESP_ERROR_CHECK(nvs_get_blob(_handle, key, data.data(), &sizeCopy));
			}

			void writeBlob(const char* key, const std::span<const uint8_t> data) const {
				ESP_ERROR_CHECK(nvs_set_blob(_handle, key, data.data(), data.size_bytes()));
			}

			void erase(const char* key) const {
				const auto status = nvs_erase_key(_handle, key);
				assert(status == ESP_OK || status == ESP_ERR_NVS_NOT_FOUND);
			}

			template<typename T>
			size_t readObjectSize(const char* key) const {
				return readBlobSize(key) / sizeof(T);
			}

			template<typename T>
			void readObject(const char* key, const std::span<T> data) const {
				readBlob(
					key,
					std::span {
						reinterpret_cast<uint8_t*>(data.data()),
						data.size_bytes()
					}
				);
			}

			template<typename T>
			void writeObject(const char* key, const std::span<const T> data) const {
				writeBlob(
					key,
					std::span {
						reinterpret_cast<const uint8_t*>(data.data()),
						data.size_bytes()
					}
				);
			}

			template<typename TEnum, typename = std::enable_if_t<std::is_enum_v<TEnum>>>
			TEnum readEnum(const char* key, const TEnum defaultValue) const {
				if constexpr (std::is_same_v<std::underlying_type_t<TEnum>, uint32_t>) {
					return static_cast<TEnum>(readUint32(key, static_cast<uint32_t>(defaultValue)));
				}
				else if constexpr (std::is_same_v<std::underlying_type_t<TEnum>, uint32_t>) {
					return static_cast<TEnum>(readUint16(key, static_cast<uint32_t>(defaultValue)));
				}

				return static_cast<TEnum>(readUint8(key, static_cast<uint8_t>(defaultValue)));
			}

			template<typename TEnum, typename = std::enable_if_t<std::is_enum_v<TEnum>>>
			void writeEnum(const char* key, const TEnum value) const {
				if constexpr (std::is_same_v<std::underlying_type_t<TEnum>, uint32_t>) {
					writeUint32(key, static_cast<uint32_t>(value));
				}
				else if constexpr (std::is_same_v<std::underlying_type_t<TEnum>, uint32_t>) {
					writeUint16(key, static_cast<uint32_t>(value));
				}

				writeUint8(key, static_cast<uint8_t>(value));
			}

			void testForBullshit() {
				ESP_LOGI("NVS test", "Writing");

				openForWriting("pizda");
				writeUint8("uint8Test", 123);
				writeUint16("uint16Test", 12345);
				writeUint32("uint32Test", 12345);
				writeFloat("floatTest", 123.456);
//				writeString("stringTest", "Pizda penisa");
				commit();
				close();

				ESP_LOGI("NVS test", "Reading");

				openForReading("pizda");
				ESP_LOGI("NVS test", "Value: %d", readUint8("uint8Test"));
				ESP_LOGI("NVS test", "Value: %d", readUint16("uint16Test"));
				ESP_LOGI("NVS test", "Value: %lu", readUint32("uint32Test"));
				ESP_LOGI("NVS test", "Value: %f", readFloat("floatTest"));
				ESP_LOGI("NVS test", "Value: %s", readString("stringTest").c_str());
				close();
			}

		private:
			nvs_handle_t _handle {};

			static void initialize() {
//				static bool initialized = false;
//
//				if (initialized)
//					return;
//
//				initialized = true;

				const auto status = nvs_flash_init();

				if (status == ESP_ERR_NVS_NO_FREE_PAGES || status == ESP_ERR_NVS_NEW_VERSION_FOUND) {
					// NVS partition was truncated and needs to be erased
					ESP_ERROR_CHECK(nvs_flash_erase());
					// Retry init
					ESP_ERROR_CHECK(nvs_flash_init());
				}
				else {
					ESP_ERROR_CHECK(status);
				}
			}

			template<typename TResult, typename TGet, auto Function>
			TResult readValue(const char* key, TResult defaultValue) const {
				TGet got;

				if (Function(_handle, key, &got) == ESP_OK)
					return static_cast<TResult>(got);

				return defaultValue;
			}

			template<typename TValue, auto Function>
			void writeValue(const char* key, TValue value) const {
				ESP_ERROR_CHECK(Function(_handle, key, value));
			}

			template<typename TChar>
			std::string readStringT(const char* key, const std::basic_string<TChar>& defaultValue = std::basic_string<TChar>()) const {
				size_t bufferSize = 0;

				if (nvs_get_blob(_handle, key, nullptr, &bufferSize) != ESP_OK)
					return defaultValue;

				auto str = std::string();
				str.resize(bufferSize);

				if (nvs_get_blob(_handle, key, str.data(), &bufferSize) != ESP_OK)
					return defaultValue;

				return str;
			}

			template<typename TChar>
			void writeStringT(const char* key, const std::basic_string<TChar>& value) const {
				ESP_ERROR_CHECK(nvs_set_blob(
					_handle,
					key,
					reinterpret_cast<const uint8_t*>(value.data()),
					(value.size() + 1) * sizeof(char)
				));
			}
	};
}
