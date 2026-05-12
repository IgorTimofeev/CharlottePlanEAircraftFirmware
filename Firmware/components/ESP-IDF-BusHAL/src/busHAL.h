#pragma once

#include <bit>
#include <span>

#include <driver/i2c_master.h>
#include <esp_err.h>

namespace YOBA{
	class BusHAL {
		public:
			virtual ~BusHAL() = default;

			virtual bool write(const std::span<const uint8_t> data) = 0;
			virtual bool read(const std::span<uint8_t> data) = 0;

			virtual bool read(uint8_t reg, const std::span<uint8_t> data) = 0;
			virtual bool write(uint8_t reg, const std::span<const uint8_t> data) = 0;

			// 8
			bool writeUint8(const uint8_t reg, const uint8_t value) {
				const uint8_t buffer[2] {
					reg,
					value
				};

				return write({ buffer, 2 });
			}

			bool readUint8(const uint8_t reg, uint8_t& value) {
				return read(reg, { &value, 1 });
			}

			// 16 LE
			bool writeUint16LE(const uint8_t reg, uint16_t& value) {
				return writeUintLE<uint16_t>(reg, value);
			}

			bool readUint16LE(const uint8_t reg, uint16_t& value) {
				return readUintLE<uint16_t>(reg, value);
			}

			bool readInt16LE(const uint8_t reg, int16_t& value) {
				return readIntLE<uint16_t, int16_t>(reg, value);
			}

			// 16 BE
			bool writeUint16BE(const uint8_t reg, uint16_t& value) {
				return writeUintBE<uint16_t>(reg, value);
			}

			bool readUint16BE(const uint8_t reg, uint16_t& value) {
				return readUintBE<uint16_t>(reg, value);
			}

			bool readInt16BE(const uint8_t reg, int16_t& value) {
				return readIntBE<uint16_t, int16_t>(reg, value);
			}

			// 32 LE
			bool writeUint32LE(const uint8_t reg, uint32_t& value) {
				return writeUintLE<uint32_t>(reg, value);
			}

			bool readUint32LE(const uint8_t reg, uint32_t& value) {
				return readUintLE<uint32_t>(reg, value);
			}

			// 32 BE
			bool writeUint32BE(const uint8_t reg, uint32_t& value) {
				return writeUintBE<uint32_t>(reg, value);
			}

			bool readUint32BE(const uint8_t reg, uint32_t& value) {
				return readUintBE<uint32_t>(reg, value);
			}

			bool readInt32BE(const uint8_t reg, int32_t& value) {
				return readIntBE<uint32_t, int32_t>(reg, value);
			}
			
		protected:
			constexpr static auto _logTag = "BusStream";

		private:
			template<std::unsigned_integral T>
			bool writeUintLE(const uint8_t reg, T& value) {
				#pragma pack(push, 1)
					struct {
						uint8_t reg;
						T value;
					} data = {
						reg,
						value
					};
				#pragma pack(pop)

				return write({ reinterpret_cast<uint8_t*>(&data), sizeof(data) });
			}

			template<std::unsigned_integral T>
			bool readUintLE(const uint8_t reg, T& value) {
				return read(reg, { reinterpret_cast<uint8_t*>(&value), sizeof(T) });
			}

			template<std::unsigned_integral UT, std::signed_integral ST>
			bool readIntLE(const uint8_t reg, ST& value) {
				UT unsignedValue = 0;

				if (!readUintLE<UT>(reg, unsignedValue))
					return false;

				value = static_cast<ST>(unsignedValue);

				return true;
			}

			template<typename T>
			bool writeUintBE(const uint8_t reg, T& value) {
				#pragma pack(push, 1)
					struct {
						uint8_t reg;
						T value;
					} data = {
						reg,
						std::byteswap(value)
					};
				#pragma pack(pop)

				return write({ reinterpret_cast<uint8_t*>(&data), sizeof(data) });
			}

			template<std::unsigned_integral T>
			bool readUintBE(const uint8_t reg, T& value) {
				if (!readUintLE<T>(reg, value))
					return false;

				value = std::byteswap(value);

				return true;
			}

			template<std::unsigned_integral UT, std::signed_integral ST>
			bool readIntBE(const uint8_t reg, ST& value) {
				UT unsignedValue = 0;

				if (!readUintBE<UT>(reg, unsignedValue))
					return false;

				value = static_cast<ST>(unsignedValue);

				return true;
			}
	};

	class I2CBusHAL : public BusHAL {
		public:
			bool setup(const i2c_master_bus_handle_t& bus, const uint8_t address, const uint32_t clockSpeedHz, const i2c_addr_bit_len_t addressLength = I2C_ADDR_BIT_LEN_7) {
				i2c_device_config_t deviceConfig {};
				deviceConfig.dev_addr_length = addressLength;
				deviceConfig.device_address = address;
				deviceConfig.scl_speed_hz = clockSpeedHz;

				const auto state = i2c_master_bus_add_device(bus, &deviceConfig, &_device);
				ESP_ERROR_CHECK_WITHOUT_ABORT(state);

				return state == ESP_OK;
			}

			bool write(const std::span<const uint8_t> data) override {
				const auto state = i2c_master_transmit(_device, data.data(), data.size(), -1);
				ESP_ERROR_CHECK_WITHOUT_ABORT(state);

				return state == ESP_OK;
			}

			bool read(const std::span<uint8_t> data) override {
				const auto state = i2c_master_receive(_device, data.data(), data.size(), -1);
				ESP_ERROR_CHECK_WITHOUT_ABORT(state);

				return state == ESP_OK;
			}

			bool write(const uint8_t reg, const std::span<const uint8_t> data) override {
				if (!write({ &reg, 1 }))
					return false;

				return write({ data.data(), data.size() });
			}

			bool read(const uint8_t reg, const std::span<uint8_t> data) override {
				if (!write({ &reg, 1 }))
					return false;

				return read({ data.data(), data.size() });
			}

		private:
			i2c_master_dev_handle_t _device {};
	};
}
