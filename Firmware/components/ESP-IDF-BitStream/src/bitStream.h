#pragma once

#include <cmath>
#include <algorithm>
#include <bit>

namespace YOBA {
	class BitStream {
		public:
			explicit BitStream(uint8_t* buffer, const size_t bitIndex = 0) :
				_buffer(buffer)
			{
				setBitIndex(bitIndex);
			}

			uint8_t* getBuffer() const {
				return _buffer;
			}

			size_t getBitIndex() const {
				return _bitIndex;
			}
			
			void setBitIndex(const size_t value) {
				_bitIndex = value;
				
				updateByteIndicesFromBitIndex();
			}

			size_t getByteIndex() const {
				return _byteIndex;
			}
			
			size_t getBytesProcessed() const {
				return _byteBitIndex == 0 ? _byteIndex : _byteIndex + 1;
			}
			
			bool readBool() {
				const auto result = (_buffer[_byteIndex] >> _byteBitIndex) & 0b1;
				nextBit();
				
				return result;
			}
			
			uint8_t readUint8(const uint8_t bits = 8) {
				return readUnsigned<uint8_t>(bits);
			}
			
			int16_t readInt8(const uint8_t bits = 8) {
				return readSigned<int8_t, uint8_t>(bits);
			}
			
			uint16_t readUint16(const uint8_t bits = 16) {
				return readUnsigned<uint16_t>(bits);
			}
			
			int16_t readInt16(const uint8_t bits = 16) {
				return readSigned<int16_t, uint16_t>(bits);
			}
			
			int32_t readInt32(const uint8_t bits = 32) {
				return readSigned<int32_t, uint32_t>(bits);
			}
			
			uint32_t readUint32(const uint8_t bits = 32) {
				return readUnsigned<uint32_t>(bits);
			}
			
			float readFloat(const uint8_t bits = 32) {
				return std::bit_cast<float>(readUint32(bits));
			}
			
			void writeBool(const bool value) {
				writeBit(value);
			}
			
			void writeUint8(const uint8_t value, const uint8_t bits = 8) {
				writeUnsigned<uint8_t>(value, bits);
			}
			
			void writeInt8(const int8_t value, const uint8_t bits = 8) {
				writeSigned<int8_t, uint8_t>(value, bits);
			}
			
			void writeUint16(const uint16_t value, const uint8_t bits = 16) {
				writeUnsigned<uint16_t>(value, bits);
			}
			
			void writeInt16(const int16_t value, const uint8_t bits = 16) {
				writeSigned<int16_t, uint16_t>(value, bits);
			}
			
			void writeUint32(const uint32_t value, const uint8_t bits = 32) {
				writeUnsigned<uint32_t>(value, bits);
			}
			
			void writeInt32(const int32_t value, const uint8_t bits = 32) {
				writeSigned<int32_t, uint32_t>(value, bits);
			}
			
			void writeFloat(const float value, const uint8_t bits = 32) {
				writeUint32(std::bit_cast<uint32_t>(value), bits);
			}
			
			void nextByte() {
				_bitIndex += 8 - _byteBitIndex;
				_byteIndex++;
				_byteBitIndex = 0;
			}
			
			void finishByte() {
				if (_byteBitIndex == 0)
					return;
				
				nextByte();
			}
		
		private:
			uint8_t* _buffer;
			
			size_t _bitIndex = 0;
			size_t _byteIndex = 0;
			uint8_t _byteBitIndex = 0;
			
			void updateByteIndicesFromBitIndex() {
				_byteIndex = _bitIndex / 8;
				_byteBitIndex = _bitIndex % 8;
			}
			
			void nextBit() {
				_bitIndex++;
				_byteBitIndex++;
				
				if (_byteBitIndex >= 8) {
					_byteIndex++;
					_byteBitIndex = 0;
				}
			}
			
			template<typename TNumber>
			TNumber readUnsigned(const uint8_t bits) {
				TNumber result = 0;
				
				for (int i = 0; i < bits; ++i) {
					result |= (((_buffer[_byteIndex] >> _byteBitIndex) & 0b1) << i);
					nextBit();
				}
				
				return result;
			}
			
			template<std::signed_integral TSigned, std::unsigned_integral TUnsigned>
			TSigned readSigned(const uint8_t bits) {
				// Magnitude
				TUnsigned magnitude = 0;
				
				for (int i = 0; i < bits - 1; ++i) {
					magnitude |= (((_buffer[_byteIndex] >> _byteBitIndex) & 0b1) << i);
					nextBit();
				}
				
				// Sign
				const auto sign = ((_buffer[_byteIndex] >> _byteBitIndex) & 0b1) == 1;
				nextBit();
				
				return sign ? -static_cast<TSigned>(magnitude) : static_cast<TSigned>(magnitude);
			}
			
			void writeBit(const bool value) {
				_buffer[_byteIndex] =
					value
					? static_cast<uint8_t>(_buffer[_byteIndex] | (1 << _byteBitIndex))
					: static_cast<uint8_t>(_buffer[_byteIndex] & ~(1 << _byteBitIndex));
				
				nextBit();
			}
			
			template<std::unsigned_integral TNumber>
			void writeUnsigned(TNumber value, const uint8_t bits) {
				for (uint8_t i = 0; i < bits; ++i) {
					writeBit(((value >> i) & 0b1) == 1);
				}
			}
			
			template<std::signed_integral TSigned, std::unsigned_integral TUnsigned>
			void writeSigned(TSigned value, const uint8_t bits) {
				const auto unsignedValue = static_cast<TUnsigned>(std::abs(value));
				
				// Magnitude
				for (uint8_t i = 0; i < bits - 1; ++i) {
					writeBit(((unsignedValue >> i) & 0b1) == 1);
				}
				
				// Sign
				writeBit(value < 0);
			}
	};
}