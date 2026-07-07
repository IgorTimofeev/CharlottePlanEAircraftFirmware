#pragma once

#include <array>
#include <span>
#include <atomic>

#include <NVSSettings.hpp>
#include <NVSStream.hpp>
#include <Vector3.hpp>

#include "Config.hpp"

namespace pizda {
	using namespace YOBA;
	
	class ADIRSSettings : public NVSSettings {
		public:
			// Accel
			Vector3F getAccelBias() const {
				return {
					_accelBiasX.load(std::memory_order_acquire),
					_accelBiasY.load(std::memory_order_acquire),
					_accelBiasZ.load(std::memory_order_acquire)
				};
			}
			
			void setAccelBias(const Vector3F& value) {
				_accelBiasX.store(value.getX(), std::memory_order_release);
				_accelBiasY.store(value.getY(), std::memory_order_release);
				_accelBiasZ.store(value.getZ(), std::memory_order_release);
			}
			
			// Gyro
			Vector3F getGyroBias() const {
				return {
					_gyroBiasX.load(std::memory_order_acquire),
					_gyroBiasY.load(std::memory_order_acquire),
					_gyroBiasZ.load(std::memory_order_acquire)
				};
			}
			
			void setGyroBias(const Vector3F& value) {
				_gyroBiasX.store(value.getX(), std::memory_order_release);
				_gyroBiasY.store(value.getY(), std::memory_order_release);
				_gyroBiasZ.store(value.getZ(), std::memory_order_release);
			}
			
			// Mag
			Vector3F getMagBias() const {
				return {
					_magBiasX.load(std::memory_order_acquire),
					_magBiasY.load(std::memory_order_acquire),
					_magBiasZ.load(std::memory_order_acquire)
				};
			}
			
			void setMagBias(const Vector3F& value) {
				_magBiasX.store(value.getX(), std::memory_order_release);
				_magBiasY.store(value.getY(), std::memory_order_release);
				_magBiasZ.store(value.getZ(), std::memory_order_release);
			}

			// Pressure
			uint32_t getReferencePressurePa() const {
				return _referencePressurePa.load(std::memory_order_acquire);
			}

			void setReferencePressurePa(const uint32_t value) {
				_referencePressurePa.store(value, std::memory_order_release);
			}

			// Mag dec
			int16_t getMagneticDeclinationDeg() const {
				return _magneticDeclinationDeg.load(std::memory_order_acquire);
			}

			void setMagneticDeclinationDeg(const int16_t value) {
				_magneticDeclinationDeg.store(value, std::memory_order_release);
			}

		protected:
			const char* getNamespace() override {
				return "adirs1";
			}

			void onRead(const NVSStream& stream) override {
				setAccelBias({
					stream.readFloat(_accelBiasXKey, 0),
					stream.readFloat(_accelBiasYKey, 0),
					stream.readFloat(_accelBiasZKey, 0)
				});

				setGyroBias({
					stream.readFloat(_gyroBiasXKey, 0),
					stream.readFloat(_gyroBiasYKey, 0),
					stream.readFloat(_gyroBiasZKey, 0)
				});

				setMagBias({
					stream.readFloat(_magBiasXKey, 0),
					stream.readFloat(_magBiasYKey, 0),
					stream.readFloat(_magBiasZKey, 0)
				});

				// ESP_LOGI("pizda", "acc bias: %f, %f, %f", getAccelBias().getX(), getAccelBias().getY(), getAccelBias().getZ());
				// ESP_LOGI("pizda", "gyr bias: %f, %f, %f", getGyroBias().getX(), getGyroBias().getY(), getGyroBias().getZ());
				// ESP_LOGI("pizda", "mag bias: %f, %f, %f", getMagBias().getX(), getMagBias().getY(), getMagBias().getZ());

				setReferencePressurePa(stream.readUint32(_referencePressurePaKey, 101325));
				setMagneticDeclinationDeg(stream.readInt16(_magneticDeclinationDegKey, 0));
			}

			void onWrite(const NVSStream& stream) override {
				auto bias = getAccelBias();
				stream.writeFloat(_accelBiasXKey, bias.getX());
				stream.writeFloat(_accelBiasYKey, bias.getY());
				stream.writeFloat(_accelBiasZKey, bias.getZ());

				bias = getGyroBias();
				stream.writeFloat(_gyroBiasXKey, bias.getX());
				stream.writeFloat(_gyroBiasYKey, bias.getY());
				stream.writeFloat(_gyroBiasZKey, bias.getZ());

				bias = getMagBias();
				stream.writeFloat(_magBiasXKey, bias.getX());
				stream.writeFloat(_magBiasYKey, bias.getY());
				stream.writeFloat(_magBiasZKey, bias.getZ());

				stream.writeUint32(_referencePressurePaKey, getReferencePressurePa());
				stream.writeInt16(_magneticDeclinationDegKey, getMagneticDeclinationDeg());
			}
			
		private:
			constexpr static auto _units = "un";
			
			constexpr static auto _accelBiasXKey = "abx";
			constexpr static auto _accelBiasYKey = "aby";
			constexpr static auto _accelBiasZKey = "abz";
			
			constexpr static auto _gyroBiasXKey = "gbx";
			constexpr static auto _gyroBiasYKey = "gby";
			constexpr static auto _gyroBiasZKey = "gbz";
			
			constexpr static auto _magBiasXKey = "mbx";
			constexpr static auto _magBiasYKey = "mby";
			constexpr static auto _magBiasZKey = "mbz";
			
			constexpr static auto _referencePressurePaKey = "rpp";
			constexpr static auto _magneticDeclinationDegKey = "mdd";

			std::atomic<float> _accelBiasX { 0 };
			std::atomic<float> _accelBiasY { 0 };
			std::atomic<float> _accelBiasZ { 0 };
			
			std::atomic<float> _gyroBiasX { 0 };
			std::atomic<float> _gyroBiasY { 0 };
			std::atomic<float> _gyroBiasZ { 0 };
			
			std::atomic<float> _magBiasX { 0 };
			std::atomic<float> _magBiasY { 0 };
			std::atomic<float> _magBiasZ { 0 };
			
			std::atomic<uint32_t> _referencePressurePa { 0 };
			std::atomic<int16_t> _magneticDeclinationDeg { 0 };
	};
}