#pragma once

#include <array>
#include <span>
#include <atomic>

#include <NVSSettings.h>
#include <NVSStream.h>
#include <vector3.h>

#include "config.h"

namespace pizda {
	using namespace YOBA;
	
	class ADIRSSettingsUnit {
		public:
			Vector3F accelBias {};
			Vector3F gyroBias {};
			Vector3F magBias {};
	};
	
	class ADIRSSettings : public NVSSettings {
		public:
			uint32_t getReferencePressurePa() const {
				return _referencePressurePa.load(std::memory_order_acquire);
			}

			void setReferencePressurePa(const uint32_t value) {
				_referencePressurePa.store(value, std::memory_order_release);
			}

			int16_t getMagneticDeclinationDeg() const {
				return _magneticDeclinationDeg.load(std::memory_order_acquire);
			}

			void setMagneticDeclinationDeg(const int16_t value) {
				_magneticDeclinationDeg.store(value, std::memory_order_release);
			}

			std::array<ADIRSSettingsUnit, config::ADIRS::unitCount> units {};

		protected:
			const char* getNamespace() override {
				return "adirs1";
			}

			void onRead(const NVSStream& stream) override {
				setReferencePressurePa(stream.readUint32(_referencePressurePaKey, 101325));
				setMagneticDeclinationDeg(stream.readInt16(_magneticDeclinationDegKey, 0));

				// Units
				{
					const auto readUnitCount = stream.readObjectSize<ADIRSSettingsUnit>(_units);

					if (readUnitCount == config::ADIRS::unitCount) {
						stream.readObject<ADIRSSettingsUnit>(_units, std::span { units.data(), readUnitCount });
					}
					else {
						ESP_LOGI("ADIRSSettings", "read units length (%d) != config length (%d)", readUnitCount, config::ADIRS::unitCount);
					}
				}
			}

			void onWrite(const NVSStream& stream) override {
				stream.writeUint32(_referencePressurePaKey, getReferencePressurePa());
				stream.writeInt16(_magneticDeclinationDegKey, getMagneticDeclinationDeg());

				stream.writeObject<ADIRSSettingsUnit>(_units, units);
			}
			
		private:
			constexpr static auto _units = "un";
			constexpr static auto _referencePressurePaKey = "rp";
			constexpr static auto _magneticDeclinationDegKey = "md";

			std::atomic<uint32_t> _referencePressurePa { 0 };
			std::atomic<int16_t> _magneticDeclinationDeg { 0 };
	};
}