#pragma once

#include <array>
#include <span>

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
			uint32_t referencePressurePa = 0;
			int16_t magneticDeclinationDeg = 0;

			std::array<ADIRSSettingsUnit, config::ADIRS::unitCount> units {};

		protected:
			const char* getNamespace() override {
				return "adirs1";
			}

			void onRead(const NVSStream& stream) override {
				referencePressurePa = stream.readUint32(_referencePressurePa, 101325);
				magneticDeclinationDeg = stream.readInt16(_magneticDeclinationDeg, 0);

				// Units
				{
					const auto readUnitCount = stream.readObjectSize<ADIRSSettingsUnit>(_units);

					if (readUnitCount <= 0)
						return;

					if (readUnitCount != config::ADIRS::unitCount) {
						ESP_LOGI("ADIRSSettings", "read units length (%d) != config length (%d)", readUnitCount, config::ADIRS::unitCount);
						return;
					}

					stream.readObject<ADIRSSettingsUnit>(_units, std::span { units.data(), readUnitCount });
				}
			}

			void onWrite(const NVSStream& stream) override {
				stream.writeUint32(_referencePressurePa, referencePressurePa);
				stream.writeInt16(_magneticDeclinationDeg, magneticDeclinationDeg);
				stream.writeObject<ADIRSSettingsUnit>(_units, units);
			}
			
		private:
			constexpr static auto _units = "un";
			constexpr static auto _referencePressurePa = "rp";
			constexpr static auto _magneticDeclinationDeg = "md";
	};
}