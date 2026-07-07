#pragma once

#include <cstdint>
#include <atomic>

#include <NVSSettings.hpp>

#include "Types/Generic.hpp"
#include "Utilities/Math.hpp"
#include "Vector3.hpp"

namespace pizda {
	using namespace YOBA;

	class FlightModeSelectionSettings : public NVSSettings {
		public:
			// Speed
			float getSelectedSpeedMPS() const {
				return _selectedSpeedMPS.load(std::memory_order_acquire);
			}

			void setSelectedSpeedMPS(const float value) {
				_selectedSpeedMPS.store(value, std::memory_order_release);
			}

			// Heading
			uint16_t getSelectedHeadingDeg() const {
				return _selectedHeadingDeg.load(std::memory_order_acquire);
			}

			void setSelectedHeadingDeg(const uint16_t value) {
				_selectedHeadingDeg.store(value, std::memory_order_release);
			}

			// Altitude
			float getSelectedAltitudeM() const {
				return _selectedAltitudeM.load(std::memory_order_acquire);
			}

			void setSelectedAltitudeM(const float value) {
				_selectedAltitudeM.store(value, std::memory_order_release);
			}

			// Pressure
			uint32_t getReferencePressurePa() const {
				return _referencePressurePa.load(std::memory_order_acquire);
			}

			void setReferencePressurePa(const uint32_t value) {
				_referencePressurePa.store(value, std::memory_order_release);
			}

		protected:
			const char* getNamespace() override {
				return _namespace;
			}

			void onRead(const NVSStream& stream) override {
				setSelectedSpeedMPS(stream.readFloat(_selectedSpeedMPSKey, 0.0f));
				setSelectedHeadingDeg(stream.readUint16(_selectedHeadingDegKey, 0));
				setSelectedAltitudeM(stream.readFloat(_selectedAltitudeMKey, 0.0f));

				setReferencePressurePa(stream.readUint32(_referencePressurePaKey, 101325));
			}

			void onWrite(const NVSStream& stream) override {
				stream.writeFloat(_selectedSpeedMPSKey, getSelectedSpeedMPS());
				stream.writeUint16(_selectedHeadingDegKey, getSelectedHeadingDeg());
				stream.writeFloat(_selectedAltitudeMKey, getSelectedAltitudeM());

				stream.writeUint32(_referencePressurePaKey, getReferencePressurePa());
			}

		private:
			constexpr static auto _namespace = "fms";

			constexpr static auto _selectedSpeedMPSKey = "sp";
			constexpr static auto _selectedHeadingDegKey = "hd";
			constexpr static auto _selectedAltitudeMKey = "as";

			constexpr static auto _referencePressurePaKey = "rpp";

			std::atomic<float> _selectedSpeedMPS { 0.0f };
			std::atomic<uint16_t> _selectedHeadingDeg { 0 };
			std::atomic<float> _selectedAltitudeM { 0.0f };

			std::atomic<uint32_t> _referencePressurePa { 0 };
	};
}