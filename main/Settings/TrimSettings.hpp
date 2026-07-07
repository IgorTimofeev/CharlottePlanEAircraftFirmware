#pragma once

#include <atomic>

#include <NVSSettings.hpp>

namespace pizda {
	using namespace YOBA;
	
	class TrimSettings : public NVSSettings {
		public:
			float getAileronsTrim() const {
				return _aileronsTrim.load(std::memory_order_acquire);
			}

			void setAileronsTrim(const float value) {
				_aileronsTrim.store(value, std::memory_order_release);
			}

			float getElevatorTrim() const {
				return _elevatorTrim.load(std::memory_order_acquire);
			}

			void setElevatorTrim(const float value) {
				_elevatorTrim.store(value, std::memory_order_release);
			}

			float getRudderTrim() const {
				return _rudderTrim.load(std::memory_order_acquire);
			}

			void setRudderTrim(const float value) {
				_rudderTrim.store(value, std::memory_order_release);
			}
		
		protected:
			const char* getNamespace() override {
				return "ct1";
			}
			
			void onRead(const NVSStream& stream) override {
				setAileronsTrim(stream.readFloat(_aileronsTrimKey, 0));
				setElevatorTrim(stream.readFloat(_elevatorTrimKey, 0));
				setRudderTrim(stream.readFloat(_rudderTrimKey, 0));
			}
			
			void onWrite(const NVSStream& stream) override {
				stream.writeFloat(_aileronsTrimKey, getAileronsTrim());
				stream.writeFloat(_elevatorTrimKey, getElevatorTrim());
				stream.writeFloat(_rudderTrimKey, getRudderTrim());
			}
		
		private:
			constexpr static auto _configurations = "tr1";
			
			constexpr static auto _aileronsTrimKey = "ta";
			constexpr static auto _elevatorTrimKey = "te";
			constexpr static auto _rudderTrimKey = "tr";

			// Pre-mapped to [-0.5; 0.5]
			std::atomic<float> _aileronsTrim { 0.0f };
			std::atomic<float> _elevatorTrim { 0.0f };
			std::atomic<float> _rudderTrim { 0.0f };
	};
}