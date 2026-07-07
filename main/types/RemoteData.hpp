#pragma once

#include <cmath>
#include <atomic>

#include "Types/Generic.hpp"

namespace pizda {
	class RemoteDataRawControls {
		public:
			float getThrottle() const {
				return _throttle.load(std::memory_order_acquire);
			}

			void setThrottle(const float value) {
				_throttle.store(value, std::memory_order_release);
			}

			float getAilerons() const {
				return _ailerons.load(std::memory_order_acquire);
			}

			void setAilerons(const float value) {
				_ailerons.store(value, std::memory_order_release);
			}

			float getElevator() const {
				return _elevator.load(std::memory_order_acquire);
			}

			void setElevator(const float value) {
				_elevator.store(value, std::memory_order_release);
			}

			float getRudder() const {
				return _rudder.load(std::memory_order_acquire);
			}

			void setRudder(const float value) {
				_rudder.store(value, std::memory_order_release);
			}

			float getFlaps() const {
				return _flaps.load(std::memory_order_acquire);
			}

			void setFlaps(const float value) {
				_flaps.store(value, std::memory_order_release);
			}

		private:
			// Factor in [0.0; 1.0] range
			std::atomic<float> _throttle { 0.0f };
			std::atomic<float> _ailerons { 0.5f };
			std::atomic<float> _elevator { 0.5f };
			std::atomic<float> _rudder { 0.5f };
			std::atomic<float> _flaps { 0.0f };
	};
	
	class RemoteData {
		public:
			RemoteDataRawControls controls {};
	};
}