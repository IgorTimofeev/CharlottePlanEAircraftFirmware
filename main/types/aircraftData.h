#pragma once

#include <cmath>
#include <atomic>

#include "types/generic.h"

namespace pizda {
	class AircraftDataCalibration {
		public:
			bool isCalibrating() const {
				return _calibrating.load(std::memory_order_acquire);
			}

			void setCalibrating(const bool value) {
				_calibrating.store(value, std::memory_order_release);
			}

			AircraftCalibrationSystem getSystem() const {
				return _system.load(std::memory_order_acquire);
			}

			void setSystem(const AircraftCalibrationSystem value) {
				_system.store(value, std::memory_order_release);
			}

			uint8_t getProgress() const {
				return _progress.load(std::memory_order_acquire);
			}

			void setProgress(const uint8_t value) {
				_progress.store(value, std::memory_order_release);
			}

		private:
			std::atomic<bool> _calibrating { false };
			std::atomic<AircraftCalibrationSystem> _system { AircraftCalibrationSystem::accelAndGyro };
			std::atomic<uint8_t> _progress { 0xFF };
	};

	class AircraftDataCamera {
		public:
			int16_t getPitchDeg() const {
				return _pitchDeg.load(std::memory_order_acquire);
			}

			void setPitchDeg(const int16_t value) {
				_pitchDeg.store(value, std::memory_order_release);
			}

			int16_t getYawDeg() const {
				return _yawDeg.load(std::memory_order_acquire);
			}

			void setYawDeg(const int16_t value) {
				_yawDeg.store(value, std::memory_order_release);
			}

		private:
			std::atomic<int16_t> _pitchDeg = 0;
			std::atomic<int16_t> _yawDeg = 0;
	};

	class AircraftData {
		public:
			AircraftDataCalibration calibration {};
			AircraftDataCamera camera {};
	};
}