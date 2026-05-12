#pragma once

namespace pizda {
	class PulseWidthModulator {
		public:
			virtual ~PulseWidthModulator() = default;

			virtual void setPulseWidth(uint32_t pulseWidthUs) = 0;
	};
}
