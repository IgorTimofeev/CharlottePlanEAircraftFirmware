#pragma once

#include "NVSStream.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_timer.h>
#include <esp_log.h>
#include <memory>

namespace YOBA {
	class NVSSettings {
		public:
			virtual ~NVSSettings() = default;

			void read() {
				NVSStream stream {};
				stream.openForReading(getNamespace());

				onRead(stream);

				stream.close();
			}

			void write() {
				ESP_LOGI("NVSSettings", "Writing %s", getNamespace());

				NVSStream stream {};
				stream.openForWriting(getNamespace());

				onWrite(stream);

				stream.commit();
				stream.close();
			}

			void scheduleWrite() {
				const auto alreadyScheduled = _scheduledWriteTimeUs > 0;

				_scheduledWriteTimeUs = esp_timer_get_time() + 2'500'000;

				if (alreadyScheduled)
					return;

				xTaskCreate(
					[](void* arg) {
						const auto instance = static_cast<NVSSettings*>(arg);

						while (true) {
							const auto time = esp_timer_get_time();

							if (time >= instance->_scheduledWriteTimeUs)
								break;

							vTaskDelay(pdMS_TO_TICKS((instance->_scheduledWriteTimeUs - time) / 1000));
						}

						instance->_scheduledWriteTimeUs = 0;
						instance->write();

						vTaskDelete(nullptr);
					},
					"NVSSerWrite",
					4096,
					this,
					1,
					nullptr
				);
			}

		protected:
			virtual const char* getNamespace() = 0;
			virtual void onRead(const NVSStream& stream) = 0;
			virtual void onWrite(const NVSStream& stream) = 0;

		private:
			constexpr static uint32_t _writeDelayTicks = pdMS_TO_TICKS(2500);
			int64_t _scheduledWriteTimeUs = 0;
	};
}