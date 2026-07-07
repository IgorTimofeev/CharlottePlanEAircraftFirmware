#include "Systems/SimLink/SimLink.hpp"
#include "Aircraft.hpp"

namespace pizda {
	void SimLink::setup() {
		QueueHandle_t queue;
		ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, _bufferLength, _bufferLength, 10, &queue, 0));

		uart_config_t uartConfig {};
		uartConfig.baud_rate = 115200;
		uartConfig.data_bits = UART_DATA_8_BITS;
		uartConfig.parity = UART_PARITY_DISABLE;
		uartConfig.stop_bits = UART_STOP_BITS_1;
		uartConfig.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
		uartConfig.source_clk = UART_SCLK_DEFAULT;
		ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uartConfig));

		ESP_ERROR_CHECK(uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

		xTaskCreatePinnedToCore(
			[](void* arg) {
				static_cast<SimLink*>(arg)->taskBody();
			},
			"SimLink",
			8 * 1024,
			this,
			10,
			nullptr,
			0
		);
	}
	
	const SimLinkSimPacket& SimLink::getLastSimPacket() const {
		return _lastSimPacket;
	}
	
	void SimLink::taskBody() {
		auto& ac = Aircraft::getInstance();
		
		while (true) {
			const int bytesRead = uart_read_bytes(UART_NUM_0, _buffer, _bufferLength, pdMS_TO_TICKS(16));
			
			// Reading
			if (bytesRead >= sizeof(SimLinkSimPacket)) {
				const auto packet = reinterpret_cast<SimLinkSimPacket*>(_buffer);

				if (packet->header == SimLinkPacket::header) {
					_lastSimPacket = *packet;

//					ESP_LOGI("SimLink", "pitchRad: %f",
//						toDegrees(packet->pitchRad)
//					);
				}
				else {
					ESP_LOGI("SimLink", "invalid header: %d", packet->header);
				}
			}

			// Writing
			{
//				ESP_LOGI("SimLink", "writing");

				const auto packet = reinterpret_cast<SimLinkAircraftPacket*>(_buffer);

				packet->header = SimLinkPacket::header;
				packet->throttle = ac.motors.getByType(MotorType::throttleLeft)->getRawPowerF();
				packet->ailerons = ac.motors.getByType(MotorType::aileronLeft)->getRawPowerF();
				packet->elevator = ac.motors.getByType(MotorType::tailLeft)->getRawPowerF();
				packet->rudder = ac.motors.getByType(MotorType::tailRight)->getRawPowerF();
				packet->flaps = ac.motors.getByType(MotorType::flapLeft)->getRawPowerF();

				packet->lights =
					(ac.settings.lights.getNav())
					| (ac.settings.lights.getStrobe() << 1)
					| (ac.settings.lights.getLanding() << 2)
					| (ac.settings.lights.getCabin() << 3);

				uart_write_bytes(UART_NUM_0, _buffer, sizeof(SimLinkAircraftPacket));
			}
			
			vTaskDelay(pdMS_TO_TICKS(1'000 / 20));
		}
	}
}