#include "aircraft.h"

#include <esp_log.h>

#include "config.h"

namespace pizda {
	Aircraft& Aircraft::getInstance() {
		static Aircraft instance {};

		return instance;
	}
	
	[[noreturn]] void Aircraft::start() {
		// SPI
		{
			spi_bus_config_t busConfig {};
			busConfig.mosi_io_num = config::SPI::MOSI;
			busConfig.miso_io_num = config::SPI::MISO;
			busConfig.sclk_io_num = config::SPI::SCK;
			busConfig.quadwp_io_num = -1;
			busConfig.quadhd_io_num = -1;
			busConfig.max_transfer_sz = 512;
			ESP_ERROR_CHECK(spi_bus_initialize(config::SPI::device, &busConfig, SPI_DMA_CH_AUTO));
		}

		// ADC
		{
			adc_oneshot_unit_init_cfg_t unitConfig {};
			unitConfig.unit_id = ADC_UNIT_2;
			unitConfig.clk_src = ADC_RTC_CLK_SRC_DEFAULT;
			unitConfig.ulp_mode = ADC_ULP_MODE_DISABLE;
			ESP_ERROR_CHECK(adc_oneshot_new_unit(&unitConfig, &_ADCOneshotUnit2));
		}

		// Settings come first because they contain XCVR modulation params, motor configurations, etc.
		settings.setup();

		// Transceiver, motors and FBW should be initialized ASAP for ESC calibration possibility
		if (!transceiver.setup())
			startErrorLoop("XCVR setup failed");

		motors.setup();
		fbw.setup();

		// Everything else can be safely delayed
		lights.setup();
		battery.setup();

		adirs.setupAsync();

		#ifdef SIM
			simLink.setup();
		#endif

		// -------------------------------- Main loop --------------------------------

		while (true) {
			batteryTick();
			
			vTaskDelay(pdMS_TO_TICKS(1'000 / 30));
		}
	}
	
	[[noreturn]] void Aircraft::startErrorLoop(const char* error) {
		ESP_LOGE(_logTag, "%s", error);
		
		while (true) {
			vTaskDelay(pdMS_TO_TICKS(1'000));
		}
	}

	void Aircraft::batteryTick() {
		if (esp_timer_get_time() < _batteryTickTime)
			return;

		battery.tick();

		_batteryTickTime = esp_timer_get_time() + 1'000'000 / (1 * 8);
	}
}
