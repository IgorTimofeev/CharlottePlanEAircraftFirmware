#include <esp_adc/adc_continuous.h>
#include <esp_log.h>

#include "Config.hpp"
#include "Aircraft.hpp"

namespace pizda {
	Aircraft& Aircraft::getInstance() {
		static Aircraft instance {};

		return instance;
	}
	
	[[noreturn]] void Aircraft::start() {
		// I2C
		{
			i2c_master_bus_config_t bus {};
			bus.clk_source = I2C_CLK_SRC_DEFAULT;
			bus.i2c_port = config::I2C::port;
			bus.scl_io_num = config::I2C::SCL;
			bus.sda_io_num = config::I2C::SDA;
			bus.glitch_ignore_cnt = 7;
			// Do we need this?
			bus.flags.enable_internal_pullup = false;

			ESP_ERROR_CHECK(i2c_new_master_bus(&bus, &I2CMasterBusHandle));
		}

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

		// Transceiver, motors and FBW should be initialized ASAP if we want to perform ESC calibration via thrust levers
		if (!transceiver.setup())
			startErrorLoop("XCVR setup failed");

		motors.setup();
		fbw.setup();

		// Everything else can be safely delayed
		lights.setup();

		{
			adc_unit_t ADCUnit;
			adc_channel_t ADCChannel;
			ESP_ERROR_CHECK(adc_continuous_io_to_channel(config::battery::pin, &ADCUnit, &ADCChannel));

			battery.setup(
				ADCUnit,
				getAssignedADCOneshotUnit(ADCUnit),
				ADCChannel,

				config::battery::voltageMin,
				config::battery::voltageMax,
				config::battery::voltageDividerR1,
				config::battery::voltageDividerR2
			);
		}

		adirs.setup();

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
