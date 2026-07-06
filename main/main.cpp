#include "aircraft.h"

extern "C" void app_main(void) {
	pizda::Aircraft::getInstance().start();
}

// #include <freertos/FreeRTOS.h>
// #include <freertos/task.h>
// #include <driver/gpio.h>
// #include <driver/i2c_master.h>
// #include <esp_log.h>
//
// #include <MS4525.h>
//
// extern "C" {
// 	using namespace YOBA;
//
// 	// Checks for sensor errors, logs them and puts ESP into an infinite waiting loop if something fails
// 	static MS4525Error checkSensorError(const MS4525Error error, const char* prefix) {
// 		if (error == MS4525Error::none)
// 			return error;
//
// 		ESP_LOGE("main", "%s: %s", prefix, MS4525::errorToString(error));
//
// 		// Deal with error
// 		while (true)
// 			vTaskDelay(pdMS_TO_TICKS(1000));
//
// 		return error;
// 	}
//
// 	void app_main(void) {
//
// 		// -------------------------------- I2C initialization --------------------------------
//
// 		i2c_master_bus_config_t I2CBusConfig {};
// 		I2CBusConfig.i2c_port = I2C_NUM_0;
// 		I2CBusConfig.sda_io_num = GPIO_NUM_3;              // Change to your pin
// 		I2CBusConfig.scl_io_num = GPIO_NUM_2;              // Change to your pin
// 		I2CBusConfig.clk_source = I2C_CLK_SRC_DEFAULT;
// 		I2CBusConfig.glitch_ignore_cnt = 7;
// 		I2CBusConfig.flags.enable_internal_pullup = false;
//
// 		i2c_master_bus_handle_t I2CBusHandle;
// 		ESP_ERROR_CHECK(i2c_new_master_bus(&I2CBusConfig, &I2CBusHandle));
//
// 		// -------------------------------- Sensor initialization --------------------------------
//
// 		MS4525 sensor {};
//
// 		sensor.setup(I2CBusHandle, MS4525::defaultI2CAddress, 100'000);
//
// 		// Reading & calculating the median value for the sensor bias, since without prior
// 		// software calibration, most sensor will produce messy results
// 		float differentialPressureBias = 0;
// 		auto error = sensor.computeMedianDifferentialPressureBias(differentialPressureBias);
// 		checkSensorError(error, "failed to compute median diff pressure bias");
//
// 		// Setting computed bias
// 		sensor.setDifferentialPressureBias(differentialPressureBias);
//
// 		ESP_LOGI("main", "median diff pressure bias: %f", differentialPressureBias);
//
// 		// -------------------------------- Airspeed reading --------------------------------
//
// 		float differentialPressurePSI, temperatureC, indicatedAirspeedMPS;
//
// 		while (true) {
// 			// Reading diff pressure & temperature
// 			error = sensor.readDifferentialPressureAndTemperature(differentialPressurePSI, temperatureC);
// 			checkSensorError(error, "failed to read diff pressure & temperature");
//
// 			// Computing IAS using diff pressure. It's highly recommended to apply EMA or similar filter to
// 			// smooth out airspeed value, but who cares...
// 			indicatedAirspeedMPS = MS4525::computeIndicatedAirspeedMPS(differentialPressurePSI);
//
// 			// Printing whole stuff out
// 			ESP_LOGI("main", "diff pressure: %f PSI, temperature: %f deg C, airspeed: %f m/s", differentialPressurePSI, temperatureC, indicatedAirspeedMPS);
//
// 			vTaskDelay(pdMS_TO_TICKS(1'000 / 50));
// 		}
// 	}
// }