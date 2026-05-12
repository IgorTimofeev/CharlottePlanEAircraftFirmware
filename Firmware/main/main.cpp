#include "aircraft.h"

extern "C" void app_main(void) {
	pizda::Aircraft::getInstance().start();
}

// #include <freertos/FreeRTOS.h>
// #include <freertos/task.h>
//
// #include <driver/gpio.h>
// #include <esp_log.h>
//
// extern "C" void app_main(void) {
// 	gpio_config_t g {};
// 	g.pin_bit_mask = (1ULL << GPIO_NUM_2) | (1ULL << GPIO_NUM_1);
// 	g.mode = GPIO_MODE_OUTPUT;
// 	g.pull_up_en = GPIO_PULLUP_DISABLE;
// 	g.pull_down_en = GPIO_PULLDOWN_DISABLE;
// 	g.intr_type = GPIO_INTR_DISABLE;
// 	ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_config(&g));
//
// 	ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_level(GPIO_NUM_1, true));
// 	ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_level(GPIO_NUM_2, true));
//
// 	while (true) {
// 		ESP_LOGI("pizda", "tick");
// 		vTaskDelay(pdMS_TO_TICKS(1000));
// 	}
// }
