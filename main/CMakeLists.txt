file(GLOB_RECURSE pizda_sources
    "*.*"
)

idf_component_register(
    SRCS ${pizda_sources}
    INCLUDE_DIRS "."
    REQUIRES
        esp_timer
        esp_driver_ledc
        esp_driver_uart
        esp_adc
        ESP-IDF-NVSExtensions
        ESP-IDF-Units
        ESP-IDF-SpatialTypes
        ESP-IDF-BitStream
        ESP-IDF-EMAFilter
        ESP-IDF-PIDController
        ESP-IDF-CircularQueue
        ESP-IDF-BusHAL
        ESP-IDF-SX1262
        ESP-IDF-MPU9250
        ESP-IDF-BMP280
        ESP-IDF-ADCVoltmeter
        ESP-IDF-PCA9685
        ESP-IDF-MS4525
)

target_compile_options(${COMPONENT_LIB} PUBLIC -std=gnu++26)