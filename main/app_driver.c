#include <driver/gpio.h>
#include <string.h>


#define PUMP_GPIO GPIO_NUM_2  // Relay GPIO for the watering pump

void app_driver_init() {
    gpio_set_direction(PUMP_GPIO, GPIO_MODE_OUTPUT);
}

esp_err_t app_driver_set_gpio(const char *param_name, bool state) {
    if (strcmp(param_name, "Pump") == 0) {
        gpio_set_level(PUMP_GPIO, state);
    } else {
        return ESP_FAIL;
    }
    return ESP_OK;
}