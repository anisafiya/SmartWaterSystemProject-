#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <esp_rmaker_core.h>
#include <esp_rmaker_standard_types.h>
#include <app_network.h>
#include <app_insights.h>
#include "app_priv.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp32-dht11.h"
#include <app_wifi.h>
#include <time.h>
#include "freertos/queue.h"
#include "esp_task_wdt.h"
#include "app_driver.c"
#include "esp_rmaker_ota.h"

// Logging tag for ESP RainMaker
#define TAG "ESP_RAINMAKER"

// GPIO and ADC configurations
#define SENSOR_ADC_CHANNEL ADC_CHANNEL_4 // Soil moisture sensor channel
#define SENSOR_ADC_UNIT    ADC_UNIT_1    // ADC unit used
#define CONFIG_DHT11_PIN GPIO_NUM_3      // DHT11 data pin
#define CONFIG_CONNECTION_TIMEOUT 5      // DHT11 read timeout in seconds
#define RELAY_GPIO GPIO_NUM_2            // GPIO for water pump relay
#define BUTTON_PIN GPIO_NUM_5            // GPIO for manual watering button
#define INTERRUPT_NOTIFICATION_VALUE 1   // Value sent on notification

// ESP RainMaker parameter handles
esp_rmaker_param_t *temperature_value_param;
esp_rmaker_param_t *humidity_value_param;
esp_rmaker_param_t *moisture_alert_param;
esp_rmaker_param_t *soil_moisture_state_param;
esp_rmaker_param_t *plant_status_param;
esp_rmaker_param_t *pump_param;
esp_rmaker_param_t *watering_schedule_param;

#define MAX_WATERING_TIMES 5 // Maximum number of scheduled watering times
struct tm watering_times[MAX_WATERING_TIMES] = {0}; // Stores schedule

// Sensor data
float temperature, humidity;
int raw_value;

// Task and synchronization handles
TaskHandle_t receiverTaskHandle = NULL;
SemaphoreHandle_t watering_mutex = NULL;
QueueHandle_t notification_queue;

// Queue message structure
typedef struct {
    char message[64];
} notification_msg_t;

// Callback to handle write requests from RainMaker (generic GPIO control)
static esp_err_t write_cb(const esp_rmaker_device_t *device, const esp_rmaker_param_t *param,
    const esp_rmaker_param_val_t val, void *priv_data, esp_rmaker_write_ctx_t *ctx)
{
    if (ctx) {
        ESP_LOGI(TAG, "Received write request via : %s", esp_rmaker_device_cb_src_to_str(ctx->src));
    }
    if (app_driver_set_gpio(esp_rmaker_param_get_name(param), val.val.b) == ESP_OK) {
        esp_rmaker_param_update(param, val);
    }
    return ESP_OK;
}

// Callback for voice assistant control of the water pump
static esp_err_t voice_control_callback(const esp_rmaker_device_t *device, 
    const esp_rmaker_param_t *param, 
    const esp_rmaker_param_val_t val, 
    void *priv_data, 
    esp_rmaker_write_ctx_t *ctx)
{
    if (ctx) {
        ESP_LOGI(TAG, "Received control request via: %s", esp_rmaker_device_cb_src_to_str(ctx->src));
    }

    if (val.type == RMAKER_VAL_TYPE_BOOLEAN) {
        if (xSemaphoreTake(watering_mutex, pdMS_TO_TICKS(1000))) {
            bool new_state = val.val.b;
            const char *action = new_state ? "ON" : "OFF";
            const char *source = ctx ? esp_rmaker_device_cb_src_to_str(ctx->src) : "local";
            
            // Set the GPIO state for the pump
            gpio_set_level(RELAY_GPIO, new_state ? 1 : 0);
            ESP_LOGI(TAG, "Pump turned %s via %s", action, source);
            
            // Update RainMaker parameter state
            esp_rmaker_param_update(param, val);
            
            // Prepare and send notification message
            notification_msg_t msg;
            snprintf(msg.message, sizeof(msg.message), 
                "Water pump turned %s via %s", action, source);
            xQueueSend(notification_queue, &msg, portMAX_DELAY);
            
            xSemaphoreGive(watering_mutex);
            return ESP_OK;
        }
        return ESP_ERR_TIMEOUT;
    }
    return ESP_ERR_INVALID_ARG;
}

// Local function to control pump GPIO state directly
void water_pump_control(bool state) {
    if (state) {
        gpio_set_level(RELAY_GPIO, 1);  // ON
        ESP_LOGI(TAG, "Water Pump ON");
    } else {
        gpio_set_level(RELAY_GPIO, 0);  // OFF
        ESP_LOGI(TAG, "Water Pump OFF");
    }
}

// Sets a new watering time from string input (e.g., "08:00")
void set_new_watering_time(const char *time_str) {
    int hour, minute;
    if (sscanf(time_str, "%d:%d", &hour, &minute) == 2) {
        watering_times->tm_hour = hour;
        watering_times->tm_min = minute;

        ESP_LOGI(TAG, "Updated watering time: %02d:%02d", hour, minute);
    } else {
        ESP_LOGE(TAG, "Invalid time format for watering schedule: %s", time_str);
    }
}

// RainMaker callback for schedule updates (comma-separated "HH:MM" strings)
static esp_err_t watering_schedules_cb(const esp_rmaker_device_t *device, const esp_rmaker_param_t *param,
    const esp_rmaker_param_val_t val, void *priv_data, esp_rmaker_write_ctx_t *ctx) {
    if (val.type == RMAKER_VAL_TYPE_STRING) {
        ESP_LOGI(TAG, "New Watering Schedules: %s", val.val.s);

        char *schedule = strtok(val.val.s, ",");
        int i = 0;
        while (schedule != NULL && i < MAX_WATERING_TIMES) {
            int hour, minute;
            if (sscanf(schedule, "%d:%d", &hour, &minute) == 2) {
                watering_times[i].tm_hour = hour;
                watering_times[i].tm_min = minute;
                ESP_LOGI(TAG, "Stored watering time: %02d:%02d", hour, minute);
                i++;
            }
            schedule = strtok(NULL, ",");
        }
    }
    return ESP_OK;
}

// Send unified push + UI notification via RainMaker
void send_unified_notification(const char *message) {
    esp_rmaker_param_update_and_report(moisture_alert_param, esp_rmaker_str(message));
    
    esp_err_t err = esp_rmaker_raise_alert(message);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Notification sent and alert updated: %s", message);
    } else {
        ESP_LOGE(TAG, "Failed to send notification! Error: %d", err);
    }
}

// Notification handler task (listens for messages on queue)
void notification_task(void *arg) {
    notification_msg_t received_msg;

    while (1) {
        if (xQueueReceive(notification_queue, &received_msg, portMAX_DELAY)) {
            ESP_LOGI(TAG, "Processing notification: %s", received_msg.message);
            send_unified_notification(received_msg.message);
        }
    }
}

// Reads temperature, humidity, and soil moisture periodically
void sensor_task(void *arg) {
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t adc1_init_config = {
        .unit_id = SENSOR_ADC_UNIT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    adc_oneshot_new_unit(&adc1_init_config, &adc1_handle);

    adc_oneshot_chan_cfg_t adc_channel_config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    adc_oneshot_config_channel(adc1_handle, SENSOR_ADC_CHANNEL, &adc_channel_config);

    dht11_t dht11_sensor;
    dht11_sensor.dht11_pin = CONFIG_DHT11_PIN;

    while (1) {
        // Read temperature and humidity from DHT11
        if (dht11_read(&dht11_sensor, CONFIG_CONNECTION_TIMEOUT) == ESP_OK) {
            temperature = dht11_sensor.temperature;
            humidity = dht11_sensor.humidity;
            ESP_LOGI(TAG, "Sensor Read -> Temp: %.2f°C, Humidity: %.2f%%", temperature, humidity);

            esp_rmaker_param_update_and_report(temperature_value_param, esp_rmaker_float(temperature));
            esp_rmaker_param_update_and_report(humidity_value_param, esp_rmaker_float(humidity));
        } else {
            ESP_LOGE(TAG, "DHT11 read failed!");
        }

        // Read soil moisture value
        adc_oneshot_read(adc1_handle, SENSOR_ADC_CHANNEL, &raw_value);
        ESP_LOGI(TAG, "Soil Moisture Value: %d", raw_value);

        // Determine soil state and act accordingly
        notification_msg_t msg;
        const char* soil_state;
        
        if (raw_value > 2700) {
            soil_state = "Dry";
            snprintf(msg.message, sizeof(msg.message), "ALERT: Soil is Dry! Auto-watering activated");

            if (receiverTaskHandle != NULL) {
                xTaskNotifyGive(receiverTaskHandle); // Notify watering task
            }
        } 
        else if (raw_value > 1500) {
            soil_state = "OK";
            snprintf(msg.message, sizeof(msg.message), "Soil moisture is normal");
        } 
        else {
            soil_state = "Wet";
            snprintf(msg.message, sizeof(msg.message), "WARNING: Soil is too Wet! Manual intervention needed");
        }

        esp_rmaker_param_update_and_report(soil_moisture_state_param, esp_rmaker_str(soil_state));

        if (strcmp(soil_state, "OK") == 0) {
            esp_rmaker_param_update_and_report(moisture_alert_param, esp_rmaker_str(msg.message));
            ESP_LOGI(TAG, "Updated alert (UI only): %s", msg.message);
        } else {
            xQueueSend(notification_queue, &msg, portMAX_DELAY); // Send notification
        }

        vTaskDelay(pdMS_TO_TICKS(30000)); // Delay between readings
    }
}

// Watering task triggered on dry soil
void watering_task1(void *arg) {
    gpio_set_direction(RELAY_GPIO, GPIO_MODE_OUTPUT);

    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // Wait for signal
        if (watering_mutex && xSemaphoreTake(watering_mutex, portMAX_DELAY)) {
            ESP_LOGI(TAG, "Dry Soil: Starting watering...");
            gpio_set_level(RELAY_GPIO, 1);
            vTaskDelay(pdMS_TO_TICKS(2000)); 
            gpio_set_level(RELAY_GPIO, 0);
            ESP_LOGI(TAG, "Watering done.");
            xSemaphoreGive(watering_mutex);
        }
    }
}

// Watering task triggered on schedule
void watering_task2(void *arg) {
    esp_task_wdt_reset();
    gpio_set_direction(RELAY_GPIO, GPIO_MODE_OUTPUT);

    while (1) {
        esp_rmaker_param_val_t schedule_val = *esp_rmaker_param_get_val(watering_schedule_param);
        if (schedule_val.type == RMAKER_VAL_TYPE_STRING) {
            char schedule_copy[128];
            strncpy(schedule_copy, schedule_val.val.s, sizeof(schedule_copy));
            schedule_copy[sizeof(schedule_copy) - 1] = '\0';

            char *schedule = strtok(schedule_copy, ",");
            while (schedule != NULL) {
                int hour, minute;
                if (sscanf(schedule, "%d:%d", &hour, &minute) == 2) {
                    time_t now;
                    struct tm timeinfo;
                    time(&now);
                    localtime_r(&now, &timeinfo);

                    if (timeinfo.tm_hour == hour &&
                        timeinfo.tm_min == minute &&
                        timeinfo.tm_sec == 0) {

                        if (watering_mutex && xSemaphoreTake(watering_mutex, 0)) {
                            notification_msg_t msg;
                            snprintf(msg.message, sizeof(msg.message), "Scheduled watering started at %02d:%02d", hour, minute);
                            xQueueSend(notification_queue, &msg, portMAX_DELAY);

                            ESP_LOGI(TAG, "Scheduled watering at %02d:%02d...", hour, minute);
                            gpio_set_level(RELAY_GPIO, 1);
                            vTaskDelay(pdMS_TO_TICKS(2000)); 
                            gpio_set_level(RELAY_GPIO, 0);
                            ESP_LOGI(TAG, "Watering done.");

                            xSemaphoreGive(watering_mutex);
                        }
                    }
                }

                schedule = strtok(NULL, ",");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1000)); 
    }
}

// Manual watering task triggered by physical button press
void watering_task3(void *arg) {
    gpio_set_direction(RELAY_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(BUTTON_PIN, GPIO_MODE_INPUT);
    while (1) {
        if(gpio_get_level(BUTTON_PIN) == 0){
            if (watering_mutex && xSemaphoreTake(watering_mutex, portMAX_DELAY)) {
                ESP_LOGI(TAG, "Button pushed: Starting watering...");
                gpio_set_level(RELAY_GPIO, 1);
                vTaskDelay(pdMS_TO_TICKS(2000)); 
                gpio_set_level(RELAY_GPIO, 0);
                ESP_LOGI(TAG, "Watering done.");
                xSemaphoreGive(watering_mutex);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); 
    }
}

// Main application setup
void app_main(){
    // NVS initialization
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    // Wi-Fi setup
    app_network_init();

    // RainMaker node configuration
    esp_rmaker_config_t rainmaker_cfg = {
        .enable_time_sync = false,
    };

    esp_rmaker_node_t *node = esp_rmaker_node_init(&rainmaker_cfg, "Plant Watering System", "Test Notifications");
    if (!node) {
        ESP_LOGE(TAG, "Could not initialize node. Aborting!!!");
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        abort();
    }

    // Temperature and humidity sensor device
    esp_rmaker_device_t *status_device = esp_rmaker_device_create("Plant Status", ESP_RMAKER_DEVICE_TEMP_SENSOR, NULL);
    
    // Add sensor parameters to device
    temperature_value_param = esp_rmaker_param_create("Temperature Value", NULL, esp_rmaker_float(0.0), PROP_FLAG_READ | PROP_FLAG_PERSIST);
    humidity_value_param = esp_rmaker_param_create("Humidity Value", NULL, esp_rmaker_float(0.0), PROP_FLAG_READ | PROP_FLAG_PERSIST);
    soil_moisture_state_param = esp_rmaker_param_create("Soil Moisture State", NULL, esp_rmaker_str("OK"), PROP_FLAG_READ | PROP_FLAG_PERSIST);
    moisture_alert_param = esp_rmaker_param_create("Alert", NULL, esp_rmaker_str("No Alerts"), PROP_FLAG_READ | PROP_FLAG_PERSIST);

    esp_rmaker_device_add_param(status_device, temperature_value_param);
    esp_rmaker_device_add_param(status_device, humidity_value_param);
    esp_rmaker_device_add_param(status_device, soil_moisture_state_param);
    esp_rmaker_device_add_param(status_device, moisture_alert_param);
    
    esp_rmaker_node_add_device(node, status_device);

    // Pump switch device
    esp_rmaker_device_t *pump_device = esp_rmaker_device_create("Water Pump", ESP_RMAKER_DEVICE_SWITCH, NULL);
    pump_param = esp_rmaker_param_create("Switch", ESP_RMAKER_PARAM_TOGGLE, esp_rmaker_bool(false), PROP_FLAG_READ | PROP_FLAG_WRITE | PROP_FLAG_PERSIST);
    esp_rmaker_param_add_ui_type(pump_param, ESP_RMAKER_UI_TOGGLE);
    esp_rmaker_device_add_param(pump_device, pump_param);
    esp_rmaker_device_add_cb(pump_device, voice_control_callback, NULL);
    esp_rmaker_node_add_device(node, pump_device);

    // Add schedule configuration parameter
    watering_schedule_param = esp_rmaker_param_create("Watering Schedule", NULL, esp_rmaker_str("08:00,12:00,16:00"), PROP_FLAG_READ | PROP_FLAG_WRITE | PROP_FLAG_PERSIST);
    esp_rmaker_device_add_param(pump_device, watering_schedule_param);

    // Enable OTA and diagnostics
    esp_rmaker_ota_enable_default();
    app_insights_enable();

    // Start RainMaker agent and Wi-Fi
    esp_rmaker_start();
    err = app_network_start(POP_TYPE_RANDOM);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not start Wi-Fi. Aborting!!!");
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        abort();
    }

    // Create synchronization primitives
    watering_mutex = xSemaphoreCreateMutex();
    notification_queue = xQueueCreate(5, sizeof(notification_msg_t));
    if (notification_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create notification queue");
        abort();
    }

    // Launch tasks
    xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 5, NULL);
    xTaskCreate(watering_task1, "watering_task1", 4096, NULL, 5, &receiverTaskHandle);
    xTaskCreate(watering_task2, "watering_task2", 4096, NULL, 5 ,NULL);
    xTaskCreate(watering_task3, "watering_task3", 4096, NULL, 5 ,NULL);
    xTaskCreate(notification_task, "notification_task", 4096, NULL, 5, NULL);
}
