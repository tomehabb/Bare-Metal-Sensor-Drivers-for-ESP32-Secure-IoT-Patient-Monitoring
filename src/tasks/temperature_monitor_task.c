#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdlib.h>
#include "drivers/i2c_bus.h"
#include "drivers/bmp280.h"
#include "drivers/ds18b20_sensor.h"
#include "esp_log.h"

static const char *TAG = "temperature_monitor_task";

void temperature_monitor_task(void *pvParameters)
{
    i2c_bus_handle_t *dev = (i2c_bus_handle_t *)pvParameters;

    if (bmp280_init(dev) != ESP_OK) {
        ESP_LOGE(TAG, "BMP280 init failed");
        vTaskDelete(NULL);
    }
    ESP_LOGI(TAG, "BMP280 init OK");

    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(1000);

    while (1) {
        vTaskDelayUntil(&last_wake_time, period);

        uint8_t status;
        if (bmp280_read_reg(dev, 0xF3, &status) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read status register");
            continue;
        }

        // Bit 3 = measuring (1 = busy, 0 = result ready)
        if (status & 0x08) {
            ESP_LOGD(TAG, "BMP280 conversion in progress, skipping");
            continue;
        }

        int32_t temp;
        uint32_t press;
        if (bmp280_read_data(dev, &temp, &press) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read raw data");
            continue;
        }

        float body_temp = 0.0f;
        esp_err_t ds_err = ds18b20_read_temperature(&body_temp);

        if (ds_err == ESP_OK) {
            ESP_LOGI(TAG, "Temp: %d.%02d | Pressure: %.2f Pa | Body Temp: %0.2f C",
                     temp / 100, abs(temp % 100), ((float)press / 256), body_temp);
        } else {
            ESP_LOGI(TAG, "Temp: %ld | Raw Pressure: %ld | Raw Body Temp: unavailable (err 0x%x)",
                     temp, press, ds_err);
        }

        // TODO: post to queue
    }
}
