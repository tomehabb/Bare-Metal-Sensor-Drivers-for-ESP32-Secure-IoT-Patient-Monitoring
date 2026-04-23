#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include "app/project_config.h"
#include "drivers/i2c_bus.h"
#include "tasks/pulse_oximeter_task.h"
#include "tasks/temperature_monitor_task.h"
#include "esp_log.h"

static const char *TAG = "main";

void app_main(void)
{
    static i2c_bus_handle_t i2c_bus;

    esp_err_t err = i2c_init(
        &i2c_bus,
        SECUREVITALS_I2C_SDA_PIN,
        SECUREVITALS_I2C_SCL_PIN,
        SECUREVITALS_I2C_CLOCK_HZ);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C init failed: %s", esp_err_to_name(err));
        vTaskDelete(NULL);
    }

    ESP_LOGI(TAG, "I2C initialized successfully");

    xTaskCreate(pulse_oximeter_task, "pulse_ox_task", 8192, &i2c_bus, 5, NULL);
    xTaskCreate(temperature_monitor_task, "temp_task", 8192, &i2c_bus, 5, NULL);
}
