#include "drivers/i2c_bus.h"
#include "driver/i2c_master.h"

#define BMP280_I2C_ADDR 0x76
#define MAX30102_I2C_ADDR 0x57

esp_err_t i2c_init(i2c_bus_handle_t *dev, int sda_pin, int scl_pin, uint32_t max_clock_speed_hz)
{
    // 1. Configure and create the I2C master bus
    i2c_master_bus_config_t bus_config = {
        .i2c_port = -1,
        .sda_io_num = sda_pin,
        .scl_io_num = scl_pin,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t err = i2c_new_master_bus(&bus_config, &dev->bus_handle);

    if (err != ESP_OK)
    {
        return err;
    };

    // 2. Create device handles for sensors on the shared bus.
    i2c_device_config_t bmp280_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BMP280_I2C_ADDR,
        .scl_speed_hz = max_clock_speed_hz
    };

    err = i2c_master_bus_add_device(dev ->bus_handle, &bmp280_config, &dev->bmp280_handle);
    if (err != ESP_OK) return err;

    i2c_device_config_t max30102_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MAX30102_I2C_ADDR,
        .scl_speed_hz = max_clock_speed_hz
    };

    err = i2c_master_bus_add_device(dev->bus_handle, &max30102_config, &dev->max30102_handle);
    if (err != ESP_OK) return err;

    return ESP_OK;
}
