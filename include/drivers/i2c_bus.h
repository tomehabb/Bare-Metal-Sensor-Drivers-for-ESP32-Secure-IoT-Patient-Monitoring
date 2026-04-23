#ifndef I2C_BUS_H
#define I2C_BUS_H

#include "driver/i2c_master.h"
#include "esp_err.h"

typedef struct
{
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t bmp280_handle;
    i2c_master_dev_handle_t max30102_handle;
} i2c_bus_handle_t;

esp_err_t i2c_init(i2c_bus_handle_t *dev, int sda_pin, int scl_pin, uint32_t max_clock_speed_hz);


#endif // I2C_BUS_H
