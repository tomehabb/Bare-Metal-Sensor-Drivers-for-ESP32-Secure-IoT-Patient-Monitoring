#include "esp_err.h"
#include "drivers/i2c_bus.h"

#ifndef BMP280_H
#define BMP280_H

typedef struct 
{
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;
} bmp280_calib_data_t;

extern bmp280_calib_data_t bmp280_calib_data;

esp_err_t bmp280_write_reg(i2c_bus_handle_t *dev, uint8_t reg_addr, uint8_t value);
esp_err_t bmp280_read_reg(i2c_bus_handle_t *dev, uint8_t reg_addr, uint8_t *value);
esp_err_t bmp280_init(i2c_bus_handle_t *dev);
esp_err_t bmp280_read_data(i2c_bus_handle_t *dev, int32_t *temp, uint32_t *press);
#endif // BMP280_H