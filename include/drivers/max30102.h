#ifndef MAX30102_H
#define MAX30102_H

#include <stdbool.h>
#include <stdint.h>
#include "drivers/i2c_bus.h"
#include "esp_err.h"

typedef struct
{
    uint32_t red;
    uint32_t ir;
} max30102_sample_t;

esp_err_t max30102_init(i2c_bus_handle_t *dev);
esp_err_t max30102_clear_interrupts(i2c_bus_handle_t *dev);
esp_err_t max30102_read_reg(i2c_bus_handle_t *dev, uint8_t reg_addr, uint8_t *value);
esp_err_t max30102_write_reg(i2c_bus_handle_t *dev, uint8_t reg_addr, uint8_t value);
esp_err_t max30102_get_fifo_sample_count(i2c_bus_handle_t *dev, uint8_t *sample_count);
esp_err_t max30102_read_fifo_sample(i2c_bus_handle_t *dev, max30102_sample_t *sample);

#endif // MAX30102_H
