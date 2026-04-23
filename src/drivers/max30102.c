#include "drivers/max30102.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define MAX30102_PART_ID_VALUE 0x15

#define MAX30102_REG_INT_STATUS_1 0x00
#define MAX30102_REG_INT_STATUS_2 0x01
#define MAX30102_REG_INT_ENABLE_1 0x02
#define MAX30102_REG_INT_ENABLE_2 0x03
#define MAX30102_REG_FIFO_WR_PTR 0x04
#define MAX30102_REG_FIFO_OVF_COUNTER 0x05
#define MAX30102_REG_FIFO_RD_PTR 0x06
#define MAX30102_REG_FIFO_DATA 0x07
#define MAX30102_REG_FIFO_CONFIG 0x08
#define MAX30102_REG_MODE_CONFIG 0x09
#define MAX30102_REG_SPO2_CONFIG 0x0A
#define MAX30102_REG_LED1_PA 0x0C
#define MAX30102_REG_LED2_PA 0x0D
#define MAX30102_REG_PART_ID 0xFF

#define MAX30102_MODE_RESET 0x40
#define MAX30102_MODE_SPO2 0x03

#define MAX30102_INT_PPG_RDY 0x40
#define MAX30102_INT_A_FULL 0x80

#define MAX30102_FIFO_ROLLOVER_EN 0x10
#define MAX30102_FIFO_A_FULL_15 0x0F

#define MAX30102_SPO2_ADC_RANGE_4096 0x20
#define MAX30102_SPO2_SR_100HZ 0x04
#define MAX30102_SPO2_PW_411US 0x03

#define MAX30102_LED_CURRENT_7MA 0x24

esp_err_t max30102_write_reg(i2c_bus_handle_t *dev, uint8_t reg_addr, uint8_t value)
{
    uint8_t data[2] = {reg_addr, value};
    return i2c_master_transmit(dev->max30102_handle, data, sizeof(data), 1000);
}

esp_err_t max30102_read_reg(i2c_bus_handle_t *dev, uint8_t reg_addr, uint8_t *value)
{
    return i2c_master_transmit_receive(dev->max30102_handle, &reg_addr, 1, value, 1, 1000);
}

esp_err_t max30102_clear_interrupts(i2c_bus_handle_t *dev)
{
    uint8_t status;
    esp_err_t err = max30102_read_reg(dev, MAX30102_REG_INT_STATUS_1, &status);
    if (err != ESP_OK) return err;

    return max30102_read_reg(dev, MAX30102_REG_INT_STATUS_2, &status);
}

esp_err_t max30102_init(i2c_bus_handle_t *dev)
{
    uint8_t part_id;
    esp_err_t err = max30102_read_reg(dev, MAX30102_REG_PART_ID, &part_id);
    if (err != ESP_OK) return err;
    if (part_id != MAX30102_PART_ID_VALUE) return ESP_ERR_NOT_FOUND;

    err = max30102_write_reg(dev, MAX30102_REG_MODE_CONFIG, MAX30102_MODE_RESET);
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(100));

    err = max30102_write_reg(dev, MAX30102_REG_FIFO_WR_PTR, 0x00);
    if (err != ESP_OK) return err;
    err = max30102_write_reg(dev, MAX30102_REG_FIFO_OVF_COUNTER, 0x00);
    if (err != ESP_OK) return err;
    err = max30102_write_reg(dev, MAX30102_REG_FIFO_RD_PTR, 0x00);
    if (err != ESP_OK) return err;

    err = max30102_write_reg(dev, MAX30102_REG_FIFO_CONFIG,
                             MAX30102_FIFO_ROLLOVER_EN |
                             MAX30102_FIFO_A_FULL_15);
    if (err != ESP_OK) return err;

    err = max30102_write_reg(dev, MAX30102_REG_SPO2_CONFIG,
                             MAX30102_SPO2_ADC_RANGE_4096 |
                             MAX30102_SPO2_SR_100HZ |
                             MAX30102_SPO2_PW_411US);
    if (err != ESP_OK) return err;

    err = max30102_write_reg(dev, MAX30102_REG_LED1_PA, MAX30102_LED_CURRENT_7MA);
    if (err != ESP_OK) return err;
    err = max30102_write_reg(dev, MAX30102_REG_LED2_PA, MAX30102_LED_CURRENT_7MA);
    if (err != ESP_OK) return err;

    err = max30102_clear_interrupts(dev);
    if (err != ESP_OK) return err;

    err = max30102_write_reg(dev, MAX30102_REG_INT_ENABLE_1,
                             MAX30102_INT_A_FULL | MAX30102_INT_PPG_RDY);
    if (err != ESP_OK) return err;
    err = max30102_write_reg(dev, MAX30102_REG_INT_ENABLE_2, 0x00);
    if (err != ESP_OK) return err;

    return max30102_write_reg(dev, MAX30102_REG_MODE_CONFIG, MAX30102_MODE_SPO2);
}

esp_err_t max30102_get_fifo_sample_count(i2c_bus_handle_t *dev, uint8_t *sample_count)
{
    uint8_t write_ptr;
    uint8_t read_ptr;
    esp_err_t err = max30102_read_reg(dev, MAX30102_REG_FIFO_WR_PTR, &write_ptr);
    if (err != ESP_OK) return err;

    err = max30102_read_reg(dev, MAX30102_REG_FIFO_RD_PTR, &read_ptr);
    if (err != ESP_OK) return err;

    write_ptr &= 0x1F;
    read_ptr &= 0x1F;
    *sample_count = (write_ptr >= read_ptr) ? (write_ptr - read_ptr) : (32 + write_ptr - read_ptr);
    return ESP_OK;
}

esp_err_t max30102_read_fifo_sample(i2c_bus_handle_t *dev, max30102_sample_t *sample)
{
    uint8_t reg_addr = MAX30102_REG_FIFO_DATA;
    uint8_t data[6];
    esp_err_t err = i2c_master_transmit_receive(dev->max30102_handle,
                                                &reg_addr,
                                                1,
                                                data,
                                                sizeof(data),
                                                1000);
    if (err != ESP_OK) return err;

    sample->red = (((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | data[2]) & 0x3FFFF;
    sample->ir = (((uint32_t)data[3] << 16) | ((uint32_t)data[4] << 8) | data[5]) & 0x3FFFF;
    return ESP_OK;
}
