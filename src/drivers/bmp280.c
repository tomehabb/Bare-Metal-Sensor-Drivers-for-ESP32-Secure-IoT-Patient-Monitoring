#include "driver/i2c_master.h"
#include "esp_err.h"
#include "drivers/i2c_bus.h"
#include "freertos/FreeRTOS.h"
#include "drivers/bmp280.h"

bmp280_calib_data_t bmp280_calib_data;


esp_err_t bmp280_write_reg(i2c_bus_handle_t *dev, uint8_t reg_addr, uint8_t value)
{
    uint8_t data[2] = {reg_addr, value};
    return i2c_master_transmit(dev ->bmp280_handle, data, 2, 1000);
}


esp_err_t bmp280_read_reg(i2c_bus_handle_t *dev, uint8_t reg_addr, uint8_t *value)
{
    return i2c_master_transmit_receive(dev ->bmp280_handle, &reg_addr, 1, value, 1, 1000);
}



esp_err_t bmp280_init(i2c_bus_handle_t *dev)
{
    // 1. Soft reset the sensor
    esp_err_t err = bmp280_write_reg(dev, 0xE0, 0xB6);
    if (err != ESP_OK) return err;
    vTaskDelay(100 / portTICK_PERIOD_MS);


    // 2. Read calibration data
    uint8_t calib_data[24];
    err = i2c_master_transmit_receive(dev ->bmp280_handle, (uint8_t[]){0x88}, 1, calib_data, sizeof(calib_data), 1000);
    if (err != ESP_OK) return err;

    bmp280_calib_data.dig_T1 = (uint16_t)(calib_data[1] << 8 | calib_data[0]);
    bmp280_calib_data.dig_T2 = (int16_t)(calib_data[3] << 8 | calib_data[2]);
    bmp280_calib_data.dig_T3 = (int16_t)(calib_data[5] << 8 | calib_data[4]);
    bmp280_calib_data.dig_P1 = (uint16_t)(calib_data[7] << 8 | calib_data[6]);
    bmp280_calib_data.dig_P2 = (int16_t)(calib_data[9] << 8 | calib_data[8]);
    bmp280_calib_data.dig_P3 = (int16_t)(calib_data[11] << 8 | calib_data[10]);
    bmp280_calib_data.dig_P4 = (int16_t)(calib_data[13] << 8 | calib_data[12]);
    bmp280_calib_data.dig_P5 = (int16_t)(calib_data[15] << 8 | calib_data[14]);
    bmp280_calib_data.dig_P6 = (int16_t)(calib_data[17] << 8 | calib_data[16]);
    bmp280_calib_data.dig_P7 = (int16_t)(calib_data[19] << 8 | calib_data[18]);
    bmp280_calib_data.dig_P8 = (int16_t)(calib_data[21] << 8 | calib_data[20]);
    bmp280_calib_data.dig_P9 = (int16_t)(calib_data[23] << 8 | calib_data[22]);

    // 3. Configure the sensor: normal mode, temp and pressure oversampling x1, standby time 1000ms
    err = bmp280_write_reg(dev, 0xF4, 0x24);
    if (err != ESP_OK) return err;

    err = bmp280_write_reg(dev, 0xF5, 0x42);
    if (err != ESP_OK) return err;

    err = bmp280_write_reg(dev, 0xF4, 0x27);
    if (err != ESP_OK) return err;

    return ESP_OK;
}

// Return Temperature in celsius
int32_t t_fine;
int32_t bmp280_compensate_T_int32(int32_t adc_T)
{
    int32_t var1, var2, T;
    var1 = ((((adc_T >> 3) - ((int32_t)bmp280_calib_data.dig_T1<<1))) * ((int32_t)bmp280_calib_data.dig_T2)) >> 11;
    var2 = (((((adc_T>>4) - ((int32_t)bmp280_calib_data.dig_T1)) * ((adc_T>>4) - ((int32_t)bmp280_calib_data.dig_T1))) >> 12) *
        ((int32_t)bmp280_calib_data.dig_T3)) >> 14;

    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    return T;
}

// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa

uint32_t bmp280_compensate_P_int32(int32_t adc_P)
{
    int64_t var1, var2, p;
    var1 = ((int64_t)t_fine - 128000);
    var2 = var1 * var1 * (int64_t)bmp280_calib_data.dig_P6;
    var2 = var2 + ((var1*(int64_t)bmp280_calib_data.dig_P5)<<17);
    var2 = var2 + (((int64_t)bmp280_calib_data.dig_P4)<<35);
    var1 = ((var1 * var1 * (int64_t)bmp280_calib_data.dig_P3)>>8) + ((var1 * (uint64_t)bmp280_calib_data.dig_P2)<<12);
    var1 = (((((int64_t)1)<<47)+var1))*((int64_t)bmp280_calib_data.dig_P1)>>33;
    if (var1 == 0)
    {
        return 0; // avoid exception caused by devision by 0
    }
    p = 1048576-adc_P;
    p = (((p<<31)-var2)*3125)/var1;
    var1 = (((int64_t)bmp280_calib_data.dig_P9) * (p>>13) * (p>>13)) >> 25;
    var2 = (((int64_t)bmp280_calib_data.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)bmp280_calib_data.dig_P7)<<4);
    return (uint32_t)p;
}

esp_err_t bmp280_read_data(i2c_bus_handle_t *dev, int32_t *temp, uint32_t *press)
{
    // 1. Burst read for registers 0xF7 to 0xFC (6 bytes)
    uint8_t raw_data[6];
    esp_err_t err = i2c_master_transmit_receive(dev ->bmp280_handle, (uint8_t[]){0xF7}, 1, raw_data, sizeof(raw_data), 1000);
    if (err != ESP_OK) return err;

    // 2. Extract temperature and pressure data
    uint32_t adc_T = (int32_t)(raw_data[3] << 12 | raw_data[4] << 4 | raw_data[5] >> 4);
    *temp = bmp280_compensate_T_int32(adc_T);
    uint32_t adc_P = (uint32_t)(raw_data[0] << 12 | raw_data[1] << 4 | raw_data[2] >> 4);
    *press = bmp280_compensate_P_int32(adc_P);

    return ESP_OK;

}