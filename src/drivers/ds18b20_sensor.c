#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

static portMUX_TYPE ow_spinlock = portMUX_INITIALIZER_UNLOCKED;

#define OW_GPIO GPIO_NUM_4


// All timing in microseconds

#define OW_RESET_PULSE_US 480
#define OW_RESET_WAIT_US 70
#define OW_SLOT_US 60
#define OW_WRITE1_LOW_US 6
#define OW_WRITE0_LOW_US 60
#define OW_READ_INIT_LOW_US 6
#define OW_READ_SAMPLE_US 9


/*
 OPEN-DRAIN OUTPUT EMULATION

 ESP32 doesn't have a true open-drain output, so we should toggle between input
 (line released, pulled high) and output-low
 */

static inline void ow_release(void) {
    gpio_set_direction(OW_GPIO, GPIO_MODE_INPUT);
}

static inline void ow_pull_low(void) {
    gpio_set_level(OW_GPIO, 0);
    gpio_set_direction(OW_GPIO, GPIO_MODE_OUTPUT);
}

static inline int ow_read_bit(void) {
    return gpio_get_level(OW_GPIO);
}

// Reset & Presence detection

bool ow_reset(void) {
    ow_pull_low();
    esp_rom_delay_us(OW_RESET_PULSE_US);
    ow_release();
    esp_rom_delay_us(OW_RESET_WAIT_US);

    bool presence = (ow_read_bit() == 0); // sensor pulls low = present

    esp_rom_delay_us(OW_RESET_PULSE_US - OW_RESET_WAIT_US); // wait for end of timeslot

    return presence;
}

// Bit-level read/write

static void ow_write_byte(uint8_t byte) {
    for (int i = 0; i < 8; i++) {
        if (byte & (1 << i)) {
            // write 1 : pull low 6 us, then release, wait for rest of timeslot
            ow_pull_low();
            esp_rom_delay_us(OW_WRITE1_LOW_US);
            ow_release();
            esp_rom_delay_us(OW_SLOT_US - OW_WRITE1_LOW_US);
        } else {
            // Write 0 : pull low for full slot
            ow_pull_low();
            esp_rom_delay_us(OW_WRITE0_LOW_US);
            ow_release();
            esp_rom_delay_us(OW_SLOT_US - OW_WRITE0_LOW_US);
        }
    }
}

static uint8_t ow_read_byte(void) {
    uint8_t byte = 0;
    for (int i = 0; i < 8; i++) {
        // Initiate read slot by pulling low for 6 us, then release
        ow_pull_low();
        esp_rom_delay_us(OW_READ_INIT_LOW_US);
        ow_release();

        // Sample bit value at 9 us from start of slot
        esp_rom_delay_us(OW_READ_SAMPLE_US);
        if (ow_read_bit()) {
            byte |= (1 << i);
        }

        // Wait for rest of timeslot
        esp_rom_delay_us(OW_SLOT_US - OW_READ_INIT_LOW_US - OW_READ_SAMPLE_US);
    }
    return byte;
}

// Step 6 — DS18B20 ROM Commands
#define CMD_SKIP_ROM 0xCC // Single sensor on bus, skip ROM selection
#define CMD_CONVERT_T 0x44 // Start temperature conversion
#define CMD_READ_SCRATCHPAD 0xBE // Read 9-byte scratchpad memory (temperature result)

// Step 8 — CRC-8 (Maxim/Dallas polynomial 0x31)
static uint8_t ds18b20_crc8(const uint8_t *data, size_t len) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) {
        uint8_t byte = data[i];
        for (int b = 0; b < 8; b++) {
            if ((crc ^ byte) & 0x01)
                crc = (crc >> 1) ^ 0x8C;
            else
                crc >>= 1;
            byte >>= 1;
        }
    }
    return crc;
}

// Step 7 — Temperature Read Sequence
esp_err_t ds18b20_read_temperature(float *out_temp) {
    taskENTER_CRITICAL(&ow_spinlock);

    // 1. Trigger conversion
    if (!ow_reset()) {
        taskEXIT_CRITICAL(&ow_spinlock);
        return ESP_ERR_NOT_FOUND;
    }
    ow_write_byte(CMD_SKIP_ROM);
    ow_write_byte(CMD_CONVERT_T);

    taskEXIT_CRITICAL(&ow_spinlock);

    // Conversion takes up to 750 ms at 12-bit resolution
    vTaskDelay(pdMS_TO_TICKS(800));

    taskENTER_CRITICAL(&ow_spinlock);

    // 2. Read scratchpad
    if (!ow_reset()) {
        taskEXIT_CRITICAL(&ow_spinlock);
        return ESP_ERR_NOT_FOUND;
    }
    ow_write_byte(CMD_SKIP_ROM);
    ow_write_byte(CMD_READ_SCRATCHPAD);

    uint8_t scratchpad[9];
    for (int i = 0; i < 9; i++) {
        scratchpad[i] = ow_read_byte();
    }

    taskEXIT_CRITICAL(&ow_spinlock);

    // 3. CRC-8 validation
    if (ds18b20_crc8(scratchpad, 8) != scratchpad[8]) {
        return ESP_ERR_INVALID_CRC;
    }

    int16_t raw = (int16_t)((scratchpad[1] << 8) | scratchpad[0]);
    // *out_raw = raw;

    // TODO: Convert raw to Celsius (12-bit default: LSB = 0.0625°C)
    *out_temp = raw / 16.0f;

    return ESP_OK;
}