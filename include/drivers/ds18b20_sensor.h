#pragma once

#include "esp_err.h"
#include <stdint.h>

esp_err_t ds18b20_read_temperature(float *out_temp);
