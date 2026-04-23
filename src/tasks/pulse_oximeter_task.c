#include "tasks/pulse_oximeter_task.h"
#include <inttypes.h>
#include <math.h>
#include "app/project_config.h"
#include "driver/gpio.h"
#include "drivers/i2c_bus.h"
#include "drivers/max30102.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define PULSE_OX_WINDOW_SECONDS 4U
#define PULSE_OX_WINDOW_SIZE (SECUREVITALS_PULSE_OX_SAMPLE_RATE_HZ * PULSE_OX_WINDOW_SECONDS)
#define PULSE_OX_FINGER_PRESENT_IR 10000U
#define PULSE_OX_SIGNAL_JUMP_NUMERATOR 5U
#define PULSE_OX_SIGNAL_JUMP_DENOMINATOR 2U
#define PULSE_OX_MIN_READY_SAMPLES (SECUREVITALS_PULSE_OX_SAMPLE_RATE_HZ * 3U)
#define PULSE_OX_MIN_PEAK_DISTANCE_SAMPLES (SECUREVITALS_PULSE_OX_SAMPLE_RATE_HZ * 45U / 100U)
#define PULSE_OX_MIN_PERFUSION_INDEX 0.002f
#define PULSE_OX_MAX_INTERVAL_VARIATION 0.18f
#define PULSE_OX_INITIAL_CONFIRM_WINDOWS 2U
#define PULSE_OX_HR_CHANGE_CONFIRM_WINDOWS 3U
#define PULSE_OX_PENDING_HR_MATCH_BPM 10.0f
#define PULSE_OX_MAX_ACCEPTED_HR_STEP_BPM 12.0f
#define PULSE_OX_HR_SMOOTHING_ALPHA 0.35f
#define PULSE_OX_SPO2_SMOOTHING_ALPHA 0.25f

static const char *TAG = "pulse_oximeter_task";

static TaskHandle_t pulse_ox_task_handle;

static void IRAM_ATTR max30102_gpio_isr_handler(void *arg)
{
    (void)arg;
    BaseType_t higher_priority_task_woken = pdFALSE;

    if (pulse_ox_task_handle != NULL) {
        vTaskNotifyGiveFromISR(pulse_ox_task_handle, &higher_priority_task_woken);
    }

    if (higher_priority_task_woken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

static esp_err_t configure_max30102_interrupt(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << SECUREVITALS_MAX30102_INT_PIN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };

    esp_err_t err = gpio_config(&io_conf);
    if (err != ESP_OK) return err;

    err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;

    return gpio_isr_handler_add(SECUREVITALS_MAX30102_INT_PIN,
                                max30102_gpio_isr_handler,
                                NULL);
}

static uint32_t history_get(const uint32_t *history, size_t next_index, size_t count, size_t offset)
{
    size_t start = (next_index + PULSE_OX_WINDOW_SIZE - count) % PULSE_OX_WINDOW_SIZE;
    return history[(start + offset) % PULSE_OX_WINDOW_SIZE];
}

static void reset_history(uint32_t *red_history,
                          uint32_t *ir_history,
                          size_t *next_index,
                          size_t *sample_count)
{
    for (size_t i = 0; i < PULSE_OX_WINDOW_SIZE; i++) {
        red_history[i] = 0;
        ir_history[i] = 0;
    }

    *next_index = 0;
    *sample_count = 0;
}

static bool signal_changed_abruptly(uint32_t previous_ir, uint32_t current_ir)
{
    if (previous_ir < PULSE_OX_FINGER_PRESENT_IR || current_ir < PULSE_OX_FINGER_PRESENT_IR) {
        return false;
    }

    return current_ir > (previous_ir * PULSE_OX_SIGNAL_JUMP_NUMERATOR / PULSE_OX_SIGNAL_JUMP_DENOMINATOR) ||
           current_ir < (previous_ir * PULSE_OX_SIGNAL_JUMP_DENOMINATOR / PULSE_OX_SIGNAL_JUMP_NUMERATOR);
}

typedef struct
{
    bool valid;
    float heart_rate_bpm;
    float spo2_percent;
    float ir_perfusion_index;
    size_t peak_count;
    const char *status;
} pulse_ox_vitals_t;

typedef struct
{
    bool has_accepted;
    float heart_rate_bpm;
    float spo2_percent;
    float pending_heart_rate_bpm;
    float pending_spo2_percent;
    size_t pending_count;
} pulse_ox_report_filter_t;

static void reset_report_filter(pulse_ox_report_filter_t *filter)
{
    filter->has_accepted = false;
    filter->heart_rate_bpm = 0.0f;
    filter->spo2_percent = 0.0f;
    filter->pending_heart_rate_bpm = 0.0f;
    filter->pending_spo2_percent = 0.0f;
    filter->pending_count = 0;
}

static void track_pending_candidate(pulse_ox_report_filter_t *filter,
                                    float heart_rate_bpm,
                                    float spo2_percent)
{
    if (filter->pending_count == 0 ||
        fabsf(heart_rate_bpm - filter->pending_heart_rate_bpm) > PULSE_OX_PENDING_HR_MATCH_BPM) {
        filter->pending_heart_rate_bpm = heart_rate_bpm;
        filter->pending_spo2_percent = spo2_percent;
        filter->pending_count = 1;
        return;
    }

    float next_count = (float)(filter->pending_count + 1U);
    filter->pending_heart_rate_bpm += (heart_rate_bpm - filter->pending_heart_rate_bpm) / next_count;
    filter->pending_spo2_percent += (spo2_percent - filter->pending_spo2_percent) / next_count;
    filter->pending_count++;
}

static float move_toward(float current, float target, float max_step)
{
    float delta = target - current;
    if (delta > max_step) return current + max_step;
    if (delta < -max_step) return current - max_step;
    return target;
}

static bool filter_vitals_for_reporting(pulse_ox_report_filter_t *filter,
                                        pulse_ox_vitals_t *vitals)
{
    if (!vitals->valid) {
        return false;
    }

    if (!filter->has_accepted) {
        track_pending_candidate(filter, vitals->heart_rate_bpm, vitals->spo2_percent);
        if (filter->pending_count < PULSE_OX_INITIAL_CONFIRM_WINDOWS) {
            vitals->valid = false;
            vitals->status = "settling hr";
            return false;
        }

        filter->heart_rate_bpm = filter->pending_heart_rate_bpm;
        filter->spo2_percent = filter->pending_spo2_percent;
        filter->has_accepted = true;
        filter->pending_count = 0;
    } else {
        float hr_delta = fabsf(vitals->heart_rate_bpm - filter->heart_rate_bpm);
        if (hr_delta > PULSE_OX_MAX_ACCEPTED_HR_STEP_BPM) {
            track_pending_candidate(filter, vitals->heart_rate_bpm, vitals->spo2_percent);
            if (filter->pending_count < PULSE_OX_HR_CHANGE_CONFIRM_WINDOWS) {
                vitals->valid = false;
                vitals->status = "confirming hr";
                return false;
            }

            filter->heart_rate_bpm = move_toward(filter->heart_rate_bpm,
                                                 filter->pending_heart_rate_bpm,
                                                 PULSE_OX_MAX_ACCEPTED_HR_STEP_BPM);
            filter->spo2_percent += PULSE_OX_SPO2_SMOOTHING_ALPHA *
                                    (filter->pending_spo2_percent - filter->spo2_percent);
            filter->pending_count = 0;
        } else {
            filter->heart_rate_bpm += PULSE_OX_HR_SMOOTHING_ALPHA *
                                      (vitals->heart_rate_bpm - filter->heart_rate_bpm);
            filter->spo2_percent += PULSE_OX_SPO2_SMOOTHING_ALPHA *
                                    (vitals->spo2_percent - filter->spo2_percent);
            filter->pending_count = 0;
        }
    }

    vitals->heart_rate_bpm = filter->heart_rate_bpm;
    vitals->spo2_percent = filter->spo2_percent;
    vitals->status = "ok";
    return true;
}

static bool calculate_vitals(const uint32_t *red_history,
                             const uint32_t *ir_history,
                             size_t next_index,
                             size_t count,
                             pulse_ox_vitals_t *vitals)
{
    vitals->valid = false;
    vitals->heart_rate_bpm = 0.0f;
    vitals->spo2_percent = 0.0f;
    vitals->ir_perfusion_index = 0.0f;
    vitals->peak_count = 0;
    vitals->status = "stabilizing";

    if (count < PULSE_OX_MIN_READY_SAMPLES) {
        return false;
    }

    float red_dc = 0.0f;
    float ir_dc = 0.0f;
    for (size_t i = 0; i < count; i++) {
        red_dc += (float)history_get(red_history, next_index, count, i);
        ir_dc += (float)history_get(ir_history, next_index, count, i);
    }
    red_dc /= (float)count;
    ir_dc /= (float)count;

    if (red_dc < PULSE_OX_FINGER_PRESENT_IR || ir_dc < PULSE_OX_FINGER_PRESENT_IR) {
        vitals->status = "no finger";
        return false;
    }

    float red_ac_sum = 0.0f;
    float ir_ac_sum = 0.0f;
    float filtered_ir[PULSE_OX_WINDOW_SIZE] = {0};
    for (size_t i = 0; i < count; i++) {
        float red_delta = (float)history_get(red_history, next_index, count, i) - red_dc;
        float ir_delta = (float)history_get(ir_history, next_index, count, i) - ir_dc;
        filtered_ir[i] = ir_delta;
        red_ac_sum += red_delta * red_delta;
        ir_ac_sum += ir_delta * ir_delta;
    }

    float red_ac = sqrtf(red_ac_sum / (float)count);
    float ir_ac = sqrtf(ir_ac_sum / (float)count);
    if (red_ac <= 0.0f || ir_ac <= 0.0f) {
        return false;
    }

    for (size_t i = 2; i + 2 < count; i++) {
        filtered_ir[i] = (filtered_ir[i - 2] +
                          filtered_ir[i - 1] +
                          filtered_ir[i] +
                          filtered_ir[i + 1] +
                          filtered_ir[i + 2]) / 5.0f;
    }

    vitals->ir_perfusion_index = ir_ac / ir_dc;
    if (vitals->ir_perfusion_index < PULSE_OX_MIN_PERFUSION_INDEX) {
        vitals->status = "weak signal";
        return false;
    }

    float ratio = (red_ac / red_dc) / (ir_ac / ir_dc);
    if (ratio < 0.3f || ratio > 1.3f) {
        vitals->status = "unstable spo2 ratio";
        return false;
    }

    float threshold = ir_ac * 0.6f;
    size_t peak_indices[16] = {0};
    size_t peak_count = 0;

    for (size_t i = 2; i + 2 < count; i++) {
        float prev = filtered_ir[i - 1];
        float curr = filtered_ir[i];
        float next = filtered_ir[i + 1];

        if (curr > threshold && curr > prev && curr >= next) {
            if (peak_count == 0 || (i - peak_indices[peak_count - 1U]) >= PULSE_OX_MIN_PEAK_DISTANCE_SAMPLES) {
                if (peak_count < (sizeof(peak_indices) / sizeof(peak_indices[0]))) {
                    peak_indices[peak_count] = i;
                    peak_count++;
                }
            }
        }
    }

    vitals->peak_count = peak_count;
    if (peak_count < 3) {
        vitals->status = "finding pulse";
        return false;
    }

    float interval_sum = 0.0f;
    for (size_t i = 1; i < peak_count; i++) {
        interval_sum += (float)(peak_indices[i] - peak_indices[i - 1U]);
    }

    float average_interval = interval_sum / (float)(peak_count - 1U);
    float variance_sum = 0.0f;
    for (size_t i = 1; i < peak_count; i++) {
        float interval = (float)(peak_indices[i] - peak_indices[i - 1U]);
        float diff = interval - average_interval;
        variance_sum += diff * diff;
    }

    float interval_stddev = sqrtf(variance_sum / (float)(peak_count - 1U));
    if ((interval_stddev / average_interval) > PULSE_OX_MAX_INTERVAL_VARIATION) {
        vitals->status = "motion/noise";
        return false;
    }

    vitals->heart_rate_bpm = 60.0f * (float)SECUREVITALS_PULSE_OX_SAMPLE_RATE_HZ / average_interval;
    if (vitals->heart_rate_bpm < 45.0f || vitals->heart_rate_bpm > 140.0f) {
        vitals->status = "untrusted hr";
        return false;
    }

    vitals->spo2_percent = -45.060f * ratio * ratio + 30.354f * ratio + 94.845f;
    if (vitals->spo2_percent > 100.0f) vitals->spo2_percent = 100.0f;
    if (vitals->spo2_percent < 80.0f || vitals->spo2_percent > 100.0f) {
        vitals->status = "untrusted spo2";
        return false;
    }

    vitals->valid = true;
    vitals->status = "ok";
    return true;
}

void pulse_oximeter_task(void *pvParameters)
{
    i2c_bus_handle_t *dev = (i2c_bus_handle_t *)pvParameters;
    uint32_t red_history[PULSE_OX_WINDOW_SIZE] = {0};
    uint32_t ir_history[PULSE_OX_WINDOW_SIZE] = {0};
    size_t next_index = 0;
    size_t sample_count = 0;
    max30102_sample_t latest_sample = {0};
    uint32_t previous_ir = 0;
    pulse_ox_report_filter_t report_filter = {0};

    pulse_ox_task_handle = xTaskGetCurrentTaskHandle();

    esp_err_t err = configure_max30102_interrupt();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "MAX30102 interrupt setup failed: %s", esp_err_to_name(err));
        vTaskDelete(NULL);
    }

    err = max30102_init(dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "MAX30102 init failed: %s", esp_err_to_name(err));
        vTaskDelete(NULL);
    }

    ESP_LOGI(TAG, "MAX30102 init OK");

    TickType_t last_log_time = xTaskGetTickCount();

    while (1) {
        (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(SECUREVITALS_PULSE_OX_POLL_PERIOD_MS));

        err = max30102_clear_interrupts(dev);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Failed to clear MAX30102 interrupt status: %s", esp_err_to_name(err));
        }

        while (1) {
            uint8_t fifo_samples = 0;
            err = max30102_get_fifo_sample_count(dev, &fifo_samples);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to read MAX30102 FIFO count: %s", esp_err_to_name(err));
                break;
            }

            if (fifo_samples == 0) {
                break;
            }

            for (uint8_t i = 0; i < fifo_samples; i++) {
                err = max30102_read_fifo_sample(dev, &latest_sample);
                if (err != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to read MAX30102 FIFO sample: %s", esp_err_to_name(err));
                    break;
                }

                if (latest_sample.ir < PULSE_OX_FINGER_PRESENT_IR) {
                    if (sample_count > 0) {
                        reset_history(red_history, ir_history, &next_index, &sample_count);
                    }
                    reset_report_filter(&report_filter);
                    previous_ir = latest_sample.ir;
                    continue;
                }

                if (signal_changed_abruptly(previous_ir, latest_sample.ir)) {
                    reset_history(red_history, ir_history, &next_index, &sample_count);
                    reset_report_filter(&report_filter);
                }

                previous_ir = latest_sample.ir;

                red_history[next_index] = latest_sample.red;
                ir_history[next_index] = latest_sample.ir;
                next_index = (next_index + 1U) % PULSE_OX_WINDOW_SIZE;
                if (sample_count < PULSE_OX_WINDOW_SIZE) {
                    sample_count++;
                }
            }

            if (err != ESP_OK) {
                break;
            }
        }

        TickType_t now = xTaskGetTickCount();
        if ((now - last_log_time) >= pdMS_TO_TICKS(SECUREVITALS_PULSE_OX_LOG_PERIOD_MS)) {
            pulse_ox_vitals_t vitals;
            bool vitals_ready = calculate_vitals(red_history,
                                                 ir_history,
                                                 next_index,
                                                 sample_count,
                                                 &vitals);
            if (vitals_ready) {
                vitals_ready = filter_vitals_for_reporting(&report_filter, &vitals);
            }
            if (!vitals_ready && latest_sample.ir < PULSE_OX_FINGER_PRESENT_IR) {
                vitals.status = "no finger";
            }

            if (vitals_ready) {
                ESP_LOGI(TAG, "Heart Rate: %.0f bpm | SpO2: %.1f %% | PI: %.3f | Red: %" PRIu32 " | IR: %" PRIu32,
                         vitals.heart_rate_bpm,
                         vitals.spo2_percent,
                         vitals.ir_perfusion_index,
                         latest_sample.red,
                         latest_sample.ir);
            } else {
                ESP_LOGI(TAG, "Heart Rate: -- bpm | SpO2: -- %% | Status: %s | Samples: %u | PI: %.3f | Red: %" PRIu32 " | IR: %" PRIu32,
                         vitals.status,
                         (unsigned int)sample_count,
                         vitals.ir_perfusion_index,
                         latest_sample.red,
                         latest_sample.ir);
            }

            last_log_time = now;
        }
    }
}
