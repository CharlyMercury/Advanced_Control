#pragma once
#include <stdint.h>
#include "esp_err.h"

typedef struct {
    int64_t position_cnt;
    int32_t delta_cnt;
    float   rpm;
    float   rad_s;
    uint32_t sample_ms;
    uint32_t glitch_ns;
    int64_t t_us;
} encoder_data_t;

esp_err_t encoder_init_pcnt_x4(
    int gpio_a,
    int gpio_b,
    uint32_t glitch_ns,
    uint32_t cpr_x4,
    uint32_t sample_period_ms
);

void encoder_get_data(encoder_data_t *out);

int64_t encoder_get_position_cnt(void);
float encoder_get_rpm(void);