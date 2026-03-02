/* =========================================================
 * File: main/encoder_pcnt.h
 * ========================================================= */
#pragma once
#include <stdint.h>
#include "esp_err.h"

typedef struct {
    int64_t position_cnt;   // acumulado (cuentas) desde arranque (int64)
    int32_t delta_cnt;      // cuentas en la última ventana
    float   rpm;            // velocidad (RPM)
    float   rad_s;          // velocidad (rad/s)
    uint32_t sample_ms;     // periodo de muestreo
    uint32_t glitch_ns;     // glitch aplicado (ns)
    int64_t t_us;           // timestamp último update (us)
} encoder_data_t;

esp_err_t encoder_init_pcnt_x4(
    int gpio_a,
    int gpio_b,
    uint32_t glitch_ns,
    uint32_t cpr_x4,            // counts per revolution (PPR*4)
    uint32_t sample_period_ms
);

void encoder_get_data(encoder_data_t *out);

int64_t encoder_get_position_cnt(void);
float encoder_get_rpm(void);

