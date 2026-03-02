

/* =========================================================
 * File: main/motor_l298.h
 * ========================================================= */
#pragma once
#include <stdint.h>
#include "esp_err.h"

// cmd: rango [-1.0 .. +1.0]
//  + => forward, - => reverse, 0 => stop (coast/brake según config)
esp_err_t motor_l298_init(int gpio_in1, int gpio_in2, int gpio_ena_pwm, uint32_t pwm_freq_hz);
esp_err_t motor_l298_set(float cmd);
