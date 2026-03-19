#pragma once
#include <stdint.h>
#include "esp_err.h"

esp_err_t motor_l298_init(int gpio_in1, int gpio_in2, int gpio_ena_pwm, uint32_t pwm_freq_hz);
esp_err_t motor_l298_set(float duty_cmd);