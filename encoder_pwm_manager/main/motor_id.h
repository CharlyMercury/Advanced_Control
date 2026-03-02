#pragma once
#include <stdint.h>
#include "esp_err.h"

typedef struct {
    // estimación discreta: w[k+1] = alpha*w[k] + beta*u[k]
    float alpha;
    float beta;

    // equivalente continuo: wdot = -a*w + b*u
    float a;      // [1/s]
    float b;      // [rad/s^2] por unidad de u (PWM)
    float tau;    // [s]
    float K;      // [rad/s] por unidad de u

    // info
    float Ts;     // [s]
    uint32_t Ts_ms;
} motor_id_params_t;

typedef struct motor_id motor_id_t;

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t motor_id_init(motor_id_t **out, uint32_t Ts_ms, float lambda, float p0);
void motor_id_update(motor_id_t *id, float u_cmd, float omega_rad_s);
void motor_id_get(motor_id_t *id, motor_id_params_t *out);
void motor_id_reset(motor_id_t *id, float p0);

#ifdef __cplusplus
}
#endif