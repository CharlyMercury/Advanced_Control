#pragma once

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

typedef struct {
    uint32_t k;
    float t_s;
    float phi0;
    float phi1;
    float y;
} motor_id_sample_t;

typedef struct {
    float alpha;
    float beta;

    float a;
    float b;
    float tau;
    float K;

    float Ts;
    uint32_t Ts_ms;

    size_t offline_samples;
    size_t offline_capacity;
} motor_id_params_t;

typedef struct motor_id motor_id_t;

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t motor_id_init(
    motor_id_t **out,
    uint32_t Ts_ms,
    float lambda,
    float p0,
    size_t offline_capacity
);

void motor_id_update(motor_id_t *id, float u_cmd, float omega_rad_s);
void motor_id_get(motor_id_t *id, motor_id_params_t *out);
void motor_id_reset(motor_id_t *id, float p0);

size_t motor_id_get_sample_count(motor_id_t *id);
esp_err_t motor_id_get_sample(motor_id_t *id, size_t index, motor_id_sample_t *out);
void motor_id_clear_samples(motor_id_t *id);
void motor_id_dump_csv(motor_id_t *id);

#ifdef __cplusplus
}
#endif