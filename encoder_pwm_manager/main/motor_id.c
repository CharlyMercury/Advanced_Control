#include "motor_id.h"
#include "rls2.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

struct motor_id {
    rls2_t rls;

    float Ts;
    uint32_t Ts_ms;

    float w_k;
    float u_k;
    int have_prev;

    motor_id_params_t p;
};

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static void derive_params(struct motor_id *id)
{
    float alpha = id->rls.theta0;
    float beta  = id->rls.theta1;

    alpha = clampf(alpha, 0.001f, 0.9999f);

    float Ts = id->Ts;
    float a = -logf(alpha) / Ts;

    float one_minus_alpha = (1.0f - alpha);
    if (fabsf(one_minus_alpha) < 1e-6f) {
        one_minus_alpha = (one_minus_alpha >= 0.0f) ? 1e-6f : -1e-6f;
    }

    float b = a * beta / one_minus_alpha;

    float tau = (a > 1e-6f) ? (1.0f / a) : 1e6f;
    float K = b / a;

    id->p.alpha = alpha;
    id->p.beta  = beta;
    id->p.a     = a;
    id->p.b     = b;
    id->p.tau   = tau;
    id->p.K     = K;
}

esp_err_t motor_id_init(motor_id_t **out, uint32_t Ts_ms, float lambda, float p0)
{
    if (!out || Ts_ms == 0) return ESP_ERR_INVALID_ARG;

    struct motor_id *id = (struct motor_id *)calloc(1, sizeof(*id));
    if (!id) return ESP_ERR_NO_MEM;

    id->Ts_ms = Ts_ms;
    id->Ts = ((float)Ts_ms) / 1000.0f;

    rls2_init(&id->rls, lambda, p0);

    id->have_prev = 0;

    id->p.Ts = id->Ts;
    id->p.Ts_ms = Ts_ms;
    id->p.alpha = 0.9f;
    id->p.beta  = 0.0f;
    id->p.a     = 1.0f;
    id->p.b     = 0.0f;
    id->p.tau   = 1.0f;
    id->p.K     = 0.0f;

    *out = (motor_id_t *)id;
    return ESP_OK;
}

void motor_id_reset(motor_id_t *base, float p0)
{
    struct motor_id *id = (struct motor_id *)base;
    if (!id) return;

    rls2_reset(&id->rls, p0);
    id->have_prev = 0;
}

void motor_id_update(motor_id_t *base, float u_cmd, float omega_rad_s)
{
    struct motor_id *id = (struct motor_id *)base;
    if (!id) return;

    if (!id->have_prev) {
        id->w_k = omega_rad_s;
        id->u_k = u_cmd;
        id->have_prev = 1;
        return;
    }

    float x0 = id->w_k;
    float x1 = id->u_k;
    float y  = omega_rad_s;

    if (fabsf(x1) > 0.02f || fabsf(x0) > 0.2f || fabsf(y) > 0.2f) {
        rls2_update(&id->rls, x0, x1, y);
        derive_params(id);
    }

    id->w_k = omega_rad_s;
    id->u_k = u_cmd;
}

void motor_id_get(motor_id_t *base, motor_id_params_t *out)
{
    struct motor_id *id = (struct motor_id *)base;
    if (!id || !out) return;
    *out = id->p;
}