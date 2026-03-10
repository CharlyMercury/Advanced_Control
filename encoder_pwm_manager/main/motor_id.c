#include "motor_id.h"
#include "rls2.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

struct motor_id {
    rls2_t rls;

    float Ts;
    uint32_t Ts_ms;

    float w_k;
    float u_k;
    int have_prev;

    motor_id_sample_t *samples;
    size_t sample_count;
    size_t sample_capacity;
    uint32_t sample_index;

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
    id->p.offline_samples = id->sample_count;
    id->p.offline_capacity = id->sample_capacity;
}

static void store_sample(struct motor_id *id, float x0, float x1, float y)
{
    if (id->samples && id->sample_count < id->sample_capacity) {
        motor_id_sample_t *s = &id->samples[id->sample_count++];

        s->k = id->sample_index;
        s->t_s = ((float)id->sample_index) * id->Ts;
        s->phi0 = x0;
        s->phi1 = x1;
        s->y = y;

        id->p.offline_samples = id->sample_count;
    }

    id->sample_index++;
}

esp_err_t motor_id_init(
    motor_id_t **out,
    uint32_t Ts_ms,
    float lambda,
    float p0,
    size_t offline_capacity
)
{
    if (!out || Ts_ms == 0) return ESP_ERR_INVALID_ARG;

    struct motor_id *id = (struct motor_id *)calloc(1, sizeof(*id));
    if (!id) return ESP_ERR_NO_MEM;

    id->Ts_ms = Ts_ms;
    id->Ts = ((float)Ts_ms) / 1000.0f;

    if (offline_capacity > 0) {
        id->samples = (motor_id_sample_t *)calloc(offline_capacity, sizeof(motor_id_sample_t));
        if (!id->samples) {
            free(id);
            return ESP_ERR_NO_MEM;
        }
        id->sample_capacity = offline_capacity;
    }

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
    id->p.offline_samples = 0;
    id->p.offline_capacity = id->sample_capacity;

    *out = (motor_id_t *)id;
    return ESP_OK;
}

void motor_id_reset(motor_id_t *base, float p0)
{
    struct motor_id *id = (struct motor_id *)base;
    if (!id) return;

    rls2_reset(&id->rls, p0);

    id->have_prev = 0;
    id->w_k = 0.0f;
    id->u_k = 0.0f;

    id->sample_count = 0;
    id->sample_index = 0;

    id->p.offline_samples = 0;
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

    store_sample(id, x0, x1, y);

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

size_t motor_id_get_sample_count(motor_id_t *base)
{
    struct motor_id *id = (struct motor_id *)base;
    if (!id) return 0;
    return id->sample_count;
}

esp_err_t motor_id_get_sample(motor_id_t *base, size_t index, motor_id_sample_t *out)
{
    struct motor_id *id = (struct motor_id *)base;
    if (!id || !out) return ESP_ERR_INVALID_ARG;
    if (index >= id->sample_count) return ESP_ERR_INVALID_ARG;

    *out = id->samples[index];
    return ESP_OK;
}

void motor_id_clear_samples(motor_id_t *base)
{
    struct motor_id *id = (struct motor_id *)base;
    if (!id) return;

    id->sample_count = 0;
    id->sample_index = 0;
    id->p.offline_samples = 0;
}

void motor_id_dump_csv(motor_id_t *base)
{
    struct motor_id *id = (struct motor_id *)base;
    if (!id) return;

    printf("BEGIN_OFFLINE_ID_DATA\n");
    printf("# Ts_s,%.6f\n", (double)id->Ts);
    printf("# model,omega_k1 = alpha*omega_k + beta*u_k\n");
    printf("# columns,k,t_s,phi0_omega_k,phi1_u_k,y_omega_k1\n");
    printf("k,t_s,phi0_omega_k,phi1_u_k,y_omega_k1\n");

    for (size_t i = 0; i < id->sample_count; i++) {
        const motor_id_sample_t *s = &id->samples[i];
        printf("%lu,%.6f,%.8f,%.8f,%.8f\n",
               (unsigned long)s->k,
               (double)s->t_s,
               (double)s->phi0,
               (double)s->phi1,
               (double)s->y);
    }

    printf("END_OFFLINE_ID_DATA\n");
}