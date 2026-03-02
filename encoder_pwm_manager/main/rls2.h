#pragma once
#include <stdint.h>

typedef struct {
    // theta = [a0, a1] en y = a0*x0 + a1*x1
    float theta0;
    float theta1;

    // P (2x2)
    float P00, P01, P10, P11;

    float lambda;   // factor de olvido (0.98..1.0)
} rls2_t;

void rls2_init(rls2_t *r, float lambda, float p0);
void rls2_reset(rls2_t *r, float p0);
void rls2_update(rls2_t *r, float x0, float x1, float y);