#pragma once
#include <stdint.h>

typedef struct {
    float theta0;
    float theta1;
    float P00, P01, P10, P11;
    float lambda;
} rls2_t;

void rls2_init(rls2_t *r, float lambda, float p0);
void rls2_reset(rls2_t *r, float p0);
void rls2_update(rls2_t *r, float x0, float x1, float y);