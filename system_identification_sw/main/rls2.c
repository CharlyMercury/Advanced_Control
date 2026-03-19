#include "rls2.h"

void rls2_init(rls2_t *r, float lambda, float p0)
{
    r->theta0 = 0.0f;
    r->theta1 = 0.0f;

    r->P00 = p0; r->P01 = 0.0f;
    r->P10 = 0.0f; r->P11 = p0;

    r->lambda = lambda;
}

void rls2_reset(rls2_t *r, float p0)
{
    r->theta0 = 0.0f;
    r->theta1 = 0.0f;

    r->P00 = p0; r->P01 = 0.0f;
    r->P10 = 0.0f; r->P11 = p0;
}

void rls2_update(rls2_t *r, float x0, float x1, float y)
{
    float Pphi0 = r->P00 * x0 + r->P01 * x1;
    float Pphi1 = r->P10 * x0 + r->P11 * x1;

    float denom = r->lambda + (x0 * Pphi0 + x1 * Pphi1);
    if (denom < 1e-9f) denom = 1e-9f;

    float K0 = Pphi0 / denom;
    float K1 = Pphi1 / denom;

    float yhat = x0 * r->theta0 + x1 * r->theta1;
    float e = y - yhat;

    r->theta0 += K0 * e;
    r->theta1 += K1 * e;

    float phiTP0 = x0 * r->P00 + x1 * r->P10;
    float phiTP1 = x0 * r->P01 + x1 * r->P11;

    float P00n = r->P00 - K0 * phiTP0;
    float P01n = r->P01 - K0 * phiTP1;
    float P10n = r->P10 - K1 * phiTP0;
    float P11n = r->P11 - K1 * phiTP1;

    float invL = 1.0f / r->lambda;
    r->P00 = P00n * invL;
    r->P01 = P01n * invL;
    r->P10 = P10n * invL;
    r->P11 = P11n * invL;
}