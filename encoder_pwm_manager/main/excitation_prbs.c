#include "excitation_prbs.h"
#include <math.h>

static uint32_t lfsr_step(uint32_t x)
{
    // LFSR 32-bit (polinomio típico); suficiente para PRBS
    // taps: 32, 22, 2, 1 (variantes existen; esta funciona bien)
    uint32_t lsb = x & 1u;
    x >>= 1;
    if (lsb) x ^= 0x80200003u;
    return x ? x : 0xABCDEu; // evitar quedar en 0
}

static float rand01(excitation_prbs_t *e)
{
    e->lfsr = lfsr_step(e->lfsr);
    // 24 bits -> [0,1)
    return (float)(e->lfsr & 0x00FFFFFFu) / (float)0x01000000u;
}

static uint32_t rand_range(excitation_prbs_t *e, uint32_t lo, uint32_t hi)
{
    if (hi <= lo) return lo;
    float r = rand01(e);
    uint32_t span = hi - lo + 1;
    return lo + (uint32_t)(r * (float)span);
}

void excitation_prbs_init(
    excitation_prbs_t *e,
    uint32_t seed,
    float u_dead,
    const float *levels,
    int n_levels,
    float p_zero,
    uint32_t hold_min,
    uint32_t hold_max
)
{
    if (!e) return;

    e->lfsr = seed ? seed : 0x12345678u;

    e->u_dead = u_dead;
    e->p_zero = (p_zero < 0.0f) ? 0.0f : (p_zero > 0.9f ? 0.9f : p_zero);

    e->hold_min = (hold_min < 1) ? 1 : hold_min;
    e->hold_max = (hold_max < e->hold_min) ? e->hold_min : hold_max;

    if (n_levels < 1) n_levels = 1;
    if (n_levels > 4) n_levels = 4;

    e->n_levels = n_levels;
    for (int i = 0; i < n_levels; i++) {
        float v = levels[i];
        // Fuerza niveles >= u_dead + margen
        if (v < (u_dead + 0.02f)) v = u_dead + 0.02f;
        if (v > 1.0f) v = 1.0f;
        e->u_levels[i] = v;
    }

    e->hold_left = 0;
    e->current_u = 0.0f;
}

float excitation_prbs_step(excitation_prbs_t *e)
{
    if (!e) return 0.0f;

    if (e->hold_left > 0) {
        e->hold_left--;
        return e->current_u;
    }

    // Elegir nuevo hold
    e->hold_left = rand_range(e, e->hold_min, e->hold_max);

    // Con prob p_zero: 0 (útil para estimar la dinámica de decaimiento "a")
    if (rand01(e) < e->p_zero) {
        e->current_u = 0.0f;
        return e->current_u;
    }

    // Elegir nivel (magnitud)
    int idx = 0;
    if (e->n_levels > 1) {
        float r = rand01(e);
        idx = (int)(r * (float)e->n_levels);
        if (idx >= e->n_levels) idx = e->n_levels - 1;
    }
    float mag = e->u_levels[idx];

    // Elegir signo (para PE más rica)
    // Si no quieres reversa, aquí pon siempre +1
    float sign = (rand01(e) < 0.5f) ? 1.0f : -1.0f;

    e->current_u = sign * mag;
    return e->current_u;
}