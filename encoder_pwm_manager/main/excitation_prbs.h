#include <stdint.h>

typedef struct {
    float u_dead;
    float u_levels[4];
    int   n_levels;

    float p_zero;
    uint32_t hold_min;
    uint32_t hold_max;

    uint32_t lfsr;
    uint32_t hold_left;
    float current_u;
} excitation_prbs_t;

void excitation_prbs_init(
    excitation_prbs_t *e,
    uint32_t seed,
    float u_dead,
    const float *levels,
    int n_levels,
    float p_zero,
    uint32_t hold_min,
    uint32_t hold_max
);

float excitation_prbs_step(excitation_prbs_t *e);