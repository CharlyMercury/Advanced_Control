#pragma once
#include <stdint.h>

typedef struct {
    // Config
    float u_dead;          // zona muerta (ej. 0.50)
    float u_levels[4];     // niveles (magnitudes) >= u_dead
    int   n_levels;        // cuántos niveles usas (1..4)

    float p_zero;          // prob de mandar 0 (0..1), ej 0.2
    uint32_t hold_min;     // en muestras (ticks de Ts)
    uint32_t hold_max;     // en muestras

    // Estado
    uint32_t lfsr;         // PRBS generator
    uint32_t hold_left;    // cuántas muestras faltan para cambiar
    float current_u;       // salida actual
} excitation_prbs_t;

// seed: cualquier entero != 0
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

// Llamar cada Ts (cada iteración del loop de identificación)
float excitation_prbs_step(excitation_prbs_t *e);