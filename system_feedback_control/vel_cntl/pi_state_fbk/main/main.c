#include <stdio.h>
#include <math.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"

#include "encoder_pcnt.h"
#include "motor_l298.h"

static const char *TAG = "VEL_PI_STATE";

#define ENC_A_GPIO              33
#define ENC_B_GPIO              25

#define L298_IN1_GPIO           32
#define L298_IN2_GPIO           27
#define L298_ENA_GPIO           26

#define ENCODER_CPR_X4          1976
#define ENCODER_GLITCH_NS       8000

#define TS_MS                   20
#define PWM_FREQ_HZ             20000

// Constantes del modelo identificado
#define MODEL_A                 11.11281274f
#define MODEL_B               (-165.65653960f)
#define MODEL_C                 0.37449236f

// Ganancias del control:
// u = ( -k1*e + k2*integral(e) + c - a*r ) / b
#define K1_ERR                  5.0f
#define K2_INT                  10.0f

// Referencia
#define OMEGA_REF_RAD_S         8.0f

// Filtros
#define OMEGA_LPF_ALPHA         0.85f

/* Saturación del estado integral */
#define E0_MIN                 (-20.0f)
#define E0_MAX                  (20.0f)

/* Saturación de la entrada efectiva del modelo */
#define U_EFF_MIN              (-1.0f)
#define U_EFF_MAX               (1.0f)

/* Zona muerta identificada del motor */
#define U_START_POS             0.55f
#define U_START_NEG             0.55f

/* Banda de reposo */
#define REF_STOP_EPS            0.10f
#define OMEGA_STOP_EPS          0.20f

/* =========================================================
 * Si gira al revés, cambia SOLO UNO:
 * - MOTOR_CMD_SIGN
 * - ENCODER_SIGN
 * ========================================================= */
#define MOTOR_CMD_SIGN          1.0f
#define ENCODER_SIGN            -1.0f

// Utilerías
static inline float clampf(float x, float xmin, float xmax)
{
    if (x < xmin) {
        return xmin;
    }
    if (x > xmax) {
        return xmax;
    }
    return x;
}

/*
 * Convierte la entrada efectiva del modelo u_eff a PWM real,
 * compensando la zona muerta del motor.
 *
 * u_eff = 0      -> pwm = 0
 * |u_eff| > 0    -> pwm >= umbral de arranque
 */
static float effective_input_to_pwm(float u_eff)
{
    float mag = fabsf(u_eff);
    float u0;
    float pwm_mag;

    if (mag < 1e-6f) {
        return 0.0f;
    }

    u0 = (u_eff >= 0.0f) ? U_START_POS : U_START_NEG;
    mag = clampf(mag, 0.0f, 1.0f);
    pwm_mag = u0 + (1.0f - u0) * mag;
    pwm_mag = clampf(pwm_mag, 0.0f, 1.0f);

    return copysignf(pwm_mag, u_eff);
}

// Tarea principal - control
static void velocity_pi_state_task(void *arg)
{
    (void)arg;

    const float Ts = ((float)TS_MS) / 1000.0f;
    TickType_t last = xTaskGetTickCount();

    encoder_data_t enc = {0};

    float t_s = 0.0f;

    /* Señales medidas */
    float omega_meas = 0.0f;
    float omega_filt = 0.0f;
    float omega_ref  = OMEGA_REF_RAD_S;

    // Variables de estado del controlador
    float e0 = 0.0f;   /* integral del error */
    float e1 = 0.0f;   /* error instantáneo */

    // Variables del control
    float v_ff = 0.0f;
    float v_fb = 0.0f;
    float u_eff = 0.0f;
    float pwm_cmd = 0.0f;

    ESP_ERROR_CHECK(motor_l298_set(0.0f));
    vTaskDelay(pdMS_TO_TICKS(200));

    printf("t_s,omega_ref_rad_s,omega_meas_rad_s,omega_filt_rad_s,"
           "e0_int_error,e1_error,v_ff,v_fb,u_eff,pwm_cmd,delta_cnt,position_cnt\n");

    while (1) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(TS_MS));
        t_s += Ts;

        // Lectura del encoder
        encoder_get_data(&enc);
        omega_meas = ENCODER_SIGN * enc.rad_s;

        /* Filtro paso bajo simple */
        omega_filt = (OMEGA_LPF_ALPHA * omega_filt) +
                     ((1.0f - OMEGA_LPF_ALPHA) * omega_meas);

        // Referencia
        omega_ref = OMEGA_REF_RAD_S;

        // Estado aumentado
        e1 = omega_ref - omega_filt;
        e0 += Ts * e1; // Integral del error
        e0 = clampf(e0, E0_MIN, E0_MAX);

        /* -------------------------------------------------
         * Ley de control:
         * u = ( -k1*e - k2*integral(e) + c - a*r ) / b
         * ------------------------------------------------- */
        v_ff = (MODEL_C - (MODEL_A * omega_ref)) / MODEL_B;
        v_fb = ((-K1_ERR * e1) + (-K2_INT * e0)) / MODEL_B;

        u_eff = v_ff + v_fb;
        u_eff = clampf(u_eff, U_EFF_MIN, U_EFF_MAX);

        // Bloque de reinicio de la integral del error
        if ((fabsf(omega_ref) < REF_STOP_EPS) &&
            (fabsf(omega_filt) < OMEGA_STOP_EPS)) {
            u_eff = 0.0f;

            // Reinicio de la integral del error
            if (fabsf(e0) < 0.5f) {
                e0 = 0.0f;
            }
        }

        // Conversión de u_eff a PWM
        pwm_cmd = effective_input_to_pwm(u_eff);
        pwm_cmd *= MOTOR_CMD_SIGN;
        pwm_cmd = clampf(pwm_cmd, -1.0f, 1.0f);

        // Señal de control aplicada al motor
        ESP_ERROR_CHECK(motor_l298_set(pwm_cmd));

        // Log
        printf("%.3f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%"
               PRId32 ",%" PRId64 "\n",
               (double)t_s,
               (double)omega_ref,
               (double)omega_meas,
               (double)omega_filt,
               (double)e0,
               (double)e1,
               (double)v_ff,
               (double)v_fb,
               (double)u_eff,
               (double)pwm_cmd,
               enc.delta_cnt,
               enc.position_cnt);
    }
}

// Programa principal
void app_main(void)
{
    ESP_LOGI(TAG, "Inicializando encoder...");
    ESP_ERROR_CHECK(
        encoder_init_pcnt_x4(
            ENC_A_GPIO,
            ENC_B_GPIO,
            ENCODER_GLITCH_NS,
            ENCODER_CPR_X4,
            TS_MS
        )
    );

    ESP_LOGI(TAG, "Inicializando L298...");
    ESP_ERROR_CHECK(
        motor_l298_init(
            L298_IN1_GPIO,
            L298_IN2_GPIO,
            L298_ENA_GPIO,
            PWM_FREQ_HZ
        )
    );

    xTaskCreate(
        velocity_pi_state_task,
        "velocity_pi_state_task",
        4096,
        NULL,
        5,
        NULL
    );
}