#include <math.h>
#include <stddef.h>

#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "encoder_pcnt.h"
#include "motor_l298.h"
#include "motor_id.h"
#include "excitation_prbs.h"

#define ENC_A_GPIO      33
#define ENC_B_GPIO      25
#define L298_IN1_GPIO   32
#define L298_IN2_GPIO   27
#define L298_ENA_GPIO   26

#define ENCODER_CPR_X4  1976

#define TS_MS           20
#define PWM_FREQ_HZ     20000

#define RLS_LAMBDA      0.995f
#define RLS_P0          500.0f

#define OFFLINE_MAX_SAMPLES     2500
#define OFFLINE_TARGET_SAMPLES  1500

#define U_START_POS     0.55f
#define U_START_NEG     0.55f

#define A_SLEW_MAX      0.08f

static float slew_limit_logic(float a_prev, float a_target, float da_max)
{
    float da = a_target - a_prev;

    if (da > da_max) {
        da = da_max;
    }
    if (da < -da_max) {
        da = -da_max;
    }

    return a_prev + da;
}


static float logical_to_pwm(float a)
{
    float mag = fabsf(a);
    float u0;
    float u;

    if (mag < 1e-6f) {
        return 0.0f;
    }

    if (mag > 1.0f) {
        mag = 1.0f;
    }

    u0 = (a >= 0.0f) ? U_START_POS : U_START_NEG;
    u = u0 + (1.0f - u0) * mag;

    return copysignf(u, a);
}


static float pwm_to_effective_input(float u)
{
    float mag = fabsf(u);
    float u0;
    float a;

    if (mag < 1e-6f) {
        return 0.0f;
    }

    u0 = (u >= 0.0f) ? U_START_POS : U_START_NEG;

    if (mag <= u0) {
        return 0.0f;
    }

    a = (mag - u0) / (1.0f - u0);

    if (a > 1.0f) {
        a = 1.0f;
    }

    return copysignf(a, u);
}

static void identification_task(void *arg)
{
    (void)arg;

    motor_id_t *id = NULL;
    ESP_ERROR_CHECK(
        motor_id_init(&id, TS_MS, RLS_LAMBDA, RLS_P0, OFFLINE_MAX_SAMPLES)
    );


    const float levels[] = {
         0.10f,  0.30f,  0.50f,  0.75f,  0.95f,
        -0.10f, -0.30f, -0.50f, -0.75f, -0.95f
    };
    const size_t levels_count = sizeof(levels) / sizeof(levels[0]);

    excitation_prbs_t ex;
    excitation_prbs_init(
        &ex,
        0xC0FFEEu,
        0.0f,                
        levels,
        levels_count,
        0.20f,
        10,
        50
    );

    TickType_t last = xTaskGetTickCount();

    float a_target = 0.0f;   
    float a_cmd = 0.0f;     
    float u_pwm = 0.0f;     
    float u_id = 0.0f;

    size_t printed_samples = 0;
    int finished = 0;

    printf("phi0_omega_k,phi1_u_eff_k,phi2_bias,y_omega_k1\n");

    while (1) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(TS_MS));

        if (!finished) {
            a_target = excitation_prbs_step(&ex);
            a_cmd = slew_limit_logic(a_cmd, a_target, A_SLEW_MAX);
            u_pwm = logical_to_pwm(a_cmd);
        } else {
            a_target = 0.0f;
            a_cmd = 0.0f;
            u_pwm = 0.0f;
        }

        ESP_ERROR_CHECK(motor_l298_set(u_pwm));

        encoder_data_t d;
        encoder_get_data(&d);


        u_id = pwm_to_effective_input(u_pwm);
        motor_id_update(id, u_id, d.rad_s);

        {
            size_t n = motor_id_get_sample_count(id);

            while (printed_samples < n) {
                motor_id_sample_t s;

                if (motor_id_get_sample(id, printed_samples, &s) == ESP_OK) {
                    printf("%.8f,%.8f,1.00000000,%.8f\n",
                           (double)s.phi0,
                           (double)s.phi1,
                           (double)s.y);
                }

                printed_samples++;
            }

            if (!finished && n >= OFFLINE_TARGET_SAMPLES) {
                finished = 1;
                ESP_ERROR_CHECK(motor_l298_set(0.0f));
                vTaskDelete(NULL);
            }
        }
    }
}

void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_NONE);

    ESP_ERROR_CHECK(
        encoder_init_pcnt_x4(
            ENC_A_GPIO,
            ENC_B_GPIO,
            8000,
            ENCODER_CPR_X4,
            TS_MS
        )
    );

    ESP_ERROR_CHECK(
        motor_l298_init(
            L298_IN1_GPIO,
            L298_IN2_GPIO,
            L298_ENA_GPIO,
            PWM_FREQ_HZ
        )
    );

    xTaskCreate(identification_task, "id_task", 4096, NULL, 5, NULL);
}