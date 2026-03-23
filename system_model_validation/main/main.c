#include <stdio.h>
#include <math.h>

#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "encoder_pcnt.h"
#include "motor_l298.h"

#define ENC_A_GPIO      33
#define ENC_B_GPIO      25
#define L298_IN1_GPIO   32
#define L298_IN2_GPIO   27
#define L298_ENA_GPIO   26

#define ENC_SWAP_AB     1
#define ENC_INVERT_DIR  0

#define ENCODER_CPR_X4  1976
#define TS_MS           20
#define PWM_FREQ_HZ     20000

/* Entrada oficial del experimento en u_eff */
#define U_EFF_POS       (0.75f)
#define U_EFF_NEG       (-0.75f)

#define SEG_0A_STEPS    50
#define SEG_POS_STEPS   250
#define SEG_0B_STEPS    100
#define SEG_NEG_STEPS   250
#define SEG_0C_STEPS    100

#define U_START_POS     0.55f
#define U_START_NEG     0.55f

static float effective_to_pwm(float a)
{
    float mag = fabsf(a);
    float u0;

    if (mag < 1e-6f) {
        return 0.0f;
    }

    if (mag > 1.0f) {
        mag = 1.0f;
    }

    u0 = (a >= 0.0f) ? U_START_POS : U_START_NEG;

    return copysignf(u0 + (1.0f - u0) * mag, a);
}

static float validation_input_ueff(int k)
{
    const int s1 = SEG_0A_STEPS;
    const int s2 = s1 + SEG_POS_STEPS;
    const int s3 = s2 + SEG_0B_STEPS;
    const int s4 = s3 + SEG_NEG_STEPS;
    const int s5 = s4 + SEG_0C_STEPS;

    if (k < s1) return 0.0f;
    if (k < s2) return U_EFF_POS;
    if (k < s3) return 0.0f;
    if (k < s4) return U_EFF_NEG;
    if (k < s5) return 0.0f;

    return 0.0f;
}

static void acquire_validation_data_task(void *arg)
{
    (void)arg;

    const float Ts = ((float)TS_MS) / 1000.0f;
    const int total_steps = SEG_0A_STEPS + SEG_POS_STEPS + SEG_0B_STEPS +
                            SEG_NEG_STEPS + SEG_0C_STEPS;

    esp_log_level_set("*", ESP_LOG_NONE);

    ESP_ERROR_CHECK(motor_l298_set(0.0f));
    vTaskDelay(pdMS_TO_TICKS(200));

    encoder_data_t d;
    encoder_get_data(&d);
    float omega_k = d.rad_s;

    /* Ya no usamos pwm como entrada oficial del log */
    printf("k,t_k1_s,u_eff_k,omega_k_rad_s,omega_k1_rad_s\n");

    TickType_t last = xTaskGetTickCount();

    for (int k = 0; k < total_steps; k++) {
        float u_eff_k = validation_input_ueff(k);
        float pwm_k = effective_to_pwm(u_eff_k);

        ESP_ERROR_CHECK(motor_l298_set(pwm_k));
        vTaskDelayUntil(&last, pdMS_TO_TICKS(TS_MS));

        encoder_get_data(&d);
        float omega_k1 = d.rad_s;

        float t_k1 = (float)(k + 1) * Ts;

        printf("%d,%.6f,%.6f,%.8f,%.8f\n",
               k,
               (double)t_k1,
               (double)u_eff_k,
               (double)omega_k,
               (double)omega_k1);

        omega_k = omega_k1;
    }

    ESP_ERROR_CHECK(motor_l298_set(0.0f));
    vTaskDelete(NULL);
}

void app_main(void)
{
    ESP_ERROR_CHECK(
        encoder_init_pcnt_x4(
            ENC_A_GPIO,
            ENC_B_GPIO,
            8000,
            ENCODER_CPR_X4,
            TS_MS,
            ENC_SWAP_AB,
            ENC_INVERT_DIR
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

    xTaskCreate(
        acquire_validation_data_task,
        "acquire_validation_data_task",
        4096,
        NULL,
        5,
        NULL
    );
}