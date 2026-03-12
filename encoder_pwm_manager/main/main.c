#include <stdio.h>
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

#define ENCODER_CPR_X4  1976
#define TS_MS           20
#define PWM_FREQ_HZ     20000

#define PWM_POS         (0.70f)
#define PWM_NEG         (-0.70f)

#define SEG_0A_STEPS    50   
#define SEG_POS_STEPS   250
#define SEG_0B_STEPS    100 
#define SEG_NEG_STEPS   250 
#define SEG_0C_STEPS    100 

static float identification_input(int k)
{
    const int s1 = SEG_0A_STEPS;
    const int s2 = s1 + SEG_POS_STEPS;
    const int s3 = s2 + SEG_0B_STEPS;
    const int s4 = s3 + SEG_NEG_STEPS;
    const int s5 = s4 + SEG_0C_STEPS;

    if (k < s1) return 0.0f;
    if (k < s2) return PWM_POS;
    if (k < s3) return 0.0f;
    if (k < s4) return PWM_NEG;
    if (k < s5) return 0.0f;

    return 0.0f;
}

static void acquire_identification_data_task(void *arg)
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

    printf("k,t_k1_s,u_k,omega_k_rad_s,omega_k1_rad_s\n");

    TickType_t last = xTaskGetTickCount();

    for (int k = 0; k < total_steps; k++) {
        float u_k = identification_input(k);

        ESP_ERROR_CHECK(motor_l298_set(u_k));
        vTaskDelayUntil(&last, pdMS_TO_TICKS(TS_MS));

        encoder_get_data(&d);
        float omega_k1 = d.rad_s;

        float t_k1 = (float)(k + 1) * Ts;

        printf("%d,%.6f,%.6f,%.8f,%.8f\n",
               k,
               (double)t_k1,
               (double)u_k,
               (double)omega_k,
               (double)omega_k1);

        omega_k = omega_k1;
    }

    ESP_ERROR_CHECK(motor_l298_set(0.0f));
    vTaskDelete(NULL);
}

void app_main(void)
{
    ESP_ERROR_CHECK(encoder_init_pcnt_x4(
        ENC_A_GPIO,
        ENC_B_GPIO,
        8000,
        ENCODER_CPR_X4,
        TS_MS
    ));

    ESP_ERROR_CHECK(motor_l298_init(
        L298_IN1_GPIO,
        L298_IN2_GPIO,
        L298_ENA_GPIO,
        PWM_FREQ_HZ
    ));

    xTaskCreate(
        acquire_identification_data_task,
        "acquire_identification_data_task",
        4096,
        NULL,
        5,
        NULL
    );
}