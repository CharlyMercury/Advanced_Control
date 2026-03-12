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

#define U_DEAD                  0.50f
#define OFFLINE_MAX_SAMPLES     2500
#define OFFLINE_TARGET_SAMPLES  1500

static float slew_limit(float u_prev, float u_target, float du_max)
{
    float du = u_target - u_prev;
    if (du > du_max) du = du_max;
    if (du < -du_max) du = -du_max;
    return u_prev + du;
}

static void identification_task(void *arg)
{
    (void)arg;

    motor_id_t *id = NULL;
    ESP_ERROR_CHECK(motor_id_init(&id, TS_MS, RLS_LAMBDA, RLS_P0, OFFLINE_MAX_SAMPLES));

    const float levels[] = { 0.55f, 0.70f, 0.85f };

    excitation_prbs_t ex;
    excitation_prbs_init(
        &ex,
        0xC0FFEEu,
        U_DEAD,
        levels,
        3,
        0.20f,
        10,
        50
    );

    TickType_t last = xTaskGetTickCount();

    float u_cmd = 0.0f;
    float u_target = 0.0f;
    const float DU_MAX = 0.04f;

    size_t printed_samples = 0;
    int finished = 0;

    printf("phi0_omega_k,phi1_u_k,phi2_bias,y_omega_k1\n");

    while (1) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(TS_MS));

        if (!finished) {
            u_target = excitation_prbs_step(&ex);
            u_cmd = slew_limit(u_cmd, u_target, DU_MAX);
        } else {
            u_target = 0.0f;
            u_cmd = 0.0f;
        }

        ESP_ERROR_CHECK(motor_l298_set(u_cmd));

        encoder_data_t d;
        encoder_get_data(&d);

        ESP_ERROR_CHECK(motor_id_update(id, u_cmd, d.rad_s));

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

void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_NONE);

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

    xTaskCreate(identification_task, "id_task", 4096, NULL, 5, NULL);
}