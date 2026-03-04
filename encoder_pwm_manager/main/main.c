#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "encoder_pcnt.h"
#include "motor_l298.h"
#include "motor_id.h"

#include "excitation_prbs.h"

static const char *TAG = "APP";

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

#define U_DEAD          0.50f

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
    ESP_ERROR_CHECK(motor_id_init(&id, TS_MS, RLS_LAMBDA, RLS_P0));

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

    ESP_LOGI(TAG, "PE excitation ON. deadzone=%.2f, levels=[0.55 0.70 0.85], p_zero=0.20", (double)U_DEAD);

    while (1) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(TS_MS));

        u_target = excitation_prbs_step(&ex);

        u_cmd = slew_limit(u_cmd, u_target, DU_MAX);

        ESP_ERROR_CHECK(motor_l298_set(u_cmd));

        encoder_data_t d;
        encoder_get_data(&d);

        motor_id_update(id, u_cmd, d.rad_s);

        static uint32_t acc = 0;
        acc += TS_MS;
        if (acc >= 200) {
            acc = 0;

            motor_id_params_t p;
            motor_id_get(id, &p);

            ESP_LOGI(TAG,
                "u=%.2f w=%.2f rad/s | alpha=%.4f beta=%.4f | a=%.3f 1/s tau=%.3fs | K=%.2f rad/s/u",
                (double)u_cmd, (double)d.rad_s,
                (double)p.alpha, (double)p.beta,
                (double)p.a, (double)p.tau, (double)p.K
            );
        }
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Init: ENC A=%d B=%d | L298 IN1=%d IN2=%d ENA=%d",
             ENC_A_GPIO, ENC_B_GPIO, L298_IN1_GPIO, L298_IN2_GPIO, L298_ENA_GPIO);

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