#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "encoder_pcnt.h"
#include "motor_l298.h"

static const char *TAG = "APP";

#define ENC_A_GPIO      33
#define ENC_B_GPIO      25
#define L298_IN1_GPIO   32
#define L298_IN2_GPIO   27
#define L298_ENA_GPIO   26

#define ENCODER_CPR_X4  1976

static void app_logic_task(void *arg)
{
    (void)arg;

    float cmd = 0.6f;

    while (1) {
        encoder_data_t d;
        encoder_get_data(&d);

        ESP_LOGI(TAG, "pos=%lld cnt | delta=%d | rpm=%.2f | rad/s=%.2f | Ts=%ums",
                 (long long)d.position_cnt, d.delta_cnt, d.rpm, d.rad_s, (unsigned)d.sample_ms);

        ESP_ERROR_CHECK(motor_l298_set(cmd));

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Pins: ENC A=%d B=%d | L298 IN1=%d IN2=%d ENA=%d",
             ENC_A_GPIO, ENC_B_GPIO, L298_IN1_GPIO, L298_IN2_GPIO, L298_ENA_GPIO);

    ESP_ERROR_CHECK(encoder_init_pcnt_x4(
        ENC_A_GPIO,
        ENC_B_GPIO,
        8000,
        ENCODER_CPR_X4,
        20
    ));

    ESP_ERROR_CHECK(motor_l298_init(
        L298_IN1_GPIO,
        L298_IN2_GPIO,
        L298_ENA_GPIO,
        20000
    ));

    xTaskCreate(app_logic_task, "app_logic", 4096, NULL, 4, NULL);
}