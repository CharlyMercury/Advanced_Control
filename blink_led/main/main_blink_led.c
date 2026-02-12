#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#include "esp_log.h"
#include "esp_err.h"

#define LED1 2

static const char *TAG = "Logging";

static uint8_t led_level = 0;
static uint8_t count = 0;

esp_err_t init_led(void)
{
    gpio_reset_pin(LED1);
    esp_err_t err = gpio_set_direction(LED1, GPIO_MODE_OUTPUT);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "LED initialized on GPIO %d", LED1);
        ESP_LOGD(TAG, "Debug: init_led() returned ESP_OK");
    } else {
        ESP_LOGE(TAG, "Failed to init LED (err=%s)", esp_err_to_name(err));
    }

    return err;
}

esp_err_t blink_led(void)
{
    led_level = !led_level;
    ESP_LOGD(TAG, "Debug: toggling LED, next level=%u", led_level);
    return gpio_set_level(LED1, led_level);
}

void app_main(void)
{
    // ESP_LOG_ERROR   -> solo E
    // ESP_LOG_WARN    -> W y E
    // ESP_LOG_INFO    -> I, W y E
    // ESP_LOG_DEBUG   -> D, I, W y E
    esp_log_level_set(TAG, ESP_LOG_DEBUG);

    ESP_ERROR_CHECK(init_led());

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));

        ESP_ERROR_CHECK(blink_led());
        ESP_LOGI(TAG, "LED state: %u", led_level);

        count++;
        if (count > 30) count = 0;

        ESP_LOGD(TAG, "Debug: count=%u (raw)", count);

        if (count < 10) {
            ESP_LOGI(TAG, "Value: %u", count);
        } else if (count < 20) {
            ESP_LOGW(TAG, "Value: %u", count);
        } else {
            ESP_LOGE(TAG, "Value: %u", count);
        }
    }
}
