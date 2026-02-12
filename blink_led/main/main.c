#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "freertos/timers.h"

/*
    5 niveles de log

    - Error (lowest)
    - Warning
    - Info
    - Debug
    - Verbose (highest)
*/

#define led1 2

uint8_t led_level = 0;
uint8_t count = 0;
int timer_interval = 100;
int timerId = 1;
TimerHandle_t xTimers;

static const char *TAG = "Logging";

esp_err_t init_led(void);
esp_err_t blink_led(void);
esp_err_t set_timer(void);

esp_err_t init_led(void)
{
    gpio_reset_pin(led1);
    gpio_set_direction(led1, GPIO_MODE_OUTPUT);
    return ESP_OK;
}

esp_err_t blink_led(void)
{
    led_level = !led_level;
    gpio_set_level(led1, led_level);
    return ESP_OK;
}

void vTimerCallback(TimerHandle_t pxTimer)
{
    ESP_LOGI(TAG, "Event was called from timer");
    blink_led();
}

esp_err_t set_timer(void)
{

    ESP_LOGI(TAG, "Timer init configuration");
    xTimers = xTimerCreate("Timer",
                           (pdMS_TO_TICKS(timer_interval)),
                           pdTRUE,
                           (void *)timerId,
                           vTimerCallback);

    if (xTimers == NULL)
    {
        ESP_LOGE(TAG, "The timer was not created");
    }
    else
    {
        if (xTimerStart(xTimers, 0) != pdPASS)
        {
            ESP_LOGE(TAG, "The timer could not be set into the Active state.");
        }
    }
    return ESP_OK;
}

void app_main(void)
{
    init_led();
    set_timer();
}

/*

    init_led();
    while(1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
        blink_led();
        printf("Led state: %u \n", led_level);

        count += 1;
        if(count>30){
            count=0;
        }
        if (count<10){
            ESP_LOGI(TAG, "Value: %u", count);
        }
        if (count<20&&count>=10){
            ESP_LOGW(TAG, "Value: %u", count);
        }

        if (count<30&&count>=20){
            ESP_LOGE(TAG, "Value: %u", count);
        }
    }

*/