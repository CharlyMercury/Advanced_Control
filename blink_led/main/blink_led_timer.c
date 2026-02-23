#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "freertos/timers.h"

#define led1 2

uint8_t led_level = 0;
uint8_t count = 0;
int timer_interval = 1000;
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
    xTimers = xTimerCreate("Timer",  //  Nombre del timer
                           (pdMS_TO_TICKS(timer_interval)), // Periodo del timer en ticks 
                           pdTRUE, // Auto-reload , la función se realiza cada x_ms
                           (void *)timerId, // identificador
                           vTimerCallback); // Función a ejecutar

    if (xTimers == NULL)
    {
        ESP_LOGE(TAG, "The timer was not created");
    }
    else
    {
        if (xTimerStart(xTimers, 0) != pdPASS) // Arranca el timer
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
