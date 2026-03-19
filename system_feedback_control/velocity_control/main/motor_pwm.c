#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_private/esp_clk.h"
#include "esp_timer.h"

#include "driver/gpio.h"
#include "driver/ledc.h"

#include "motor_pwm.h"

static const char *TAG = "L298";

typedef struct {
    float cmd;   // [-1..+1]
} motor_cmd_t;

static QueueHandle_t g_q;
static int g_in1, g_in2, g_ena;

static ledc_channel_config_t g_ch;
static ledc_timer_config_t g_tmr;
static uint32_t g_max_duty;

// Modo de stop:
// 0 = coast (IN1=0 IN2=0)
// 1 = brake (IN1=1 IN2=1)
#define MOTOR_STOP_BRAKE 0

static void set_dir_forward(void)
{
    gpio_set_level(g_in1, 1);
    gpio_set_level(g_in2, 0);
}

static void set_dir_reverse(void)
{
    gpio_set_level(g_in1, 0);
    gpio_set_level(g_in2, 1);
}

static void set_stop(void)
{
#if MOTOR_STOP_BRAKE
    gpio_set_level(g_in1, 1);
    gpio_set_level(g_in2, 1);
#else
    gpio_set_level(g_in1, 0);
    gpio_set_level(g_in2, 0);
#endif
}

static void pwm_apply(float duty_0_to_1)
{
    if (duty_0_to_1 < 0.0f) duty_0_to_1 = 0.0f;
    if (duty_0_to_1 > 1.0f) duty_0_to_1 = 1.0f;

    uint32_t duty_int = (uint32_t)(duty_0_to_1 * (float)g_max_duty);
    ledc_set_duty(g_ch.speed_mode, g_ch.channel, duty_int);
    ledc_update_duty(g_ch.speed_mode, g_ch.channel);
}

static void motor_task(void *arg)
{
    (void)arg;
    motor_cmd_t m;

    while (1) {
        if (xQueueReceive(g_q, &m, portMAX_DELAY) == pdTRUE) {
            float cmd = m.cmd;
            if (cmd > 1.0f) cmd = 1.0f;
            if (cmd < -1.0f) cmd = -1.0f;

            if (fabsf(cmd) < 0.0001f) {
                set_stop();
                pwm_apply(0.0f);
                ESP_LOGI(TAG, "STOP");
                continue;
            }

            if (cmd > 0) set_dir_forward();
            else        set_dir_reverse();

            pwm_apply(fabsf(cmd));
            ESP_LOGI(TAG, "cmd=%.3f (dir=%s duty=%.3f)", cmd, (cmd>0)?"FWD":"REV", fabsf(cmd));
        }
    }
}

esp_err_t motor_l298_init(int gpio_in1, int gpio_in2, int gpio_ena_pwm, uint32_t pwm_freq_hz)
{
    g_in1 = gpio_in1;
    g_in2 = gpio_in2;
    g_ena = gpio_ena_pwm;

    // IN1/IN2 como salidas
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << g_in1) | (1ULL << g_in2),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io), TAG, "gpio_config");
    set_stop();

    // LEDC timer
    g_tmr.speed_mode = LEDC_LOW_SPEED_MODE;
    g_tmr.timer_num = LEDC_TIMER_0;
    g_tmr.duty_resolution = LEDC_TIMER_11_BIT; // OK para 20kHz
    g_tmr.freq_hz = pwm_freq_hz;
    g_tmr.clk_cfg = LEDC_AUTO_CLK;
    ESP_RETURN_ON_ERROR(ledc_timer_config(&g_tmr), TAG, "ledc_timer");

    g_max_duty = (1U << g_tmr.duty_resolution) - 1;

    // LEDC channel en ENA
    memset(&g_ch, 0, sizeof(g_ch));
    g_ch.gpio_num = g_ena;
    g_ch.speed_mode = LEDC_LOW_SPEED_MODE;
    g_ch.channel = LEDC_CHANNEL_0;
    g_ch.intr_type = LEDC_INTR_DISABLE;
    g_ch.timer_sel = LEDC_TIMER_0;
    g_ch.duty = 0;
    g_ch.hpoint = 0;
    ESP_RETURN_ON_ERROR(ledc_channel_config(&g_ch), TAG, "ledc_channel");

    // Cola: tamaño 1 (si mandas 100 comandos, solo importa el último)
    g_q = xQueueCreate(1, sizeof(motor_cmd_t));
    if (!g_q) return ESP_ERR_NO_MEM;

    xTaskCreate(motor_task, "motor_task", 2048, NULL, 5, NULL);

    ESP_LOGI(TAG, "L298 ready: IN1=%d IN2=%d ENA(PWM)=%d freq=%uHz",
             g_in1, g_in2, g_ena, (unsigned)pwm_freq_hz);

    return ESP_OK;
}

esp_err_t motor_l298_set(float duty_cmd)
{
    if (!g_q) return ESP_ERR_INVALID_STATE;

    motor_cmd_t m = { .cmd = duty_cmd };
    if (xQueueOverwrite(g_q, &m) != pdPASS) return ESP_FAIL;

    return ESP_OK;
}