#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_private/esp_clk.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "encoder_pcnt.h"

static const char *TAG = "ENC";

typedef struct {
    pcnt_unit_handle_t unit;
    pcnt_channel_handle_t ch_a;
    pcnt_channel_handle_t ch_b;
    uint32_t glitch_ns_use;

    uint32_t cpr;
    uint32_t sample_ms;

    int last_count;

    encoder_data_t data;
    SemaphoreHandle_t mtx;
} enc_ctx_t;

static enc_ctx_t g_enc;

static uint32_t pcnt_hw_max_glitch_ns(void)
{
    uint32_t apb = esp_clk_apb_freq();
    uint32_t thres_max = 1023;
    uint32_t ns = (uint32_t)((1000000000ULL * thres_max) / apb);
    return ns + 50;
}

static esp_err_t encoder_gpio_init(int gpio_a, int gpio_b)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << gpio_a) | (1ULL << gpio_b),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    return gpio_config(&io);
}

static esp_err_t pcnt_qdec_init(enc_ctx_t *q, int gpio_a, int gpio_b, uint32_t glitch_ns)
{
    pcnt_unit_config_t unit_cfg = {
        .high_limit = 32767,
        .low_limit  = -32768,
    };
    ESP_RETURN_ON_ERROR(pcnt_new_unit(&unit_cfg, &q->unit), TAG, "pcnt_new_unit");

    uint32_t hw_max = pcnt_hw_max_glitch_ns();
    q->glitch_ns_use = (glitch_ns > hw_max) ? hw_max : glitch_ns;

    pcnt_glitch_filter_config_t flt = { .max_glitch_ns = q->glitch_ns_use };
    ESP_RETURN_ON_ERROR(pcnt_unit_set_glitch_filter(q->unit, &flt), TAG, "set_glitch");

    pcnt_chan_config_t chA_cfg = {
        .edge_gpio_num  = gpio_a,
        .level_gpio_num = gpio_b,
    };
    ESP_RETURN_ON_ERROR(pcnt_new_channel(q->unit, &chA_cfg, &q->ch_a), TAG, "new_ch_a");

    pcnt_chan_config_t chB_cfg = {
        .edge_gpio_num  = gpio_b,
        .level_gpio_num = gpio_a,
    };
    ESP_RETURN_ON_ERROR(pcnt_new_channel(q->unit, &chB_cfg, &q->ch_b), TAG, "new_ch_b");

    ESP_RETURN_ON_ERROR(
        pcnt_channel_set_edge_action(q->ch_a,
            PCNT_CHANNEL_EDGE_ACTION_INCREASE,
            PCNT_CHANNEL_EDGE_ACTION_DECREASE),
        TAG, "edge_a");

    ESP_RETURN_ON_ERROR(
        pcnt_channel_set_level_action(q->ch_a,
            PCNT_CHANNEL_LEVEL_ACTION_KEEP,
            PCNT_CHANNEL_LEVEL_ACTION_INVERSE),
        TAG, "level_a");

    ESP_RETURN_ON_ERROR(
        pcnt_channel_set_edge_action(q->ch_b,
            PCNT_CHANNEL_EDGE_ACTION_DECREASE,
            PCNT_CHANNEL_EDGE_ACTION_INCREASE),
        TAG, "edge_b");

    ESP_RETURN_ON_ERROR(
        pcnt_channel_set_level_action(q->ch_b,
            PCNT_CHANNEL_LEVEL_ACTION_KEEP,
            PCNT_CHANNEL_LEVEL_ACTION_INVERSE),
        TAG, "level_b");

    ESP_RETURN_ON_ERROR(pcnt_unit_enable(q->unit), TAG, "enable");
    ESP_RETURN_ON_ERROR(pcnt_unit_clear_count(q->unit), TAG, "clear");
    ESP_RETURN_ON_ERROR(pcnt_unit_start(q->unit), TAG, "start");

    return ESP_OK;
}

static inline int pcnt_get_count(pcnt_unit_handle_t unit)
{
    int v = 0;
    ESP_ERROR_CHECK(pcnt_unit_get_count(unit, &v));
    return v;
}

static void encoder_task(void *arg)
{
    enc_ctx_t *q = (enc_ctx_t *)arg;

    TickType_t last = xTaskGetTickCount();
    int64_t last_us = esp_timer_get_time();
    q->last_count = pcnt_get_count(q->unit);

    while (1) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(q->sample_ms));

        int now = pcnt_get_count(q->unit);
        int delta = now - q->last_count;
        q->last_count = now;

        int64_t now_us = esp_timer_get_time();
        float dt = (float)(now_us - last_us) / 1e6f; 
        last_us = now_us;

        float cps = (dt > 0.0f) ? ((float)delta / dt) : 0.0f;

        float rps = (q->cpr > 0) ? (cps / (float)q->cpr) : 0.0f;
        float rpm = rps * 60.0f;
        float rad_s = rps * 2.0f * (float)M_PI;

        if (xSemaphoreTake(q->mtx, pdMS_TO_TICKS(10)) == pdTRUE) {
            q->data.delta_cnt = delta;
            q->data.position_cnt = (int64_t)now;
            q->data.rpm = rpm;
            q->data.rad_s = rad_s;
            q->data.sample_ms = q->sample_ms;
            q->data.glitch_ns = q->glitch_ns_use;
            q->data.t_us = now_us;
            xSemaphoreGive(q->mtx);
        }
    }
}

esp_err_t encoder_init_pcnt_x4(int gpio_a, int gpio_b, uint32_t glitch_ns, uint32_t cpr_x4, uint32_t sample_period_ms)
{
    memset(&g_enc, 0, sizeof(g_enc));

    g_enc.cpr = cpr_x4;
    g_enc.sample_ms = sample_period_ms;
    g_enc.mtx = xSemaphoreCreateMutex();
    if (!g_enc.mtx) return ESP_ERR_NO_MEM;

    ESP_RETURN_ON_ERROR(encoder_gpio_init(gpio_a, gpio_b), TAG, "gpio_init");
    ESP_RETURN_ON_ERROR(pcnt_qdec_init(&g_enc, gpio_a, gpio_b, glitch_ns), TAG, "pcnt_init");

    ESP_LOGI(TAG, "PCNT x4 ready: A=%d B=%d | CPR=%u | Ts=%ums | glitch=%uns (HWmax~%uns)",
             gpio_a, gpio_b, (unsigned)g_enc.cpr, (unsigned)g_enc.sample_ms,
             (unsigned)g_enc.glitch_ns_use, (unsigned)pcnt_hw_max_glitch_ns());

    xTaskCreate(encoder_task, "enc_task", 4096, &g_enc, 6, NULL);

    return ESP_OK;
}

void encoder_get_data(encoder_data_t *out)
{
    if (!out) return;

    if (xSemaphoreTake(g_enc.mtx, pdMS_TO_TICKS(10)) == pdTRUE) {
        *out = g_enc.data;
        xSemaphoreGive(g_enc.mtx);
    } else {
        *out = g_enc.data;
    }
}

int64_t encoder_get_position_cnt(void)
{
    encoder_data_t d;
    encoder_get_data(&d);
    return d.position_cnt;
}

float encoder_get_rpm(void)
{
    encoder_data_t d;
    encoder_get_data(&d);
    return d.rpm;
}