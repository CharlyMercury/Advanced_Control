#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_private/esp_clk.h"
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"

static const char *TAG = "ENCODER x4";

// Parámetros de configuración
#define ENC_A_GPIO 25
#define ENC_B_GPIO 33

// Ventana de muestreo para imprimir delta (ms)
#define SAMPLE_PERIOD_MS 50

// Filtro anti-ruido (tu valor decidido)
#define GLITCH_NS 8000

// Si la dirección te queda invertida:
// 1) Cambia ENC_SWAP_AB a 1 (más limpio), o
// 2) Cambia ENC_INVERT_DIR a 1 (más rápido)
#define ENC_SWAP_AB    0
#define ENC_INVERT_DIR 0

// ===================== ESTRUCTURA PCNT =====================
// Un "unit" = un contador hardware.
// Dos "channels" = lógica para cuadratura (uno mira A con B, otro mira B con A).
typedef struct {
    pcnt_unit_handle_t unit;
    pcnt_channel_handle_t ch_a;
    pcnt_channel_handle_t ch_b;
    uint32_t glitch_ns_use;
} pcnt_qdec_t;

static pcnt_qdec_t g_q;

// --------------------- Helper: máximo glitch posible ---------------------
// PCNT usa un umbral en ticks del APB (típicamente max 1023).
// Convertimos ese máximo a nanosegundos para no pedir algo imposible.
static uint32_t pcnt_hw_max_glitch_ns(void)
{
    uint32_t apb = esp_clk_apb_freq();   // normalmente 80MHz en ESP32 clásico
    uint32_t thres_max = 1023;
    uint32_t ns = (uint32_t)((1000000000ULL * thres_max) / apb);
    return ns + 50;
}

// --------------------- GPIO init ---------------------
// Pull-up interno: útil si encoder es open-collector.
// Si tu encoder es push-pull, normalmente no estorba.
static esp_err_t encoder_gpio_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << ENC_A_GPIO) | (1ULL << ENC_B_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    return gpio_config(&io);
}

// --------------------- PCNT init (Quadrature x4) ---------------------
// Esta es la parte clave.
//
// Modelo mental (muy importante):
// - Canal A cuenta flancos de A, pero decide + o - mirando el NIVEL de B.
// - Canal B cuenta flancos de B, pero decide + o - mirando el NIVEL de A.
// - Con edge_action + level_action se arma la tabla de cuadratura.
// - Resultado: conteo con signo (dirección) y con x4.
static esp_err_t pcnt_qdec_init(pcnt_qdec_t *q)
{
    memset(q, 0, sizeof(*q));

    // 1) Crear el contador (unit)
    pcnt_unit_config_t unit_cfg = {
        .high_limit = 32767,
        .low_limit  = -32768,
    };
    ESP_RETURN_ON_ERROR(pcnt_new_unit(&unit_cfg, &q->unit), TAG, "pcnt_new_unit");

    // 2) Configurar filtro de glitch (anti rebote/ruido)
    uint32_t hw_max = pcnt_hw_max_glitch_ns();
    q->glitch_ns_use = (GLITCH_NS > hw_max) ? hw_max : GLITCH_NS;

    pcnt_glitch_filter_config_t flt = {
        .max_glitch_ns = q->glitch_ns_use,
    };
    ESP_RETURN_ON_ERROR(pcnt_unit_set_glitch_filter(q->unit, &flt), TAG, "set_glitch");

    // 3) Definir qué GPIO usa cada canal
    #if ENC_SWAP_AB
        const int A_EDGE = ENC_B_GPIO;  // Canal A contará flancos de B
        const int A_LVL  = ENC_A_GPIO;  // y mirará nivel de A
        const int B_EDGE = ENC_A_GPIO;  // Canal B contará flancos de A
        const int B_LVL  = ENC_B_GPIO;  // y mirará nivel de B
    #else
        const int A_EDGE = ENC_A_GPIO;  // Canal A: flancos de A
        const int A_LVL  = ENC_B_GPIO;  // mira nivel de B
        const int B_EDGE = ENC_B_GPIO;  // Canal B: flancos de B
        const int B_LVL  = ENC_A_GPIO;  // mira nivel de A
    #endif

    // 4) Crear canal A
    pcnt_chan_config_t chA_cfg = {
        .edge_gpio_num  = A_EDGE,
        .level_gpio_num = A_LVL,
    };
    ESP_RETURN_ON_ERROR(pcnt_new_channel(q->unit, &chA_cfg, &q->ch_a), TAG, "new_ch_a");

    // 5) Crear canal B
    pcnt_chan_config_t chB_cfg = {
        .edge_gpio_num  = B_EDGE,
        .level_gpio_num = B_LVL,
    };
    ESP_RETURN_ON_ERROR(pcnt_new_channel(q->unit, &chB_cfg, &q->ch_b), TAG, "new_ch_b");

    // 6) “Tabla” de acciones para cuadratura x4 (patrón estándar ESP-IDF)
    //
    // Explicación práctica:
    // - edge_action define qué pasa en flanco positivo/negativo de la señal edge.
    // - level_action permite invertir esas acciones dependiendo del nivel de la otra señal.
    //
    // Con esto, cada transición A/B suma o resta correctamente (x4 + dirección).

    // Canal A: posedge => +1, negedge => -1
    ESP_RETURN_ON_ERROR(
        pcnt_channel_set_edge_action(q->ch_a,
                                     PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                     PCNT_CHANNEL_EDGE_ACTION_DECREASE),
        TAG, "edge_a");

    // Canal A: si la señal "level" (B) está HIGH, invierte el sentido
    ESP_RETURN_ON_ERROR(
        pcnt_channel_set_level_action(q->ch_a,
                                      PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                      PCNT_CHANNEL_LEVEL_ACTION_INVERSE),
        TAG, "level_a");

    // Canal B: posedge => -1, negedge => +1 (opuesto al canal A)
    ESP_RETURN_ON_ERROR(
        pcnt_channel_set_edge_action(q->ch_b,
                                     PCNT_CHANNEL_EDGE_ACTION_DECREASE,
                                     PCNT_CHANNEL_EDGE_ACTION_INCREASE),
        TAG, "edge_b");

    // Canal B: si la señal "level" (A) está HIGH, invierte el sentido
    ESP_RETURN_ON_ERROR(
        pcnt_channel_set_level_action(q->ch_b,
                                      PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                      PCNT_CHANNEL_LEVEL_ACTION_INVERSE),
        TAG, "level_b");

    // 7) Habilitar y arrancar el contador
    ESP_RETURN_ON_ERROR(pcnt_unit_enable(q->unit), TAG, "enable");
    ESP_RETURN_ON_ERROR(pcnt_unit_clear_count(q->unit), TAG, "clear");
    ESP_RETURN_ON_ERROR(pcnt_unit_start(q->unit), TAG, "start");

    return ESP_OK;
}

static inline int pcnt_get_count(pcnt_qdec_t *q)
{
    int v = 0;
    ESP_ERROR_CHECK(pcnt_unit_get_count(q->unit, &v));
    return v;
}

static inline void pcnt_clear(pcnt_qdec_t *q)
{
    pcnt_unit_clear_count(q->unit);
}

// --------------------- Task: mostrar incremento/decremento ---------------------
// Estrategia:
//  Cada Ts ms:
//    delta = leer contador (cuentas desde el último clear)
//    clear()
//    total += delta
//
// Esto te da:
//  - delta con signo (dirección)
//  - total acumulado (sube/baja)
static void encoder_print_task(void *arg)
{
    int64_t total = 0;

    pcnt_clear(&g_q);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));

        int delta = pcnt_get_count(&g_q);
        pcnt_clear(&g_q);

        #if ENC_INVERT_DIR
                delta = -delta;
        #endif

        total += (int64_t)delta;

        const char *dir = (delta > 0) ? "FWD" : (delta < 0) ? "REV" : "STOP";

        ESP_LOGI(TAG,
                 "delta=%d cnt | total=%lld cnt | dir=%s | glitch=%uns",
                 delta, (long long)total, dir, (unsigned)g_q.glitch_ns_use);
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Init encoder x4 (PCNT): A=%d B=%d | Ts=%dms | glitch=%uns",
             ENC_A_GPIO, ENC_B_GPIO, SAMPLE_PERIOD_MS, GLITCH_NS);

    ESP_ERROR_CHECK(encoder_gpio_init());
    ESP_ERROR_CHECK(pcnt_qdec_init(&g_q));

    ESP_LOGI(TAG, "Glitch applied: %u ns (HW max ~%u ns)",
             (unsigned)g_q.glitch_ns_use, (unsigned)pcnt_hw_max_glitch_ns());

    xTaskCreate(encoder_print_task, "enc_print", 4096, NULL, 5, NULL);
}
