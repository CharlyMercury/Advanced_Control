#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_timer.h"

#include "encoder_pcnt.h"
#include "motor_l298.h"

// Pinout
#define ENC_A_GPIO              33
#define ENC_B_GPIO              25
#define L298_IN1_GPIO           32
#define L298_IN2_GPIO           27
#define L298_ENA_GPIO           26

#define ENCODER_CPR_X4          1976U
#define ENCODER_GLITCH_NS       1000U
#define TS_MS                   20U
#define PWM_FREQ_HZ             10000U

/* =========================================================
 * Modelo
 * x1_dot = x2
 * x2_dot = -a*x2 + b*u + c
 * ========================================================= */
#define MODEL_A                 4.0f
#define MODEL_B                 25.0f
#define MODEL_C                 0.0f

// Ganancias del control
#define K1_POS                  2.0f
#define K2_VEL                  0.20f

// Ganancias del observador
#define L1_OBS                  20.0f
#define L2_OBS                  120.0f

// Referencia
#define POS_REF_RAD             1.0f

// Signos
#define MOTOR_CMD_SIGN          1.0f
#define ENCODER_SIGN           -1.0f

// Condición de reinicio
#define POS_EPS_RAD             0.03f
#define VEL_EPS_RAD_S           0.20f

// Umbral para evitar reenviar exactamente el mismo comando
// y disparar logs/acciones innecesarias en el driver
#define CMD_EPS                 0.0005f

typedef struct {
    float x1_hat;
    float x2_hat;
} observer_state_t;

static inline float clampf(float x, float xmin, float xmax)
{
    if (x < xmin) {
        return xmin;
    }
    if (x > xmax) {
        return xmax;
    }
    return x;
}

static inline float counts_to_rad(int64_t counts, uint32_t cpr_x4)
{
    if (cpr_x4 == 0U) {
        return 0.0f;
    }
    return ((float)counts * 2.0f * (float)M_PI) / (float)cpr_x4;
}

static void observer_init(observer_state_t *obs, float x1_init, float x2_init)
{
    if (obs == NULL) {
        return;
    }

    obs->x1_hat = x1_init;
    obs->x2_hat = x2_init;
}

static void observer_update(observer_state_t *obs, float y_pos, float u, float Ts)
{
    if (obs == NULL) {
        return;
    }

    const float innovation = y_pos - obs->x1_hat;

    const float x1_hat_dot =
        obs->x2_hat + (L1_OBS * innovation);

    const float x2_hat_dot =
        (-MODEL_A * obs->x2_hat) +
        (MODEL_B * u) +
        MODEL_C +
        (L2_OBS * innovation);

    obs->x1_hat += Ts * x1_hat_dot;
    obs->x2_hat += Ts * x2_hat_dot;
}

static float state_feedback_measured_control(float e_pos, float e_vel)
{
    float u = -(K1_POS * e_pos) - (K2_VEL * e_vel);
    return clampf(u, -1.0f, 1.0f);
}

static void print_csv_header(void)
{
    printf("t_ms,ref_pos,y_pos,y_vel,e_pos,e_vel,x1_hat,x2_hat,u,delta_cnt,cnt_raw,cnt_rel\n");
}

static void print_csv_row(
    int64_t t_ms,
    float ref_pos,
    float y_pos,
    float y_vel,
    float e_pos,
    float e_vel,
    float x1_hat,
    float x2_hat,
    float u,
    int32_t delta_cnt,
    int64_t cnt_raw,
    int64_t cnt_rel
)
{
    printf(
        "%" PRId64 ",%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%" PRId32 ",%" PRId64 ",%" PRId64 "\n",
        t_ms,
        ref_pos,
        y_pos,
        y_vel,
        e_pos,
        e_vel,
        x1_hat,
        x2_hat,
        u,
        delta_cnt,
        cnt_raw,
        cnt_rel
    );
}

static void control_task(void *arg)
{
    (void)arg;

    const float Ts = ((float)TS_MS) / 1000.0f;
    TickType_t last = xTaskGetTickCount();

    encoder_data_t enc = {0};
    encoder_data_t enc_zero = {0};
    observer_state_t obs;

    int64_t cnt_rel = 0;
    int64_t t_ms = 0;

    float y_pos = 0.0f;
    float y_vel = 0.0f;
    float ref_pos = POS_REF_RAD;
    float ref_vel = 0.0f;

    float e_pos = 0.0f;
    float e_vel = 0.0f;
    float u = 0.0f;
    float u_applied = 999.0f;

    observer_init(&obs, 0.0f, 0.0f);

    vTaskDelay(pdMS_TO_TICKS(300));

    encoder_get_data(&enc_zero);

    print_csv_header();

    while (1) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(TS_MS));

        // Lectura del encoder
        encoder_get_data(&enc);

        cnt_rel = enc.position_cnt - enc_zero.position_cnt;

        y_pos = counts_to_rad(cnt_rel, ENCODER_CPR_X4);
        y_pos *= ENCODER_SIGN;

        y_vel = enc.rad_s;
        y_vel *= ENCODER_SIGN;

        // Error medio
        e_pos = y_pos - ref_pos;
        e_vel = y_vel - ref_vel;

        // Control con mediciones
        u = state_feedback_measured_control(e_pos, e_vel);

        if ((fabsf(e_pos) < POS_EPS_RAD) && (fabsf(y_vel) < VEL_EPS_RAD_S)) {
            u = 0.0f;
        }

        u *= MOTOR_CMD_SIGN;
        u = clampf(u, -1.0f, 1.0f);

        // Evitar reenviar el mismo comando y generar logs innecesarios en el driver
        if (fabsf(u - u_applied) > CMD_EPS) {
            ESP_ERROR_CHECK(motor_l298_set(u));
            u_applied = u;
        }

        // Observador separado
        observer_update(&obs, y_pos, u_applied, Ts);

        // Salida CSV limpia
        t_ms = esp_timer_get_time() / 1000LL;

        print_csv_row(
            t_ms,
            ref_pos,
            y_pos,
            y_vel,
            e_pos,
            e_vel,
            obs.x1_hat,
            obs.x2_hat,
            u_applied,
            enc.delta_cnt,
            enc.position_cnt,
            cnt_rel
        );
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(
        motor_l298_init(
            L298_IN1_GPIO,
            L298_IN2_GPIO,
            L298_ENA_GPIO,
            PWM_FREQ_HZ
        )
    );

    ESP_ERROR_CHECK(
        encoder_init_pcnt_x4(
            ENC_A_GPIO,
            ENC_B_GPIO,
            ENCODER_GLITCH_NS,
            ENCODER_CPR_X4,
            TS_MS
        )
    );

    xTaskCreate(
        control_task,
        "control_task",
        4096,
        NULL,
        5,
        NULL
    );
}