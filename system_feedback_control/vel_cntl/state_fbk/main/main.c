#include <stdio.h>
#include <math.h>
#include <inttypes.h>

#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "encoder_pcnt.h"
#include "motor_l298.h"

#define ENC_A_GPIO              33
#define ENC_B_GPIO              25
#define L298_IN1_GPIO           32
#define L298_IN2_GPIO           27
#define L298_ENA_GPIO           26
#define ENCODER_CPR_X4          1976
#define TS_MS                   20
#define PWM_FREQ_HZ             20000
#define MODEL_A                 11.11281274f
#define MODEL_B                (-165.65653960f)
#define MODEL_C                 0.37449236f
#define CTRL_K                  1.0f
#define OMEGA_REF_RAD_S         8.0f
#define U_START_POS             0.55f
#define U_START_NEG             0.55f
#define OMEGA_LPF_ALPHA         0.85f
#define OMEGA_STOP_EPS          0.20f
#define REF_STOP_EPS            0.10f
#define MOTOR_CMD_SIGN          -1.0f
#define ENCODER_SIGN            -1.0f

static const char *TAG = "VEL_CTRL";

static inline float clampf(float x, float xmin, float xmax)
{
    if (x < xmin) return xmin;
    if (x > xmax) return xmax;
    return x;
}

/**
 * Convierte entrada efectiva del modelo u_eff [-1,1]
 * a PWM real del motor considerando zona muerta.
 *
 * Si:
 *   |u_eff| = 0   -> pwm = 0
 *   |u_eff| = 1   -> pwm = 1
 *   0 < |u_eff| < 1 -> pwm = u0 + (1-u0)|u_eff|
 */
static float effective_input_to_pwm(float u_eff)
{
    float mag = fabsf(u_eff);
    float u0;
    float pwm_mag;

    if (mag < 1e-6f) {
        return 0.0f;
    }

    u0 = (u_eff >= 0.0f) ? U_START_POS : U_START_NEG;

    mag = clampf(mag, 0.0f, 1.0f);
    pwm_mag = u0 + (1.0f - u0) * mag;
    pwm_mag = clampf(pwm_mag, 0.0f, 1.0f);

    return copysignf(pwm_mag, u_eff);
}

static float speed_control_u_eff(float omega_ref, float omega_meas)
{
    const float e = omega_ref - omega_meas;
    float u_eff = ((MODEL_A * omega_ref) - MODEL_C + (CTRL_K * e)) / MODEL_B;

    u_eff = clampf(u_eff, -1.0f, 1.0f);

    if ((fabsf(omega_ref) < REF_STOP_EPS) && (fabsf(omega_meas) < OMEGA_STOP_EPS)) {
        u_eff = 0.0f;
    }

    return u_eff;
}

static void velocity_control_task(void *arg)
{
    (void)arg;

    const float Ts = ((float)TS_MS) / 1000.0f;
    TickType_t last = xTaskGetTickCount();

    encoder_data_t d = {0};

    float t_s = 0.0f;
    float omega_meas = 0.0f;
    float omega_filt = 0.0f;
    float omega_ref = 0.0f;
    float error = 0.0f;
    float u_eff = 0.0f;
    float pwm_cmd = 0.0f;

    esp_err_t err = motor_l298_set(0.0f);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "motor_l298_set(0.0f) failed: %s", esp_err_to_name(err));
        vTaskDelete(NULL);
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(200));

    printf("t_s,omega_ref_rad_s,omega_meas_rad_s,omega_filt_rad_s,error_rad_s,u_eff,pwm_cmd,delta_cnt,position_cnt\n");

    while (1) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(TS_MS));
        t_s += Ts;

        encoder_get_data(&d);

        omega_meas = ENCODER_SIGN * d.rad_s;

        omega_filt = (OMEGA_LPF_ALPHA * omega_filt) +
                     ((1.0f - OMEGA_LPF_ALPHA) * omega_meas);

        omega_ref = OMEGA_REF_RAD_S;

        error = omega_ref - omega_filt;
        u_eff = speed_control_u_eff(omega_ref, omega_filt);
        pwm_cmd = effective_input_to_pwm(u_eff);
        pwm_cmd *= MOTOR_CMD_SIGN;

        err = motor_l298_set(pwm_cmd);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "motor_l298_set(%.6f) failed: %s", (double)pwm_cmd, esp_err_to_name(err));
            vTaskDelete(NULL);
            return;
        }

        printf("%.3f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%" PRId32 ",%" PRId64 "\n",
               (double)t_s,
               (double)omega_ref,
               (double)omega_meas,
               (double)omega_filt,
               (double)error,
               (double)u_eff,
               (double)pwm_cmd,
               d.delta_cnt,
               d.position_cnt);
    }
}

void app_main(void)
{
    esp_err_t err;

    err = encoder_init_pcnt_x4(
        ENC_A_GPIO,
        ENC_B_GPIO,
        8000,
        ENCODER_CPR_X4,
        TS_MS
    );
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "encoder_init_pcnt_x4 failed: %s", esp_err_to_name(err));
        return;
    }

    err = motor_l298_init(
        L298_IN1_GPIO,
        L298_IN2_GPIO,
        L298_ENA_GPIO,
        PWM_FREQ_HZ
    );
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "motor_l298_init failed: %s", esp_err_to_name(err));
        return;
    }

    BaseType_t task_ok = xTaskCreate(
        velocity_control_task,
        "velocity_control_task",
        4096,
        NULL,
        5,
        NULL
    );

    if (task_ok != pdPASS) {
        ESP_LOGE(TAG, "xTaskCreate failed");
        return;
    }
}