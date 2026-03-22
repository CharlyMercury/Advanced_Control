# State Feedback Speed Control for ESP32 (ESP-IDF)

Control de velocidad para un motor DC con **ESP32 + ESP-IDF**, usando:

* **Encoder incremental en cuadratura** con periférico **PCNT** en modo x4.
* **Driver L298** con control de dirección y PWM mediante **LEDC**.
* **Ley de control basada en el error** y en un **modelo identificado** del motor.
* **Registro por `printf`** para analizar el comportamiento del sistema y comparar resultados con **Python + Matplotlib**.

El proyecto está pensado para correr sobre un ESP32 con un motor DC instrumentado con encoder, y sirve como base para pruebas de identificación, validación del modelo y cierre de lazo en velocidad.

---

## 1. Descripción general

El repositorio implementa un controlador de velocidad de la forma:

[
e = \omega_{ref} - \omega
]

[
u_{eff} = \frac{a,\omega_{ref} - c + K e}{b}
]

con el modelo identificado del sistema:

[
\dot{\omega} = -a\omega + b,u_{eff} + c
]

En el código actual se usan los parámetros:

* `a = 11.11281274`
* `b = -165.65653960`
* `c = 0.37449236`
* `K = 1.0`

Como el motor presenta **zona muerta** al arranque, la salida del controlador no se manda directo al puente H. Primero se calcula una **entrada efectiva** `u_eff`, y luego se transforma a un **PWM real** compensando el umbral mínimo de activación.

En otras palabras: el control está bien vestido de teoría, pero también se ensucia las botas con la fricción real del motor.

---

## 2. Características del proyecto

* Lectura de encoder incremental con **PCNT**.
* Estimación de:

  * `delta_cnt`
  * `position_cnt`
  * `rpm`
  * `rad_s`
* Driver para L298 con:

  * control de dirección (`IN1`, `IN2`)
  * PWM por `LEDC`
  * cola FreeRTOS para desacoplar escritura del actuador
* Bucle de control discreto con periodo de muestreo fijo.
* Filtro paso bajo simple para la velocidad medida.
* Registro CSV por consola serial para análisis posterior.
* Script Python para:

  * extraer logs con ruido
  * reconstruir el modelo
  * comparar referencia, medición, señal filtrada y modelo
  * generar figuras con Matplotlib

---

## 3. Estructura del repositorio

```text
.
├── CMakeLists.txt
├── estimacion_18_03_2026.txt
├── velocity_feedback_control.ssp
├── velocity_feedback_control_error.ssp
└── main/
    ├── CMakeLists.txt
    ├── encoder_pcnt.c
    ├── encoder_pcnt.h
    ├── motor_l298.c
    ├── motor_l298.h
    ├── main.c
    ├── speed_log_comparation.py
    ├── output.txt
    ├── comparison_speed.png
    ├── comparison_errors.png
    ├── comparison_control.png
    ├── comparison_summary.png
    └── comparison_enriched.csv
```

### Archivos principales

* **`main/main.c`**
  Implementa la inicialización del encoder, el L298 y la tarea de control de velocidad.

* **`main/encoder_pcnt.c` / `main/encoder_pcnt.h`**
  Módulo de adquisición del encoder usando el periférico `pulse_cnt` del ESP-IDF.

* **`main/motor_l298.c` / `main/motor_l298.h`**
  Módulo de control del motor con L298 usando señales de dirección y PWM.

* **`main/speed_log_comparation.py`**
  Script para analizar el log serial y generar gráficas comparativas con Matplotlib.

* **`velocity_feedback_control.ssp` y `velocity_feedback_control_error.ssp`**
  Archivos de simulación para validar la estructura del control en entorno gráfico.

* **`estimacion_18_03_2026.txt`**
  Archivo asociado a identificación/estimación del modelo del motor.

---

## 4. Hardware considerado

El código actual está configurado con los siguientes pines:

### Encoder

* Canal A: `GPIO 33`
* Canal B: `GPIO 25`

### L298

* `IN1`: `GPIO 32`
* `IN2`: `GPIO 27`
* `ENA` PWM: `GPIO 26`

### Parámetros relevantes

* `ENCODER_CPR_X4 = 1976`
* `TS_MS = 20`
* `PWM_FREQ_HZ = 20000`

Si tu hardware cambia, aquí no hay drama filosófico: cambia los `#define` y listo.

---

## 5. Funcionamiento del control

### 5.1 Modelo utilizado

El modelo usado en el repositorio es:

[
\dot{\omega} = -a\omega + b,u_{eff} + c
]

Este modelo no trabaja directamente con el PWM crudo del motor, sino con una **entrada efectiva** `u_eff` que compensa la zona muerta.

### 5.2 Ley de control

Se define el error de velocidad como:

[
e = \omega_{ref} - \omega
]

La ley implementada es:

[
u_{eff} = \frac{a\omega_{ref} - c + K(\omega_{ref} - \omega)}{b}
]

Posteriormente:

1. `u_eff` se satura a `[-1, 1]`
2. se transforma a `pwm_cmd`
3. se aplica signo de actuador con `MOTOR_CMD_SIGN`
4. se manda al L298 con `motor_l298_set()`

### 5.3 Zona muerta

Se usan estos umbrales:

* `U_START_POS = 0.55`
* `U_START_NEG = 0.55`

Esto significa que comandos pequeños pueden no mover el motor si no se compensan. Por eso el mapeo `u_eff -> pwm_cmd` es obligatorio. Si no lo haces, el control “funciona” en papel y el motor se queda con cara de “¿y luego?”.

### 5.4 Convenciones de signo

El proyecto incluye dos constantes importantes:

* `MOTOR_CMD_SIGN`
* `ENCODER_SIGN`

Sirven para corregir el sentido del actuador o de la medición según cómo esté cableado el sistema.

**Recomendación:** corrige solo uno si el sentido está invertido. Cambiar ambos es una forma elegante de perder tiempo.

---

## 6. Módulos del proyecto

### 6.1 Módulo `encoder_pcnt`

Este módulo:

* configura GPIOs del encoder como entrada con `pull-up`
* inicializa `PCNT`
* aplica filtro de glitch
* cuenta pulsos en cuadratura x4
* calcula velocidad a partir del conteo cada `sample_period_ms`
* publica la información en una estructura `encoder_data_t`

### API pública

```c
esp_err_t encoder_init_pcnt_x4(
    int gpio_a,
    int gpio_b,
    uint32_t glitch_ns,
    uint32_t cpr_x4,
    uint32_t sample_period_ms
);

void encoder_get_data(encoder_data_t *out);
int64_t encoder_get_position_cnt(void);
float encoder_get_rpm(void);
```

### Estructura principal

```c
typedef struct {
    int64_t position_cnt;
    int32_t delta_cnt;
    float   rpm;
    float   rad_s;
    uint32_t sample_ms;
    uint32_t glitch_ns;
    int64_t t_us;
} encoder_data_t;
```

---

### 6.2 Módulo `motor_l298`

Este módulo:

* configura las salidas de dirección
* configura PWM con `LEDC`
* utiliza una cola FreeRTOS para recibir comandos
* permite mandar un comando firmado en el rango `[-1, 1]`

### API pública

```c
esp_err_t motor_l298_init(int gpio_in1, int gpio_in2, int gpio_ena_pwm, uint32_t pwm_freq_hz);
esp_err_t motor_l298_set(float cmd);
```

### Convención del comando

* `cmd > 0` → giro hacia adelante
* `cmd < 0` → giro hacia atrás
* `cmd = 0` → paro

---

## 7. Formato del log serial

El `main.c` imprime un encabezado CSV seguido de las muestras del control:

```text
t_s,omega_ref_rad_s,omega_meas_rad_s,omega_filt_rad_s,error_rad_s,u_eff,pwm_cmd,delta_cnt,position_cnt
```

Cada fila contiene:

* `t_s`: tiempo en segundos
* `omega_ref_rad_s`: referencia de velocidad
* `omega_meas_rad_s`: velocidad medida por encoder
* `omega_filt_rad_s`: velocidad filtrada
* `error_rad_s`: error de seguimiento
* `u_eff`: entrada efectiva del modelo
* `pwm_cmd`: comando PWM aplicado al L298
* `delta_cnt`: incremento de cuentas en la muestra
* `position_cnt`: posición acumulada

Este formato lo consume directamente el script Python incluido en el repo.

---

## 8. Requisitos de software

Para compilar el firmware necesitas:

* **ESP-IDF** instalado y exportado en el entorno
* Toolchain para ESP32
* Python del entorno de ESP-IDF

Para el análisis del log necesitas:

* Python 3.10+ recomendado
* `numpy`
* `pandas`
* `matplotlib`

Instalación rápida de dependencias para el script:

```bash
pip install numpy pandas matplotlib
```

---

## 9. Compilación y carga

Desde la raíz del proyecto:

```bash
idf.py set-target esp32
idf.py build
idf.py flash
idf.py monitor
```

Si el puerto serial no es detectado automáticamente:

```bash
idf.py -p /dev/ttyUSB0 flash monitor
```

En Windows, cambia el puerto por algo como:

```bash
idf.py -p COM5 flash monitor
```

---

## 10. Uso básico

1. Conecta el ESP32, el encoder y el L298 según los pines configurados.
2. Compila y flashea el firmware.
3. Abre el monitor serial.
4. Guarda la salida del log en un archivo de texto.
5. Procesa el archivo con el script Python.

---

## 11. Análisis de logs con Python

El script incluido:

```bash
python main/speed_log_comparation.py main/output.txt --show
```

o con un archivo propio:

```bash
python main/speed_log_comparation.py mi_log.txt --show
```

### Qué hace el script

* localiza el encabezado CSV en un log aunque haya ruido alrededor
* limpia filas inválidas
* reconstruye `omega_model` a partir de `u_eff`
* calcula métricas básicas
* guarda gráficos como:

  * `comparison_speed.png`
  * `comparison_errors.png`
  * `comparison_control.png`
  * `comparison_summary.png`
* exporta `comparison_enriched.csv`

---

## 12. Parámetros importantes a ajustar

En `main/main.c` los parámetros más importantes son:

### Referencia

```c
#define OMEGA_REF_RAD_S 8.0f
```

### Ganancia del controlador

```c
#define CTRL_K 1.0f
```

### Signo del actuador y del encoder

```c
#define MOTOR_CMD_SIGN -1.0f
#define ENCODER_SIGN    1.0f
```

### Filtro de medición

```c
#define OMEGA_LPF_ALPHA 0.85f
```

### Zona muerta

```c
#define U_START_POS 0.55f
#define U_START_NEG 0.55f
```

---

## 13. Consejos de validación

Antes de cerrar el lazo completo, conviene revisar esto:

### 13.1 Sentido de giro

Si la referencia es positiva y el motor gira al revés:

* invierte `MOTOR_CMD_SIGN`, o
* invierte `ENCODER_SIGN`

No ambos.

### 13.2 Zona muerta

Si el motor no arranca con comandos pequeños, revisa:

* `U_START_POS`
* `U_START_NEG`

### 13.3 Ganancia `K`

* Si responde muy lento, sube `CTRL_K`
* Si oscila o se vuelve nervioso, bájala

### 13.4 Filtrado

Si la medición es muy ruidosa, aumenta `OMEGA_LPF_ALPHA`.

---

## 14. Limitaciones actuales

* La referencia en `main.c` es constante.
* No hay interfaz por UART/MQTT/CLI para cambiar la referencia en tiempo real.
* No hay anti-windup porque el controlador no es integral; eso simplifica, pero también limita el ajuste fino ante perturbaciones persistentes.
* El sistema depende de una identificación previa del motor y de una compensación razonable de la zona muerta.

---

## 15. Posibles mejoras

* Referencias por escalón programables dentro del firmware.
* Cambio de referencia por puerto serial.
* Registro en archivo o streaming de telemetría.
* Separar configuración de hardware en un `config.h`.
* Añadir pruebas para identificación automática.
* Sustituir el L298 por un driver más eficiente si se quiere algo más serio y menos calefactor disfrazado de puente H.

---

## 16. Ejemplo de flujo de trabajo

1. Ejecutar una prueba de identificación.
2. Obtener un modelo aproximado del motor.
3. Ajustar `a`, `b`, `c` en `main.c`.
4. Cerrar el lazo en velocidad con ganancia `K`.
5. Guardar el log serial.
6. Analizar el seguimiento y el error con el script Python.
7. Repetir ajuste hasta obtener la respuesta deseada.

---

## 17. Notas sobre simulación

Los archivos `.ssp` del repositorio corresponden al esquema de simulación del control. Son útiles para:

* validar signos
* revisar la realimentación
* comprobar que la ley de control implementada en firmware coincide con la derivación
