# README — system_identification_sw

Proyecto para **identificación de un motor DC con ESP32**, usando:

* **ESP-IDF**
* **Driver L298N**
* **Encoder incremental en cuadratura** con **PCNT**
* Generación de excitación tipo **PRBS**
* Estimación offline de parámetros con **Python**

El flujo general es simple:

1. El ESP32 aplica una señal de excitación al motor.
2. Lee la velocidad angular desde el encoder.
3. Imprime por serial un bloque CSV con las muestras.
4. Ese log se procesa con `main/offline_estimation.py` para estimar el modelo discreto y continuo del motor.

---

## 1. Estructura del proyecto

```text
.
├── CMakeLists.txt
├── sdkconfig
├── estimacion_18_03_2026.txt
├── pwm_to_ueff.sci
├── main/
│   ├── CMakeLists.txt
│   ├── main.c
│   ├── encoder_pcnt.c
│   ├── encoder_pcnt.h
│   ├── motor_l298.c
│   ├── motor_l298.h
│   ├── motor_id.c
│   ├── motor_id.h
│   ├── rls2.c
│   ├── rls2.h
│   ├── excitation_prbs.c
│   ├── excitation_prbs.h
│   ├── offline_estimation.py
│   ├── output.txt
│   └── simulation.ssp
```

---

## 2. Requisitos

### Hardware

* ESP32
* Driver **L298N**
* Motor DC con encoder incremental
* Fuente adecuada para el motor
* Cables y tierra común entre fuentes

### Software

#### Firmware

* **ESP-IDF 5.x** o compatible con los drivers usados en el proyecto
* Python 3.10+ recomendado
* CMake y Ninja (normalmente ya vienen integrados con ESP-IDF)

#### Postproceso / estimación

* Python 3.10+
* Paquetes:

  * `numpy`
  * `pandas`

Instalación rápida:

```bash
pip install numpy pandas
```

---

## 3. Configuración de pines

En `main/main.c` el proyecto está configurado así:

```c
#define ENC_A_GPIO      33
#define ENC_B_GPIO      25
#define L298_IN1_GPIO   32
#define L298_IN2_GPIO   27
#define L298_ENA_GPIO   26

#define ENCODER_CPR_X4  1976
#define TS_MS           20
#define PWM_FREQ_HZ     20000
```

### Significado

* `ENC_A_GPIO` y `ENC_B_GPIO`: canales A y B del encoder
* `L298_IN1_GPIO`, `L298_IN2_GPIO`: dirección de giro del motor
* `L298_ENA_GPIO`: PWM hacia el pin ENA del L298N
* `ENCODER_CPR_X4`: cuentas por vuelta en modo cuadratura x4
* `TS_MS`: periodo de muestreo en milisegundos
* `PWM_FREQ_HZ`: frecuencia PWM

Si tu hardware usa otros pines o diferente encoder, cambia esos defines antes de compilar.

---

## 4. Corrección importante antes de compilar

En `main/CMakeLists.txt` aparece esto:

```cmake
idf_component_register(
    SRCS
        "encoder_pcnt.c"
        "excitation_prbs.c"
        "motor_l298.c"
        "rls2.c"
        "motor_id.c"
        "main"
    INCLUDE_DIRS
        "."
)
```

La última entrada debe ser **`"main.c"`** y no **`"main"`**.

Déjalo así:

```cmake
idf_component_register(
    SRCS
        "encoder_pcnt.c"
        "excitation_prbs.c"
        "motor_l298.c"
        "rls2.c"
        "motor_id.c"
        "main.c"
    INCLUDE_DIRS
        "."
)
```

Si no haces ese cambio, el proyecto no va a compilar. Sin drama, pero sí sin piedad.

---

## 5. Cómo correr el firmware

En este proyecto, el flujo principal para **compilar, flashear y abrir el monitor** se realiza con la **extensión ESP-IDF de VS Code**.

### Flujo recomendado en VS Code

1. Abre la carpeta del proyecto en **Visual Studio Code**.
2. Asegúrate de tener instalada y configurada la extensión **ESP-IDF**.
3. Selecciona el target **ESP32** desde la extensión si todavía no está definido.
4. Usa las acciones de la extensión para:

   * **Build / Compile project**
   * **Flash device**
   * **Monitor device**
5. Selecciona el puerto serial correcto del ESP32 antes de flashear.

### Notas importantes

* Si es la primera vez que abres el proyecto en otra máquina, revisa que el entorno de **ESP-IDF** esté correctamente configurado dentro de VS Code.
* El archivo `sdkconfig` ya existe en el proyecto, así que normalmente no deberías empezar desde cero.
* Si el build falla, primero revisa el detalle del `main/CMakeLists.txt` indicado en la sección anterior.

### Alternativa por terminal

También puedes compilar por terminal con `idf.py`, pero en este proyecto la referencia principal del README será el flujo usando la extensión de VS Code.

---

## 6. Qué hace el firmware al ejecutarse

Al arrancar:

1. Inicializa el encoder con PCNT.
2. Inicializa el L298N con PWM.
3. Lanza una tarea de identificación.
4. Genera una señal PRBS sobre niveles definidos en `main.c`.
5. Convierte la señal lógica a PWM efectivo.
6. Mide `omega` desde el encoder.
7. Imprime por serial un CSV con esta cabecera:

```csv
phi0_omega_k,phi1_u_eff_k,phi2_bias,y_omega_k1
```

Cada línea representa una muestra para la estimación offline.

---

## 7. Guardar la salida serial en un archivo

Necesitas capturar el bloque CSV para luego estimar parámetros.

### Linux / macOS

```bash
idf.py -p /dev/ttyUSB0 monitor | tee main/output.txt
```

### Windows PowerShell

```powershell
idf.py -p COM3 monitor | Tee-Object -FilePath main\output.txt
```

Cuando ya tengas suficientes muestras, detén el monitor con:

```text
Ctrl + ]
```

---

## 8. Estimación offline del modelo

El script `main/offline_estimation.py` busca dentro del archivo una cabecera válida y procesa únicamente las líneas numéricas correctas.

### Ejecutar con el periodo por defecto (`Ts = 0.02 s`)

```bash
python main/offline_estimation.py main/output.txt
```

### Ejecutar indicando `Ts`

```bash
python main/offline_estimation.py main/output.txt 0.02
```

### Salida esperada

El script reporta:

* Parámetros discretos estimados:

  * `alpha_hat`
  * `beta_hat`
  * `gamma_hat`
* Calidad del ajuste:

  * `RMSE`
  * `R^2`
* Parámetros continuos estimados:

  * `a_hat`
  * `b_hat`
  * `c_hat`
  * `tau_hat`
  * `K_hat`
  * `d_hat`

---

## 9. Ejemplo de resultado incluido en el proyecto

En `estimacion_18_03_2026.txt` ya viene un ejemplo de estimación con resultados como:

* `alpha_hat = 0.80071015`
* `beta_hat = -2.97077503`
* `gamma_hat = 0.00671590`
* `R^2 = 0.99624897`
* `tau_hat = 0.08998622 s`

Eso indica un ajuste bastante bueno. El motor no se anduvo con rodeos.

---

## 10. Parámetros importantes a ajustar

Desde `main/main.c` puedes modificar:

### Muestreo y PWM

```c
#define TS_MS           20
#define PWM_FREQ_HZ     20000
```

### Encoder

```c
#define ENCODER_CPR_X4  1976
```

### Parámetros RLS / almacenamiento

```c
#define RLS_LAMBDA      0.995f
#define RLS_P0          500.0f
#define OFFLINE_MAX_SAMPLES     2500
#define OFFLINE_TARGET_SAMPLES  1500
```

### Mapeo lógico a PWM y zona muerta

```c
#define U_START_POS     0.55f
#define U_START_NEG     0.55f
#define A_SLEW_MAX      0.08f
```

Estos parámetros son clave porque compensan la zona muerta del motor y limitan cambios bruscos.

---

## 11. Recomendaciones de conexión

### Encoder

* Canal A → `GPIO 33`
* Canal B → `GPIO 25`
* GND común con ESP32

### L298N

* `IN1` → `GPIO 32`
* `IN2` → `GPIO 27`
* `ENA` → `GPIO 26`
* GND del driver unido al GND del ESP32
* Fuente del motor separada o suficientemente robusta

### Muy importante

* No alimentes el motor directamente desde el pin de 5V del ESP32 como si no hubiera consecuencias.
* Usa tierra común entre control y potencia.
* Verifica niveles lógicos y corriente disponible.

---

## 12. Solución de problemas

### Error de compilación por archivo `main`

Causa probable: `main/CMakeLists.txt` usa `"main"` en vez de `"main.c"`.

Solución: corrige ese archivo como se muestra arriba.

### El motor no gira con comandos bajos

Causa probable: zona muerta mecánica/eléctrica.

Solución: ajusta:

```c
#define U_START_POS     0.55f
#define U_START_NEG     0.55f
```

### Velocidad con signo invertido

Revisa el sentido físico del encoder o la lógica de inversión en `encoder_pcnt.c`:

```c
#define ENC_INVERT_DIR 0
```

Si hace falta, cámbialo a `1`.

### Conteo incorrecto del encoder

Revisa:

* `ENCODER_CPR_X4`
* cableado A/B
* ruido eléctrico
* filtro glitch (`glitch_ns`)

### El script Python no encuentra cabecera

Asegúrate de que el archivo de salida contenga una de estas cabeceras:

```text
phi0_omega_k,phi1_u_eff_k,phi2_bias,y_omega_k1
```

ó

```text
phi0_omega_k,phi1_u_k,phi2_bias,y_omega_k1
```

---

## 13. Comandos rápidos

### Compilar

```bash
idf.py build
```

### Flashear y monitorear

```bash
idf.py -p <PUERTO> flash monitor
```

### Guardar log serial

```bash
idf.py -p <PUERTO> monitor | tee main/output.txt
```

### Estimar parámetros

```bash
python main/offline_estimation.py main/output.txt 0.02
```

---

## 14. Resumen

Este proyecto permite:

* Excitar un motor DC desde un ESP32
* Medir velocidad angular con encoder incremental
* Obtener un dataset listo para identificación
* Estimar un modelo discreto y continuo del motor con Python

Si quieres dejarlo todavía más redondo, el siguiente paso lógico sería agregar:

* un `requirements.txt` para Python
* un diagrama de conexiones
* guardado automático del log sin depender del monitor
* separación entre modo adquisición y modo control

