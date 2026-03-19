# README — angular_velocity_data

Proyecto para **adquisición de velocidad angular de un motor DC con ESP32**, usando:

- **ESP-IDF**
- **ESP32**
- **Driver L298N**
- **Encoder incremental en cuadratura** con **PCNT**
- Experimento de entrada por tramos: reposo → escalón positivo → reposo → escalón negativo → reposo
- Script en Python para **comparar la respuesta real** contra un **modelo continuo/discreto**

La idea del proyecto es sencilla:

1. El ESP32 aplica una secuencia de mando al motor.
2. El encoder mide la velocidad angular.
3. El firmware imprime un CSV por serial.
4. Ese CSV se guarda en un archivo.
5. Luego se usa `compare_plot.py` para comparar el experimento real contra un modelo matemático.

---

## 1. Estructura del proyecto

```text
.
├── CMakeLists.txt
├── sdkconfig
├── sdkconfig.old
└── main/
    ├── CMakeLists.txt
    ├── main.c
    ├── encoder_pcnt.c
    ├── encoder_pcnt.h
    ├── motor_l298.c
    ├── motor_l298.h
    ├── angular_velocity.txt
    └── compare_plot.py
```

---

## 2. Qué hace este proyecto

Este firmware ejecuta un experimento de adquisición de datos para el motor.

La secuencia definida en `main/main.c` es:

- un tramo inicial en `0`
- un tramo con PWM positivo
- otro tramo en `0`
- un tramo con PWM negativo
- un tramo final en `0`

Durante cada periodo de muestreo:

- se calcula el comando `pwm_k`
- se convierte a una entrada efectiva `u_eff_k` compensando la zona muerta
- se aplica el PWM al L298N
- se mide la velocidad angular con el encoder
- se imprime una fila CSV con el estado actual y el siguiente valor medido

La salida serial usa esta cabecera:

```csv
k,t_k1_s,pwm_k,u_eff_k,omega_k_rad_s,omega_k1_rad_s
```

---

## 3. Requisitos

### Hardware

- ESP32
- Driver **L298N**
- Motor DC con encoder incremental
- Fuente adecuada para el motor
- Tierra común entre potencia y control

### Software

#### Firmware

- **Visual Studio Code**
- **Extensión ESP-IDF** para VS Code
- Entorno ESP-IDF configurado correctamente

#### Validación en Python

- Python 3.10+
- `numpy`
- `pandas`
- `matplotlib`

Instalación rápida:

```bash
pip install numpy pandas matplotlib
```

---

## 4. Configuración de pines y parámetros principales

En `main/main.c` están definidos:

```c
#define ENC_A_GPIO      33
#define ENC_B_GPIO      25
#define L298_IN1_GPIO   32
#define L298_IN2_GPIO   27
#define L298_ENA_GPIO   26

#define ENCODER_CPR_X4  1976
#define TS_MS           20
#define PWM_FREQ_HZ     20000

#define PWM_POS         (0.70f)
#define PWM_NEG         (-0.70f)
```

### Significado

- `ENC_A_GPIO`, `ENC_B_GPIO`: canales del encoder
- `L298_IN1_GPIO`, `L298_IN2_GPIO`: dirección del motor
- `L298_ENA_GPIO`: PWM hacia el ENA del L298N
- `ENCODER_CPR_X4`: cuentas por vuelta en cuadratura x4
- `TS_MS`: periodo de muestreo
- `PWM_FREQ_HZ`: frecuencia PWM
- `PWM_POS`, `PWM_NEG`: niveles del experimento

También se compensa la zona muerta con:

```c
#define U_START_POS     0.55f
#define U_START_NEG     0.55f
```

Si tu motor necesita otro umbral para empezar a moverse, ajusta esos valores.

---

## 5. Corrección importante antes de compilar

En `main/CMakeLists.txt` aparece esto:

```cmake
idf_component_register(
    SRCS
        "encoder_pcnt"
        "motor_l298"
        "main"
    INCLUDE_DIRS
        "."
)
```

Eso está mal para ESP-IDF. Deben ponerse los archivos con extensión `.c`.

Déjalo así:

```cmake
idf_component_register(
    SRCS
        "encoder_pcnt.c"
        "motor_l298.c"
        "main.c"
    INCLUDE_DIRS
        "."
)
```

Si no haces ese cambio, el build no va a salir. Aquí el compilador no perdona distracciones.

---

## 6. Cómo compilar, flashear y monitorear

En este repo, el flujo principal se realiza con la **extensión ESP-IDF de VS Code**.

### Flujo recomendado en VS Code

1. Abre la carpeta del proyecto en **Visual Studio Code**.
2. Verifica que la extensión **ESP-IDF** esté instalada y configurada.
3. Abre la paleta de comandos o usa los botones de la extensión.
4. Selecciona el target **ESP32** si hace falta.
5. Ejecuta desde la extensión:
   - **Build Project**
   - **Flash Device**
   - **Monitor Device**
6. Selecciona el puerto serial correcto del ESP32.

### Observaciones

- El archivo `sdkconfig` ya está incluido, así que normalmente el proyecto ya trae una configuración base.
- Si algo falla al compilar, revisa primero la corrección del `main/CMakeLists.txt`.
- Si cambias pines, PWM o tiempos de muestreo, vuelve a compilar antes de flashear.

---

## 7. Qué imprime el firmware

El firmware imprime un CSV por el monitor serial con filas como esta:

```csv
k,t_k1_s,pwm_k,u_eff_k,omega_k_rad_s,omega_k1_rad_s
0,0.020000,0.000000,0.000000,0.00000000,0.00000000
1,0.040000,0.000000,0.000000,0.00000000,0.00000000
...
```

Columnas:

- `k`: índice de muestra
- `t_k1_s`: tiempo acumulado de la muestra siguiente
- `pwm_k`: comando PWM aplicado
- `u_eff_k`: entrada efectiva compensada por zona muerta
- `omega_k_rad_s`: velocidad actual
- `omega_k1_rad_s`: velocidad siguiente

---

## 8. Guardar la salida del monitor en un archivo

Para validar el modelo necesitas guardar la salida CSV del experimento.

Puedes copiar el contenido desde el monitor serial y guardarlo en un archivo `.txt`, por ejemplo:

```text
angular_velocity.txt
```

En este repo ya viene un ejemplo de archivo:

```text
main/angular_velocity.txt
```

Ese archivo puede usarse directamente para probar el script Python.

---

## 9. Cómo correr la validación en Python

El script `main/compare_plot.py`:

- carga el archivo de log
- detecta la cabecera válida
- toma `a`, `b` y `c` del modelo continuo
- convierte esos parámetros a una forma discreta
- simula la salida del modelo
- compara la velocidad real vs la velocidad simulada
- calcula métricas y muestra la gráfica

### Ejecución básica

```bash
python main/compare_plot.py main/angular_velocity.txt
```

### Ejecución con parámetros del modelo

```bash
python main/compare_plot.py main/angular_velocity.txt --a 8.0 --b 25.0 --c 0.0
```

### Parámetros aceptados

- `archivo_log`: archivo `.txt` o `.log` con la cabecera del CSV
- `--a`: parámetro continuo `a`
- `--b`: parámetro continuo `b`
- `--c`: parámetro continuo `c`
- `--title`: título de la gráfica

Ejemplo:

```bash
python main/compare_plot.py main/angular_velocity.txt \
  --a 8.0 \
  --b 25.0 \
  --c 0.0 \
  --title "Validación del modelo del motor"
```

---

## 10. Qué muestra el script de validación

El script imprime:

### Parámetros continuos

- `a`
- `b`
- `c`

### Parámetros discretos

- `Ts`
- `alpha`
- `beta`
- `gamma`

### Métricas

- `RMSE`
- `MAE`
- `R^2`
- `Max error`

Además abre una gráfica con:

- velocidad real
- velocidad simulada
- cuadro con métricas

---

## 11. Recomendación de organización del repo

En este momento tienes dentro de `main/` tanto el firmware como archivos de apoyo para validación:

- `angular_velocity.txt`
- `compare_plot.py`

Funciona, sí. Elegante, no tanto.

### Recomendación

Deja `main/` únicamente para el código del firmware y mueve los archivos de validación a una carpeta aparte.

### Nombre sugerido para esa carpeta

La mejor opción aquí sería:

```text
validation
```

Quedaría así:

```text
.
├── CMakeLists.txt
├── sdkconfig
├── main/
│   ├── CMakeLists.txt
│   ├── main.c
│   ├── encoder_pcnt.c
│   ├── encoder_pcnt.h
│   ├── motor_l298.c
│   └── motor_l298.h
└── validation/
    ├── angular_velocity.txt
    └── compare_plot.py
```

### Por qué `validation`

Porque esa carpeta no es firmware, ni drivers, ni componentes del build. Su función real es:

- guardar logs del experimento
- ejecutar comparación con el modelo
- apoyar la validación del sistema

### Otras opciones válidas

Si quieres un nombre más técnico o más académico:

- `experiments`
- `model_validation`
- `identification`

Pero si quieres algo claro, limpio y mantenible: **`validation`**.

---

## 12. Recomendaciones de conexión

### Encoder

- Canal A → `GPIO 33`
- Canal B → `GPIO 25`
- GND común con ESP32

### L298N

- `IN1` → `GPIO 32`
- `IN2` → `GPIO 27`
- `ENA` → `GPIO 26`
- GND del driver unido al GND del ESP32
- Fuente del motor separada o con capacidad suficiente

### Importante

- No alimentes el motor desde una salida que no soporte la corriente real.
- Usa tierra común entre electrónica de control y potencia.
- Revisa bien el sentido del encoder y del motor antes de sacar conclusiones del modelo.

---

## 13. Troubleshooting

### No compila el proyecto

Revisa `main/CMakeLists.txt`.

Lo más probable es que el problema sea que faltan las extensiones `.c` en `SRCS`.

### El motor no se mueve con PWM bajos

Ajusta:

```c
#define U_START_POS 0.55f
#define U_START_NEG 0.55f
```

Eso controla la compensación de la zona muerta.

### La velocidad sale con signo invertido

Revisa en `encoder_pcnt.c`:

```c
#define ENC_INVERT_DIR 0
```

Si el signo está invertido, cambia a:

```c
#define ENC_INVERT_DIR 1
```

### La gráfica no coincide con el modelo

Eso no necesariamente significa que el firmware esté mal. Puede significar que los parámetros `a`, `b` y `c` no representan bien a tu motor.

### El script Python no encuentra la cabecera

El archivo debe contener exactamente esta línea:

```text
k,t_k1_s,pwm_k,u_eff_k,omega_k_rad_s,omega_k1_rad_s
```

---

## 14. Resumen

Este proyecto sirve para:

- medir velocidad angular con encoder en ESP32
- excitar el motor con una secuencia definida
- registrar datos experimentales
- comparar la respuesta real contra un modelo matemático

Y el siguiente paso lógico, si quieres dejarlo más profesional, sería:

- mover `compare_plot.py` y los logs a `validation/`
- agregar un `requirements.txt`
- guardar varios experimentos en una subcarpeta `validation/logs/`
- documentar parámetros validados del motor en un archivo aparte

---

## 15. Licencia

No se encontró una licencia explícita en el proyecto. Si lo vas a compartir o subir a un repositorio público, conviene agregar una.
