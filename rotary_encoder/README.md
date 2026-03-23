# README — ESP-IDF + Encoder cuadratura con PCNT (x4)

## Descripción

Este proyecto usa el periférico **PCNT** del **ESP32** para leer un **encoder incremental en cuadratura** en modo **x4**, es decir, contando los cuatro flancos útiles de las señales **A** y **B**.

El código actual está configurado para:

* **Canal A del encoder → GPIO 33**
* **Canal B del encoder → GPIO 25**
* **Pull-up interno habilitado**
* **Muestreo cada 50 ms**
* **Filtro anti-rebote / anti-ruido = 8000 ns**

Esto se define directamente en `main.c`:

```c
#define ENC_A_GPIO 33
#define ENC_B_GPIO 25
#define ENC_SWAP_AB    1
#define ENC_INVERT_DIR 0
#define SAMPLE_PERIOD_MS 50
#define GLITCH_NS 8000
```

---

## Objetivo de este README

El objetivo principal es dejar claro **cómo conectar correctamente el encoder** para que las mediciones salgan bien desde el primer intento, y qué revisar cuando:

* el conteo va al revés,
* el conteo brinca solo,
* no cuenta,
* o cuenta mal.

---

## Tipo de encoder esperado

Este proyecto espera un encoder incremental con señales:

* **A**
* **B**
* **GND común**

En algunos encoders también existe:

* **VCC**

### Casos típicos

#### 1. Encoder mecánico simple de 3 pines

Normalmente tiene:

* A
* C o COM
* B

En este caso:

* **COM/C → GND del ESP32**
* **A → GPIO 33**
* **B → GPIO 25**

No requiere alimentar `VCC` porque el código ya activa **pull-up interno** en ambos GPIO.

#### 2. Encoder integrado en motor DC con salidas A/B

Muchos motores con encoder traen más cables, por ejemplo:

* alimentación del motor,
* alimentación del encoder,
* GND,
* A,
* B.

En este caso:

* **No mezclar cables del motor con cables del encoder**.
* El encoder debe compartir **GND** con el ESP32.
* Las señales **A** y **B** deben llegar al ESP32 con nivel lógico seguro.

> **Importante:** los GPIO del ESP32 **no son tolerantes a 5 V**. Si el encoder entrega 5 V en A/B, debes usar adaptación de nivel antes de entrar al ESP32.

---

## Conexión recomendada

### Opción 1 — Encoder de 3 pines (mecánico)

```text
Encoder         ESP32
--------------------------
A       ------> GPIO 33
COM/C   ------> GND
B       ------> GPIO 25
```

### Opción 2 — Encoder activo con alimentación

```text
Encoder         ESP32
--------------------------
VCC     ------> 3.3 V (*)
GND     ------> GND
A       ------> GPIO 33
B       ------> GPIO 25
```

`(*)` Solo si el encoder trabaja correctamente a 3.3 V.

Si el encoder **requiere 5 V** para funcionar:

* aliméntalo según su especificación,
* pero **no conectes A/B directamente al ESP32 si salen a 5 V**,
* usa **level shifter**, divisor resistivo o una interfaz apropiada.

---

## Pinout actual del proyecto

El proyecto ya está cableado lógicamente así:

```c
#define ENC_A_GPIO 33
#define ENC_B_GPIO 25
```

Por lo tanto, físicamente debes conectar:

* **Señal A del encoder → GPIO 33**
* **Señal B del encoder → GPIO 25**
* **GND del encoder → GND del ESP32**

Además, en este código está activado:

```c
#define ENC_SWAP_AB 1
#define ENC_INVERT_DIR 0
```

Eso significa que internamente el software ya está compensando el orden A/B para que la dirección salga correcta con tu montaje actual.

---

## Cómo obtener el pinout correcto si no conoces los cables del encoder

Si no estás 100% seguro de cuáles cables son **A**, **B**, **GND** y **VCC**, sigue este procedimiento.

### Paso 1 — Identifica si tu encoder es pasivo o activo

#### Encoder pasivo / mecánico

Suele tener 3 terminales:

* A
* COM/C
* B

No necesita alimentación independiente.

#### Encoder activo

Suele tener 4 o más cables:

* VCC
* GND
* A
* B
* a veces Z o Index

Este sí necesita alimentación.

---

### Paso 2 — Encuentra GND y VCC

Usa hoja de datos o multímetro.

#### Si tienes datasheet

Primero verifica el pinout ahí. Es el camino serio y el que evita humo caro.

#### Si no tienes datasheet

Haz lo siguiente:

* Identifica pares de cables relacionados al encoder, no al motor.
* Busca un cable común que sirva como referencia para A y B.
* Si hay cable de alimentación, identifica cuál es GND y cuál es VCC.
* Si el encoder pertenece a un motor con caja reductora, muchas veces el encoder sale por un conector aparte o comparte arnés con el motor.

> Nunca adivines VCC y lo conectes directo al ESP32. Primero verifica niveles.

---

### Paso 3 — Verifica señales A y B

Con el encoder energizado correctamente:

* gira lentamente el eje,
* mide A respecto a GND,
* mide B respecto a GND.

Debes observar dos señales digitales desfasadas.

Lo ideal es verificar con:

* **osciloscopio**, o
* **analizador lógico**.

También puedes usar multímetro, aunque es mucho menos claro.

#### Lo que debes ver

* A cambia entre 0 y 1 lógico.
* B cambia entre 0 y 1 lógico.
* A y B no cambian exactamente al mismo tiempo; están desfasadas.

---

## Cómo validar que la conexión quedó correcta

Una vez conectado, ejecuta:

```bash
idf.py flash monitor
```

Debes ver mensajes parecidos a:

```text
I (...) ENCODER x4: delta=4 cnt | total=4 cnt | dir=FWD | glitch=8000ns
I (...) ENCODER x4: delta=-4 cnt | total=0 cnt | dir=REV | glitch=8000ns
```

### Interpretación

* `delta > 0` → movimiento en una dirección.
* `delta < 0` → movimiento en la dirección contraria.
* `delta = 0` → sin movimiento.
* `total` → acumulado total desde el arranque.

---

## Cómo saber si A y B están invertidos

Si giras el eje hacia adelante y el sistema reporta la dirección al revés, tienes tres opciones:

### Opción A — Intercambiar físicamente A y B

Cambiar:

* A ↔ B

### Opción B — Mantener el cableado y usar software

Tu código ya contempla esto con:

```c
#define ENC_SWAP_AB 1
#define ENC_INVERT_DIR 0
```

### Opción C — Solo invertir dirección lógica

Si el conteo está bien pero el sentido quedó invertido:

```c
#define ENC_INVERT_DIR 1
```

### Recomendación práctica

* Si todo cuenta bien pero la dirección sale al revés, usa **`ENC_INVERT_DIR 1`**.
* Si sospechas que A y B están conectados cruzados desde origen, usa **`ENC_SWAP_AB 1`** o intercambia el cableado.

---

## Cómo saber si el conteo es correcto

Este proyecto trabaja en **modo x4**.

Eso significa que:

* cada ciclo completo de cuadratura produce **4 cuentas**,
* por eso no debes esperar “1 cuenta por clic” salvo que ya hayas convertido la escala.

Si tu encoder o motor especifica algo como:

```c
#define ENC_PPR_DATASHEET 493.9f
```

debes tener claro si ese valor corresponde a:

* pulsos por vuelta del eje del encoder,
* cuentas por vuelta ya considerando cuadratura,
* o pulsos por vuelta del eje de salida del motor con reducción.

Eso cambia completamente la interpretación.

### Regla práctica

Antes de convertir a RPM o ángulo:

1. gira manualmente una vuelta completa,
2. observa cuánto cambia `total`,
3. compara contra lo esperado del datasheet.

Si no coincide, revisa:

* si el datasheet habla de **PPR**, **CPR** o **líneas**,
* si el conteo es **x1**, **x2** o **x4**,
* si estás midiendo eje de motor o eje con reductora.

---

## Problemas comunes y solución

### 1. No cuenta nada

Revisa:

* que el encoder comparta **GND** con el ESP32,
* que A realmente esté en **GPIO 33**,
* que B realmente esté en **GPIO 25**,
* que el encoder tenga alimentación si la necesita,
* que las señales no estén saliendo a 5 V directo al ESP32.

También valida que el encoder realmente genere cuadratura y no otro tipo de señal.

---

### 2. Cuenta pero mete ruido o brincos falsos

Revisa:

* cableado largo,
* tierra deficiente,
* ruido del motor,
* rebote mecánico,
* ausencia de desacoplo.

Puedes mejorar con:

* cables cortos,
* GND común sólido,
* separación física entre motor y señales del encoder,
* pull-up externo de **4.7 kΩ a 10 kΩ** si hace falta,
* ajustar el filtro:

```c
#define GLITCH_NS 8000
```

Si sigue habiendo ruido, prueba subirlo gradualmente.

---

### 3. Cuenta al revés

Solución rápida:

* cambia `ENC_INVERT_DIR` a `1`, o
* intercambia A y B, o
* ajusta `ENC_SWAP_AB`.

---

### 4. Cuenta menos o más de lo esperado

Revisa:

* interpretación del datasheet,
* si estás usando x4,
* si el encoder tiene reducción mecánica asociada,
* velocidad de giro demasiado alta,
* o si el filtro está eliminando pulsos válidos.

---

### 5. El motor mete demasiado ruido al encoder

Muy común en motores DC.

Recomendaciones:

* usa fuente estable,
* comparte GND correctamente,
* separa alimentación del motor y señales,
* agrega filtrado eléctrico si aplica,
* revisa con osciloscopio antes de culpar al software.

El software ayuda, pero no hace milagros. Si el cableado está mal, el contador también va a rezar raro.

---

## Procedimiento recomendado de puesta en marcha

1. **Conecta solo encoder y ESP32**, sin poner carga rara al sistema.
2. Confirma:

   * A → GPIO 33
   * B → GPIO 25
   * GND → GND
3. Si el encoder es activo, alimenta con el voltaje correcto.
4. Flashea el proyecto.
5. Abre monitor serie.
6. Gira lentamente el encoder.
7. Verifica que:

   * el contador cambia,
   * el signo cambia al invertir giro,
   * no haya brincos sin mover el eje.
8. Si la dirección está invertida, ajusta `ENC_INVERT_DIR` o `ENC_SWAP_AB`.
9. Si hay ruido, ajusta `GLITCH_NS` y mejora cableado.
10. Solo después convierte cuentas a RPM, posición o velocidad angular.

---

## Resumen rápido de conexión correcta

### Para este `main.c`

```text
Encoder A  -> GPIO 33
Encoder B  -> GPIO 25
Encoder GND -> GND ESP32
```

Si el encoder requiere alimentación:

```text
Encoder VCC -> 3.3 V o fuente adecuada según datasheet
Encoder GND -> GND común con ESP32
```

### Para tener mediciones correctas debes cumplir sí o sí con esto:

* A y B bien identificados
* GND común
* niveles lógicos compatibles con ESP32
* dirección validada en monitor
* ruido controlado
* interpretación correcta de PPR/x4

---

## Archivo base considerado

Este README fue redactado tomando como base el comportamiento implementado en `main.c` del proyecto, especialmente:

* `ENC_A_GPIO 33`
* `ENC_B_GPIO 25`
* `ENC_SWAP_AB 1`
* `ENC_INVERT_DIR 0`
* `GLITCH_NS 8000`
* lectura con PCNT en cuadratura x4
