# Propuesta de Proyecto Final (Modificada)

## 1. Definición del sistema
El sistema es un control de dirección sobre un **motor paso a paso (step motor)**, a través de **dos fotorresistencias**, un **teclado matricial de 4x4** y un **display LCD**.

* **Modo Tracking:** El motor funcionará de manera automatizada buscando la mayor intensidad lumínica del entorno.
* **Alternancia de modos:** A través del teclado se podrán alternar los modos de operación, permitiendo seleccionar la orientación del sistema en un punto específico (**Modo Fijo**) o modificar su posición con los botones de dirección (**Modo Manual**).
* **Display LCD:** El display LCD físico se utilizará únicamente para indicar el modo actual de operación del sistema (Tracking, Manual o Fijo).
* **Comunicación UART:** Toda la información relevante del sistema como los niveles de luz captados por cada sensor, la posición angular del motor y otros datos de interés, será enviada mediante comunicación UART hacia una computadora. En esta, se implementará una interfaz gráfica que permitirá visualizar dicha información en tiempo real.

---

## 2. Definición de comportamiento

### FOTORRESISTENCIAS (LDR)
Son las entradas analógicas del sistema y estarán conectadas a distintos canales del ADC. Las variaciones de luz modifican su resistencia interna, cambiando así su tensión, que será luego interpretada por el LPC para determinar hacia dónde moverse.

### ADC
Lee y convierte los valores analógicos proporcionados por las fotorresistencias para su posterior procesamiento.

### STEP MOTOR
Girará de acuerdo al sistema para obtener la mayor cantidad de luz posible (Modo Tracking) o según decida el usuario (Modo Manual o Modo Fijo).

### DISPLAY LCD
Se utilizará como indicador de estado del sistema. Mostrará únicamente el modo de operación actual:
* Tracking
* Manual
* Fijo

### INTERFAZ UART (COMPUTADORA)
Se encargará de transmitir la información del sistema hacia una computadora, donde se visualizará mediante una interfaz gráfica. Entre los datos enviados se incluyen:
* Niveles de luz de cada fotorresistencia
* Ángulo actual del motor
* Estado del sistema
* Otros parámetros relevantes

### TECLADO MATRICIAL 4x4
Periférico de entrada principal utilizado para la gestión de modos, velocidades y posicionamiento:

* **CAMBIO DE MODO:** Un botón *toggle* permitirá cambiar entre los modos Tracking, Manual o Fijo.
* **BOTONES DE DIRECCIÓN:** Dos botones que funcionarán como comandos de izquierda y derecha para orientar la estructura de forma manual paso a paso, únicamente cuando el Modo Manual esté activo.
* **POSICIONES O ÁNGULOS FIJOS:** Teclas numéricas asignadas
* **BOTÓN DE RESET:** Botón que simula una posición “0” o “Home” de calibración inicial.
