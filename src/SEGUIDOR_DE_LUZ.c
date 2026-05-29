#include "LPC17xx.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_gpdma.h"
#include "lpc17xx_uart.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_systick.h"

//-----------------------DECLARACIONES Y DEFINES VARIOS PARA EL MOTOR--------------------------------------

// DEFINES PARA LA DIRECCION DEL MOTOR
#define DIRECCION_RELOJ   1
#define DIRECCION_CONTRA  0

// REDUZCO LA CANTIDAD DE ANGULOS Y PASOS PARA NO TENER FLOATS (4096 PASOS / 360 GRADOS = 512/45)
#define MOTOR_PASOS_NUM   256
#define MOTOR_PASOS_DEN   45

// ENUMS INDIVIDUALES PARA LOS PINES DEL STEP MOTOR
typedef enum
{
    MOTOR_MASK_IN1 = (1 << PIN_6),  // P0[6]
    MOTOR_MASK_IN2 = (1 << PIN_7),  // P0[7]
    MOTOR_MASK_IN3 = (1 << PIN_8),  // P0[8]
    MOTOR_MASK_IN4 = (1 << PIN_9)   // P0[9]
} StepperMasks;

// MATRIZ DE SECUENCIA DEL  MOTOR (2 BOBINAS A LA VEZ POR SEGURIDAD)
// TRABAJA CON MASCARAS
static const uint32_t secuencia_stepper[4] =
{
    (MOTOR_MASK_IN1 | MOTOR_MASK_IN2), // PASO 0
    (MOTOR_MASK_IN2 | MOTOR_MASK_IN3), // PASO 1
    (MOTOR_MASK_IN3 | MOTOR_MASK_IN4), // PASO 2
    (MOTOR_MASK_IN4 | MOTOR_MASK_IN1)  // PASO 3
};

typedef enum
{
    MOTOR_IN1 = 6,   // P0[6]
    MOTOR_IN2 = 7,   // P0[7]
    MOTOR_IN3 = 8,   // P0[8]
    MOTOR_IN4 = 9    // P0[9]
} StepperPins;

// VARIABLES GLOBALES
static uint16_t cant_steps = 0;

// AGRUPO TODAS LAS MASCARAS EN UNA DE 32BITS PARA PODER MOVER COMPLETO MAS FACIL
#define ALL_MOTOR_PINS  (MOTOR_MASK_IN1 | MOTOR_MASK_IN2 | MOTOR_MASK_IN3 | MOTOR_MASK_IN4)

// DECLARACION DE FUNCIONES
void Stepper_Init(void);
void Stepper_Step(uint8_t direction);
void Stepper_MoveToAngle(uint16_t angulo_destino);

//-----------------------DECLARACIONESS Y DEFINES VARIOS PARA EL TECLADO--------------------------------------

// ENUMS PARA LAS MASCARAS DEL TECLADO CON LOS PINES INVERTIDOS
typedef enum
{
    // COLUMNAS EN J6-11 A J6-14 (AHORA SON ENTRADAS CON INTERRUPCION)
    TECLADO_COL_1  = (1 << PIN_18),  // P0[18]
    TECLADO_COL_2  = (1 << PIN_17),  // P0[17]
    TECLADO_COL_3  = (1 << PIN_15),  // P0[15]
    TECLADO_COL_4  = (1 << PIN_16),  // P0[16]

    // FILAS EN J6-15 A J6-18 (AHORA SON SALIDAS DIGITALES)
    TECLADO_FILA_1 = (1 << PIN_23),  // P0[23]
    TECLADO_FILA_2 = (1 << PIN_24),  // P0[24]
    TECLADO_FILA_3 = (1 << PIN_25),  // P0[25]
    TECLADO_FILA_4 = (1 << PIN_26)   // P0[26]
} MaskTeclado;

// MASCARAS AGRUPADAS ACTUALIZADAS CON EL INTERCAMBIO
#define TECLADO_FILAS (TECLADO_FILA_1 | TECLADO_FILA_2 | TECLADO_FILA_3 | TECLADO_FILA_4)
#define TECLADO_COLUMNAS  (TECLADO_COL_1 | TECLADO_COL_2 | TECLADO_COL_3 | TECLADO_COL_4)

// VARIABLE GLOBAL PARA GUARDAR LA TECLA DETECTADA
volatile uint8_t tecla_presionada = 0;

// DECLARACION DE FUNCIONES
void Teclado_Init(void);
uint8_t Teclado_Scan(void);

//-----------------------DECLARACIONESS Y DEFINES VARIOS PARA EL ADC-------------------------------------

// SUPONEMOS UN PANEL DE 5V (5000 mV)
#define PANEL_VMAX_MV  5000

void ADC_Config(void);
uint16_t ADC_Leer_Canal(ADC_CHANNEL);
uint16_t Calcular_Voltaje_Panel(uint16_t);


//-----------------------DECLARACIONES Y AUXILIARES PARA UART--------------------------------------
void UART0_InitConsole(void);
void UART0_SendString(char* str);
void UART0_SendNumber(uint16_t num); // ACTUALIZADO A UINT16_T


//-----------------------CODIGO MAIN--------------------------------------
int main(void)
{
    uint16_t luz_derecha = 0;
    uint16_t luz_izquierda = 0;
    uint16_t luz_promedio = 0;
    uint16_t voltaje_panel = 0;

    // INICIALIZAMOS TODOS LOS PERIFERICOS
    Stepper_Init();
    Teclado_Init();
    ADC_Config();
    UART0_InitConsole();

    UART0_SendString("CONSOLA INICIALIZADA. LEYENDO ADC...\n\r");

    while(1)
    {
        // LEEMOS LOS CANALES 4 Y 5 (PUERTO 1) DE FORMA CONTINUA
        luz_derecha = ADC_Leer_Canal(ADC_CHANNEL_4);
        luz_izquierda = ADC_Leer_Canal(ADC_CHANNEL_5);

        luz_promedio = (luz_derecha + luz_izquierda) / 2;
        voltaje_panel = Calcular_Voltaje_Panel(luz_promedio);

        // IMPRIMIMOS AL TOQUE
        UART0_SendString("DERECHA: ");
        UART0_SendNumber(luz_derecha);
        UART0_SendString(" | IZQUIERDA: ");
        UART0_SendNumber(luz_izquierda);
        UART0_SendString(" | VOLTAJEPROMEDIO: ");
        UART0_SendNumber(voltaje_panel);
        UART0_SendString("\n\r");

        // UN PEQUEÑO RETARDO MANUAL PARA PODER LEER LA PANTALLA
        for(volatile uint32_t i = 0; i < 2000000; i++);
    }

    return 0;
}

//-----------------------IMPLEMENTACIONES--------------------------------------

void Stepper_Init()
{
    PINSEL_CFG_T pinConfig;

    pinConfig.port = PORT_0;
    pinConfig.func = PINSEL_FUNC_00;
    pinConfig.mode = PINSEL_PULLUP;
    pinConfig.openDrain = DISABLE;

    PINSEL_ConfigMultiplePins(&pinConfig, ALL_MOTOR_PINS);

    GPIO_SetDir(PORT_0, ALL_MOTOR_PINS, GPIO_OUTPUT);

    GPIO_ClearPins(PORT_0, ALL_MOTOR_PINS);

    cant_steps = 0;
}

void Stepper_Step(uint8_t direccion)
{
    static uint8_t step_actual = 0;

    if (direccion == DIRECCION_RELOJ)
    {
        step_actual = (step_actual + 1) % 4;
        cant_steps++;
    }
    else
    {
        if (step_actual == 0)
        {
            step_actual = 3;
        }
        else
        {
            step_actual = step_actual - 1;
        }
        cant_steps--;
    }

    GPIO_ClearPins(PORT_0, ALL_MOTOR_PINS);
    GPIO_SetPins(PORT_0, secuencia_stepper[step_actual]);
}

void Stepper_MoveToAngle(uint16_t angulo_destino)
{
    uint16_t paso_objetivo = ((angulo_destino * MOTOR_PASOS_NUM) + (MOTOR_PASOS_DEN / 2)) / MOTOR_PASOS_DEN;

    while (cant_steps != paso_objetivo)
    {
        if (SYSTICK_HasFired() == SET)
        {
            if (cant_steps < paso_objetivo)
            {
                Stepper_Step(DIRECCION_RELOJ);
            }
            else
            {
                Stepper_Step(DIRECCION_CONTRA);
            }
            SYSTICK_ClearCounterFlag();
        }
    }
}

void Teclado_Init(void)
{
    PINSEL_CFG_T tecladoPinConfig;

    tecladoPinConfig.port = PORT_0;
    tecladoPinConfig.func = PINSEL_FUNC_00;
    tecladoPinConfig.mode = PINSEL_PULLUP;
    tecladoPinConfig.openDrain = DISABLE;

    PINSEL_ConfigMultiplePins(&tecladoPinConfig, TECLADO_FILAS);
    PINSEL_ConfigMultiplePins(&tecladoPinConfig, TECLADO_COLUMNAS);

    // SE INVIERTEN LAS DIRECCIONES DE LOS GRUPOS DE PINES
    GPIO_SetDir(PORT_0, TECLADO_FILAS, GPIO_OUTPUT);    // FILAS (J6-15 A 18) AHORA SON SALIDAS
    GPIO_SetDir(PORT_0, TECLADO_COLUMNAS, GPIO_INPUT);  // COLUMNAS (J6-11 A 14) AHORA SON ENTRADAS

    // DEJAMOS TODAS LAS FILAS EN 0 LOGICO EN EL REPOSO
    GPIO_ClearPins(PORT_0, TECLADO_FILAS);

    // LA INTERRUPCION DE FLANCO DESCENDENTE SE PASA A LAS NUEVAS COLUMNAS (P0.15, P0.16, P0.17, P0.18)
    GPIO_IntConfigPort(PORT_0, TECLADO_COLUMNAS, GPIO_INT_FALLING);

    NVIC_ClearPendingIRQ(EINT3_IRQn);
    NVIC_EnableIRQ(EINT3_IRQn);
}

uint8_t Teclado_Scan(void)
{
    static const uint8_t mapa_teclas[4][4] =
    {
        {1,  2,  3,  4},
        {5,  6,  7,  8},
        {9,  10, 11, 12},
        {13, 14, 15, 16}
    };

    // SE ACTUALIZAN LOS ARREGLOS CON LAS NUEVAS ASIGNACIONES DE PINES
    uint32_t filas[4] = {TECLADO_FILA_1, TECLADO_FILA_2, TECLADO_FILA_3, TECLADO_FILA_4};
    uint32_t columnas[4] = {TECLADO_COL_1, TECLADO_COL_2, TECLADO_COL_3, TECLADO_COL_4};
    uint8_t tecla_local = 0;

    // SE LEVANTAN LAS FILAS A 1 LOGICO PARA INICIAR BARRIDO
    GPIO_SetPins(PORT_0, TECLADO_FILAS);

    for (uint8_t f = 0; f < 4; f++)
    {
        // PASO LA FILA EVALUADA A 0 LOGICO
        GPIO_ClearPins(PORT_0, filas[f]);

        for (uint8_t c = 0; c < 4; c++)
        {
            if ((GPIO_ReadValue(PORT_0) & columnas[c]) == 0)
            {
                tecla_local = mapa_teclas[f][c];
                break;
            }
        }

        GPIO_SetPins(PORT_0, filas[f]);

        if (tecla_local != 0)
        {
            break;
        }
    }

    // REESTABLEZCO LAS FILAS EN BAJO PARA LA PROXIMA INTERRUPCION
    GPIO_ClearPins(PORT_0, TECLADO_FILAS);
    return tecla_local;
}

void EINT3_IRQHandler(void)
{
    if (GPIO_GetPortIntStatus(PORT_0) == SET)
    {
        // COMPRUEBO LAS NUEVAS COLUMNAS EN EL MANEJADOR (PIN_15, PIN_16, PIN_17, PIN_18)
        if (GPIO_GetPinIntStatus(PORT_0, PIN_18, GPIO_INT_FALLING) == SET ||
            GPIO_GetPinIntStatus(PORT_0, PIN_17, GPIO_INT_FALLING) == SET ||
            GPIO_GetPinIntStatus(PORT_0, PIN_15, GPIO_INT_FALLING) == SET ||
            GPIO_GetPinIntStatus(PORT_0, PIN_16, GPIO_INT_FALLING) == SET)
        {
            tecla_presionada = Teclado_Scan();

            // SE LIMPIAN LAS BANDERAS DE LA INTERRUPCION DE LAS NUEVAS COLUMNAS
            GPIO_ClearInt(PORT_0, TECLADO_COLUMNAS);
        }
    }
}

void ADC_Config(void)
{
    ADC_Init(100000);
    ADC_PinConfig(ADC_CHANNEL_4);
    ADC_PinConfig(ADC_CHANNEL_5);
}

uint16_t ADC_Leer_Canal(ADC_CHANNEL canal)
{
    uint16_t valor_adc = 0;

    //PRENDO EL CANAL SEGUN PARAMETRO
    ADC_ChannelEnable(canal);

    //MANDO A CONVERTIR
    ADC_StartCmd(ADC_START_NOW);

    //METO UN WHILE HASTA QUE TERMINE
    while (ADC_ChannelGetStatus(canal, ADC_DATA_DONE) == RESET)
    {
        // BUCLE CORTO DE 10US
    }

    // GUARDO DATOS UY DE PASO LIMPIO BANDERA
    valor_adc = ADC_ChannelGetData(canal);

    ADC_ChannelDisable(canal); //APAGO EL CANAL DE NUEVO

    // DEVUEVLO EL VALOR MEDIDO
    return valor_adc;
}

uint16_t Calcular_Voltaje_Panel(uint16_t valor_adc)
{
    // MULTIPLICAMOS PRIMERO, SUMAMOS EL REDONDEO (CUANDO SUMO LA MITAD DEL DENOM. REDONDEO DE LA FORMA NATURAL) Y DIVIDIMOS AL FINAL
    // USAMOS UINT32_T TEMPORALMENTE PORQUE 4095 * 5000 = 20.475.000 (DESBORDA 16 BITS)
    uint32_t calculo = ((uint32_t)valor_adc * PANEL_VMAX_MV) + (4095 / 2);

    return (uint16_t)(calculo / 4095);
}


//-----------------------IMPLEMENTACIONES AUXILIARES PARA EL MODULO UART--------------------------------------

void UART0_InitConsole(void)
{
    UART_CFG_T uartConfig;

    // CONFIGURAMOS LOS PINES FISICOS DE LA UART0 (P0.2 COMO TX0 Y P0.3 COMO RX0)
    UART_PinConfig(UART_TX0_P0_2);
    UART_PinConfig(UART_RX0_P0_3);

    // RELLENAMOS LA ESTRUCTURA DE CONFIGURACION REQUERIDA POR EL DRIVER
    uartConfig.baudRate = 9600;
    uartConfig.dataBits = UART_DBITS_8;
    uartConfig.parity   = UART_PARITY_NONE;
    uartConfig.stopBits = UART_STOPBIT_1;

    // INICIALIZAMOS EL PERIFERICO
    UART_Init(UART0, &uartConfig);

    // ACTIVAMOS LA CAPACIDAD DE TRANSMISION DEL PERIFERICO UART0
    UART_TxEnable(UART0);
}

void UART0_SendString(char* str)
{
    uint32_t longitud = 0;

    // CALCULAMOS LA LONGITUD DE LA CADENA DE TEXTO HASTA ENCONTRAR EL CARACTER NULO
    while (str[longitud] != '\0')
    {
        longitud++;
    }

    // MANDAMOS EL BLOQUE COMPLETO EN MODO BLOCKING
    UART_Send(UART0, (uint8_t*)str, longitud, BLOCKING);
}

void UART0_SendNumber(uint16_t num) // AHORA RECIBE UINT16_T (HASTA 65535)
{
    char buffer_ascci[6]; // CAPACIDAD PARA 5 DIGITOS MAS EL CARACTER NULO
    char buffer_invertido[6];
    uint8_t i = 0;
    uint8_t j = 0;

    // CASO ESPECIAL SI EL NUMERO ES EXACTAMENTE 0
    if (num == 0)
    {
        buffer_ascci[0] = '0';
        buffer_ascci[1] = '\0';
        UART0_SendString(buffer_ascci);
        return;
    }

    // EXTRAEMOS LOS DIGITOS MATEMATICAMENTE (QUEDAN AL REVES EN EL BUFFER)
    while (num > 0)
    {
        buffer_invertido[i] = (num % 10) + '0'; // EXTRAE EL ULTIMO DIGITO Y LO PASA A ASCII
        num = num / 10;                         // ACHICA EL NUMERO
        i++;
    }

    // INVERTIMOS EL ORDEN PARA QUE SE PUEDA IMPRIMIR BIEN EN LA PANTALLA
    while (i > 0)
    {
        i--;
        buffer_ascci[j] = buffer_invertido[i];
        j++;
    }

    // CERRAMOS EL STRING
    buffer_ascci[j] = '\0';

    // ENVIAMOS EL TEXTO RESULTANTE POR LA UART
    UART0_SendString(buffer_ascci);
}












