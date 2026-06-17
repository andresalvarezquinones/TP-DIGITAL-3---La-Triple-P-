#include "LPC17xx.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_gpdma.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_systick.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_uart.h"
#include "motor.h"
#include <string.h>

// --- DEFINES Y ENUMS DEL TECLADO ---
// ENUMS PARA LAS MASCARAS DEL TECLADO
typedef enum
{
    // COLUMNAS EN J6-11 A J6-14
    TECLADO_COL_1 = (1 << PIN_18), // P0[18]
    TECLADO_COL_2 = (1 << PIN_17), // P0[17]
    TECLADO_COL_3 = (1 << PIN_15), // P0[15]
    TECLADO_COL_4 = (1 << PIN_16), // P0[16]

    // FILAS EN J6-15 A J6-18
    TECLADO_FILA_1 = (1 << PIN_23), // P0[23]
    TECLADO_FILA_2 = (1 << PIN_24), // P0[24]
    TECLADO_FILA_3 = (1 << PIN_25), // P0[25]
    TECLADO_FILA_4 = (1 << PIN_26)  // P0[26]
} MaskTeclado;

// MASCARAS AGRUPADAS ASHEI
#define TECLADO_FILAS    (TECLADO_FILA_1 | TECLADO_FILA_2 | TECLADO_FILA_3 | TECLADO_FILA_4)
#define TECLADO_COLUMNAS (TECLADO_COL_1 | TECLADO_COL_2 | TECLADO_COL_3 | TECLADO_COL_4)

// ------------------------------------------------------- DEFINES DEL ADC
// --------------------------------------------------------- SUPONEMOS UN PANEL DE 5V (5000 mV)
#define PANEL_mVMAX    5000
#define ADC_RESOLUCION 4095

// ZONA MUERTA DEL SEGUIDOR: solo se mueve si la diferencia entre LDRs supera este umbral
// (evita hunting por ruido del ADC cuando ambos sensores ven luz similar)
#define ZONA_MUERTA_ADC 300

// HABILITA DMA EN EL ADC (BIT 17 DEL ADCR)
#define ADC_CR_DMA_ENABLE ((1UL << 17))

// --------------------------------------------------------- VARIABLES GLOBALES
// ----------------------------------------------------------------------

// VARIABLE GLOBAL PARA LA TECLA DETECTADA
volatile uint8_t tecla_presionada = 0;

// CONTADOR PARA PRINT PERIODICO POR UART (~500ms)
static uint16_t uart_print_tick = 0;

// BUFFER DMA PARA ADC (32 WORDS — CADA UNA CONTIENE UN ADGDR)
static volatile uint32_t adc_dma_buf[32];

// --- MÁQUINA DE ESTADOS DEL TECLADO (MET) ---
typedef enum
{
    MET_REPOSO,
    MET_ANTIRREBOTE,
    MET_PRESIONADO,
    MET_REPETIR
} MetEstado;

#define KEY_DEBOUNCE_TICKS 10  // 30ms de antirrebote
#define KEY_HOLD_TICKS     100 // 300ms antes del primer auto-repeat
#define KEY_REPEAT_TICKS   10  // 30ms entre repeticiones
#define KEY_QUIET_TICKS    5   // 15ms sin tecla antes de apnea

static MetEstado met_estado = MET_REPOSO;
static uint8_t met_tecla = 0;
static uint8_t met_cnt = 0;

// --- ESTADO DEL SISTEMA ---

// DEFINICION DE TECLAS
#define TECLA_MANUAL   1
#define TECLA_SEGUIDOR 2
#define TECLA_FIJO     3
#define TECLA_HOME     4
#define TECLA_IZQ      5
#define TECLA_DER      6
// 7 AL 14 SON ANGULOS FIJOS, DIRECTAMENTE MULTIPLICO EN LA FUNCION
#define TECLA_EMERGENCIA 15
#define TECLA_VELOCIDAD  16

// MODOS DE OPERACION
typedef enum
{
    MODO_MANUAL,
    MODO_SEGUIDOR,
    MODO_FIJO
} ModoMotor;

static ModoMotor modo_actual = MODO_MANUAL;
static uint8_t motor_habilitado = 1;
static uint8_t velocidad_rapida = 1; // 1 = paso cada tick, 0 = paso cada 4 ticks

// DEFINES PARA EL UART

static const char* Modo_Nombre(ModoMotor modo) // CHARS PARA MANDAR EN LOS CAMBIOS DE MODOS
{
    switch (modo)
    {
        case MODO_MANUAL: return "MANUAL";
        case MODO_SEGUIDOR: return "SEGUIDOR";
        case MODO_FIJO: return "FIJO";
        default: return "???";
    }
}

// LOGICA DEL SISTEMA
void ProcesarTecla(uint8_t tecla);
uint16_t CalcularAnguloSeguidor(uint16_t adc_izq, uint16_t adc_der);

// TECLADO
void Teclado_Init(void);
uint8_t Teclado_Scan(void);

// ADC
void ADC_Config(void);
void ADC_DMA_Config(void);
void ADC_Leer_Ambos(uint16_t* izq, uint16_t* der);
uint16_t Calcular_Voltaje_Panel(uint16_t valor_adc);

// UART
void UART_Config(void);
void UART_Imprimir(const char* str);
void UART_ImprimirEstado(void);
void UART_ImprimirEntero(uint16_t valor);
// DMA

void GPDMA_Init();
void ADC_DMA_Restart();
// ==========================================================
//	MAINNNNNNNNNNNN (HASTA AHORA TODOS POR IA PARA TESTEAR, ESTE TIENE UN POCO DE LOS 2 PQ ME RE COSTO EL DEBOUNCE DEL
// TECLADO)
// ==========================================================

int main(void)
{
    SystemCoreClockUpdate();

    // CONFIGURAMOS EL TICK A 3MS
    SYSTICK_InternalInit(3);
    SYSTICK_Cmd(ENABLE);

    // INICIALIZAMOS TODO
    UART_Config();

    UART_Imprimir("HOLA MUNDO :)\r\n");

    Motor_Init();
    Teclado_Init();
    ADC_Config();
    ADC_DMA_Config();

    // --- CONFIGURAMOS TIMER0 PERIÓDICO (3ms) para antirrebote + auto-repeat ---
    TIM_TIMERCFG_T timerCfg = {TIM_US, 1000}; // 1ms por tick
    TIM_InitTimer(LPC_TIM0, &timerCfg);

    TIM_MATCHCFG_T matchCfg;
    matchCfg.channel = TIM_MATCH_0;
    matchCfg.matchValue = 3;   // 3 ticks = 3ms (mismo período que SysTick)
    matchCfg.intEn = ENABLE;   // interrupción al hacer match
    matchCfg.stopEn = DISABLE; // periódico: no se apaga solo
    matchCfg.resetEn = ENABLE; // resetea TC al hacer match
    matchCfg.extOpt = TIM_NOTHING;
    TIM_ConfigMatch(LPC_TIM0, &matchCfg);

    NVIC_EnableIRQ(TIMER0_IRQn); // habilitamos la interrupción del timer
    TIM_Disable(LPC_TIM0);       // arranca apagado, lo prende EINT3

    uint8_t step_divider = 0;

    while (1)
    {
        if (SYSTICK_HasFired() == SET)
        {
            // --- TECLA PRESIONADA (la ISR del TIMER0 ya hizo el debounce) ---
            if (tecla_presionada != 0)
            {
                ProcesarTecla(tecla_presionada);
                tecla_presionada = 0;
            }

            // --- PRINT PERIODICO POR UART (~500ms) ---
            if (++uart_print_tick >= 167)
            {
                uart_print_tick = 0;

                // DEBUG: valores raw del ADC para identificar físicamente qué LDR es cuál
                uint16_t dbg_izq, dbg_der;
                ADC_Leer_Ambos(&dbg_izq, &dbg_der);
                UART_Imprimir("[DBG] ADC_IZQ=");
                UART_ImprimirEntero(dbg_izq);
                UART_Imprimir(" ADC_DER=");
                UART_ImprimirEntero(dbg_der);
                UART_Imprimir("\r\n");

                UART_ImprimirEstado();
            }

            // --- TIMING DEL PASO: rapido (cada tick) o lento (cada 4 ticks) ---
            step_divider++;

            if (motor_habilitado)
            {
                uint8_t debe_avanzar = velocidad_rapida ? 1 : ((step_divider & 3) == 0);

                // MODO SEGUIDOR: control proporcional directo sobre el diff del ADC.
                // No usamos angulo absoluto porque causa wrap-around (0° vs 360°).
                // El motor da un paso por tick hacia donde hay mas luz.
                if (modo_actual == MODO_SEGUIDOR && debe_avanzar)
                {
                    uint16_t adc_izq, adc_der;
                    ADC_Leer_Ambos(&adc_izq, &adc_der);

                    int16_t diff = (int16_t)adc_izq - (int16_t)adc_der;

                    if (diff > ZONA_MUERTA_ADC)
                    {
                        // Mas luz a la izquierda (desde frente) -> girar a la izquierda (antihorario)
                        Motor_MoverPasosRelativo(-1);
                    }
                    else if (diff < -ZONA_MUERTA_ADC)
                    {
                        // Mas luz a la derecha (desde frente) -> girar a la derecha (horario)
                        Motor_MoverPasosRelativo(1);
                    }
                }

                Motor_Update();
            }
        }
    }

    return 0;
}

// ---------------------------------------------------------------------- TECLADO
// ----------------------------------------------------------------------

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
            // DESHABILITO EINT3
            NVIC_DisableIRQ(EINT3_IRQn);

            // LIMPIO BANDERAS
            GPIO_ClearInt(PORT_0, TECLADO_COLUMNAS);
            NVIC_ClearPendingIRQ(EINT3_IRQn);

            // ARRANCO TIMER0 (EL TIMER0 RE-HABILITA EINT3 AL VOLVER A REPOSO)
            TIM_Enable(LPC_TIM0);
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
    GPIO_SetDir(PORT_0, TECLADO_FILAS, GPIO_OUTPUT);   // FILAS (J6-15 A 18) AHORA SON SALIDAS
    GPIO_SetDir(PORT_0, TECLADO_COLUMNAS, GPIO_INPUT); // COLUMNAS (J6-11 A 14) AHORA SON ENTRADAS

    // DEJAMOS TODAS LAS FILAS EN 0 LOGICO EN EL REPOSO
    GPIO_ClearPins(PORT_0, TECLADO_FILAS);

    // LA INTERRUPCION DE FLANCO DESCENDENTE SE PASA A LAS NUEVAS COLUMNAS (P0.15, P0.16, P0.17, P0.18)
    GPIO_IntConfigPort(PORT_0, TECLADO_COLUMNAS, GPIO_INT_FALLING);

    NVIC_ClearPendingIRQ(EINT3_IRQn);
    NVIC_EnableIRQ(EINT3_IRQn);
}

uint8_t Teclado_Scan(void)
{
    static const uint8_t mapa_teclas[4][4] = {{1, 2, 3, 4}, {5, 6, 7, 8}, {9, 10, 11, 12}, {13, 14, 15, 16}};

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
            break;
    }

    // REESTABLEZCO LAS FILAS EN BAJO PARA LA PROXIMA INTERRUPCION
    GPIO_ClearPins(PORT_0, TECLADO_FILAS);
    return tecla_local;
}

void TIMER0_IRQHandler(void)
{
    uint8_t tecla = Teclado_Scan();
    uint8_t detener_timer = 0;

    switch (met_estado)
    {
        case MET_REPOSO:
            if (tecla != 0)
            {
                met_tecla = tecla;
                met_cnt = 0;
                met_estado = MET_ANTIRREBOTE;
            }
            else
            {
                // Varios ticks sin tecla → apagamos TIMER0 y reactivamos EINT3
                if (++met_cnt >= KEY_QUIET_TICKS)
                {
                    detener_timer = 1;
                }
            }
            break;

        case MET_ANTIRREBOTE:
            if (tecla == met_tecla)
            {
                if (++met_cnt >= KEY_DEBOUNCE_TICKS)
                {
                    tecla_presionada = met_tecla; // EVENTO ÚNICO DE PRESIÓN
                    met_cnt = 0;
                    met_estado = MET_PRESIONADO;
                }
            }
            else
            {
                // Rebote o se soltó — reiniciamos
                met_tecla = 0;
                met_cnt = 0;
                met_estado = MET_REPOSO;
            }
            break;

        case MET_PRESIONADO:
            if (tecla != met_tecla)
            {
                met_tecla = 0;
                met_cnt = 0;
                met_estado = MET_REPOSO;
            }
            else if (++met_cnt >= KEY_HOLD_TICKS)
            {
                met_cnt = 0;
                met_estado = MET_REPETIR;
            }
            break;

        case MET_REPETIR:
            if (tecla != met_tecla)
            {
                met_tecla = 0;
                met_cnt = 0;
                met_estado = MET_REPOSO;
            }
            else if (++met_cnt >= KEY_REPEAT_TICKS)
            {
                met_cnt = 0;
                if (met_tecla == TECLA_IZQ || met_tecla == TECLA_DER)
                {
                    tecla_presionada = met_tecla; // AUTO-REPEAT solo navegación
                }
            }
            break;
    }

    if (detener_timer)
    {
        TIM_Disable(LPC_TIM0);
        NVIC_ClearPendingIRQ(EINT3_IRQn);
        NVIC_EnableIRQ(EINT3_IRQn);
    }

    TIM_ClearIntPending(LPC_TIM0, TIM_MR0_INT);
}

// ----------------------------------------------------------------------- ADC
// ----------------------------------------------------------------------

void ADC_Config(void)
{
    ADC_Init(100000);
    ADC_PinConfig(ADC_CHANNEL_4);
    ADC_PinConfig(ADC_CHANNEL_5);

    // HABILITAMOS LOS CANALES EN EL ADCR (SEL bits) — ADC no convierte si SEL=0
    ADC_ChannelEnable(ADC_CHANNEL_4);
    ADC_ChannelEnable(ADC_CHANNEL_5);

    ADC_BurstEnable();

    // BIT 17 DEL ADCR: habilita DMA request en el ADC
    LPC_ADC->ADCR |= ADC_CR_DMA_ENABLE;
}

void ADC_DMA_Config(void)
{
    GPDMA_Init();
    ADC_DMA_Restart();
}

void ADC_DMA_Restart(void)
{
    GPDMA_Channel_CFG_T chCfg;

    chCfg.channelNum = GPDMA_CH_0;
    chCfg.transferSize = 32;
    chCfg.type = GPDMA_P2M;

    chCfg.srcMemAddr = 0;                     // fuente: periférico (ADC)
    chCfg.dstMemAddr = (uint32_t)adc_dma_buf; // destino: buffer en RAM

    chCfg.srcConn = GPDMA_ADC;
    chCfg.dstConn = 0; // destino en memoria, sin conexión periférica

    chCfg.src.width = GPDMA_WORD;    // 32 bits por transferencia
    chCfg.src.burst = GPDMA_BSIZE_1; // 1 transferencia por request
    chCfg.src.increment = DISABLE;   // ADC es un registro fijo

    chCfg.dst.width = GPDMA_WORD;
    chCfg.dst.burst = GPDMA_BSIZE_1;
    chCfg.dst.increment = ENABLE; // avanza en el buffer

    chCfg.intTC = DISABLE; // sin interrupciones
    chCfg.intErr = DISABLE;
    chCfg.linkedList = 0; // sin scatter/gather

    GPDMA_SetupChannel(&chCfg);
    GPDMA_ChannelStart(GPDMA_CH_0);
}

void ADC_Leer_Ambos(uint16_t* izq, uint16_t* der)
{
    static uint16_t last_izq = 0, last_der = 0;
    uint32_t remaining = LPC_GPDMACH0->DMACCControl & 0xFFF;
    uint32_t escritos = 32 - remaining;
    uint32_t sum4 = 0, sum5 = 0;
    uint32_t count4 = 0, count5 = 0;

    if (remaining == 0)
    {
        escritos = 32;
    }

    // RECORREMOS EL BUFFER UNA SOLA VEZ, ACUMULANDO AMBOS CANALES
    for (uint32_t i = 0; i < escritos; i++)
    {
        uint32_t raw = adc_dma_buf[i];
        uint8_t canal = ADC_GDR_CH(raw);
        uint16_t valor = (uint16_t)ADC_GDR_RESULT(raw);

        if (canal == 4)
        {
            sum4 += valor;
            count4++;
        }
        else if (canal == 5)
        {
            sum5 += valor;
            count5++;
        }
    }

    // PROMEDIO POR CANAL, O ÚLTIMO VALOR CONOCIDO SI NO HAY MUESTRAS
    if (count4 > 0)
    {
        last_der = (uint16_t)(sum4 / count4); // CH4 ahora se trata como DERECHA
    }
    if (count5 > 0)
    {
        last_izq = (uint16_t)(sum5 / count5); // CH5 ahora se trata como IZQUIERDA
    }

    *izq = last_izq;
    *der = last_der;

    // REINICIAMOS SOLO SI EL DMA COMPLETÓ EL CICLO COMPLETO
    if (remaining == 0)
    {
        ADC_DMA_Restart();
    }
}

uint16_t Calcular_Voltaje_Panel(uint16_t valor_adc)
{
    // MULTIPLICAMOS PRIMERO, SUMAMOS EL REDONDEO (CUANDO SUMO LA MITAD DEL DENOM. REDONDEO DE LA FORMA NATURAL) Y
    // DIVIDIMOS AL FINAL USAMOS UINT32_T TEMPORALMENTE PORQUE 4095 * 5000 = 20.475.000 (DESBORDA 16 BITS)
    uint32_t calculo = ((uint32_t)valor_adc * PANEL_mVMAX) + (ADC_RESOLUCION / 2);

    return (uint16_t)(calculo / ADC_RESOLUCION);
}

// ---------------------------------------------------------------------- UART DEBUG
// ----------------------------------------------------------------------

void UART_Config(void)
{
    UART_PinConfig(UART_TX0_P0_2);
    UART_PinConfig(UART_RX0_P0_3);

    UART_CFG_T cfg = {
        .baudRate = 115200, .parity = UART_PARITY_NONE, .dataBits = UART_DBITS_8, .stopBits = UART_STOPBIT_1};

    UART_Init(UART0, &cfg);
    UART_TxEnable(UART0);
}

void UART_Imprimir(const char* str)
{
    UART_Send(UART0, (const uint8_t*)str, strlen(str), BLOCKING); // funcion que envia data,
}

void UART_ImprimirEntero(uint16_t valor) // FUNCION PARA PASAR DE NUMERO A STRING ASI SE PUEDE IMPRIMIR
{
    char buf[6];
    uint8_t i = 0;

    if (valor == 0)
    {
        UART_Imprimir("0");
        return;
    }

    while (valor > 0 && i < sizeof(buf) - 1)
    {
        buf[i++] = '0' + (valor % 10);
        valor /= 10;
    }
    buf[i] = '\0';

    // INVIERTO EL STRING
    for (uint8_t j = 0; j < i / 2; j++)
    {
        char t = buf[j];
        buf[j] = buf[i - 1 - j];
        buf[i - 1 - j] = t;
    }

    UART_Imprimir(buf);
}

void UART_ImprimirEstado(void) // ESTA FUNCION SE USA PARA ARMAR LA LINEA Y MANDAR BIEN LA INFO
{
    uint16_t angulo = Motor_GetAngle();
    const char* estado = Motor_IsIdle() ? "DETENIDO" : "MOVIENDO";

    uint16_t adc_izq, adc_der;
    ADC_Leer_Ambos(&adc_izq, &adc_der);
    uint16_t voltaje = Calcular_Voltaje_Panel((adc_izq + adc_der) / 2);

    UART_Imprimir("{\"modo\":\"");
    UART_Imprimir(Modo_Nombre(modo_actual));
    UART_Imprimir("\",\"ang\":");
    UART_ImprimirEntero(angulo);
    UART_Imprimir(",\"est\":\"");
    UART_Imprimir(estado);
    UART_Imprimir("\",\"v\":");
    UART_ImprimirEntero(voltaje);
    UART_Imprimir("}\r\n");
}

// ---------------------------------------------------------------------- LOGICA DEL SISTEMA
// ----------------------------------------------------------------------

uint16_t CalcularAnguloSeguidor(uint16_t adc_izq, uint16_t adc_der)
{
    int16_t diff = (int16_t)adc_izq - (int16_t)adc_der; // -4095 .. +4095
    int32_t angulo = 180 + ((int32_t)diff * 180 / (int32_t)ADC_RESOLUCION);

    if (angulo < 0)
        angulo = 0;
    if (angulo > 360)
        angulo = 360;

    return (uint16_t)angulo;
}

void ProcesarTecla(uint8_t tecla)
{
    switch (tecla)
    {
        case TECLA_MANUAL:
            modo_actual = MODO_MANUAL;
            motor_habilitado = 1;
            Motor_Resume(); // re-energiza por si venia de emergencia
            UART_Imprimir("Modo: ");
            UART_Imprimir(Modo_Nombre(modo_actual));
            UART_Imprimir("\r\n");
            return;

        case TECLA_SEGUIDOR:
            modo_actual = MODO_SEGUIDOR;
            motor_habilitado = 1;
            Motor_Resume();
            UART_Imprimir("Modo: ");
            UART_Imprimir(Modo_Nombre(modo_actual));
            UART_Imprimir("\r\n");
            return;

        case TECLA_FIJO:
            modo_actual = MODO_FIJO;
            motor_habilitado = 1;
            Motor_Resume();
            UART_Imprimir("Modo: ");
            UART_Imprimir(Modo_Nombre(modo_actual));
            UART_Imprimir("\r\n");
            return;

        case TECLA_HOME:
            if (modo_actual == MODO_FIJO)
                return;        // EN FIJO NO HACE NADA
            Motor_MoverA(180); // CENTRO DEL RECORRIDO
            return;

        case TECLA_EMERGENCIA:
            if (motor_habilitado)
            {
                motor_habilitado = 0;
                Motor_Stop();
            }
            else
            {
                motor_habilitado = 1;
                Motor_Resume();
            }
            return;

        case TECLA_VELOCIDAD: velocidad_rapida = !velocidad_rapida; return;
    }

    // SI ESTA EN EMERGENCIA O EN MODO FIJO, IGNORAMOS TODO LO DEMAS
    if (!motor_habilitado)
        return;
    if (modo_actual == MODO_FIJO)
        return;

    // --- FLECHAS DE MANUAL ---
    if (tecla == TECLA_IZQ && modo_actual == MODO_MANUAL)
    {
        Motor_MoverPasosRelativo(-57); // ~5° contrareloj
        return;
    }

    if (tecla == TECLA_DER && modo_actual == MODO_MANUAL)
    {
        Motor_MoverPasosRelativo(57); // ~5° reloj
        return;
    }

    // --- ANGULOS FIJOS (TECLAS 7 A 14) — SOLO EN MANUAL ---
    if (tecla >= 7 && tecla <= 14 && modo_actual == MODO_MANUAL)
    {
        uint16_t angulo = (tecla - 6) * 45; // tecla 7→45°, 8→90°, ..., 14→360°
        Motor_MoverA(angulo);
    }
}
