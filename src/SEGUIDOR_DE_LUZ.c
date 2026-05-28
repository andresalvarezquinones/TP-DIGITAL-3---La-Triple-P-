#include "LPC17xx.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_gpdma.h"
#include "lpc17xx_uart.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_systick.h"

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


int main(void)
{
    // VARIABLES AUXILIARES PARA EL CONTEO DE TIEMPO DE ESPERA EN HOME
    uint16_t contador_espera_home = 0;
    uint16_t angulo_actual_test = 0;

    // INICIALIZO EL STEP
    Stepper_Init();

    // INICIALIZO EL SYSTICK A 4 MS
    SYSTICK_InternalInit(4);
    SYSTICK_Cmd(ENABLE);

    // --- PRIMERA ETAPA: PERMANECER EN HOME 5 SEGUNDOS ---
    // 5000MS / 4MS POR DISPARO = 1250 CONTEOS DE LA BANDERA DEL SYSTICK
    while (contador_espera_home < 500)
    {
        if (SYSTICK_HasFired() == SET)
        {
            contador_espera_home++;
            SYSTICK_ClearCounterFlag();
        }
    }

    // --- SEGUNDA ETAPA: AVANZAR DE A 90 GRADOS HACIA LA DERECHA HASTA LA VUELTA COMPLETA ---
    // INCREMENTAMOS EL ANGULO DE 90 EN 90 HASTA LLEGAR A LOS 360 GRADOS ABSOLUTOS
    while(1){


    while (angulo_actual_test < 360)
    {
        angulo_actual_test = angulo_actual_test + 90;
        Stepper_MoveToAngle(angulo_actual_test);

        // ADVERTENCIA OPCIONAL: EN CASO DE QUERER VER CADA TRAMO DE 90 GRADOS DE MANERA AISLADA
        // SE PODRIA CLAVAR OTRO PEQUEÑO BUCLE DE ESPERA DEL SYSTICK AQUI
    }

    // --- TERCERA ETAPA: VOLVER TOTALMENTE HACIA LA IZQUIERDA HASTA LA POSICION 0 ---
    // RETORNAMOS EL MOTOR DIRECTAMENTE A SU COORDENADA DE CALIBRACION INICIAL (HOME)
    Stepper_MoveToAngle(0);
	}

    return 0 ;
}


void Stepper_Init()
{
    // CONFIGURO LOS PINES DE CONTROL: P0[9]->J6-5 | P0[8] ->J6-6 | P0[7]->J6-7 | P0[6]->J6-8

    // PRIMERO HAGO PINSEL:
    PINSEL_CFG_T pinConfig; //

    pinConfig.port = PORT_0; //
    pinConfig.func = PINSEL_FUNC_00;
    pinConfig.mode = PINSEL_PULLUP;
    pinConfig.openDrain = DISABLE;

    PINSEL_ConfigMultiplePins(&pinConfig, ALL_MOTOR_PINS); //

    GPIO_SetDir(PORT_0, ALL_MOTOR_PINS, GPIO_OUTPUT); //

    GPIO_ClearPins(PORT_0, ALL_MOTOR_PINS);

    cant_steps = 0;
}

void Stepper_Step(uint8_t direccion)
{
    // VARIABLE SABER CUAL ES EL PASO ELECTRICO DE LA MATRIZ
    static uint8_t step_actual = 0;

    // CALCULAR EL SIGUIENTE ESTADO DE LA SECUENCIA CIRCULAR (0 A 3)
    if (direccion == DIRECCION_RELOJ)
    {
        // AVANZA EN LA SECUENCIA Y SE MANTIENE ENTRE 0 Y 3
        step_actual = (step_actual + 1) % 4;

        // INCREMENTA EL CONTADOR DE POSICION ABSOLUTO DEL SISTEMA
        cant_steps++;
    }
    else
    {
        if (step_actual == 0)
        {
            step_actual = 3; // SI ESTABA EN 0, VUELVE AL PASO MAXIMO
        }
        else
        {
            step_actual = step_actual - 1; // SI NO, RESTA UN PASO NORMALMENTE
        }

        // DECREMENTA EL CONTADOR DE POSICION ABSOLUTO DEL SISTEMA
        cant_steps--;
    }

    // APAGAR TODAS LAS BOBINAS DEL MOTOR ANTES DE APLICAR EL NUEVO PASO PARA NO TENER PROBLEMAS
    GPIO_ClearPins(PORT_0, ALL_MOTOR_PINS); // [cite: 308]

    // PRENDO UNICAMENTE LAS BOBINAS QUE CORRESPONDEN AL PASO ACTUAL.
    GPIO_SetPins(PORT_0, secuencia_stepper[step_actual]);
}

void Stepper_MoveToAngle(uint16_t angulo_destino)
{

    //CALCULO BIEN LA CANT DE PASOS A LA QUE QUIERO LLEGAR, LA FORMULA ES ASI PARA EVITAR FLOATS Y USAR ENTEROS
    uint16_t paso_objetivo = ((angulo_destino * MOTOR_PASOS_NUM) + (MOTOR_PASOS_DEN / 2)) / MOTOR_PASOS_DEN;

    // WHILE PROVISIONAL PARA TESTEAR, POR AHORA BLOQUEA PERO NO IMPORTA, DSP SE ACTUALIZA
    while (cant_steps != paso_objetivo)
    {
        // CHEQUEO SI SALTO O NO EL SYSTYCK (SI SALTO PASARON LOS 4MS DE CALIBRACION PA LAS BOBINA)
        if (SYSTICK_HasFired() == SET) //
        {
            // DETERMINAMOS EL SENTIDO DE GIRO MAS CORTO HACIA EL OBJETIVO
            if (cant_steps < paso_objetivo)
            {
                Stepper_Step(DIRECCION_RELOJ);
            }
            else
            {
                Stepper_Step(DIRECCION_CONTRA);
            }

            // LIMPIO LA BANDERA DEL SYSTICK
            SYSTICK_ClearCounterFlag();
        }
    }
}
