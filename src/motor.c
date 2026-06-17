#include "motor.h"
#include "LPC17xx.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_pinsel.h"

// --------------------------------------------------------------- PINES DEL MOTOR
// ----------------------------------------------------------------------

#define MOTOR_IN1 (1 << PIN_6) // P0[6]
#define MOTOR_IN2 (1 << PIN_7) // P0[7]
#define MOTOR_IN3 (1 << PIN_8) // P0[8]
#define MOTOR_IN4 (1 << PIN_9) // P0[9]

#define MOTOR_PINES (MOTOR_IN1 | MOTOR_IN2 | MOTOR_IN3 | MOTOR_IN4)

// PASOS PARA UNA VUELTA COMPLETA (MEDIO PASO = 4096 PASOS/360°)
#define MOTOR_PASOS_360 4096

// --------------------------------------------------------------- ESTADO INTERNO
// ----------------------------------------------------------------------

// SECUENCIA DE MEDIO PASO (8 PASOS)
static const uint32_t secuencia_motor[8] = {(MOTOR_IN1),
                                            (MOTOR_IN4 | MOTOR_IN1),
                                            (MOTOR_IN4),
                                            (MOTOR_IN3 | MOTOR_IN4),
                                            (MOTOR_IN3),
                                            (MOTOR_IN2 | MOTOR_IN3),
                                            (MOTOR_IN2),
                                            (MOTOR_IN1 | MOTOR_IN2)};

static uint16_t cant_steps = 0;    // POSICION ACTUAL (HOME = 0)
static uint16_t paso_objetivo = 0; // POSICION DESEADA
static uint8_t step_actual = 0;    // INDICE EN secuencia_motor[]

// --------------------------------------------------------------- FUNCIONES PUBLICAS
// ----------------------------------------------------------------------

void Motor_Init(void)
{
    PINSEL_CFG_T pinConfig;

    pinConfig.port = PORT_0;
    pinConfig.func = PINSEL_FUNC_00;
    pinConfig.mode = PINSEL_PULLUP;
    pinConfig.openDrain = DISABLE;

    PINSEL_ConfigMultiplePins(&pinConfig, MOTOR_PINES);

    GPIO_SetDir(PORT_0, MOTOR_PINES, GPIO_OUTPUT);
    GPIO_ClearPins(PORT_0, MOTOR_PINES);

    // PRENDO LA PRIMERA SECUENCIA PARA ENERGIZAR BOBINAS
    GPIO_SetPins(PORT_0, secuencia_motor[0]);

    // HOME ES EL CENTRO DEL RECORRIDO (180° = 2048 PASOS) PARA PODER GIRAR A AMBOS LADOS
    cant_steps = 2048;
    paso_objetivo = 2048;
}

void Motor_Paso(uint8_t direccion)
{
    if (direccion == DIRECCION_RELOJ)
    {
        if (cant_steps < MOTOR_PASOS_360)
        {
            step_actual = (step_actual + 1) % 8;
            cant_steps++;
        }
    }
    else
    {
        if (cant_steps > 0)
        {
            if (step_actual == 0)
                step_actual = 7;
            else
                step_actual = step_actual - 1;

            cant_steps--;
        }
    }

    // CAMBIO DIFERENCIAL: SOLO CAMBIO LOS PINES QUE NECESITO
    uint32_t estado_actual = LPC_GPIO0->FIOPIN;
    uint32_t nuevo_estado = secuencia_motor[step_actual];

    LPC_GPIO0->FIOCLR = (estado_actual & ~nuevo_estado) & MOTOR_PINES;
    LPC_GPIO0->FIOSET = (nuevo_estado & ~estado_actual) & MOTOR_PINES;
}

void Motor_MoverA(uint16_t angulo_destino)
{
    if (angulo_destino > 360)
        angulo_destino = 360;

    uint32_t calculo_base = (uint32_t)angulo_destino * MOTOR_PASOS_360;
    uint16_t nuevo_objetivo = (uint16_t)((calculo_base + 180) / 360);

    if (cant_steps == nuevo_objetivo)
        return;

    paso_objetivo = nuevo_objetivo;
}

void Motor_MoverPasosRelativo(int16_t delta_pasos)
{
    int32_t nuevo_objetivo = (int32_t)cant_steps + (int32_t)delta_pasos;

    if (nuevo_objetivo > (int32_t)MOTOR_PASOS_360)
        nuevo_objetivo = (int32_t)MOTOR_PASOS_360;
    if (nuevo_objetivo < 0)
        nuevo_objetivo = 0;

    if ((uint16_t)nuevo_objetivo == cant_steps)
        return;

    paso_objetivo = (uint16_t)nuevo_objetivo;
}

void Motor_Resume(void)
{
    LPC_GPIO0->FIOSET = secuencia_motor[step_actual] & MOTOR_PINES;
}

void Motor_Stop(void)
{
    LPC_GPIO0->FIOCLR = MOTOR_PINES;
}

uint8_t Motor_Update(void)
{
    if (cant_steps == paso_objetivo)
        return 0;

    if (cant_steps < paso_objetivo)
        Motor_Paso(DIRECCION_RELOJ);
    else
        Motor_Paso(DIRECCION_CONTRA);

    return 1;
}

uint16_t Motor_GetAngle(void)
{
    return (uint16_t)(((uint32_t)cant_steps * 360 + (MOTOR_PASOS_360 / 2)) / MOTOR_PASOS_360);
}

uint8_t Motor_IsIdle(void)
{
    return (cant_steps == paso_objetivo);
}
