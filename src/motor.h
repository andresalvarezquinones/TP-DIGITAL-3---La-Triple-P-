#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include <stdbool.h>

// DIRECCION DE GIRO
#define DIRECCION_RELOJ   1
#define DIRECCION_CONTRA  0

// FUNCIONES PUBLICAS

void     Motor_Init        (void);
void     Motor_Paso        (uint8_t direccion);
void     Motor_MoverA      (uint16_t angulo_destino);
void     Motor_MoverPasosRelativo (int16_t delta_pasos);
void     Motor_Stop        (void);
void     Motor_Resume      (void);
uint8_t  Motor_Update      (void);
uint16_t Motor_GetAngle    (void);
uint8_t  Motor_IsIdle      (void);

#endif
