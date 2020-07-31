#ifndef __ANOC_H
#define __ANOC_H

/* Includes ------------------------------------------------------------------*/
#include "sys.h"

/* Global macro --------------------------------------------------------------*/
/* Global TypeDef ------------------------------------------------------------*/
/* ## Global Variables Prototypes ## -----------------------------------------*/
/* API functions Prototypes --------------------------------------------------*/
void ANOC_SendToPc1(int16_t rol, int16_t pit, int16_t yaw);
void ANOC_SendToPc(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz, int16_t mx, int16_t my, int16_t mz);
void RP_SendToPc(float yaw, float pitch, float roll, int16_t rateYaw, int16_t ratePitch, int16_t rateRoll);
void RP_SendToPc2(uint8_t shoot_freq, uint16_t shoot_ping, uint16_t shoot_heat, uint16_t shoot_pwm, float shoot_speed);

#endif
