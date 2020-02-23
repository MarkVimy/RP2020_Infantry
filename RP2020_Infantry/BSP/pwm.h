#ifndef __PWM_H__
#define __PWM_H__

#include "sys.h"

#define PWM1  	TIM3->CCR1
#define PWM2  	TIM3->CCR2
#define SERVO		TIM1->CCR2

void TIM3_Init(void);
void TIM1_Init(void);
void FRICTION_PwmOut(int16_t pwm1, int16_t pwm2);
void MAGZINE_PwmOut(int16_t pwm);

#endif

