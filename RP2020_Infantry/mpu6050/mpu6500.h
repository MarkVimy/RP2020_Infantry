#ifndef __MPU6500_H
#define __MPU6500_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

/* Global Macro --------------------------------------------------------------*/


/* Global TypeDef ------------------------------------------------------------*/
/* ## Global Variables Prototypes ## -----------------------------------------*/
/* API functions Prototypes --------------------------------------------------*/
uint8_t MPU6500_Init(void);
uint8_t MPU6500_WriteReg(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *txBuf);
uint8_t MPU6500_ReadReg(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *rxBuf);

#endif
