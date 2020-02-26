#ifndef __SPI1_H
#define __SPI1_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

/* Global Macro --------------------------------------------------------------*/
/* Global TypeDef ------------------------------------------------------------*/
/* ## Global Variables Prototypes ## -----------------------------------------*/
/* API functions Prototypes --------------------------------------------------*/
void SPI_Params_Init(SPI_InitTypeDef* SPI_InitStructure);
void SPI_SetSpeed(SPI_TypeDef* SPIx, uint8_t SPI_BaudRatePrescaler);
uint8_t SPI_Transfer(SPI_TypeDef* SPIx, uint8_t data, int8_t *status);
void SPI1_Init(void);

#endif



