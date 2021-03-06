#ifndef __USART1_H
#define __USART1_H

/* Includes ------------------------------------------------------------------*/
#include "sys.h"

/* Global macro --------------------------------------------------------------*/

/* Global TypeDef ------------------------------------------------------------*/
/* ## Global Variables Prototypes ## -----------------------------------------*/

/* API functions Prototypes --------------------------------------------------*/
void USART1_Init( void );
void USART1_DMA_SendBuf(uint8_t *buf, uint16_t size);
void USART1_SendChar( uint8_t cData );

#endif
