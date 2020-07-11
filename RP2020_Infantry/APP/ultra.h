#ifndef __ULTRA_H
#define __ULTRA_H

/* Includes ------------------------------------------------------------------*/
#include "sys.h"

/* Global macro --------------------------------------------------------------*/
#define ULTRA_USE_USART		1
#define ULTRA_USE_IIC		(!ULTRA_USE_USART)

/* Global TypeDef ------------------------------------------------------------*/
typedef struct
{
	float 		dis;
	uint16_t 	time;
	bool  		update;
	uint8_t		cnt;
}Ultra_Info_t;

/* ## Global Variables Prototypes ## -----------------------------------------*/
extern Ultra_Info_t Ultra;

/* API functions Prototypes --------------------------------------------------*/
/* #������# ---------------------------------------------------------------------------------------------------------------------------------------*/

uint16_t ULTRA_ReadData( uint8_t *rxBuf);

/* #Ӧ�ò�# ---------------------------------------------------------------------------------------------------------------------------------------*/
void ULTRA_Detect( void );
void ULTRA_IICReadResult( void );
void ULTRA_Config( void );

/* #�����# ---------------------------------------------------------------------------------------------------------------------------------------*/
void ULTRA_Init(void);
void ULTRA_Control(void);

#endif
