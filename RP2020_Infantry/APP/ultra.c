/**
 * @file        ultra.c
 * @author      MaWeiming
 * @Version     V1.0
 * @date        21-January-2020
 * @brief       This file includes the Ultrasonic(������) external functions 
 * 				(based on ST Peripherals Libaray Keil.STM32F4xx_DFP.2.13.0)
 * @Version		
 */

/**
 *	# ������
 */

/* Includes ------------------------------------------------------------------*/
#include "ultra.h"

#include "usart3.h"
#include "delay.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#define ULTRA_SPEED 0.34f	// mm/us

#define ULTRA_ADDR	0xe8

/* Private variables ---------------------------------------------------------*/
/* ## Global variables ## ----------------------------------------------------*/
Ultra_Info_t Ultra;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
 *	@brief	������������������
 *	@note		���ú�����ֲ��ʵ���ʱ�������ϵ�ᰴ����������
 */
static void ULTRA_SendCmd( uint8_t addr, uint8_t cmd)
{
	USART3_SendChar(addr);	// I2C��ַ
	delay_us(100);			// ��ʱ 20~100us
	USART3_SendChar(0x02);	// �Ĵ���0x02
	delay_us(100);			// ��ʱ 20~100us
	USART3_SendChar(cmd);	// ������������	
}

/* API functions -------------------------------------------------------------*/
/* #������# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	��������ȡ����
 *	@note		�ж��е���
 */
uint16_t ULTRA_ReadData( uint8_t *rxBuf )
{
	uint16_t res;
	
	res = (((uint16_t)rxBuf[0]) << 8) | rxBuf[1];
	
	return res;
}

/**
 *	@brief	������̽������
 */
void ULTRA_Detect( void )
{
	ULTRA_SendCmd( ULTRA_ADDR, 0x1e );
}

/**
 *	@brief	������ģ������
 */
void ULTRA_Config( void )
{
	ULTRA_SendCmd( ULTRA_ADDR, 0x71);	// ��Դ��������(USB����)
	delay_ms(2500);
}

/* #��Ϣ��# ---------------------------------------------------------------------------------------------------------------------------------------*/

/* #Ӧ�ò�# ---------------------------------------------------------------------------------------------------------------------------------------*/

/* #�����# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	�������Զ���λ
 */
void ULTRA_Control( void )
{
	ULTRA_Detect();
}
