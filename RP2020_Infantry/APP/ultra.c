/**
 * @file        ultra.c
 * @author      MaWeiming
 * @Version     V1.0
 * @date        21-January-2020
 * @brief       This file includes the Ultrasonic(超声波) external functions 
 * 				(based on ST Peripherals Libaray Keil.STM32F4xx_DFP.2.13.0)
 * @Version		
 */

/**
 *	# 超声波
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
 *	@brief	超声波发送配置命令
 *	@note		配置后根据手册适当延时，重新上电会按新配置运行
 */
static void ULTRA_SendCmd( uint8_t addr, uint8_t cmd)
{
	USART3_SendChar(addr);	// I2C地址
	delay_us(100);			// 延时 20~100us
	USART3_SendChar(0x02);	// 寄存器0x02
	delay_us(100);			// 延时 20~100us
	USART3_SendChar(cmd);	// 发送配置命令	
}

/* API functions -------------------------------------------------------------*/
/* #驱动层# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	超声波读取数据
 *	@note		中断中调用
 */
uint16_t ULTRA_ReadData( uint8_t *rxBuf )
{
	uint16_t res;
	
	res = (((uint16_t)rxBuf[0]) << 8) | rxBuf[1];
	
	return res;
}

/**
 *	@brief	超声波探测命令
 */
void ULTRA_Detect( void )
{
	ULTRA_SendCmd( ULTRA_ADDR, 0x1e );
}

/**
 *	@brief	超声波模块配置
 */
void ULTRA_Config( void )
{
	ULTRA_SendCmd( ULTRA_ADDR, 0x71);	// 电源二级降噪(USB供电)
	delay_ms(2500);
}

/* #信息层# ---------------------------------------------------------------------------------------------------------------------------------------*/

/* #应用层# ---------------------------------------------------------------------------------------------------------------------------------------*/

/* #任务层# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	超声波自动对位
 */
void ULTRA_Control( void )
{
	ULTRA_Detect();
}
