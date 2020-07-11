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

#include "usart1.h"
#include "delay.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/
#define SCL_GPIO		GPIOA
#define SCL_GPIO_PIN	GPIO_Pin_10
#define SCL_GPIO_CLK	RCC_AHB1Periph_GPIOA

#define SDA_GPIO		GPIOA
#define SDA_GPIO_PIN	GPIO_Pin_9
#define SDA_GPIO_CLK	RCC_AHB1Periph_GPIOA

#define ULTRA_SCL		PAout(10)
#define ULTRA_SDA		PAout(9)
#define ULTRA_READ_SDA	PAin(9)

#define ULTRA_SPEED 	0.34f	// mm/us

#define ULTRA_ADDR		0xe8

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
	USART1_SendChar(addr);	// I2C地址
	delay_us(100);			// 延时 20~100us
	USART1_SendChar(0x02);	// 寄存器0x02
	delay_us(100);			// 延时 20~100us
	USART1_SendChar(cmd);	// 发送配置命令	
}

static void IIC_Init(void)
{
	GPIO_InitTypeDef xGpioInit;
	
	RCC_AHB1PeriphClockCmd(SCL_GPIO_CLK | SDA_GPIO_CLK, ENABLE);
	
	xGpioInit.GPIO_Pin = SCL_GPIO_PIN;
	xGpioInit.GPIO_Mode = GPIO_Mode_OUT;
	xGpioInit.GPIO_OType = GPIO_OType_OD;	// 开漏输出
	xGpioInit.GPIO_Speed = GPIO_Speed_100MHz;
	xGpioInit.GPIO_PuPd = GPIO_PuPd_UP;
	
	GPIO_Init(SCL_GPIO, &xGpioInit);
	
	xGpioInit.GPIO_Pin = SDA_GPIO_PIN;
	GPIO_Init(SDA_GPIO, &xGpioInit);
	
	ULTRA_SCL = 1;
	ULTRA_SDA = 1;
}

static void IIC_Start(void)
{
	ULTRA_SCL = 1;
	ULTRA_SDA = 1;
	delay_us(4);
	ULTRA_SDA = 0;	// SCL:H SDA:H->L => #Start
	delay_us(4);
	ULTRA_SCL = 0;
}

static void IIC_Stop(void)
{
	ULTRA_SCL = 0;
	ULTRA_SDA = 0;
	delay_us(4);
	ULTRA_SCL = 1;
	ULTRA_SDA = 1;	// SCL:H SDA:L->H => #Stop
	delay_us(4);
}

/**
 *	@return 0 - 接受应答成功
 *			1 - 接受应答失败
 */
static uint8_t IIC_WaitAck(void)
{
	uint8_t ucErrTime = 0;
	ULTRA_SDA = 1; delay_us(1);
	ULTRA_SCL = 1; delay_us(1);
	
	while(ULTRA_READ_SDA) {
		ucErrTime++;
		if(ucErrTime > 250) {
			IIC_Stop();
			return 1;
		}
	}
	
	ULTRA_SCL = 0;
	return 0;
}

/**
 *	@brief	SCL在SDA一直为低电平时完成高低电平转换
 */
static void IIC_Ack(void)
{
	ULTRA_SCL = 0;
	ULTRA_SDA = 0;
	delay_us(2);
	ULTRA_SCL = 1;
	delay_us(2);
	ULTRA_SCL = 0;
}

/**
 *	@brief	SCL在SDA一直为高电平时完成高低电平转换
 */
static void IIC_NAck(void)
{
	ULTRA_SCL = 0;
	ULTRA_SDA = 1;
	delay_us(2);
	ULTRA_SCL = 1;
	delay_us(2);
	ULTRA_SCL = 0;
}

static void IIC_WriteByte(uint8_t txByte)
{
	uint8_t i;
	
	ULTRA_SCL = 0;	// 拉低时钟开始数据传输
	
	for(i = 0; i < 8; i++) {
		ULTRA_SDA = (txByte & 0x80)>>7;
		txByte <<= 1;
		delay_us(2);
		ULTRA_SCL = 1;
		delay_us(2);
		ULTRA_SCL = 0;
		delay_us(2);
	}
}

/**
 *	@brief	
 *	@param[in]	ack - 1 => 发送ACK
 *				ack - 0 => 发送NACK
 */
static uint8_t IIC_ReadByte(uint8_t ack)
{
	uint8_t i;
	uint8_t rxByte = 0;
	
	for(i = 0; i < 8; i++) {
		ULTRA_SCL = 0;
		delay_us(2);
		ULTRA_SCL = 1;
		rxByte <<= 1;
		if(ULTRA_READ_SDA)
			rxByte++;
		delay_us(1);
	}
	
	if(!ack)
		IIC_NAck();
	else
		IIC_Ack();
	
	return rxByte;
}

/**
 *	@brief	发送探测指令
 */
static uint8_t ULTRA_IICWriteByte(uint8_t addr, uint8_t reg, uint8_t cmd)
{
	IIC_Start();
	
	IIC_WriteByte(addr);	// 写命令
	IIC_WaitAck();
	
	IIC_WriteByte(reg);
	IIC_WaitAck();
	
	IIC_WriteByte(cmd);
	IIC_WaitAck();
	
	IIC_Stop();
}

/**
 *	@brief	读任意寄存器指令格式
 */
static uint8_t ULTRA_IICReadByte(uint8_t addr, uint8_t reg)
{
	uint8_t rxByte;
	
	IIC_Start();
	IIC_WriteByte(addr);	// 写命令
	IIC_WaitAck();
	IIC_WriteByte(reg);		// 读寄存器2
	IIC_WaitAck();
	
	IIC_Start();
	IIC_WriteByte(addr+1);	// 读命令
	IIC_WaitAck();
	rxByte = IIC_ReadByte(0);	// NACK
	IIC_Stop();
	
	return rxByte;
}

/* API functions -------------------------------------------------------------*/
/* #驱动层# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	超声波读取数据
 *	@note	中断中调用
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
	#if (ULTRA_USE_USART)
	ULTRA_SendCmd(ULTRA_ADDR, 0xb0);	//0xb2
	#else
	ULTRA_IICWriteByte(ULTRA_ADDR, 0x02, 0x0a);
	
	//delay_ms(1);	// 安全延时，由任务延时提供
	
	while( !ULTRA_SCL );
	Ultra.time = ULTRA_IICReadByte( ULTRA_ADDR, 0x02);
	Ultra.time <<= 8;
	Ultra.time += ULTRA_IICReadByte( ULTRA_ADDR, 0x03);
	Ultra.dis = 0.34f * Ultra.time / 2.f;
	#endif
}

/**
 *	@brief	超声波反馈距离结果
 */
void ULTRA_IICReadResult( void )
{
	Ultra.time = ULTRA_IICReadByte(ULTRA_ADDR, 0x02);
	Ultra.time <<= 8;
	Ultra.time += ULTRA_IICReadByte(ULTRA_ADDR, 0x03);
	Ultra.dis = 0.34f * Ultra.time / 2.f;	
}

/**
 *	@brief	超声波模块配置
 */
void ULTRA_Config( void )
{
	ULTRA_SendCmd(ULTRA_ADDR, 0x71);	// 电源二级降噪(USB供电)
	delay_ms(2500);
}

/* #信息层# ---------------------------------------------------------------------------------------------------------------------------------------*/

/* #应用层# ---------------------------------------------------------------------------------------------------------------------------------------*/

/* #任务层# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	超声波初始化
 */
void ULTRA_Init(void)
{
	#if (ULTRA_USE_USART)
	USART1_Init();
	#else
	IIC_Init();
	#endif
}

/**
 *	@brief	超声波自动对位
 */
void ULTRA_Control( void )
{
	ULTRA_Detect();
}
