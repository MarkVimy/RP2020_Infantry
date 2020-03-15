/**
 * @file        spi1.c
 * @author      MaWeiming
 * @Version     V1.0
 * @date        24-February-2020
 * @brief       This file includes the SPI1 driver external functions 
 * 				(based on ST Peripherals Libaray Keil.STM32F4xx_DFP.2.9.0)
 */

/**
 *	# SPI1
 */

/* Includes ------------------------------------------------------------------*/
#include "spi1.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/**
 *	@var	SPI_DefaultParams
 *	@brief	����:			ȫ˫��
 *			����ģʽ:		����
 *			���ݴ�С:		8λ
 *			ʱ��ģʽ:		ģʽ3(CPOL_1, CPHA_2)
 *			Ƭѡģʽ:		���Ƭѡ
 *			�����ʷ�Ƶ��:	128 fpclk(APB2) = 84MHz => �����ٶ� = 84M/128 = 656.25KHz
 *			��λ����:		MSB
 *			CRC����ʽ:		7
 */
static SPI_InitTypeDef SPI_DefaultParams = {
	.SPI_Direction = SPI_Direction_2Lines_FullDuplex,
	.SPI_Mode = SPI_Mode_Master,
	.SPI_DataSize = SPI_DataSize_8b,
	.SPI_CPOL = SPI_CPOL_High,
	.SPI_CPHA = SPI_CPHA_2Edge,
	.SPI_NSS = SPI_NSS_Soft,
	.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128,
	.SPI_FirstBit = SPI_FirstBit_MSB,
	.SPI_CRCPolynomial = 7,
};

/* ## Global variables ## ----------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void SPI1_GPIO_Init(void);

/* Private functions ---------------------------------------------------------*/
/**
 *	@brief	SPI1 GPIO ��ʼ��
 *	@note	PA15 - SPI1_NSS
 *			PB3 - SPI1_SCK
 *			PB4 - SPI1_MISO
 *			PB5 - SPI1_MOSI
 */
static void SPI1_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	// https://www.cnblogs.com/pingwen/p/11041151.html
	
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	/* ��ʼ������ */
	GPIO_SetBits(GPIOB, GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5);	
	
	/* ���� */
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_SPI1);
}

/* API functions -------------------------------------------------------------*/
/**
 *	@brief	SPI Ĭ�ϲ�����ʼ��
 */
void SPI_Params_Init(SPI_InitTypeDef* SPI_InitStructure)
{
	SPI_InitStructure->SPI_Direction = SPI_DefaultParams.SPI_Direction;
	SPI_InitStructure->SPI_Mode = SPI_DefaultParams.SPI_Mode;
	SPI_InitStructure->SPI_DataSize = SPI_DefaultParams.SPI_DataSize;
	SPI_InitStructure->SPI_CPOL = SPI_DefaultParams.SPI_CPOL;
	SPI_InitStructure->SPI_CPHA = SPI_DefaultParams.SPI_CPHA;
	SPI_InitStructure->SPI_NSS = SPI_DefaultParams.SPI_NSS;
	SPI_InitStructure->SPI_BaudRatePrescaler = SPI_DefaultParams.SPI_BaudRatePrescaler;
	SPI_InitStructure->SPI_FirstBit = SPI_DefaultParams.SPI_FirstBit;
	SPI_InitStructure->SPI_CRCPolynomial = SPI_DefaultParams.SPI_CRCPolynomial;
}

/**
 *	@brief	SPIx �ٶ�����
 */
void SPI_SetSpeed(SPI_TypeDef* SPIx, uint8_t SPI_BaudRatePrescaler)
{
	assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler));
	SPI_Cmd(SPIx, DISABLE);
	SPIx->CR1 &= 0xFFC7;
	SPIx->CR1 |= SPI_BaudRatePrescaler;
	SPI_Cmd(SPIx, ENABLE);
}

/**
 *	@brief	SPIx ��д�ֽں���
 *	@param	int8_t *status	0 	- ����
 *							���� - �������
 *					
 */
uint8_t SPI_Transfer(SPI_TypeDef* SPIx, uint8_t data, int8_t *status)
{
	uint8_t retry = 0;
	/* �ȴ����ͻ�����Ϊ�� */
	while(SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET) {
		retry++;
		if(retry > 200) {
			*status  = 1;
			return 0;
		}
	}
	
	SPI_I2S_SendData(SPIx, data);
	retry = 0;
	
	/* �ȴ����ջ������ǿ� */
	while(SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET) {
		retry++;
		if(retry > 200)
			*status = 2;
			return 0;
	}
	
	*status = 0;
	return SPI_I2S_ReceiveData(SPIx);	
}

/**
 *	@brief	SPI1 ��ʼ��(����MPU6500)
 */
void SPI1_Init(void)
{	    
	int8_t res;
 	SPI_InitTypeDef SPI_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	
	SPI1_GPIO_Init();
	SPI_Params_Init(&SPI_InitStructure);
	SPI_Init(SPI1, &SPI_InitStructure);
	
	SPI_Cmd(SPI1, ENABLE);
	
	SPI_Transfer(SPI1, 0xff, &res);	// ��������
}
