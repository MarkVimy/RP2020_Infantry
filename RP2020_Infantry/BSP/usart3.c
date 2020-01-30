/**
 * @file        usart3.c
 * @author      MaWeiming
 * @Version     V1.0
 * @date        21-January-2020
 * @brief       This file includes the USART3 driver external functions 
 * 						(based on ST Peripherals Libaray Keil.STM32F4xx_DFP.2.13.0)
 * @Version		
 */

/**
 *	# �Ӿ�ͨ��ʹ��USART4��ΪͨѶ�ӿ�
 */

/* UART5_RX  ----> DMA1 Ch4 Stream0 */
/* UART4_RX  ----> DMA1 Ch4 Stream2 */
/* USART3_RX ----> DMA1 Ch4 Stream1 */
/* USART2_RX ----> DMA1 Ch4 Stream5 */

/* Includes ------------------------------------------------------------------*/
#include "usart3.h"

#include "ultra.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* TX */
#define    GPIO_TX                   GPIOB
#define    GPIO_PIN_TX               GPIO_Pin_10
#define    GPIO_PINSOURCE_TX         GPIO_PinSource10
#define    RCC_AHB1PERIPH_GPIO_TX    RCC_AHB1Periph_GPIOB

/* RX */
#define    GPIO_RX                   GPIOB
#define    GPIO_PIN_RX               GPIO_Pin_11
#define    GPIO_PINSOURCE_RX         GPIO_PinSource11
#define    RCC_AHB1PERIPH_GPIO_RX    RCC_AHB1Periph_GPIOB

/* DMA */
#define    DMA1_Stream_RX            DMA1_Stream1

/* Buffer Len */
#define    ULTRA_BUFFER_LEN         2

/* Private variables ---------------------------------------------------------*/
/* ## Global variables ## ----------------------------------------------------*/
uint8_t  Ultra_Buffer[ ULTRA_BUFFER_LEN ] = {0};	//�Ӿ��������������ݴ�������

/* Private function prototypes -----------------------------------------------*/
static void USART3_DMA_init( void );

/* Private functions ---------------------------------------------------------*/
/**
 *	@brief ����4 DMA��ʼ��
 */
static void USART3_DMA_init( void )
{		
	DMA_InitTypeDef xCom3DMAInit;
	
	DMA_DeInit( DMA1_Stream_RX );
	xCom3DMAInit.DMA_Channel = DMA_Channel_4;

	xCom3DMAInit.DMA_DIR = DMA_DIR_PeripheralToMemory;//�������赽�洢��

	xCom3DMAInit.DMA_PeripheralBaseAddr  = (uint32_t)&(USART3->DR);
	xCom3DMAInit.DMA_Memory0BaseAddr     = (uint32_t)Ultra_Buffer;
	xCom3DMAInit.DMA_DIR = DMA_DIR_PeripheralToMemory;
	xCom3DMAInit.DMA_BufferSize = ULTRA_BUFFER_LEN;
	xCom3DMAInit.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	xCom3DMAInit.DMA_MemoryInc = DMA_MemoryInc_Enable;
	xCom3DMAInit.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	xCom3DMAInit.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	xCom3DMAInit.DMA_Mode = DMA_Mode_Circular;
	xCom3DMAInit.DMA_Priority = DMA_Priority_VeryHigh;
	xCom3DMAInit.DMA_FIFOMode = DMA_FIFOMode_Enable;
	xCom3DMAInit.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	xCom3DMAInit.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	xCom3DMAInit.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

	DMA_Init( DMA1_Stream_RX, &xCom3DMAInit );	
	DMA_Cmd( DMA1_Stream_RX, ENABLE);  // stream1
}

/* API functions -------------------------------------------------------------*/
/**
 *	@brief	�Ӿ�ͨ�Ŵ��ڳ�ʼ��(USART3)
 */
void USART3_init( void )
{
	USART_InitTypeDef  xUsartInit;
	GPIO_InitTypeDef   xGpioInit;
	NVIC_InitTypeDef   xNvicInit;

	RCC_AHB1PeriphClockCmd( RCC_AHB1PERIPH_GPIO_TX | RCC_AHB1PERIPH_GPIO_RX, ENABLE );
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_USART3, ENABLE );

	GPIO_PinAFConfig( GPIO_TX, GPIO_PINSOURCE_TX, GPIO_AF_USART3 );
	GPIO_PinAFConfig( GPIO_RX, GPIO_PINSOURCE_RX, GPIO_AF_USART3 ); 

	xGpioInit.GPIO_Pin   = GPIO_PIN_TX;
	xGpioInit.GPIO_Mode  = GPIO_Mode_AF;
	xGpioInit.GPIO_OType = GPIO_OType_PP;
	xGpioInit.GPIO_Speed = GPIO_Speed_100MHz;
	xGpioInit.GPIO_PuPd  = GPIO_PuPd_UP;

	GPIO_Init( GPIO_TX, &xGpioInit );

	xGpioInit.GPIO_Pin = GPIO_PIN_RX;
	GPIO_Init( GPIO_RX, &xGpioInit );

	xUsartInit.USART_BaudRate            = 9600;   
	xUsartInit.USART_WordLength          = USART_WordLength_8b;
	xUsartInit.USART_StopBits            = USART_StopBits_1;
	xUsartInit.USART_Parity              = USART_Parity_No;
	xUsartInit.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;
	xUsartInit.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	
	USART_Init( USART3, &xUsartInit );
	USART_Cmd( USART3, ENABLE );
	
	USART_ITConfig( USART3, USART_IT_IDLE, ENABLE  ); // ע��Ҫ���óɴ��ڿ����ж� 

	USART_DMACmd( USART3, USART_DMAReq_Rx, ENABLE );
	USART_DMACmd( USART3, USART_DMAReq_Tx, ENABLE );
	
	USART3_DMA_init( );	// ��ʼ��uart4��DMA
	
	xNvicInit.NVIC_IRQChannel                    = USART3_IRQn;
	xNvicInit.NVIC_IRQChannelPreemptionPriority  = USART3_IT_PRIO_PRE;
	xNvicInit.NVIC_IRQChannelSubPriority         = USART3_IT_PRIO_SUB;
	xNvicInit.NVIC_IRQChannelCmd                 = ENABLE;
	NVIC_Init( &xNvicInit );
}

/**
 *	@brief	����4�жϺ���
 */
uint32_t js_ultra_time = 0;
float 	 js_ultra_dis = 0;
void USART3_IRQHandler( void )
{
	uint8_t res;
	/* �����ж� */
	if(USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)
	{		
		res = res;	// ��������Ŀղ���
		//����ST�ٷ��ֲ�,��һ��SR��DR�Ĵ���,IDLE�����ٴ�ʹ��,�����һֱ�����ж�,�ͻ�����ڽ����ж�û����
		res = USART3->SR ;
		res = USART3->DR ;
		
		DMA_Cmd(DMA1_Stream_RX, DISABLE);
		
		res = ULTRA_BUFFER_LEN - DMA_GetCurrDataCounter(DMA1_Stream_RX);
		
		js_ultra_time = ULTRA_readData(Ultra_Buffer);		// ��ȡ������̽��ʱ��(us)
		js_ultra_dis = 0.34f * js_ultra_time / 2.f;
		memset(Ultra_Buffer, 0, ULTRA_BUFFER_LEN);	// ����֮����������
		DMA_Cmd(DMA1_Stream_RX, ENABLE);
	}
}


/**
  * @brief  ����һ�η���һ���ֽ�����
  * @param  �Լ�����õ�Ҫ�������е�����
  * @retval void
  * @attention  ������λ����
  */
void USART3_sendChar(uint8_t cData)
{
	while (USART_GetFlagStatus( USART3, USART_FLAG_TC ) == RESET);
	
	USART_SendData( USART3, cData );   
}