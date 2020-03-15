/*
 * @file        remote.c
 * @author      MaWeiming
 * @Version     V1.0
 * @date        24-September-2019
 * @brief       This file includes the DJI remote controller's external functions 
 * 							(based on ST Peripherals Libaray Keil.STM32F4xx_DFP.2.13.0)
 */

/**
 *	# DJI remote (DT7 & DR16 2.4GHz ң�ؽ���ϵͳ)
 *	@brief
 *		DT7 - ������2.4GHzƵ�ε����ߵ�ͨ���豸��������DR16���ջ����ʹ��
 *					�������⣬�����Ʒ�Χ�ɴﵽ1000m�������ʱ��ɴ�12Сʱ
 *
 *	������
 *		DT7 - ���� - 7ͨ��
 *					��� - 3.7V 2000 mAh ﮵��
 *	 	DR16 - ���� - 2.4GHz D-BUSЭ��
 *
 *	@note
 *	1. Robomaster ս������Э��
 *		DR16���ջ�������ź�Ϊ��׼��DBUSЭ������
 *		ң����<-��������->���ջ� => ���ջ� ÿ��14msͨ��DBUS����һ֡18�ֽڵ�����
 *		�����˶���ļ�����������Ϣ
 *
 *	2. 18�ֽڿ���֡�ṹ
 *		
 *		@��					ͨ��0				ͨ��1					ͨ��2					ͨ��3
 *		@ƫ��				0					11						22						33
 *		@����(bit)			11					11						11						11
 *		@����λ				��					��						��						��
 *		@��Χ		min		364					364						364						364
 *					mid		1024				1024					1024					1024
 *					max 	1684				1684					1684					1684
 *		@����				ң����ͨ��0			ң����ͨ��1				ң����ͨ��2				ң����ͨ��3
 *
 *		@��					S1					S2	
 *		@ƫ��				44					46						
 *		@����(bit)			11					11						
 *		@����λ				��					��						
 *		@��Χ		min		1					1						
 *					mid		2					2
 *					max 	3					3
 *		@����				ң���������		ң���������
 *							S1����λ			S2����λ
 *		@��ע				1����				1����
 *							3����				3����
 *							2����				2����
 *								
 *
 *		@��					���X��				���Y��				���Z��	
 *		@ƫ��				48					64					80
 *		@����(bit)			16					16					16
 *		@����λ				��					��					��
 *		@��Χ		min		-32768				-32768				-32768	
 *					mid		0					0					0
 *					max 	32767				32767				32767
 *		@����				���X����			���Y����			���Z����
 *							�ƶ��ٶ�				�ƶ��ٶ�				�ƶ��ٶ�			
 *							- -> ����			- -> ����			- -> ����
 *							+ -> ����			+ -> ����			+ -> ����
 *
 *		@��					������				����Ҽ�				����				�����ֶ�
 *		@ƫ��				96					104					112					128
 *		@����(bit)			8					8					16					16
 *		@����λ				��					��					��
 *		@��Χ		min		0					0					λֵ��ʶ
 *					max 	1					1							
 *		@����				0 - û����			0 - û����			Bit0
 *							1 - ����			1 - ����			Bit1
 *																	Bit2
 *																	...
 *																	Bit15
 *
 *	3. ҡ����ͨ���������Ϣ
 *		��ҡ��			��ҡ��
 *			ch3			   ch1
 *			��			   ��
 *		  <-*-> ch2		 <-*-> ch0
 *			��			   ��
 */	

/* Includes ------------------------------------------------------------------*/
#include "remote.h"

#include "math.h"
#include "exti.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#define USART2_RX_MAX_LEN		(RC_DBUS_FRAME_LEN + 6u)	// ���Է��ֱ����RC_DBUS_FRAME_LEN�󣬷�����ջ����
// ����DMA˫����ģʽ�Ľ��⣺http://www.sohu.com/a/260229041_807475

/* Private variables ---------------------------------------------------------*/
static char  sbus_rx_buffer[2][USART2_RX_MAX_LEN];	// ˫SBUS���ջ�����(��������)
static uint16_t sbus_rx_len = 0;

/* ## Global variables ## ----------------------------------------------------*/
RC_Ctl_t Remote;

/* Private function prototypes -----------------------------------------------*/
static void USART2_GPIO_Init(void);
static void USART2_ParamsInit(uint32_t baudrate);
static void USART2_DMA_Init(void);
static void USART2_Init(uint32_t baudrate);

/* Private functions ---------------------------------------------------------*/
/* #������# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	USART2 GPIO ��ʼ��(ң����-ֻ��)
 *	@note		PA2	- USART2_TX
 *					PA3	- USART2_RX
 */
static void USART2_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	// ʹ��GPIOAʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);// ʹ��USART2ʱ��
	
	/* Read Only */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/**
 *	@brief	ң�������ڲ�������(���ڿ����ж� - ֻ��)
 *	@param	(u32)baudrate	���ڲ�����
 *	@note
 *					DBUS ͨ�Ų���
 *					������ - 100Kbps
 *					��Ԫ���ݳ��� - 8
 *					��żУ��λ - ż����
 *					����λ - 1
 *					������ - ��
 *					
 *					# DBUS�źſ��Ƶ�ƽ����TTL��ȴ����ͨ��UART�ź��෴
 *						��Ҫ��MCU������������ȡ����· -> MCU��������ʶ��UART�ź�
 */
static void USART2_ParamsInit(uint32_t baudrate)
{
	USART_InitTypeDef USART_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);	// ʹ�� USART2 ʱ��
	
	USART_InitStructure.USART_BaudRate = baudrate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;	// 8λ����λ
	USART_InitStructure.USART_StopBits = 1;	// 1λֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_Even;	// żУ��
	USART_InitStructure.USART_Mode = USART_Mode_Rx;	// ֻ��
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // ��Ӳ��������
	USART_Init(USART2, &USART_InitStructure);		// ��ʼ������2	
	USART_Cmd(USART2, ENABLE);	// ʹ�ܴ���2

	USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);	// ���� ����2 ���տ����ж�
}

/**
 *	@brief	ң��������DMA��ʼ��(����������)
 */
static void USART2_DMA_Init(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);	// ʹ�� DMA1 ʱ��
	
	DMA_DeInit(DMA1_Stream5);
	DMA_StructInit(&DMA_InitStructure);	
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART2->DR);
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&sbus_rx_buffer[0][0];
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = USART2_RX_MAX_LEN;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;	

	DMA_DoubleBufferModeConfig(DMA1_Stream5, (uint32_t)&sbus_rx_buffer[1][0], DMA_Memory_0);
	DMA_DoubleBufferModeCmd(DMA1_Stream5, ENABLE);
	DMA_Init(DMA1_Stream5, &DMA_InitStructure);
	
	DMA_Cmd(DMA1_Stream5, ENABLE);
	USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);	// ʹ�� ����2 DMA ����
}

/**
 *	@brief	ң�������ڳ�ʼ��(���ڿ����ж�+DMA - ֻ��)
 *	@param	(u32)baudrate	���ڲ�����
 */
static void USART2_Init(uint32_t baudrate)
{
	/* USART2 NVIC Configuration */
	NVICx_init(USART2_IRQn, USART2_IT_PRIO_PRE, USART2_IT_PRIO_SUB);
	
	USART2_GPIO_Init();
	USART2_ParamsInit(baudrate);
	USART2_DMA_Init();
}

/**
 *	@brief	����2�жϷ�����(����ң������)
 */
void USART2_IRQHandler(void)
{
	uint8_t res;

	/* �������������� */
	res = res;
	
	if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET) {
		/* ��������жϱ�־λ */
		res = USART2->SR;
		res = USART2->DR;
		
		if(DMA_GetCurrentMemoryTarget(DMA1_Stream5) == 0) {	// Memory 0
			DMA_Cmd(DMA1_Stream5, DISABLE);
			sbus_rx_len = USART2_RX_MAX_LEN - DMA_GetCurrDataCounter(DMA1_Stream5);
			DMA1_Stream5->NDTR = (uint16_t)USART2_RX_MAX_LEN;	// Relocate the dma memory pointer to the beginning position
			DMA1_Stream5->CR |= (uint32_t)(DMA_SxCR_CT);	// Enable the current selected memory is Memory 1
			DMA_Cmd(DMA1_Stream5, ENABLE);
			/* ���ݳ����ж� */
			if(sbus_rx_len == RC_DBUS_FRAME_LEN) {
				/* RC ���ݴ��� */
				REMOTE_ProcessData(&Remote, sbus_rx_buffer[0]);
			}
		}	else {	// Memory 1
			DMA_Cmd(DMA1_Stream5, DISABLE);
			sbus_rx_len = USART2_RX_MAX_LEN - DMA_GetCurrDataCounter(DMA1_Stream5);
			DMA1_Stream5->NDTR = (uint16_t)USART2_RX_MAX_LEN;	// Relocate the dma memory pointer to the beginning position
			DMA1_Stream5->CR &= ~(uint32_t)(DMA_SxCR_CT);	// Enable the current selected memory is Memory 0
			DMA_Cmd(DMA1_Stream5, ENABLE);
			/* ���ݳ����ж� */
			if(sbus_rx_len == RC_DBUS_FRAME_LEN) {
				/* RC ���ݴ��� */
				REMOTE_ProcessData(&Remote, sbus_rx_buffer[1]);
			}
		}
	}
}

/**
 *	@brief	ң�س�ʼ��
 */
void REMOTE_Init(void)
{
	USART2_Init(RC_BAUDRATE);
}

/**
 *	@brief	��ң������ǿ�и�λ�ɰ�ȫ״̬
 */
void REMOTE_ResetRcData(RC_Ctl_t *remote)
{
	/* ͨ��ֵǿ�����ó��м�ֵ(������ҡ�˵�״̬) */
	remote->rc.ch0 = RC_CH_VALUE_OFFSET;
	remote->rc.ch1 = RC_CH_VALUE_OFFSET;
	remote->rc.ch2 = RC_CH_VALUE_OFFSET;
	remote->rc.ch3 = RC_CH_VALUE_OFFSET;	
	
	/* ���ҿ���ѡ��ǿ�����ó��м�ֵ״̬ */
	remote->rc.s1 = RC_SW_MID;
	remote->rc.s2 = RC_SW_MID;
	
	/* ��� */
	remote->mouse.x = 0;
	remote->mouse.y = 0;
	remote->mouse.z = 0;
	
	/* ��������Ƿ��� */
	remote->mouse.press_l = 0;
	remote->mouse.press_r = 0;
		
	/* ���̼�ֵ */
	remote->key.v= 0;
	
	/* ���� */
	remote->rc.thumbwheel = RC_CH_VALUE_OFFSET;
}

/**
 *	@brief	������յ���ң������
 */
void REMOTE_ProcessData(RC_Ctl_t *remote, char *rxBuf)
{
	if(rxBuf == NULL) {
		return;
	} else {
	/* ģ��19��������(�ο�RMң���ֲ�) ��ң�ؿ���ʹ�ò���(��ˢ�̼�����) */
	/* �ֽ�λ�������ݵĶ�Ӧ��ϵ��
		 �ֽ�    ����
		 ��0��   0000 0000    0��ch0        uint16_t  ʵ����Чλ�� 11 λ
		 ��1��   1111 1000    1��ch1        uint16_t
		 ��2��   2211 1111    2��ch2        uint16_t
		 ��3��   2222 2222    3��ch3        uint16_t
		 ��4��   3333 3332    
		 ��5��   1122 3333    1��s1  2��s2  uint8_t   ʵ����Чλ�� 2 λ
		 ��6��   xxxx xxxx    x��mouse.x    int16_t   ʵ����Чλ�� 16 λ
		 ��7��   xxxx xxxx    
		 ��8��   yyyy yyyy    y��mouse.y    int16_t   ʵ����Чλ�� 16 λ
		 ��9��   yyyy yyyy
		 ��10��  zzzz zzzz    z��mouse.z    int16_t   ʵ����Чλ�� 16 λ
		 ��11��  zzzz zzzz
		 ��12��  llll llll    l��mouse.left   uint8_t   ʵ����Чλ�� 8 λ
		 ��13��  rrrr rrrr    r��mouse.right  uint8_t   ʵ����Чλ�� 8 λ
		 ��14��  kkkk kkkk    k��key          uint16_t  ʵ����Чλ�� 16 λ
		 ��15��  kkkk kkkk    k��key
		 ��16��  wwww wwww    w��thumbwheel����		uint16_t	ʵ����Чλ�� 16λ
		 ��17��  wwww wwww    w��thumbwheel����
		 
		 �ܹ�18���ֽڣ���Чλ�� 128 = 11*4 + 2*2 + 16*3 + 8*2 + 16*1 + 16*1 
	*/

		/* Channel 0, 1, 2, 3 */
		remote->rc.ch0 = (  rxBuf[0]       | (rxBuf[1] << 8 ) ) & 0x07ff;
		remote->rc.ch1 = ( (rxBuf[1] >> 3) | (rxBuf[2] << 5 ) ) & 0x07ff;
		remote->rc.ch2 = ( (rxBuf[2] >> 6) | (rxBuf[3] << 2 ) | (rxBuf[4] << 10) ) & 0x07ff;
		remote->rc.ch3 = ( (rxBuf[4] >> 1) | (rxBuf[5] << 7 ) ) & 0x07ff;

		/* Switch left, right */
		remote->rc.s1  = ( (rxBuf[5] >> 4) & 0x000C ) >> 2;
		remote->rc.s2  = ( (rxBuf[5] >> 4) & 0x0003 );

		/* Mouse axis: X, Y, Z */
		remote->mouse.x = rxBuf[6]  | (rxBuf[7 ] << 8);
		remote->mouse.y = rxBuf[8]  | (rxBuf[9 ] << 8);
		remote->mouse.z = rxBuf[10] | (rxBuf[11] << 8);

		/* Mouse Left, Right Is Press ? */
		remote->mouse.press_l = rxBuf[12];
		remote->mouse.press_r = rxBuf[13];

		/* KeyBoard value */
		remote->key.v = rxBuf[14] | (rxBuf[15] << 8);	
		
		/* Thumb Wheel */
		remote->rc.thumbwheel = ((int16_t)rxBuf[16] | ((int16_t)rxBuf[17] << 8)) & 0x07ff;
	}
	
	/* ���ݳ�����ֹ���� */
	if(REMOTE_IsRcDataValid(remote) == false) {
		Flag.Remote.RcErr = true;
		REMOTE_ResetRcData(remote);	// ǿ�н�ң��ֵ��λ
	}
	
	Cnt.Remote.RcLost = 0;	// ���ʧ������
}

/* API functions -------------------------------------------------------------*/
/* #��Ϣ��# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	�ж�ң�������Ƿ����
 */
bool REMOTE_IsRcDataValid(RC_Ctl_t *remote)
{
	if((remote->rc.s1 != RC_SW_UP && remote->rc.s1 != RC_SW_MID && remote->rc.s1 != RC_SW_DOWN)
		|| (remote->rc.s2 != RC_SW_UP  && remote->rc.s2 != RC_SW_MID && remote->rc.s2 != RC_SW_DOWN)
		|| (remote->rc.ch0 > RC_CH_VALUE_MAX || remote->rc.ch0 < RC_CH_VALUE_MIN)
		|| (remote->rc.ch1 > RC_CH_VALUE_MAX || remote->rc.ch1 < RC_CH_VALUE_MIN)
		|| (remote->rc.ch2 > RC_CH_VALUE_MAX || remote->rc.ch2 < RC_CH_VALUE_MIN)
		|| (remote->rc.ch3 > RC_CH_VALUE_MAX || remote->rc.ch3 < RC_CH_VALUE_MIN) 
		|| (remote->rc.thumbwheel  > RC_CH_VALUE_MAX || remote->rc.thumbwheel  < RC_CH_VALUE_MIN)) 
	{
		return false;
	} 
	else 
	{
		return true;
	}
}

/**
 *	@brief	�ж�ң���Ƿ�λ����ֵ
 */
bool REMOTE_IsRcChannelReset(RC_Ctl_t *remote)
{
	if(  (myDeathZoom(RC_CH_VALUE_OFFSET, 10, remote->rc.ch0) == 0) && 
		 (myDeathZoom(RC_CH_VALUE_OFFSET, 10, remote->rc.ch1) == 0) && 
		 (myDeathZoom(RC_CH_VALUE_OFFSET, 10, remote->rc.ch2) == 0) && 
		 (myDeathZoom(RC_CH_VALUE_OFFSET, 10, remote->rc.ch3) == 0)   )	
	{
		return true;
	}
	return false;	
}

/**
 *	@brief	����ң����Ϣ
 *	@note	��ֹ����
 */
static int8_t  prev_sw2 = RC_SW_MID;
static uint8_t change_pid_mode = false;

static uint8_t MouseLockFlag_R = false;
static uint8_t RcLockFlag_Sw1 = false;
static uint8_t KeyLockFlag_Ctrl = false;

void REMOTE_RcUpdateInfo(System_t *sys, RC_Ctl_t *remote)
{
	if(Flag.Remote.RcLost || Flag.Remote.RcErr)
	{
		prev_sw2 = RC_SW_MID;	
		change_pid_mode = false;
		
		MouseLockFlag_R = false;
		RcLockFlag_Sw1 = false;
		KeyLockFlag_Ctrl = false;
	}
	else
	{
		if(sys->RemoteMode == RC) {
			MouseLockFlag_R = false;
			RcLockFlag_Sw1 = false;
			KeyLockFlag_Ctrl = false;			
		}
	}
}

/* #Ӧ�ò�# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	ң��ʧ������
 */
void REMOTE_RcLostProcess(System_t *sys)
{
	Cnt.Remote.RcLost++;
	/* 40msң��ʧ������(����̫С��ң������ ֡/14ms����ҪԤ������ʱ��) */
	if(Cnt.Remote.RcLost > 20) {	
		Cnt.Remote.RcLost = 20;	 // ��ֹ���
		Flag.Remote.RcLost = true;
	} 
	/* ���лָ��ĵ���(ң����������) */
	else {
		Flag.Remote.RcLost = false;
		
		/* ϵͳĿǰ����ң��ʧ��״̬ */
		if(sys->State == SYSTEM_STATE_RCLOST) 
		{	
			/* �ⲿң��ͨ���Ѹ�λ */
			if(REMOTE_IsRcChannelReset(&Remote)) 
			{
				BM_Set(BitMask.System.BM_Reset, BM_RESET_GIMBAL);	// ������̨��λ
				Flag.Gimbal.AngleRecordStart = true;
				
				/* ϵͳ������λ(sw2����ʱ�Ĳ���) */
				sys->State = SYSTEM_STATE_NORMAL;	// ϵͳ�ָ�����
				sys->Action = SYS_ACT_NORMAL;		// ����ģʽ
				sys->PidMode = MECH;				// ��еģʽ
				sys->RemoteMode = RC;				// ң����ģʽ
			}
		}
	}	
}

/**
 *	@brief	ң��ʧ������
 */
void REMOTE_RcLostReport(System_t *sys)
{
	sys->State = SYSTEM_STATE_RCLOST;
	REMOTE_ResetRcData(&Remote);	
}

/**
 *	@brief	ң�����ݴ�����
 */
void REMOTE_RcErrReport(System_t *sys)
{
	sys->State = SYSTEM_STATE_RCERR;
	REMOTE_ResetRcData(&Remote);	
}

/**
 *	@brief	����ң�����л����Ʒ�ʽ
 */
void REMOTE_SysCtrlModeSwitch(System_t *sys, RC_Ctl_t *remote)
{
	uint8_t sw2;
	
	sw2 = RC_SW2_VALUE;
	
	if(sw2 == RC_SW_UP) 
	{
		/* ң�ػ�еģʽ -> ����ģʽ */
		if(prev_sw2 == RC_SW_MID) 
		{	
			sys->RemoteMode = KEY;
			sys->PidMode = GYRO;
			// ���л���ȥ��ʱ������Ϊ������Ϊ
			sys->Action = SYS_ACT_NORMAL;	
		}
	} 
	else if(sw2 == RC_SW_MID) 
	{
		// ң�ؿ���Ĭ��ϵͳΪ������Ϊ
		sys->Action = SYS_ACT_NORMAL;
		
		//Gimbal.State.FLAG_topGyroOpen = false;
		/* ����ģʽ -> ң�ػ�еģʽ */
		if(prev_sw2 == RC_SW_UP) 
		{
			/* ���̻�δ����(�ȴ�����) */
			if(abs(Gimbal_PID[MECH][YAW_205].Angle.erro) > 10) {
				change_pid_mode = true;
			} 
			/* �����ѻ���(�����л�) */
			else {
				change_pid_mode	= false;
				sys->RemoteMode = RC;
				sys->PidMode = MECH;
			}
		} 
		/* ң��������ģʽ -> ң�ػ�еģʽ */
		else if(prev_sw2 == RC_SW_DOWN) 
		{	
			/* ���̻�δ����(�ȴ�����) */
			if(abs(Gimbal_PID[MECH][YAW_205].Angle.erro) > 10) {
				change_pid_mode = true;
			} 
			/* �����ѻ���(�����л�) */
			else {	
				change_pid_mode	= false;
				sys->RemoteMode = RC;
				sys->PidMode = MECH;
			}
		}
		/* �ȴ����̻��� */
		if(change_pid_mode == true && abs(Gimbal_PID[MECH][YAW_205].Angle.erro) <= 10) {
			change_pid_mode = false;
			sys->RemoteMode = RC;
			sys->PidMode = MECH;
		}
	} 
	else if(sw2 == RC_SW_DOWN) 
	{
		/* ң�ؿ���Ĭ��ϵͳΪ������Ϊ */
		sys->Action = SYS_ACT_NORMAL;

		/* ң�ػ�еģʽ -> ң��������ģʽ */
		if(prev_sw2 == RC_SW_MID) 
		{	
			sys->RemoteMode = RC;
			sys->PidMode = GYRO;
		}
	}
	
	prev_sw2 = sw2;	
}

/**
 *	@brief	����ң�����л�ϵͳ��Ϊ
 */
uint8_t test_auto_pid = 0;	// ��������

void REMOTE_SysActSwitch(System_t *sys, RC_Ctl_t *remote)
{	
	static portTickType KeyCtrlTime_Cur = 0;	
	static portTickType KeyLockTime_Ctrl_F = 0;
	static portTickType KeyLockTime_Ctrl_V = 0;
	
	KeyCtrlTime_Cur = xTaskGetTickCount();
	
	if(test_auto_pid == 0) {
		/* ����Ҽ� */
		if(IF_MOUSE_PRESSED_RIGH) {
			/* ���� -> ���� */
			if(MouseLockFlag_R == false 
				&& sys->Action == SYS_ACT_NORMAL) {
				sys->Action = SYS_ACT_AUTO;
				sys->PidMode = GYRO;
			}
			MouseLockFlag_R = true;	// �Ҽ�����
		}
		/* �ɿ��Ҽ� */
		else {
			/* ���� -> ���� */
			if(sys->Action == SYS_ACT_AUTO) {
				sys->Action = SYS_ACT_NORMAL;
				sys->PidMode = GYRO;
			}
			MouseLockFlag_R = false;// �Ҽ�����
		}
	} else {
		/* Sw1 ���� */
		if(IF_RC_SW1_MID) {
			/* ���� -> ���� */
			if(RcLockFlag_Sw1 == false
				&& sys->Action == SYS_ACT_NORMAL) {
				sys->Action = SYS_ACT_AUTO;
				sys->PidMode = GYRO;
			}
			RcLockFlag_Sw1 = true;// Sw1����
		} 
		/* Sw1 ������ */
		else {
			/* ���� -> ���� */
			if(sys->Action == SYS_ACT_AUTO) {
				sys->Action = SYS_ACT_NORMAL;
				sys->PidMode = GYRO;
			}
			RcLockFlag_Sw1 = false;// Sw1����
		}
	}
	
	/* ����Ctrl�� */
	if(IF_KEY_PRESSED_CTRL) {
		
		/* ǿ������ ������Ϊ+��еģʽ */
		if(KeyLockFlag_Ctrl == false) {
			sys->Action = SYS_ACT_NORMAL;
			if(abs(Gimbal_PID[MECH][YAW_205].Angle.erro) > 10) {
				change_pid_mode = true;
			} else {
				change_pid_mode = false;
				sys->PidMode = MECH;
			}
		}
		
		/* �ȴ����̻��� */
		if(sys->Action == SYS_ACT_NORMAL
			&& change_pid_mode == true
			&& abs(Gimbal_PID[MECH][YAW_205].Angle.erro) < 10) 
		{
			change_pid_mode = false;
			sys->PidMode = MECH;
		}
		
		/* Ctrl+V */
		if(KeyCtrlTime_Cur > KeyLockTime_Ctrl_V) {	
			KeyLockTime_Ctrl_V = KeyCtrlTime_Cur + TIME_STAMP_400MS;
			if(IF_KEY_PRESSED_V) {
				/* ���� -> ��� */
				if(sys->Action == SYS_ACT_NORMAL) {
					sys->Action = SYS_ACT_BUFF;
					sys->BranchAction = BCH_ACT_SMALL_BUFF;
					sys->PidMode = GYRO;
				}
			}
		}
		
		/* Ctrl+F */
		if(KeyCtrlTime_Cur > KeyLockTime_Ctrl_F) {
			KeyLockTime_Ctrl_F = KeyCtrlTime_Cur + TIME_STAMP_400MS;
			if(IF_KEY_PRESSED_F) {
				/* ���� -> ��� */
				if(sys->Action == SYS_ACT_NORMAL) {
					sys->Action = SYS_ACT_BUFF;
					sys->BranchAction = BCH_ACT_BIG_BUFF;
					sys->PidMode = GYRO;
				}
			}
		}
		
		/* Ctrl+R */
		if(IF_KEY_PRESSED_R) {
			/* ���� -> ��λ */
			if(sys->Action == SYS_ACT_NORMAL) {
				sys->Action = SYS_ACT_PARK;
				sys->PidMode = MECH;
			}
		}
		
		KeyLockFlag_Ctrl = true;	// Ctrl������
	} 
	/* �ɿ�Ctrl�� */
	else {
		/* �����е -> ���������� */
		if(KeyLockFlag_Ctrl == true && sys->Action == SYS_ACT_NORMAL) {
			sys->PidMode = GYRO;
			change_pid_mode = false;
		}
		KeyLockFlag_Ctrl = false;	// Ctrl������
	}	
	
	/* �˳����ģʽ�ж�(��� -> ����) */
	if(sys->Action == SYS_ACT_BUFF) {
		if((IF_KEY_PRESSED_W || IF_KEY_PRESSED_S || IF_KEY_PRESSED_A || IF_KEY_PRESSED_D
			|| IF_KEY_PRESSED_Q || IF_KEY_PRESSED_E || IF_KEY_PRESSED_V)&&(!IF_KEY_PRESSED_CTRL)) 	
		{
			sys->Action = SYS_ACT_NORMAL;
			sys->PidMode = GYRO;			
		}
	}
	
	/* �˳���λģʽ�ж�(��λ -> ����) */
	if(sys->Action == SYS_ACT_PARK) {
		if((IF_KEY_PRESSED_W || IF_KEY_PRESSED_S || IF_KEY_PRESSED_A || IF_KEY_PRESSED_D
			|| IF_KEY_PRESSED_Q || IF_KEY_PRESSED_E || IF_KEY_PRESSED_V)&&(!IF_KEY_PRESSED_CTRL)) 	
		{
			sys->Action = SYS_ACT_NORMAL;
			sys->PidMode = GYRO;			
		}
	}	
}

/**
 *	@brief	ң�س��洦��(ģʽ�л�)
 */
void REMOTE_RcCorrectProcess(System_t *sys, RC_Ctl_t *remote)
{
	// ���Ʒ�ʽ�л�
	REMOTE_SysCtrlModeSwitch(sys, remote);
	// ϵͳ��Ϊ�л�(����ģʽ�������л�)
	if(sys->RemoteMode == KEY)
		REMOTE_SysActSwitch(sys, remote);
}

/* #�����# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	ң�ؿ���
 */
void REMOTE_Ctrl(void)
{
	taskENTER_CRITICAL();
	
	/*----��Ϣ����----*/
	REMOTE_RcUpdateInfo(&System, &Remote);
	
	// ʧ���жϼ��ָ�����
	REMOTE_RcLostProcess(&System);	
	
	/* ң��ʧ�� */
	if(Flag.Remote.RcLost == 1) {	
		REMOTE_RcLostReport(&System);
	} 
	/* ң�ش��� */
	else if(Flag.Remote.RcErr == 1) {	
		REMOTE_RcErrReport(&System);
	} 
	/* ң������ */
	else {	
		/* ��̨��λ��ɺ������л� */
		if(BM_IfReset(BitMask.System.BM_Reset, BM_RESET_GIMBAL))
			REMOTE_RcCorrectProcess(&System, &Remote);
	}
	
	taskEXIT_CRITICAL();
}
