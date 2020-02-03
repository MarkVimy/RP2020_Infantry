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
 *	 DR16 - ���� - 2.4GHz D-BUSЭ��
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
 *		@����				ң����ͨ��0		ң����ͨ��1		ң����ͨ��2		ң����ͨ��3
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
 *							�ƶ��ٶ�			�ƶ��ٶ�			�ƶ��ٶ�			
 *							- -> ����			- -> ����			- -> ����
 *							+ -> ����			+ -> ����			+ -> ����
 *
 *		@��					������			����Ҽ�			����				�����ֶ�
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
RC_Ctl_t RC_Ctl_Info = {.rc.isDataValid = 1};	// Ĭ��ң�����ݿɿ�

/* Private function prototypes -----------------------------------------------*/
static void USART2_GPIO_init(void);
static void USART2_ParamsInit(uint32_t baudrate);
static void USART2_DMA_init(void);
static void USART2_init(uint32_t baudrate);
static bool REMOTE_isRCDataValid(RC_Ctl_t *rcInfo);

/* Private functions ---------------------------------------------------------*/
/**
 *	@brief	USART2 GPIO ��ʼ��(ң����-ֻ��)
 *	@note		PA2	- USART2_TX
 *					PA3	- USART2_RX
 */
static void USART2_GPIO_init(void)
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
static void USART2_DMA_init(void)
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
static void USART2_init(uint32_t baudrate)
{
	/* USART2 NVIC Configuration */
	NVICx_init(USART2_IRQn, USART2_IT_PRIO_PRE, USART2_IT_PRIO_SUB);
	
	USART2_GPIO_init();
	USART2_ParamsInit(baudrate);
	USART2_DMA_init();
}

/**
 *	@brief	�ж�ң�������Ƿ����
 */
static bool REMOTE_isRCDataValid(RC_Ctl_t *rcInfo)
{
	if((rcInfo->rc.s1 != RC_SW_UP && rcInfo->rc.s1 != RC_SW_MID && rcInfo->rc.s1 != RC_SW_DOWN)
		|| (rcInfo->rc.s2 != RC_SW_UP  && rcInfo->rc.s2 != RC_SW_MID && rcInfo->rc.s2 != RC_SW_DOWN)
	  || (rcInfo->rc.ch0 > RC_CH_VALUE_MAX || rcInfo->rc.ch0 < RC_CH_VALUE_MIN)
		|| (rcInfo->rc.ch1 > RC_CH_VALUE_MAX || rcInfo->rc.ch1 < RC_CH_VALUE_MIN)
		|| (rcInfo->rc.ch2 > RC_CH_VALUE_MAX || rcInfo->rc.ch2 < RC_CH_VALUE_MIN)
		|| (rcInfo->rc.ch3 > RC_CH_VALUE_MAX || rcInfo->rc.ch3 < RC_CH_VALUE_MIN) 
		|| (rcInfo->rc.thumbwheel  > RC_CH_VALUE_MAX || rcInfo->rc.thumbwheel  < RC_CH_VALUE_MIN)) 
	{
		return false;
	} 
	else 
	{
		return true;
	}
}

/* API functions -------------------------------------------------------------*/
/* #������# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	����ʹ��RCң��
 */
void REMOTE_reenableRC(RC_Ctl_t *remoteInfo)
{
	remoteInfo->rc.isDataValid = 1;
}

/**
 *	@brief	��ң������ǿ�и�λ�ɰ�ȫ״̬
 */
void REMOTE_resetRCData(RC_Ctl_t *remoteInfo)
{
	/* ͨ��ֵǿ�����ó��м�ֵ(������ҡ�˵�״̬) */
	remoteInfo->rc.ch0 = RC_CH_VALUE_OFFSET;
	remoteInfo->rc.ch1 = RC_CH_VALUE_OFFSET;
	remoteInfo->rc.ch2 = RC_CH_VALUE_OFFSET;
	remoteInfo->rc.ch3 = RC_CH_VALUE_OFFSET;	
	
	/* ���ҿ���ѡ��ǿ�����ó��м�ֵ״̬ */
	remoteInfo->rc.s1 = RC_SW_MID;
	remoteInfo->rc.s2 = RC_SW_MID;
	
	/* ��� */
	remoteInfo->mouse.x = 0;
	remoteInfo->mouse.y = 0;
	remoteInfo->mouse.z = 0;
	
	/* ��������Ƿ��� */
	remoteInfo->mouse.press_l = 0;
	remoteInfo->mouse.press_r = 0;
		
	/* ���̼�ֵ */
	remoteInfo->key.v= 0;
	
	/* ���� */
	remoteInfo->rc.thumbwheel = RC_CH_VALUE_OFFSET;
}

/**
 *	@brief	�ж�ң���Ƿ�λ����ֵ
 */
bool REMOTE_isRCChannelReset(RC_Ctl_t *remoteInfo)
{
	if(  remoteInfo->rc.ch0 == RC_CH_VALUE_OFFSET && 
		 remoteInfo->rc.ch1 == RC_CH_VALUE_OFFSET && 
		 remoteInfo->rc.ch2 == RC_CH_VALUE_OFFSET && 
		 remoteInfo->rc.ch3 == RC_CH_VALUE_OFFSET   )
	
	{
		return true;
	}
	return false;	
}

/**
 *	@brief	ң�س�ʼ��
 */
void REMOTE_init(void)
{
	USART2_init(RC_BAUDRATE);
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
				REMOTE_processData(&RC_Ctl_Info, sbus_rx_buffer[0]);
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
				REMOTE_processData(&RC_Ctl_Info, sbus_rx_buffer[1]);
			}
		}
	}
}


/* #Ӧ�ò�# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	������յ���ң������
 */
void REMOTE_processData(RC_Ctl_t *remoteInfo, char *rxBuf)
{
	if(rxBuf == NULL) {
		return;
	} else {
	/* ģ��19��������(�ο�RMң���ֲ�) ��ң�ؿ���ʹ�ò��� */
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
		remoteInfo->rc.ch0 = (  rxBuf[0]       | (rxBuf[1] << 8 ) ) & 0x07ff;
		remoteInfo->rc.ch1 = ( (rxBuf[1] >> 3) | (rxBuf[2] << 5 ) ) & 0x07ff;
		remoteInfo->rc.ch2 = ( (rxBuf[2] >> 6) | (rxBuf[3] << 2 ) | (rxBuf[4] << 10) ) & 0x07ff;
		remoteInfo->rc.ch3 = ( (rxBuf[4] >> 1) | (rxBuf[5] << 7 ) ) & 0x07ff;

		/* Switch left, right */
		remoteInfo->rc.s1  = ( (rxBuf[5] >> 4) & 0x000C ) >> 2;
		remoteInfo->rc.s2  = ( (rxBuf[5] >> 4) & 0x0003 );

		/* Mouse axis: X, Y, Z */
		remoteInfo->mouse.x = rxBuf[6]  | (rxBuf[7 ] << 8);
		remoteInfo->mouse.y = rxBuf[8]  | (rxBuf[9 ] << 8);
		remoteInfo->mouse.z = rxBuf[10] | (rxBuf[11] << 8);

		/* Mouse Left, Right Is Press ? */
		remoteInfo->mouse.press_l = rxBuf[12];
		remoteInfo->mouse.press_r = rxBuf[13];

		/* KeyBoard value */
		remoteInfo->key.v = rxBuf[14] | (rxBuf[15] << 8);	
		
		/* Thumb Wheel */
		remoteInfo->rc.thumbwheel = ((int16_t)rxBuf[16] | ((int16_t)rxBuf[17] << 8)) & 0x07ff;
	}
	
	/* ���ݳ�����ֹ���� */
	if(REMOTE_isRCDataValid(remoteInfo) == false) {
		remoteInfo->rc.isDataValid = 0;
		REMOTE_resetRCData(remoteInfo);	// ǿ�н�ң��ֵ��λ
		Flag.Remote.FLAG_rcErr = 1;
	}
	
	Cnt.Remote.CNT_rcLost = 0;	// ���ʧ������
}

/**
 *	@brief	ң��ʧ������
 */
void REMOTE_rcLostProcess(void)
{
	Cnt.Remote.CNT_rcLost++;
	if(Cnt.Remote.CNT_rcLost > 2) {	// 40msң��ʧ������(����̫С��ң������ ֡/14ms����ҪԤ������ʱ��)
		Cnt.Remote.CNT_rcLost = 2;	 // ��ֹ���
		Flag.Remote.FLAG_rcLost = 1;
	} else {	// ���лָ��ĵ���(ң����������)
		Flag.Remote.FLAG_rcLost = 0;
		if(System_State == SYSTEM_STATE_RCLOST) {	// ϵͳĿǰ����ң��ʧ��״̬
			if(REMOTE_isRCChannelReset(&RC_Ctl_Info)) {	// �ⲿң��ͨ���Ѹ�λ
				Cnt.Remote.CNT_rcLostRecover++;	// �ָ�ȷ�ϼ���
				if(Cnt.Remote.CNT_rcLostRecover > 10) {	// >200ms��Ϊ�����ָ�����(�ɵȴ�ң�ع��вŻָ�)
					Cnt.Remote.CNT_rcLostRecover = 0;
					BM_set(BitMask.System.BM_reset, BM_RESET_GIMBAL);	// ������̨��λ
					System_State = SYSTEM_STATE_NORMAL;
				}
			}
		}
	}	
}

/**
 *	@brief	ң��ʧ������
 */
void REMOTE_rcLostReport(void)
{
	System_State = SYSTEM_STATE_RCLOST;
	REMOTE_resetRCData(&RC_Ctl_Info);	
}

/**
 *	@brief	ң�����ݴ�����
 */
void REMOTE_rcErrReport(void)
{
	System_State = SYSTEM_STATE_RCERR;
	REMOTE_resetRCData(&RC_Ctl_Info);	
}

/**
 *	@brief	ң�س��洦��(ģʽ�л�)
 */
void REMOTE_rcCorrectProcess(void)
{
	static int8_t  prev_sw2 = -1;
	static uint8_t sw2;
	static uint8_t change_pid_mode = false;
		
	sw2 = RC_Ctl_Info.rc.s2;
	
	if(sw2 == RC_SW_UP) 
	{
		/* ң�ػ�еģʽ -> ����ģʽ */
		if(prev_sw2 == RC_SW_MID) 
		{	
			Flag.Remote.FLAG_mode = KEY;
			Flag.Gimbal.FLAG_pidMode = GYRO;
			GIMBAL_rcMech_To_keyGyro();
		}
	} 
	else if(sw2 == RC_SW_MID) 
	{
		Gimbal.State.FLAG_topGyroOpen = false;
		/* ����ģʽ -> ң�ػ�еģʽ */
		if(prev_sw2 == RC_SW_UP) 
		{
			// ���̻�δ����(�ȴ�����)
			if(abs(Gimbal_PID[MECH][YAW_205].Angle.erro) > 10) {
				change_pid_mode = 1;
			} 
			// �����ѻ���(�����л�)
			else {
				change_pid_mode	= 0;
				Flag.Remote.FLAG_mode = RC;
				Flag.Gimbal.FLAG_pidMode = MECH;
				GIMBAL_keyGyro_To_rcMech();
			}
		} 
		/* ң��������ģʽ -> ң�ػ�еģʽ */
		else if(prev_sw2 == RC_SW_DOWN) 
		{	
			// ���̻�δ����(�ȴ�����)
			if(abs(Gimbal_PID[MECH][YAW_205].Angle.erro) > 10) {
				change_pid_mode = 1;
			} 
			// �����ѻ���(�����л�)
			else {	
				change_pid_mode	= 0;
				Flag.Remote.FLAG_mode = RC;
				Flag.Gimbal.FLAG_pidMode = MECH;
				GIMBAL_rcGyro_To_rcMech();
			}
		}
		// �ȴ����̻���
		if(change_pid_mode == 1 && abs(Gimbal_PID[MECH][YAW_205].Angle.erro) <= 10) {
			change_pid_mode = 0;
			Flag.Remote.FLAG_mode = RC;
			Flag.Gimbal.FLAG_pidMode = MECH;
			GIMBAL_keyGyro_To_rcMech();
		}
	} 
	else if(sw2 == RC_SW_DOWN) 
	{
		/* ң�ػ�еģʽ -> ң��������ģʽ */
		if(prev_sw2 == RC_SW_MID) 
		{	
			Flag.Remote.FLAG_mode = RC;
			Flag.Gimbal.FLAG_pidMode = GYRO;
			GIMBAL_rcMech_To_rcGyro();
		}
	}
	
	prev_sw2 = sw2;
}

/* #�����# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	ң�ؿ���
 */
void REMOTE_control(void)
{
	REMOTE_rcLostProcess();	// ʧ���жϼ��ָ�����
	
	if(Flag.Remote.FLAG_rcLost == 1) {	// ң��ʧ��
		REMOTE_rcLostReport();
	} else if(Flag.Remote.FLAG_rcErr == 1) {	// ң�ش���
		REMOTE_rcErrReport();
	} else {	// ң������
//		if(BitMask.System.BM_reset == 0) {
			REMOTE_rcCorrectProcess();
//		}
	}
}
