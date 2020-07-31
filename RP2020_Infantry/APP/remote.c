/*
 * @file        remote.c
 * @author      MaWeiming
 * @Version     V1.0
 * @date        24-September-2019
 * @brief       This file includes the DJI remote controller's external functions 
 * 							(based on ST Peripherals Libaray Keil.STM32F4xx_DFP.2.13.0)
 */

/**
 *	# DJI remote (DT7 & DR16 2.4GHz 遥控接收系统)
 *	@brief
 *		DT7 - 工作于2.4GHz频段的无线电通信设备，仅能与DR16接收机配合使用
 *					开阔室外，最大控制范围可达到1000m，最长工作时间可达12小时
 *
 *	规格参数
 *		DT7 - 特性 - 7通道
 *					电池 - 3.7V 2000 mAh 锂电池
 *	 	DR16 - 特性 - 2.4GHz D-BUS协议
 *
 *	@note
 *	1. Robomaster 战车控制协议
 *		DR16接收机输出的信号为标准的DBUS协议数据
 *		遥控器<-建立连接->接收机 => 接收机 每隔14ms通过DBUS发送一帧18字节的数据
 *		增加了额外的键盘鼠标控制信息
 *
 *	2. 18字节控制帧结构
 *		
 *		@域					通道0				通道1					通道2					通道3
 *		@偏移				0					11						22						33
 *		@长度(bit)			11					11						11						11
 *		@符号位				无					无						无						无
 *		@范围		min		364					364						364						364
 *					mid		1024				1024					1024					1024
 *					max 	1684				1684					1684					1684
 *		@功能				遥控器通道0			遥控器通道1				遥控器通道2				遥控器通道3
 *
 *		@域					S1					S2	
 *		@偏移				44					46						
 *		@长度(bit)			11					11						
 *		@符号位				无					无						
 *		@范围		min		1					1						
 *					mid		2					2
 *					max 	3					3
 *		@功能				遥控器发射机		遥控器发射机
 *							S1开关位			S2开关位
 *		@备注				1：上				1：上
 *							3：中				3：中
 *							2：下				2：下
 *								
 *
 *		@域					鼠标X轴				鼠标Y轴				鼠标Z轴	
 *		@偏移				48					64					80
 *		@长度(bit)			16					16					16
 *		@符号位				有					有					有
 *		@范围		min		-32768				-32768				-32768	
 *					mid		0					0					0
 *					max 	32767				32767				32767
 *		@功能				鼠标X方向			鼠标Y方向			鼠标Z方向
 *							移动速度				移动速度				移动速度			
 *							- -> 左移			- -> 左移			- -> 左移
 *							+ -> 右移			+ -> 右移			+ -> 右移
 *
 *		@域					鼠标左键				鼠标右键				按键				保留字段
 *		@偏移				96					104					112					128
 *		@长度(bit)			8					8					16					16
 *		@符号位				无					无					无
 *		@范围		min		0					0					位值标识
 *					max 	1					1							
 *		@功能				0 - 没按下			0 - 没按下			Bit0
 *							1 - 按下			1 - 按下			Bit1
 *																	Bit2
 *																	...
 *																	Bit15
 *
 *	3. 摇杆与通道的配对信息
 *		左摇杆			右摇杆
 *			ch3			   ch1
 *			↑			   ↑
 *		  <-*-> ch2		 <-*-> ch0
 *			↓			   ↓
 */	

/* Includes ------------------------------------------------------------------*/
#include "remote.h"

#include "math.h"
#include "exti.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#define USART2_RX_MAX_LEN		(RC_DBUS_FRAME_LEN + 6u)	// 测试发现必须比RC_DBUS_FRAME_LEN大，否则接收会出错
// 关于DMA双缓冲模式的讲解：http://www.sohu.com/a/260229041_807475

/* Private variables ---------------------------------------------------------*/
static char  sbus_rx_buffer[2][USART2_RX_MAX_LEN];	// 双SBUS接收缓冲区(接收数据)
static uint16_t sbus_rx_len = 0;

/* ## Global variables ## ----------------------------------------------------*/
RC_Ctl_t Remote;

/* Private function prototypes -----------------------------------------------*/
static void USART2_GPIO_Init(void);
static void USART2_ParamsInit(uint32_t baudrate);
static void USART2_DMA_Init(void);
static void USART2_Init(uint32_t baudrate);

/* Private functions ---------------------------------------------------------*/
/* #驱动层# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	USART2 GPIO 初始化(遥控器-只收)
 *	@note		PA2	- USART2_TX
 *					PA3	- USART2_RX
 */
static void USART2_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	// 使能GPIOA时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);// 使能USART2时钟
	
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
 *	@brief	遥控器串口参数配置(串口空闲中断 - 只读)
 *	@param	(u32)baudrate	串口波特率
 *	@note
 *					DBUS 通信参数
 *					波特率 - 100Kbps
 *					单元数据长度 - 8
 *					奇偶校验位 - 偶检验
 *					结束位 - 1
 *					流控制 - 无
 *					
 *					# DBUS信号控制电平符合TTL，却和普通的UART信号相反
 *						需要在MCU端增加三极管取反电路 -> MCU才能正常识别UART信号
 */
static void USART2_ParamsInit(uint32_t baudrate)
{
	USART_InitTypeDef USART_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);	// 使能 USART2 时钟
	
	USART_InitStructure.USART_BaudRate = baudrate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;	// 8位数据位
	USART_InitStructure.USART_StopBits = 1;	// 1位停止位
	USART_InitStructure.USART_Parity = USART_Parity_Even;	// 偶校验
	USART_InitStructure.USART_Mode = USART_Mode_Rx;	// 只读
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 无硬件流控制
	USART_Init(USART2, &USART_InitStructure);		// 初始化串口2	
	USART_Cmd(USART2, ENABLE);	// 使能串口2

	USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);	// 开启 串口2 接收空闲中断
}

/**
 *	@brief	遥控器串口DMA初始化(不定长接收)
 */
static void USART2_DMA_Init(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);	// 使能 DMA1 时钟
	
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
	USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);	// 使能 串口2 DMA 接收
}

/**
 *	@brief	遥控器串口初始化(串口空闲中断+DMA - 只读)
 *	@param	(u32)baudrate	串口波特率
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
 *	@brief	串口2中断服务函数(接收遥控数据)
 */
void USART2_IRQHandler(void)
{
	uint8_t res;

	/* 消除编译器警告 */
	res = res;
	
	if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET) {
		/* 清除空闲中断标志位 */
		res = USART2->SR;
		res = USART2->DR;
		
		if(DMA_GetCurrentMemoryTarget(DMA1_Stream5) == 0) {	// Memory 0
			DMA_Cmd(DMA1_Stream5, DISABLE);
			sbus_rx_len = USART2_RX_MAX_LEN - DMA_GetCurrDataCounter(DMA1_Stream5);
			DMA1_Stream5->NDTR = (uint16_t)USART2_RX_MAX_LEN;	// Relocate the dma memory pointer to the beginning position
			DMA1_Stream5->CR |= (uint32_t)(DMA_SxCR_CT);	// Enable the current selected memory is Memory 1
			DMA_Cmd(DMA1_Stream5, ENABLE);
			/* 数据长度判断 */
			if(sbus_rx_len == RC_DBUS_FRAME_LEN) {
				/* RC 数据处理 */
				REMOTE_ProcessData(&Remote, sbus_rx_buffer[0]);
			}
		}	else {	// Memory 1
			DMA_Cmd(DMA1_Stream5, DISABLE);
			sbus_rx_len = USART2_RX_MAX_LEN - DMA_GetCurrDataCounter(DMA1_Stream5);
			DMA1_Stream5->NDTR = (uint16_t)USART2_RX_MAX_LEN;	// Relocate the dma memory pointer to the beginning position
			DMA1_Stream5->CR &= ~(uint32_t)(DMA_SxCR_CT);	// Enable the current selected memory is Memory 0
			DMA_Cmd(DMA1_Stream5, ENABLE);
			/* 数据长度判断 */
			if(sbus_rx_len == RC_DBUS_FRAME_LEN) {
				/* RC 数据处理 */
				REMOTE_ProcessData(&Remote, sbus_rx_buffer[1]);
			}
		}
	}
}

/**
 *	@brief	遥控初始化
 */
void REMOTE_Init(void)
{
	USART2_Init(RC_BAUDRATE);
}

/**
 *	@brief	将遥控数据强行复位成安全状态
 */
void REMOTE_ResetRcData(RC_Ctl_t *remote)
{
	/* 通道值强行设置成中间值(不拨动摇杆的状态) */
	remote->rc.ch0 = RC_CH_VALUE_OFFSET;
	remote->rc.ch1 = RC_CH_VALUE_OFFSET;
	remote->rc.ch2 = RC_CH_VALUE_OFFSET;
	remote->rc.ch3 = RC_CH_VALUE_OFFSET;	
	
	/* 左右开关选择强行设置成中间值状态 */
	remote->rc.s1 = RC_SW_MID;
	remote->rc.s2 = RC_SW_MID;
	
	/* 鼠标 */
	remote->mouse.x = 0;
	remote->mouse.y = 0;
	remote->mouse.z = 0;
	
	/* 左右鼠标是否点击 */
	remote->mouse.press_l = 0;
	remote->mouse.press_r = 0;
		
	/* 键盘键值 */
	remote->key.v= 0;
	
	/* 左拨轮 */
	remote->rc.thumbwheel = RC_CH_VALUE_OFFSET;
}

/**
 *	@brief	处理接收到的遥控数据
 */
void REMOTE_ProcessData(RC_Ctl_t *remote, char *rxBuf)
{
	if(rxBuf == NULL) {
		return;
	} else {
	/* 模仿19步兵代码(参考RM遥控手册) 新遥控可以使用拨轮(可刷固件升级) */
	/* 字节位数与数据的对应关系：
		 字节    数据
		 第0个   0000 0000    0：ch0        uint16_t  实际有效位数 11 位
		 第1个   1111 1000    1：ch1        uint16_t
		 第2个   2211 1111    2：ch2        uint16_t
		 第3个   2222 2222    3：ch3        uint16_t
		 第4个   3333 3332    
		 第5个   1122 3333    1：s1  2：s2  uint8_t   实际有效位数 2 位
		 第6个   xxxx xxxx    x：mouse.x    int16_t   实际有效位数 16 位
		 第7个   xxxx xxxx    
		 第8个   yyyy yyyy    y：mouse.y    int16_t   实际有效位数 16 位
		 第9个   yyyy yyyy
		 第10个  zzzz zzzz    z：mouse.z    int16_t   实际有效位数 16 位
		 第11个  zzzz zzzz
		 第12个  llll llll    l：mouse.left   uint8_t   实际有效位数 8 位
		 第13个  rrrr rrrr    r：mouse.right  uint8_t   实际有效位数 8 位
		 第14个  kkkk kkkk    k：key          uint16_t  实际有效位数 16 位
		 第15个  kkkk kkkk    k：key
		 第16个  wwww wwww    w：thumbwheel左拨轮		uint16_t	实际有效位数 16位
		 第17个  wwww wwww    w：thumbwheel左拨轮
		 
		 总共18个字节，有效位数 128 = 11*4 + 2*2 + 16*3 + 8*2 + 16*1 + 16*1 
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
	
	/* 数据出错，防止暴走 */
	if(REMOTE_IsRcDataValid(remote) == false) {
		Flag.Remote.RcErr = true;
		REMOTE_ResetRcData(remote);	// 强行将遥控值复位
	}
	
	Cnt.Remote.RcLost = 0;	// 请除失联计数
}

/* API functions -------------------------------------------------------------*/
/* #信息层# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	判断遥控数据是否出错
 */
bool REMOTE_IsRcDataValid(RC_Ctl_t *remote)
{
	if((remote->rc.s1 != RC_SW_UP && remote->rc.s1 != RC_SW_MID && remote->rc.s1 != RC_SW_DOWN)
		|| (remote->rc.s2 != RC_SW_UP  && remote->rc.s2 != RC_SW_MID && remote->rc.s2 != RC_SW_DOWN)
		|| (remote->rc.ch0 > RC_CH_VALUE_MAX || remote->rc.ch0 < RC_CH_VALUE_MIN)
		|| (remote->rc.ch1 > RC_CH_VALUE_MAX || remote->rc.ch1 < RC_CH_VALUE_MIN)
		|| (remote->rc.ch2 > RC_CH_VALUE_MAX || remote->rc.ch2 < RC_CH_VALUE_MIN)
		|| (remote->rc.ch3 > RC_CH_VALUE_MAX || remote->rc.ch3 < RC_CH_VALUE_MIN))
//		如果临时换遥控（没烧最新固件会导致这里出错）
//		|| (remote->rc.thumbwheel  > RC_CH_VALUE_MAX || remote->rc.thumbwheel  < RC_CH_VALUE_MIN)) 
	{
		return false;
	} 
	else 
	{
		return true;
	}
}

/**
 *	@brief	判断遥控是否复位到中值
 */
bool REMOTE_IsRcChannelReset(RC_Ctl_t *remote)
{
	if(  (myDeathZoom(RC_CH_VALUE_OFFSET, 20, remote->rc.ch0) == RC_CH_VALUE_OFFSET) && 
		 (myDeathZoom(RC_CH_VALUE_OFFSET, 20, remote->rc.ch1) == RC_CH_VALUE_OFFSET) && 
		 (myDeathZoom(RC_CH_VALUE_OFFSET, 20, remote->rc.ch2) == RC_CH_VALUE_OFFSET) && 
		 (myDeathZoom(RC_CH_VALUE_OFFSET, 20, remote->rc.ch3) == RC_CH_VALUE_OFFSET)   )	
	{
		return true;
	}
	return false;	
}

/**
 *	@brief	更新遥控信息
 *	@note	防止出错
 */
static int8_t  prev_sw2 = RC_SW_MID;
static uint8_t gyro2mech_pid_mode = false;

static uint8_t MouseLockFlag_R = false;
static uint8_t RcLockFlag_Sw1 = false;
static uint8_t KeyLockFlag_Ctrl = false;

void REMOTE_RcUpdateInfo(System_t *sys, RC_Ctl_t *remote)
{
	if(Flag.Remote.RcLost || Flag.Remote.RcErr)
	{
		prev_sw2 = RC_SW_MID;	
		gyro2mech_pid_mode = false;
		
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

/* #应用层# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	遥控失联处理
 */
void REMOTE_RcLostProcess(System_t *sys)
{
	Cnt.Remote.RcLost++;
	/* 40ms遥控失联容限(不能太小，遥控数据 帧/14ms，需要预留处理时间) */
	if(Cnt.Remote.RcLost > 20) {	
		Cnt.Remote.RcLost = 20;	 // 防止溢出
		Flag.Remote.RcLost = true;
	} 
	/* 具有恢复的弹性(遥控重新连接) */
	else {
		Flag.Remote.RcLost = false;
		
		/* 系统目前处于遥控失联状态 */
		if(sys->State == SYSTEM_STATE_RCLOST) 
		{	
			/* 外部遥控通道已复位 */
			if(REMOTE_IsRcChannelReset(&Remote)) 
			{
				BM_Set(BitMask.System.Reset, BM_RESET_GIMBAL);	// 启动云台复位
				Flag.Gimbal.AngleRecordStart = true;
				
				/* 系统参数复位(sw2归中时的参数) */
				sys->State = SYSTEM_STATE_NORMAL;	// 系统恢复正常
				sys->Action = SYS_ACT_NORMAL;		// 常规模式
				sys->PidMode = MECH;				// 机械模式
				sys->RemoteMode = RC;				// 遥控器模式
			}
		}
	}	
}

/**
 *	@brief	遥控失联报告
 */
void REMOTE_RcLostReport(System_t *sys)
{
	sys->State = SYSTEM_STATE_RCLOST;
	REMOTE_ResetRcData(&Remote);	
}

/**
 *	@brief	遥控数据错误处理
 */
void REMOTE_RcErrReport(System_t *sys)
{
	sys->State = SYSTEM_STATE_RCERR;
	REMOTE_ResetRcData(&Remote);	
}

/**
 *	@brief	根据遥控器切换控制方式
 */
void REMOTE_SysCtrlModeSwitch(System_t *sys, RC_Ctl_t *remote)
{
	uint8_t sw2;
	
	sw2 = RC_SW2_VALUE;
	
	if(sw2 == RC_SW_UP) 
	{
		/* 遥控机械模式 -> 键盘模式 */
		if(prev_sw2 == RC_SW_MID) 
		{	
			gyro2mech_pid_mode = false;
			sys->RemoteMode = KEY;
			sys->PidMode = GYRO;
			// 刚切换过去的时候设置为常规行为
			sys->Action = SYS_ACT_NORMAL;	
		}
	} 
	else if(sw2 == RC_SW_MID) 
	{
		// 遥控控制默认系统为常规行为
		sys->Action = SYS_ACT_NORMAL;
		
		/* 键盘模式 -> 遥控机械模式 */
		if(prev_sw2 == RC_SW_UP) 
		{
			/* 底盘还未回来(等待回来) */
			if( !CHASSIS_IfBackToMiddleAngle() ) {
				gyro2mech_pid_mode = true;
			} 
			/* 底盘已回来(可以切换) */
			else {
				gyro2mech_pid_mode	= false;
				sys->RemoteMode = RC;
				sys->PidMode = MECH;
			}
		} 
		/* 遥控陀螺仪模式 -> 遥控机械模式 */
		else if(prev_sw2 == RC_SW_DOWN) 
		{	
			/* 底盘还未回来(等待回来) */
			if( !CHASSIS_IfBackToMiddleAngle() ) {
				gyro2mech_pid_mode = true;
			} 
			/* 底盘已回来(可以切换) */
			else {	
				gyro2mech_pid_mode	= false;
				sys->RemoteMode = RC;
				sys->PidMode = MECH;
			}
		}
		/* 等待底盘回来 */
		if(gyro2mech_pid_mode && CHASSIS_IfBackToMiddleAngle()) {
			gyro2mech_pid_mode = false;
			sys->RemoteMode = RC;
			sys->PidMode = MECH;
		}
	} 
	else if(sw2 == RC_SW_DOWN) 
	{
		gyro2mech_pid_mode = false;
		/* 遥控控制默认系统为常规行为 */
		sys->Action = SYS_ACT_NORMAL;

		/* 遥控机械模式 -> 遥控陀螺仪模式 */
		if(prev_sw2 == RC_SW_MID) 
		{	
			sys->RemoteMode = RC;
			sys->PidMode = GYRO;
		}
	}
	
	prev_sw2 = sw2;	
}

/**
 *	@brief	根据遥控器切换系统行为
 */
uint8_t test_auto_pid = 0;	// 调试自瞄

void REMOTE_SysActSwitch(System_t *sys, RC_Ctl_t *remote)
{	
	static portTickType KeyCtrlTime_Cur = 0;	
	static portTickType KeyLockTime_Ctrl_F = 0;
	static portTickType KeyLockTime_Ctrl_V = 0;
	
	KeyCtrlTime_Cur = xTaskGetTickCount();
	
	if(test_auto_pid == 0) {
		/* 鼠标右键 */
		if(IF_MOUSE_PRESSED_RIGH) {
			/* 常规 -> 自瞄 */
			if(MouseLockFlag_R == false 
				&& sys->Action == SYS_ACT_NORMAL) {
				sys->Action = SYS_ACT_AUTO;
				sys->PidMode = GYRO;
			}
			MouseLockFlag_R = true;	// 右键锁定
		}
		/* 松开右键 */
		else {
			/* 退出自瞄模式判断(自瞄 -> 常规)*/
			if(sys->Action == SYS_ACT_AUTO) {
				sys->Action = SYS_ACT_NORMAL;
				sys->PidMode = GYRO;
			}
			MouseLockFlag_R = false;// 右键解锁
		}
	} else {
		/* Sw1 归中 */
		if(IF_RC_SW1_MID) {
			/* 常规 -> 自瞄 */
			if(RcLockFlag_Sw1 == false
				&& sys->Action == SYS_ACT_NORMAL) {
				sys->Action = SYS_ACT_AUTO;
				sys->PidMode = GYRO;
			}
			RcLockFlag_Sw1 = true;// Sw1锁定
		} 
		/* Sw1 其他档 */
		else {
			/* 退出自瞄模式判断(自瞄 -> 常规)*/
			if(sys->Action == SYS_ACT_AUTO) {
				sys->Action = SYS_ACT_NORMAL;
				sys->PidMode = GYRO;
			}
			RcLockFlag_Sw1 = false;// Sw1解锁
		}
	}
	
	/* 按下Ctrl键 */
	if(IF_KEY_PRESSED_CTRL) {
		
		/* 强制设置 常规行为+机械模式 */
		if(KeyLockFlag_Ctrl == false) {
			sys->Action = SYS_ACT_NORMAL;
			if( !CHASSIS_IfBackToMiddleAngle() ) {
				gyro2mech_pid_mode = true;
			} else {
				gyro2mech_pid_mode = false;
				sys->PidMode = MECH;
			}
		}
		
		/* 等待底盘回来 */
		if(sys->Action == SYS_ACT_NORMAL
			&& gyro2mech_pid_mode
			&& CHASSIS_IfBackToMiddleAngle()) 
		{
			gyro2mech_pid_mode = false;
			sys->PidMode = MECH;
		}
		
		/* Ctrl+V */
		if(KeyCtrlTime_Cur > KeyLockTime_Ctrl_V) {	
			KeyLockTime_Ctrl_V = KeyCtrlTime_Cur + TIME_STAMP_400MS;
			if(IF_KEY_PRESSED_V) {
				/* 常规 -> 打符 */
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
				/* 常规 -> 打符 */
				if(sys->Action == SYS_ACT_NORMAL) {
					sys->Action = SYS_ACT_BUFF;
					sys->BranchAction = BCH_ACT_BIG_BUFF;
					sys->PidMode = GYRO;
				}
			}
		}
		
		/* Ctrl+R */
		if(IF_KEY_PRESSED_R) {
			/* 常规 -> 对位 */
			if(sys->Action == SYS_ACT_NORMAL) {
				sys->Action = SYS_ACT_PARK;
				sys->PidMode = GYRO;//MECH;
			}
		}
		
		KeyLockFlag_Ctrl = true;	// Ctrl键锁定
	} 
	/* 松开Ctrl键 */
	else {
		/* 常规机械 -> 常规陀螺仪 */
		if(KeyLockFlag_Ctrl == true && sys->Action == SYS_ACT_NORMAL) {
			sys->PidMode = GYRO;
			gyro2mech_pid_mode = false;
		}
		KeyLockFlag_Ctrl = false;	// Ctrl键解锁
	}	
	
	/* 退出打符模式判断(打符 -> 常规) */
	if(sys->Action == SYS_ACT_BUFF) {
		if((IF_KEY_PRESSED_W || IF_KEY_PRESSED_S || IF_KEY_PRESSED_A || IF_KEY_PRESSED_D
			|| IF_KEY_PRESSED_Q || IF_KEY_PRESSED_E || IF_KEY_PRESSED_V)&&(!IF_KEY_PRESSED_CTRL)) 	
		{
			sys->Action = SYS_ACT_NORMAL;
			sys->PidMode = GYRO;			
		}
	}
	
	/* 退出对位模式判断(对位 -> 常规) */
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
 *	@brief	遥控常规处理(模式切换)
 */
void REMOTE_RcCorrectProcess(System_t *sys, RC_Ctl_t *remote)
{
	// 控制方式切换
	REMOTE_SysCtrlModeSwitch(sys, remote);
	// 系统行为切换(键盘模式下允许切换)
	if(sys->RemoteMode == KEY)
		REMOTE_SysActSwitch(sys, remote);
}

/* #任务层# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	遥控控制
 */
void REMOTE_Ctrl(void)
{
	taskENTER_CRITICAL();
	
	/*----信息读入----*/
	REMOTE_RcUpdateInfo(&System, &Remote);
	
	// 失联判断及恢复处理
	REMOTE_RcLostProcess(&System);	
	
	/* 遥控失联 */
	if(Flag.Remote.RcLost == true) {	
		REMOTE_RcLostReport(&System);
	} 
	/* 遥控错误 */
	else if(Flag.Remote.RcErr == true) {	
		REMOTE_RcErrReport(&System);
	} 
	/* 遥控正常 */
	else {	
		/* 云台复位完成后允许切换 */
		if(BM_IfReset(BitMask.System.Reset, BM_RESET_GIMBAL))
			REMOTE_RcCorrectProcess(&System, &Remote);
	}
	
	taskEXIT_CRITICAL();
}
