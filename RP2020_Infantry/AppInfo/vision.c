/**
 * @file        vision.c
 * @author      MaWeiming
 * @Version     V1.0
 * @date        27-October-2019
 * @brief       This file includes the VISION Communication Protocols(视觉通信协议) external functions 
 * 				(based on ST Peripherals Libaray Keil.STM32F4xx_DFP.2.13.0)
 * @Version		
 */

/**
 *	# 与视觉的通信使用USART4作为通讯接口
 *	通信协议的格式与裁判系统协议的格式类似
 *	通信协议格式:
 *				帧头		数据段			帧尾
 *	域		frame_header 	data  			frame_tailer(CRC16,整包校验)
 *	字节		3			x				2
 *	
 *	帧头格式:
 *		  起始字节(0xA5)	数据命令长度		帧头CRC8校验
 *	域			SOF 	 	cmd_id    		CRC8
 *	字节		0			1				1
 */

/* Includes ------------------------------------------------------------------*/
#include "vision.h"

#include "uart4.h"
#include "crc.h"
#include "can.h"
#include "kalman.h"

/* Private typedef -----------------------------------------------------------*/
/**
 *	电控->视觉
 */
typedef enum {
	CMD_AIM_OFF 		= 0x00,	// 不启动自瞄(实际是视觉那边跑程序但不发送数据)
	CMD_AIM_AUTO		= 0x01,	// 启动自瞄
	CMD_AIM_SMALL_BUFF	= 0x02,	// 识别小符
	CMD_AIM_BIG_BUFF	= 0x03,	// 识别大符
	CMD_AIM_SENTRY		= 0x04,	// 击打哨兵
	CMD_AIM_BASE		= 0x05	// 吊射基地
} Vision_Cmd_Id_t;

/* 帧头字节偏移 */
typedef enum {
	SOF			= 0,
	CMD_ID		= 1,
	CRC8		= 2,
	DATA		= 3,
	TX_CRC16	= 20,
} Vision_Frame_Header_Offset_t;

/* 数据长度 */
typedef enum {
	/* Std */
	LEN_FRAME_HEADER 		= 3,	// 帧头长度
	LEN_RX_DATA 			= 18,	// 接收数据段长度
	LEN_TX_DATA 			= 17,	// 发送数据段长度
	LEN_FRAME_TAILER 		= 2,	// 帧尾CRC16
	LEN_VISION_RX_PACKET	= 23,	// 接收包整包长度
	LEN_VISION_TX_PACKET	= 22,	// 发送包整包长度
} Vision_Data_Length_t;

/* 帧头格式 */
typedef __packed struct
{
	uint8_t  			sof;		// 同步头
	Vision_Cmd_Id_t  	cmd_id;		// 命令码
	uint8_t  			crc8;		// CRC8校验码
} Vision_Frame_Header_t;

/* 帧尾格式 */
typedef __packed struct 
{
	uint16_t crc16;					// CRC16校验码
} Vision_Frame_Tailer_t;

/* 接收数据段格式 */
typedef __packed struct 
{
	float 	pitch_angle;	// pitch偏差角度/像素点	单位：角度/像素点
	float 	yaw_angle;		// yaw偏差角度/像素点	单位：角度/像素点
	float 	distance;			// 距离				单位：mm
	uint8_t buff_change_armor_four;	// 是否切换到第四块装甲板		单位：0/1
	uint8_t identify_target;// 是否识别到目标	单位：0/1
	uint8_t identify_buff;	// 是否识别到Buff	单位：0/1
	uint8_t identify_too_close;	// 目标距离过近	单位：0/1
	uint8_t anti_gyro;	// 是否识别到小陀螺	单位：0/1
	uint8_t	anti_gyro_change_armor;	// 是否在反陀螺状态下切换装甲板	单位：0/1
} Vision_Rx_Data_t;

/* 发送数据段格式 */
typedef __packed struct
{
	uint8_t buff_shoot_four;// 打符的时候射出第四颗子弹
	uint8_t fric_speed;		// 射速档位(根据等级来分)
	uint8_t my_color;		// 我自己的颜色
} Vision_Tx_Data_t;

/* 接收包格式 */
typedef __packed struct 
{
	Vision_Frame_Header_t FrameHeader;	// 帧头
	Vision_Rx_Data_t	  RxData;		// 数据
	Vision_Frame_Tailer_t FrameTailer;	// 帧尾	
} Vision_Rx_Packet_t;

/* 发送包格式 */
typedef __packed struct
{
	Vision_Frame_Header_t FrameHeader;	// 帧头
	Vision_Tx_Data_t	  TxData;		// 数据
	Vision_Frame_Tailer_t FrameTailer;	// 帧尾		
} Vision_Tx_Packet_t;

/* 辅助标识变量 */
typedef struct
{
	Color_t 		my_color;			// 机器人自己的颜色
	Vision_Mode_t	mode;				// 视觉模式
	uint16_t		unconnected_cnt;	// 视觉失联计数
	uint8_t  		rx_data_valid;		// 接收数据的正确性
	uint16_t 		rx_err_cnt;			// 接收数据的错误统计
	uint32_t		rx_cnt;				// 接收数据包的统计
	uint8_t 		rx_data_update;		// 接收数据是否更新
	uint32_t 		rx_time_prev;		// 接收数据的前一时刻
	uint32_t 		rx_time_now;		// 接收数据的当前时刻
	uint16_t 		rx_time_ping;		// 帧率
} Vision_State_t;

/* 视觉通信数据包格式 */
typedef struct {
	Vision_Rx_Packet_t RxPacket;
	Vision_Tx_Packet_t TxPacket;
	Vision_State_t     State;
} Vision_Info_t;

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#define VISION_TX_BUFFER_LEN	50
#define VISION_FRAME_HEADER		(0xA5)

/*-------视觉分辨率预编译--------*/
/* #目前视觉无法切换分辨率，目前只能在调试的时候手动切换 */
#define	VISION_960P_BUFF	0	// 960(H)*1024(V)	主要用于打符的时候利用像素点
#define	VISION_1280P_AUTO	1	// 1280(H)*720(V)

#define VISION_DPI		VISION_1280P_AUTO
#if VISION_DPI == VISION_960P_BUFF
	#define VISION_MID_YAW		480		// 960
	#define VISION_MID_PITCH	512		// 1024
#elif VISION_DPI == VISION_1280P_AUTO
	#define	VISION_MID_YAW		640		// 1280
	#define	VISION_MID_PITCH	360		// 720
#endif


/* Private variables ---------------------------------------------------------*/
QueueObj VisionQueue =
{
	.nowLength = 0,
	.queueLength = 20,	// 最大支持20
	.queue = {0}
};

QueueObj VisionQueueAccel =
{
	.nowLength = 0,
	.queueLength = 20,	// 最大支持20
	.queue = {0}
};

float visionYawMathExpect;
//数学期望
float visionYawStdDevia;
//标准差


/* ## Global variables ## ----------------------------------------------------*/
uint8_t Vision_Tx_Buffer[VISION_TX_BUFFER_LEN];

Vision_Info_t Vision = {
	.TxPacket.TxData.buff_shoot_four = 0,
	.TxPacket.TxData.fric_speed = 0,
	.TxPacket.TxData.my_color = RED,
	.State.my_color = RED,
};

extKalman_t Gimbal_Gyro_Auto_kalmanError[2];

/* Private function prototypes -----------------------------------------------*/


/* Private functions ---------------------------------------------------------*/
/* API functions -------------------------------------------------------------*/
/* #驱动层# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	读取视觉通信数据
 *	@return false - 数据错误
 *			true  - 数据正确	
 *	@note	uart4.c中IRQ调用
 */
bool VISION_ReadData(uint8_t *rxBuf)
{
	uint8_t res = false;
	Vision.State.rx_cnt++;
	Vision.State.unconnected_cnt = 0;
	/* 帧首字节是否为0xA5 */
	if(rxBuf[SOF] == VISION_FRAME_HEADER) 
	{	
		res = Verify_CRC8_Check_Sum( rxBuf, LEN_FRAME_HEADER );
		/* 帧头CRC8校验*/
		if(res == true)
		{
			res = Verify_CRC16_Check_Sum( rxBuf, LEN_VISION_RX_PACKET );
			/* 帧尾CRC16校验 */
			if(res == true) 
			{
				/* 数据正确则拷贝接收包 */
				memcpy(&Vision.RxPacket, rxBuf, LEN_VISION_RX_PACKET);
				Vision.State.rx_data_update = true;	// 视觉数据更新
				
				/* 帧率计算 */
				Vision.State.rx_time_now = xTaskGetTickCount();
				Vision.State.rx_time_ping = Vision.State.rx_time_now - Vision.State.rx_time_prev;
				Vision.State.rx_time_prev = Vision.State.rx_time_now;
				
			}
		}
	}
	
	/* 数据有效性判断 */
	if(res == true) {
		Vision.State.rx_data_valid = true;
	} else if(res == false) {
		Vision.State.rx_data_valid = false;
		Vision.State.rx_err_cnt++;
	}
	
	return res;
}

/**
 *	@brief	发送视觉通信数据
 *	@note	uart4发送
 */
void VISION_SendData(Vision_Tx_Packet_t *txPacket, Vision_Cmd_Id_t cmd_id)
{
	uint8_t i;
	/* 帧头格式 */
	txPacket->FrameHeader.sof = VISION_FRAME_HEADER;
	txPacket->FrameHeader.cmd_id = cmd_id;
	/* 写入帧头 */
	memcpy( Vision_Tx_Buffer, &txPacket->FrameHeader, LEN_FRAME_HEADER );
	/* 写入帧头CRC8 */
	Append_CRC8_Check_Sum( Vision_Tx_Buffer, LEN_FRAME_HEADER);
	Vision.TxPacket.FrameHeader.crc8 = Vision_Tx_Buffer[CRC8];
	
	/* 数据段填充 */
	memcpy( &Vision_Tx_Buffer[DATA], &txPacket->TxData, LEN_TX_DATA );
	/* 写入帧尾CRC16 */
	Append_CRC16_Check_Sum( Vision_Tx_Buffer, LEN_VISION_TX_PACKET);
	Vision.TxPacket.FrameTailer.crc16 = Vision_Tx_Buffer[TX_CRC16];
	
	/* 将打包后的数据包发送给视觉PC */
	for(i = 0; i < LEN_VISION_TX_PACKET; i++) {
		UART4_SendChar(Vision_Tx_Buffer[i]);
	}
	
	/* 发送数据包清零 */
	memset(Vision_Tx_Buffer, 0, VISION_TX_BUFFER_LEN);
}


/* #信息层# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	反馈视觉标志位
 */
bool VISION_GetFlagStatus(Vision_Flag_t flag)
{
	switch(flag)
	{
		case VISION_FLAG_DATA_VALID:
			return Vision.State.rx_data_valid;
		case VISION_FLAG_DATA_UPDATE:
			return Vision.State.rx_data_update;
		case VISION_FLAG_LOCK_TARGET:
			return Vision.RxPacket.RxData.identify_target;
		case VISION_FLAG_LOCK_BUFF:
			return Vision.RxPacket.RxData.identify_buff;
		case VISION_FLAG_CHANGE_ARMOR:
			return (Vision.RxPacket.RxData.identify_buff == 2)?1:0;
		case VISION_FLAG_CHANGE_ARMOR_4:
			return Vision.RxPacket.RxData.buff_change_armor_four;
		default:
			break;
	}
	return false;
}

/**
 *	@brief	设置视觉标志位
 */
void VISION_SetFlagStatus(Vision_Flag_t flag)
{
	switch(flag)
	{
		case VISION_FLAG_SHOOT_ARMOR_4:
			Vision.TxPacket.TxData.buff_shoot_four = true;
		default:
			break;
	}
}

/**
 *	@brief	清除视觉标志位
 */
void VISION_ClearFlagStatus(Vision_Flag_t flag)
{
	switch(flag)
	{
		case VISION_FLAG_DATA_VALID:
			Vision.State.rx_data_valid = false;
			break;
		case VISION_FLAG_DATA_UPDATE:
			Vision.State.rx_data_update = false;
			break;
		case VISION_FLAG_SHOOT_ARMOR_4:
			Vision.TxPacket.TxData.buff_shoot_four = false;
		default:
			break;
	}
}

/**
 *	@brief	设置视觉模式
 */
void VISION_SetMode(Vision_Mode_t mode)
{
	Vision.State.mode = mode;
}

/**
 *	@brief	反馈视觉数据是否可信
 */
bool VISION_IsDataValid(void)
{
	return VISION_GetFlagStatus(VISION_FLAG_DATA_VALID);
}

/**
 *	@brief	反馈视觉距离
 */
float VISION_GetDistance(void)
{
	return Vision.RxPacket.RxData.distance;
}

/**
 *	@brief	获取yaw误差角度( #机械角度 )
 *	@note	镜头中心为(0,0)，把摄像头当成自己的眼睛(单目)
 *			物体位于镜头左边 - yaw_angle为负
 *				位于镜头右边 - yaw_angle为正
 *					云台左摆 - yaw减小	-> -YAW_DIR -> yaw增大
 *					云台右摆 - yaw增大 -> -YAW_DIR -> yaw减小
 */
float VISION_MECH_GetYawFeedback(void)
{
 	float yaw_err = 0.0f;
	/* 滤波/死区处理 */
	if(abs(Vision.RxPacket.RxData.yaw_angle) > 0.1f) {	
		yaw_err = (YAW_DIR) * Vision.RxPacket.RxData.yaw_angle/360.0f*8192.0f;
	}
	return yaw_err; 	
}

/**
 *	@brief	获取pitch误差角度( #机械角度 )
 *	@note	镜头中心为(0,0)，把摄像头当成自己的眼睛(单目)
 *			物体位于镜头下方 - pitch_angle为正
 *				位于镜头上方 - pitch_angle为负
 *					云台低头 - pitch增大
 *					云台抬头 - pitch减小
 */
float VISION_MECH_GetPitchFeedback(void)
{
	float pitch_err = 0.0f;
	
	if(abs(Vision.RxPacket.RxData.pitch_angle) > 65.f) {
		pitch_err = 0.0f;
	} else {
		pitch_err = Vision.RxPacket.RxData.pitch_angle/360.0f*8192.0f;
	}
	
	return pitch_err;
}

/**
 *	@brief	获取yaw误差角度( #放大后的欧拉角 )
 *	@note	镜头中心为(0,0)，把摄像头当成自己的眼睛(单目)
 *			物体位于镜头左边 - yaw_angle为负
 *				位于镜头右边 - yaw_angle为正
 *					云台左摆 - yaw减小	-> -YAW_DIR -> yaw增大
 *					云台右摆 - yaw增大 -> -YAW_DIR -> yaw减小
 */
float VISION_GYRO_GetYawFeedback(void)
{
	float yaw_err = 0.0f;
	
	yaw_err = (YAW_DIR) * Vision.RxPacket.RxData.yaw_angle * GIMBAL_GYRO_ANGLE_ZOOM_INDEX;
	return yaw_err; 	
}

/**
 *	@brief	获取pitch误差角度( #放大后的欧拉角 )
 *	@note	镜头中心为(0,0)，把摄像头当成自己的眼睛(单目)
 *			物体位于镜头下方 - pitch_angle为正
 *				位于镜头上方 - pitch_angle为负
 *					云台低头 - pitch增大
 *					云台抬头 - pitch减小
 */
float VISION_GYRO_GetPitchFeedback(void)
{
	float pitch_err = 0.0f;
	/* 滤波/死区处理 */
	if(abs(Vision.RxPacket.RxData.pitch_angle) > 0.1f) {
		pitch_err = Vision.RxPacket.RxData.pitch_angle * GIMBAL_GYRO_ANGLE_ZOOM_INDEX;
	} else {
		pitch_err = 0.0f;
	}
	return pitch_err;
}

/**
 *	@brief	获取yaw误差像素点，打符专用
 *	@note	左上角为(0,0)，把摄像头当成自己的眼睛(单目)
 *			物体位于镜头左边 - (Vision.RxPacket.RxData.yaw_angle - VISION_MID_YAW)为负
 *				位于镜头右边 - (Vision.RxPacket.RxData.yaw_angle - VISION_MID_YAW)为正
 *					云台左摆 - yaw减小	-> -YAW_DIR -> yaw增大
 *					云台右摆 - yaw增大 -> -YAW_DIR -> yaw减小
 */
float VISION_BUFF_GetYawFeedback(void)
{
	float yaw_err = 0.0f;
	yaw_err = (YAW_DIR) * (Vision.RxPacket.RxData.yaw_angle - VISION_MID_YAW);
	return yaw_err;
}

/**
 *	@brief	获取pitch误差角度，打符专用
 *	@note	左上角为(0,0)，把摄像头当成自己的眼睛(单目)
 *			物体位于镜头下方 - (Vision.RxPacket.RxData.pitch_angle - VISION_MID_PITCH)为正
 *				位于镜头上方 - (Vision.RxPacket.RxData.pitch_angle - VISION_MID_PITCH)为负
 *					云台低头 - pitch增大
 *					云台抬头 - pitch减小
 */
float VISION_BUFF_GetPitchFeedback(void)
{
	float pitch_err = 0.0f;
	pitch_err = (Vision.RxPacket.RxData.pitch_angle - VISION_MID_PITCH);
	return pitch_err;
}

/**
 *	@brief	信息反馈给视觉
 *	@note	
 */
uint8_t mode;
void VISION_OutputInfo(void)
{
	//Vision.TxPacket.TxData.fric_speed = Friction_Pwm_Speed[Friction.speedLevel];
	
	/* 通信测试函数 */
//	Vision.TxPacket.FrameHeader.cmd_id = mode;
//	VISION_SendData(&Vision.TxPacket, mode);
}

/**
 *	@brief	视觉获取系统信息
 */
void VISION_GetSysInfo(System_t *sys, Vision_Info_t *vision)
{
	/*----模式修改----*/
	switch(sys->Action)
	{
		case SYS_ACT_NORMAL: 
			{
				vision->State.mode = VISION_MODE_MANUAL;
			}break;
		case SYS_ACT_AUTO: 
			{
				vision->State.mode = VISION_MODE_AUTO;
			}break;
		case SYS_ACT_BUFF:
			{
				if(sys->BranchAction == BCH_ACT_SMALL_BUFF)
					vision->State.mode = VISION_MODE_SMALL_BUFF;
				else if(sys->BranchAction == BCH_ACT_BIG_BUFF)
					vision->State.mode = VISION_MODE_BIG_BUFF;
			}break;
		case SYS_ACT_PARK:
			{
				vision->State.mode = VISION_MODE_MANUAL;
			}break;
	}	
}

/**
 *	@brief	视觉获取系统信息
 */
void VISION_GetJudgeInfo(Judge_Info_t *judge, Vision_Info_t *vision)
{
	/* 裁判系统数据处理 */
	if(JUDGE_IfDataValid() == true) {
		Vision.TxPacket.TxData.my_color = JUDGE_eGeyMyColor();
		Vision.State.my_color = JUDGE_eGeyMyColor();
	}	
}

/**
 *	@brief	视觉获取摩擦轮射速信息
 */
void VISION_GetFrictionInfo(Friction_Info_t *fric, Vision_Info_t *vision)
{
	vision->TxPacket.TxData.fric_speed = fric->BulletSpeed;
}

/* #应用层# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	卡尔曼滤波器
 */
float kalman_kYaw_Q = 1;
float kalman_kYaw_R = 15;
float kalman_kPitch_Q = 1;
float kalman_kPitch_R = 10;
void VISION_Init(void)
{
	KalmanCreate(&Gimbal_Gyro_Auto_kalmanError[YAW_205], kalman_kYaw_Q, kalman_kYaw_R);
	KalmanCreate(&Gimbal_Gyro_Auto_kalmanError[PITCH_206], kalman_kPitch_Q, kalman_kPitch_R);

	for(uint16_t z = 0; z < VisionQueue.queueLength; z++) {
		VisionQueue.queue[z] = 0;
	}
	VisionQueue.nowLength = 0;
	visionYawMathExpect = 0;
	visionYawStdDevia = 0;
}

/** 
 *	@brief	离线状态下清除数据，防止出错
 */
void VISION_ClearRxPacket(void)
{
	Vision.RxPacket.RxData.pitch_angle = 0;
	Vision.RxPacket.RxData.yaw_angle = 0;
	Vision.RxPacket.RxData.distance = 0;
	Vision.RxPacket.RxData.buff_change_armor_four = 0;
	Vision.RxPacket.RxData.identify_target = 0;
	Vision.RxPacket.RxData.identify_buff = 0;
	Vision.RxPacket.RxData.identify_too_close = 0;
	Vision.RxPacket.RxData.anti_gyro = 0;
	Vision.RxPacket.RxData.anti_gyro_change_armor = 0;	
}

/**
 *	@brief	视觉离线处理
 */
void VISION_LostProc(void)
{
	/* 失联计数自增，UART4_IRQ 清零 */
	Vision.State.unconnected_cnt++;
	/* 失联判断 */
	if(Vision.State.unconnected_cnt > 50) {	// 500ms没接收到视觉反馈则认为视觉失联
		Vision.State.rx_data_valid = false;
		memset(&Vision.RxPacket.RxData, 0, sizeof(Vision_Rx_Data_t));	// 接收数据包清零
	}	
}

/**
 *	@brief	视觉信息更新
 *	@author	LZX	
 */
void VISION_UpdateInfo(float input, uint16_t length, QueueObj *queue, float *mathExpect, float *standardDeviation)
{
	float sum = 0;
	
	/* 防止数据溢出 */
	if(length>queue->queueLength)
		length = queue->queueLength;
	
	/* 队列未满的时候只进不出 */
	if(queue->nowLength < length)
	{
		queue->queue[queue->nowLength] = input;
		queue->nowLength++;
	}
	else
	{
		/* 队列已满，FIFO */
		for(uint16_t i = 0; i < length-1; i++)
		{
			/* 更新队列 */
			queue->queue[i] = queue->queue[i+1];
		}
		queue->queue[length-1] = input;
	}
	
	/* 更新完队列计算 数学期望 和 标准差 */
	
	for(uint16_t j = 0; j < length; j++)
	{
		sum += queue->queue[j];
	}
	*mathExpect = sum/(length/1.f);
	
	*standardDeviation = (input - *mathExpect);
}

/**
 *	@brief	u8类型的队列入队
 */
void cQueue_in(QueueObj *queue, uint8_t length, uint8_t input)
{
	/* 防止数据溢出 */
	if(length >= queue->queueLength)
		length = queue->queueLength;
	
	/* 队列未满的时候只进不出 */
	if(queue->nowLength < length)
	{
		queue->queue[queue->nowLength] = input;
		queue->nowLength++;
	}
	else
	{
		/* 队列已满，FIFO */
		for(uint16_t i = 0; i < length-1; i++)
		{
			/* 更新队列 */
			queue->queue[i] = queue->queue[i+1];
		}
		queue->queue[length-1] = input;
	}
}

/**
 *	@brief	u8类型的队列填满
 */
void cQueue_fill(QueueObj *queue, uint8_t length, uint8_t fill)
{
	uint8_t i;
	
	if(length >= queue->queueLength)
		length = queue->queueLength;
	
	for(i = 0; i < length; i++) {
		queue->queue[i] = fill;
	}
}

/**
 *	@brief	u8类型的队列填满
 */
bool cQueue_ifFilled(QueueObj *queue, uint8_t length, uint8_t fill)
{
	uint8_t i;
	
	for(i = 0; i < length; i++) {
		if(queue->queue[i] != fill) {
			return false;
		}
	}
	
	return true;
}

QueueObj target_speed=
{
	.nowLength =0,
	.queueLength = 100,
	.queue = {0}
};

/**
* @brief 获取目标的速度
* @param void
* @return void
*		以队列的逻辑
*/
float Get_Target_Speed(uint8_t queue_len,float angle)
{
	float sum=0;
	static float tmp=0;
	static float last_tmp=0;
	
	last_tmp = tmp;
	
	if(queue_len>target_speed.queueLength)
		queue_len=target_speed.queueLength;
	//防止溢出
	
	if(target_speed.nowLength<queue_len)
	{
		//队列未满，只进不出
		target_speed.queue[target_speed.nowLength] = angle;
		target_speed.nowLength++;
	}
	else
	{
		//队列已满，FIFO。
		for(uint16_t i=0;i<queue_len-1;i++)
		{
			target_speed.queue[i] = target_speed.queue[i+1];
			//更新队列
		
		}
		target_speed.queue[queue_len-1] = angle;
	}
	
	//更新完队列
	
	
	for(uint16_t j=0;j<target_speed.nowLength;j++)
	{
		sum+=target_speed.queue[j];
	}
	tmp = sum/(target_speed.nowLength/1.f);
	
//	tmp = (angle - tmp);	
	
	return tmp - last_tmp;
}


QueueObj target_accel=
{
	.nowLength =0,
	.queueLength = 100,
	.queue = {0}
};

/**
* @brief 获取目标的加速度
* @param void
* @return void
*		以队列的逻辑
*/
float Get_Target_Accel(uint8_t queue_len,float speed)
{
	float sum=0;
	float tmp=0;
	
	if(queue_len>target_accel.queueLength)
		queue_len=target_accel.queueLength;
	//防止溢出
	
	
	if(target_accel.nowLength<queue_len)
	{
		//队列未满，只进不出
		target_accel.queue[target_accel.nowLength] = speed;
		target_accel.nowLength++;
	}
	else
	{
		//队列已满，FIFO。
		for(uint16_t i=0;i<queue_len-1;i++)
		{
			target_accel.queue[i] = target_accel.queue[i+1];
			//更新队列
		
		}
		target_accel.queue[queue_len-1] = speed;
	}
	
	//更新完队列
	
	
	for(uint16_t j=0;j<target_accel.nowLength;j++)
	{
		sum+=target_accel.queue[j];
	}
	tmp = sum/(target_accel.nowLength/1.f);
	
	tmp = (speed - tmp);	
	
	return tmp;
}

/* #任务层# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	视觉信息记录反馈
 *	@note	失联处理可能存在问题
 */
void VISION_GetInfo(void)
{
	// 视觉获取系统信息
	VISION_GetSysInfo(&System, &Vision);
	// 视觉获取裁判系统信息
	VISION_GetJudgeInfo(&Judge, &Vision);
	// 视觉获取射速信息
	VISION_GetFrictionInfo(&Friction, &Vision);
}

/**
 *	@brief	手动控制
 */
void VISION_ManualCtrl(void)
{
	/* 关闭自瞄 */
	VISION_SendData(&Vision.TxPacket, CMD_AIM_OFF);
}

/**
 *	@brief	自动控制
 */
void VISION_AutoCtrl(void)
{
	VISION_SendData(&Vision.TxPacket, CMD_AIM_AUTO);
}

/**
 *	@brief	打符控制
 *	@note	Ctrl+V - 击打小符
 *			Ctrl+F - 击打大符
 *			
 */
void VISION_BuffCtrl(void)
{
	/* 根据自身颜色判断击打的能量机关颜色 */
	if(Vision.State.mode == VISION_MODE_BIG_BUFF) {	// 大符模式
		VISION_SendData(&Vision.TxPacket, CMD_AIM_BIG_BUFF);	
	} 
	else if(Vision.State.mode == VISION_MODE_SMALL_BUFF) {	// 小符模式
		VISION_SendData(&Vision.TxPacket, CMD_AIM_SMALL_BUFF);		
	}
}

/**
 *	@brief	视觉控制
 */
void VISION_Ctrl(void)
{
	/*----信息读入----*/
	VISION_GetInfo();
	VISION_LostProc();
	/*----信息输出----*/
	//VISION_OutputInfo();
	/*----最终输出----*/
	switch(Vision.State.mode)
	{
		case VISION_MODE_MANUAL:
			VISION_ManualCtrl();// 手动控制
			break;
		case VISION_MODE_AUTO:
			VISION_AutoCtrl();	// 自动控制
			break;
		case VISION_MODE_BIG_BUFF:
		case VISION_MODE_SMALL_BUFF:
			VISION_BuffCtrl();	// 打符控制
			break;
		default:
			break;
	}
}

