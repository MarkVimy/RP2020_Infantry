/**
 * @file        vision.c
 * @author      MaWeiming
 * @Version     V1.0
 * @date        27-October-2019
 * @brief       This file includes the VISION Communication Protocols(�Ӿ�ͨ��Э��) external functions 
 * 				(based on ST Peripherals Libaray Keil.STM32F4xx_DFP.2.13.0)
 * @Version		
 */

/**
 *	# ���Ӿ���ͨ��ʹ��USART4��ΪͨѶ�ӿ�
 *	ͨ��Э��ĸ�ʽ�����ϵͳЭ��ĸ�ʽ����
 *	ͨ��Э���ʽ:
 *				֡ͷ		���ݶ�			֡β
 *	��		frame_header 	data  			frame_tailer(CRC16,����У��)
 *	�ֽ�		3			x				2
 *	
 *	֡ͷ��ʽ:
 *		  ��ʼ�ֽ�(0xA5)	���������		֡ͷCRC8У��
 *	��			SOF 	 	cmd_id    		CRC8
 *	�ֽ�		0			1				1
 */

/* Includes ------------------------------------------------------------------*/
#include "vision.h"

#include "uart4.h"
#include "crc.h"
#include "can.h"
#include "kalman.h"

/* Private typedef -----------------------------------------------------------*/
/**
 *	���->�Ӿ�
 */
typedef enum {
	CMD_AIM_OFF 		= 0x00,	// ����������(ʵ�����Ӿ��Ǳ��ܳ��򵫲���������)
	CMD_AIM_AUTO		= 0x01,	// ��������
	CMD_AIM_SMALL_BUFF	= 0x02,	// ʶ��С��
	CMD_AIM_BIG_BUFF	= 0x03,	// ʶ����
	CMD_AIM_SENTRY		= 0x04,	// �����ڱ�
	CMD_AIM_BASE		= 0x05	// �������
} Vision_Cmd_Id_t;

/* ֡ͷ�ֽ�ƫ�� */
typedef enum {
	SOF			= 0,
	CMD_ID		= 1,
	CRC8		= 2,
	DATA		= 3,
	TX_CRC16		= 20,
} Vision_Frame_Header_Offset_t;

/* ���ݳ��� */
typedef enum {
	/* Std */
	LEN_FRAME_HEADER 		= 3,	// ֡ͷ����
	LEN_RX_DATA 			= 18,	// �������ݶγ���
	LEN_TX_DATA 			= 17,	// �������ݶγ���
	LEN_FRAME_TAILER 		= 2,	// ֡βCRC16
	LEN_VISION_RX_PACKET	= 23,	// ���հ���������
	LEN_VISION_TX_PACKET	= 22,	// ���Ͱ���������
} Vision_Data_Length_t;

/* ֡ͷ��ʽ */
typedef __packed struct
{
	uint8_t  			sof;		// ͬ��ͷ
	Vision_Cmd_Id_t  	cmd_id;		// ������
	uint8_t  			crc8;		// CRC8У����
} Vision_Frame_Header_t;

/* ֡β��ʽ */
typedef __packed struct 
{
	uint16_t crc16;					// CRC16У����
} Vision_Frame_Tailer_t;

/* �������ݶθ�ʽ */
typedef __packed struct 
{
	float 	pitch_angle;	// pitchƫ��Ƕ�/���ص�	��λ���Ƕ�/���ص�
	float 	yaw_angle;		// yawƫ��Ƕ�/���ص�	��λ���Ƕ�/���ص�
	float 	distance;			// ����				��λ��mm
	uint8_t buff_change_armor_four;	// �Ƿ��л������Ŀ�װ�װ�		��λ��0/1
	uint8_t identify_target;// �Ƿ�ʶ��Ŀ��	��λ��0/1
	uint8_t identify_buff;	// �Ƿ�ʶ��Buff	��λ��0/1
	uint8_t identify_too_close;	// Ŀ��������	��λ��0/1
	uint8_t anti_gyro;	// �Ƿ�ʶ��С����	��λ��0/1
	uint8_t	anti_gyro_change_armor;	// �Ƿ��ڷ�����״̬���л�װ�װ�	��λ��0/1
} Vision_Rx_Data_t;

/* �������ݶθ�ʽ */
typedef __packed struct
{
	uint8_t buff_shoot_four;// �����ʱ��������Ŀ��ӵ�
	uint8_t fric_speed;		// ���ٵ�λ(���ݵȼ�����)
	uint8_t my_color;		// ���Լ�����ɫ
} Vision_Tx_Data_t;

/* ���հ���ʽ */
typedef __packed struct 
{
	Vision_Frame_Header_t FrameHeader;	// ֡ͷ
	Vision_Rx_Data_t	  RxData;		// ����
	Vision_Frame_Tailer_t FrameTailer;	// ֡β	
} Vision_Rx_Packet_t;

/* ���Ͱ���ʽ */
typedef __packed struct
{
	Vision_Frame_Header_t FrameHeader;	// ֡ͷ
	Vision_Tx_Data_t	  TxData;		// ����
	Vision_Frame_Tailer_t FrameTailer;	// ֡β		
} Vision_Tx_Packet_t;

/* ������ʶ���� */
typedef struct
{
	Color_t 		my_color;			// �������Լ�����ɫ
	Vision_Mode_t	mode;				// �Ӿ�ģʽ
	uint16_t		unconnected_cnt;	// �Ӿ�ʧ������
	uint8_t  		rx_data_valid;		// �������ݵ���ȷ��
	uint16_t 		rx_err_cnt;			// �������ݵĴ���ͳ��
	uint32_t		rx_cnt;				// �������ݰ���ͳ��
	uint8_t 		rx_data_update;		// ���������Ƿ����
	uint32_t 		rx_time_prev;		// �������ݵ�ǰһʱ��
	uint32_t 		rx_time_now;		// �������ݵĵ�ǰʱ��
	uint16_t 		rx_time_ping;		// ֡��
} Vision_State_t;

/* �Ӿ�ͨ�����ݰ���ʽ */
typedef struct {
	Vision_Rx_Packet_t RxPacket;
	Vision_Tx_Packet_t TxPacket;
	Vision_State_t     State;
} Vision_Info_t;

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#define VISION_TX_BUFFER_LEN	50
#define VISION_FRAME_HEADER		(0xA5)

/*-------�Ӿ��ֱ���Ԥ����--------*/
#define	VISION_1280P	0	// 1280(H)*720(V)	��Ҫ���ڴ����ʱ���������ص�
#define	VISION_640P		1	// 640(H)*480(V)

#define VISION_DPI		VISION_1280P
#if VISION_DPI == VISION_1280P
	#define VISION_MID_YAW		480
	#define VISION_MID_PITCH	512
#elif VISION_DPI == VISION_640P
	#define	VISION_MID_YAW		320
	#define	VISION_MID_PITCH	240
#endif


/* Private variables ---------------------------------------------------------*/
QueueObj VisionQueue =
{
	.nowLength = 0,
	.queueLength = 20,
	.queue = {0}
};


float visionYawMathExpect;
//��ѧ����
float visionYawStdDevia;
//��׼��


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
/* #������# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	��ȡ�Ӿ�ͨ������
 *	@return false - ���ݴ���
 *					true  - ������ȷ	
 *	@note	uart4.c��IRQ����
 */
bool VISION_ReadData(uint8_t *rxBuf)
{
	uint8_t res = false;
	Vision.State.rx_cnt++;
	Vision.State.unconnected_cnt = 0;
	/* ֡���ֽ��Ƿ�Ϊ0xA5 */
	if(rxBuf[SOF] == VISION_FRAME_HEADER) 
	{	
		res = Verify_CRC8_Check_Sum( rxBuf, LEN_FRAME_HEADER );
		/* ֡ͷCRC8У��*/
		if(res == true)
		{
			res = Verify_CRC16_Check_Sum( rxBuf, LEN_VISION_RX_PACKET );
			/* ֡βCRC16У�� */
			if(res == true) 
			{
				/* ������ȷ�򿽱����հ� */
				memcpy(&Vision.RxPacket, rxBuf, LEN_VISION_RX_PACKET);
				Vision.State.rx_data_update = true;	// �Ӿ����ݸ���
				
				/* ֡�ʼ��� */
				Vision.State.rx_time_now = xTaskGetTickCount();
				Vision.State.rx_time_ping = Vision.State.rx_time_now - Vision.State.rx_time_prev;
				Vision.State.rx_time_prev = Vision.State.rx_time_now;
				
			}
		}
	}
	
	/* ������Ч���ж� */
	if(res == true) {
		Vision.State.rx_data_valid = true;
	} else if(res == false) {
		Vision.State.rx_data_valid = false;
		Vision.State.rx_err_cnt++;
	}
	
	return res;
}

/**
 *	@brief	�����Ӿ�ͨ������
 *	@note	uart4����
 */
void VISION_SendData(Vision_Tx_Packet_t *txPacket, Vision_Cmd_Id_t cmd_id)
{
	uint8_t i;
	/* ֡ͷ��ʽ */
	txPacket->FrameHeader.sof = VISION_FRAME_HEADER;
	txPacket->FrameHeader.cmd_id = cmd_id;
	/* д��֡ͷ */
	memcpy( Vision_Tx_Buffer, &txPacket->FrameHeader, LEN_FRAME_HEADER );
	/* д��֡ͷCRC8 */
	Append_CRC8_Check_Sum( Vision_Tx_Buffer, LEN_FRAME_HEADER);
	Vision.TxPacket.FrameHeader.crc8 = Vision_Tx_Buffer[CRC8];
	
	/* ���ݶ���� */
	memcpy( &Vision_Tx_Buffer[DATA], &txPacket->TxData, LEN_TX_DATA );
	/* д��֡βCRC16 */
	Append_CRC16_Check_Sum( Vision_Tx_Buffer, LEN_VISION_TX_PACKET);
	Vision.TxPacket.FrameTailer.crc16 = Vision_Tx_Buffer[TX_CRC16];
	
	/* �����������ݰ����͸��Ӿ�PC */
	for(i = 0; i < LEN_VISION_TX_PACKET; i++) {
		UART4_SendChar(Vision_Tx_Buffer[i]);
	}
	
	/* �������ݰ����� */
	memset(Vision_Tx_Buffer, 0, VISION_TX_BUFFER_LEN);
}


/* #��Ϣ��# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	�����Ӿ���־λ
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
 *	@brief	�����Ӿ���־λ
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
 *	@brief	����Ӿ���־λ
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
 *	@brief	�����Ӿ�ģʽ
 */
void VISION_SetMode(Vision_Mode_t mode)
{
	Vision.State.mode = mode;
}

/**
 *	@brief	�����Ӿ������Ƿ����
 */
bool VISION_IsDataValid(void)
{
	return VISION_GetFlagStatus(VISION_FLAG_DATA_VALID);
}

/**
 *	@brief	�����Ӿ�����
 */
float VISION_GetDistance(void)
{
	return Vision.RxPacket.RxData.distance;
}

/**
 *	@brief	��ȡyaw���Ƕ�( #��е�Ƕ� )
 *	@note	��ͷ����Ϊ(0,0)��������ͷ�����Լ����۾�(��Ŀ)
 *			����λ�ھ�ͷ��� - yaw_angleΪ��
 *				λ�ھ�ͷ�ұ� - yaw_angleΪ��
 *					��̨��� - yaw��С	-> -YAW_DIR -> yaw����
 *					��̨�Ұ� - yaw���� -> -YAW_DIR -> yaw��С
 */
float VISION_MECH_GetYawFeedback(void)
{
 	float yaw_err = 0.0f;
	/* �˲�/�������� */
	if(abs(Vision.RxPacket.RxData.yaw_angle) > 0.1f) {	
		yaw_err = (YAW_DIR) * Vision.RxPacket.RxData.yaw_angle/360.0f*8192.0f;
	}
	return yaw_err; 	
}

/**
 *	@brief	��ȡpitch���Ƕ�( #��е�Ƕ� )
 *	@note	��ͷ����Ϊ(0,0)��������ͷ�����Լ����۾�(��Ŀ)
 *			����λ�ھ�ͷ�·� - pitch_angleΪ��
 *				λ�ھ�ͷ�Ϸ� - pitch_angleΪ��
 *					��̨��ͷ - pitch����
 *					��̨̧ͷ - pitch��С
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
 *	@brief	��ȡyaw���Ƕ�( #�Ŵ���ŷ���� )
 *	@note	��ͷ����Ϊ(0,0)��������ͷ�����Լ����۾�(��Ŀ)
 *			����λ�ھ�ͷ��� - yaw_angleΪ��
 *				λ�ھ�ͷ�ұ� - yaw_angleΪ��
 *					��̨��� - yaw��С	-> -YAW_DIR -> yaw����
 *					��̨�Ұ� - yaw���� -> -YAW_DIR -> yaw��С
 */
float VISION_GYRO_GetYawFeedback(void)
{
	float yaw_err = 0.0f;
	
	yaw_err = (YAW_DIR) * Vision.RxPacket.RxData.yaw_angle * GIMBAL_GYRO_ANGLE_ZOOM_INDEX;
	return yaw_err; 	
}

/**
 *	@brief	��ȡpitch���Ƕ�( #�Ŵ���ŷ���� )
 *	@note	��ͷ����Ϊ(0,0)��������ͷ�����Լ����۾�(��Ŀ)
 *			����λ�ھ�ͷ�·� - pitch_angleΪ��
 *				λ�ھ�ͷ�Ϸ� - pitch_angleΪ��
 *					��̨��ͷ - pitch����
 *					��̨̧ͷ - pitch��С
 */
float VISION_GYRO_GetPitchFeedback(void)
{
	float pitch_err = 0.0f;
	/* �˲�/�������� */
	if(abs(Vision.RxPacket.RxData.pitch_angle) > 0.1f) {
		pitch_err = Vision.RxPacket.RxData.pitch_angle * GIMBAL_GYRO_ANGLE_ZOOM_INDEX;
	} else {
		pitch_err = 0.0f;
	}
	return pitch_err;
}

/**
 *	@brief	��ȡyaw������ص㣬���ר��
 *	@note	���Ͻ�Ϊ(0,0)��������ͷ�����Լ����۾�(��Ŀ)
 *			����λ�ھ�ͷ��� - (Vision.RxPacket.RxData.yaw_angle - VISION_MID_YAW)Ϊ��
 *				λ�ھ�ͷ�ұ� - (Vision.RxPacket.RxData.yaw_angle - VISION_MID_YAW)Ϊ��
 *					��̨��� - yaw��С	-> -YAW_DIR -> yaw����
 *					��̨�Ұ� - yaw���� -> -YAW_DIR -> yaw��С
 */
float VISION_BUFF_GetYawFeedback(void)
{
	float yaw_err = 0.0f;
	yaw_err = (YAW_DIR) * (Vision.RxPacket.RxData.yaw_angle - VISION_MID_YAW);
	return yaw_err;
}

/**
 *	@brief	��ȡpitch���Ƕȣ����ר��
 *	@note	���Ͻ�Ϊ(0,0)��������ͷ�����Լ����۾�(��Ŀ)
 *			����λ�ھ�ͷ�·� - (Vision.RxPacket.RxData.pitch_angle - VISION_MID_PITCH)Ϊ��
 *				λ�ھ�ͷ�Ϸ� - (Vision.RxPacket.RxData.pitch_angle - VISION_MID_PITCH)Ϊ��
 *					��̨��ͷ - pitch����
 *					��̨̧ͷ - pitch��С
 */
float VISION_BUFF_GetPitchFeedback(void)
{
	float pitch_err = 0.0f;
	pitch_err = (Vision.RxPacket.RxData.pitch_angle - VISION_MID_PITCH);
	return pitch_err;
}

/**
 *	@brief	��Ϣ�������Ӿ�
 *	@note	
 */
uint8_t mode;
void VISION_OutputInfo(void)
{
	//Vision.TxPacket.TxData.fric_speed = Friction_Pwm_Speed[Friction.speedLevel];
	
	/* ͨ�Ų��Ժ��� */
//	Vision.TxPacket.FrameHeader.cmd_id = mode;
//	VISION_SendData(&Vision.TxPacket, mode);
}

/**
 *	@brief	�Ӿ���ȡϵͳ��Ϣ
 */
void VISION_GetSysInfo(System_t *sys, Vision_Info_t *vision)
{
	/*----ģʽ�޸�----*/
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
 *	@brief	�Ӿ���ȡϵͳ��Ϣ
 */
void VISION_GetJudgeInfo(Judge_Info_t *judge, Vision_Info_t *vision)
{
	/* ����ϵͳ���ݴ��� */
	if(JUDGE_IfDataValid() == true) {
		Vision.TxPacket.TxData.my_color = JUDGE_eGeyMyColor();
		Vision.State.my_color = JUDGE_eGeyMyColor();
	}	
}

/**
 *	@brief	�Ӿ���ȡĦ����������Ϣ
 */
void VISION_GetFrictionInfo(Friction_Info_t *fric, Vision_Info_t *vision)
{
	vision->TxPacket.TxData.fric_speed = fric->Speed;
}

/* #Ӧ�ò�# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	�������˲���
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
 *	@brief	����״̬��������ݣ���ֹ����
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
 *	@brief	�Ӿ����ߴ���
 */
void VISION_LostProc(void)
{
	/* ʧ������������UART4_IRQ ���� */
	Vision.State.unconnected_cnt++;
	/* ʧ���ж� */
	if(Vision.State.unconnected_cnt > 50) {	// 500msû���յ��Ӿ���������Ϊ�Ӿ�ʧ��
		Vision.State.rx_data_valid = false;
		memset(&Vision.RxPacket.RxData, 0, sizeof(Vision_Rx_Data_t));	// �������ݰ�����
	}	
}

/**
 *	@brief	�Ӿ���Ϣ����
 *	@author	LZX	
 */
void VISION_UpdateInfo(float input, uint16_t length, QueueObj *queue, float *mathExpect, float *standardDeviation)
{
	float sum = 0;
	
	/* ��ֹ������� */
	if(length>queue->queueLength)
		length = queue->queueLength;
	
	/* ����δ����ʱ��ֻ������ */
	if(queue->nowLength < length)
	{
		queue->queue[queue->nowLength] = input;
		queue->nowLength++;
	}
	else
	{
		/* ����������FIFO */
		for(uint16_t i = 0; i < length-1; i++)
		{
			/* ���¶��� */
			queue->queue[i] = queue->queue[i+1];
		}
		queue->queue[length-1] = input;
	}
	
	/* ��������м��� ��ѧ���� �� ��׼�� */
	
	for(uint16_t j = 0; j < length; j++)
	{
		sum += queue->queue[j];
	}
	*mathExpect = sum/(length/1.f);
	
	*standardDeviation = (input - *mathExpect);
}

/**
 *	@brief	u8���͵Ķ������
 */
void cQueue_in(QueueObj *queue, uint8_t length, uint8_t input)
{
	/* ��ֹ������� */
	if(length >= queue->queueLength)
		length = queue->queueLength;
	
	/* ����δ����ʱ��ֻ������ */
	if(queue->nowLength < length)
	{
		queue->queue[queue->nowLength] = input;
		queue->nowLength++;
	}
	else
	{
		/* ����������FIFO */
		for(uint16_t i = 0; i < length-1; i++)
		{
			/* ���¶��� */
			queue->queue[i] = queue->queue[i+1];
		}
		queue->queue[length-1] = input;
	}
}

/**
 *	@brief	u8���͵Ķ�������
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
 *	@brief	u8���͵Ķ�������
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
* @brief ��ȡĿ����ٶ�
* @param void
* @return void
*		�Զ��е��߼�
*/
float Get_Target_Speed(uint8_t queue_len,float angle)
{
	float sum=0;
	float tmp=0;
	
	if(queue_len>target_speed.queueLength)
		queue_len=target_speed.queueLength;
	//��ֹ���
	
	
	if(target_speed.nowLength<queue_len)
	{
		//����δ����ֻ������
		target_speed.queue[target_speed.nowLength] = angle;
		target_speed.nowLength++;
	}
	else
	{
		//����������FIFO��
		for(uint16_t i=0;i<queue_len-1;i++)
		{
			target_speed.queue[i] = target_speed.queue[i+1];
			//���¶���
		
		}
		target_speed.queue[queue_len-1] = angle;
	}
	
	//���������
	
	
	for(uint16_t j=0;j<queue_len;j++)
	{
		sum+=target_speed.queue[j];
	}
	tmp = sum/(queue_len/1.f);
	
	tmp = (angle - tmp);	
	
	return tmp;
}


QueueObj target_accel=
{
	.nowLength =0,
	.queueLength = 100,
	.queue = {0}
};

/**
* @brief ��ȡĿ��ļ��ٶ�
* @param void
* @return void
*		�Զ��е��߼�
*/
float Get_Target_Accel(uint8_t queue_len,float speed)
{
	float sum=0;
	float tmp=0;
	
	if(queue_len>target_accel.queueLength)
		queue_len=target_accel.queueLength;
	//��ֹ���
	
	
	if(target_accel.nowLength<queue_len)
	{
		//����δ����ֻ������
		target_accel.queue[target_accel.nowLength] = speed;
		target_accel.nowLength++;
	}
	else
	{
		//����������FIFO��
		for(uint16_t i=0;i<queue_len-1;i++)
		{
			target_accel.queue[i] = target_accel.queue[i+1];
			//���¶���
		
		}
		target_accel.queue[queue_len-1] = speed;
	}
	
	//���������
	
	
	for(uint16_t j=0;j<queue_len;j++)
	{
		sum+=target_accel.queue[j];
	}
	tmp = sum/(queue_len/1.f);
	
	tmp = (speed - tmp);	
	
	return tmp;
}

/* #�����# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	�Ӿ���Ϣ��¼����
 *	@note	ʧ��������ܴ�������
 */
void VISION_GetInfo(void)
{
	// �Ӿ���ȡϵͳ��Ϣ
	VISION_GetSysInfo(&System, &Vision);
	// �Ӿ���ȡ����ϵͳ��Ϣ
	VISION_GetJudgeInfo(&Judge, &Vision);
	// �Ӿ���ȡ������Ϣ
	VISION_GetFrictionInfo(&Friction, &Vision);
}

/**
 *	@brief	�ֶ�����
 */
void VISION_ManualCtrl(void)
{
	/* �ر����� */
	VISION_SendData(&Vision.TxPacket, CMD_AIM_OFF);
}

/**
 *	@brief	�Զ�����
 */
void VISION_AutoCtrl(void)
{
	VISION_SendData(&Vision.TxPacket, CMD_AIM_AUTO);
}

/**
 *	@brief	�������
 *	@note	Ctrl+V - ����С��
 *			Ctrl+F - ������
 *			
 */
void VISION_BuffCtrl(void)
{
	/* ����������ɫ�жϻ��������������ɫ */
	if(Vision.State.mode == VISION_MODE_BIG_BUFF) {	// ���ģʽ
		VISION_SendData(&Vision.TxPacket, CMD_AIM_BIG_BUFF);	
	} 
	else if(Vision.State.mode == VISION_MODE_SMALL_BUFF) {	// С��ģʽ
		VISION_SendData(&Vision.TxPacket, CMD_AIM_SMALL_BUFF);		
	}
}

/**
 *	@brief	�Ӿ�����
 */
void VISION_Ctrl(void)
{
	/*----��Ϣ����----*/
	VISION_GetInfo();
	VISION_LostProc();
	/*----��Ϣ���----*/
	//VISION_OutputInfo();
	/*----�������----*/
	switch(Vision.State.mode)
	{
		case VISION_MODE_MANUAL:
			VISION_ManualCtrl();// �ֶ�����
			break;
		case VISION_MODE_AUTO:
			VISION_AutoCtrl();	// �Զ�����
			break;
		case VISION_MODE_BIG_BUFF:
		case VISION_MODE_SMALL_BUFF:
			VISION_BuffCtrl();	// �������
			break;
		default:
			break;
	}
}

