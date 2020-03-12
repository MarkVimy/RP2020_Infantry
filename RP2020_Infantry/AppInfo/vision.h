#ifndef __VISION_H
#define __VISION_H

/* Includes ------------------------------------------------------------------*/
#include "sys.h"

/* Global macro --------------------------------------------------------------*/
/* Global TypeDef ------------------------------------------------------------*/
/**
 *	@brief	�Ӿ���־λ
 */
typedef enum
{
	VISION_FLAG_DATA_VALID  	= 0, // �Ӿ�������Ч���ж�
	VISION_FLAG_DATA_UPDATE 	= 1, // �Ӿ����ݸ��±�־λ
	VISION_FLAG_LOCK_TARGET 	= 2, // ʶ�𵽵з�װ�װ�
	VISION_FLAG_LOCK_BUFF   	= 3, // ʶ�𵽴�С��
	VISION_FLAG_CHANGE_ARMOR 	= 4, // �л�װ�װ�
	VISION_FLAG_CHANGE_ARMOR_4 	= 5, // �����ʱ���л������Ŀ�װ�װ�
	VISION_FLAG_SHOOT_ARMOR_4 	= 6, // �����ʱ�����˵��Ŀ��ӵ�
} Vision_Flag_t;

/**
 *	@brief	�Ӿ�ģʽ
 */
typedef enum
{
	VISION_MODE_MANUAL		= 0,	// �ֶ�ģʽ
	VISION_MODE_AUTO		= 1,	// ����ģʽ
	VISION_MODE_BIG_BUFF	= 2,	// ����ģʽ
	VISION_MODE_SMALL_BUFF	= 3,	// ��С��ģʽ
} Vision_Mode_t;

/* ## Global Variables Prototypes ## -----------------------------------------*/
extern QueueObj VisionQueue;

/* API functions Prototypes --------------------------------------------------*/
/* #������# ---------------------------------------------------------------------------------------------------------------------------------------*/
bool VISION_ReadData(uint8_t *rxBuf);
//void VISION_SendData(Vision_Tx_Packet_t *txPacket, Vision_Cmd_Id_t cmd_id);
//Vision_Color_t VISION_GetMyColor(Judge_Info_t *info);

/* #��Ϣ��# ---------------------------------------------------------------------------------------------------------------------------------------*/
bool VISION_GetFlagStatus(Vision_Flag_t flag);
void VISION_SetFlagStatus(Vision_Flag_t flag);
void VISION_ClearFlagStatus(Vision_Flag_t flag);
void VISION_SetMode(Vision_Mode_t mode);
bool VISION_IsDataValid(void);

float VISION_GetDistance(void);
float VISION_MECH_GetYawFeedback(void);
float VISION_MECH_GetPitchFeedback(void);
float VISION_GYRO_GetYawFeedback(void);
float VISION_GYRO_GetPitchFeedback(void);
float VISION_BUFF_GetYawFeedback(void);
float VISION_BUFF_GetPitchFeedback(void);

/* #Ӧ�ò�# ---------------------------------------------------------------------------------------------------------------------------------------*/
void VISION_Init(void);
void VISION_ClearRxPacket(void);void VISION_UpdateInfo(float input, uint16_t length, QueueObj *queue, float *mathExpect, float *standardDeviation);
/* u8������� */
void cQueue_in(QueueObj *queue, uint8_t length, uint8_t input);
void cQueue_fill(QueueObj *queue, uint8_t length, uint8_t fill);
bool cQueue_ifFilled(QueueObj *queue, uint8_t length, uint8_t fill);

float Get_Target_Speed(uint8_t queue_len,float angle);
float Get_Target_Accel(uint8_t queue_len,float speed);

/* #�����# ---------------------------------------------------------------------------------------------------------------------------------------*/
void VISION_GetInfo(void);
void VISION_ManualCtrl(void);
void VISION_AutoCtrl(void);
void VISION_BuffCtrl(void);
void VISION_Ctrl(void);

#endif
