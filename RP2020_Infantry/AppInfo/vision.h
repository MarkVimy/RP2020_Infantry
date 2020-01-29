#ifndef __VISION_H
#define __VISION_H

/* Includes ------------------------------------------------------------------*/
#include "sys.h"
#include "my_app.h"

/* Global macro --------------------------------------------------------------*/
/* Global TypeDef ------------------------------------------------------------*/
/**
 *	@brief	�Ӿ���־λ
 */
typedef enum
{
	VISION_FLAG_DATA_VALID  = 0, // �Ӿ�������Ч���ж�
	VISION_FLAG_DATA_UPDATE = 1, // �Ӿ����ݸ��±�־λ
	VISION_FLAG_LOCK_TARGET = 2, // ʶ�𵽵з�װ�װ�
	VISION_FLAG_LOCK_BUFF   = 3, // ʶ�𵽴�С��
	VISION_FLAG_CHANGE_ARMOR = 4,// �л�װ�װ�
	VISION_FLAG_CHANGE_ARMOR_4 = 5, // �����ʱ���л������Ŀ�װ�װ�
	VISION_FLAG_SHOOT_ARMOR_4 = 6,	// �����ʱ�����˵��Ŀ��ӵ�
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
bool VISION_readData(uint8_t *rxBuf);
//void VISION_sendData(Vision_Tx_Packet_t *txPacket, Vision_Cmd_Id_t cmd_id);
//Vision_Color_t VISION_getMyColor(Judge_Info_t *info);

/* #Ӧ�ò�# ---------------------------------------------------------------------------------------------------------------------------------------*/
void VISION_init(void);
bool VISION_getFlagStatus(Vision_Flag_t flag);
void VISION_setFlagStatus(Vision_Flag_t flag);
void VISION_clearFlagStatus(Vision_Flag_t flag);
void VISION_setMode(Vision_Mode_t mode);
bool VISION_isDataValid(void);

float VISION_getDistance(void);
float VISION_MECH_getYawFeedback(void);
float VISION_MECH_getPitchFeedback(void);
float VISION_GYRO_getYawFeedback(void);
float VISION_GYRO_getPitchFeedback(void);
float VISION_BUFF_getYawFeedback(void);
float VISION_BUFF_getPitchFeedback(void);

void VISION_updateInfo(float input, uint16_t length, QueueObj *queue, float *mathExpect, float *standardDeviation);
/* u8������� */
void cQueue_in(QueueObj *queue, uint8_t length, uint8_t input);
void cQueue_fill(QueueObj *queue, uint8_t length, uint8_t fill);
bool cQueue_ifFilled(QueueObj *queue, uint8_t length, uint8_t fill);

/* #�����# ---------------------------------------------------------------------------------------------------------------------------------------*/
void VISION_rcControlTask(void);
void VISION_keyControlTask(void);
void VISION_manualControl(void);
void VISION_autoControl(void);
void VISION_buffControl(void);
void VISION_control(void);

#endif
