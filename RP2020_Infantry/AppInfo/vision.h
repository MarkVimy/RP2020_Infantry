#ifndef __VISION_H
#define __VISION_H

/* Includes ------------------------------------------------------------------*/
#include "sys.h"

/* Global macro --------------------------------------------------------------*/
/* Global TypeDef ------------------------------------------------------------*/
/**
 *	@brief	视觉标志位
 */
typedef enum
{
	VISION_FLAG_DATA_VALID  	= 0, // 视觉数据有效性判断
	VISION_FLAG_DATA_UPDATE 	= 1, // 视觉数据更新标志位
	VISION_FLAG_LOCK_TARGET 	= 2, // 识别到敌方装甲板
	VISION_FLAG_LOCK_BUFF   	= 3, // 识别到大小符
	VISION_FLAG_CHANGE_ARMOR 	= 4, // 切换装甲板
	VISION_FLAG_CHANGE_ARMOR_4 	= 5, // 打符的时候切换到第四块装甲板
	VISION_FLAG_SHOOT_ARMOR_4 	= 6, // 打符的时候打出了第四颗子弹
} Vision_Flag_t;

/**
 *	@brief	视觉模式
 */
typedef enum
{
	VISION_MODE_MANUAL		= 0,	// 手动模式
	VISION_MODE_AUTO		= 1,	// 自瞄模式
	VISION_MODE_BIG_BUFF	= 2,	// 打大符模式
	VISION_MODE_SMALL_BUFF	= 3,	// 打小符模式
} Vision_Mode_t;

/* ## Global Variables Prototypes ## -----------------------------------------*/
extern QueueObj VisionQueue;
extern QueueObj VisionQueueAccel;

/* API functions Prototypes --------------------------------------------------*/
/* #驱动层# ---------------------------------------------------------------------------------------------------------------------------------------*/
bool VISION_ReadData(uint8_t *rxBuf);
//void VISION_SendData(Vision_Tx_Packet_t *txPacket, Vision_Cmd_Id_t cmd_id);
//Vision_Color_t VISION_GetMyColor(Judge_Info_t *info);

/* #信息层# ---------------------------------------------------------------------------------------------------------------------------------------*/
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

/* #应用层# ---------------------------------------------------------------------------------------------------------------------------------------*/
void VISION_Init(void);
void VISION_ClearRxPacket(void);void VISION_UpdateInfo(float input, uint16_t length, QueueObj *queue, float *mathExpect, float *standardDeviation);
/* u8队列入队 */
void cQueue_in(QueueObj *queue, uint8_t length, uint8_t input);
void cQueue_fill(QueueObj *queue, uint8_t length, uint8_t fill);
bool cQueue_ifFilled(QueueObj *queue, uint8_t length, uint8_t fill);

float Get_Target_Speed(uint8_t queue_len,float angle);
float Get_Target_Accel(uint8_t queue_len,float speed);

/* #任务层# ---------------------------------------------------------------------------------------------------------------------------------------*/
void VISION_GetInfo(void);
void VISION_ManualCtrl(void);
void VISION_AutoCtrl(void);
void VISION_BuffCtrl(void);
void VISION_Ctrl(void);

#endif
