/**
 * @file        Task_Timing_Report.c
 * @author      Copy From 19RP_Infantry
 * @Version     V1.0
 * @date        10-October-2019
 * @brief       This file includes the Timing Report external functions 
 */
 
/* Includes ------------------------------------------------------------------*/
#include "Task_Timing_Report.h"

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* ## Global variables ## ----------------------------------------------------*/
QueueHandle_t CAN1_Queue;	// CAN1��Ϣ���о��
QueueHandle_t CAN2_Queue;	// CAN2��Ϣ���о��

TimerHandle_t CAN1_Timer_Handle;	//���ڶ�ʱ�����					
TimerHandle_t CAN2_Timer_Handle;	//���ڶ�ʱ�����

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* API functions -------------------------------------------------------------*/
/**
  * @brief  �����ʱ�ص�����
  * @param  void
  * @retval void
  * @attention can���н��õȴ�������delay
  */
void CAN1_Timer_Callback( TimerHandle_t xTimer )
{
	CanTxMsg	SendCanTxMsg;
	
	while(xQueueReceive(CAN1_Queue, &SendCanTxMsg, 0))	// ���ն�����Ϣ����ֹ�ȴ�������
	{
//        do
//		{
//			//CAN��������
//			if(CAN1->ESR)
//			{
//				CAN1->MCR |= 0x02;
//				CAN1->MCR &= 0xFD;
//			}
//		}while(!(CAN1->TSR & 0x1C000000));
		CAN_Transmit(CAN1, &SendCanTxMsg);	// ����Ŀ��ֵ
	}
}

/**
  * @brief  �����ʱ�ص�����
  * @param  void
  * @retval void
  * @attention can���н��õȴ�������delay
  */
void CAN2_Timer_Callback( TimerHandle_t xTimer )
{
	CanTxMsg	SendCanTxMsg;
	while(xQueueReceive(CAN2_Queue, &SendCanTxMsg, 0))	// ���ն�����Ϣ����ֹ�ȴ�������
	{
//        do
//		{
//			//CAN��������
//			if(CAN2->ESR)
//			{
//				CAN2->MCR |= 0x02;
//				CAN2->MCR &= 0xFD;
//			}
//		}while(!(CAN2->TSR & 0x1C000000));

		CAN_Transmit(CAN2, &SendCanTxMsg);	// ����Ŀ��ֵ
	}
}

/**
  * @brief  ����ϵͳ�����ʱ������(��ʱ����CAN��Ϣ)
  * @param  void
  * @retval void
  */
void Timer_Send_Create(void)
{
	taskENTER_CRITICAL();	// �����ٽ���
	
	/*--------------------------���ݽ���--------------------------*/	
	
	/* ����CAN1���ն���
		 CAN1���յ��ı��Ĵ���ڴ˶����� */
	CAN1_Queue = xQueueCreate( 128, sizeof(CanTxMsg) );	// �����Ա���128��CanTxMsg
	
	/* ����CAN2���ն���
		 CAN2���յ��ı��Ĵ���ڴ˶����� */
	CAN2_Queue = xQueueCreate( 128, sizeof(CanTxMsg) );	// �����Ա���128��CanTxMsg
	
	/* ����CAN1���Ͷ�ʱ�� */
	CAN1_Timer_Handle = xTimerCreate((const char*)"CAN1_Timer",
									(TickType_t)TIME_STAMP_2MS,	// 2ms
									(UBaseType_t)pdTRUE,		// ����ִ��
									(void*)1,					// ���1
									(TimerCallbackFunction_t)CAN1_Timer_Callback);	// �ص�����
																
	/* �����ɹ�
		 ����CAN1��ʱ�������ҽ��ܿ���һ�Σ����������������򲻻ᷢ���� */
	if( CAN1_Timer_Handle != NULL )
	{
		xTimerStart(CAN1_Timer_Handle, 0);	// ���ȴ�
	}

	/* ����CAN2���Ͷ�ʱ�� */
	CAN2_Timer_Handle = xTimerCreate((const char*)"CAN2_Timer",
									(TickType_t)TIME_STAMP_1MS,	// 1ms
									(UBaseType_t)pdTRUE,		// ����ִ��
									(void*)2,					// ���2
									(TimerCallbackFunction_t)CAN2_Timer_Callback);	// �ص�����
																
	/* �����ɹ�
		 ����CAN2��ʱ�������ҽ��ܿ���һ�Σ����������������򲻻ᷢ���� */
	if( CAN2_Timer_Handle != NULL )
	{
		xTimerStart(CAN2_Timer_Handle, 0);	// ���ȴ�
	}
	
	taskEXIT_CRITICAL();	// �˳��ٽ���
}
