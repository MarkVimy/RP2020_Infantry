#include "init.h"
#include "scheduler.h"

#define START_TASK_PRIO		1			// �������ȼ�
#define START_STK_SIZE		128			// �����ջ��С
TaskHandle_t StartTask_Handler;			// ������

uint32_t now_time, delta_time;
static void my_hardware_init(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	Delay_init(168);// 1ms Systick
	LED_Init();		// LED��ʼ��
	LASER_Init();	// �����ʼ��
	REMOTE_Init();	// ң��ͨ��USART2��ʼ��
	USART3_Init();	// ������ͨ��USART3��ʼ��
	RNG_Init();		// �������������ʼ��
	UART4_Init();	// �Ӿ�ͨ��UART4��ʼ��
	UART5_Init();	// ����ϵͳUART5��ʼ��
	CAN1_Init();	// CAN1��ʼ��
	CAN2_Init();	// CAN2��ʼ��
	TIM1_Init();	// ���ֶ�ʱ��1��ʼ��
	TIM3_Init();	// Ħ���ֶ�ʱ��3��ʼ��	
	USART1_Init();
	
	IMU_Init();		// �洫IMU��ʼ��
	SuperCAP_Init();

//	while(1) {
////		now_time = millis();
//		IMU_Task();
////		delta_time = millis() - now_time;
////		ANOC_SendToPc1((int16_t)(Mpu_Info.yaw*100), (int16_t)(Mpu_Info.pitch*100), (int16_t)(Mpu_Info.roll*100));
////		ANOC_SendToPc((int16_t)Mpu_Info.yaw, (int16_t)Mpu_Info.pitch, (int16_t)Mpu_Info.roll, (int16_t)Mpu_Info.rateYaw, (int16_t)Mpu_Info.ratePitch, (int16_t)Mpu_Info.rateRoll, 0, 0, 0);
//		RP_SendToPc(Mpu_Info.yaw, Mpu_Info.pitch, delta_time, Mpu_Info.rateYaw, Mpu_Info.ratePitch, Mpu_Info.rateRoll);
////		RP_SendToPc(1, 2, 3, 4, 5, 6);
////		delay_ms(2);
//	}

//	JUDGE_SendToClient();
//	while(1) {	
////		JUDGE_SendToTeammate(0x0103);
////		JUDGE_SendToClient();
//		delay_ms(1000);
////		JUDGE_SendToTeammate(0x0103);
////		delay_ms(500);
//	}
}

static void my_system_init(void)
{
	/* ������ʼ������ */
	xTaskCreate((TaskFunction_t		)start_task,						// ������
							(const char*		)"start_task",			// ��������
							(uint16_t			)START_STK_SIZE,		// �����ջ��С
							(void*				)NULL,					// ���ݸ��������Ĳ���
							(UBaseType_t		)START_TASK_PRIO,		// �������ȼ�
							(TaskHandle_t*		)&StartTask_Handler);	// ������
	vTaskStartScheduler();	// �����������
}

void All_Init(void)
{
	my_hardware_init();
	my_system_init();
}

