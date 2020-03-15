#include "init.h"
#include "scheduler.h"

#define START_TASK_PRIO		1			// �������ȼ�
#define START_STK_SIZE		128			// �����ջ��С
TaskHandle_t StartTask_Handler;			// ������

static void my_hardware_init(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	Delay_init(168);// 1ms Systick
	LED_Init();		// LED��ʼ��
	LASER_Init();	// �����ʼ��
	REMOTE_Init();	// ң��ͨ��USART2��ʼ��
	USART3_Init();	// ������ͨ��USART3��ʼ��
	
	//ULTRA_Config();
		
	UART4_Init();	// �Ӿ�ͨ��UART4��ʼ��
	UART5_Init();	// ����ϵͳUART5��ʼ��
	CAN1_Init();	// CAN1��ʼ��
	CAN2_Init();	// CAN2��ʼ��
	TIM1_Init();	// ���ֶ�ʱ��1��ʼ��
	TIM3_Init();	// Ħ���ֶ�ʱ��3��ʼ��
	
	IMU_Init();		// �洫IMU��ʼ��
	
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

