#include "init.h"
#include "scheduler.h"

#define START_TASK_PRIO		1			// �������ȼ�
#define START_STK_SIZE		128			// �����ջ��С
TaskHandle_t StartTask_Handler;			// ������

/**
 *	@note	����pass_flag��ʼ����ʱ��һ��Ҫ��1����Ȼ����������Ư��һ������
 *			�����˲��1��������̨Ư�ƵĽ���취������ж����������ˣ�
 */
bool pass_flag = 1;
static void my_hardware_init(void)
{
	static portTickType ulCurrentTime = 0;
	static portTickType ulLoopTime    = 0;
	static int16_t  	sTimeCnt      = 0;	
	RCC_ClocksTypeDef clocks;
	RCC_GetClocksFreq(&clocks);	
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

//	delay_init(clocks.SYSCLK_Frequency/1000000);	// 1ms Systick
	Delay_init(168);
	LED_init();		// LED��ʼ��
	LASER_init();	// �����ʼ��
	REMOTE_init();	// ң��ͨ��USART2��ʼ��
	USART3_init();	// ������ͨ��USART3��ʼ��
	UART4_init();	// �Ӿ�ͨ��UART4��ʼ��
	UART5_init();	// ����ϵͳUART5��ʼ��
	CAN1_init();	// CAN1��ʼ��
	CAN2_init();	// CAN2��ʼ��
	TIM1_init();	// ���ֶ�ʱ��1��ʼ��
	TIM3_init();	// Ħ���ֶ�ʱ��3��ʼ��
	
	//�洫MPU��ʼ��
	MPU_Init();
	while (mpu_dmp_init( )) 
	{
			ulCurrentTime = xTaskGetTickCount();

			if (ulCurrentTime >= ulLoopTime)  
			{
				  /* 100MS��ʱ */
					ulLoopTime = ulCurrentTime + TIME_STAMP_100MS;
				
				  /* 300ms�����Լ� */
					if (sTimeCnt >= 2) 
					{
							pass_flag = 0;
							sTimeCnt  = 0;//10;
					}
					else
					{
							sTimeCnt++;
					}
			}
	}	
	
	/* ���������ʱ�������շ�����,can1,can2,��Ҫ��̫�ຯ�� */
	Timer_Send_Create();
}

static void my_system_init(void)
{
	/* ������ʼ������ */
	xTaskCreate((TaskFunction_t		)start_task,					// ������
							(const char*		)"start_task",					// ��������
							(uint16_t			)START_STK_SIZE,					// �����ջ��С
							(void*				)NULL,										// ���ݸ��������Ĳ���
							(UBaseType_t		)START_TASK_PRIO,				// �������ȼ�
							(TaskHandle_t*		)&StartTask_Handler);	// ������
	vTaskStartScheduler();	// �����������
}

void All_Init(void)
{
	my_hardware_init();
	my_system_init();
}

