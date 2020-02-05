#include "init.h"
#include "scheduler.h"

#define START_TASK_PRIO		1			// 任务优先级
#define START_STK_SIZE		128			// 任务堆栈大小
TaskHandle_t StartTask_Handler;			// 任务句柄

/**
 *	@note	这里pass_flag初始化的时候一定要置1，不然陀螺仪数据漂得一批！！
 *			（想了差不多1个半月云台漂移的解决办法，真的有毒！！告诫后人）
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
	LED_init();		// LED初始化
	LASER_init();	// 激光初始化
	REMOTE_init();	// 遥控通信USART2初始化
	USART3_init();	// 超声波通信USART3初始化
	UART4_init();	// 视觉通信UART4初始化
	UART5_init();	// 裁判系统UART5初始化
	CAN1_init();	// CAN1初始化
	CAN2_init();	// CAN2初始化
	TIM1_init();	// 弹仓定时器1初始化
	TIM3_init();	// 摩擦轮定时器3初始化
	
	RNG_init();	// 不使用随机数的时候注释掉
	
	//祖传MPU初始化
	MPU_Init();
	while (mpu_dmp_init( )) 
	{
			ulCurrentTime = xTaskGetTickCount();

			if (ulCurrentTime >= ulLoopTime)  
			{
				  /* 100MS延时 */
					ulLoopTime = ulCurrentTime + TIME_STAMP_100MS;
				
				  /* 300ms屏蔽自检 */
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
	
	/* 创建软件定时器数据收发任务,can1,can2,不要放太多函数 */
	Timer_Send_Create();
}

static void my_system_init(void)
{
	/* 创建初始化任务 */
	xTaskCreate((TaskFunction_t		)start_task,					// 任务函数
							(const char*		)"start_task",					// 任务名称
							(uint16_t			)START_STK_SIZE,					// 任务堆栈大小
							(void*				)NULL,										// 传递给任务函数的参数
							(UBaseType_t		)START_TASK_PRIO,				// 任务优先级
							(TaskHandle_t*		)&StartTask_Handler);	// 任务句柄
	vTaskStartScheduler();	// 开启任务调度
}

void All_Init(void)
{
	my_hardware_init();
	my_system_init();
}

