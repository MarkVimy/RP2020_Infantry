#include "init.h"
#include "scheduler.h"

#define START_TASK_PRIO		1			// 任务优先级
#define START_STK_SIZE		128			// 任务堆栈大小
TaskHandle_t StartTask_Handler;			// 任务句柄

uint32_t now_time, delta_time;
static void my_hardware_init(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	Delay_init(168);// 1ms Systick
	LED_Init();		// LED初始化
	LASER_Init();	// 激光初始化
	REMOTE_Init();	// 遥控通信USART2初始化
	USART3_Init();	// 超声波通信USART3初始化
	RNG_Init();		// 随机数发生器初始化
	UART4_Init();	// 视觉通信UART4初始化
	UART5_Init();	// 裁判系统UART5初始化
	CAN1_Init();	// CAN1初始化
	CAN2_Init();	// CAN2初始化
	TIM1_Init();	// 弹仓定时器1初始化
	TIM3_Init();	// 摩擦轮定时器3初始化	
	USART1_Init();
	
	IMU_Init();		// 祖传IMU初始化
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
	/* 创建初始化任务 */
	xTaskCreate((TaskFunction_t		)start_task,						// 任务函数
							(const char*		)"start_task",			// 任务名称
							(uint16_t			)START_STK_SIZE,		// 任务堆栈大小
							(void*				)NULL,					// 传递给任务函数的参数
							(UBaseType_t		)START_TASK_PRIO,		// 任务优先级
							(TaskHandle_t*		)&StartTask_Handler);	// 任务句柄
	vTaskStartScheduler();	// 开启任务调度
}

void All_Init(void)
{
	my_hardware_init();
	my_system_init();
}

