#include "scheduler.h"

/* ## Global variables ## ------------------------------------------------------*/
Flag_t	Flag = {
	/* Remote */
	.Remote.RcLost = false,
	.Remote.RcErr = false,
	/* Chassis */
	.Chassis.FLAG_angleTurnOk = 0,
	.Chassis.GoHome = false,
	.Chassis.ReloadBulletStart = true,	// false
	.Chassis.ReloadBulletJudge = false,
	/* Gimbal */
	.Gimbal.ResetOk = false,
	.Gimbal.AngleRecordStart = true,
	.Gimbal.FLAG_angleTurnOk = 0,
	/* Friction */
	.Friction.ResetOk = false,
};

Cnt_t	Cnt = {
	/* System */
	.System.Err = 0,
	.System.Reset = 2,	// # 注意这里的初始值要和BM_systemReset值的数量相同
	/* Remote */
	.Remote.RcLost = 0,
	.Remote.RcLostRecover = 0,
	/* Chassis */
	.Chassis.CNT_angleTurnOk = 0,
	.Chassis.ReloadBulletJudge = 0,
	/* Gimbal */
	.Gimbal.ResetOk = 0,
	.Gimbal.CNT_angleTurnOk = 0,
};

BitMask_t	BitMask = {
	/* System */
	.System.BM_Reset = BM_RESET_GIMBAL | BM_RESET_FRIC,
	/* Chassis */
	.Chassis.BM_rxReport = 0,
	/* Gimbal */
	.Gimbal.BM_rxReport = 0,
	/* Revolver */
	.Revolver.BM_rxReport = 0,
};

Mpu_Info_t Mpu_Info = {
	.pitch = 0,
	.roll = 0,
	.yaw = 0,
	.ratePitch = 0,
	.rateRoll = 0,
	.rateYaw = 0,
	.pitchOffset = 0,
	.rollOffset = 0,
	.yawOffset = 0,
	.ratePitchOffset = 0,
	.rateRollOffset = 0,
	.rateYawOffset = 0,
};

System_t System = {
	.State = SYSTEM_STATE_RCLOST,	// 默认系统状态 - 失联
	.RemoteMode = RC,				// 默认控制方式 - 遥控器
	.PidMode = MECH,				// 默认Pid模式 - 机械模式
	.Action = SYS_ACT_NORMAL		// 默认系统行为 - 常规行为
};

/* ## Task Manangement Table ## ------------------------------------------------*/
//--- Start Task ---//
// Defined in the init.c file => #define START_TASK_PRIO	1
//--- System State Task ---//
#define SYSTEM_STATE_TASK_PRIO				2		// 任务优先级
#define SYSTEM_STATE_STK_SIZE				128		// 任务堆栈大小
TaskHandle_t SystemStateTask_Handler;				// 任务句柄
void system_state_task(void *p_arg);
//--- Chassis Task ---//
#define CHASSIS_TASK_PRIO					3		// 任务优先级
#define CHASSIS_STK_SIZE					256		// 任务堆栈大小
TaskHandle_t ChassisTask_Handler;					// 任务句柄
void chassis_task(void *p_arg);
//--- Gimbal Task ---//
#define GIMBAL_TASK_PRIO					4		// 任务优先级
#define GIMBAL_STK_SIZE						256		// 任务堆栈大小
TaskHandle_t GimbalTask_Handler;					// 任务句柄
void gimbal_task(void *p_arg);
//--- Revolver Task ---//
#define REVOLVER_TASK_PRIO					5		// 任务优先级
#define REVOLVER_STK_SIZE					256		// 任务堆栈大小
TaskHandle_t RevolverTask_Handler;					// 任务句柄
void revolver_task(void *p_arg);
//--- Friction Task ---//
#define DUTY_TASK_PRIO						6		// 任务优先级
#define DUTY_STK_SIZE						128		// 任务堆栈大小
TaskHandle_t DutyTask_Handler;						// 任务句柄
void duty_task(void *p_arg);
//--- Vision Task ---//
#define VISION_TASK_PRIO					7		// 任务优先级
#define VISION_STK_SIZE						256		// 任务堆栈大小
TaskHandle_t VisionTask_Handler;					// 任务句柄
void vision_task(void *p_arg);

/* ## Semphore Manangement Table ## --------------------------------------------*/

/* ## Task List ## -------------------------------------------------------------*/
/*!# Start Task #!*/
extern TaskHandle_t StartTask_Handler;				// 任务句柄(在init.c中定义)
void start_task(void *pvParameters)
{
	taskENTER_CRITICAL();	// 进入临界区
	/* 创建系统状态机任务 */
	xTaskCreate((TaskFunction_t		)system_state_task,						// 任务函数
							(const char*		)"system_state_task",		// 任务名称
							(uint16_t			)SYSTEM_STATE_STK_SIZE,		// 任务堆栈大小
							(void*				)NULL,						// 传递给任务函数的参数
							(UBaseType_t		)SYSTEM_STATE_TASK_PRIO,	// 任务优先级
							(TaskHandle_t*		)&SystemStateTask_Handler);	// 任务句柄
	/* 创建底盘任务 */
	xTaskCreate((TaskFunction_t		)chassis_task,							// 任务函数
							(const char*		)"chassis_task",			// 任务名称
							(uint16_t			)CHASSIS_STK_SIZE,			// 任务堆栈大小
							(void*				)NULL,						// 传递给任务函数的参数
							(UBaseType_t		)CHASSIS_TASK_PRIO,			// 任务优先级
							(TaskHandle_t*		)&ChassisTask_Handler);		// 任务句柄
	/* 创建云台任务 */
	xTaskCreate((TaskFunction_t		)gimbal_task,							// 任务函数
							(const char*		)"gimbal_task",				// 任务名称
							(uint16_t			)GIMBAL_STK_SIZE,			// 任务堆栈大小
							(void*				)NULL,						// 传递给任务函数的参数
							(UBaseType_t		)GIMBAL_TASK_PRIO,			// 任务优先级
							(TaskHandle_t*		)&GimbalTask_Handler);		// 任务句柄							
	/* 创建拨盘电机任务 */
	xTaskCreate((TaskFunction_t		)revolver_task,							// 任务函数
							(const char*		)"revovler_task",			// 任务名称
							(uint16_t			)REVOLVER_STK_SIZE,			// 任务堆栈大小
							(void*				)NULL,						// 传递给任务函数的参数
							(UBaseType_t		)REVOLVER_TASK_PRIO,		// 任务优先级
							(TaskHandle_t*		)&RevolverTask_Handler);	// 任务句柄							
	/* 创建常规任务 */
	xTaskCreate((TaskFunction_t		)duty_task,								// 任务函数
							(const char*		)"duty_task",				// 任务名称
							(uint16_t			)DUTY_STK_SIZE,				// 任务堆栈大小
							(void*				)NULL,						// 传递给任务函数的参数
							(UBaseType_t		)DUTY_TASK_PRIO,			// 任务优先级
							(TaskHandle_t*		)&DutyTask_Handler);		// 任务句柄
	/* 创建视觉任务 */
	xTaskCreate((TaskFunction_t		)vision_task,							// 任务函数
							(const char*		)"vision_task",				// 任务名称
							(uint16_t			)VISION_STK_SIZE,			// 任务堆栈大小
							(void*				)NULL,						// 传递给任务函数的参数
							(UBaseType_t		)VISION_TASK_PRIO,			// 任务优先级
							(TaskHandle_t*		)&VisionTask_Handler);		// 任务句柄
							
	vTaskDelete(StartTask_Handler);	
	taskEXIT_CRITICAL();	// 退出临界区
}

/**	!# 1 - System State Task #!
 *	@note 
 *		Loop time:	2ms
 */
void system_state_task(void *p_arg)	// 系统状态机
{
	portTickType ulCurrentTime;
	while(1) {
		ulCurrentTime = xTaskGetTickCount();//当前系统时间

		/* IMU读取任务 */
		IMU_Task();
		
		/* 遥控任务 */
		REMOTE_Ctrl();
		
		/* 系统状态灯 */
		if(System.State == SYSTEM_STATE_NORMAL){
			LED_ALL_OFF();
			LED_GREEN_ON();
		}
		else {
			LED_ALL_ON();
		}

		vTaskDelayUntil(&ulCurrentTime, TIME_STAMP_2MS);//绝对延时
	}
}

/**	!# 2 - Chassis Task #!
 *	@note 
 *		Loop time:	2ms
 */
void chassis_task(void *p_arg)
{
	portTickType ulCurrentTime;
	CHASSIS_Init();
	while(1) {
		ulCurrentTime = xTaskGetTickCount();//当前系统时间
		switch(System.State)
		{												
			case SYSTEM_STATE_NORMAL:
				/* 刚上电时等待云台归中后才允许控制*/
				if(BM_IfReset(BitMask.System.BM_Reset, BM_RESET_GIMBAL)) {
					CHASSIS_Ctrl();
				}
				break;
			case SYSTEM_STATE_RCERR:
			case SYSTEM_STATE_RCLOST:
				CHASSIS_SelfProtect();
				break;
		}
		vTaskDelayUntil(&ulCurrentTime, TIME_STAMP_2MS);//绝对延时
	}
}

/**	!# 3 - Gimbal Task #!
 *	@note 
 *		Loop time:	2ms
 */
void gimbal_task(void *p_arg)
{
	portTickType ulCurrentTime;
	GIMBAL_Init();
	while(1) {
		ulCurrentTime = xTaskGetTickCount();//当前系统时间
		switch(System.State)
		{
			case SYSTEM_STATE_NORMAL:
				GIMBAL_Ctrl();
				break;
			case SYSTEM_STATE_RCERR:
			case SYSTEM_STATE_RCLOST:
				GIMBAL_SelfProtect();
				break;
		}
		vTaskDelayUntil(&ulCurrentTime, TIME_STAMP_2MS);//绝对延时
	}
}

/**	!# 4 - Revolver Task #!
 *	@note 
 *		Loop time:	5ms
 */
void revolver_task(void *p_arg)
{
	portTickType ulCurrentTime;
	while(1) {
		ulCurrentTime = xTaskGetTickCount();//当前系统时间
		switch(System.State)
		{
			case SYSTEM_STATE_NORMAL:
				REVOLVER_Ctrl();
				break;
			case SYSTEM_STATE_RCERR:
			case SYSTEM_STATE_RCLOST:
				REVOLVER_SelfProtect();
				break;
		}
		vTaskDelayUntil(&ulCurrentTime, TIME_STAMP_1MS);//绝对延时
	}
}

/**	!# 5 - Friction Task #!
 *	@note 
 *		Loop time:	10ms
 */
void duty_task(void *p_arg)
{	
	while(1) {		
		switch(System.State) 
		{
			case SYSTEM_STATE_NORMAL:
			{
				FRICTION_Ctrl();
				MAGZINE_Ctrl();
				break;
			}
			case SYSTEM_STATE_RCERR:
			case SYSTEM_STATE_RCLOST:
			{
				FRICTION_SelfProtect();
				MAGZINE_SelfProtect();
				break;
			}
		}
		vTaskDelay(10);	// 10ms
	}
}

/**	!# 6 - Vision Task #!
 *	@note 
 *		Loop time:	100ms
 */
void vision_task(void *p_arg)
{
	VISION_Init();
	while(1) {
		VISION_Ctrl();
		vTaskDelay(10);	// 10ms
	}
}
