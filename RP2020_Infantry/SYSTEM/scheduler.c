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
	.System.Reset = 2,	// # ע������ĳ�ʼֵҪ��BM_systemResetֵ��������ͬ
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
	.State = SYSTEM_STATE_RCLOST,	// Ĭ��ϵͳ״̬ - ʧ��
	.RemoteMode = RC,				// Ĭ�Ͽ��Ʒ�ʽ - ң����
	.PidMode = MECH,				// Ĭ��Pidģʽ - ��еģʽ
	.Action = SYS_ACT_NORMAL		// Ĭ��ϵͳ��Ϊ - ������Ϊ
};

/* ## Task Manangement Table ## ------------------------------------------------*/
//--- Start Task ---//
// Defined in the init.c file => #define START_TASK_PRIO	1
//--- System State Task ---//
#define SYSTEM_STATE_TASK_PRIO				2		// �������ȼ�
#define SYSTEM_STATE_STK_SIZE				128		// �����ջ��С
TaskHandle_t SystemStateTask_Handler;				// ������
void system_state_task(void *p_arg);
//--- Chassis Task ---//
#define CHASSIS_TASK_PRIO					3		// �������ȼ�
#define CHASSIS_STK_SIZE					256		// �����ջ��С
TaskHandle_t ChassisTask_Handler;					// ������
void chassis_task(void *p_arg);
//--- Gimbal Task ---//
#define GIMBAL_TASK_PRIO					4		// �������ȼ�
#define GIMBAL_STK_SIZE						256		// �����ջ��С
TaskHandle_t GimbalTask_Handler;					// ������
void gimbal_task(void *p_arg);
//--- Revolver Task ---//
#define REVOLVER_TASK_PRIO					5		// �������ȼ�
#define REVOLVER_STK_SIZE					256		// �����ջ��С
TaskHandle_t RevolverTask_Handler;					// ������
void revolver_task(void *p_arg);
//--- Friction Task ---//
#define DUTY_TASK_PRIO						6		// �������ȼ�
#define DUTY_STK_SIZE						128		// �����ջ��С
TaskHandle_t DutyTask_Handler;						// ������
void duty_task(void *p_arg);
//--- Vision Task ---//
#define VISION_TASK_PRIO					7		// �������ȼ�
#define VISION_STK_SIZE						256		// �����ջ��С
TaskHandle_t VisionTask_Handler;					// ������
void vision_task(void *p_arg);

/* ## Semphore Manangement Table ## --------------------------------------------*/

/* ## Task List ## -------------------------------------------------------------*/
/*!# Start Task #!*/
extern TaskHandle_t StartTask_Handler;				// ������(��init.c�ж���)
void start_task(void *pvParameters)
{
	taskENTER_CRITICAL();	// �����ٽ���
	/* ����ϵͳ״̬������ */
	xTaskCreate((TaskFunction_t		)system_state_task,						// ������
							(const char*		)"system_state_task",		// ��������
							(uint16_t			)SYSTEM_STATE_STK_SIZE,		// �����ջ��С
							(void*				)NULL,						// ���ݸ��������Ĳ���
							(UBaseType_t		)SYSTEM_STATE_TASK_PRIO,	// �������ȼ�
							(TaskHandle_t*		)&SystemStateTask_Handler);	// ������
	/* ������������ */
	xTaskCreate((TaskFunction_t		)chassis_task,							// ������
							(const char*		)"chassis_task",			// ��������
							(uint16_t			)CHASSIS_STK_SIZE,			// �����ջ��С
							(void*				)NULL,						// ���ݸ��������Ĳ���
							(UBaseType_t		)CHASSIS_TASK_PRIO,			// �������ȼ�
							(TaskHandle_t*		)&ChassisTask_Handler);		// ������
	/* ������̨���� */
	xTaskCreate((TaskFunction_t		)gimbal_task,							// ������
							(const char*		)"gimbal_task",				// ��������
							(uint16_t			)GIMBAL_STK_SIZE,			// �����ջ��С
							(void*				)NULL,						// ���ݸ��������Ĳ���
							(UBaseType_t		)GIMBAL_TASK_PRIO,			// �������ȼ�
							(TaskHandle_t*		)&GimbalTask_Handler);		// ������							
	/* �������̵������ */
	xTaskCreate((TaskFunction_t		)revolver_task,							// ������
							(const char*		)"revovler_task",			// ��������
							(uint16_t			)REVOLVER_STK_SIZE,			// �����ջ��С
							(void*				)NULL,						// ���ݸ��������Ĳ���
							(UBaseType_t		)REVOLVER_TASK_PRIO,		// �������ȼ�
							(TaskHandle_t*		)&RevolverTask_Handler);	// ������							
	/* ������������ */
	xTaskCreate((TaskFunction_t		)duty_task,								// ������
							(const char*		)"duty_task",				// ��������
							(uint16_t			)DUTY_STK_SIZE,				// �����ջ��С
							(void*				)NULL,						// ���ݸ��������Ĳ���
							(UBaseType_t		)DUTY_TASK_PRIO,			// �������ȼ�
							(TaskHandle_t*		)&DutyTask_Handler);		// ������
	/* �����Ӿ����� */
	xTaskCreate((TaskFunction_t		)vision_task,							// ������
							(const char*		)"vision_task",				// ��������
							(uint16_t			)VISION_STK_SIZE,			// �����ջ��С
							(void*				)NULL,						// ���ݸ��������Ĳ���
							(UBaseType_t		)VISION_TASK_PRIO,			// �������ȼ�
							(TaskHandle_t*		)&VisionTask_Handler);		// ������
							
	vTaskDelete(StartTask_Handler);	
	taskEXIT_CRITICAL();	// �˳��ٽ���
}

/**	!# 1 - System State Task #!
 *	@note 
 *		Loop time:	2ms
 */
void system_state_task(void *p_arg)	// ϵͳ״̬��
{
	portTickType ulCurrentTime;
	while(1) {
		ulCurrentTime = xTaskGetTickCount();//��ǰϵͳʱ��

		/* IMU��ȡ���� */
		IMU_Task();
		
		/* ң������ */
		REMOTE_Ctrl();
		
		/* ϵͳ״̬�� */
		if(System.State == SYSTEM_STATE_NORMAL){
			LED_ALL_OFF();
			LED_GREEN_ON();
		}
		else {
			LED_ALL_ON();
		}

		vTaskDelayUntil(&ulCurrentTime, TIME_STAMP_2MS);//������ʱ
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
		ulCurrentTime = xTaskGetTickCount();//��ǰϵͳʱ��
		switch(System.State)
		{												
			case SYSTEM_STATE_NORMAL:
				/* ���ϵ�ʱ�ȴ���̨���к���������*/
				if(BM_IfReset(BitMask.System.BM_Reset, BM_RESET_GIMBAL)) {
					CHASSIS_Ctrl();
				}
				break;
			case SYSTEM_STATE_RCERR:
			case SYSTEM_STATE_RCLOST:
				CHASSIS_SelfProtect();
				break;
		}
		vTaskDelayUntil(&ulCurrentTime, TIME_STAMP_2MS);//������ʱ
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
		ulCurrentTime = xTaskGetTickCount();//��ǰϵͳʱ��
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
		vTaskDelayUntil(&ulCurrentTime, TIME_STAMP_2MS);//������ʱ
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
		ulCurrentTime = xTaskGetTickCount();//��ǰϵͳʱ��
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
		vTaskDelayUntil(&ulCurrentTime, TIME_STAMP_1MS);//������ʱ
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
