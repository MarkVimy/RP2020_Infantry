#include "scheduler.h"

/* ## Global variables ## ------------------------------------------------------*/
Flag_t	Flag = {
	/* Remote */
	.Remote.FLAG_rcLost = 0,
	.Remote.FLAG_mode = RC,
	/* Chassis */
	.Chassis.FLAG_angleTurnOk = 0,
	.Chassis.FLAG_goHome = 0,
	/* Gimbal */
	.Gimbal.FLAG_pidMode = MECH,	// Ĭ���ǻ�еģʽ
	.Gimbal.FLAG_resetOK = false,
	.Gimbal.FLAG_angleRecordStart = 0,
	.Gimbal.FLAG_angleTurnOk = 0,
	/* Friction */
	.Friction.FLAG_resetOK = false,
};

Cnt_t	Cnt = {
	/* System */
	.System.CNT_err = 0,
	.System.CNT_reset = 2,	// # ע������ĳ�ʼֵҪ��BM_systemResetֵ��������ͬ
	/* Remote */
	.Remote.CNT_rcLost = 0,
	.Remote.CNT_rcLostRecover = 0,
	/* Chassis */
	.Chassis.CNT_angleTurnOk = 0,
	/* Gimbal */
	.Gimbal.CNT_resetOK = 0,
	.Gimbal.CNT_enableChangePidMode = 0,
	.Gimbal.CNT_angleTurnOk = 0,
};

BitMask_t	BitMask = {
	/* System */
	.System.BM_reset = BM_RESET_GIMBAL | BM_RESET_FRIC,
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

/* # ϵͳ״̬ # */
System_State_t System_State = SYSTEM_STATE_RCLOST;	// �ϵ��ʱ��Ϊң�ض�ʧ״̬

/* ## Task Manangement Table ## ------------------------------------------------*/
//--- Start Task ---//
// Defined in the init.c file => #define START_TASK_PRIO	1
//--- System State Task ---//
#define SYSTEM_STATE_TASK_PRIO				2		// �������ȼ�
#define SYSTEM_STATE_STK_SIZE				128		// �����ջ��С
TaskHandle_t SystemStateTask_Handler;			// ������
void system_state_task(void *p_arg);
//--- Chassis Task ---//
#define CHASSIS_TASK_PRIO							3		// �������ȼ�
#define CHASSIS_STK_SIZE						256		// �����ջ��С
TaskHandle_t ChassisTask_Handler;					// ������
void chassis_task(void *p_arg);
//--- Gimbal Task ---//
#define GIMBAL_TASK_PRIO							4		// �������ȼ�
#define GIMBAL_STK_SIZE							256		// �����ջ��С
TaskHandle_t GimbalTask_Handler;					// ������
void gimbal_task(void *p_arg);
//--- Revolver Task ---//
#define REVOLVER_TASK_PRIO						5		// �������ȼ�
#define REVOLVER_STK_SIZE						256		// �����ջ��С
TaskHandle_t RevolverTask_Handler;				// ������
void revolver_task(void *p_arg);
//--- Friction Task ---//
#define DUTY_TASK_PRIO								6		// �������ȼ�
#define DUTY_STK_SIZE								128		// �����ջ��С
TaskHandle_t DutyTask_Handler;						// ������
void duty_task(void *p_arg);
//--- Remote Task ---//
#define REMOTE_TASK_PRIO							7		// �������ȼ�
#define REMOTE_STK_SIZE							256		// �����ջ��С
TaskHandle_t RemoteTask_Handler;					// ������
void remote_task(void *p_arg);
//--- Vision Task ---//
#define VISION_TASK_PRIO							8		// �������ȼ�
#define VISION_STK_SIZE							256		// �����ջ��С
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
	xTaskCreate((TaskFunction_t		)duty_task,							// ������
							(const char*		)"duty_task",			// ��������
							(uint16_t			)DUTY_STK_SIZE,			// �����ջ��С
							(void*				)NULL,						// ���ݸ��������Ĳ���
							(UBaseType_t		)DUTY_TASK_PRIO,		// �������ȼ�
							(TaskHandle_t*		)&DutyTask_Handler);	// ������
	/* ����ң������ */
	xTaskCreate((TaskFunction_t		)remote_task,							// ������
							(const char*		)"remote_task",				// ��������
							(uint16_t			)REMOTE_STK_SIZE,			// �����ջ��С
							(void*				)NULL,						// ���ݸ��������Ĳ���
							(UBaseType_t		)REMOTE_TASK_PRIO,			// �������ȼ�
							(TaskHandle_t*		)&RemoteTask_Handler);		// ������
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
	portTickType currentTime;
	while(1) {
		currentTime = xTaskGetTickCount();//��ǰϵͳʱ��
		GIMBAL_IMU_recordFeedback(Gimbal_PID);
		/* ϵͳ״̬�� */
		switch(System_State) {
			case SYSTEM_STATE_NORMAL:	LED_ALL_OFF();LED_GREEN_ON();break;
			case SYSTEM_STATE_RCERR: 
			case SYSTEM_STATE_RCLOST: 	LED_ALL_ON();break;
		}
		vTaskDelayUntil(&currentTime, TIME_STAMP_2MS);//������ʱ
	}
}

/**	!# 2 - Chassis Task #!
 *	@note 
 *		Loop time:	2ms
 */
void chassis_task(void *p_arg)
{
	portTickType currentTime;
	
	CHASSIS_init();
	while(1) {
		currentTime = xTaskGetTickCount();//��ǰϵͳʱ��
		switch(System_State)
		{												
			case SYSTEM_STATE_NORMAL:					
				if(BM_ifReset(BitMask.System.BM_reset, BM_RESET_GIMBAL)) {
					CHASSIS_control();
				}
				break;
			case SYSTEM_STATE_RCERR:
			case SYSTEM_STATE_RCLOST:
				CHASSIS_selfProtect();
				break;
		}
		vTaskDelayUntil(&currentTime, TIME_STAMP_2MS);//������ʱ
	}
}

/**	!# 3 - Gimbal Task #!
 *	@note 
 *		Loop time:	2ms
 */
void gimbal_task(void *p_arg)
{
	portTickType currentTime;
	
	GIMBAL_init();
	while(1) {
		currentTime = xTaskGetTickCount();//��ǰϵͳʱ��
		//GIMBAL_IMU_recordFeedback(Gimbal_PID);
		switch(System_State)
		{
			case SYSTEM_STATE_NORMAL:
				GIMBAL_control();
				break;
			case SYSTEM_STATE_RCERR:
			case SYSTEM_STATE_RCLOST:
				GIMBAL_selfProtect();
				break;
		}
		vTaskDelayUntil(&currentTime, TIME_STAMP_2MS);//������ʱ
	}
}

/**	!# 4 - Revolver Task #!
 *	@note 
 *		Loop time:	5ms
 */
void revolver_task(void *p_arg)
{
	portTickType currentTime;
	
	while(1) {
		currentTime = xTaskGetTickCount();//��ǰϵͳʱ��
		switch(System_State)
		{
			case SYSTEM_STATE_NORMAL:
				REVOLVER_control();
				break;
			case SYSTEM_STATE_RCERR:
			case SYSTEM_STATE_RCLOST:
				REVOLVER_selfProtect();
				break;
		}
		vTaskDelayUntil(&currentTime, TIME_STAMP_1MS);//������ʱ
	}
}

/**	!# 5 - Friction Task #!
 *	@note 
 *		Loop time:	10ms
 */
void duty_task(void *p_arg)
{	
	portTickType currentTime;
	
	while(1) {		
		currentTime = xTaskGetTickCount();
		switch(System_State) 
		{
			case SYSTEM_STATE_NORMAL:
			{
				FRICTION_control();
				MAGZINE_control();
				break;
			}
			case SYSTEM_STATE_RCERR:
			case SYSTEM_STATE_RCLOST:
			{
				FRICTION_selfProtect();
				MAGZINE_selfProtect();
				/* #���Ե�ʱ����ʱ�������� */
				ULTRA_control();
				break;
			}
		}
		vTaskDelay(10);	// 10ms
	}
}

/**	!# 6 - Remote Task #!
 *	@note 
 *		Loop time:	20ms
 */
void remote_task(void *p_arg)
{		
	while(1) {
		REMOTE_control();
		vTaskDelay(20);	// 20ms
	}
}

/**	!# 7 - Vision Task #!
 *	@note 
 *		Loop time:	100ms
 */
void vision_task(void *p_arg)
{
	VISION_init();
	while(1) {
		VISION_control();
		vTaskDelay(10);	// 10ms
	}
}
