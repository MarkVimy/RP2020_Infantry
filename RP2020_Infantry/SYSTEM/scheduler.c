#include "scheduler.h"
#include "semphr.h"

/* ## Global variables ## ------------------------------------------------------*/
Flag_t	Flag = {
	/* Remote */
	.Remote.RcLost = true,	// ��ʼ״̬Ϊң��ʧ��״̬
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
	.Remote.RcLost = 20,	// ��ʼ��Ϊң��ʧ��״̬
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
	.System.Reset = BM_RESET_GIMBAL | BM_RESET_FRIC,
	/* Chassis */
	.Chassis.CanReport = 0,
	/* Gimbal */
	.Gimbal.CanReport = 0,
	/* Revolver */
	.Revolver.CanReport = 0,
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
#define CHASSIS_TASK_PRIO					4		// �������ȼ�
#define CHASSIS_STK_SIZE					256		// �����ջ��С
TaskHandle_t ChassisTask_Handler;					// ������
void chassis_task(void *p_arg);
//--- Gimbal Task ---//
#define GIMBAL_TASK_PRIO					4		// �������ȼ�
#define GIMBAL_STK_SIZE						256		// �����ջ��С
TaskHandle_t GimbalTask_Handler;					// ������
void gimbal_task(void *p_arg);
//--- Revolver Task ---//
#define REVOLVER_TASK_PRIO					4		// �������ȼ�
#define REVOLVER_STK_SIZE					256		// �����ջ��С
TaskHandle_t RevolverTask_Handler;					// ������
void revolver_task(void *p_arg);
//--- Friction Task ---//
#define DUTY_TASK_PRIO						3		// �������ȼ�
#define DUTY_STK_SIZE						128		// �����ջ��С
TaskHandle_t DutyTask_Handler;						// ������
void duty_task(void *p_arg);
//--- Vision Task ---//
#define VISION_TASK_PRIO					3		// �������ȼ�
#define VISION_STK_SIZE						256		// �����ջ��С
TaskHandle_t VisionTask_Handler;					// ������
void vision_task(void *p_arg);
//--- Imu Task ---//
#define IMU_TASK_PRIO						1		// �������ȼ�
#define IMU_STK_SIZE						256		// �����ջ��С
TaskHandle_t ImuTask_Handler;						// ������
void imu_task(void *p_arg);

/* ## Semphore Manangement Table ## --------------------------------------------*/
SemaphoreHandle_t Semaphore_Usart1_Tx;

/* ## Task List ## -------------------------------------------------------------*/
/*!# Start Task #!*/
extern TaskHandle_t StartTask_Handler;				// ������(��init.c�ж���)
void start_task(void *pvParameters)
{
	taskENTER_CRITICAL();	// �����ٽ���
	
//	Semaphore_Usart1_Tx = xSemaphoreCreateBinary();
//	xSemaphoreGive(Semaphore_Usart1_Tx);
	
	/* ���������ʱ�������շ�����,can1,can2,��Ҫ��̫�ຯ�� */
	Timer_Send_Create();		
	
	/* ����ϵͳ״̬������ */
	xTaskCreate((TaskFunction_t		)system_state_task,						// ������
							(const char*		)"system_state_task",		// ��������
							(uint16_t			)SYSTEM_STATE_STK_SIZE,		// �����ջ��С
							(void*				)NULL,						// ���ݸ��������Ĳ���
							(UBaseType_t		)SYSTEM_STATE_TASK_PRIO,	// �������ȼ�
							(TaskHandle_t*		)&SystemStateTask_Handler);	// ������
	/* ������������ */
//	xTaskCreate((TaskFunction_t		)chassis_task,							// ������
//							(const char*		)"chassis_task",			// ��������
//							(uint16_t			)CHASSIS_STK_SIZE,			// �����ջ��С
//							(void*				)NULL,						// ���ݸ��������Ĳ���
//							(UBaseType_t		)CHASSIS_TASK_PRIO,			// �������ȼ�
//							(TaskHandle_t*		)&ChassisTask_Handler);		// ������
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
	/* ����IMU���� */
	xTaskCreate((TaskFunction_t		)imu_task,								// ������
							(const char*		)"imu_task",				// ��������
							(uint16_t			)IMU_STK_SIZE,				// �����ջ��С
							(void*				)NULL,						// ���ݸ��������Ĳ���
							(UBaseType_t		)IMU_TASK_PRIO,				// �������ȼ�
							(TaskHandle_t*		)&ImuTask_Handler);
							
	vTaskDelete(StartTask_Handler);	
	taskEXIT_CRITICAL();	// �˳��ٽ���
}

/**	!# 1 - System State Task #!
 *	@note 
 *		Loop time:	2ms
 */
void system_state_task(void *p_arg)	// ϵͳ״̬��
{
	while(1) {
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

		vTaskDelay(TIME_STAMP_2MS);
	}
}

/**	!# 2 - Chassis Task #!
 *	@note 
 *		Loop time:	2ms
 */


void chassis_task(void *p_arg)
{
	CHASSIS_Init();
	while(1) {
		switch(System.State)
		{												
			case SYSTEM_STATE_NORMAL:
//				SuperCAP_Ctrl();
				/* ���ϵ�ʱ�ȴ���̨���к���������*/
				if(BM_IfReset(BitMask.System.Reset, BM_RESET_GIMBAL)) {
					CHASSIS_Ctrl();
				}
				break;
			case SYSTEM_STATE_RCERR:
			case SYSTEM_STATE_RCLOST:
				CHASSIS_SelfProtect();
//				SuperCAP_SelfProtect();
				break;
		}
		
		vTaskDelay(TIME_STAMP_2MS);
	}
}

/**	!# 3 - Gimbal Task #!
 *	@note 
 *		Loop time:	2ms
 */
void gimbal_task(void *p_arg)
{
	GIMBAL_Init();
	while(1) {
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
		vTaskDelay(TIME_STAMP_2MS);		
	}
}

/**	!# 4 - Revolver Task #!
 *	@note 
 *		Loop time:	5ms
 */
void revolver_task(void *p_arg)
{
	while(1) {
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
		vTaskDelay(TIME_STAMP_1MS);
	}
}

/**	!# 5 - Duty Task #!
 *	@note 
 *		Loop time:	10ms
 */
extern float js_rotate_speed;
extern uint16_t chas_stuck_cnt;
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
//		RP_SendToPc(js_rotate_speed, 0, 0, chas_stuck_cnt, 0, 0);
		vTaskDelay(10);	// 10ms
	}
}

/**	!# 6 - Vision Task #!
 *	@note 
 *		Loop time:	10ms
 */
void vision_task(void *p_arg)
{
	VISION_Init();
	while(1) {
		VISION_Ctrl();
		vTaskDelay(10);	// 10ms
	}
}

/**	!# 7 - Imu Task #!
 *	@note 
 *		Loop time:	loop forever
 */
//	portTickType ulCurrentTime;
//	portTickType ulRespondTime;

void imu_task(void *p_arg)
{
	while(1) {
		/* IMU��ȡ���� */
		IMU_Task();		
//		RP_SendToPc2(BitMask.Chassis.CanReport, BitMask.Gimbal.CanReport, 0, 0, 0);
//		ANOC_SendToPc(js_pos_x*100, js_pos_y*100, js_agl_z*100, 	
//		RP_SendToPc(Mpu_Info.yaw, Mpu_Info.pitch, Mpu_Info.roll, Mpu_Info.rateYaw, Mpu_Info.ratePitch, Mpu_Info.rateRoll);
//		RP_SendToPc(js_pos_x, js_pos_y, js_agl_z, js_v_x, js_v_y, js_w_z);
//		RP_SendToPc2(Friction.SpeedFeedback, JUDGE_fGetBulletSpeed17(), Judge.PowerHeatData.shooter_heat0, Judge.PowerHeatData.chassis_power);
		
//		if(Judge.power_heat_update) {
//			Judge.power_heat_update = false;
//			RP_SendToPc(Judge.GameRobotPos.yaw, 0, 0, Judge.PowerHeatData.shooter_heat0, Judge.PowerHeatData.shooter_heat1, Judge.PowerHeatData.mobile_shooter_heat2);
//		}
//		

		
//		if(Judge.shoot_update) {
//			Judge.shoot_update = false;
//			RP_SendToPc2(Judge.ShootData.bullet_freq, Revolver.Shoot.real_ping, Judge.PowerHeatData.shooter_heat0, Friction.SpeedFeedback, Judge.ShootData.bullet_speed);
//		}
		
		vTaskDelay(1);	// 1ms��ע�ͺ�����˿�������
	}
}
