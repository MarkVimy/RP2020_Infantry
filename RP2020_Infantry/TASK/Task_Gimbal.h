#ifndef __TASK_GIMBAL_H
#define __TASK_GIMBAL_H

/* Includes ------------------------------------------------------------------*/
#include "sys.h"
#include "my_app.h"

/* Global macro --------------------------------------------------------------*/
#define GIMBAL_MODE_COUNT		PID_MODE_COUNT

#define YAW_DIR	(-1)

#define GIMBAL_MECH_YAW_MID_ANGLE			GIMBAL_TOP_YAW_MID_ANGLE	//(4104)	// �м�ֵ

#define GIMBAL_TOP_YAW_MID_ANGLE			(6325)	// ���������߼���е��ֵ	6325
#define GIMBAL_REVERT_YAW_MID_ANGLE			(2229)	// ���̷����߼���е��ֵ

#define GIMBAL_MECH_PITCH_UP_ANGLE			(3410 + 150)	// ʵ����С�ɴ�3312(������)
#define GIMBAL_MECH_PITCH_MID_ANGLE			(4048)				// �м�ֵ
#define GIMBAL_MECH_PITCH_DOWN_ANGLE		(4435 - 100)	// ʵ����С�ɴ�4382(������)

#define GIMBAL_AUTO_LOCK_SENTRY_ANGLE		(GIMBAL_MECH_PITCH_UP_ANGLE + 150)

#define GIMBAL_GYRO_PITCH_UP_ANGLE			GIMBAL_MECH_PITCH_UP_ANGLE 		//((-28+2) * GIMBAL_GYRO_ANGLE_ZOOM_INDEX)
#define GIMBAL_GYRO_PITCH_MID_ANGLE			GIMBAL_MECH_PITCH_MID_ANGLE 	//( +1 * GIMBAL_GYRO_ANGLE_ZOOM_INDEX)
#define GIMBAL_GYRO_PITCH_DOWN_ANGLE		GIMBAL_MECH_PITCH_DOWN_ANGLE 	//((+13-2) * GIMBAL_GYRO_ANGLE_ZOOM_INDEX)

#define GIMBAL_GYRO_ANGLE_ZOOM_INDEX		(20.0f)	// IMU�Ƕȵ�����ϵ��(�����˲��͵���)

#define GIMBAL_RAMP_BEGIN_YAW				2
#define GIMBAL_RAMP_BEGIN_PITCH				1

/* Global TypeDef ------------------------------------------------------------*/
/**
 *	@brief	��̨���ѡ��
 */
typedef enum {
	YAW_205 = 0, 	// Yaw
	PITCH_206 = 1,	// Pitch
	GIMBAL_MOTOR_COUNT = 2
}Gimbal_Motor_Names_t;

/**
 *	@brief	��̨ģʽ
 */
typedef enum {
	GIMBAL_MODE_NORMAL 		= 0, // ����ģʽ
	GIMBAL_MODE_AUTO   		= 1, // ����ģʽ
	GIMBAL_MODE_BIG_BUFF	= 2, // ���ģʽ
	GIMBAL_MODE_SMALL_BUFF	= 3, // С��ģʽ
	GIMBAL_MODE_RELOAD_BULLET = 4,	// ����ģʽ
}Gimbal_Mode_t;	

/**
 *	@brief	��̨PID
 */
typedef struct {
	PID_Object_t	Speed;
	PID_Object_t	Angle;
	float			AngleRampTarget;
	float			AngleRampFeedback;
	float			Out;
}Gimbal_PID_t;

/**
 *	@brief	��̨״̬
 */
typedef struct {
	Gimbal_Mode_t 		mode;
	Remote_Mode_Names_t	remote_mode;
	Pid_Mode_Names_t	pid_mode;
}Gimbal_State_t;

/**
 *	@brief	�ٶȽ�����Ϣ
 */
typedef struct  //�Ӿ�Ŀ���ٶȲ���
{
  int 	delay_cnt;//����������֡Ŀ�겻�����ʱ��,�����ж��ٶ��Ƿ�Ϊ0
  int 	freq;
  int 	last_time;//�ϴ��ܵ�Ŀ��Ƕȵ�ʱ��
  float last_position;//�ϸ�Ŀ��Ƕ�
  float speed;//�ٶ�
  float last_speed;//�ϴ��ٶ�
  float processed_speed;//�ٶȼ�����
}Speed_Calculate_t;

/**
 *	@brief	��̨����PID��Ϣ
 */
typedef struct {
	float kp;
	float erro;
	float target;
}Gimbal_Vision_Info_t;

/**
 *	@brief	��̨����PID��Ϣ
 */
typedef struct {
	PID_Object_t	Yaw;
	PID_Object_t	Pitch;
	uint32_t		Time[TIME_STATE_COUNT];
	int8_t			FLAG_first_into_auto;
}Gimbal_Vision_Auto_t;

/**
 *	@brief	��̨���PID��Ϣ
 */
typedef struct {
	PID_Object_t	Yaw;
	PID_Object_t	Pitch;
	uint32_t		Time[TIME_STATE_COUNT];
	int8_t			FLAG_first_into_buff;
}Gimbal_Vision_Buff_t;

/**
 *	@brief	��̨�ۺ���Ϣ�ṹ��
 */
typedef struct {
	Gimbal_State_t 			State;
	Gimbal_Vision_Auto_t 	Auto;
	Gimbal_Vision_Buff_t	Buff;
}Gimbal_Info_t;

/* ## Global Variables Prototypes ## -----------------------------------------*/
extern Gimbal_PID_t  Gimbal_PID[GIMBAL_MODE_COUNT][GIMBAL_MOTOR_COUNT];
extern Gimbal_Info_t Gimbal;

/* API functions Prototypes --------------------------------------------------*/
/* #������# ---------------------------------------------------------------------------------------------------------------------------------------*/
void GIMBAL_PidParamsSwitch(Gimbal_PID_t pid[GIMBAL_MODE_COUNT][GIMBAL_MOTOR_COUNT], Gimbal_Info_t *gimbal);
void GIMBAL_PidParamsInit(Gimbal_PID_t *pid, uint8_t motor_cnt);
void GIMBAL_KalmanCreate(void);
void GIMBAL_Stop(Gimbal_PID_t *pid);
void GIMBAL_GetImuInfo(Gimbal_PID_t pid[GIMBAL_MODE_COUNT][GIMBAL_MOTOR_COUNT]);
void GIMBAL_GYRO_CalcAverageOffset(Mpu_Info_t mpuInfo);
void GIMBAL_Speed_PidCalc(Gimbal_PID_t *pid, Gimbal_Motor_Names_t MOTORx);
void GIMBAL_Angle_PidCalc(Gimbal_PID_t *pid, Gimbal_Motor_Names_t MOTORx);
void GIMBAL_PidOut(Gimbal_PID_t *pid);

/* #��Ϣ��# ---------------------------------------------------------------------------------------------------------------------------------------*/
void GIMBAL_SetMode(Gimbal_Mode_t mode);
Gimbal_Mode_t GIMBAL_GetMode(void);
float GIMBAL_GetMiddleAngle(void);
bool GIMBAL_IfMechMode(void);
bool GIMBAL_IfGyroMode(void);
bool GIMBAL_IfNormalMode(void);
bool GIMBAL_ifTopGyroMode(void);
bool GIMBAL_IfAutoMode(void);
bool GIMBAL_IfBuffMode(void);
bool GIMBAL_IfAimSentry(void);
bool GIMBAL_IfReloadBullet(void);
bool GIMBAL_BUFF_IfChaseReady(void);
bool GIMBAL_IfAutoMobiPre(void);
bool GIMBAL_AUTO_IfChaseReady(void);
float GIMBAL_GetTopGyroAngleOffset(void);
void GIMBAL_CalPredictInfo(void);
	
/* #Ӧ�ò�# ---------------------------------------------------------------------------------------------------------------------------------------*/
void GIMBAL_Init(void);
void GIMBAL_Reset(void);
float GIMBAL_MECH_YawBoundaryProc(Gimbal_PID_t *pid, float delta_target);
float GIMBAL_GYRO_YawBoundaryProc(Gimbal_PID_t *pid, float delta_target);
//void REMOTE_SetGimbalAngle(RC_Ctl_t *remote);
//void KEY_SetGimbalAngle(RC_Ctl_t *remote);
//void KEY_SetQuickPickUp(RC_Ctl_t *remote);
void GIMBAL_rcMech_To_rcGyro(void);
void GIMBAL_rcGyro_To_rcMech(void);
void GIMBAL_rcMech_To_keyGyro(void);
void GIMBAL_keyGyro_To_rcMech(void);
void GIMBAL_keyGyro_To_keyMech(void);
void GIMBAL_keyMech_To_keyGyro(void);
void GIMBAL_PidParamsSwitch(Gimbal_PID_t pid[GIMBAL_MODE_COUNT][GIMBAL_MOTOR_COUNT], Gimbal_Info_t *gimbal);
void GIMBAL_NormalCtrl(void);
void GIMBAL_AutoCtrl(void);
void GIMBAL_BuffCtrl(void);

/* #�����# ---------------------------------------------------------------------------------------------------------------------------------------*/
void GIMBAL_GetInfo(void);
void GIMBAL_PidCtrlTask(void);
void GIMBAL_visionControlTask(void);
void GIMBAL_RcCtrlTask(void);
void GIMBAL_KeyCtrlTask(void);
void GIMBAL_SelfProtect(void);
void GIMBAL_Ctrl(void);

#endif
