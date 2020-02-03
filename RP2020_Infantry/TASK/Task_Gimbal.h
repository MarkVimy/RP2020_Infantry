#ifndef __TASK_GIMBAL_H
#define __TASK_GIMBAL_H

/* Includes ------------------------------------------------------------------*/
#include "sys.h"
#include "my_app.h"

/* Global macro --------------------------------------------------------------*/
#define GIMBAL_MODE_COUNT		PID_MODE_COUNT

#define YAW_DIR	(-1)

//#define GIMBAL_MECH_YAW_ANGLE_LEFT_LIMIT		(1998 + 200)	// ʵ����С�ɴ�1896(������) 1892(����)
#define GIMBAL_MECH_YAW_MID_ANGLE			GIMBAL_TOP_YAW_MID_ANGLE	//(4104)	// �м�ֵ
//#define GIMBAL_MECH_YAW_ANGLE_RIGHT_LIMIT		(6235 - 200)	// ʵ�����ɴ�6180(������)	6184(����)

#define GIMBAL_TOP_YAW_MID_ANGLE			(6325)	// ���������߼���е��ֵ	6325
#define GIMBAL_REVERT_YAW_MID_ANGLE		(2229)	// ���̷����߼���е��ֵ

#define GIMBAL_MECH_PITCH_UP_ANGLE		(3410 + 150)	// ʵ����С�ɴ�3312(������)
#define GIMBAL_MECH_PITCH_MID_ANGLE		(4048)				// �м�ֵ
#define GIMBAL_MECH_PITCH_DOWN_ANGLE	(4435 - 100)	// ʵ����С�ɴ�4382(������)

#define GIMBAL_AUTO_LOCK_SENTRY_ANGLE			(GIMBAL_MECH_PITCH_UP_ANGLE + 150)

#define GIMBAL_GYRO_PITCH_UP_ANGLE		GIMBAL_MECH_PITCH_UP_ANGLE 		//((-28+2) * GIMBAL_GYRO_ANGLE_ZOOM_INDEX)
#define GIMBAL_GYRO_PITCH_MID_ANGLE		GIMBAL_MECH_PITCH_MID_ANGLE 	//( +1 * GIMBAL_GYRO_ANGLE_ZOOM_INDEX)
#define GIMBAL_GYRO_PITCH_DOWN_ANGLE	GIMBAL_MECH_PITCH_DOWN_ANGLE 	//((+13-2) * GIMBAL_GYRO_ANGLE_ZOOM_INDEX)

#define GIMBAL_GYRO_ANGLE_ZOOM_INDEX				(20.0f)	// IMU�Ƕȵ�����ϵ��(�����˲��͵���)

#define GIMBAL_RAMP_BEGIN_YAW						2
#define GIMBAL_RAMP_BEGIN_PITCH					1

/* Global TypeDef ------------------------------------------------------------*/
/**
 *	@brief	��̨Pidģʽѡ��
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
	float					AngleRampTarget;
	float					AngleRampFeedback;
	float					Out;
}Gimbal_PID_t;

/**
 *	@brief	�Ӿ�������Ϣ
 */
typedef struct {
	Gimbal_Mode_t mode;
	uint8_t		  	FLAG_topGyroOpen;
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
	Gimbal_State_t 				State;
	Gimbal_Vision_Auto_t 	Auto;
	Gimbal_Vision_Buff_t	Buff;
}Gimbal_Info_t;

/* ## Global Variables Prototypes ## -----------------------------------------*/
extern Gimbal_PID_t  Gimbal_PID[GIMBAL_MODE_COUNT][GIMBAL_MOTOR_COUNT];
extern Gimbal_Info_t Gimbal;

/* API functions Prototypes --------------------------------------------------*/
/* #������# ---------------------------------------------------------------------------------------------------------------------------------------*/
void GIMBAL_pidParamsSwitch(Gimbal_PID_t pid[GIMBAL_MODE_COUNT][GIMBAL_MOTOR_COUNT], Gimbal_Info_t *gimbal);
void GIMBAL_pidParamsInit(Gimbal_PID_t *pid, uint8_t motor_cnt);
void GIMBAL_kalmanCreate(void);
void GIMBAL_stop(Gimbal_PID_t *pid);
void GIMBAL_IMU_recordFeedback(Gimbal_PID_t pid[GIMBAL_MODE_COUNT][GIMBAL_MOTOR_COUNT]);
void GIMBAL_GYRO_calAverageOffset(Mpu_Info_t mpuInfo);
void GIMBAL_Speed_pidCalculate(Gimbal_PID_t *pid, Gimbal_Motor_Names_t MOTORx);
void GIMBAL_Angle_pidCalculate(Gimbal_PID_t *pid, Gimbal_Motor_Names_t MOTORx);
void GIMBAL_pidOut(Gimbal_PID_t *pid);

/* #��Ϣ��# ---------------------------------------------------------------------------------------------------------------------------------------*/
void GIMBAL_setMode(Gimbal_Mode_t mode);
Gimbal_Mode_t GIMBAL_getMode(void);
float GIMBAL_getMiddleAngle(void);
bool GIMBAL_ifMechMode(void);
bool GIMBAL_ifGyroMode(void);
bool GIMBAL_ifNormalMode(void);
bool GIMBAL_ifTopGyroMode(void);
bool GIMBAL_ifAutoMode(void);
bool GIMBAL_ifBuffMode(void);
bool GIMBAL_ifAimSentry(void);
bool GIMBAL_BUFF_chaseReady(void);
bool GIMBAL_ifAutoMobiPre(void);
bool GIMBAL_AUTO_chaseReady(void);
float GIMBAL_getTopGyroAngleOffset(void);
void GIMBAL_calPredictInfo(void);
void GIMBAL_getInfo(void);
	
/* #Ӧ�ò�# ---------------------------------------------------------------------------------------------------------------------------------------*/
void GIMBAL_init(void);
void GIMBAL_reset(void);
float GIMBAL_MECH_yawTargetBoundaryProcess(Gimbal_PID_t *pid, float delta_target);
float GIMBAL_GYRO_yawTargetBoundaryProcess(Gimbal_PID_t *pid, float delta_target);
//void REMOTE_setGimbalAngle(RC_Ctl_t *remoteInfo);
//void KEY_setGimbalAngle(RC_Ctl_t *remoteInfo);
//void KEY_setQuickPickUp(RC_Ctl_t *remoteInfo);
void GIMBAL_rcMech_To_rcGyro(void);
void GIMBAL_rcGyro_To_rcMech(void);
void GIMBAL_rcMech_To_keyGyro(void);
void GIMBAL_keyGyro_To_rcMech(void);
void GIMBAL_keyGyro_To_keyMech(void);
void GIMBAL_keyMech_To_keyGyro(void);
void GIMBAL_pidParamsSwitch(Gimbal_PID_t pid[GIMBAL_MODE_COUNT][GIMBAL_MOTOR_COUNT], Gimbal_Info_t *gimbal);
void GIMBAL_normalControl(void);
void GIMBAL_autoControl(void);
void GIMBAL_buffControl(void);

/* #�����# ---------------------------------------------------------------------------------------------------------------------------------------*/
void GIMBAL_pidControlTask(void);
void GIMBAL_visionControlTask(void);
void GIMBAL_rcControlTask(void);
void GIMBAL_keyControlTask(void);
void GIMBAL_selfProtect(void);
void GIMBAL_control(void);

#endif
