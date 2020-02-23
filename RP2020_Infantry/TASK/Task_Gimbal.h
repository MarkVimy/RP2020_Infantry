#ifndef __TASK_GIMBAL_H
#define __TASK_GIMBAL_H

/* Includes ------------------------------------------------------------------*/
#include "sys.h"
#include "my_app.h"

/* Global macro --------------------------------------------------------------*/
#define GIMBAL_MODE_COUNT		PID_MODE_COUNT

#define YAW_DIR	(-1)

#define GIMBAL_MECH_YAW_MID_ANGLE			GIMBAL_TOP_YAW_MID_ANGLE	//(4104)	// 中间值

#define GIMBAL_TOP_YAW_MID_ANGLE			(6325)	// 底盘正向逻辑机械中值	6325
#define GIMBAL_REVERT_YAW_MID_ANGLE			(2229)	// 底盘反向逻辑机械中值

#define GIMBAL_MECH_PITCH_UP_ANGLE			(3410 + 150)	// 实测最小可达3312(不发力)
#define GIMBAL_MECH_PITCH_MID_ANGLE			(4048)				// 中间值
#define GIMBAL_MECH_PITCH_DOWN_ANGLE		(4435 - 100)	// 实测最小可达4382(不发力)

#define GIMBAL_AUTO_LOCK_SENTRY_ANGLE		(GIMBAL_MECH_PITCH_UP_ANGLE + 150)

#define GIMBAL_GYRO_PITCH_UP_ANGLE			GIMBAL_MECH_PITCH_UP_ANGLE 		//((-28+2) * GIMBAL_GYRO_ANGLE_ZOOM_INDEX)
#define GIMBAL_GYRO_PITCH_MID_ANGLE			GIMBAL_MECH_PITCH_MID_ANGLE 	//( +1 * GIMBAL_GYRO_ANGLE_ZOOM_INDEX)
#define GIMBAL_GYRO_PITCH_DOWN_ANGLE		GIMBAL_MECH_PITCH_DOWN_ANGLE 	//((+13-2) * GIMBAL_GYRO_ANGLE_ZOOM_INDEX)

#define GIMBAL_GYRO_ANGLE_ZOOM_INDEX		(20.0f)	// IMU角度的缩放系数(方便滤波和调参)

#define GIMBAL_RAMP_BEGIN_YAW				2
#define GIMBAL_RAMP_BEGIN_PITCH				1

/* Global TypeDef ------------------------------------------------------------*/
/**
 *	@brief	云台电机选择
 */
typedef enum {
	YAW_205 = 0, 	// Yaw
	PITCH_206 = 1,	// Pitch
	GIMBAL_MOTOR_COUNT = 2
}Gimbal_Motor_Names_t;

/**
 *	@brief	云台模式
 */
typedef enum {
	GIMBAL_MODE_NORMAL 		= 0, // 正常模式
	GIMBAL_MODE_AUTO   		= 1, // 自瞄模式
	GIMBAL_MODE_BIG_BUFF	= 2, // 大符模式
	GIMBAL_MODE_SMALL_BUFF	= 3, // 小符模式
	GIMBAL_MODE_RELOAD_BULLET = 4,	// 补弹模式
}Gimbal_Mode_t;	

/**
 *	@brief	云台PID
 */
typedef struct {
	PID_Object_t	Speed;
	PID_Object_t	Angle;
	float			AngleRampTarget;
	float			AngleRampFeedback;
	float			Out;
}Gimbal_PID_t;

/**
 *	@brief	云台状态
 */
typedef struct {
	Gimbal_Mode_t 		mode;
	Remote_Mode_Names_t	remote_mode;
	Pid_Mode_Names_t	pid_mode;
}Gimbal_State_t;

/**
 *	@brief	速度解算信息
 */
typedef struct  //视觉目标速度测量
{
  int 	delay_cnt;//计算相邻两帧目标不变持续时间,用来判断速度是否为0
  int 	freq;
  int 	last_time;//上次受到目标角度的时间
  float last_position;//上个目标角度
  float speed;//速度
  float last_speed;//上次速度
  float processed_speed;//速度计算结果
}Speed_Calculate_t;

/**
 *	@brief	云台自瞄PID信息
 */
typedef struct {
	float kp;
	float erro;
	float target;
}Gimbal_Vision_Info_t;

/**
 *	@brief	云台自瞄PID信息
 */
typedef struct {
	PID_Object_t	Yaw;
	PID_Object_t	Pitch;
	uint32_t		Time[TIME_STATE_COUNT];
	int8_t			FLAG_first_into_auto;
}Gimbal_Vision_Auto_t;

/**
 *	@brief	云台打符PID信息
 */
typedef struct {
	PID_Object_t	Yaw;
	PID_Object_t	Pitch;
	uint32_t		Time[TIME_STATE_COUNT];
	int8_t			FLAG_first_into_buff;
}Gimbal_Vision_Buff_t;

/**
 *	@brief	云台综合信息结构体
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
/* #驱动层# ---------------------------------------------------------------------------------------------------------------------------------------*/
void GIMBAL_PidParamsSwitch(Gimbal_PID_t pid[GIMBAL_MODE_COUNT][GIMBAL_MOTOR_COUNT], Gimbal_Info_t *gimbal);
void GIMBAL_PidParamsInit(Gimbal_PID_t *pid, uint8_t motor_cnt);
void GIMBAL_KalmanCreate(void);
void GIMBAL_Stop(Gimbal_PID_t *pid);
void GIMBAL_GetImuInfo(Gimbal_PID_t pid[GIMBAL_MODE_COUNT][GIMBAL_MOTOR_COUNT]);
void GIMBAL_GYRO_CalcAverageOffset(Mpu_Info_t mpuInfo);
void GIMBAL_Speed_PidCalc(Gimbal_PID_t *pid, Gimbal_Motor_Names_t MOTORx);
void GIMBAL_Angle_PidCalc(Gimbal_PID_t *pid, Gimbal_Motor_Names_t MOTORx);
void GIMBAL_PidOut(Gimbal_PID_t *pid);

/* #信息层# ---------------------------------------------------------------------------------------------------------------------------------------*/
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
	
/* #应用层# ---------------------------------------------------------------------------------------------------------------------------------------*/
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

/* #任务层# ---------------------------------------------------------------------------------------------------------------------------------------*/
void GIMBAL_GetInfo(void);
void GIMBAL_PidCtrlTask(void);
void GIMBAL_visionControlTask(void);
void GIMBAL_RcCtrlTask(void);
void GIMBAL_KeyCtrlTask(void);
void GIMBAL_SelfProtect(void);
void GIMBAL_Ctrl(void);

#endif
