#ifndef __TASK_CHASSIS_H
#define __TASK_CHASSIS_H

/* Includes ------------------------------------------------------------------*/
#include "sys.h"

#include "my_app.h"
/* Global macro --------------------------------------------------------------*/
#define CHASSIS_MODE_COUNT	PID_MODE_COUNT

#define CHASSIS_SPEED_IOUT_MAX		6000	// 底盘电机速度环pid积分输出限幅
#define CHASSIS_SPEED_INTEGRATE_MAX	18000	// 底盘电机速度环pid积分限幅
#define CHASSIS_ANGLE_IOUT_MAX		3000	// 底盘电机位置环pid积分输出限幅
#define CHASSIS_ANGLE_INTEGRATE_MAX	6000	// 底盘电机位置环pid积分限幅
#define CHASSIS_MAX_SPEED			8000	// 底盘电机转子最大转速
#define CHASSIS_PID_OUT_MAX			9000	// 底盘电机pid输出最大值(功率限制)

#define WARNING_REMAIN_POWER		60
#define CHASSIS_MAX_CURRENT_LIMIT	36000	// 单个电机最大输出9000

/* Global TypeDef ------------------------------------------------------------*/
typedef enum {
	LEFT_FRON_201 = 0,  // 左前
	RIGH_FRON_202 = 1,  // 右前
	LEFT_BACK_203 = 2,  // 左后
	RIGH_BACK_204 = 3,  // 右后
	CHASSIS_MOTOR_COUNT = 4
}Chassis_Motor_Names;

typedef struct {
	PID_Object_t	Speed;
	PID_Object_t	Angle;
	float			Out;
}Chassis_PID_t;

typedef struct {
	PID_Object_t	Angle;
	float			AngleRampTarget;
	float			AngleRampFeedback;
	float			Out;
}Chassis_Z_PID_t;

typedef struct {
	float max_limit;	// 最大限制
	float cur_limit;	// 当前限制
}Chassis_Power_t;

typedef enum {
	CHAS_MODE_NORMAL = 0,	// 正常模式
	CHAS_MODE_BUFF = 1,		// 打符模式
	CHAS_MODE_RELOAD_BULLET = 2,	// 底盘低速补弹模式
	CHAS_MODE_SZUPUP = 3,	// SZU爬坡模式
}Chassis_Mode_Names_t;

typedef enum {
	CHAS_LOGIC_NORMAL = 0,
	CHAS_LOGIC_REVERT = 1,
}Chassis_Logic_Names_t;

typedef struct {
	Remote_Mode_Names_t		RemoteMode;
	Pid_Mode_Names_t		PidMode;
	Chassis_Mode_Names_t	Mode;
	Chassis_Power_t			Power;
	Chassis_Logic_Names_t 	Logic;
	uint8_t					TopGyro;
	uint8_t					Twist;
}Chassis_Info_t;

typedef struct {
	/* Ctrl */
	Chassis_PID_t	Pid[CHASSIS_MOTOR_COUNT];
	/* Z Angle Ctrl */
	Chassis_Z_PID_t	ZPid;
	/* Locate Ctrl */
	PID_Object_t	LocatePid[2];
	/* Info */
	Remote_Mode_Names_t		RemoteMode;
	Pid_Mode_Names_t		PidMode;
	Chassis_Mode_Names_t	Mode;
	Chassis_Logic_Names_t 	Logic;
	uint8_t					TopGyro;
	uint8_t					Twist;
}Chassis_Handler_t;

/* ## Global Variables Prototypes ## -----------------------------------------*/
extern Chassis_PID_t Chassis_PID[CHASSIS_MOTOR_COUNT];
extern Chassis_Z_PID_t Chassis_Z_PID;
extern Chassis_Info_t Chassis;
/* API functions Prototypes --------------------------------------------------*/
/* #驱动层# ---------------------------------------------------------------------------------------------------------------------------------------*/
void CHASSIS_UpdateMotorSpeed(Chassis_PID_t *pid, Chassis_Motor_Names MOTORx);
void CHASSIS_UpdateMotorAngle(Chassis_PID_t *pid, Chassis_Motor_Names MOTORx);
void CHASSIS_UpdateMotorCurrent(Chassis_PID_t *pid, Chassis_Motor_Names MOTORx);

void CHASSIS_PID_ParamsInit(Chassis_PID_t *pid, uint8_t motor_cnt);
void CHASSIS_Z_PID_ParamsInit(PID_Object_t *pid);
void CHASSIS_Stop(Chassis_PID_t *pid);
void CHASSIS_Speed_PidCalc(Chassis_PID_t *pid, Chassis_Motor_Names MOTORx);
void CHASSIS_Angle_PidCalc(Chassis_PID_t *pid, Chassis_Motor_Names MOTORx);
float CHASSIS_Z_Angle_PidCalc(PID_Object_t *pid, float kp);
void CHASSIS_Angle_ClearSum(Chassis_PID_t *pid);
void CHASSIS_PidOut(Chassis_PID_t *pid);

/* #信息层# ---------------------------------------------------------------------------------------------------------------------------------------*/
void CHASSIS_GetInfo(void);
void CHASSIS_SetMode(Chassis_Mode_Names_t mode);
void CHASSIS_LogicRevert(void);
Chassis_Mode_Names_t CHASSIS_GetMode(void);
Chassis_Logic_Names_t CHASSIS_GetLogic(void);
int16_t CHASSIS_GetMiddleAngle(void);
int16_t CHASSIS_GetMiddleAngleOffset(void);
bool CHASSIS_IfNormalMode(void);
bool CHASSIS_IfTopGyroOpen(void);
bool CHASSIS_IfTwistOpen(void);
bool CHASSIS_IfLogicRevert(void);
	
/* #应用层# ---------------------------------------------------------------------------------------------------------------------------------------*/
void CHASSIS_Init(void);
float CHASSIS_TwistTargetCalc(int16_t maxTarget, int16_t rampTarget);
//void CHASSIS_PowerLimit(Chassis_Power_t *power, Chassis_PID_t *pid, Judge_Info_t *judge_info);
void CHASSIS_OmniSpeedCalc(void);
float CHASSIS_MECH_YawBoundaryProc(Chassis_Z_PID_t *pid, float delta_target);

/* #任务层# ---------------------------------------------------------------------------------------------------------------------------------------*/
void CHASSIS_GetInfo(void);
void CHASSIS_PidCtrlTask(void);
void CHASSIS_RcCtrlTask(void);
void CHASSIS_KeyCtrlTask(void);
void CHASSIS_SelfProtect(void);	// 底盘失控保护
void CHASSIS_Ctrl(void);	// 底盘控制任务

#endif
