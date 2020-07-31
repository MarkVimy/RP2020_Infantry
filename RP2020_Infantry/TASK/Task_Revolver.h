#ifndef __TASK_REVOLVER_H
#define __TASK_REVOLVER_H

/* Includes ------------------------------------------------------------------*/
#include "sys.h"
#include "my_app.h"

/* Global macro --------------------------------------------------------------*/
#define REVOLVER_ID					0x207
#define REVOLVER_BM_CAN_REPORT		BM_CAN_REPORT_207

#define REVOLVER_SPEED_RATIO		2160		// 1秒转一圈(= 60rpm*36(减速比) => 1rps)
#define REVOLVER_SPEED_GRID			12.0f		// 拨盘12格
#define SPEED_AN_BULLET_PER_SECOND	(REVOLVER_SPEED_RATIO/REVOLVER_SPEED_GRID)	// 1发/s 的期望速度
#define ANGLE_AN_BULLET				24576.0f	// 8192*36/12 = 24576.0f
#define ANGLE_AN_BULLET_RAMP		ANGLE_AN_BULLET/20	//(ANGLE_AN_BULLET/20)	// 最快20ms转一格(一定不能超过50ms)

#define HEAT_AN_BULLET				10	// 打一颗弹丸的热量增加(RM2020固定为10)


/* Global TypeDef ------------------------------------------------------------*/
typedef enum {
	REVO_STATE_OFF = 0,
	REVO_STATE_ON = 1,
	REVO_STATE_COUNT =2
}Revolver_State_Names_t;

typedef enum {
	REVO_POSI_MODE  = 0,	// 点射模式(位置环)
	REVO_SPEED_MODE = 1,	// 连发模式(速度环)
	REVO_MODE_COUNT = 2
}Revolver_Mode_Names_t;

typedef enum {
	SHOOT_NORMAL = 0,		// 常规模式(不发弹/点射/连发)
//	SHOOT_SINGLE = 1,		// 单发模式
//	SHOOT_TRIPLE = 2,		// 三连发
//	SHOOT_HIGH_F_LOW_S = 3,	// 高射频低射速(推家)
	SHOOT_BUFF	 = 1,		// 打符模式
	SHOOT_AUTO	 = 2,		// 自瞄模式
	SHOOT_BAN	 = 3,		// 禁止打弹模式
	REVO_ACTION_COUNT = 4,
}Revolver_Action_t;

typedef struct {
	PID_Object_t	Speed;
	PID_Object_t	Angle;
	float			AngleRampBuffer;
	float			Out;
}Revolver_PID_t;

typedef struct {
	uint32_t lost_time;
	uint32_t lost_time_boundary;
	uint32_t lock_time;
	uint32_t lock_time_boundary;
	uint16_t change_armor_delay;
	uint16_t change_armor_delay_boundary;
	uint32_t respond_interval;
	bool	 change_armor;
	bool	 change_armor_4;
	bool	 fire;
}Revolver_Buff_Info_t;

typedef struct {
	uint16_t 	total_count;	// 总发弹量
	uint32_t 	real_time;		// 单点实时时间(用来测试射击延迟)
	uint16_t	real_ping;		// 单点实时响应(射击延迟)
	uint16_t	freq;			// 射频
	uint16_t	interval;		// 发射时间间隔(与射频相关)
	uint16_t	num;			// 需要发射的子弹数(发射指令数)
	uint16_t	num_buffer; 	// 允许发射的子弹数
	uint16_t	heat_cooling_rate; // 17mm枪口每秒冷却值
	uint16_t	heat_real;		// 17mm枪口热量
	uint16_t	heat_limit;		// 17mm枪口热量限制
	uint16_t 	stuck_count;	// 卡弹计数	
	uint8_t		suicide_mode;	// 自杀模式(枪口热量超限)
}Revolver_Shoot_Info_t;

typedef struct {
	Remote_Mode_Names_t		RemoteMode;
	Revolver_State_Names_t 	State;
	Revolver_Mode_Names_t 	PidMode; 
	Revolver_Action_t		Action;
	Revolver_Shoot_Info_t	Shoot;
	Revolver_Buff_Info_t	Buff;
}Revolver_Info_t;

/* ## Global Variables Prototypes ## -----------------------------------------*/
extern Revolver_PID_t Revolver_PID;
extern Revolver_Info_t Revolver;

/* API functions Prototypes --------------------------------------------------*/
/* #驱动层# ---------------------------------------------------------------------------------------------------------------------------------------*/
void REVOLVER_PidParamsInit(Revolver_PID_t *pid);
void REVOLVER_Stop(Revolver_PID_t *pid);
void REVOLVER_Speed_PidCalc(Revolver_PID_t *pid);
void REVOLVER_Angle_PidCalc(Revolver_PID_t *pid);
void REVOLVER_PidOut(Revolver_PID_t *pid);

/* #信息层# ---------------------------------------------------------------------------------------------------------------------------------------*/
void REVOLVER_SetAction(Revolver_Action_t action);
void REVOLVER_AddShootCount(void);

/* info -> out */
uint32_t REVOLVER_GetRealShootTime(void);
uint16_t REVOLVER_CalcRealShootPing(uint32_t feedback_time);

/* in <- info */
void REVOLVER_GetSysInfo(System_t *sys, Revolver_Info_t *revo);
void REVOLVER_GetJudgeInfo(Judge_Info_t *judge, Revolver_Info_t *revo);
//void REVOLVER_GetRemoteInfo(System_t *sys, RC_Ctl_t *remote, Revolver_Info_t *revo);

/* #应用层# ---------------------------------------------------------------------------------------------------------------------------------------*/
uint8_t REVOLVER_HeatLimit(Revolver_Info_t *revo, uint8_t target_num);
void REVOLVER_NormalCtrl(Revolver_Info_t *revo);
void REVOLVER_SingleShootCtrl(Revolver_Info_t *revo);
void REVOLVER_TripleShootCtrl(Revolver_Info_t *revo);
void REVOLVER_BuffShootCtrl(Revolver_Info_t *revo);

/* #任务层# ---------------------------------------------------------------------------------------------------------------------------------------*/
void REVOLVER_GetInfo(void);
void REVOLVER_RcCtrlTask(void);
void REVOLVER_KeyCtrlTask(void);
void REVOLVER_SelfProtect(void);
void REVOLVER_Ctrl(void);

#endif
