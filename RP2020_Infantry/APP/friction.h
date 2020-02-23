#ifndef __FRICTION_H
#define __FRICTION_H

/* Includes ------------------------------------------------------------------*/
#include "sys.h"

/* Global macro --------------------------------------------------------------*/
/* Global TypeDef ------------------------------------------------------------*/
typedef enum {
	FRIC_SPEED_OFF = 0,			// 关闭
	FRIC_SPEED_LOW = 1,			// 低速
	FRIC_SPEED_MID = 2,			// 中速
	FRIC_SPEED_HIGH = 3,		// 高速
	FRIC_SPEED_VERYHIGH = 4,	// 超高速
	FRIC_SPEED_SENTRY = 5, 		// 打哨兵
	FRIC_SPEED_COUNT = 6,
}Friction_Speed_Names_t;

typedef enum {
	FRIC_STATE_OFF = 0,
	FRIC_STATE_ON = 1,
	FRIC_STATE_COUNT = 2,
}Friction_State_Names_t;

typedef struct {
	Remote_Mode_Names_t		RemoteMode;
	Friction_State_Names_t 	State;
	uint16_t				Speed;
	Friction_Speed_Names_t 	SpeedLevel;
	int16_t					SpeedTarget;
	int16_t					SpeedFeedback;	
}Friction_Info_t;

/* ## Global Variables Prototypes ## -----------------------------------------*/
extern Friction_Info_t Friction;
extern uint16_t Friction_Pwm_Output[FRIC_SPEED_COUNT];
extern uint16_t Friction_Pwm_Speed[FRIC_SPEED_COUNT];

/* API functions Prototypes --------------------------------------------------*/
/* #驱动层# ---------------------------------------------------------------------------------------------------------------------------------------*/
void FRICTION_ParamsInit(Friction_Info_t *fric);
void FRICTION_Stop(Friction_Info_t *fric);
void FRICTION_PwmCalc(Friction_Info_t *fric);
void FRICTION_SelfCalibrate(void);

/* #应用层# ---------------------------------------------------------------------------------------------------------------------------------------*/
void FRICTION_Reset(void);
bool FRICTION_IfOpen(void);
bool FRICTION_IfReady(void);

/* #任务层# ---------------------------------------------------------------------------------------------------------------------------------------*/
void FRICTION_RcCtrlTask(void);
void FRICTION_KeyCtrlTask(void);
void FRICTION_SelfProtect(void);
void FRICTION_Ctrl(void);

#endif
