#ifndef __FRICTION_H
#define __FRICTION_H

/* Includes ------------------------------------------------------------------*/
#include "sys.h"

/* Global macro --------------------------------------------------------------*/
/* Global TypeDef ------------------------------------------------------------*/
typedef enum {
	FRIC_SPEED_OFF = 0,			// �ر�
	FRIC_SPEED_LOW = 1,			// ����
	FRIC_SPEED_MID = 2,			// ����
	FRIC_SPEED_HIGH = 3,		// ����
	FRIC_SPEED_VERYHIGH = 4,	// ������
	FRIC_SPEED_SENTRY = 5, 		// ���ڱ�
	FRIC_SPEED_COUNT = 6,
}Friction_Speed_Names_t;

typedef enum {
	FRIC_STATE_OFF = 0,
	FRIC_STATE_ON = 1,
	FRIC_STATE_COUNT = 2,
}Friction_State_Names_t;

typedef struct {
	Friction_State_Names_t state;
	Friction_Speed_Names_t speedLevel;
	int16_t	speedTarget;
	int16_t	speedFeedback;
}Friction_Info_t;

/* ## Global Variables Prototypes ## -----------------------------------------*/
extern Friction_Info_t Friction;
extern uint16_t Friction_Pwm_Output[FRIC_SPEED_COUNT];
extern uint16_t Friction_Pwm_Speed[FRIC_SPEED_COUNT];

/* API functions Prototypes --------------------------------------------------*/
/* #������# ---------------------------------------------------------------------------------------------------------------------------------------*/
void FRICTION_ParamsInit(Friction_Info_t *fric);
void FRICTION_stop(Friction_Info_t *fric);
void FRICTION_pwmCalculate(Friction_Info_t *fric);
void FRICTION_selfCalibration(void);

/* #Ӧ�ò�# ---------------------------------------------------------------------------------------------------------------------------------------*/
void FRICTION_reset(void);
bool FRICTION_ifReady(void);

/* #�����# ---------------------------------------------------------------------------------------------------------------------------------------*/
void FRICTION_rcControlTask(void);
void FRICTION_keyControlTask(void);
void FRICTION_selfProtect(void);
void FRICTION_control(void);

#endif
