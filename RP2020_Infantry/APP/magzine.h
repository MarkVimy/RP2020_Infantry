#ifndef __MAGZINE_H
#define __MAGZINE_H

/* Includes ------------------------------------------------------------------*/
#include "sys.h"

#include "my_app.h"
/* Global macro --------------------------------------------------------------*/
#define MAGZ_ANGLE_CLOSE	89
#define MAGZ_ANGLE_OPEN		52

/* Global TypeDef ------------------------------------------------------------*/
typedef enum {
	MAGZ_STATE_CLOSE = 0,
	MAGZ_STATE_OPEN = 1,
	MAGZ_STATE_COUNT = 2,
}Magzine_State_Names_t;

typedef enum {
	MAGZ_MODE_FREE	= 0,// ���ɿ���
	MAGZ_MODE_NC = 1,	// ����ģʽ(Normal Close)
	MAGZ_MODE_NO = 2,	// ����ģʽ(Normal Open)
}Magzine_Mode_Names_t;

typedef struct {
	int16_t	target;
	int16_t	feedback;
}Magzine_Pwm_t;

typedef struct {
	/* Info */
	Magzine_State_Names_t	State;
	Magzine_Mode_Names_t	Mode;
	Remote_Mode_Names_t		RemoteMode;
	/* Ctrl */
	Magzine_Pwm_t			Angle;
}Magzine_Handler_t;

/* ## Global Variables Prototypes ## -----------------------------------------*/
extern Magzine_Handler_t Magzine;

/* API functions Prototypes --------------------------------------------------*/
/* #������# ---------------------------------------------------------------------------------------------------------------------------------------*/
void MAGZINE_ParamsInit(Magzine_Handler_t *magz);
void MAGZINE_Stop(Magzine_Handler_t *magz);
void MAGZINE_Close(Magzine_Handler_t *magz);
void MAGZINE_Output(Magzine_Handler_t *magz);

/* #��Ϣ��# ---------------------------------------------------------------------------------------------------------------------------------------*/
bool MAGZINE_IfOpen(void);

/* #Ӧ�ò�# ---------------------------------------------------------------------------------------------------------------------------------------*/
//void REMOTE_SetMagzineState(RC_Ctl_t *remote);
//void KEY_SetMagzineState(RC_Ctl_t *remote);
	
/* #�����# ---------------------------------------------------------------------------------------------------------------------------------------*/
void MAGZINE_RcCtrlTask(void);
void MAGZINE_KeyCtrlTask(void);
void MAGZINE_SelfProtect(void);
void MAGZINE_Ctrl(void);

#endif
