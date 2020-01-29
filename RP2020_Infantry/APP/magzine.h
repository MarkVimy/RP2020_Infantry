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

typedef struct {
	Magzine_State_Names_t state;
	int16_t	angleTarget;
	int16_t	angleFeedback;
}Magzine_Info_t;

/* ## Global Variables Prototypes ## -----------------------------------------*/
extern Magzine_Info_t Magzine;

/* API functions Prototypes --------------------------------------------------*/
/* #������# ---------------------------------------------------------------------------------------------------------------------------------------*/
void MAGZINE_ParamsInit(Magzine_Info_t *magz);
void MAGZINE_stop(Magzine_Info_t *magz);
void MAGZINE_close(Magzine_Info_t *magz);
void MAGZINE_output(Magzine_Info_t *magz);

/* #��Ϣ��# ---------------------------------------------------------------------------------------------------------------------------------------*/
bool MAGZINE_ifOpen(void);

/* #Ӧ�ò�# ---------------------------------------------------------------------------------------------------------------------------------------*/
//void REMOTE_setMagzineState(RC_Ctl_t *remoteInfo);
//void KEY_setMagzineState(RC_Ctl_t *remoteInfo);
	
/* #�����# ---------------------------------------------------------------------------------------------------------------------------------------*/
void MAGZINE_rcControlTask(void);
void MAGZINE_keyControlTask(void);
void MAGZINE_selfProtect(void);
void MAGZINE_control(void);

#endif
