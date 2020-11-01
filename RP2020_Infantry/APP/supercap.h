#ifndef __SUPERCAP_H
#define __SUPERCAP_H

/* Includes ------------------------------------------------------------------*/
#include "sys.h"

/* Global macro --------------------------------------------------------------*/
// [注] RP2019MC2.0.pdf中这两个引脚错位(已在代码中交换控制引脚)
//      后续若重新打板，需注意调换引脚
#define CHARGE_PIN		PCout(6)	// 原PC(3)
#define DISCHARGE_PIN	PAout(5)

/* Global TypeDef ------------------------------------------------------------*/
typedef enum {
	ON,
	OFF,
	CAP_STATUS_COUNT
}SuperCap_Status_t;

typedef struct {
	float target;
	float feedback;
	float err;
	float integrate;
	float integrate_max;
	float kp;
	float ki;
	float kd;
	float pout;
	float iout;
	float dout;
	float out;
	float out_max;
}SuperCap_Ctrl_t;

typedef struct {
	Remote_Mode_Names_t remote_mode;
	SuperCap_Status_t charge_status;
	SuperCap_Status_t discharge_status;
	float real_vol;
	float real_power;
	float real_power_buff;
	float max_power;
	uint16_t set_vol;
}SuperCap_Info_t;

typedef struct {
	SuperCap_Ctrl_t *ctrl;
	SuperCap_Info_t *info;
}SuperCap_t;
/* ## Global Variables Prototypes ## -----------------------------------------*/
extern SuperCap_t SuperCap;

/* API functions Prototypes --------------------------------------------------*/
/* #驱动层# ---------------------------------------------------------------------------------------------------------------------------------------*/


void SuperCAP_GetInfo(void);

/* #应用层# ---------------------------------------------------------------------------------------------------------------------------------------*/
void SuperCAP_Init(void);

/* #任务层# ---------------------------------------------------------------------------------------------------------------------------------------*/
void SuperCAP_SelfProtect(void);
void SuperCAP_Ctrl(void);


#endif
