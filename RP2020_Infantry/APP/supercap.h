#ifndef __SUPERCAP_H
#define __SUPERCAP_H

/* Includes ------------------------------------------------------------------*/
#include "sys.h"

/* Global macro --------------------------------------------------------------*/
// [ע] RP2019MC2.0.pdf�����������Ŵ�λ(���ڴ����н�����������)
//      ���������´�壬��ע���������
#define CHARGE_PIN		PCout(3)
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
	float kp;
	float ki;
	float kd;
	float pout;
	float iout;
	float dout;
	float out;
	float iout_max;
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
/* API functions Prototypes --------------------------------------------------*/
/* #������# ---------------------------------------------------------------------------------------------------------------------------------------*/

/* #Ӧ�ò�# ---------------------------------------------------------------------------------------------------------------------------------------*/


/* #�����# ---------------------------------------------------------------------------------------------------------------------------------------*/
void SuperCAP_SelfProtect(void);
void SuperCAP_Ctrl(void);


#endif
