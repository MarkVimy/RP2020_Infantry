#ifndef __TASK_REVOLVER_H
#define __TASK_REVOLVER_H

/* Includes ------------------------------------------------------------------*/
#include "sys.h"
#include "my_app.h"

/* Global macro --------------------------------------------------------------*/
#define REVOLVER_ID					0x207
#define REVOLVER_BM_RX_REPORT		BM_RX_REPORT_207

#define REVOLVER_SPEED_RATIO		2160		// 1��תһȦ(= 60rpm*36(���ٱ�) => 1rps)
#define REVOLVER_SPEED_GRID			12.0f		// ����12��
#define SPEED_AN_BULLET_PER_SECOND	(REVOLVER_SPEED_RATIO/REVOLVER_SPEED_GRID)	// 1��/s �������ٶ�
#define ANGLE_AN_BULLET				24576.0f	// 8192*36/12 = 24576.0f
#define ANGLE_AN_BULLET_RAMP		(ANGLE_AN_BULLET/20)	// ���20msתһ��(һ�����ܳ���50ms)

#define HEAT_AN_BULLET				10	// ��һ�ŵ������������(RM2020�̶�Ϊ10)


/* Global TypeDef ------------------------------------------------------------*/
typedef enum {
	REVO_STATE_OFF = 0,
	REVO_STATE_ON = 1,
	REVO_STATE_COUNT =2
}Revolver_State_Names_t;

typedef enum {
	REVO_POSI_MODE  = 0,	// ����ģʽ(λ�û�)
	REVO_SPEED_MODE = 1,	// ����ģʽ(�ٶȻ�)
	REVO_MODE_COUNT = 2
}Revolver_Mode_Names_t;

typedef enum {
	SHOOT_NORMAL = 0,		// ����ģʽ(������/����/����)
//	SHOOT_SINGLE = 1,		// ����ģʽ
//	SHOOT_TRIPLE = 2,		// ������
//	SHOOT_HIGH_F_LOW_S = 3,	// ����Ƶ������(�Ƽ�)
	SHOOT_BUFF	 = 1,		// ���ģʽ
	SHOOT_AUTO	 = 2,		// ����ģʽ
	SHOOT_BAN	 = 3,		// ��ֹ��ģʽ
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
	uint16_t 	total_count;	// �ܷ�����
	uint32_t 	real_time;		// ����ʵʱʱ��(������������ӳ�)
	uint16_t	freq;			// ��Ƶ
	uint16_t	interval;		// ����ʱ����
	uint16_t	num;			// ��Ҫ������ӵ���(����ָ����)
	uint16_t	num_buffer; 	// ��������ӵ���
	uint16_t	heat_cooling_rate; // 17mmǹ��ÿ����ȴֵ
	uint16_t	heat_real;		// 17mmǹ������
	uint16_t	heat_limit;		// 17mmǹ����������
	uint16_t 	stuck_count;	// ��������	
	uint8_t		suicide_mode;	// ��ɱģʽ(ǹ����������)
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
/* #������# ---------------------------------------------------------------------------------------------------------------------------------------*/
void REVOLVER_PidParamsInit(Revolver_PID_t *pid);
void REVOLVER_Stop(Revolver_PID_t *pid);
void REVOLVER_Speed_PidCalc(Revolver_PID_t *pid);
void REVOLVER_Angle_PidCalc(Revolver_PID_t *pid);
void REVOLVER_PidOut(Revolver_PID_t *pid);

/* #��Ϣ��# ---------------------------------------------------------------------------------------------------------------------------------------*/
void REVOLVER_SetAction(Revolver_Action_t action);
void REVOLVER_AddShootCount(void);

/* info -> out */
uint32_t REVOLVER_GetRealShootTime(void);

/* in <- info */
void REVOLVER_GetSysInfo(System_t *sys, Revolver_Info_t *revo);
void REVOLVER_GetJudgeInfo(Judge_Info_t *judge, Revolver_Info_t *revo);
//void REVOLVER_GetRemoteInfo(System_t *sys, RC_Ctl_t *remote, Revolver_Info_t *revo);

/* #Ӧ�ò�# ---------------------------------------------------------------------------------------------------------------------------------------*/
uint8_t REVOLVER_HeatLimit(Revolver_Info_t *revo, uint8_t target_num);
void REVOLVER_NormalCtrl(Revolver_Info_t *revo);
void REVOLVER_SingleShootCtrl(Revolver_Info_t *revo);
void REVOLVER_TripleShootCtrl(Revolver_Info_t *revo);
void REVOLVER_BuffShootCtrl(Revolver_Info_t *revo);

/* #�����# ---------------------------------------------------------------------------------------------------------------------------------------*/
void REVOLVER_GetInfo(void);
void REVOLVER_RcCtrlTask(void);
void REVOLVER_KeyCtrlTask(void);
void REVOLVER_SelfProtect(void);
void REVOLVER_Ctrl(void);

#endif
