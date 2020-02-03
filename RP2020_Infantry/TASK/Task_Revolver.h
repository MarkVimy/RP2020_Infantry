#ifndef __TASK_REVOLVER_H
#define __TASK_REVOLVER_H

/* Includes ------------------------------------------------------------------*/
#include "sys.h"

/* Global macro --------------------------------------------------------------*/
#define REVOLVER_ID							0x207
#define REVOLVER_BM_RX_REPORT		BM_RX_REPORT_207

#define REVOLVER_SPEED_RATIO		2160		// 1��תһȦ(= 60rpm*36(���ٱ�) => 1rps)
#define REVOLVER_SPEED_GRID			12.0f		// ����12��
#define SPEED_AN_BULLET_PER_SECOND	(REVOLVER_SPEED_RATIO/REVOLVER_SPEED_GRID)	// 1��/s �������ٶ�
#define ANGLE_AN_BULLET				24576.0f	// 8192*36/12 = 24576.0f
#define ANGLE_AN_BULLET_RAMP		(ANGLE_AN_BULLET/20)

#define HEAT_AN_BULLET				30	// ��һ�ŵ������������

//#define LEVEL_COUNT				3
//uint16_t REVOLVER_HEAT_LIMIT[LEVEL_COUNT] = {240, 360, 480};

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
	SHOOT_NORMAL = 0,		// ����ģʽ
	SHOOT_SINGLE = 1,		// ����ģʽ
	SHOOT_TRIPLE = 2,		// ������
	SHOOT_HIGH_F_LOW_S = 3,	// ����Ƶ������(�Ƽ�)
	SHOOT_MID_F_HIGH_S = 4,	// ����Ƶ������(����)
	SHOOT_BUFF	 = 5,		// ���ģʽ
	SHOOT_AUTO	 = 6,		// ����ģʽ
	REVO_ACTION_COUNT = 7,
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
	uint16_t	freq;					// ��Ƶ
	int16_t		interval;			// ����ʱ����
	int16_t		num;					// ��Ҫ������ӵ���
	int16_t		num_buffer; 	// ��������ӵ���
	int16_t		heat_cooling_rate; // 17mmǹ��ÿ����ȴֵ
	int16_t		heat_real;		// 17mmǹ������
	int16_t		heat_limit;		// 17mmǹ����������
	uint16_t 	stuck_count;	// ��������	
}Revolver_Shoot_Info_t;

typedef struct {
	Revolver_State_Names_t 	state;
	Revolver_Mode_Names_t 	pidMode; 
	Revolver_Action_t				action;
	Revolver_Shoot_Info_t		Shoot;
	uint16_t	freq;	// ��Ƶ
	int16_t		shootInterval;	// ����ʱ����
	int16_t		shootNum;		// ��Ҫ������ӵ���
	int16_t		shootNumBuffer; // ��������ӵ���
	int16_t		shootHeatCoolingRate; // 17mmǹ��ÿ����ȴֵ
	int16_t		shootHeatReal;	// 17mmǹ������
	int16_t		shootHeatLimit;	// 17mmǹ����������
	uint16_t 	stuckCount;	// ��������
	Revolver_Buff_Info_t	buff;
}Revolver_Info_t;

/* ## Global Variables Prototypes ## -----------------------------------------*/
extern Revolver_PID_t Revolver_PID;
extern Revolver_Info_t Revolver;

/* API functions Prototypes --------------------------------------------------*/
/* #������# ---------------------------------------------------------------------------------------------------------------------------------------*/
void REVOLVER_pidParamsInit(Revolver_PID_t *pid);
void REVOLVER_stop(Revolver_PID_t *pid);
void REVOLVER_Speed_pidCalculate(Revolver_PID_t *pid);
void REVOLVER_Angle_pidCalculate(Revolver_PID_t *pid);
void REVOLVER_pidOut(Revolver_PID_t *pid);

/* #��Ϣ��# ---------------------------------------------------------------------------------------------------------------------------------------*/
uint32_t REVOLVER_getRealShootTime(void);
void REVOLVER_addShootCount(void);

/* #Ӧ�ò�# ---------------------------------------------------------------------------------------------------------------------------------------*/
void REVOLVER_setAction(Revolver_Action_t action);
uint8_t REVOLVER_heatLimit(Revolver_Info_t *info, uint8_t targetNum);
void REVOLVER_normalControl(Revolver_Info_t *info);
void REVOLVER_singleShootControl(Revolver_Info_t *info);
void REVOLVER_tripleShootControl(Revolver_Info_t *info);
void REVOLVER_buffShootControl(Revolver_Info_t *info);

/* #�����# ---------------------------------------------------------------------------------------------------------------------------------------*/
void REVOLVER_rcControlTask(void);
void REVOLVER_keyControlTask(void);
void REVOLVER_selfProtect(void);
void REVOLVER_control(void);

#endif
