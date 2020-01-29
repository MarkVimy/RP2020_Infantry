#ifndef __TASK_CHASSIS_H
#define __TASK_CHASSIS_H

/* Includes ------------------------------------------------------------------*/
#include "sys.h"

#include "my_app.h"
/* Global macro --------------------------------------------------------------*/
#define CHASSIS_MODE_COUNT	PID_MODE_COUNT

#define CHASSIS_SPEED_IOUT_MAX		6000	// ���̵���ٶȻ�pid�����޷�
#define CHASSIS_ANGLE_IOUT_MAX		3000	// ���̵��λ�û�pid�����޷�
#define CHASSIS_PID_OUT_MAX				9000	// ���̵��pid������ֵ(��������)

#define WARNING_REMAIN_POWER			60
#define CHASSIS_MAX_CURRENT_LIMIT	36000	// �������������9000

/* Global TypeDef ------------------------------------------------------------*/
typedef enum {
	LEFT_FRON_201 = 0,  // ��ǰ
	RIGH_FRON_202 = 1,  // ��ǰ
	LEFT_BACK_203 = 2,  // ���
	RIGH_BACK_204 = 3,  // �Һ�
	CHASSIS_MOTOR_COUNT = 4
}Chassis_Motor_Names;

typedef struct {
	PID_Object_t	Speed;
	PID_Object_t	Angle;
	float					Out;
}Chassis_PID_t;

typedef struct {
	PID_Object_t	Angle;
	float					AngleRampTarget;
	float					AngleRampFeedback;
	float					Out;
}Chassis_Z_PID_t;

typedef struct {
	float maxLimit;		// �������
	float currentLimit;	// ��ǰ����
}Chassis_Power_t;

typedef enum {
	CHAS_MODE_NORMAL = 0,	// ����ģʽ
	CHAS_MODE_TWIST = 1,	// Ť��ģʽ
	CHAS_MODE_BUFF = 2,		// ���ģʽ
	CHAS_MODE_SLOW = 3,		// ���̵��ٲ���ģʽ
	CHAS_MODE_SZUPUP = 4,	// SZU����ģʽ
}Chassis_Mode_Names_t;

typedef enum {
	CHAS_LOGIC_NORMAL = 0,
	CHAS_LOGIC_REVERT = 1,
}Chassis_Logic_Names_t;

typedef struct {
	Pid_Mode_Names_t			pid_mode;
	Chassis_Mode_Names_t	mode;
	Chassis_Logic_Names_t logic;
	uint8_t								top_gyro;
}Chassis_Info_t;

/* ## Global Variables Prototypes ## -----------------------------------------*/
extern Chassis_PID_t Chassis_PID[CHASSIS_MOTOR_COUNT];
extern Chassis_Z_PID_t Chassis_Z_PID;
extern Chassis_Info_t Chassis;
/* API functions Prototypes --------------------------------------------------*/
/* #������# ---------------------------------------------------------------------------------------------------------------------------------------*/
void CHASSIS_updateMotorSpeed(Chassis_PID_t *pid, Chassis_Motor_Names MOTORx);
void CHASSIS_updateMotorAngle(Chassis_PID_t *pid, Chassis_Motor_Names MOTORx);
void CHASSIS_updateMotorCurrent(Chassis_PID_t *pid, Chassis_Motor_Names MOTORx);

void CHASSIS_PID_ParamsInit(Chassis_PID_t *pid, uint8_t motor_cnt);
void CHASSIS_Z_Speed_PID_ParamsInit(PID_Object_t *pid);
void CHASSIS_stop(Chassis_PID_t *pid);
void CHASSIS_Speed_PID_calculate(Chassis_PID_t *pid, Chassis_Motor_Names MOTORx);
void CHASSIS_Angle_PID_calculate(Chassis_PID_t *pid, Chassis_Motor_Names MOTORx);
float CHASSIS_Z_Speed_PID_calculate(PID_Object_t *pid, float kp);
void CHASSIS_PID_out(Chassis_PID_t *pid);

/* #��Ϣ��# ---------------------------------------------------------------------------------------------------------------------------------------*/
void CHASSIS_getInfo(void);
void CHASSIS_setMode(Chassis_Mode_Names_t mode);
void CHASSIS_logicRevert(void);
Chassis_Mode_Names_t CHASSIS_getMode(void);
Chassis_Logic_Names_t CHASSIS_getLogic(void);
float CHASSIS_getMiddleAngle(void);
bool CHASSIS_ifNormalMode(void);
bool CHASSIS_ifTopGyroOpen(void);
bool CHASSIS_ifLogicRevert(void);
	
/* #Ӧ�ò�# ---------------------------------------------------------------------------------------------------------------------------------------*/
void CHASSIS_init(void);
float CHASSIS_twistTargetCalculate(int16_t maxTarget, int16_t rampTarget);
//void CHASSIS_powerLimit(Chassis_Power_t *power, Chassis_PID_t *pid, Judge_Info_t *judge_info);
void CHASSIS_omniSpeedCalculate(void);
float CHASSIS_MECH_yawTargetBoundaryProcess(Chassis_Z_PID_t *pid, float delta_target);

/* #�����# ---------------------------------------------------------------------------------------------------------------------------------------*/
void CHASSIS_selfProtect(void);	// ����ʧ�ر���
void CHASSIS_control(void);	// ���̿�������

#endif
