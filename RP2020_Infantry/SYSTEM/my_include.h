#ifndef __INCLUDE_H
#define __INCLUDE_H

/* Interrupt Priority Management Table */
#define USART1_IT_PRIO_PRE	3
#define USART1_IT_PRIO_SUB	3

#define USART2_IT_PRIO_PRE	1
#define USART2_IT_PRIO_SUB	1

#define USART3_IT_PRIO_PRE	3
#define USART3_IT_PRIO_SUB	3

/* �Ӿ�ͨ�Ŵ���4�ж� */
#define UART4_IT_PRIO_PRE	0
#define UART4_IT_PRIO_SUB	0

/* ����ϵͳ����5�ж� */
#define UART5_IT_PRIO_PRE	0
#define UART5_IT_PRIO_SUB	1

//#define DMA1_Stream3_PRIO_PRE	3
//#define DMA1_Stream3_PRIO_SUB	3

#define CAN1_RX0_PRIO_PRE		0
#define CAN1_RX0_PRIO_SUB		1

#define CAN2_RX0_PRIO_PRE		0
#define CAN2_RX0_PRIO_SUB		1

#define CAN2_TX_PRIO_PRE		0
#define CAN2_TX_PRIO_SUB		3

/* Flag Asset Structure */
typedef struct {
	struct {
		uint8_t		RcLost;
		uint8_t		RcErr;
	}Remote;
	struct {
		uint8_t		FLAG_angleTurnOk;
		uint8_t		FLAG_mode;
		uint8_t		GoHome;	// Ťͷ����
		uint8_t		ReloadBulletStart;
		uint8_t		ReloadBulletJudge;
	}Chassis;
	struct {
		uint8_t		ResetOk;
		uint8_t		AngleRecordStart;
		uint8_t		FLAG_angleTurnOk;
	}Gimbal;
	struct {
		uint8_t		ResetOk;
	}Friction;
}Flag_t;

/* Counter Asset Structure */
typedef struct {
	struct {
		uint8_t		Reset;
		uint8_t		Err;
	}System;
	struct {
		uint32_t	RcLost;	
		uint32_t  	RcLostRecover;
	}Remote;
	struct {
		uint8_t		CNT_angleTurnOk;
		uint8_t		ReloadBulletJudge;
	}Chassis;
	struct {
		uint8_t		ResetOk;
		uint8_t		CNT_angleTurnOk;
	}Gimbal;
}Cnt_t;

/* Bit Mask Asset Structure */
typedef struct {
	struct {
		uint16_t	BM_Reset;	// ϵͳ��λ����λ��Ĥ
	}System;
	struct {
		uint8_t		BM_rxReport;	// ���̵�����ձ���λ��Ĥ
	}Chassis;
	struct {
		uint8_t 	BM_rxReport;	// ��̨������ձ���λ��Ĥ
	}Gimbal;
	struct {
		uint8_t 	BM_rxReport;	// ����������ձ���λ��Ĥ
	}Revolver;
}BitMask_t;

/* Bit Mask Can Report Enum */
typedef enum {
	BM_RX_REPORT_201 = 0x01,
	BM_RX_REPORT_202 = 0x02,
	BM_RX_REPORT_203 = 0x04,
	BM_RX_REPORT_204 = 0x08,
	BM_RX_REPORT_205 = 0x10,
	BM_RX_REPORT_206 = 0x20,
	BM_RX_REPORT_207 = 0x40,
	BM_RX_REPORT_208 = 0x80,
}BM_Rx_Report_t;

/* Bit Mask Reset Enum */
typedef enum {
	BM_RESET_GIMBAL = 0x01,	// ��λ��̨
	BM_RESET_FRIC   = 0x02, 	// ��λĦ����
}BM_System_Reset_t;

/* Pid Structure */
typedef struct{
	float kp;
	float ki;
	float kd;
	float target;
	float feedback;
	float erro;
	float last_erro;
	float integrate;
	float integrate_max;
	float pout;
	float iout;
	float dout;
	float out;
	float out_max;
}PID_Object_t;

/* Mpu Structure */
typedef struct {
	float pitch;
	float roll;
	float yaw;
	short ratePitch;
	short rateRoll;
	short rateYaw;
	float pitchOffset;
	float rollOffset;
	float yawOffset;
	float ratePitchOffset;
	float rateRollOffset;
	float rateYawOffset;
}Mpu_Info_t;

/* Queue Structure */
typedef struct {
	uint8_t nowLength;
	uint8_t queueLength;
	float	queue[20];
}QueueObj;

/* System State Enum */
typedef enum {
	SYSTEM_STATE_NORMAL  = 0,
	SYSTEM_STATE_RCLOST	 = 1,
	SYSTEM_STATE_RCERR	 = 2,
}System_State_t;

/* Remote Mode Enum */
typedef enum {
	RC = 0,
	KEY = 1,
	REMOTE_MODE_COUNT = 2,
}Remote_Mode_Names_t;

/* Pid Mode Enum */
typedef enum {
	MECH = 0,
	GYRO = 1,
	PID_MODE_COUNT = 2
}Pid_Mode_Names_t;

/* Infantry Action Enum */
typedef enum {
	SYS_ACT_NORMAL = 0,	// ������Ϊ
	SYS_ACT_AUTO = 1,	// ������Ϊ
	SYS_ACT_BUFF = 2,	// �����Ϊ
	SYS_ACT_PARK = 3,	// �Զ���λ��Ϊ
	SYS_ACT_COUNT = 4,
}System_Action_Names_t;

/* Infantry Branch Action Enum */
typedef enum {
	BCH_ACT_SMALL_BUFF = 0,
	BCH_ACT_BIG_BUFF = 1,
	BCH_ACT_COUNT = 2,
}Branch_Action_Names_t;

/* Time State Structure */
typedef enum {
	NOW = 0,
	PREV = 1,
	DELTA = 2,
	TIME_STATE_COUNT = 3,
}Time_State_Names_t;

/* System Structure */
typedef struct {
	System_State_t			State;
	Remote_Mode_Names_t		RemoteMode;
	Pid_Mode_Names_t		PidMode;
	System_Action_Names_t	Action;
	Branch_Action_Names_t	BranchAction;
}System_t;

extern Flag_t 	 	Flag;
extern Cnt_t		Cnt;
extern BitMask_t 	BitMask;
extern Mpu_Info_t 	Mpu_Info;

extern System_t		System;
#endif

