#ifndef __INCLUDE_H
#define __INCLUDE_H

#include "sys.h"

/* Interrupt Priority Management Table */
#define USART1_IT_PRIO_PRE	3
#define USART1_IT_PRIO_SUB	3

#define USART2_IT_PRIO_PRE	1
#define USART2_IT_PRIO_SUB	1

#define USART3_IT_PRIO_PRE	3
#define USART3_IT_PRIO_SUB	3

/* 视觉通信串口4中断 */
#define UART4_IT_PRIO_PRE	0
#define UART4_IT_PRIO_SUB	0

/* 裁判系统串口5中断 */
#define UART5_IT_PRIO_PRE	0
#define UART5_IT_PRIO_SUB	0

//#define DMA1_Stream3_PRIO_PRE	3
//#define DMA1_Stream3_PRIO_SUB	3

#define CAN1_RX0_PRIO_PRE		0
#define CAN1_RX0_PRIO_SUB		1

#define CAN2_RX0_PRIO_PRE		0
#define CAN2_RX0_PRIO_SUB		2

#define CAN2_TX_PRIO_PRE		0
#define CAN2_TX_PRIO_SUB		3

/* Flag Asset Structure */
typedef struct {
	struct {
		uint8_t		FLAG_rcLost;
		uint8_t		FLAG_rcErr;
		uint8_t		FLAG_mode;
	}Remote;
	struct {
		uint8_t		FLAG_pidStart;
		uint8_t		FLAG_angleTurnOk;
		uint8_t		FLAG_mode;
		uint8_t		FLAG_goHome;	// 扭头就跑
	}Chassis;
	struct {
		uint8_t		FLAG_pidStart;
		uint8_t		FLAG_pidMode;
		uint8_t		FLAG_resetOK;
		uint8_t		FLAG_angleRecordStart;
		uint8_t		FLAG_angleTurnOk;
	}Gimbal;
	struct {
		uint8_t		FLAG_pidStart;
	}Revolver;
	struct {
		uint8_t		FLAG_resetOK;
	}Friction;
}Flag_t;

/* Counter Asset Structure */
typedef struct {
	struct {
		uint8_t		CNT_reset;
		uint8_t		CNT_err;
	}System;
	struct {
		uint32_t	CNT_rcLost;	
		uint32_t  CNT_rcLostRecover;
	}Remote;
	struct {
		uint8_t		CNT_angleTurnOk;
	}Chassis;
	struct {
		uint8_t		CNT_resetOK;
		uint16_t	CNT_enableChangePidMode;
		uint8_t		CNT_angleTurnOk;
	}Gimbal;
}Cnt_t;

/* Bit Mask Asset Structure */
typedef struct {
	struct {
		uint16_t	BM_reset;	// 系统复位任务位掩膜
	}System;
	struct {
		uint8_t		BM_rxReport;	// 底盘电机接收报文位掩膜
	}Chassis;
	struct {
		uint8_t 	BM_rxReport;	// 云台电机接收报文位掩膜
	}Gimbal;
	struct {
		uint8_t 	BM_rxReport;	// 拨弹电机接收报文位掩膜
	}Revolver;
}BitMask_t;

/* System State Enum */
typedef enum {
	SYSTEM_STATE_NORMAL  = 0,
	SYSTEM_STATE_RCLOST	 = 1,
	SYSTEM_STATE_RCERR	 = 2,
}System_State_t;

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

typedef enum {
	BM_RESET_GIMBAL = 0x01,	// 复位云台
	BM_RESET_FRIC   = 0x02, 	// 复位摩擦轮
}BM_System_Reset_t;

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

typedef struct {
	uint8_t nowLength;
	uint8_t queueLength;
	float	data[20];
}QueueObj;

typedef enum {
	RC = 0,
	KEY = 1,
	REMOTE_MODE_COUNT = 2,
}Remote_Mode_Names_t;

typedef enum {
	MECH = 0,
	GYRO = 1,
	PID_MODE_COUNT = 2
}Pid_Mode_Names_t;

typedef enum {
	NOW = 0,
	PREV = 1,
	DELTA = 2,
	TIME_STATE_COUNT = 3,
}Time_State_Names_t;

extern Flag_t 	 	Flag;
extern Cnt_t		Cnt;
extern BitMask_t 	BitMask;
extern Mpu_Info_t 	Mpu_Info;

extern System_State_t	System_State;
#endif

