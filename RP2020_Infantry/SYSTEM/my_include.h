#ifndef __INCLUDE_H
#define __INCLUDE_H

/* Interrupt Priority Management Table */
/* FreeRTOS中设置为4位抢占优先级，故子优先级无用(默认设置为0) */
/* Zigbee串口1中断 */
#define USART1_IT_PRIO_PRE	3
#define USART1_IT_PRIO_SUB	0

/* 遥控器串口2中断 */
#define USART2_IT_PRIO_PRE	0
#define USART2_IT_PRIO_SUB	0

/* 超声波串口3中断 */
#define USART3_IT_PRIO_PRE	2
#define USART3_IT_PRIO_SUB	0

/* 视觉通信串口4中断 */
#define UART4_IT_PRIO_PRE	1
#define UART4_IT_PRIO_SUB	0

/* 裁判系统串口5中断 */
#define UART5_IT_PRIO_PRE	0
#define UART5_IT_PRIO_SUB	0

/* CAN1接收中断 */
#define CAN1_RX0_PRIO_PRE	1
#define CAN1_RX0_PRIO_SUB	0

/* CAN2接收中断 */
#define CAN2_RX0_PRIO_PRE	1
#define CAN2_RX0_PRIO_SUB	0

#define CAN2_TX_PRIO_PRE	3
#define CAN2_TX_PRIO_SUB	0

/* Flag Asset Structure */
typedef struct {
	struct {
		uint8_t		RcLost;
		uint8_t		RcErr;
	}Remote;
	struct {
		uint8_t		FLAG_angleTurnOk;
		uint8_t		FLAG_mode;
		uint8_t		GoHome;	// 扭头就跑
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
		uint16_t	BM_Reset;		// 系统复位任务位掩膜
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
	BM_RESET_GIMBAL = 0x01,	// 复位云台
	BM_RESET_FRIC   = 0x02, // 复位摩擦轮
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
	SYS_ACT_NORMAL = 0,	// 常规行为
	SYS_ACT_AUTO = 1,	// 自瞄行为
	SYS_ACT_BUFF = 2,	// 打符行为
	SYS_ACT_PARK = 3,	// 自动对位行为
	SYS_ACT_COUNT = 4,
}System_Action_Names_t;

/* Infantry Branch Action Enum */
typedef enum {
	BCH_ACT_SMALL_BUFF = 0,	// 打小符
	BCH_ACT_BIG_BUFF = 1,	// 打大符
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

