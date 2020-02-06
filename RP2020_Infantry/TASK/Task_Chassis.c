/**
 * @file        Task_Chassis.c
 * @author      MaWeiming
 * @Version     V1.0
 * @date        30-September-2019
 * @brief       This file includes the Chassis(底盘) external functions 
 *
 * @Verison			V1.1 (1-October-2019)
 */

/**
 *	PID调参口诀
 *	kp大 -> 响应快 -> 超调
 *	ki大 -> 消除静差 -> 
 *	参考网址：https://www.jianshu.com/p/4b0fa85cd353
 */

/* Includes ------------------------------------------------------------------*/
#include "Task_Chassis.h"

#include "arm_math.h"
#include "kalman.h"
#include "can.h"
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#define X	0
#define Y 1
#define Z 2
#define ALL 3

/* 19步兵车参数 */
#define RADIUS     76      //麦轮半径
#define PERIMETER  478     //麦轮周长
#define WHEELTRACK 360//398     //左右轮距
#define WHEELBASE  300     //前后轴距
#define GIMBAL_X_OFFSET 0//75  //云台相对底盘中心的前后偏移量
#define GIMBAL_Y_OFFSET 0 //云台相对底盘中心的左右偏移量
#define CHASSIS_DECELE_RATIO (1.0f/19.0f)  //电机减数比
#define RADIAN_COEF 57.3f  

/* 不同模式下底盘最大速度 */
#define STANDARD_MAX_NORMAL	CHASSIS_PID_OUT_MAX
#define SPIN_MAX_NORMAL			CHASSIS_PID_OUT_MAX

#define STANDARD_MAX_SZUPUP	3000
#define SPIN_MAX_SZUPUP			9000

/* 不同模式下底盘斜坡 */
#define TIME_INC_NORMAL	2
#define TIME_DEC_NORMAL	500

#define TIME_INC_SZUPUP	3

#define SPIN_ANGLE	(35)

/* Private variables ---------------------------------------------------------*/
/* 卡尔曼滤波器 */
extKalman_t Chassis_Kalman_Error;

/* 限幅 */
float Chas_Spin_Move_Max = CHASSIS_PID_OUT_MAX;			// 底盘旋转-限速
float Chas_Standard_Move_Max = CHASSIS_PID_OUT_MAX;	// 底盘平移-限速

float Chas_Top_Spin_Max = CHASSIS_PID_OUT_MAX;
float Chas_Top_Move_Max = CHASSIS_PID_OUT_MAX;

float Chas_Top_Spin_scale = 0.8;	// 小陀螺旋转所能分配的速度比例
float Chas_Top_Move_scale = 0.2;	// 小陀螺行进所能分配的速度比例

/* 斜坡 */
uint16_t Time_Inc_Normal;			// 正常斜坡增加量
uint16_t Time_Inc_Saltation = 1;	// 前后方向突变斜坡增加量

/* 键盘模式下的前后左后以及旋转斜坡变量 */
float	Chas_Slope_Move_Fron, Chas_Slope_Move_Back, Chas_Slope_Move_Left, Chas_Slope_Move_Righ;
float Chas_Slope_Spin_Move;

/* 期望速度 */
float Chas_Target_Speed[4];

/* 摇杆灵敏度 */
//机械模式下底盘比例系数,控制摇杆响应速度,如果过小也会限制最高转速,max = 此系数 *660
float kRc_Mech_Chas_Standard, kRc_Mech_Chas_Spin;//平移，旋转,

//陀螺仪模式下底盘比例系数,控制摇杆响应速度,如果过小也会限制最高转速,max = 此系数 *660
float kRc_Gyro_Chas_Standard, kRc_Gyro_Chas_Spin;//平移，旋转

//机械模式下底盘比例系数,控制键盘斜坡变化率
float kKey_Mech_Chas_Standard, kKey_Mech_Chas_Spin;//平移，旋转

//陀螺仪模式下底盘比例系数,控制键盘斜坡变化率
float kKey_Gyro_Chas_Standard, kKey_Gyro_Chas_Spin;//平移，旋转

//小陀螺模式下底盘比例系数
float k_Gyro_Chas_Top;

//扭腰模式下底盘比例系数
float k_Gyro_Chas_Twist;

//小陀螺旋转转速步长
float Chas_Top_Gyro_Step;

//扭腰旋转步长
float Chas_Twist_Step;

/* ## Global variables ## ----------------------------------------------------*/
/* #驱动层# ---------------------------------------------------------------------------------------------------------------------------------------*/
float speed_target_203;
float speed_feedback_203;
float angle_target_203;
float angle_feedback_203;

/**
 *	@brief	底盘PID
 */
Chassis_PID_t Chassis_PID[CHASSIS_MOTOR_COUNT] = {
	{	// LEFT_FRON_201
		/* 速度环 */
		.Speed.kp = 11.5,		// 10.8			11.2
		.Speed.ki = 236,		// 243			238
		.Speed.kd = 0.0112,	// 0.0112		0.0112
		.Speed.target = 0,
		.Speed.feedback = 0,
		.Speed.erro = 0,
		.Speed.last_erro = 0,
		.Speed.integrate = 0,
		.Speed.pout = 0,
		.Speed.iout = 0,
		.Speed.dout = 0,
		.Speed.out = 0,
		/* 位置环 */
		.Angle.kp = 0,
		.Angle.ki = 0,
		.Angle.kd = 0,
		.Angle.target = 0,
		.Angle.feedback = 0,
		.Angle.erro = 0,
		.Angle.last_erro = 0,
		.Angle.integrate = 0,
		.Angle.pout = 0,
		.Angle.iout = 0,
		.Angle.dout = 0,
		.Angle.out = 0,		
	},
	{	// RIGH_FRON_202
		/* 速度环 */
		.Speed.kp = 11.5,		// 10.8
		.Speed.ki = 236,		// 243
		.Speed.kd = 0.0112,	// 0.0112
		.Speed.target = 0,
		.Speed.feedback = 0,
		.Speed.erro = 0,
		.Speed.last_erro = 0,
		.Speed.integrate = 0,
		.Speed.pout = 0,
		.Speed.iout = 0,
		.Speed.dout = 0,
		.Speed.out = 0,
		/* 位置环 */
		.Angle.kp = 0,
		.Angle.ki = 0,
		.Angle.kd = 0,
		.Angle.target = 0,
		.Angle.feedback = 0,
		.Angle.erro = 0,
		.Angle.last_erro = 0,
		.Angle.integrate = 0,
		.Angle.pout = 0,
		.Angle.iout = 0,
		.Angle.dout = 0,
		.Angle.out = 0,		
	},
	{	// LEFT_BACK_203
		/* 速度环 */
		.Speed.kp = 11.5,		// 10.8
		.Speed.ki = 236,		// 243
		.Speed.kd = 0.0112,	// 0.0112
		.Speed.target = 0,
		.Speed.feedback = 0,
		.Speed.erro = 0,
		.Speed.last_erro = 0,
		.Speed.integrate = 0,
		.Speed.pout = 0,
		.Speed.iout = 0,
		.Speed.dout = 0,
		.Speed.out = 0,
		/* 位置环 */
		.Angle.kp = 1.172,		// 1.15					1.16				1.172		(这套参数适合大角度转动)
		.Angle.ki = 0.7,			// 1						0.82					0.7
		.Angle.kd = 0.0021,		// 0.002				0.0021			0.0021
		.Angle.target = 0,
		.Angle.feedback = 0,
		.Angle.erro = 0,
		.Angle.last_erro = 0,
		.Angle.integrate = 0,
		.Angle.pout = 0,
		.Angle.iout = 0,
		.Angle.dout = 0,
		.Angle.out = 0,		
	},
	{	// RIGH_BACK_204
		/* 速度环 */
		.Speed.kp = 11.5,		// 10.8
		.Speed.ki = 236,		// 243
		.Speed.kd = 0.0112,	// 0.0112
		.Speed.target = 0,
		.Speed.feedback = 0,
		.Speed.erro = 0,
		.Speed.last_erro = 0,
		.Speed.integrate = 0,
		.Speed.pout = 0,
		.Speed.iout = 0,
		.Speed.dout = 0,
		.Speed.out = 0,
		/* 位置环 */
		.Angle.kp = 0,
		.Angle.ki = 0,
		.Angle.kd = 0,
		.Angle.target = 0,
		.Angle.feedback = 0,
		.Angle.erro = 0,
		.Angle.last_erro = 0,
		.Angle.integrate = 0,
		.Angle.pout = 0,
		.Angle.iout = 0,
		.Angle.dout = 0,
		.Angle.out = 0,		
	},
};

/**
 *	@brief	底盘Z方向Pid
 */
Chassis_Z_PID_t Chassis_Z_PID = {
	.Angle = 
	{
		.kp = 0.48,
		.ki = 0,
		.kd = 0,
		.target = GIMBAL_MECH_YAW_MID_ANGLE,
		.feedback = GIMBAL_MECH_YAW_MID_ANGLE,
		.erro = 0,
		.last_erro = 0,
		.integrate = 0,
		.pout = 0,
		.iout = 0,
		.dout = 0,
		.out = 0,
	},
	.AngleRampTarget = 0,
	.AngleRampFeedback = 0,
	.Out = 0
};

/**
 *	@brief	底盘定位PID
 */
PID_Object_t Chas_Locate_PID[2];

/**
 *	@brief	底盘功率
 */
Chassis_Power_t Chassis_Power = {
	.maxLimit = CHASSIS_MAX_CURRENT_LIMIT,
	.currentLimit = 0,
};

/**
 *	@brief	底盘信息结构体
 */
Chassis_Info_t Chassis = {
	.pid_mode = MECH,
	.mode = CHAS_MODE_NORMAL,
	.logic = CHAS_LOGIC_NORMAL,
	.top_gyro = false,
};


/* #应用层# ---------------------------------------------------------------------------------------------------------------------------------------*/

/* #任务层# ---------------------------------------------------------------------------------------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* API functions -------------------------------------------------------------*/
/* #驱动层# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	底盘参数初始化
 */
void CHASSIS_initParameter(void)
{
	/* 平移摇杆灵敏度 */
	kRc_Mech_Chas_Standard = 14;	// 14*660 = 9240 -> 9000
	kRc_Gyro_Chas_Standard = 14;	// 14*660 = 9240 -> 9000
	
	/* 旋转摇杆灵敏度 */
	kRc_Mech_Chas_Spin = 11.4;	// 11.4*660 = 7524
	kKey_Mech_Chas_Spin = 40;		// 键盘机械模式扭头响应快慢 #未测
	
	/* 陀螺仪模式底盘系数 */
	kRc_Gyro_Chas_Spin = -4.8;	// (反向)跟随
	kKey_Gyro_Chas_Spin = kRc_Gyro_Chas_Spin;
	
	/* 小陀螺底盘系数 */
	k_Gyro_Chas_Top = -4.8;
	
	/* 扭腰底盘系数 */
	k_Gyro_Chas_Twist = -4.8;
	
	/* 小陀螺转速步长 */
	Chas_Top_Gyro_Step = 10;
	
	/* 扭腰旋转步长 */
	Chas_Twist_Step = 4;
}

/**
 *	@brief	底盘电机PID参数初始化
 */
void CHASSIS_PID_ParamsInit(Chassis_PID_t *pid, uint8_t motor_cnt)
{
	uint8_t i;
	for(i = 0; i < motor_cnt; i++) {
		pid[i].Speed.target = 0;
		pid[i].Speed.feedback = 0,
		pid[i].Speed.erro = 0;
		pid[i].Speed.last_erro = 0;
		pid[i].Speed.integrate = 0;
		pid[i].Speed.pout = 0;
		pid[i].Speed.dout = 0;
		pid[i].Speed.iout = 0;
		pid[i].Speed.out = 0;

		pid[i].Angle.target = 0;
		pid[i].Angle.feedback = 0,
		pid[i].Angle.erro = 0;
		pid[i].Angle.last_erro = 0;
		pid[i].Angle.integrate = 0;
		pid[i].Angle.pout = 0;
		pid[i].Angle.dout = 0;
		pid[i].Angle.iout = 0;
		pid[i].Angle.out = 0;
	}
}

/**
 *	@brief	底盘陀螺仪模式参数重置
 */
void CHASSIS_Z_PID_ParamsInit(PID_Object_t *pid)
{
		pid->target = CHASSIS_getMiddleAngle();
		pid->feedback = CHASSIS_getMiddleAngle();
		pid->erro = 0;
		pid->last_erro = 0;
		pid->integrate = 0;
		pid->pout = 0;
		pid->iout = 0;
		pid->dout = 0;
		pid->out = 0;
}

/**
 *	@brief	底盘陀螺仪模式卡尔曼滤波器初始化
 */
void CHASSIS_kalmanCreate(void)
{
	/* 假定机械角度的反馈没有噪声误差 */
	KalmanCreate(&Chassis_Kalman_Error, 1, 0);
}	

/**
 *	@brief	底盘电机更新实时速度
 */
void CHASSIS_updateMotorSpeed(Chassis_PID_t *pid, Chassis_Motor_Names MOTORx)
{
	pid[MOTORx].Speed.feedback = g_Chassis_Motor_Info[MOTORx].speed;
}

/**
 *	@brief	底盘电机更新实时角度(累积转过的角度)
 */
void CHASSIS_updateMotorAngle(Chassis_PID_t *pid, Chassis_Motor_Names MOTORx)
{
  //pid[MOTORx].Angle.feedback = g_Chassis_Motor_Info[MOTORx].angle;
	pid[MOTORx].Angle.feedback = g_Chassis_Motor_Info[MOTORx].angle_sum;
}

/**
 *	@brief	底盘电机更新实时电流值
 */
void CHASSIS_updateMotorCurrent(Chassis_PID_t *pid, Chassis_Motor_Names MOTORx)
{
	//pid[MOTORx].Speed.feedback = g_Chassis_Motor_Info[MOTORx].speed;
}

/**
 *	@brief	底盘电机紧急刹车
 */
void CHASSIS_stop(Chassis_PID_t *pid)
{
	static int16_t pid_out[4] = {0, 0, 0, 0};
	
	/* 内环速度环最终输出 */
	pid[LEFT_FRON_201].Out = 0;
	pid[RIGH_FRON_202].Out = 0;
	pid[LEFT_BACK_203].Out = 0;
	pid[RIGH_BACK_204].Out = 0;
	
	CAN1_send(0x200, pid_out);	
}

/**
 *	@brief	底盘电机速度环
 */
void CHASSIS_Speed_pidCalculate(Chassis_PID_t *pid, Chassis_Motor_Names MOTORx)
{
	pid[MOTORx].Speed.erro = pid[MOTORx].Speed.target - pid[MOTORx].Speed.feedback;
	pid[MOTORx].Speed.integrate += pid[MOTORx].Speed.erro;
	
	//pid[MOTORx].Speed.integrate = constrain(pid[MOTORx].Speed.integrate, -3000, 3000);
	// #integrate是否限幅?
	
	pid[MOTORx].Speed.integrate = constrain(pid[MOTORx].Speed.integrate, -18000, 18000);
	
	/* Pout */
	pid[MOTORx].Speed.pout = pid[MOTORx].Speed.kp * pid[MOTORx].Speed.erro;
	/* Iout */
	pid[MOTORx].Speed.iout = pid[MOTORx].Speed.ki * pid[MOTORx].Speed.integrate * 0.002f; // 模仿19步兵代码(0.002f是2ms的意思)
	/* Iout Limits */
	pid[MOTORx].Speed.iout = constrain(pid[MOTORx].Speed.iout, -CHASSIS_SPEED_IOUT_MAX, CHASSIS_SPEED_IOUT_MAX);
	/* Dout*/
	pid[MOTORx].Speed.dout = pid[MOTORx].Speed.kd * (pid[MOTORx].Speed.erro - pid[MOTORx].Speed.last_erro)/0.002f;
	/* Record Last Error */
	pid[MOTORx].Speed.last_erro = pid[MOTORx].Speed.erro;
	
	/* 19步兵代码 - 积分项缓慢减小，防止误差为0时突然失力 */
	if( pid[MOTORx].Speed.pout * pid[MOTORx].Speed.iout < 0) {
		pid[MOTORx].Speed.integrate = constrain(pid[MOTORx].Speed.integrate, 
												-CHASSIS_SPEED_IOUT_MAX/pid[MOTORx].Speed.ki/5.f,
												+CHASSIS_SPEED_IOUT_MAX/pid[MOTORx].Speed.ki/5.f);
	}
	
	/* Total PID Output*/
	pid[MOTORx].Speed.out = pid[MOTORx].Speed.pout + pid[MOTORx].Speed.iout + pid[MOTORx].Speed.dout;
	/* Total PID Output Limits */
	pid[MOTORx].Speed.out = constrain(pid[MOTORx].Speed.out, -CHASSIS_PID_OUT_MAX, CHASSIS_PID_OUT_MAX);
	pid[MOTORx].Out = pid[MOTORx].Speed.out;
	
	/* test */
	speed_target_203 = pid[LEFT_BACK_203].Speed.target;
	speed_feedback_203 = pid[LEFT_BACK_203].Speed.feedback;
}

/**
 *	@brief	底盘单电机位置环
 *	@note
 *			串环PID
 *			内环 期望值 期望角速度(实际上就是速度环)
 *					 输出值 期望角加速度
 *
 *			外环 期望值 期望角度
 *					 输出值 期望角速度
 */
void CHASSIS_Angle_pidCalculate(Chassis_PID_t *pid, Chassis_Motor_Names MOTORx)
{
	pid[MOTORx].Angle.erro = pid[MOTORx].Angle.target - pid[MOTORx].Angle.feedback;
	pid[MOTORx].Angle.integrate += pid[MOTORx].Angle.erro;
	//pid[MOTORx].Angle.integrate = constrain(pid[MOTORx].Angle.integrate, -3000, 3000);
	// #integrate是否限幅?
	
	/* Pout */
	pid[MOTORx].Angle.pout = pid[MOTORx].Angle.kp * pid[MOTORx].Angle.erro;
	/* Iout */
	pid[MOTORx].Angle.iout = pid[MOTORx].Angle.ki * pid[MOTORx].Angle.integrate * 0.002f; // 模仿19步兵代码(0.002f是2ms的意思)
	/* Iout Limits */
	pid[MOTORx].Angle.iout = constrain(pid[MOTORx].Angle.iout, -CHASSIS_ANGLE_IOUT_MAX, CHASSIS_ANGLE_IOUT_MAX);
	/* Dout*/
	pid[MOTORx].Angle.dout = pid[MOTORx].Angle.kd * (pid[MOTORx].Angle.erro - pid[MOTORx].Angle.last_erro)/0.002f;
	/* Record Last Error */
	pid[MOTORx].Angle.last_erro = pid[MOTORx].Angle.erro;
	
//	/* 19步兵代码 - 积分项缓慢减小，防止误差为0时突然失力 */
//	if( pid[MOTORx].Angle.pout * pid[MOTORx].Angle.iout < 0) {
//		pid[MOTORx].Angle.integrate = constrain(pid[MOTORx].Angle.integrate, 
//												-CHASSIS_SPEED_IOUT_MAX/pid[MOTORx].Angle.ki/5.f,
//												+CHASSIS_SPEED_IOUT_MAX/pid[MOTORx].Angle.ki/5.f);
//	}
	
	/* Total PID Output*/
	pid[MOTORx].Angle.out = pid[MOTORx].Angle.pout + pid[MOTORx].Angle.iout + pid[MOTORx].Angle.dout;
	/* Total PID Output Limits */
	pid[MOTORx].Angle.out = constrain(pid[MOTORx].Angle.out, -CHASSIS_PID_OUT_MAX, CHASSIS_PID_OUT_MAX);
	
	/* test */
	angle_target_203 = pid[LEFT_BACK_203].Angle.target;
	angle_feedback_203 = pid[LEFT_BACK_203].Angle.feedback;
}

/**
 *	@brief	底盘陀螺仪模式加上Z方向的速度环(实际上是位置环)
 *	@note
 *				将当前YAW机械角和中值YAW机械角的差值作为误差送进 Z_PID控制器。
 *				反馈期望的角速度
 */
float CHASSIS_Z_Angle_pidCalculate(PID_Object_t *pid, float kp)
{
	pid->kp = kp;
	pid->erro = pid->target - pid->feedback;
	
	/* 临界处理 */
	
	/*
	
		0 - 8191 

		误差=期望-反馈
	
		误差大于4096
	
		反馈是落后于期望的 --> 确定期望的方向可认为期望从0-》8191 --》--- 反馈大于期望
	
		实际误差 = -（8191 - 期望 + 反馈） = -8191 + 误差
	
		误差小于-4096
	
		反馈落后于期望 -> 确定期望方向 8191->0  == 期望大于反馈
	
		实际误差 = 8191 + 期望 -反馈 = 8191 + (期望 - 反馈) = 8191 + 误差 
	*/
	
	
	if(pid->erro >= 4096) {
		pid->erro = -8192 + pid->erro;
	} 
	else if(pid->erro <= -4096) {
		pid->erro = 8192 + pid->erro;
	}
	
	pid->erro = KalmanFilter(&Chassis_Kalman_Error, pid->erro);
	
	/* 底盘处于普通跟随模式 */
	if(CHASSIS_ifTopGyroOpen() == false) {
		// 死区控制算法
		if(abs(pid->erro) < 15) {
			pid->erro = 0;
		}
	}
	/* 底盘处于小陀螺模式 */
	else {
		//..不做死区处理
	}
	
	/* Pout */
	pid->pout = pid->kp * pid->erro;
	if(abs(pid->erro) > SPIN_ANGLE) {
		/* Dout*/
		pid->dout = pid->kd * (pid->erro - pid->last_erro)/0.002f;
	} else { 
		pid->dout = 0;
	}
	
	/* Record Last Error */
	pid->last_erro = pid->erro;
	
	/* Total PID Output*/
	pid->out = pid->pout + pid->dout;
	/* Total PID Output Limits */
	pid->out = constrain(pid->out, -Chas_Spin_Move_Max, Chas_Spin_Move_Max);	
	return pid->out;
}

/**
 *	@brief	底盘电机PID的最终输出
 */
void CHASSIS_pidOut(Chassis_PID_t *pid)
{
	int16_t pidOut[4];
	
	if(BitMask.Chassis.BM_rxReport & BM_RX_REPORT_201) {
		pidOut[LEFT_FRON_201] = (int16_t)pid[LEFT_FRON_201].Out;
	} else {
		pidOut[LEFT_FRON_201] = 0;	// 失联后卸力
	}
	
	if(BitMask.Chassis.BM_rxReport & BM_RX_REPORT_202) {
		pidOut[RIGH_FRON_202] = (int16_t)pid[RIGH_FRON_202].Out;
	} else {
		pidOut[LEFT_FRON_201] = 0;	// 失联后卸力
	}
	
	if(BitMask.Chassis.BM_rxReport & BM_RX_REPORT_203) {
		pidOut[LEFT_BACK_203] = (int16_t)pid[LEFT_BACK_203].Out;
	} else {
		pidOut[LEFT_BACK_203] = 0;	// 失联后卸力
	}
	
	if(BitMask.Chassis.BM_rxReport & BM_RX_REPORT_204) {
		pidOut[RIGH_BACK_204] = (int16_t)pid[RIGH_BACK_204].Out;
	} else {
		pidOut[RIGH_BACK_204] = 0;	// 失联后卸力
	}
	
	CAN1_queueSend(0x200, pidOut);
}

/* #信息层# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	底盘获取pid模式信息
 */
void CHASSIS_getInfo(void)
{
	static Gimbal_Mode_t now_mode;
	static float yaw[TIME_STATE_COUNT];
	
	/* # Yaw # */
	yaw[NOW] = Mpu_Info.yaw;
	yaw[DELTA] = yaw[NOW] - yaw[PREV];
	if(yaw[DELTA] > +180.f) {
		yaw[DELTA] = -360.f + yaw[DELTA];
	} else if(yaw[DELTA] < -180.f) {
		yaw[DELTA] = +360.f + yaw[DELTA];
	}
	yaw[PREV] = yaw[NOW]; 	
	
	/* 利用陀螺仪数据作扭头补偿(使得云台与底盘独立运动) */
	if(CHASSIS_ifTopGyroOpen() == true
		|| CHASSIS_ifTwistOpen() == true) {
		Chassis_Z_PID.Angle.target = CHASSIS_MECH_yawTargetBoundaryProcess(&Chassis_Z_PID, 		
																	yaw[DELTA] * 8192 / 360.f);
	}
	
	/* 底盘pid模式 */
	if(GIMBAL_ifMechMode()) {
		Chassis.pid_mode = MECH;
		Chassis.top_gyro = false;	// 关陀螺
		Chassis.twist = false;		// 关扭腰
	} 
	else if(GIMBAL_ifGyroMode()) {
		Chassis.pid_mode = GYRO;
	}
	
	/* 底盘模式调整 */
	now_mode = GIMBAL_getMode();
	switch(now_mode)
	{
		case GIMBAL_MODE_NORMAL:
		case GIMBAL_MODE_AUTO:
			/*
				云台常规和自瞄时底盘恢复常规
			*/
			if(Chassis.mode != CHAS_MODE_SZUPUP)
				CHASSIS_setMode(CHAS_MODE_NORMAL);
			break;
		case GIMBAL_MODE_BIG_BUFF:
		case GIMBAL_MODE_SMALL_BUFF:
			/*
				云台打符时底盘进入打符模式
			*/
			CHASSIS_setMode(CHAS_MODE_BUFF);
			break;
		case GIMBAL_MODE_RELOAD_BULLET:
			/*
				自动对位时底盘进入慢速对位模式
			*/
			CHASSIS_setMode(CHAS_MODE_RELOAD_BULLET);
			break;
		default:
			break;
	}
	
//	/* 小陀螺模式开启 */
//	if(GIMBAL_ifTopGyroMode() == true) {
//			CHASSIS_setMode(CHAS_MODE_TOP_GYRO);
//	} else {
//		/* 底盘小陀螺模式 -> 底盘常规控制模式 */
//		if(CHASSIS_ifTopGyroMode() == true) {
//			CHASSIS_setMode(CHAS_MODE_NORMAL);
//		}
//	}
	
//	/* 底盘逻辑取反 */
//	if(CHASSIS_ifLogicRevert() == true) {
//		CHASSIS_setMode(CHAS_MODE_REVERT);
//	}
	
	/* 弹仓打开后慢速 */
//	if(MAGZINE_ifOpen() == true) {
//		CHASSIS_setMode(CHAS_MODE_RELOAD_BULLET);
//	}
}

/**	
 *	@breif	反馈底盘模式
 */
Chassis_Mode_Names_t CHASSIS_getMode(void)
{
	return Chassis.mode;
}

/**	
 *	@breif	反馈底盘逻辑
 */
Chassis_Logic_Names_t CHASSIS_getLogic(void)
{
	return Chassis.logic;
}

/**	
 *	@breif	反馈底盘机械中值目标
 */
float CHASSIS_getMiddleAngle(void)
{
	if(Chassis.logic == 0) {
		return GIMBAL_TOP_YAW_MID_ANGLE;
	}
	else {
		return GIMBAL_REVERT_YAW_MID_ANGLE;
	}
}

/**
 *	@brief	设置底盘的模式
 */
void CHASSIS_setMode(Chassis_Mode_Names_t mode)
{
	Chassis.mode = mode;
}

/**
 *	@brief	底盘逻辑取反
 */
void CHASSIS_logicRevert(void)
{
	if(Chassis.logic == CHAS_LOGIC_NORMAL) {
		Chassis.logic = CHAS_LOGIC_REVERT;
	} else {
		Chassis.logic = CHAS_LOGIC_NORMAL;	
	}
}

/**
 *	@brief	底盘是否处于机械模式
 */
bool CHASSIS_ifMechMode(void)
{
	if(Chassis.pid_mode == MECH)
		return true;
	else
		return false;
}

/**
 *	@brief	底盘是否处于陀螺仪模式
 */
bool CHASSIS_ifGyroMode(void)
{
	if(Chassis.pid_mode == GYRO)
		return true;
	else
		return false;
}

/**
 *	@brief	底盘是否处于常规模式
 */
bool CHASSIS_ifNormalMode(void)
{
	if(Chassis.mode == CHAS_MODE_NORMAL)
		return true;
	else 
		return false;
}

/**
 * @brief		底盘是否处于小陀螺模式
 */
bool CHASSIS_ifTopGyroOpen(void)
{
	if(Chassis.pid_mode == GYRO
			&& Chassis.top_gyro == true)
		return true;

	return false;
}

/**
 *	@brief	底盘是否处于扭腰模式
 */
bool CHASSIS_ifTwistOpen(void)
{
	if(Chassis.pid_mode == GYRO
			&& Chassis.twist == true)
		return true;
			
	return false;
}

/**	
 *	@breif	底盘逻辑是否取反
 */
bool CHASSIS_ifLogicRevert(void)
{
	static Chassis_Logic_Names_t prev_logic = CHAS_LOGIC_NORMAL;
	
	/* 底盘逻辑取反检测 */
	if(prev_logic != Chassis.logic) {
		return true;
	}
	prev_logic = Chassis.logic;
	
	return false;
}


/* #应用层# ---------------------------------------------------------------------------------------------------------------------------------------*/

/* #键盘鼠标# -------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	根据遥控值设置速度环的期望值(非小陀螺版本)
 *	@note		涉及麦克纳姆轮的运动合成
 *					target均为+时:
 *					LF201	↑	↓	RF202	
 *					
 *					LB203	↑	↓	RB204
 *					
 *					-660 ~ +660 遥控差值
 *					
 *					需要考虑摇杆的分配值，保证速度的目标值最大为CHASSIS_PID_OUT_MAX(9000)
 */

void REMOTE_setChassisSpeed(void)
{
	float k_rc_z;

	/* 机械模式 */
	if(Chassis.pid_mode == MECH) {
		
		Chas_Target_Speed[ X ] = RC_LEFT_CH_LR_VALUE * kRc_Mech_Chas_Standard;
		Chas_Target_Speed[ X ] = constrain(Chas_Target_Speed[ X ], -Chas_Standard_Move_Max, Chas_Standard_Move_Max);
		
		Chas_Target_Speed[ Y ] = RC_LEFT_CH_UD_VALUE * kRc_Mech_Chas_Standard;
		Chas_Target_Speed[ Y ] = constrain(Chas_Target_Speed[ Y ], -Chas_Standard_Move_Max, Chas_Standard_Move_Max);
		
		Chas_Target_Speed[ Z ] = RC_RIGH_CH_LR_VALUE * kRc_Mech_Chas_Spin;
		Chas_Target_Speed[ Z ] = constrain(Chas_Target_Speed[ Z ], -Chas_Spin_Move_Max, Chas_Spin_Move_Max);
		
	}
	/* 陀螺仪模式 */
	else if(Chassis.pid_mode == GYRO) {

		Chas_Target_Speed[ X ] = RC_LEFT_CH_LR_VALUE * kRc_Gyro_Chas_Standard;
		Chas_Target_Speed[ X ] = constrain(Chas_Target_Speed[ X ], -Chas_Standard_Move_Max, Chas_Standard_Move_Max);
		
		Chas_Target_Speed[ Y ] = RC_LEFT_CH_UD_VALUE * kRc_Gyro_Chas_Standard;
		Chas_Target_Speed[ Y ] = constrain(Chas_Target_Speed[ Y ], -Chas_Standard_Move_Max, Chas_Standard_Move_Max);
		
		Chassis_Z_PID.Angle.target = CHASSIS_getMiddleAngle();	// 恢复底盘跟随
		Chas_Target_Speed[ Z ] = CHASSIS_Z_Angle_pidCalculate(&Chassis_Z_PID.Angle, (YAW_DIR) * kRc_Gyro_Chas_Spin);
		Chas_Target_Speed[ Z ] = constrain(Chas_Target_Speed[ Z ], -Chas_Spin_Move_Max, Chas_Spin_Move_Max);
	}
	
	/* 计算旋转因子(通过调整进入条件来设置旋转和平移的分配比例) */
	if(abs(Chas_Target_Speed[ Z ]) > 800) {	
		// 扭头速度越快，平移速度越慢
		k_rc_z = ((Chas_Spin_Move_Max - (abs(Chas_Target_Speed[ Z ]) - 800)) * (Chas_Spin_Move_Max - (abs(Chas_Target_Speed[ Z ]) - 800)))
							/ (Chas_Spin_Move_Max * Chas_Spin_Move_Max);	// 理论计算范围 (0.008, 1)
		
		k_rc_z = constrain(k_rc_z, 0.f, 1.f);
	} else {
		k_rc_z = 1.f;
	}		
	
	/* 根据旋转量重新分配速度期望值 */
	Chas_Target_Speed[ X ] = Chas_Target_Speed[ X ] * k_rc_z;
	Chas_Target_Speed[ Y ] = Chas_Target_Speed[ Y ] * k_rc_z;
	
	/* 计算当前总期望速度 */
	Chas_Target_Speed[ ALL ] = abs(Chas_Target_Speed[ X ]) + abs(Chas_Target_Speed[ Y ]) + abs(Chas_Target_Speed[ Z ]);
	
	/* 全向分配算法 */
	CHASSIS_omniSpeedCalculate();
}

/**
 *	@brief	根据遥控值设置速度环的期望值(小陀螺版本)
 *	@note		涉及麦克纳姆轮的运动合成
 *					target均为+时:
 *					LF201	↑	↓	RF202	
 *					
 *					LB203	↑	↓	RB204
 *					
 *					-660 ~ +660 遥控差值
 *					
 *					需要考虑摇杆的分配值，保证速度的目标值最大为CHASSIS_PID_OUT_MAX(9000)
 *
 *			云台坐标系(开启小陀螺之前正对机械中值)：
 *				↑枪管朝向 Y
 *				↑
 *				↑	  
 *				。→→	  X
 *
 *			底盘坐标系：
 *			^\.		^/.	
 *				↑ 		  y	顺时针角度偏差增大(+)
 *				↑ 
 *				。→→	  x
 *				
 *			^/.		^\.
 */
float delta_angle;
int8_t top_gyro_dir = 0;
int8_t twist_dir = 0;
void REMOTE_TOP_setChassisSpeed(void)
{
	float k_rc_z;
	float target_speed_x;
	float target_speed_y;
	
	/* 机械模式 */
	if(CHASSIS_ifMechMode() == true) {
		
		Chas_Target_Speed[ X ] = RC_LEFT_CH_LR_VALUE * kRc_Mech_Chas_Standard;
		Chas_Target_Speed[ X ] = constrain(Chas_Target_Speed[ X ], -Chas_Standard_Move_Max, Chas_Standard_Move_Max);
		
		Chas_Target_Speed[ Y ] = RC_LEFT_CH_UD_VALUE * kRc_Mech_Chas_Standard;
		Chas_Target_Speed[ Y ] = constrain(Chas_Target_Speed[ Y ], -Chas_Standard_Move_Max, Chas_Standard_Move_Max);
		
		Chas_Target_Speed[ Z ] = RC_RIGH_CH_LR_VALUE * kRc_Mech_Chas_Spin;
		Chas_Target_Speed[ Z ] = constrain(Chas_Target_Speed[ Z ], -Chas_Spin_Move_Max, Chas_Spin_Move_Max);
		
	}
	/* 陀螺仪模式 */
	else if(CHASSIS_ifGyroMode() == true) {

		Chas_Target_Speed[ X ] = RC_LEFT_CH_LR_VALUE * kRc_Gyro_Chas_Standard;
		Chas_Target_Speed[ X ] = constrain(Chas_Target_Speed[ X ], -Chas_Standard_Move_Max, Chas_Standard_Move_Max);
		
		Chas_Target_Speed[ Y ] = RC_LEFT_CH_UD_VALUE * kRc_Gyro_Chas_Standard;
		Chas_Target_Speed[ Y ] = constrain(Chas_Target_Speed[ Y ], -Chas_Standard_Move_Max, Chas_Standard_Move_Max);
		
		/* 开启了小陀螺 */
		if(CHASSIS_ifTopGyroOpen() == true) {

			// 外环期望斜坡变化
			Chassis_Z_PID.Angle.target = CHASSIS_MECH_yawTargetBoundaryProcess(&Chassis_Z_PID, top_gyro_dir * Chas_Top_Gyro_Step);
			// Z方向速度pid计算
			Chas_Target_Speed[ Z ] = CHASSIS_Z_Angle_pidCalculate(&Chassis_Z_PID.Angle, (YAW_DIR) * k_Gyro_Chas_Top);
			
		}
		/* 没开小陀螺 */
		else {
			
			// 恢复底盘跟随
			Chassis_Z_PID.Angle.target = CHASSIS_getMiddleAngle() ;	
			// Z方向速度pid计算
			Chas_Target_Speed[ Z ] = CHASSIS_Z_Angle_pidCalculate(&Chassis_Z_PID.Angle, (YAW_DIR) * kRc_Gyro_Chas_Spin);
			
		}
		
		// Z方向速度限幅
		Chas_Target_Speed[ Z ] = constrain(Chas_Target_Speed[ Z ], -Chas_Spin_Move_Max, Chas_Spin_Move_Max);
	}
	
	/* 陀螺仪模式下的坐标系转换 */
	if(CHASSIS_ifGyroMode() == true) {
		
		// 计算偏差机械中值的角度差
		delta_angle = GIMBAL_getTopGyroAngleOffset() * MECH_2_ANGLE_RADIAN;	//  (8192.f-GIMBAL_getTopGyroAngleOffset()) * MECH_2_ANGLE_RADIAN
		target_speed_x = Chas_Target_Speed[ X ];
		target_speed_y = Chas_Target_Speed[ Y ];
		// 旋转矩阵 将云台坐标系的期望速度 转换到 底盘坐标系的期望速度
		Chas_Target_Speed[ X ] = cos(delta_angle) * target_speed_x - sin(delta_angle) * target_speed_y;
		Chas_Target_Speed[ Y ] = sin(delta_angle) * target_speed_x + cos(delta_angle) * target_speed_y;
		
	}
	
	/* 计算旋转因子(通过调整进入条件来设置旋转和平移的分配比例) */
	if(abs(Chas_Target_Speed[ Z ]) > 800) {	
		// 扭头速度越快，平移速度越慢
		k_rc_z = ((Chas_Spin_Move_Max - (abs(Chas_Target_Speed[ Z ]) - 800)) * (Chas_Spin_Move_Max - (abs(Chas_Target_Speed[ Z ]) - 800)))
							/ (Chas_Spin_Move_Max * Chas_Spin_Move_Max);	// 理论计算范围 (0.008, 1)
		
		k_rc_z = constrain(k_rc_z, 0.f, 1.f);
	} else {
		k_rc_z = 1.f;
	}		
	
	/* 根据旋转量重新分配速度期望值 */
	Chas_Target_Speed[ X ] = Chas_Target_Speed[ X ] * k_rc_z;
	Chas_Target_Speed[ Y ] = Chas_Target_Speed[ Y ] * k_rc_z;
	
	/* 计算当前总期望速度 */
	Chas_Target_Speed[ ALL ] = abs(Chas_Target_Speed[ X ]) + abs(Chas_Target_Speed[ Y ]) + abs(Chas_Target_Speed[ Z ]);
	
	/* 全向分配算法 */
	CHASSIS_omniSpeedCalculate();	
}

/**
 *	@brief	根据遥控拨轮设置云台小陀螺模式
 */
void REMOTE_setTopGyro(void)
{
	static uint8_t rcWheelLockOnFlag = false;
	static uint8_t rcWheelLockOffFlag = false;
	
	/* 拨轮向上开启 */
	if(RC_THUMB_WHEEL_VALUE < -650) {
		if(rcWheelLockOnFlag == false) {
			/* 陀螺仪模式 */
			if(CHASSIS_ifGyroMode() == true) {
				Chassis_Z_PID.Angle.target = Chassis_Z_PID.Angle.feedback;
				Chassis.top_gyro = true;
				Chassis.twist = false;
				top_gyro_dir = -1;
			}
		}
		rcWheelLockOnFlag = true;
	} else {
		rcWheelLockOnFlag = false;
	}
	
	/* 拨轮向下关闭 */
	if(RC_THUMB_WHEEL_VALUE > 650) {
		if(rcWheelLockOffFlag == false) {
			if(CHASSIS_ifGyroMode() == true) {
				Chassis.top_gyro = false;
			}
		}
		rcWheelLockOffFlag = true;
	} else {
		rcWheelLockOffFlag = false;
	}
}

/**
 *	@brief	按键斜坡函数
 *	@note	长按返回最大系数1
 *			长时未按返回最小系数0
 */
float KEY_rampChassisSpeed(int8_t key_state, int16_t *time, uint16_t inc_ramp_step, uint16_t dec_ramp_step)
{
	float fac;
	fac = 0.15 * sqrt( 0.15 * (*time) );	// time累加到296.3斜坡就完成
	if(key_state == 1) {	// 按键按下
		if(fac < 1) {
			*time += inc_ramp_step;
		}
	} else {	// 按键松开
		if(fac > 0) {
			*time -= dec_ramp_step;
			if(*time < 0) {
				*time = 0;
			}
		}
	}
	fac = constrain(fac, 0, 1);
	return fac;
}

/**
 *	@brief	根据按键值设置旋转移动速度
 */
void KEY_setChasSpinSpeed(int16_t sSpinMax)
{
	Chas_Spin_Move_Max = sSpinMax;

/* 机械模式 */
	if(CHASSIS_ifMechMode() == true) {
		
		Chas_Target_Speed[ Z ] = MOUSE_X_MOVE_SPEED * kKey_Mech_Chas_Spin;
		Chas_Target_Speed[ Z ] = constrain(Chas_Target_Speed[ Z ], -Chas_Spin_Move_Max, Chas_Spin_Move_Max);
		
	}
	/* 陀螺仪模式 */
	else if(CHASSIS_ifGyroMode() == true) {
		/* 开启了小陀螺 */
		if(CHASSIS_ifTopGyroOpen() == true) {
			
			// 外环期望斜坡变化
			Chassis_Z_PID.Angle.target = CHASSIS_MECH_yawTargetBoundaryProcess(&Chassis_Z_PID, top_gyro_dir * Chas_Top_Gyro_Step);
			// Z方向速度pid计算
			Chas_Target_Speed[ Z ] = CHASSIS_Z_Angle_pidCalculate(&Chassis_Z_PID.Angle, (YAW_DIR) * k_Gyro_Chas_Top);
			
		}
		/* 开启了扭腰 */
		else if(CHASSIS_ifTwistOpen() == true) {
			
			if(Chassis_Z_PID.Angle.target >= CHASSIS_getMiddleAngle() + 800) {
				twist_dir = -1;
			} 
			else if(Chassis_Z_PID.Angle.target <= CHASSIS_getMiddleAngle() - 800) {
				twist_dir = +1;
			}
			Chassis_Z_PID.Angle.target = CHASSIS_MECH_yawTargetBoundaryProcess(&Chassis_Z_PID, twist_dir * Chas_Twist_Step);
			
			// Z方向速度pid计算
			Chas_Target_Speed[ Z ] = CHASSIS_Z_Angle_pidCalculate(&Chassis_Z_PID.Angle, (YAW_DIR) * k_Gyro_Chas_Twist);
			
		}
		/* 没开小陀螺/扭腰 */
		else {
			if(Flag.Chassis.FLAG_goHome == true) {
				/* 保持底盘状态
					 屏蔽底盘跟随
					 修改底盘逻辑 */
				Chassis.logic = (Chassis_Logic_Names_t)!Chassis.logic;	// 修改底盘逻辑
			}
			
			/* 设置底盘跟随逻辑 */
			Chassis_Z_PID.Angle.target = CHASSIS_getMiddleAngle();
			
			if(Flag.Chassis.FLAG_goHome == false)
				Chas_Target_Speed[ Z ] = CHASSIS_Z_Angle_pidCalculate(&Chassis_Z_PID.Angle, (YAW_DIR) * kKey_Gyro_Chas_Spin);
			else 
				Chas_Target_Speed[ Z ] = 0;	// 屏蔽底盘跟随
		}
		
		/* Z方向速度限幅 */
		Chas_Target_Speed[ Z ] = constrain(Chas_Target_Speed[ Z ], -Chas_Spin_Move_Max, Chas_Spin_Move_Max);
	}
}

/**
 *	@brief	根据按键值设置水平移动速度
 */
void KEY_setChasMoveSpeed(int16_t sMoveMax, int16_t sMoveRamp)
{
	static int16_t time_fron_y, time_back_y, time_left_x, time_righ_x;
	float target_speed_x, target_speed_y;
	float k_rc_z;
	
	Chas_Standard_Move_Max = sMoveMax;
	Time_Inc_Normal = sMoveRamp;
	
	if(IF_KEY_PRESSED_W) {
		time_back_y = 0;	// 后退斜坡清零
	} 
	if(IF_KEY_PRESSED_S) {
		time_fron_y = 0;	// 前进斜坡清零
	}
	if(IF_KEY_PRESSED_A) {
		time_righ_x = 0;	// 右移斜坡清零
	}
	if(IF_KEY_PRESSED_D) {
		time_left_x = 0;	// 左移斜坡清零
	}	
	
	if(Chassis.mode == CHAS_MODE_NORMAL) {
		
		/* 前后方向突变，刚开始一小段缓慢斜坡，防止轮子打滑浪费功率 */
		if(abs(Chas_Target_Speed[ Y ]) < (Chas_Standard_Move_Max/2.5f)) {
			Chas_Slope_Move_Fron = Chas_Standard_Move_Max * KEY_rampChassisSpeed(IF_KEY_PRESSED_W, &time_fron_y, Time_Inc_Saltation, TIME_DEC_NORMAL);
			Chas_Slope_Move_Back = Chas_Standard_Move_Max * KEY_rampChassisSpeed(IF_KEY_PRESSED_S, &time_back_y, Time_Inc_Saltation, TIME_DEC_NORMAL);
		} else {
			Chas_Slope_Move_Fron = Chas_Standard_Move_Max * KEY_rampChassisSpeed(IF_KEY_PRESSED_W, &time_fron_y, Time_Inc_Normal, TIME_DEC_NORMAL);
			Chas_Slope_Move_Back = Chas_Standard_Move_Max * KEY_rampChassisSpeed(IF_KEY_PRESSED_S, &time_back_y, Time_Inc_Normal, TIME_DEC_NORMAL);
		}
		
		/* 左右方向突变，刚开始一小段缓慢斜坡，防止轮子打滑浪费功率 */
		if(abs(Chas_Target_Speed[ X ]) < (Chas_Standard_Move_Max/2.5f)) {
			Chas_Slope_Move_Fron = Chas_Standard_Move_Max * KEY_rampChassisSpeed(IF_KEY_PRESSED_W, &time_fron_y, Time_Inc_Saltation, TIME_DEC_NORMAL);
			Chas_Slope_Move_Back = Chas_Standard_Move_Max * KEY_rampChassisSpeed(IF_KEY_PRESSED_S, &time_back_y, Time_Inc_Saltation, TIME_DEC_NORMAL);
		} else {
			Chas_Slope_Move_Left = Chas_Standard_Move_Max * KEY_rampChassisSpeed(IF_KEY_PRESSED_A, &time_left_x, Time_Inc_Normal, TIME_DEC_NORMAL);
			Chas_Slope_Move_Righ = Chas_Standard_Move_Max * KEY_rampChassisSpeed(IF_KEY_PRESSED_D, &time_righ_x, Time_Inc_Normal, TIME_DEC_NORMAL);
		}
		
	} else {
		/* 全向斜坡量计算 */
		Chas_Slope_Move_Fron = Chas_Standard_Move_Max * KEY_rampChassisSpeed(IF_KEY_PRESSED_W, &time_fron_y, Time_Inc_Normal, TIME_DEC_NORMAL);
		Chas_Slope_Move_Back = Chas_Standard_Move_Max * KEY_rampChassisSpeed(IF_KEY_PRESSED_S, &time_back_y, Time_Inc_Normal, TIME_DEC_NORMAL);
		Chas_Slope_Move_Left = Chas_Standard_Move_Max * KEY_rampChassisSpeed(IF_KEY_PRESSED_A, &time_left_x, Time_Inc_Normal, TIME_DEC_NORMAL);
		Chas_Slope_Move_Righ = Chas_Standard_Move_Max * KEY_rampChassisSpeed(IF_KEY_PRESSED_D, &time_righ_x, Time_Inc_Normal, TIME_DEC_NORMAL);
	}
	
	/* 计算旋转因子(通过调整进入条件来设置旋转和平移的分配比例) */
	if(abs(Chas_Target_Speed[ Z ]) > 800) {
		// 扭头速度越快，平移速度越慢
		k_rc_z = ((Chas_Spin_Move_Max - (abs(Chas_Target_Speed[ Z ]) - 800)) * (Chas_Spin_Move_Max - (abs(Chas_Target_Speed[ Z ]) - 800)))
							/ (Chas_Spin_Move_Max * Chas_Spin_Move_Max);
		
		k_rc_z = constrain(k_rc_z, 0.f, 1.f);
	} else {
		k_rc_z = 1.f;
	}		
	
	/* 期望速度计算 */
	Chas_Target_Speed[ Y ] = (Chas_Slope_Move_Fron - Chas_Slope_Move_Back) * k_rc_z;	
	Chas_Target_Speed[ X ] = (Chas_Slope_Move_Righ - Chas_Slope_Move_Left) * k_rc_z;
	
	/* 陀螺仪模式下的坐标系转换 */
	if(CHASSIS_ifGyroMode() == true) {
		/* 计算偏差机械中值的角度差 */
		delta_angle = GIMBAL_getTopGyroAngleOffset() * MECH_2_ANGLE_RADIAN;	//  (8192.f-GIMBAL_getTopGyroAngleOffset()) * MECH_2_ANGLE_RADIAN
		target_speed_x = Chas_Target_Speed[ X ];
		target_speed_y = Chas_Target_Speed[ Y ];
		
		if(Flag.Chassis.FLAG_goHome == true)
			delta_angle = 0;	// 保持底盘状态
		
		/* 旋转矩阵 将云台坐标系的期望速度 转换到 底盘坐标系的期望速度 */
		Chas_Target_Speed[ X ] = cos(delta_angle) * target_speed_x - sin(delta_angle) * target_speed_y;
		Chas_Target_Speed[ Y ] = sin(delta_angle) * target_speed_x + cos(delta_angle) * target_speed_y;
	}	
}

/**
 *	@brief	根据按键值设置速度环的期望值
 *  @note		Loop Time: 2ms
 *					单个麦轮的线速度(rpm)
 *					
 *					\		/			
 *						8	------	旋转中心	(旋转时各麦轮期望速度相同)
 *					/		\
 *	
 *					\	8 /	----	旋转中心	(旋转时各麦轮的期望速度不同，半径小的期望速度要小、半径大的期望速度要大)
 *					
 *					/		\
 *		
 *					可知：绕轴心旋转的运动，物体上各点的角速度(ω)相同，线速度(v)不同
 *					v = ω*r;	// 半径↑ -> 线速度↑
 *					ω = Chas_Target_Speed[ Z ];	// 机械角度/ms
 *					vz = ω*r;
 *
 *					由于我们的底盘电机PID控制的目标期望是定子转速(rpm)、速度反馈值为转子转速(rpm)
 *					# Chas_Target_Speed[ Z ] - 能在一定程度上反应转速但并不能完全等价
 *
 *					ω(机械角度/ms) => (机械角度/8192*360/ms) => (°/ms) = (1000°/s) = (1000/57.3 rad/s)
 *					ω(rad/s) = Chas_Target_Speed[ Z ] / 8192 * 360 *1000 / 57.3
 *					r(mm) => (1/1000 m)
 *					r(m) = 轮子距轴心的距离/1000;
 *					麦轮v(m/s) = ω*r = Chas_Target_Speed[ Z ] / 8192 * 360 *1000 / 57.3 * 轮子距轴心的距离/1000
 *									 					= Chas_Target_Speed[ Z ] / 8192 * 360 / 57.3 * 轮子距轴心的距离;
 *
 *					麦轮rpm(r/min) = 减速比 * 转子rpm => (麦轮周长(mm)*减速比*r/min) => (mm/min)
 *					麦轮v(m/s) = 麦轮周长(mm)*减速比*转子rpm(r/min)
 *
 *					麦轮v(m/s) => ((mm/1000)(min/60)) => (60/1000*mm/min)
 *
 *					# 麦轮周长(mm)*减速比*转子rpm(r/min) = 60/1000*麦轮v(m/s)
 *
 *					转子rpm = 60/1000*麦轮v(m/s) / (麦轮周长(mm)*减速比)
 *	
 *					转子rpm(r/min) = 60/1000/(麦轮周长(mm)*减速比) * (Chas_Target_Speed[ Z ] / 8192 * 360 / 57.3 * 轮子距轴心的距离)
 */
float rotate_ratio_fr;
float rotate_ratio_fl;
float rotate_ratio_bl;
float rotate_ratio_br;
float wheel_ratio_rpm;
void KEY_setChassisSpeed(int16_t sSpinMax, int16_t sMoveMax, int16_t sMoveRamp)
{
	/* 旋转 */
	KEY_setChasSpinSpeed(sSpinMax);
	
	/* 平移 */
	KEY_setChasMoveSpeed(sMoveMax, sMoveRamp);
	
	/* 全向分配算法 */
	CHASSIS_omniSpeedCalculate();		
}


/**
 *	@brief 根据按键值设置模式的集总函数
 */
void KEY_setMode(void)
{
	static uint8_t KeyLockFlag_F;
	static uint8_t KeyLockFlag_Ctrl;
	static uint8_t KeyLockFlag_V;
	static uint8_t MouseLockFlag_R;
	
	static portTickType KeyCurTime;
	static portTickType KeyResTime_Ctrl_V;	// Ctrl+V 响应时间
	static portTickType KeyResTime_Ctrl_F;	// Ctrl+F 响应时间
	
	KeyCurTime = xTaskGetTickCount();
	
	/* 右键 */
	if(IF_MOUSE_PRESSED_RIGH) 
	{
		if(MouseLockFlag_R == false && GIMBAL_ifBuffMode() == false) 
		{
			//云台->自瞄
			GIMBAL_setMode(GIMBAL_MODE_AUTO);
			Gimbal.Auto.FLAG_first_into_auto = true;
			//视觉->自瞄
			VISION_setMode(VISION_MODE_AUTO);
			//拨盘->自瞄
			REVOLVER_setAction(SHOOT_AUTO);
		}
		
		MouseLockFlag_R = true;
	}
	else
	{
		//退出自瞄模式
		if(GIMBAL_ifAutoMode() == true)
		{
			//云台->常规
			GIMBAL_setMode(GIMBAL_MODE_NORMAL);
			Gimbal_PID[GYRO][YAW_205].Angle.target = Gimbal_PID[GYRO][YAW_205].Angle.feedback;
			Gimbal_PID[GYRO][PITCH_206].Angle.target = Gimbal_PID[GYRO][PITCH_206].Angle.feedback;
			//视觉->手动
			VISION_setMode(VISION_MODE_MANUAL);
			//拨盘->常规
			REVOLVER_setAction(SHOOT_NORMAL);
		}
		
		MouseLockFlag_R = false;
	}
	
	/* Ctrl */
	if(IF_KEY_PRESSED_CTRL)
	{
		/* Ctrl+W */
		if(IF_KEY_PRESSED_W) 
		{
			CHASSIS_setMode(CHAS_MODE_SZUPUP);
		}
		
		/* Ctrl+R */
		if(IF_KEY_PRESSED_R) 
		{
			//云台
			GIMBAL_setMode(GIMBAL_MODE_RELOAD_BULLET);
			//底盘
			CHASSIS_setMode(CHAS_MODE_RELOAD_BULLET);
		}
		
		if(KeyLockFlag_Ctrl == false)
		{
			//云台->常规
			GIMBAL_setMode(GIMBAL_MODE_NORMAL);
			Flag.Gimbal.FLAG_pidMode = MECH;
			GIMBAL_keyGyro_To_keyMech();
			//视觉->手动
			VISION_setMode(VISION_MODE_MANUAL);
			//拨盘->常规
			REVOLVER_setAction(SHOOT_NORMAL);
		}

		if(KeyResTime_Ctrl_V < KeyCurTime)
		{
			KeyResTime_Ctrl_V = KeyCurTime + TIME_STAMP_400MS;
			/* Ctrl+V */
			if(IF_KEY_PRESSED_V)
			{
				if(Gimbal.State.mode != GIMBAL_MODE_SMALL_BUFF)
				{
					//云台->打小符
					GIMBAL_setMode(GIMBAL_MODE_SMALL_BUFF);
					Gimbal.Buff.FLAG_first_into_buff = true;
					Flag.Gimbal.FLAG_pidMode = GYRO;
					GIMBAL_keyMech_To_keyGyro();
					//视觉->打小符
					VISION_setMode(VISION_MODE_SMALL_BUFF);
					//底盘->打符
					CHASSIS_setMode(CHAS_MODE_BUFF);
				}
			}
		}
		
		if(KeyResTime_Ctrl_F < KeyCurTime)
		{
			/* Ctrl+F */
			if(IF_KEY_PRESSED_F)
			{
				KeyResTime_Ctrl_F = KeyCurTime + TIME_STAMP_400MS;
				/* Ctrl+F */
				if(IF_KEY_PRESSED_F)
				{
					if(Gimbal.State.mode != GIMBAL_MODE_BIG_BUFF)
					{
						//云台->大符
						GIMBAL_setMode(GIMBAL_MODE_BIG_BUFF);
						Gimbal.Buff.FLAG_first_into_buff = true;
						Flag.Gimbal.FLAG_pidMode = GYRO;
						GIMBAL_keyMech_To_keyGyro();
						//视觉->大符
						VISION_setMode(VISION_MODE_BIG_BUFF);
						//底盘->打符
						CHASSIS_setMode(CHAS_MODE_BUFF);
					}
				}
			}
		}
		
		KeyLockFlag_Ctrl = true;
	}
	else
	{
		KeyLockFlag_Ctrl = false;
	}
	
}

/**
 *	@brief	根据按键值设置底盘模式
 */
void KEY_setChassisMode(void)
{
	static uint8_t KeyLockFlag_F = false;
	static uint8_t KeyLockFlag_C = false;
	
	/* Ctrl+W */
	if(IF_KEY_PRESSED_W && IF_KEY_PRESSED_CTRL) {
		CHASSIS_setMode(CHAS_MODE_SZUPUP);
	}
}

/**
 *	@brief	根据按键值设置底盘小陀螺模式
 */
void KEY_setTopGyro(void)
{
	static uint8_t KeyLockFlag_F = false;
	
	if(IF_KEY_PRESSED_F) {
		if(KeyLockFlag_F == false) {
			if(CHASSIS_ifGyroMode() == true) {
				if(Chassis.top_gyro == false) {
					Chassis_Z_PID.Angle.target = Chassis_Z_PID.Angle.feedback;	// 更新刚开启小陀螺时的目标角度
					Chassis.top_gyro = true;	// 开陀螺
					Chassis.twist = false;		// 关扭腰
					top_gyro_dir = -1;	// 后面可设置随机数来使得小陀螺方向随机
				}
				else {
					Chassis.top_gyro = false;
				}
			}
		}
		KeyLockFlag_F = true;
	} else {
		KeyLockFlag_F = false;
	}	
}

/**
 *	@brief	根据按键值设置底盘扭腰模式
 */
void KEY_setTwist(void)
{
	static uint8_t KeyLockFlag_C = false;
	
	if(IF_KEY_PRESSED_C) {
		if(KeyLockFlag_C == false) {
			if(CHASSIS_ifGyroMode() == true) {
				if(Chassis.twist == false) {
					Chassis_Z_PID.Angle.target = Chassis_Z_PID.Angle.feedback;	// 更新刚开启扭腰时的目标角度
					Chassis.twist = true;			// 开扭腰
					Chassis.top_gyro = false;	// 关陀螺
					twist_dir = -1;	// 后面可设置随机数来使得扭腰方向随机
				}
				else {
					Chassis.twist = false;
				}
			}
		}
		KeyLockFlag_C = true;
	} else {
		KeyLockFlag_C = false;
	}	
}

/* #底盘# -------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	底盘初始化
 */
void CHASSIS_init(void)
{
	CHASSIS_kalmanCreate();	// 创建卡尔曼滤波器
	CHASSIS_initParameter();// 初始化底盘参数
}

/**
 *	@brief	底盘功率限制(期望再分配)
 *	@note		祖传算法。主要是比例的算法,ICRA
 */
void CHASSIS_powerLimit(Chassis_Power_t *power, Chassis_PID_t *pid, Judge_Info_t *judge_info)
{
	float kLimit;
	float totalOutput;
	float remain_J;
	static uint16_t judge_err_cnt;
	
	// 读取缓冲焦耳能量
	remain_J = judge_info->PowerHeatData.chassis_power_buffer;	
	totalOutput = abs(pid[LEFT_FRON_201].Out) + 
								abs(pid[RIGH_FRON_202].Out) + 
								abs(pid[LEFT_BACK_203].Out) + 
								abs(pid[RIGH_BACK_204].Out);
	
	if(judge_info->data_valid == false) {
		judge_err_cnt++;
		if(judge_err_cnt > 100) {
			power->currentLimit = 9000;	// 最大的1/4
		}
	} else {
		judge_err_cnt = 0;
		// 剩余焦耳量过小,开始限制输出,限制系数为平方关系
		if(remain_J < WARNING_REMAIN_POWER) {
			kLimit = (float)(remain_J / WARNING_REMAIN_POWER)
						* (float)(remain_J / WARNING_REMAIN_POWER);
			power->currentLimit =  kLimit * power->maxLimit;
		} else {	// 焦耳能量恢复到一定数值
			power->currentLimit = power->maxLimit;
		}
	}
	
	if(totalOutput > power->currentLimit) {
		pid[LEFT_FRON_201].Out = (int16_t)(pid[LEFT_FRON_201].Out / totalOutput * power->currentLimit);
		pid[RIGH_FRON_202].Out = (int16_t)(pid[RIGH_FRON_202].Out / totalOutput * power->currentLimit);
		pid[LEFT_BACK_203].Out = (int16_t)(pid[LEFT_BACK_203].Out / totalOutput * power->currentLimit);
		pid[RIGH_BACK_204].Out = (int16_t)(pid[RIGH_BACK_204].Out / totalOutput * power->currentLimit);	
	}
}

/**
 *	@brief	底盘扭腰粗糙版
 */
float CHASSIS_twistTargetCalculate(int16_t maxTarget, int16_t rampTarget)
{
	static uint8_t dir = 1;
	static float target_z_speed = 0;
	static portTickType tickTime_prev = 0;
	static portTickType tickTime_now = 0;	

	tickTime_now = xTaskGetTickCount();
	if(tickTime_now  - tickTime_prev > TIME_STAMP_200MS) {	// 超时未进入扭腰则重置数据
		dir = 1;
		target_z_speed = 0;
	}
	tickTime_prev = tickTime_now;
	
	if(target_z_speed >= maxTarget) {
		dir = 0;
	} else if(target_z_speed <= -maxTarget) {
		dir = 1;
	}
	
	if(dir == 1) {
		target_z_speed = RAMP_float(maxTarget, target_z_speed, rampTarget);
	} else if(dir == 0) {
		target_z_speed = RAMP_float((-maxTarget), target_z_speed, rampTarget);
	}
	return target_z_speed;
}

/**
 *	@brief	底盘全向运动算法
 */
void CHASSIS_omniSpeedCalculate(void)
{
	float kx, ky, kz;
	/* 计算当前总期望速度 */
	Chas_Target_Speed[ ALL ] = abs(Chas_Target_Speed[ X ]) + abs(Chas_Target_Speed[ Y ]) + abs(Chas_Target_Speed[ Z ]);
	
	kx = Chas_Target_Speed[ X ]/Chas_Target_Speed[ ALL ];
	ky = Chas_Target_Speed[ Y ]/Chas_Target_Speed[ ALL ];
	kz = Chas_Target_Speed[ Z ]/Chas_Target_Speed[ ALL ];
	
	if(Chas_Target_Speed[ ALL ] < Chas_Standard_Move_Max) {
		Chassis_PID[LEFT_FRON_201].Speed.target = (+Chas_Target_Speed[ X ] + Chas_Target_Speed[ Y ] + Chas_Target_Speed[ Z ]);
		Chassis_PID[RIGH_FRON_202].Speed.target = (+Chas_Target_Speed[ X ] - Chas_Target_Speed[ Y ] + Chas_Target_Speed[ Z ]);
		Chassis_PID[LEFT_BACK_203].Speed.target = (-Chas_Target_Speed[ X ] + Chas_Target_Speed[ Y ] + Chas_Target_Speed[ Z ]);
		Chassis_PID[RIGH_BACK_204].Speed.target = (-Chas_Target_Speed[ X ] - Chas_Target_Speed[ Y ] + Chas_Target_Speed[ Z ]);
	} else {
		Chassis_PID[LEFT_FRON_201].Speed.target = (+kx + ky + kz) * Chas_Standard_Move_Max;
		Chassis_PID[RIGH_FRON_202].Speed.target = (+kx - ky + kz) * Chas_Standard_Move_Max;
		Chassis_PID[LEFT_BACK_203].Speed.target = (-kx + ky + kz) * Chas_Standard_Move_Max;
		Chassis_PID[RIGH_BACK_204].Speed.target = (-kx - ky + kz) * Chas_Standard_Move_Max;
	}
}

/**
 *	@brief	底盘定位算法
 */
void CHASSIS_locateDis(void)
{

}

/**
 *	@brief	机械模式下云台YAW期望值(累加值)边界处理
 */
float CHASSIS_MECH_yawTargetBoundaryProcess(Chassis_Z_PID_t *pid, float delta_target)
{
	float target;
	target = pid->Angle.target + delta_target;

	if(target > 8191.f) {
		target = (target - 8191.f);
	} else if(target < 0.f) {
		target = (target + 8191.f);
	}
	
	return target;
}


/**
 *	@brief	底盘常规控制
 */
void CHASSIS_normalControl(void)
{
	KEY_setChassisMode();
	KEY_setTopGyro();
	KEY_setTwist();
	KEY_setChassisSpeed(SPIN_MAX_NORMAL, STANDARD_MAX_NORMAL, TIME_INC_NORMAL);
}

/**
 *	@brief	底盘打符时候的控制
 *	@note		打符的时候底盘不动
 */
void CHASSIS_buffControl(void)
{
	/*----期望修改----*/
	Chassis_PID[LEFT_FRON_201].Speed.target = 0;
	Chassis_PID[RIGH_FRON_202].Speed.target = 0;
	Chassis_PID[LEFT_BACK_203].Speed.target = 0;
	Chassis_PID[RIGH_BACK_204].Speed.target = 0;	
}

/**
 *	@brief	手动爬坡控制
 */
void CHASSIS_szupupControl(void)
{
	/* 松开W或者Ctrl */
	if(!IF_KEY_PRESSED_W || !IF_KEY_PRESSED_CTRL) {
		CHASSIS_setMode(CHAS_MODE_NORMAL);
	}
	
	KEY_setChassisSpeed(SPIN_MAX_SZUPUP, STANDARD_MAX_SZUPUP, TIME_INC_SZUPUP);
}

/**
 *	@brief	自动对位控制
 *	@note		x方向距离受大小补给站影响
 */
#define R_B_CASE	2

#define RELOAD_BULLET_DIS_Y		200	// 单位mm

bool Reload_Bullet_Init = false;
float ramp_angle_y = 1024;
void CHASSIS_reloadBulletControl(void)
{
	// 操作手先将车开至左上方顶住一个角
	#if (R_B_CASE == 1)
		// 无反馈定速延时模型
		/*
				if(tx) {
					tx--;
					tar_x = spd_x, 
				} else {
					tar_x = 0;
				}
	
				if(ty) {
					ty--;
					tar_y = spd_y;
				} else {
					tar_y = 0;
				}
		*/
		static uint16_t tx, ty;
	
		if(Reload_Bullet_Init == true) {
			Reload_Bullet_Init = false;
			tx = TIME_STAMP_500MS/2;	// 控制周期为2ms
			ty = TIME_STAMP_500MS/2;	// 控制周期为2ms
		}
		
		if(tx) {
			tx--;
			Chas_Target_Speed[ X ] = 3000;
		} else {
			Chas_Target_Speed[ X ] = 0;
		}

		if(ty) {
			ty--;
			Chas_Target_Speed[ Y ] = 3000;
		} else {
			Chas_Target_Speed[ Y ] = 0;
		}
		
		/* 全向分配算法 */
		CHASSIS_omniSpeedCalculate();	
		
	#elif (R_B_CASE == 2)
		// 单反馈定速延时模型
		/*
				在竖直方向y上，利用电机机械角度和麦轮半径可以定位
				在水平方向x上，给定速度+延时
		*/
		/* 
			Branch_1
				将四个电机的反馈角度取平均作为反馈值，这样的话只有一个外环pid控制器.
				tar_angle_sum_y - fed_angle_sum_y = err_y -> PID -> Chas_Target_Speed[ Y ]
				tx 定时 -> Chas_Target_Speed[ X ]
				底盘全向分配算法
		*/
		static uint16_t tx;
		float tar_angle_sum_y = -((RELOAD_BULLET_DIS_Y / PERIMETER) * 8192) / CHASSIS_DECELE_RATIO;	// 倒退取负
		float fed_angle_sum_y;
		
		if(Reload_Bullet_Init == true) {
			Reload_Bullet_Init = false;
			/* Y */
			g_Chassis_Motor_Info[LEFT_FRON_201].angle_sum = 0;
			g_Chassis_Motor_Info[RIGH_FRON_202].angle_sum = 0;
			g_Chassis_Motor_Info[LEFT_BACK_203].angle_sum = 0;
			g_Chassis_Motor_Info[RIGH_BACK_204].angle_sum = 0;
			Chassis_PID[LEFT_FRON_201].Angle.feedback = 0;
			Chassis_PID[RIGH_FRON_202].Angle.feedback = 0;
			Chassis_PID[LEFT_BACK_203].Angle.feedback = 0;
			Chassis_PID[RIGH_BACK_204].Angle.feedback = 0;
			Chas_Locate_PID[ Y ].target = 0;
			/* X */
			tx = TIME_STAMP_500MS/2;	// 控制周期2ms
		}

		/* X */
		if(tx) {
			tx--;
			Chas_Target_Speed[ X ] = 3000;
		} else {
			Chas_Target_Speed[ X ] = 0;
		}
		
		/* Y */
		fed_angle_sum_y = (Chassis_PID[LEFT_FRON_201].Angle.feedback
											+ Chassis_PID[RIGH_FRON_202].Angle.feedback
											+ Chassis_PID[LEFT_BACK_203].Angle.feedback
											+ Chassis_PID[RIGH_BACK_204].Angle.feedback)/4;
		
		Chas_Locate_PID[ Y ].target = RAMP_float(tar_angle_sum_y, Chas_Locate_PID[ Y ].target, ramp_angle_y);
		Chas_Locate_PID[ Y ].feedback = fed_angle_sum_y;
		Chas_Locate_PID[ Y ].erro = Chas_Locate_PID[ Y ].target - Chas_Locate_PID[ Y ].feedback;
		Chas_Locate_PID[ Y ].pout = Chas_Locate_PID[ Y ].kp * Chas_Locate_PID[ Y ].erro;
		Chas_Locate_PID[ Y ].out = constrain(Chas_Locate_PID[ Y ].pout, -CHASSIS_PID_OUT_MAX, CHASSIS_PID_OUT_MAX);
		
		Chas_Target_Speed[ Y ] = Chas_Locate_PID[ Y ].out;
		
		/* 全向分配算法 */
		CHASSIS_omniSpeedCalculate();	
		
	#elif (R_B_CASE == 3)
		// 单反馈半自动对位模型
		/*
				
		*/
	#elif (R_B_CASE == 4)
		// 双反馈PID控制模型
	#else
	
	#endif
}

/* #任务层# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	pid控制器最终输出
 */
void CHASSIS_pidControlTask(void)
{
	/* PID速度环计算 */
	CHASSIS_Speed_pidCalculate(Chassis_PID, LEFT_FRON_201); 	// 左前 - 速度环
	CHASSIS_Speed_pidCalculate(Chassis_PID, RIGH_FRON_202); 	// 右前 - 速度环
	CHASSIS_Speed_pidCalculate(Chassis_PID, LEFT_BACK_203); 	// 左后 - 速度环
	CHASSIS_Speed_pidCalculate(Chassis_PID, RIGH_BACK_204); 	// 右后 - 速度环
	/* 功率限制 */
	//CHASSIS_powerLimit(&Chassis_Power, Chassis_PID, &Judge_Info);
	/* 最终输出 */
	CHASSIS_pidOut(Chassis_PID);	
}

/**
 *	@brief	遥控控制底盘任务
 */
void CHASSIS_rcControlTask(void)
{
	//REMOTE_setChassisSpeed();
	REMOTE_TOP_setChassisSpeed();
	REMOTE_setTopGyro();
}

/**
 *	@brief	键盘控制底盘任务
 */
void CHASSIS_keyControlTask(void)
{	
	switch(Chassis.mode)
	{
		case CHAS_MODE_NORMAL:
			CHASSIS_normalControl();
			break;
		case CHAS_MODE_BUFF:
			CHASSIS_buffControl();
			break;
		case CHAS_MODE_RELOAD_BULLET:
			CHASSIS_reloadBulletControl();
			break;
		case CHAS_MODE_SZUPUP:
			CHASSIS_szupupControl();
			break;
		default:
			break;
	}
}

/**
 *	@brief	底盘失控保护
 */
void CHASSIS_selfProtect(void)
{
	CHASSIS_stop(Chassis_PID);
	CHASSIS_PID_ParamsInit(Chassis_PID, CHASSIS_MOTOR_COUNT);
	CHASSIS_Z_PID_ParamsInit(&Chassis_Z_PID.Angle);	
}

/**
 *	@brief	底盘控制任务
 */
void CHASSIS_control(void)
{
	/*----信息读入----*/
	CHASSIS_getInfo();
	/*----期望修改----*/
	if(Flag.Remote.FLAG_mode == RC) {
		CHASSIS_rcControlTask();
	} 
	else if(Flag.Remote.FLAG_mode == KEY) {
		CHASSIS_keyControlTask();
	}
	/*----最终输出----*/
	CHASSIS_pidControlTask();
}
