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

/* 斜坡 */
#define TIME_INC_NORMAL	2
#define TIME_DEC_NORMAL	500

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
uint16_t Time_IncSaltation;	// 前后方向突变斜坡增加量

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
float kKey_Mech_Chas_Standard, kKey_Mech_Chas_Revolve;//平移，旋转

//陀螺仪模式下底盘比例系数,控制键盘斜坡变化率
float kKey_Gyro_Chas_Standard, kKey_Gyro_Chas_Revolve;//平移，旋转

//小陀螺模式下底盘比例系数
float k_Gyro_Chas_Top;

//小陀螺旋转转速步长
float Chas_Top_Gyro_Step;

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
		.target = GIMBAL_MECH_YAW_ANGLE_MID_LIMIT,
		.feedback = GIMBAL_MECH_YAW_ANGLE_MID_LIMIT,
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
	kRc_Gyro_Chas_Spin = -4.8;	// (反向)跟随
	
	/* 小陀螺底盘系数 */
	k_Gyro_Chas_Top = -4.8;
	
	/* 小陀螺转速步长 */
	Chas_Top_Gyro_Step = 10;

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
 *	@brief	底盘电机位置环
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
 *				将当前YAW机械角和中值YAW机械角的差值作为误差送进 Z_Speed PID控制器。
 *				反馈期望的角速度
 */
float CHASSIS_Z_Speed_pidCalculate(PID_Object_t *pid, float kp)
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
void CHASSIS_PID_out(Chassis_PID_t *pid)
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
	
	CAN1_send(0x200, pidOut);
}

/* #信息层# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	底盘获取pid模式信息
 */
void CHASSIS_getInfo(void)
{
	static float last_yaw, delta_yaw;
	float yaw;
	
	/* # Yaw # */
	yaw = Mpu_Info.yaw;
	delta_yaw = yaw - last_yaw;
	if(delta_yaw > +180.f) {
		delta_yaw = -360.f + delta_yaw;
	} else if(delta_yaw < -180.f) {
		delta_yaw = +360.f + delta_yaw;
	}
	last_yaw = yaw; 	
	
	/* 利用陀螺仪数据作扭头补偿 */
	if(CHASSIS_ifTopGyroOpen() == true) {
		Chassis_Z_PID.Angle.target = CHASSIS_MECH_yawTargetBoundaryProcess(&Chassis_Z_PID, 		
																	delta_yaw * 8192 / 360.f);
	}
	
	if(GIMBAL_ifMechMode()) {
		Chassis.pid_mode = MECH;
	} 
	else if(GIMBAL_ifGyroMode()) {
		Chassis.pid_mode = GYRO;
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
//		CHASSIS_setMode(CHAS_MODE_SLOW);
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
		return GIMBAL_TOP_YAW_ANGLE_MID_LIMIT;
	}
	else {
		return GIMBAL_REVERT_YAW_ANGLE_MID_LIMIT;
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
		Chas_Target_Speed[ Z ] = CHASSIS_Z_Speed_pidCalculate(&Chassis_Z_PID.Angle, (YAW_DIR) * kRc_Gyro_Chas_Spin);
		Chas_Target_Speed[ Z ] = constrain(Chas_Target_Speed[ Z ], -Chas_Spin_Move_Max, Chas_Spin_Move_Max);
	}
	
	/* 计算旋转因子(通过调整进入条件来设置旋转和平移的分配比例) */
	if(abs(Chas_Target_Speed[ Z ]) > 800) {	// 扭头速度越快，平移速度越慢
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
		
		/* 小陀螺缓冲池清零 */
		top_gyro_dir = 0;
		Chassis_Z_PID.AngleRampTarget = 0;
		Chassis_Z_PID.AngleRampFeedback = 0;
	}
	/* 陀螺仪模式 */
	else if(CHASSIS_ifGyroMode() == true) {

		Chas_Target_Speed[ X ] = RC_LEFT_CH_LR_VALUE * kRc_Gyro_Chas_Standard;
		Chas_Target_Speed[ X ] = constrain(Chas_Target_Speed[ X ], -Chas_Standard_Move_Max, Chas_Standard_Move_Max);
		
		Chas_Target_Speed[ Y ] = RC_LEFT_CH_UD_VALUE * kRc_Gyro_Chas_Standard;
		Chas_Target_Speed[ Y ] = constrain(Chas_Target_Speed[ Y ], -Chas_Standard_Move_Max, Chas_Standard_Move_Max);
		
		/* 开启了小陀螺 */
		if(CHASSIS_ifTopGyroOpen() == true) {
			top_gyro_dir = -1;
			Chassis_Z_PID.AngleRampTarget += top_gyro_dir * Chas_Top_Gyro_Step;
			if(Chassis_Z_PID.AngleRampFeedback < Chassis_Z_PID.AngleRampTarget) // 正向累加
			{
				Chassis_Z_PID.Angle.target = CHASSIS_MECH_yawTargetBoundaryProcess(&Chassis_Z_PID, Chas_Top_Gyro_Step);
			} 
			else if(Chassis_Z_PID.AngleRampFeedback > Chassis_Z_PID.AngleRampTarget) // 反向累加
			{
				Chassis_Z_PID.Angle.target = CHASSIS_MECH_yawTargetBoundaryProcess(&Chassis_Z_PID, -Chas_Top_Gyro_Step);
			} 
			else // 缓冲池清零
			{
				Chassis_Z_PID.AngleRampTarget = 0;
				Chassis_Z_PID.AngleRampFeedback = 0;
			}			
			/* 斜坡函数给累加期望，防止突然增加很大的期望值 */
			Chassis_Z_PID.AngleRampFeedback = RAMP_float(Chassis_Z_PID.AngleRampTarget, Chassis_Z_PID.AngleRampFeedback, Chas_Top_Gyro_Step);
			
			Chas_Target_Speed[ Z ] = CHASSIS_Z_Speed_pidCalculate(&Chassis_Z_PID.Angle, (YAW_DIR) * k_Gyro_Chas_Top);
		}
		/* 没开小陀螺 */
		else {
			/* 小陀螺缓冲池清零 */
			top_gyro_dir = 0;
			Chassis_Z_PID.AngleRampTarget = 0;
			Chassis_Z_PID.AngleRampFeedback = 0;
			
			Chassis_Z_PID.Angle.target = CHASSIS_getMiddleAngle() ;	// 恢复底盘跟随
			Chas_Target_Speed[ Z ] = CHASSIS_Z_Speed_pidCalculate(&Chassis_Z_PID.Angle, (YAW_DIR) * kRc_Gyro_Chas_Spin);
			
		}
		
		/* Z方向速度限幅 */
		Chas_Target_Speed[ Z ] = constrain(Chas_Target_Speed[ Z ], -Chas_Spin_Move_Max, Chas_Spin_Move_Max);
	}
	
	/* 陀螺仪模式下的坐标系转换 */
	if(CHASSIS_ifGyroMode() == true) {
		/* 计算偏差机械中值的角度差 */
		delta_angle = GIMBAL_getTopGyroAngleOffset() * MECH_2_ANGLE_RADIAN;	//  (8192.f-GIMBAL_getTopGyroAngleOffset()) * MECH_2_ANGLE_RADIAN
		target_speed_x = Chas_Target_Speed[ X ];
		target_speed_y = Chas_Target_Speed[ Y ];
		/* 旋转矩阵 将云台坐标系的期望速度 转换到 底盘坐标系的期望速度 */
		Chas_Target_Speed[ X ] = cos(delta_angle) * target_speed_x - sin(delta_angle) * target_speed_y;
		Chas_Target_Speed[ Y ] = sin(delta_angle) * target_speed_x + cos(delta_angle) * target_speed_y;
	}
	
	/* 计算旋转因子(通过调整进入条件来设置旋转和平移的分配比例) */
	if(abs(Chas_Target_Speed[ Z ]) > 800) {	// 扭头速度越快，平移速度越慢
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
				top_gyro_dir = 0;
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
uint8_t keyFLockFlag = 0;

float rotate_ratio_fr;
float rotate_ratio_fl;
float rotate_ratio_bl;
float rotate_ratio_br;
float wheel_ratio_rpm;

void KEY_setChassisSpeed(RC_Ctl_t *remoteInfo, int16_t sMoveMax, int16_t sMoveRamp)
{
	float k_rc_z;
	static int16_t timeYFront, timeYBack, timeXLeft, timeXRight;
	float target_speed_x, target_speed_y;
	
	Chas_Standard_Move_Max = sMoveMax;
	Time_Inc_Normal = sMoveRamp;
	
	if(CHASSIS_ifMechMode() == true) {
		top_gyro_dir = 0;
		CHASSIS_setMode(CHAS_MODE_NORMAL);
	}
	else if(CHASSIS_ifGyroMode() == true) {
		if(Flag.Chassis.FLAG_goHome == true)
		{
			//保持底盘状态
			//屏蔽底盘跟随
			//修改底盘逻辑
		}		
		
		/* 开启了小陀螺 */
		if(CHASSIS_ifTopGyroOpen() == true) {
			Chassis_Z_PID.AngleRampTarget += top_gyro_dir * Chas_Top_Gyro_Step;
			if(Chassis_Z_PID.AngleRampFeedback < Chassis_Z_PID.AngleRampTarget) // 正向累加
			{
				Chassis_Z_PID.Angle.target = CHASSIS_MECH_yawTargetBoundaryProcess(&Chassis_Z_PID, Chas_Top_Gyro_Step);
			} 
			else if(Chassis_Z_PID.AngleRampFeedback > Chassis_Z_PID.AngleRampTarget) // 反向累加
			{
				Chassis_Z_PID.Angle.target = CHASSIS_MECH_yawTargetBoundaryProcess(&Chassis_Z_PID, -Chas_Top_Gyro_Step);
			} 
			else // 缓冲池清零
			{
				Chassis_Z_PID.AngleRampTarget = 0;
				Chassis_Z_PID.AngleRampFeedback = 0;
			}			
			/* 斜坡函数给累加期望，防止突然增加很大的期望值 */
			Chassis_Z_PID.AngleRampFeedback = RAMP_float(Chassis_Z_PID.AngleRampTarget, Chassis_Z_PID.AngleRampFeedback, Chas_Top_Gyro_Step);
			
			Chas_Target_Speed[ Z ] = CHASSIS_Z_Speed_pidCalculate(&Chassis_Z_PID.Angle, (YAW_DIR) * k_Gyro_Chas_Top);
		}
		/* 没开小陀螺 */
		else {
			/* 小陀螺缓冲池清零 */
			top_gyro_dir = 0;
			Chassis_Z_PID.AngleRampTarget = 0;
			Chassis_Z_PID.AngleRampFeedback = 0;
			
			if(Flag.Chassis.FLAG_goHome == true){
				Chassis.logic = (Chassis_Logic_Names_t)!Chassis.logic;
			}
			
			/* 设置底盘跟随逻辑 */
			Chassis_Z_PID.Angle.target = CHASSIS_getMiddleAngle() ;	
			
			if(Flag.Chassis.FLAG_goHome == false)
				Chas_Target_Speed[ Z ] = CHASSIS_Z_Speed_pidCalculate(&Chassis_Z_PID.Angle, (YAW_DIR) * kRc_Gyro_Chas_Spin);
			else 
				Chas_Target_Speed[ Z ] = 0;
		}
		
		
		/* Z方向速度限幅 */
		Chas_Target_Speed[ Z ] = constrain(Chas_Target_Speed[ Z ], -Chas_Spin_Move_Max, Chas_Spin_Move_Max);
	}
	
	/* 计算旋转因子(通过调整进入条件来设置旋转和平移的分配比例) */
	if(abs(Chas_Target_Speed[ Z ]) > 800) {
		k_rc_z = ((Chas_Spin_Move_Max - (abs(Chas_Target_Speed[ Z ]) - 800)) * (Chas_Spin_Move_Max - (abs(Chas_Target_Speed[ Z ]) - 800)))
							/ (Chas_Spin_Move_Max * Chas_Spin_Move_Max);
		
		k_rc_z = constrain(k_rc_z, 0.f, 1.f);
	} else {
		k_rc_z = 1.f;
	}		
	
	if(IF_KEY_PRESSED_W) {
		timeYBack = 0;	// 后退斜坡清零
	} 
	if(IF_KEY_PRESSED_S) {
		timeYFront = 0;	// 前进斜坡清零
	}
	if(IF_KEY_PRESSED_A) {
		timeXRight = 0;	// 右移斜坡清零
	}
	if(IF_KEY_PRESSED_D) {
		timeXLeft = 0;	// 左移斜坡清零
	}	
	
	/* 全向斜坡量计算 */
	Chas_Slope_Move_Fron = Chas_Standard_Move_Max * KEY_rampChassisSpeed(IF_KEY_PRESSED_W, &timeYFront, Time_Inc_Normal, TIME_DEC_NORMAL);
	Chas_Slope_Move_Back = Chas_Standard_Move_Max * KEY_rampChassisSpeed(IF_KEY_PRESSED_S, &timeYBack,  Time_Inc_Normal, TIME_DEC_NORMAL);
	Chas_Slope_Move_Left = Chas_Standard_Move_Max * KEY_rampChassisSpeed(IF_KEY_PRESSED_A, &timeXLeft,  Time_Inc_Normal, TIME_DEC_NORMAL);
	Chas_Slope_Move_Righ = Chas_Standard_Move_Max * KEY_rampChassisSpeed(IF_KEY_PRESSED_D, &timeXRight, Time_Inc_Normal, TIME_DEC_NORMAL);
	
	/* 期望速度计算 */
	Chas_Target_Speed[ Y ] = (Chas_Slope_Move_Fron - Chas_Slope_Move_Back)*k_rc_z;	
	Chas_Target_Speed[ X ] = (Chas_Slope_Move_Righ - Chas_Slope_Move_Left)*k_rc_z;
	
	/* 陀螺仪模式下的坐标系转换 */
	if(CHASSIS_ifGyroMode() == true) {
		/* 计算偏差机械中值的角度差 */
		delta_angle = GIMBAL_getTopGyroAngleOffset() * MECH_2_ANGLE_RADIAN;	//  (8192.f-GIMBAL_getTopGyroAngleOffset()) * MECH_2_ANGLE_RADIAN
		target_speed_x = Chas_Target_Speed[ X ];
		target_speed_y = Chas_Target_Speed[ Y ];
		if(Flag.Chassis.FLAG_goHome==true)
			delta_angle=0;
		/* 旋转矩阵 将云台坐标系的期望速度 转换到 底盘坐标系的期望速度 */
		Chas_Target_Speed[ X ] = cos(delta_angle) * target_speed_x - sin(delta_angle) * target_speed_y;
		Chas_Target_Speed[ Y ] = sin(delta_angle) * target_speed_x + cos(delta_angle) * target_speed_y;
	}
	
	/* 计算当前总期望速度 */
	Chas_Target_Speed[ ALL ] = abs(Chas_Target_Speed[ X ]) + abs(Chas_Target_Speed[ Y ]) + abs(Chas_Target_Speed[ Z ]);
	
	/* 全向分配算法 */
	CHASSIS_omniSpeedCalculate();		
}

/**
 *	@brief	根据按键值设置云台小陀螺模式
 */
void KEY_setTopGyro(void)
{
	static uint8_t keyFLockFlag = false;
	
	if(IF_KEY_PRESSED_F) {
		if(keyFLockFlag == false) {
			if(CHASSIS_ifGyroMode() == true) {
				if(Chassis.top_gyro == false) {
					Chassis_Z_PID.Angle.target = Chassis_Z_PID.Angle.feedback;
					Chassis.top_gyro = true;
					top_gyro_dir = -1;
				}
				else {
					Chassis.top_gyro = false;
					top_gyro_dir = 0;
				}
			}
		}
		keyFLockFlag = true;
	} else {
		keyFLockFlag = false;
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
	KEY_setChassisSpeed(&RC_Ctl_Info, Chas_Standard_Move_Max, TIME_INC_NORMAL);
	KEY_setTopGyro();
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
	CHASSIS_PID_out(Chassis_PID);	
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
		case CHAS_MODE_TWIST:
			break;
		case CHAS_MODE_BUFF:
			CHASSIS_buffControl();
			break;
		case CHAS_MODE_SLOW:
			break;
		case CHAS_MODE_SZUPUP:
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
	//..can.c中
	CHASSIS_getInfo();
	/*----期望修改----*/
	if(Flag.Remote.FLAG_mode == RC) {
		CHASSIS_rcControlTask();
	} else if(Flag.Remote.FLAG_mode == KEY) {
		CHASSIS_keyControlTask();
	}
	/*----最终输出----*/
	CHASSIS_pidControlTask();
}
