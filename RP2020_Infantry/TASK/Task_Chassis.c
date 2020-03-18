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
#include "rng.h"

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#define X	0
#define Y 	1
#define Z 	2
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
#define STANDARD_MAX_NORMAL		CHASSIS_PID_OUT_MAX
#define SPIN_MAX_NORMAL			CHASSIS_PID_OUT_MAX

#define STANDARD_MAX_SZUPUP		3000
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
uint16_t Chas_Top_Gyro_Period = 0;	// 单项赛规则(T为3~7s的随机数)

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
PID_Object_t Chas_Locate_PID[2] = 
{
	{	// X
		.kp = 0,
		.ki = 0,
		.kd = 0,
		.integrate_max = 0,
		.out_max = 3000,
	},
	{	// Y
		.kp = 0,
		.ki = 0,
		.kd = 0,
		.integrate_max = 0,
		.out_max = 3000
	}
};

/**
 *	@brief	底盘信息结构体
 */
Chassis_Info_t Chassis = {
	// 控制方式
	.RemoteMode = RC,
	// Pid模式
	.PidMode = MECH,
	// 模式
	.Mode = CHAS_MODE_NORMAL,
	// 功率
	.Power.max_limit = CHASSIS_MAX_CURRENT_LIMIT,
	.Power.cur_limit = 0,
	// 逻辑
	.Logic = CHAS_LOGIC_NORMAL,
	// 小陀螺开启标志位
	.TopGyro = false,
	// 扭腰开启标志位
	.Twist = false,
};

Chassis_Handler_t hChassis = {
	/* 电机 速度环+角度环 */
	.Pid = {
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
			.Speed.integrate_max = CHASSIS_SPEED_INTEGRATE_MAX,
			.Speed.pout = 0,
			.Speed.iout = 0,
			.Speed.dout = 0,
			.Speed.out = 0,
			.Speed.out_max = CHASSIS_PID_OUT_MAX,
			/* 位置环 */
			.Angle.kp = 0,
			.Angle.ki = 0,
			.Angle.kd = 0,
			.Angle.target = 0,
			.Angle.feedback = 0,
			.Angle.erro = 0,
			.Angle.last_erro = 0,
			.Angle.integrate = 0,
			.Angle.integrate_max = CHASSIS_ANGLE_INTEGRATE_MAX,
			.Angle.pout = 0,
			.Angle.iout = 0,
			.Angle.dout = 0,
			.Angle.out = 0,
			.Angle.out_max = CHASSIS_MAX_SPEED,
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
			.Speed.integrate_max = CHASSIS_SPEED_INTEGRATE_MAX,
			.Speed.pout = 0,
			.Speed.iout = 0,
			.Speed.dout = 0,
			.Speed.out = 0,
			.Speed.out_max = CHASSIS_PID_OUT_MAX,
			/* 位置环 */
			.Angle.kp = 0,
			.Angle.ki = 0,
			.Angle.kd = 0,
			.Angle.target = 0,
			.Angle.feedback = 0,
			.Angle.erro = 0,
			.Angle.last_erro = 0,
			.Angle.integrate = 0,
			.Angle.integrate_max = CHASSIS_ANGLE_INTEGRATE_MAX,
			.Angle.pout = 0,
			.Angle.iout = 0,
			.Angle.dout = 0,
			.Angle.out = 0,
			.Angle.out_max = CHASSIS_MAX_SPEED,	
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
			.Speed.integrate_max = CHASSIS_SPEED_INTEGRATE_MAX,
			.Speed.pout = 0,
			.Speed.iout = 0,
			.Speed.dout = 0,
			.Speed.out = 0,
			.Speed.out_max = CHASSIS_PID_OUT_MAX,
			/* 位置环 */
			.Angle.kp = 1.172,		// 1.15					1.16				1.172		(这套参数适合大角度转动)
			.Angle.ki = 0.7,			// 1						0.82					0.7
			.Angle.kd = 0.0021,		// 0.002				0.0021			0.0021
			.Angle.target = 0,
			.Angle.feedback = 0,
			.Angle.erro = 0,
			.Angle.last_erro = 0,
			.Angle.integrate = 0,
			.Angle.integrate_max = CHASSIS_ANGLE_INTEGRATE_MAX,
			.Angle.pout = 0,
			.Angle.iout = 0,
			.Angle.dout = 0,
			.Angle.out = 0,
			.Angle.out_max = CHASSIS_MAX_SPEED,
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
			.Speed.integrate_max = CHASSIS_SPEED_INTEGRATE_MAX,
			.Speed.pout = 0,
			.Speed.iout = 0,
			.Speed.dout = 0,
			.Speed.out = 0,
			.Speed.out_max = CHASSIS_PID_OUT_MAX,
			/* 位置环 */
			.Angle.kp = 0,
			.Angle.ki = 0,
			.Angle.kd = 0,
			.Angle.target = 0,
			.Angle.feedback = 0,
			.Angle.erro = 0,
			.Angle.last_erro = 0,
			.Angle.integrate = 0,
			.Angle.integrate_max = CHASSIS_ANGLE_INTEGRATE_MAX,
			.Angle.pout = 0,
			.Angle.iout = 0,
			.Angle.dout = 0,
			.Angle.out = 0,
			.Angle.out_max = CHASSIS_MAX_SPEED,	
		},
	},
	
	/* Z角度环 */
	.ZPid = {
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
	},
	
	/* 对位Pid */
	.LocatePid = {
		{	// X
			.kp = 0,
			.ki = 0,
			.kd = 0,
			.integrate_max = 0,
			.out_max = 3000,
		},
		{	// Y
			.kp = 0,
			.ki = 0,
			.kd = 0,
			.integrate_max = 0,
			.out_max = 3000
		}	
	},
	
	.RemoteMode = RC,
	.PidMode = MECH,
	.Mode = CHAS_MODE_NORMAL,
	.Logic = CHAS_LOGIC_NORMAL,
	.TopGyro = false,
	.Twist = false,
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
void CHASSIS_InitParams(void)
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
		pid->target = CHASSIS_GetMiddleAngle();
		pid->feedback = CHASSIS_GetMiddleAngle();
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
void CHASSIS_KalmanCreate(void)
{
	/* 假定机械角度的反馈没有噪声误差 */
	KalmanCreate(&Chassis_Kalman_Error, 1, 0);
}	

/**
 *	@brief	底盘电机更新实时速度
 */
void CHASSIS_UpdateMotorSpeed(Chassis_PID_t *pid, Chassis_Motor_Names MOTORx)
{
	pid[MOTORx].Speed.feedback = g_Chassis_Motor_Info[MOTORx].speed;
}

/**
 *	@brief	底盘电机更新实时角度(累积转过的角度)
 */
void CHASSIS_UpdateMotorAngle(Chassis_PID_t *pid, Chassis_Motor_Names MOTORx)
{
  //pid[MOTORx].Angle.feedback = g_Chassis_Motor_Info[MOTORx].angle;
	pid[MOTORx].Angle.feedback = g_Chassis_Motor_Info[MOTORx].angle_sum;
}

/**
 *	@brief	底盘电机更新实时电流值
 */
void CHASSIS_UpdateMotorCurrent(Chassis_PID_t *pid, Chassis_Motor_Names MOTORx)
{
	//pid[MOTORx].Speed.feedback = g_Chassis_Motor_Info[MOTORx].speed;
}

/**
 *	@brief	底盘电机紧急刹车
 */
void CHASSIS_Stop(Chassis_PID_t *pid)
{
	static int16_t pid_out[4] = {0, 0, 0, 0};
	
	/* 内环速度环最终输出 */
	pid[LEFT_FRON_201].Out = 0;
	pid[RIGH_FRON_202].Out = 0;
	pid[LEFT_BACK_203].Out = 0;
	pid[RIGH_BACK_204].Out = 0;
	
	CAN1_Send(0x200, pid_out);	
}

/**
 *	@brief	底盘电机速度环
 */
void CHASSIS_Speed_PidCalc(Chassis_PID_t *pid, Chassis_Motor_Names MOTORx)
{
	pid[MOTORx].Speed.erro = pid[MOTORx].Speed.target - pid[MOTORx].Speed.feedback;
	pid[MOTORx].Speed.integrate += pid[MOTORx].Speed.erro;
	
	//pid[MOTORx].Speed.integrate = constrain(pid[MOTORx].Speed.integrate, -3000, 3000);
	// #integrate是否限幅?
	
	pid[MOTORx].Speed.integrate = constrain(pid[MOTORx].Speed.integrate, -pid[MOTORx].Speed.integrate_max, pid[MOTORx].Speed.integrate_max);
	
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
	pid[MOTORx].Speed.out = constrain(pid[MOTORx].Speed.out, -pid[MOTORx].Speed.out_max, pid[MOTORx].Speed.out_max);
	pid[MOTORx].Out = pid[MOTORx].Speed.out;
	
	/* test */
	speed_target_203 = pid[LEFT_BACK_203].Speed.target;
	speed_feedback_203 = pid[LEFT_BACK_203].Speed.feedback;
}

///**
// *	@brief	底盘电机速度环
// */
//void CHASSIS_Speed_PidCalcTest(Chassis_PID_t *pid, Chassis_Motor_Names MOTORx)
//{
//	pid[MOTORx].Speed.Err = pid[MOTORx].Speed.Target - pid[MOTORx].Speed.Feedback;
//	pid[MOTORx].Speed.Integrate += pid[MOTORx].Speed.Err;
//	
//	//pid[MOTORx].Speed.integrate = constrain(pid[MOTORx].Speed.integrate, -3000, 3000);
//	// #integrate是否限幅?
//	
//	pid[MOTORx].Speed.Integrate = constrain(pid[MOTORx].Speed.Integrate, -18000, 18000);
//	
//	/* Pout */
//	pid[MOTORx].Speed.Pout = pid[MOTORx].Speed.Kp * pid[MOTORx].Speed.Err;
//	/* Iout */
//	pid[MOTORx].Speed.Iout = pid[MOTORx].Speed.Ki * pid[MOTORx].Speed.Integrate * 0.002f; // 模仿19步兵代码(0.002f是2ms的意思)
//	/* Iout Limits */
//	pid[MOTORx].Speed.Iout = constrain(pid[MOTORx].Speed.Iout, -CHASSIS_SPEED_IOUT_MAX, CHASSIS_SPEED_IOUT_MAX);
//	/* Dout*/
//	pid[MOTORx].Speed.Dout = pid[MOTORx].Speed.Kd * (pid[MOTORx].Speed.Err - pid[MOTORx].Speed.LastErr)/0.002f;
//	/* Record Last Error */
//	pid[MOTORx].Speed.LastErr = pid[MOTORx].Speed.Err;
//	
//	/* 19步兵代码 - 积分项缓慢减小，防止误差为0时突然失力 */
//	if( pid[MOTORx].Speed.Pout * pid[MOTORx].Speed.Iout < 0) {
//		pid[MOTORx].Speed.Integrate = constrain(pid[MOTORx].Speed.Integrate, 
//												-CHASSIS_SPEED_IOUT_MAX/pid[MOTORx].Speed.Ki/5.f,
//												+CHASSIS_SPEED_IOUT_MAX/pid[MOTORx].Speed.Ki/5.f);
//	}
//	
//	/* Total PID Output*/
//	pid[MOTORx].Speed.Out = pid[MOTORx].Speed.Pout + pid[MOTORx].Speed.Iout + pid[MOTORx].Speed.Dout;
//	/* Total PID Output Limits */
//	pid[MOTORx].Speed.out = constrain(pid[MOTORx].Speed.Out, -CHASSIS_PID_OUT_MAX, CHASSIS_PID_OUT_MAX);
//	pid[MOTORx].Out = pid[MOTORx].Speed.Out;
//	
//	/* test */
//	speed_target_203 = pid[LEFT_BACK_203].Speed.target;
//	speed_feedback_203 = pid[LEFT_BACK_203].Speed.feedback;
//}

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
void CHASSIS_Angle_PidCalc(Chassis_PID_t *pid, Chassis_Motor_Names MOTORx)
{
	pid[MOTORx].Angle.erro = pid[MOTORx].Angle.target - pid[MOTORx].Angle.feedback;
	pid[MOTORx].Angle.integrate += pid[MOTORx].Angle.erro;
	pid[MOTORx].Angle.integrate = constrain(pid[MOTORx].Angle.integrate, -pid[MOTORx].Angle.integrate_max, pid[MOTORx].Angle.integrate_max);
	
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
	pid[MOTORx].Angle.out = constrain(pid[MOTORx].Angle.out, -pid[MOTORx].Angle.out_max, pid[MOTORx].Angle.out_max);
	
	/* test */
	angle_target_203 = pid[LEFT_BACK_203].Angle.target;
	angle_feedback_203 = pid[LEFT_BACK_203].Angle.feedback;
}

/**
 *	@brief	底盘陀螺仪模式加上Z方向的速度环(实际上是位置环)
 *	@note
 *			将当前YAW机械角和中值YAW机械角的差值作为误差送进 Z_PID控制器。
 *			反馈期望的角速度 tar_spd_z
 */
float CHASSIS_Z_Angle_PidCalc(PID_Object_t *pid, float kp)
{
	pid->kp = kp;
	pid->erro = pid->target - pid->feedback;
	
	/* 临界处理 */
	
	/*
	
		0 - 8191 

		误差=期望-反馈
	
		误差大于4096
	
		反馈落后于期望 -> 确定期望的方向 0->8191 --》 反馈大于期望
	
		实际误差 = -（8191 - 期望 + 反馈） = -8191 + 误差
	
		误差小于-4096
	
		反馈落后于期望 -> 确定期望方向 8191->0  --》 期望大于反馈
	
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
	if(CHASSIS_IfTopGyroOpen() == false) {
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
 *	@brief	自动对位模式下的PID计算
 */
float CHASSIS_Locate_PidCalc(PID_Object_t* pid)
{
	pid->erro = pid->target - pid->feedback;
	
	pid->integrate += pid->erro;
	
	pid->integrate = constrain(pid->integrate, -pid->integrate_max, pid->integrate_max);
	
	pid->pout = pid->kp * pid->erro;
	
	pid->iout = pid->ki * pid->integrate * 0.002f;	// 2ms控制周期
	
	pid->dout = pid->kd * (pid->erro - pid->last_erro) / 0.002f;	// 2ms控制周期
	
	pid->last_erro = pid->erro;
	
	pid->out = pid->pout + pid->iout + pid->dout;
	
	pid->out = constrain(pid->out, -pid->out_max, pid->out_max);
	
	return pid->out;
}

/**
 *	@brief	底盘累加角度清零
 */
void CHASSIS_Angle_ClearSum(Chassis_PID_t *pid)
{
	/* 累加机械角度值清零 */
	g_Chassis_Motor_Info[LEFT_FRON_201].angle_sum = 0;
	g_Chassis_Motor_Info[RIGH_FRON_202].angle_sum = 0;
	g_Chassis_Motor_Info[LEFT_BACK_203].angle_sum = 0;
	g_Chassis_Motor_Info[RIGH_BACK_204].angle_sum = 0;

	/* 底盘角度环PID反馈清零 */
	pid[LEFT_FRON_201].Angle.feedback = 0;
	pid[RIGH_FRON_202].Angle.feedback = 0;
	pid[LEFT_BACK_203].Angle.feedback = 0;
	pid[RIGH_BACK_204].Angle.feedback = 0;	
}

/**
 *	@brief	底盘电机PID的最终输出
 */
void CHASSIS_PidOut(Chassis_PID_t *pid)
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
	
	CAN1_QueueSend(0x200, pidOut);
}

/* #信息层# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**	
 *	@breif	反馈底盘模式
 */
Chassis_Mode_Names_t CHASSIS_GetMode(void)
{
	return Chassis.Mode;
}

/**	
 *	@breif	反馈底盘逻辑
 */
Chassis_Logic_Names_t CHASSIS_GetLogic(void)
{
	return Chassis.Logic;
}

/**	
 *	@breif	反馈底盘机械中值目标
 */
float CHASSIS_GetMiddleAngle(void)
{
	if(Chassis.Logic == CHAS_LOGIC_NORMAL) {
		return GIMBAL_TOP_YAW_MID_ANGLE;
	}
	else {
		return GIMBAL_REVERT_YAW_MID_ANGLE;
	}
}

/**
 *	@brief	设置底盘的模式
 */
void CHASSIS_SetMode(Chassis_Mode_Names_t mode)
{
	Chassis.Mode = mode;
}

/**
 *	@brief	底盘逻辑取反
 */
void CHASSIS_LogicRevert(void)
{
	if(Chassis.Logic == CHAS_LOGIC_NORMAL) {
		Chassis.Logic = CHAS_LOGIC_REVERT;
	} else {
		Chassis.Logic = CHAS_LOGIC_NORMAL;	
	}
}

/**
 *	@brief	底盘是否处于机械模式
 */
bool CHASSIS_IfMechMode(void)
{
	if(Chassis.PidMode == MECH)
		return true;
	else
		return false;
}

/**
 *	@brief	底盘是否处于陀螺仪模式
 */
bool CHASSIS_IfGyroMode(void)
{
	if(Chassis.PidMode == GYRO)
		return true;
	else
		return false;
}

/**
 *	@brief	底盘是否处于常规模式
 */
bool CHASSIS_IfNormalMode(void)
{
	if(Chassis.Mode == CHAS_MODE_NORMAL)
		return true;
	else 
		return false;
}

/**
 * @brief		底盘是否处于小陀螺模式
 */
bool CHASSIS_IfTopGyroOpen(void)
{
	if(Chassis.PidMode == GYRO
			&& Chassis.TopGyro == true)
		return true;

	return false;
}

/**
 *	@brief	底盘是否处于扭腰模式
 */
bool CHASSIS_IfTwistOpen(void)
{
	if(Chassis.PidMode == GYRO
			&& Chassis.Twist == true)
		return true;
			
	return false;
}

/**	
 *	@breif	底盘逻辑是否取反
 */
bool CHASSIS_IfLogicRevert(void)
{
	static Chassis_Logic_Names_t prev_logic = CHAS_LOGIC_NORMAL;
	
	/* 底盘逻辑取反检测 */
	if(prev_logic != Chassis.Logic) {
		return true;
	}
	prev_logic = Chassis.Logic;
	
	return false;
}

/**
 *	@brief	底盘获取系统信息
 */
void CHASSIS_GetSysInfo(System_t *sys, Chassis_Info_t *chas)
{
	/*----控制方式----*/
	/* 控制方式 - 遥控器 */
	if(sys->RemoteMode == RC) {
		chas->RemoteMode = RC;
	} 
	/* 控制方式 - 键鼠 */
	else if(sys->RemoteMode == KEY) {
		chas->RemoteMode = KEY;
	}
	
	/*----模式修改----*/
	switch(sys->Action)
	{
		case SYS_ACT_NORMAL: 
			{
				// 手动爬坡可在常规模式下进入
				if(chas->Mode != CHAS_MODE_SZUPUP) {
					chas->Mode = CHAS_MODE_NORMAL;
				}
			}break;
		case SYS_ACT_AUTO: 
			{
				// 自瞄时仍为常规模式
				if(chas->Mode != CHAS_MODE_NORMAL) {
				}
				chas->Mode = CHAS_MODE_NORMAL;
			}break;
		case SYS_ACT_BUFF:
			{
				// 刚进入打符模式
				if(chas->Mode != CHAS_MODE_BUFF) {
				}
				chas->Mode = CHAS_MODE_BUFF;
			}break;
		case SYS_ACT_PARK:
			{
				// 刚进入自动对位模式
				if(chas->Mode != CHAS_MODE_RELOAD_BULLET) {
				}
				chas->Mode = CHAS_MODE_RELOAD_BULLET;
			}break;
	}
	
	/*----Pid模式----*/
	if(sys->PidMode != chas->PidMode)
	{
		/* GYRO -> MECH */
		if(sys->PidMode == MECH)
		{
			Chassis.TopGyro = false;	// 关陀螺
			Chassis.Twist = false;		// 关扭腰
		}
		/* MECH -> GYRO*/
		else if(sys->PidMode == GYRO)
		{
		}
	}
	chas->PidMode = sys->PidMode;
}

/**
 *	@brief	底盘读取裁判系统信息
 */
void CHASSIS_GetJudgeInfo(Judge_Info_t *judge, Chassis_Info_t *chas)
{
	if(JUDGE_IfDataValid() == true) {
		//chas->Power.buffer = judge->PowerHeatData.chassis_power_buffer;
	}
	else {
	
	}
}

/**
 *	@brief	底盘读取IMU数据
 */
void CHASSIS_GetImuInfo(Chassis_Z_PID_t *zpid)
{
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
	if(CHASSIS_IfTopGyroOpen() == true) {
		zpid->Angle.target = CHASSIS_MECH_YawBoundaryProc(&Chassis_Z_PID, 		
												yaw[DELTA] * 8192 / 360.f);
	}	
}

/**
 *	@brief	非匀速小陀螺
 */
void CHASSIS_GetTopGyroInfo(void)
{
	static portTickType ulCurrentTime;
	
	ulCurrentTime = xTaskGetTickCount();
	ulCurrentTime %= Chas_Top_Gyro_Period;
	
	if(CHASSIS_IfTopGyroOpen() == true) {
		// -7.5sin(2pi*t/T)+22.5
		Chas_Top_Gyro_Step = -7.5*sin(6.28f*(float)ulCurrentTime/Chas_Top_Gyro_Period) + 22.5;	// 正弦小陀螺
	}
}

/**
 *	@brief	底盘读取遥控数据
 */
static uint8_t RcLockFlag_Wheel = false;
static uint8_t RcUnlockFlag_Wheel = false;
static uint8_t KeyLockFlag_F = false;
static uint8_t KeyLockFlag_C = false;

void CHASSIS_GetRemoteInfo(System_t *sys, RC_Ctl_t *remote, Chassis_Info_t *chas)
{
	/* 系统正常 */
	if(sys->State == SYSTEM_STATE_NORMAL)
	{
		if(sys->RemoteMode == RC) {
			KeyLockFlag_F = false;
			KeyLockFlag_C = false;
		}
		else if(sys->RemoteMode == KEY) {
			RcLockFlag_Wheel = false;
			RcUnlockFlag_Wheel = false;
		}
	}
	/* 系统异常 */
	else
	{
		// 复位成初始值
		KeyLockFlag_F = false;
		KeyLockFlag_C = false;
		RcLockFlag_Wheel = false;
		RcUnlockFlag_Wheel = false;
	}	
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

void REMOTE_SetChassisSpeed(void)
{
	float k_rc_z;

	/* 机械模式 */
	if(Chassis.PidMode == MECH) {
		
		Chas_Target_Speed[ X ] = RC_LEFT_CH_LR_VALUE * kRc_Mech_Chas_Standard;
		Chas_Target_Speed[ X ] = constrain(Chas_Target_Speed[ X ], -Chas_Standard_Move_Max, Chas_Standard_Move_Max);
		
		Chas_Target_Speed[ Y ] = RC_LEFT_CH_UD_VALUE * kRc_Mech_Chas_Standard;
		Chas_Target_Speed[ Y ] = constrain(Chas_Target_Speed[ Y ], -Chas_Standard_Move_Max, Chas_Standard_Move_Max);
		
		Chas_Target_Speed[ Z ] = RC_RIGH_CH_LR_VALUE * kRc_Mech_Chas_Spin;
		Chas_Target_Speed[ Z ] = constrain(Chas_Target_Speed[ Z ], -Chas_Spin_Move_Max, Chas_Spin_Move_Max);
		
	}
	/* 陀螺仪模式 */
	else if(Chassis.PidMode == GYRO) {

		Chas_Target_Speed[ X ] = RC_LEFT_CH_LR_VALUE * kRc_Gyro_Chas_Standard;
		Chas_Target_Speed[ X ] = constrain(Chas_Target_Speed[ X ], -Chas_Standard_Move_Max, Chas_Standard_Move_Max);
		
		Chas_Target_Speed[ Y ] = RC_LEFT_CH_UD_VALUE * kRc_Gyro_Chas_Standard;
		Chas_Target_Speed[ Y ] = constrain(Chas_Target_Speed[ Y ], -Chas_Standard_Move_Max, Chas_Standard_Move_Max);
		
		Chassis_Z_PID.Angle.target = CHASSIS_GetMiddleAngle();	// 恢复底盘跟随
		Chas_Target_Speed[ Z ] = CHASSIS_Z_Angle_PidCalc(&Chassis_Z_PID.Angle, (YAW_DIR) * kRc_Gyro_Chas_Spin);
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
	CHASSIS_OmniSpeedCalc();
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
void REMOTE_TOP_SetChassisSpeed(void)
{
	float k_rc_z;
	float target_speed_x;
	float target_speed_y;
	
	/* 机械模式 */
	if(CHASSIS_IfMechMode() == true) {
		
		Chas_Target_Speed[ X ] = RC_LEFT_CH_LR_VALUE * kRc_Mech_Chas_Standard;
		Chas_Target_Speed[ X ] = constrain(Chas_Target_Speed[ X ], -Chas_Standard_Move_Max, Chas_Standard_Move_Max);
		
		Chas_Target_Speed[ Y ] = RC_LEFT_CH_UD_VALUE * kRc_Mech_Chas_Standard;
		Chas_Target_Speed[ Y ] = constrain(Chas_Target_Speed[ Y ], -Chas_Standard_Move_Max, Chas_Standard_Move_Max);
		
		Chas_Target_Speed[ Z ] = RC_RIGH_CH_LR_VALUE * kRc_Mech_Chas_Spin;
		Chas_Target_Speed[ Z ] = constrain(Chas_Target_Speed[ Z ], -Chas_Spin_Move_Max, Chas_Spin_Move_Max);
		
	}
	/* 陀螺仪模式 */
	else if(CHASSIS_IfGyroMode() == true) {

		Chas_Target_Speed[ X ] = RC_LEFT_CH_LR_VALUE * kRc_Gyro_Chas_Standard;
		Chas_Target_Speed[ X ] = constrain(Chas_Target_Speed[ X ], -Chas_Standard_Move_Max, Chas_Standard_Move_Max);
		
		Chas_Target_Speed[ Y ] = RC_LEFT_CH_UD_VALUE * kRc_Gyro_Chas_Standard;
		Chas_Target_Speed[ Y ] = constrain(Chas_Target_Speed[ Y ], -Chas_Standard_Move_Max, Chas_Standard_Move_Max);
		
		/* 开启了小陀螺 */
		if(CHASSIS_IfTopGyroOpen() == true) {

			// 外环期望斜坡变化
			Chassis_Z_PID.Angle.target = CHASSIS_MECH_YawBoundaryProc(&Chassis_Z_PID, top_gyro_dir * Chas_Top_Gyro_Step);
			// Z方向速度pid计算
			Chas_Target_Speed[ Z ] = CHASSIS_Z_Angle_PidCalc(&Chassis_Z_PID.Angle, (YAW_DIR) * k_Gyro_Chas_Top);
			
		}
		/* 没开小陀螺 */
		else {
			
			// 恢复底盘跟随
			Chassis_Z_PID.Angle.target = CHASSIS_GetMiddleAngle() ;	
			// Z方向速度pid计算
			Chas_Target_Speed[ Z ] = CHASSIS_Z_Angle_PidCalc(&Chassis_Z_PID.Angle, (YAW_DIR) * kRc_Gyro_Chas_Spin);
			
		}
		
		// Z方向速度限幅
		Chas_Target_Speed[ Z ] = constrain(Chas_Target_Speed[ Z ], -Chas_Spin_Move_Max, Chas_Spin_Move_Max);
	}
	
	/* 陀螺仪模式下的坐标系转换 */
	if(CHASSIS_IfGyroMode() == true) {
		
		// 计算偏差机械中值的角度差
		delta_angle = GIMBAL_GetTopGyroAngleOffset() * MECH_2_ANGLE_RADIAN;	//  (8192.f-GIMBAL_GetTopGyroAngleOffset()) * MECH_2_ANGLE_RADIAN
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
	CHASSIS_OmniSpeedCalc();	
}

/**
 *	@brief	根据遥控拨轮设置云台小陀螺模式
 */
void REMOTE_SetTopGyro(void)
{
	/* 拨轮向上开启 */
	if(RC_THUMB_WHEEL_VALUE < -650) {
		if(RcLockFlag_Wheel == false) {
			/* 陀螺仪模式 */
			if(CHASSIS_IfGyroMode() == true) {
				Chassis_Z_PID.Angle.target = Chassis_Z_PID.Angle.feedback;
				Chassis.TopGyro = true;
				Chassis.Twist = false;
				top_gyro_dir = -1;
				//根据单项赛要求设置周期为3~7s
				Chas_Top_Gyro_Period = RNG_ucGetRandomNum(3, 7)*1000;
			}
		}
		RcLockFlag_Wheel = true;
	} else {
		RcLockFlag_Wheel = false;
	}
	
	/* 拨轮向下关闭 */
	if(RC_THUMB_WHEEL_VALUE > 650) {
		if(RcUnlockFlag_Wheel == false) {
			if(CHASSIS_IfGyroMode() == true) {
				Chassis.TopGyro = false;
			}
		}
		RcUnlockFlag_Wheel = true;
	} else {
		RcUnlockFlag_Wheel = false;
	}
}

/**
 *	@brief	按键斜坡函数
 *	@note	长按返回最大系数1
 *			长时未按返回最小系数0
 */
#define KEY_RAMP_SCALE	500
float KEY_RampChasSpeed(int8_t key_state, int16_t *time, uint16_t inc_ramp_step, uint16_t dec_ramp_step)
{
	#if 0
	float fac;

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

	fac = 0.15 * sqrt( 0.15 * (*time) );	// time累加到296.3斜坡就完成
	fac = constrain(fac, 0, 1);
	return fac;
	
	#else
	float fac;
	
	if(key_state == 1) {
		if(fac < 1) {
			*time += inc_ramp_step;
		}
	} else {
		if(fac > 0) {
			*time -= dec_ramp_step;
			if(*time < 0) {
				*time = 0;
			}
		}
	}

	fac = (*time)*(*time) / (KEY_RAMP_SCALE*KEY_RAMP_SCALE);
	fac = constrain(fac, 0, 1);
	return fac;
	#endif
}

/**
 *	@brief	根据按键值设置旋转移动速度
 */
void KEY_SetChasSpinSpeed(int16_t sSpinMax)
{
	Chas_Spin_Move_Max = sSpinMax;

	/* 机械模式 */
	if(CHASSIS_IfMechMode() == true) {
		
		Chas_Target_Speed[ Z ] = MOUSE_X_MOVE_SPEED * kKey_Mech_Chas_Spin;
		Chas_Target_Speed[ Z ] = constrain(Chas_Target_Speed[ Z ], -Chas_Spin_Move_Max, Chas_Spin_Move_Max);
		
	}
	/* 陀螺仪模式 */
	else if(CHASSIS_IfGyroMode() == true) {
		/* 开启了小陀螺 */
		if(CHASSIS_IfTopGyroOpen() == true) {
			
			// 外环期望斜坡变化
			Chassis_Z_PID.Angle.target = CHASSIS_MECH_YawBoundaryProc(&Chassis_Z_PID, top_gyro_dir * Chas_Top_Gyro_Step);
			// Z方向速度pid计算
			Chas_Target_Speed[ Z ] = CHASSIS_Z_Angle_PidCalc(&Chassis_Z_PID.Angle, (YAW_DIR) * k_Gyro_Chas_Top);
			
		}
		/* 开启了扭腰 */
		else if(CHASSIS_IfTwistOpen() == true) {
			
			if(Chassis_Z_PID.Angle.target >= CHASSIS_GetMiddleAngle() + 800) {
				twist_dir = -1;
			} 
			else if(Chassis_Z_PID.Angle.target <= CHASSIS_GetMiddleAngle() - 800) {
				twist_dir = +1;
			}
			Chassis_Z_PID.Angle.target = CHASSIS_MECH_YawBoundaryProc(&Chassis_Z_PID, twist_dir * Chas_Twist_Step);
			
			// Z方向速度pid计算
			Chas_Target_Speed[ Z ] = CHASSIS_Z_Angle_PidCalc(&Chassis_Z_PID.Angle, (YAW_DIR) * k_Gyro_Chas_Twist);
			
		}
		/* 没开小陀螺/扭腰 */
		else {
			if(Flag.Chassis.GoHome == true) {
				/* 保持底盘状态
					 屏蔽底盘跟随
					 修改底盘逻辑 */
				CHASSIS_LogicRevert();// 修改底盘逻辑
			}
			
			/* 设置底盘跟随逻辑 */
			Chassis_Z_PID.Angle.target = CHASSIS_GetMiddleAngle();
			
			if(Flag.Chassis.GoHome == false)
				Chas_Target_Speed[ Z ] = CHASSIS_Z_Angle_PidCalc(&Chassis_Z_PID.Angle, (YAW_DIR) * kKey_Gyro_Chas_Spin);
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
void KEY_SetChasMoveSpeed(int16_t sMoveMax, int16_t sMoveRamp)
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
	
	if(Chassis.Mode == CHAS_MODE_NORMAL) {
		
		/* 前后方向突变，刚开始一小段缓慢斜坡，防止轮子打滑浪费功率 */
		if(abs(Chas_Target_Speed[ Y ]) < (Chas_Standard_Move_Max/2.5f)) {
			Chas_Slope_Move_Fron = Chas_Standard_Move_Max * KEY_RampChasSpeed(IF_KEY_PRESSED_W, &time_fron_y, Time_Inc_Saltation, TIME_DEC_NORMAL);
			Chas_Slope_Move_Back = Chas_Standard_Move_Max * KEY_RampChasSpeed(IF_KEY_PRESSED_S, &time_back_y, Time_Inc_Saltation, TIME_DEC_NORMAL);
		} else {
			Chas_Slope_Move_Fron = Chas_Standard_Move_Max * KEY_RampChasSpeed(IF_KEY_PRESSED_W, &time_fron_y, Time_Inc_Normal, TIME_DEC_NORMAL);
			Chas_Slope_Move_Back = Chas_Standard_Move_Max * KEY_RampChasSpeed(IF_KEY_PRESSED_S, &time_back_y, Time_Inc_Normal, TIME_DEC_NORMAL);
		}
		
		/* 左右方向突变，刚开始一小段缓慢斜坡，防止轮子打滑浪费功率 */
		if(abs(Chas_Target_Speed[ X ]) < (Chas_Standard_Move_Max/2.5f)) {
			Chas_Slope_Move_Fron = Chas_Standard_Move_Max * KEY_RampChasSpeed(IF_KEY_PRESSED_W, &time_fron_y, Time_Inc_Saltation, TIME_DEC_NORMAL);
			Chas_Slope_Move_Back = Chas_Standard_Move_Max * KEY_RampChasSpeed(IF_KEY_PRESSED_S, &time_back_y, Time_Inc_Saltation, TIME_DEC_NORMAL);
		} else {
			Chas_Slope_Move_Left = Chas_Standard_Move_Max * KEY_RampChasSpeed(IF_KEY_PRESSED_A, &time_left_x, Time_Inc_Normal, TIME_DEC_NORMAL);
			Chas_Slope_Move_Righ = Chas_Standard_Move_Max * KEY_RampChasSpeed(IF_KEY_PRESSED_D, &time_righ_x, Time_Inc_Normal, TIME_DEC_NORMAL);
		}
		
	} else {
		/* 全向斜坡量计算 */
		Chas_Slope_Move_Fron = Chas_Standard_Move_Max * KEY_RampChasSpeed(IF_KEY_PRESSED_W, &time_fron_y, Time_Inc_Normal, TIME_DEC_NORMAL);
		Chas_Slope_Move_Back = Chas_Standard_Move_Max * KEY_RampChasSpeed(IF_KEY_PRESSED_S, &time_back_y, Time_Inc_Normal, TIME_DEC_NORMAL);
		Chas_Slope_Move_Left = Chas_Standard_Move_Max * KEY_RampChasSpeed(IF_KEY_PRESSED_A, &time_left_x, Time_Inc_Normal, TIME_DEC_NORMAL);
		Chas_Slope_Move_Righ = Chas_Standard_Move_Max * KEY_RampChasSpeed(IF_KEY_PRESSED_D, &time_righ_x, Time_Inc_Normal, TIME_DEC_NORMAL);
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
	if(CHASSIS_IfGyroMode() == true) {
		/* 计算偏差机械中值的角度差 */
		delta_angle = GIMBAL_GetTopGyroAngleOffset() * MECH_2_ANGLE_RADIAN;	//  (8192.f-GIMBAL_GetTopGyroAngleOffset()) * MECH_2_ANGLE_RADIAN
		target_speed_x = Chas_Target_Speed[ X ];
		target_speed_y = Chas_Target_Speed[ Y ];
		
		if(Flag.Chassis.GoHome == true)
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
 *					\	8 	/	----	旋转中心	(旋转时各麦轮的期望速度不同，半径小的期望速度要小、半径大的期望速度要大)
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
void KEY_SetChassisSpeed(int16_t sSpinMax, int16_t sMoveMax, int16_t sMoveRamp)
{
	/* 旋转 */
	KEY_SetChasSpinSpeed(sSpinMax);
	
	/* 平移 */
	KEY_SetChasMoveSpeed(sMoveMax, sMoveRamp);
	
	/* 全向分配算法 */
	CHASSIS_OmniSpeedCalc();		
}

/**
 *	@brief	根据按键值设置底盘模式
 */
void KEY_SetChassisMode(void)
{
	/* Ctrl+W */
	if(IF_KEY_PRESSED_W && IF_KEY_PRESSED_CTRL) {
		CHASSIS_SetMode(CHAS_MODE_SZUPUP);
	}
}

/**
 *	@brief	根据按键值设置底盘小陀螺模式
 */
void KEY_SetTopGyro(void)
{
	if(IF_KEY_PRESSED_F) {
		if(KeyLockFlag_F == false) {
			if(CHASSIS_IfGyroMode() == true) {
				if(Chassis.TopGyro == false) {
					Chassis_Z_PID.Angle.target = Chassis_Z_PID.Angle.feedback;	// 更新刚开启小陀螺时的目标角度
					Chassis.TopGyro = true;	// 开陀螺
					Chassis.Twist = false;	// 关扭腰
					top_gyro_dir = -1;	// 后面可设置随机数来使得小陀螺方向随机
					//根据单项赛要求设置周期为3~7s
					Chas_Top_Gyro_Period = RNG_ucGetRandomNum(3, 7)*1000;					
				}
				else {
					Chassis.TopGyro = false;
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
void KEY_SetTwist(void)
{
	if(IF_KEY_PRESSED_C) {
		if(KeyLockFlag_C == false) {
			if(CHASSIS_IfGyroMode() == true) {
				if(Chassis.Twist == false) {
					Chassis_Z_PID.Angle.target = Chassis_Z_PID.Angle.feedback;	// 更新刚开启扭腰时的目标角度
					Chassis.Twist = true;		// 开扭腰
					Chassis.TopGyro = false;	// 关陀螺
					twist_dir = -1;	// 后面可设置随机数来使得扭腰方向随机
				}
				else {
					Chassis.Twist = false;
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
void CHASSIS_Init(void)
{
	CHASSIS_KalmanCreate();	// 创建卡尔曼滤波器
	CHASSIS_InitParams();	// 初始化底盘参数
}

/**
 *	@brief	底盘功率限制(期望再分配)
 *	@note	祖传算法。主要是比例的算法,ICRA
 *			# 需考虑飞坡增益所带来的缓冲能量加成！
 */
void CHASSIS_PowerLimit(Chassis_Info_t *chas, Chassis_PID_t *pid, Judge_Info_t *judge)
{
	float kLimit;
	float fTotalOutput;
	float fRemainJ;
	static uint16_t usJudgeErrCnt = 0;
	
	// 读取缓冲焦耳能量
	fRemainJ = JUDGE_fGetChassisPowerBuffer();//judge->PowerHeatData.chassis_power_buffer;	
	fTotalOutput = abs(pid[LEFT_FRON_201].Out) + 
				   abs(pid[RIGH_FRON_202].Out) + 
				   abs(pid[LEFT_BACK_203].Out) + 
				   abs(pid[RIGH_BACK_204].Out);
	
	if(JUDGE_IfDataValid() == false) {
		usJudgeErrCnt++;
		if(usJudgeErrCnt > 100) {
			usJudgeErrCnt = 0;	// 防止溢出
			chas->Power.cur_limit = 9000;	// 最大的1/4
		}
	} else {
		usJudgeErrCnt = 0;
		// 剩余焦耳量过小,开始限制输出,限制系数为平方关系
		if(fRemainJ < WARNING_REMAIN_POWER) {
			kLimit = (float)(fRemainJ / WARNING_REMAIN_POWER)
						* (float)(fRemainJ / WARNING_REMAIN_POWER);
			chas->Power.cur_limit =  kLimit * chas->Power.max_limit;
		} else {	// 焦耳能量恢复到一定数值
			chas->Power.cur_limit = chas->Power.max_limit;
		}
	}
	
	if(fTotalOutput > chas->Power.cur_limit) {
		pid[LEFT_FRON_201].Out = (int16_t)(pid[LEFT_FRON_201].Out / fTotalOutput * chas->Power.cur_limit);
		pid[RIGH_FRON_202].Out = (int16_t)(pid[RIGH_FRON_202].Out / fTotalOutput * chas->Power.cur_limit);
		pid[LEFT_BACK_203].Out = (int16_t)(pid[LEFT_BACK_203].Out / fTotalOutput * chas->Power.cur_limit);
		pid[RIGH_BACK_204].Out = (int16_t)(pid[RIGH_BACK_204].Out / fTotalOutput * chas->Power.cur_limit);	
	}
}



/**
 *	@brief	底盘扭腰粗糙版
 */
float CHASSIS_TwistTargetCalc(int16_t maxTarget, int16_t rampTarget)
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
		target_z_speed = RampFloat(maxTarget, target_z_speed, rampTarget);
	} else if(dir == 0) {
		target_z_speed = RampFloat((-maxTarget), target_z_speed, rampTarget);
	}
	return target_z_speed;
}

/**
 *	@brief	底盘全向运动算法
 */
void CHASSIS_OmniSpeedCalc(void)
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
 *	@brief	机械模式下云台YAW期望值边界处理
 */
float CHASSIS_MECH_YawBoundaryProc(Chassis_Z_PID_t *pid, float delta_target)
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
void CHASSIS_NormalCtrl(void)
{
	KEY_SetChassisMode();
	KEY_SetTopGyro();
	KEY_SetTwist();
	KEY_SetChassisSpeed(SPIN_MAX_NORMAL, STANDARD_MAX_NORMAL, TIME_INC_NORMAL);
}

/**
 *	@brief	底盘打符时候的控制
 *	@note	打符的时候底盘不动
 */
void CHASSIS_BuffCtrl(void)
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
void CHASSIS_SzuPupCtrl(void)
{
	/* 松开W或者Ctrl */
	if(!IF_KEY_PRESSED_W || !IF_KEY_PRESSED_CTRL) {
		CHASSIS_SetMode(CHAS_MODE_NORMAL);
	}
	
	KEY_SetChassisSpeed(SPIN_MAX_SZUPUP, STANDARD_MAX_SZUPUP, TIME_INC_SZUPUP);
}

/**
 *	@brief	自动对位控制
 *	@note	x方向距离受大小补给站影响
 */
#define R_B_CASE	5

#define RELOAD_BULLET_DIS_Y				200	// 单位mm
#define RELOAD_BULLET_DIS_JUDGE_CNT		3	// 判断计数
#define RELOAD_BULLET_DIS_JUDGE_SMALL	450	// 小补给站x方向判据
#define RELOAD_BULLET_DIS_JUDGE_BIG		750	// 大补给站x方向判据
#define RELOAD_BULLET_DIS_JUDGE_DEATH	100	// 10cm允许误差范围
#define RELOAD_BULLET_DIS_X_SMALL		300	// 判定为小补给站时，x方向移动距离
#define RELOAD_BULLET_DIS_X_BIG			500	// 判定为大补给站时，x方向移动距离

float tar_angle_sum_y = -((RELOAD_BULLET_DIS_Y / PERIMETER) * 8192) / CHASSIS_DECELE_RATIO;	// 倒退取负
float ramp_angle_y = 1024;

portTickType ulDetectTime[TIME_STATE_COUNT];	// 测试超声波反馈帧率

void CHASSIS_ReloadBulletCtrl(void)
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

		if(Flag.Chassis.ReloadBulletStart == true) {
			Flag.Chassis.ReloadBulletStart = false;
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
		CHASSIS_OmniSpeedCalc();	
		
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
		float fed_angle_sum_y;
		
		if(Flag.Chassis.ReloadBulletStart == true) {
			Flag.Chassis.ReloadBulletStart = false;
			/* Y */
			// 电机累加机械角度清零
			CHASSIS_Angle_ClearSum(Chassis_PID);
			// y方向目标累加角度清零
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
		
		Chas_Locate_PID[ Y ].target = RampFloat(tar_angle_sum_y, Chas_Locate_PID[ Y ].target, ramp_angle_y);
		Chas_Locate_PID[ Y ].feedback = fed_angle_sum_y;
		
		Chas_Target_Speed[ Y ] = CHASSIS_Locate_PidCalc(&Chas_Locate_PID[ Y ]);
		
		/* 全向分配算法 */
		CHASSIS_OmniSpeedCalc();	
		
	#elif (R_B_CASE == 3)
		// 单反馈半自动对位模型
		/*
				
		*/
	#elif (R_B_CASE == 4)
		// 双反馈PID控制模型
		
		static float dis_judge_x = 0;	// 触发位置距离信息判断大小补给站
		float tar_angle_sum_y = -((RELOAD_BULLET_DIS_Y / PERIMETER) * 8192) / CHASSIS_DECELE_RATIO;	// 倒退取负
		
		if(Flag.Chassis.ReloadBulletStart == true) {
			// 清除触发标志位
			Flag.Chassis.ReloadBulletStart = false;
			// 重新判定大小补给站
			Flag.Chassis.ReloadBulletJudge = false;
			// 重装载判断计数
			Cnt.Chassis.ReloadBulletJudge = RELOAD_BULLET_DIS_JUDGE_CNT;
			// 初始距离清零
			dis_judge_x = 0.f;
			// 电机累加机械角度清零
			CHASSIS_Angle_ClearSum(Chassis_PID);
			// y方向目标累加角度清零
			Chas_Locate_PID[ Y ].target = 0;
			// x方向期望速度清零
			Chas_Target_Speed[ X ] = 0;
			// y方向期望速度清零
			Chas_Target_Speed[ Y ] = 0;
			// 开启一次超声波探测
			Ultra.update = false;
			ULTRA_Detect();
		}
		
		/* 正在判断大小补给站 */
		if(Cnt.Chassis.ReloadBulletJudge) {
			
			// 等待超声波数据更新
			if(Ultra.update == true) {
				
				Ultra.update = false;	// 清除更新标志

				ULTRA_Detect();	// 再开启一次超声波探测

				ulDetectTime[NOW] = xTaskGetTickCount();
				
				ulDetectTime[DELTA] = ulDetectTime[NOW] - ulDetectTime[PREV];

				ulDetectTime[PREV] = ulDetectTime[NOW];
				
				Cnt.Chassis.ReloadBulletJudge--;
				
				dis_judge_x += Ultra.dis;
				
				/* 判断结束 */
				if(Cnt.Chassis.ReloadBulletJudge == 0) {
				
					dis_judge_x /= RELOAD_BULLET_DIS_JUDGE_CNT;	// 多次测量取平均
					
					if(myDeathZoom(RELOAD_BULLET_DIS_JUDGE_SMALL, RELOAD_BULLET_DIS_JUDGE_DEATH, dis_judge_x) == true) {
					
						Chas_Locate_PID[ X ].target = RELOAD_BULLET_DIS_X_SMALL;	// 判定为小补给站
						
						Chas_Locate_PID[ Y ].target = tar_angle_sum_y;
						
						Flag.Chassis.ReloadBulletJudge = true;
						
					}
					else if(myDeathZoom(RELOAD_BULLET_DIS_JUDGE_BIG, RELOAD_BULLET_DIS_JUDGE_DEATH, dis_judge_x) == true) {
					
						Chas_Locate_PID[ X ].target = RELOAD_BULLET_DIS_X_BIG;		// 判定为大补给站
						
						Chas_Locate_PID[ Y ].target = tar_angle_sum_y;
						
						Flag.Chassis.ReloadBulletJudge = true;
						
					}
					else {
					
						// ..错误数据
						Flag.Chassis.ReloadBulletJudge = false;
					}
					
				}
			}
		}
		
		/* 判定成功 */
		if(Flag.Chassis.ReloadBulletJudge == true) {
		
			if(Ultra.update == true) {
			
				Ultra.update = false;
				
				ULTRA_Detect();	// 再开启一次超声波探测
				
				Chas_Target_Speed[ X ] = CHASSIS_Locate_PidCalc(&Chas_Locate_PID[ X ]);
			
			}
			
			Chas_Target_Speed[ Y ] = CHASSIS_Locate_PidCalc(&Chas_Locate_PID[ Y ]);
			
		} else {
		
			Chas_Target_Speed[ X ] = 0;
			
			Chas_Target_Speed[ Y ] = CHASSIS_Locate_PidCalc(&Chas_Locate_PID[ Y ]);

//			if(Ultra.update == true) {
//			
//				Ultra.update = false;
//				
//				ULTRA_Detect();	// 再开启一次超声波探测
//				
//				ulDetectTime[NOW] = xTaskGetTickCount();
//				
//				ulDetectTime[DELTA] = ulDetectTime[NOW] - ulDetectTime[PREV];

//				ulDetectTime[PREV] = ulDetectTime[NOW];
//			
//			}
			
		}
		
		/* 全向分配算法 */
		CHASSIS_OmniSpeedCalc();
		
	#elif (R_B_CASE == 5)
		// 双反馈PID控制模型
		
		Supply_Id_t supply_id;
		
		if(Flag.Chassis.ReloadBulletStart == true) {
			// 清除触发标志位
			Flag.Chassis.ReloadBulletStart = false;
			// 重新判定大小补给站
			Flag.Chassis.ReloadBulletJudge = false;
			// 重装载判断计数
			Cnt.Chassis.ReloadBulletJudge = RELOAD_BULLET_DIS_JUDGE_CNT;
			// 电机累加机械角度清零
			CHASSIS_Angle_ClearSum(Chassis_PID);
			// y方向目标累加角度重置
			Chas_Locate_PID[ Y ].target = tar_angle_sum_y;
			// x方向期望速度清零
			Chas_Target_Speed[ X ] = 0;
			// y方向期望速度清零
			Chas_Target_Speed[ Y ] = 0;
			// 开启一次超声波探测
			Ultra.update = true;
		}
		
		/* 正在判断大小补给站 */
		if(Cnt.Chassis.ReloadBulletJudge) {
			
			supply_id = JUDGE_eGetSupplyId();
			
			if(supply_id == SUPPLY_ID_1) {
				
				Chas_Locate_PID[ X ].target = RELOAD_BULLET_DIS_X_SMALL;	// 判定为小补给站
				
				Flag.Chassis.ReloadBulletJudge = true;

				Cnt.Chassis.ReloadBulletJudge = 0;
				
			} else if(supply_id == SUPPLY_ID_2) {
				
				Chas_Locate_PID[ X ].target = RELOAD_BULLET_DIS_X_BIG;		// 判定为大补给站
				
				Flag.Chassis.ReloadBulletJudge = true;
						
				Cnt.Chassis.ReloadBulletJudge = 0;

			}
			
		}
		
		/* 判定成功 */
		if(Flag.Chassis.ReloadBulletJudge == true) {
		
			#if (ULTRA_USE_USART)
				if(Ultra.update == true) {
				
					Ultra.update = false;
					
					ULTRA_Detect();	// 再开启一次超声波探测
					
					ulDetectTime[NOW] = xTaskGetTickCount();
					
					ulDetectTime[DELTA] = ulDetectTime[NOW] - ulDetectTime[PREV];

					ulDetectTime[PREV] = ulDetectTime[NOW];

					Chas_Locate_PID[ X ].feedback = Ultra.dis;
					
					Chas_Target_Speed[ X ] = CHASSIS_Locate_PidCalc(&Chas_Locate_PID[ X ]);
				
				}
			#else
				if(Ultra.update == true) {
					
					Ultra.update = false;
					
					ULTRA_Detect();	// 再开启一次超声波探测

					ulDetectTime[PREV] = xTaskGetTickCount();
					
				} 
				else {
				
					// 读取SCL线来判断是否探测结束
					if( ULTRA_SCL ) {
						
						Ultra.update = true;

						ulDetectTime[NOW] = xTaskGetTickCount();
						
						ulDetectTime[DELTA] = ulDetectTime[NOW] - ulDetectTime[PREV];

						ULTRA_IICReadResult();
						
						Chas_Locate_PID[ X ].feedback = Ultra.dis;
						
						Chas_Target_Speed[ X ] = CHASSIS_Locate_PidCalc(&Chas_Locate_PID[ X ]);
					}					
					
				}
				
			#endif
			
			Chas_Target_Speed[ Y ] = CHASSIS_Locate_PidCalc(&Chas_Locate_PID[ Y ]);
			
		} 
		/* 判定失败 */
		else {
		
			Chas_Target_Speed[ X ] = 0;
			
			Chas_Target_Speed[ Y ] = CHASSIS_Locate_PidCalc(&Chas_Locate_PID[ Y ]);
			
		}
		
		/* 全向分配算法 */
		CHASSIS_OmniSpeedCalc();		
	
	#endif
}

/* #任务层# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	底盘获取pid模式信息
 */
void CHASSIS_GetInfo(void)
{
	// 获取系统信息
	CHASSIS_GetSysInfo(&System, &Chassis);
	// 获取裁判系统信息
	CHASSIS_GetJudgeInfo(&Judge, &Chassis);
	// 获取IMU信息
	CHASSIS_GetImuInfo(&Chassis_Z_PID);	
	// 获取小陀螺信息
	CHASSIS_GetTopGyroInfo();
	// 获取遥控信息
	CHASSIS_GetRemoteInfo(&System, &Remote, &Chassis);
}

/**
 *	@brief	pid控制器最终输出
 */
void CHASSIS_PidCtrlTask(void)
{
	/* PID速度环计算 */
	CHASSIS_Speed_PidCalc(Chassis_PID, LEFT_FRON_201); 	// 左前 - 速度环
	CHASSIS_Speed_PidCalc(Chassis_PID, RIGH_FRON_202); 	// 右前 - 速度环
	CHASSIS_Speed_PidCalc(Chassis_PID, LEFT_BACK_203); 	// 左后 - 速度环
	CHASSIS_Speed_PidCalc(Chassis_PID, RIGH_BACK_204); 	// 右后 - 速度环
	/* 功率限制 */
	//CHASSIS_PowerLimit(&Chassis_Power, Chassis_PID, &Judge);
	/* 最终输出 */
	CHASSIS_PidOut(Chassis_PID);	
}

/**
 *	@brief	遥控控制底盘任务
 */
void CHASSIS_RcCtrlTask(void)
{
	//REMOTE_SetChassisSpeed();
	REMOTE_TOP_SetChassisSpeed();
	REMOTE_SetTopGyro();
}

/**
 *	@brief	键盘控制底盘任务
 */
void CHASSIS_KeyCtrlTask(void)
{	
	switch(Chassis.Mode)
	{
		case CHAS_MODE_NORMAL:
			CHASSIS_NormalCtrl();
			break;
		case CHAS_MODE_BUFF:
			CHASSIS_BuffCtrl();
			break;
		case CHAS_MODE_RELOAD_BULLET:
			CHASSIS_ReloadBulletCtrl();
			break;
		case CHAS_MODE_SZUPUP:
			CHASSIS_SzuPupCtrl();
			break;
		default:
			break;
	}
}

/**
 *	@brief	底盘失控保护
 */
void CHASSIS_SelfProtect(void)
{
//	CHASSIS_Stop(Chassis_PID);
//	CHASSIS_PID_ParamsInit(Chassis_PID, CHASSIS_MOTOR_COUNT);
//	CHASSIS_Z_PID_ParamsInit(&Chassis_Z_PID.Angle);	
//	CHASSIS_GetRemoteInfo(&System, &Remote, &Chassis);
	CHASSIS_ReloadBulletCtrl();
}

/**
 *	@brief	底盘控制任务
 */
void CHASSIS_Ctrl(void)
{
	/*----信息读入----*/
	CHASSIS_GetInfo();
	/*----期望修改----*/
	if(Chassis.RemoteMode == RC) {
		CHASSIS_RcCtrlTask();
	}
	else if(Chassis.RemoteMode == KEY) {
		CHASSIS_KeyCtrlTask();
	}
	/*----最终输出----*/
	CHASSIS_PidCtrlTask();
}
