/**
 * @file        Task_Gimbal.c
 * @author      MaWeiming
 * @Version     V1.0
 * @date        1-October-2019
 * @brief       This file includes the Gimbal(云台) external functions 
 *
 * @Verison			V1.1 (1-October-2019)
 *							V1.2 (30-Jan-2020)		-	优化云台复位就近算法
 */

/**
 *	 串级PID	参考网址：https://www.jianshu.com/p/4b0fa85cd353
 */

/* Includes ------------------------------------------------------------------*/
#include "Task_Gimbal.h"

#include "laser.h"
#include "can.h"
#include "kalman.h"
#include "kalman_filter.h"
#include "delay.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "remote.h"

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#define GIMBAL_NORMAL_SPEED_PID_IOUT_MAX 18000

int GIMBAL_ANGLE_PID_IOUT_MAX	= 18000;
int GIMBAL_ANGLE_PID_OUT_MAX	= 8000;
int GIMBAL_SPEED_PID_IOUT_MAX	= GIMBAL_NORMAL_SPEED_PID_IOUT_MAX;
int GIMBAL_SPEED_PID_OUT_MAX	= 29999;

/* 二阶卡尔曼 */
#define KF_ANGLE	0
#define KF_SPEED	1
#define KF_ACCEL	2

/* 视觉掉帧 */
#define LOST	0
#define FOUND	1

#define K_LEVEL_0	2.5
#define K_LEVEL_1	2
#define K_LEVEL_2	1.67
#define K_LEVEL_3	1

/* Private variables ---------------------------------------------------------*/
/* 卡尔曼滤波器 */
extKalman_t Gimbal_kalmanError[GIMBAL_MODE_COUNT][GIMBAL_MOTOR_COUNT];
extKalman_t Mpu_kalmanError[2];
extKalman_t Gimbal_Auto_kalmanError[GIMBAL_MOTOR_COUNT];

extKalman_t kalman_visionYaw,kalman_cloudYaw,kalman_targetYaw,kalman_speedYaw, kalman_dist, kalman_accel;

kalman_filter_t yaw_kalman_filter;
kalman_filter_t pitch_kalman_filter;
kalman_filter_init_t yaw_kalman_filter_para = {
  .P_data = {2, 0, 0, 2},
  .A_data = {1, 0.002/*0.001*/, 0, 1},//采样时间间隔
  .H_data = {1, 0, 0, 1},
  .Q_data = {1, 0, 0, 1},
  .R_data = {2000, 0, 0, 1500}//500 1000
};//初始化yaw的部分kalman参数

kalman_filter_init_t pitch_kalman_filter_para = {
  .P_data = {2, 0, 0, 2},
  .A_data = {1, 0.002/*0.001*/, 0, 1},//采样时间间隔
  .H_data = {1, 0, 0, 1},
  .Q_data = {1, 0, 0, 1},
  .R_data = {200, 0, 0, 400}
};//初始化pitch的部分kalman参数

/* 打符参数 */
float GIMBAL_BUFF_PITCH_COMPENSATION =	5;	// 0	5
float GIMBAL_BUFF_YAW_COMPENSATION = -15;	// -175		-35
float GIMBAL_BUFF_YAW_RAMP = 170;		// 185	160	185	180
float GIMBAL_BUFF_PITCH_RAMP = 120;	// 130	105	120	140

/* 打符的时候pitch补偿表 */
float BUFF_PITCH_MODIFY_TABLE[11] = 
{
	// 从最低位置开始以4.4°(机械角 底-顶 = 4472-3045 = 1067 分成11份，100为机械角度步进)为角度步进
	/* 打符的时候发现 [3]90 ~ [7]80为有效范围 */
	/* 头抬得越高 []下标值越大 => MAX_UP -> [11] MAX_DOWN -> [0] */
	/* 补偿值越大 -> 头压得越低(补偿为0的时候头会偏得比较高) */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

/* 预测到位算法 */
bool Mobi_Pre_Yaw = false;//预测是否开启标志位
bool Mobi_Pre_Yaw_Fire = false;//默认预测没到位，禁止开枪

/* 自瞄算法 */
float start_gyro_yaw=0;
float now_gyro_yaw=0;
float last_gyro_yaw=0;
float delta_gyro_yaw=0;

float k_level;	// 射速影响因子（最高射速时为1）

/* ## Global variables ## ----------------------------------------------------*/
 
/**
 *	@pid 参数分布表
 *	1. 正常模式	- 机械+陀螺仪
 *	2. 自瞄模式 - 陀螺仪
 *	3. 打符模式 - 机械+陀螺仪
 *	4. 吊射基地模式	- 机械
 */

// 34.40 46.25 10.45
/**
 *	@brief	云台PID
 */
Gimbal_PID_t	Gimbal_PID[GIMBAL_MODE_COUNT][GIMBAL_MOTOR_COUNT] = {
	{	// MECH - 机械模式
		{	// YAW(机械模式)
			/* 速度环 */
			.Speed.kp = 34.32,		
			.Speed.ki = 38.10,		
			.Speed.kd = 0,	
			.Speed.target = 0,
			.Speed.feedback = 0,
			.Speed.erro = 0,
			.Speed.last_erro = 0,
			.Speed.integrate = 0,
			.Speed.integrate_max = 27000,
			.Speed.pout = 0,
			.Speed.iout = 0,
			.Speed.dout = 0,
			.Speed.out = 0,
			/* 位置环 */
			.Angle.kp = 10.51,	
			.Angle.ki = 0,			
			.Angle.kd = 0,
			.Angle.target = GIMBAL_MECH_YAW_MID_ANGLE,	// 归中
			.Angle.feedback = 0,
			.Angle.erro = 0,
			.Angle.last_erro = 0,
			.Angle.integrate = 0,
			.Angle.integrate_max = 0,
			.Angle.pout = 0,
			.Angle.iout = 0,
			.Angle.dout = 0,
			.Angle.out = 0,	
			/* 斜坡数据 */
			.AngleRampTarget = 0,
			.AngleRampFeedback = 0,
			/* 输出 */
			.Out = 0,
		},
		{	// PITCH(机械模式)
			/* 速度环 */
			.Speed.kp = 15.10,		
			.Speed.ki = 83.5,		
			.Speed.kd = 0,	
			.Speed.target = 0,
			.Speed.feedback = 0,
			.Speed.erro = 0,
			.Speed.last_erro = 0,
			.Speed.integrate = 0,
			.Speed.integrate_max = 54000,
			.Speed.pout = 0,
			.Speed.iout = 0,
			.Speed.dout = 0,
			.Speed.out = 0,
			/* 位置环 */
			.Angle.kp = 8.21,		
			.Angle.ki = 0,
			.Angle.kd = 0,
			.Angle.target = GIMBAL_MECH_PITCH_MID_ANGLE,
			.Angle.feedback = 0,
			.Angle.erro = 0,
			.Angle.last_erro = 0,
			.Angle.integrate = 0,
			.Angle.integrate_max = 0,
			.Angle.pout = 0,
			.Angle.iout = 0,
			.Angle.dout = 0,
			.Angle.out = 0,		
			/* 斜坡数据 */
			.AngleRampTarget = 0,
			.AngleRampFeedback = 0,
			/* 输出 */
			.Out = 0,		
		},
	},	// MECH - 机械模式
	{	// GYRO - 陀螺仪模式
		{	// YAW(陀螺仪模式)
			/* 速度环 */
			.Speed.kp = 34.5,	
			.Speed.ki = 43.5,			
			.Speed.kd = 0,	
			.Speed.target = 0,
			.Speed.feedback = 0,
			.Speed.erro = 0,
			.Speed.last_erro = 0,
			.Speed.integrate = 0,
			.Speed.integrate_max = 18000,	// 18000
			.Speed.pout = 0,
			.Speed.iout = 0,
			.Speed.dout = 0,
			.Speed.out = 0,
			/* 位置环 */
			.Angle.kp = 10.72,		
			.Angle.ki = 0,
			.Angle.kd = 0,
			.Angle.target = 0,
			.Angle.feedback = 0,
			.Angle.erro = 0,
			.Angle.last_erro = 0,
			.Angle.integrate = 0,
			.Angle.integrate_max = 0,
			.Angle.pout = 0,
			.Angle.iout = 0,
			.Angle.dout = 0,
			.Angle.out = 0,		
			/* 斜坡数据 */
			.AngleRampTarget = 0,
			.AngleRampFeedback = 0,
			/* 输出 */
			.Out = 0,		
		},
		{	// PITCH(陀螺仪模式) - 采用机械模式那一套
			/* 速度环 */
			.Speed.kp = 15.10,		
			.Speed.ki = 83.5,		
			.Speed.kd = 0,	
			.Speed.target = 0,
			.Speed.feedback = 0,
			.Speed.erro = 0,
			.Speed.last_erro = 0,
			.Speed.integrate = 0,
			.Speed.integrate_max = 54000,
			.Speed.pout = 0,
			.Speed.iout = 0,
			.Speed.dout = 0,
			.Speed.out = 0,
			/* 位置环 */
			.Angle.kp = 8.21,		
			.Angle.ki = 0,
			.Angle.kd = 0,
			.Angle.target = GIMBAL_MECH_PITCH_MID_ANGLE,
			.Angle.feedback = 0,
			.Angle.erro = 0,
			.Angle.last_erro = 0,
			.Angle.integrate = 0,
			.Angle.integrate_max = 0,
			.Angle.pout = 0,
			.Angle.iout = 0,
			.Angle.dout = 0,
			.Angle.out = 0,		
			/* 斜坡数据 */
			.AngleRampTarget = 0,
			.AngleRampFeedback = 0,
			/* 输出 */
			.Out = 0,
		},
	}	// GYRO - 陀螺仪模式
};

/**
 *	@brief	云台综合信息
 */
Gimbal_Info_t 	Gimbal = {
	.State.mode = GIMBAL_MODE_NORMAL,
	
	.Auto.Yaw.kp = 1,		
	.Auto.Yaw.target = 0,
	.Auto.Yaw.erro = 0,
	
	.Auto.Pitch.kp = 1,		
	.Auto.Pitch.target = 0,
	.Auto.Pitch.erro = 0,
	
	.Buff.Yaw.kp = 1,		
	.Buff.Yaw.target = 0,
	.Buff.Yaw.erro = 0,
	
	.Buff.Pitch.kp = 1,		
	.Buff.Pitch.target = 0,
	.Buff.Pitch.erro = 0,
};

/* 视觉掉帧处理队列 */
QueueObj LostQueue = 
{
	.nowLength = 0,
	.queueLength = 8,
	.queue = {0}
};

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* API functions -------------------------------------------------------------*/
/**
 *	@brief	云台电机PID参数初始化
 */
void GIMBAL_PidParamsInit(Gimbal_PID_t *pid, uint8_t motor_cnt)
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

		if(i == YAW_205) {
			pid[i].Angle.target = GIMBAL_MECH_YAW_MID_ANGLE;
		} else if(i == PITCH_206) {
			pid[i].Angle.target = GIMBAL_MECH_PITCH_MID_ANGLE;
		}
		//pid[i].Angle.feedback = 0,
		pid[i].Angle.erro = 0;
		pid[i].Angle.last_erro = 0;
		pid[i].Angle.integrate = 0;
		pid[i].Angle.pout = 0;
		pid[i].Angle.dout = 0;
		pid[i].Angle.iout = 0;
		pid[i].Angle.out = 0;
		pid[i].AngleRampTarget = 0;
		pid[i].AngleRampFeedback = 0;
		pid[i].Out = 0;
	}	
}

/**
 *	@brief	根据云台当前模式进行PID参数的切换
 */
float auto_yaw_angle_kp = 9.25f;	// 7(ramp280)			7.5		7.75	8.5		9.25
float auto_yaw_speed_kp = 31.5f;	// 31.5			34.5	31.5	34.5	31.5	32.5
float auto_yaw_speed_ki = 575.f;	// 550			650		550		650		575		575

float auto_pitch_angle_kp = 8.5f;		// 10.5		10.5(ramp80)	10.5		8.5		8
float auto_pitch_speed_kp = 15.25f;	// 16.65	// 11.15	18.5			12.85		15.85	15.65
float auto_pitch_speed_ki = 275.f;	// 250		250				175			200		275

float buff_yaw_angle_kp = 6.15f;	// 3.65(ramp115)	4.5			3.75		3.85	9			6.15（ramp 3000)	6.15(ramp 170)
float buff_yaw_speed_kp = 28.5f;	// 31.5						30.5		30.5		33.5	30.5	29.5							28.5
float buff_yaw_speed_ki = 560.f;	// 500						300			350.		500		550		575								560

float buff_pitch_angle_kp = 7.f;		// 3			6			5.85	(ramp 125)	6.55(ramp140)	5.85	(ramp 140)	6			7
float buff_pitch_speed_kp = 12.f;	// 13.5		13.65	12.65							13.45					13.45								14.5	12
float buff_pitch_speed_ki = 250.f;	// 250.f	400		500								250						200								215		250

float normal_mech_yaw_angle_kp = 12.05f;	// 		10.35	10.65			11.25		11.55		13.05	14.45
float normal_mech_yaw_speed_kp = 28.5f;	// 34.32	18.75	26.5			27.5		28.5	33.5	33.5
float normal_mech_yaw_speed_ki = 350.f;	// 38.10	32.5	90				225			200		550	

float normal_gyro_yaw_angle_kp = 12.25f;// 9.5		9.5	6.5		7.15	7.25	7.25		8			6.5		12.25(累加值)
float normal_gyro_yaw_speed_kp = 29.f;	// 26.75	20	13.5	14.25	14		14.25	14.35		20		29		
float normal_gyro_yaw_speed_ki = 510.f;	// 33.50	0		240		250		250		325			375		505		510

float normal_pitch_angle_kp = 12.f;	// 8.35			12.50	12		15			14.65
float normal_pitch_speed_kp = 12.65f;	// 15.10	16.65	14.25		12.85	14.75		14.65
float normal_pitch_speed_ki = 180.f;	// 39.5		55		100				150		180		180.

void GIMBAL_PidParamsSwitch(Gimbal_PID_t pid[GIMBAL_MODE_COUNT][GIMBAL_MOTOR_COUNT], Gimbal_Info_t *gimbal)
{
	switch(gimbal->State.mode)
	{
		case GIMBAL_MODE_NORMAL:
		
			pid[MECH][YAW_205].Speed.kp = normal_mech_yaw_speed_kp;		
			pid[MECH][YAW_205].Speed.ki = normal_mech_yaw_speed_ki;		
			pid[MECH][YAW_205].Speed.kd = 0;
			pid[MECH][YAW_205].Speed.out_max = 29999;
			pid[MECH][YAW_205].Angle.kp = normal_mech_yaw_angle_kp;		
			pid[MECH][YAW_205].Angle.ki = 0;		
			pid[MECH][YAW_205].Angle.kd = 0;
			pid[MECH][YAW_205].Angle.out_max = 8000;
		
			pid[MECH][PITCH_206].Speed.kp = normal_pitch_speed_kp;		
			pid[MECH][PITCH_206].Speed.ki = normal_pitch_speed_ki;		
			pid[MECH][PITCH_206].Speed.kd = 0;
			pid[MECH][PITCH_206].Speed.out_max = 29999;
			pid[MECH][PITCH_206].Angle.kp = normal_pitch_angle_kp;		
			pid[MECH][PITCH_206].Angle.ki = 0;		
			pid[MECH][PITCH_206].Angle.kd = 0;
			pid[MECH][PITCH_206].Angle.out_max = 6000;

			pid[GYRO][YAW_205].Speed.kp = normal_gyro_yaw_speed_kp;		
			pid[GYRO][YAW_205].Speed.ki = normal_gyro_yaw_speed_ki;		
			pid[GYRO][YAW_205].Speed.kd = 0;
			pid[GYRO][YAW_205].Speed.out_max = 29999;
			pid[GYRO][YAW_205].Angle.kp = normal_gyro_yaw_angle_kp;		
			pid[GYRO][YAW_205].Angle.ki = 0;		
			pid[GYRO][YAW_205].Angle.kd = 0;
			pid[GYRO][YAW_205].Angle.out_max = 8000;
			
			pid[GYRO][PITCH_206].Speed.kp = normal_pitch_speed_kp;		
			pid[GYRO][PITCH_206].Speed.ki = normal_pitch_speed_ki;		
			pid[GYRO][PITCH_206].Speed.kd = 0;
			pid[GYRO][PITCH_206].Speed.out_max = 29999;
			pid[GYRO][PITCH_206].Angle.kp = normal_pitch_angle_kp;		
			pid[GYRO][PITCH_206].Angle.ki = 0;		
			pid[GYRO][PITCH_206].Angle.kd = 0;
			pid[GYRO][PITCH_206].Angle.out_max = 6000;

			break;
		case GIMBAL_MODE_AUTO:
			if(VISION_GetFlagStatus(VISION_FLAG_LOCK_TARGET) == true) // 锁定目标
			{
				pid[GYRO][YAW_205].Speed.kp = auto_yaw_speed_kp;		
				pid[GYRO][YAW_205].Speed.ki = auto_yaw_speed_ki;		
				pid[GYRO][YAW_205].Speed.kd = 0;
				pid[GYRO][YAW_205].Angle.kp = auto_yaw_angle_kp;		
				pid[GYRO][YAW_205].Angle.ki = 0;		
				pid[GYRO][YAW_205].Angle.kd = 0;
				
				pid[GYRO][PITCH_206].Speed.kp = auto_pitch_speed_kp;		
				pid[GYRO][PITCH_206].Speed.ki = auto_pitch_speed_ki;		
				pid[GYRO][PITCH_206].Speed.kd = 0;
				pid[GYRO][PITCH_206].Angle.kp = auto_pitch_angle_kp;		
				pid[GYRO][PITCH_206].Angle.ki = 0;		
				pid[GYRO][PITCH_206].Angle.kd = 0;
							
			}
			else	// 未锁定目标则正常移动云台
			{
				pid[GYRO][YAW_205].Speed.kp = normal_gyro_yaw_speed_kp;		
				pid[GYRO][YAW_205].Speed.ki = normal_gyro_yaw_speed_ki;		
				pid[GYRO][YAW_205].Speed.kd = 0;
				pid[GYRO][YAW_205].Angle.kp = normal_gyro_yaw_angle_kp;		
				pid[GYRO][YAW_205].Angle.ki = 0;		
				pid[GYRO][YAW_205].Angle.kd = 0;

				pid[GYRO][PITCH_206].Speed.kp = normal_pitch_speed_kp;		
				pid[GYRO][PITCH_206].Speed.ki = normal_pitch_speed_ki;		
				pid[GYRO][PITCH_206].Speed.kd = 0;
				pid[GYRO][PITCH_206].Angle.kp = normal_pitch_angle_kp;		
				pid[GYRO][PITCH_206].Angle.ki = 0;		
				pid[GYRO][PITCH_206].Angle.kd = 0;			
				
			}
			
			break;
			
		case GIMBAL_MODE_BIG_BUFF:
		case GIMBAL_MODE_SMALL_BUFF:
				pid[GYRO][YAW_205].Speed.kp = buff_yaw_speed_kp;		
				pid[GYRO][YAW_205].Speed.ki = buff_yaw_speed_ki;		
				pid[GYRO][YAW_205].Speed.kd = 0;
				pid[GYRO][YAW_205].Angle.kp = buff_yaw_angle_kp;		
				pid[GYRO][YAW_205].Angle.ki = 0;		
				pid[GYRO][YAW_205].Angle.kd = 0;
				
				pid[GYRO][PITCH_206].Speed.kp = buff_pitch_speed_kp;		
				pid[GYRO][PITCH_206].Speed.ki = buff_pitch_speed_ki;		
				pid[GYRO][PITCH_206].Speed.kd = 0;
				pid[GYRO][PITCH_206].Angle.kp = buff_pitch_angle_kp;		
				pid[GYRO][PITCH_206].Angle.ki = 0;		
				pid[GYRO][PITCH_206].Angle.kd = 0;			
		
			break;
		
		default:
			break;
	}
}

/**
 *	@brief	云台电机卸力
 */
void GIMBAL_Stop(Gimbal_PID_t *pid)
{
	static int16_t pid_out[4] = {0, 0, 0, 0};
	
	/* 内环速度环最终输出 */
	pid[YAW_205].Speed.out = 0;
	pid[YAW_205].Out = 0;
	pid[PITCH_206].Speed.out = 0;
	pid[PITCH_206].Out = 0;
	
	CAN1_Send(0x1FF, pid_out);	// 云台CAN总线标准标识符
}

/**
 *	@brief	云台电机速度环
 */
float js_gimbal_yaw_speed_feedback;
float js_gimbal_yaw_speed_target;
float js_gimbal_yaw_speed_out;
float js_gimbal_pitch_speed_feedback;
float js_gimbal_pitch_speed_target;
float js_gimbal_pitch_speed_out;
float js_gimbal_speed_feedback;

void GIMBAL_Speed_PidCalc(Gimbal_PID_t *pid, Gimbal_Motor_Names_t MOTORx)
{
	js_gimbal_yaw_speed_feedback = pid[YAW_205].Speed.feedback;
	js_gimbal_yaw_speed_target = pid[YAW_205].Speed.target;
	js_gimbal_pitch_speed_feedback = pid[PITCH_206].Speed.feedback;
	js_gimbal_pitch_speed_target = pid[PITCH_206].Speed.target;
	
	pid[MOTORx].Speed.erro = pid[MOTORx].Speed.target - pid[MOTORx].Speed.feedback;
	pid[MOTORx].Speed.integrate += pid[MOTORx].Speed.erro;
	pid[MOTORx].Speed.integrate = constrain(pid[MOTORx].Speed.integrate, -pid[MOTORx].Speed.integrate_max, pid[MOTORx].Speed.integrate_max);
		
	/* Pout */
	pid[MOTORx].Speed.pout = pid[MOTORx].Speed.kp * pid[MOTORx].Speed.erro;
	
	/* Iout */
	pid[MOTORx].Speed.iout = pid[MOTORx].Speed.ki * pid[MOTORx].Speed.integrate * 0.002f; // 模仿19步兵代码(0.002f是2ms的意思)
	/* Iout Limits */
	pid[MOTORx].Speed.iout = constrain(pid[MOTORx].Speed.iout, -GIMBAL_SPEED_PID_IOUT_MAX, GIMBAL_SPEED_PID_IOUT_MAX);
	/* Dout*/
	pid[MOTORx].Speed.dout = pid[MOTORx].Speed.kd * (pid[MOTORx].Speed.erro - pid[MOTORx].Speed.last_erro)/0.002f;
	/* Record Last Error */
	pid[MOTORx].Speed.last_erro = pid[MOTORx].Speed.erro;		
	
	/* Total PID Output*/
	pid[MOTORx].Speed.out = pid[MOTORx].Speed.pout + pid[MOTORx].Speed.iout + pid[MOTORx].Speed.dout;
	/* Total PID Output Limits */
	pid[MOTORx].Speed.out = constrain(pid[MOTORx].Speed.out, -GIMBAL_SPEED_PID_OUT_MAX, GIMBAL_SPEED_PID_OUT_MAX);
	/* 内环速度环最终输出 */
	pid[MOTORx].Out = pid[MOTORx].Speed.out;
	
	js_gimbal_yaw_speed_out = pid[YAW_205].Speed.out;
	js_gimbal_pitch_speed_out = pid[PITCH_206].Speed.out;
}

/**
 *	@brief	云台电机位置环
 *	@note
 *			串环PID
 *			内环 期望值 期望角速度(实际上就是速度环)
 *					 输出值 期望角加速度
 *
 *			外环 期望值 期望角度
 *					 输出值 期望角速度
 */
float js_gimbal_yaw_angle_feedback;
float js_gimbal_yaw_angle_target;
float js_gimbal_pitch_angle_feedback;
float js_gimbal_pitch_angle_target;
float js_gimbal_yaw_angle_erro;
float js_gimbal_pitch_angle_erro;
void GIMBAL_Angle_PidCalc(Gimbal_PID_t *pid, Gimbal_Motor_Names_t MOTORx)
{	
	js_gimbal_yaw_angle_feedback = pid[YAW_205].Angle.feedback;
	js_gimbal_yaw_angle_target = pid[YAW_205].Angle.target;
	js_gimbal_pitch_angle_feedback = pid[PITCH_206].Angle.feedback;
	js_gimbal_pitch_angle_target = pid[PITCH_206].Angle.target;	
	
	pid[MOTORx].Angle.erro = pid[MOTORx].Angle.target - pid[MOTORx].Angle.feedback;
	js_gimbal_yaw_angle_erro = pid[YAW_205].Angle.erro;
	js_gimbal_pitch_angle_erro = pid[PITCH_206].Angle.erro;
	
	if((Gimbal.State.pid_mode == MECH) && (MOTORx == YAW_205)) {	// 计算有效误差
		if(pid[MOTORx].Angle.erro >= 4096.f) {
			pid[MOTORx].Angle.erro = -8192.f + pid[MOTORx].Angle.erro;
		} else if(pid[MOTORx].Angle.erro <= -4096.f) {
			pid[MOTORx].Angle.erro = 8192.f + pid[MOTORx].Angle.erro;
		}
	}	
	
	if(MOTORx == PITCH_206) {	// 计算有效误差
		if(pid[MOTORx].Angle.erro >= 4096.f) {
			pid[MOTORx].Angle.erro = -8192.f + pid[MOTORx].Angle.erro;
		} else if(pid[MOTORx].Angle.erro <= -4096.f) {
			pid[MOTORx].Angle.erro = 8192.f + pid[MOTORx].Angle.erro;
		}
	}		
	
//	if((Gimbal.State.pid_mode == GYRO) && (MOTORx == YAW_205)) {	// 计算有效误差
//		if(pid[MOTORx].Angle.erro >= 180.f*GIMBAL_GYRO_ANGLE_ZOOM_INDEX) {
//			pid[MOTORx].Angle.erro = -(360.f*GIMBAL_GYRO_ANGLE_ZOOM_INDEX - pid[MOTORx].Angle.erro);
//		} else if(pid[MOTORx].Angle.erro <= -180.f*GIMBAL_GYRO_ANGLE_ZOOM_INDEX) {
//			pid[MOTORx].Angle.erro = +(360.f*GIMBAL_GYRO_ANGLE_ZOOM_INDEX + pid[MOTORx].Angle.erro);
//		}
//	}
	
	/* 对误差进行卡尔曼滤波，消除低频等幅抖动 */
	if(Gimbal.State.mode == GIMBAL_MODE_NORMAL) {	// 正常模式下
		pid[MOTORx].Angle.erro = KalmanFilter(&Gimbal_kalmanError[Gimbal.State.pid_mode][MOTORx], pid[MOTORx].Angle.erro);
	}
	
	if(Gimbal.State.mode == GIMBAL_MODE_AUTO) {
		pid[MOTORx].Angle.erro = KalmanFilter(&Gimbal_kalmanError[Gimbal.State.pid_mode][MOTORx], pid[MOTORx].Angle.erro);
//		if(abs(pid[MOTORx].Angle.erro) < 0.5f) {
//			pid[MOTORx].Angle.erro = 0.f;
//		}
	}
	
	pid[MOTORx].Angle.integrate += pid[MOTORx].Angle.erro;

	/* Pout */
	pid[MOTORx].Angle.pout = pid[MOTORx].Angle.kp * pid[MOTORx].Angle.erro;
	
	/* Iout */
	pid[MOTORx].Angle.iout = pid[MOTORx].Angle.ki * pid[MOTORx].Angle.integrate * 0.002f; // 模仿19步兵代码(0.002f是2ms的意思)
	/* Iout Limits */
	pid[MOTORx].Angle.iout = constrain(pid[MOTORx].Angle.iout, -GIMBAL_ANGLE_PID_IOUT_MAX, GIMBAL_ANGLE_PID_IOUT_MAX);
	/* Dout*/
	pid[MOTORx].Angle.dout = pid[MOTORx].Angle.kd * (pid[MOTORx].Angle.erro - pid[MOTORx].Angle.last_erro)/0.002f;
	/* Record Last Error */
	pid[MOTORx].Angle.last_erro = pid[MOTORx].Angle.erro;

	/* Total PID Output*/
	pid[MOTORx].Angle.out = pid[MOTORx].Angle.pout + pid[MOTORx].Angle.iout + pid[MOTORx].Angle.dout;
	/* Total PID Output Limits */
	pid[MOTORx].Angle.out = constrain(pid[MOTORx].Angle.out, -GIMBAL_ANGLE_PID_OUT_MAX, GIMBAL_ANGLE_PID_OUT_MAX);
}

/**
 *	@brief	云台电机PID的最终输出
 */
void GIMBAL_PidOut(Gimbal_PID_t *pid)
{
	int16_t pidOut[4] = {0, 0, 0, 0};
	
	/* CAN发送电压值 */
	if(BitMask.Gimbal.BM_rxReport & BM_RX_REPORT_205) {
		pidOut[YAW_205] = (int16_t)pid[YAW_205].Out;		// 0x205
	}
	else {
		pidOut[YAW_205] = 0;	// 失联后卸力
	}
	
	if(BitMask.Gimbal.BM_rxReport & BM_RX_REPORT_206) {
		pidOut[PITCH_206] = (int16_t)pid[PITCH_206].Out;	// 0x206
	} 
	else {
		pidOut[PITCH_206] = 0;	// 失联后卸力
	}
	
	CAN1_QueueSend(0x1FF, pidOut);
}

/**
 *	@brief	云台卡尔曼滤波器初始化
 */
float k_Auto_Yaw_R = 20;
float k_Auto_Pitch_R = 10;
float k_Zx_Yaw_Q = 1;
float k_Zx_Yaw_R = 1000;
void GIMBAL_KalmanCreate(void)
{
	/* 卡尔曼滤波器初始化 */
	KalmanCreate(&Gimbal_kalmanError[MECH][YAW_205], 1, 40);
	KalmanCreate(&Gimbal_kalmanError[MECH][PITCH_206], 1, 60);
	KalmanCreate(&Gimbal_kalmanError[GYRO][YAW_205], 1, 40);
	KalmanCreate(&Gimbal_kalmanError[GYRO][PITCH_206], 1, 60);
	/* Mpu6050 */
	KalmanCreate(&Mpu_kalmanError[YAW_205], 1, 80);
	KalmanCreate(&Mpu_kalmanError[PITCH_206], 1, 10);	
	/* 自瞄 */
	KalmanCreate(&Gimbal_Auto_kalmanError[YAW_205], 1, k_Auto_Yaw_R);
	KalmanCreate(&Gimbal_Auto_kalmanError[PITCH_206], 1, k_Auto_Pitch_R);
	/* 自瞄卡尔曼滤波,二阶 */
	mat_init(&yaw_kalman_filter.Q,2,2, yaw_kalman_filter_para.Q_data);
	mat_init(&yaw_kalman_filter.R,2,2, yaw_kalman_filter_para.R_data);
	kalman_filter_init(&yaw_kalman_filter, &yaw_kalman_filter_para);
	
	mat_init(&pitch_kalman_filter.Q,2,2, pitch_kalman_filter_para.Q_data);
	mat_init(&pitch_kalman_filter.R,2,2, pitch_kalman_filter_para.R_data);
	kalman_filter_init(&pitch_kalman_filter, &pitch_kalman_filter_para);
	
	/* ZX卡尔曼滤波器 */
	KalmanCreate(&kalman_speedYaw, k_Zx_Yaw_Q, k_Zx_Yaw_R);
}

/* #信息层# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	云台设置模式
 */
void GIMBAL_SetMode(Gimbal_Mode_t mode)
{
	Gimbal.State.mode = mode;
}

/**
 *	@brief	返回云台当前的模式
 */
Gimbal_Mode_t GIMBAL_GetMode(void)
{
	return Gimbal.State.mode;
}

/**
 *	@brief	根据底盘控制逻辑反馈机械中值
 */
float GIMBAL_GetMiddleAngle(void)
{
	if(CHASSIS_GetLogic() == CHAS_LOGIC_NORMAL) {
		return GIMBAL_TOP_YAW_MID_ANGLE;
	} 
	else if(CHASSIS_GetLogic() == CHAS_LOGIC_REVERT) {
		return GIMBAL_REVERT_YAW_MID_ANGLE;
	}
	return GIMBAL_TOP_YAW_MID_ANGLE;
}

/**
 *	@brief	云台是否处于机械模式
 */
bool GIMBAL_IfMechMode(void)
{
	return (Gimbal.State.pid_mode == MECH);
}

/**
 *	@brief	云台是否处于陀螺仪模式
 */
bool GIMBAL_IfGyroMode(void)
{
	return (Gimbal.State.pid_mode == GYRO);
}

/**
 *	@brief	判断云台是否处于常规模式
 *	@return true - 常规模式
 *			false - 非常规模式
 */
bool GIMBAL_IfNormalMode(void)
{
	if(Gimbal.State.mode == GIMBAL_MODE_NORMAL) {
		return true;
	} else {
		return false;
	}
}

/**
 *	@brief	判断云台是否处于自瞄模式
 *	@return true - 自瞄模式
 *			false - 非自瞄模式
 */
bool GIMBAL_IfAutoMode(void)
{
	if(Gimbal.State.mode == GIMBAL_MODE_AUTO) {
		return true;
	} else {
		return false;
	}
}

/**
 *	@brief	判断云台是否处于打符模式
 *	@return true - 打符模式
 *			false - 非打符模式
 */
bool GIMBAL_IfBuffMode(void)
{
	if(Gimbal.State.mode == GIMBAL_MODE_BIG_BUFF || Gimbal.State.mode == GIMBAL_MODE_SMALL_BUFF) {
		return true;
	} else {
		return false;
	}
}

/**
 *	@brief	判断云台是否正在瞄准哨兵
 *	@return true - 正在瞄准哨兵
 *			false - 没有瞄准哨兵
 */
bool GIMBAL_IfAimSentry(void)
{
	/* 判断在击打哨兵(抬头pitch减小) */
	if((Gimbal_PID[GYRO][PITCH_206].Angle.feedback <= GIMBAL_AUTO_LOCK_SENTRY_ANGLE || Gimbal_PID[MECH][PITCH_206].Angle.feedback <= GIMBAL_AUTO_LOCK_SENTRY_ANGLE ) 
		&& Gimbal.State.mode == GIMBAL_MODE_AUTO) {
		return true;
	} else {
		return false;
	}
}

/**
 *	@brief	判断云台是否正在补子弹
 *	@return true - 正在补弹
 *			false - 没有正在补弹
 */
bool GIMBAL_IfReloadBullet(void)
{
	if(Gimbal.State.mode == GIMBAL_MODE_RELOAD_BULLET) {
		return true;
	} else {
		return false;
	}	
}

/**
 *	@brief	打符云台是否跟踪到位
 *	@return true - 跟踪到位可以打弹
 *			false - 跟踪未到位禁止打弹
 */
float debug_pix_yaw = 0;
float debug_pix_pitch = 0;
float debug_pix_yaw_ready = 35;
float debug_pix_pitch_ready = 35;
bool GIMBAL_BUFF_IfChaseReady(void)
{
	bool res = true;
	debug_pix_yaw = Gimbal.Buff.Yaw.erro;
	debug_pix_pitch = Gimbal.Buff.Pitch.erro;
	
	if((abs(debug_pix_yaw) < debug_pix_yaw_ready) && (VISION_GetFlagStatus(VISION_FLAG_LOCK_BUFF) == true)) {
		res &= true;
	} else {
		res &= false;
	}
	
	if((abs(debug_pix_pitch) < debug_pix_pitch_ready) && (VISION_GetFlagStatus(VISION_FLAG_LOCK_BUFF) == true)) {
		res &= true;
	} else {
		res &= false;
	}
	return res;
}

/**
 *	@brief	是否进入开火预测
 *	@return true - 已开启开火预测
 *			false - 未开启开火预测
 */
bool GIMBAL_IfAutoMobiPre(void)
{
	if(GIMBAL_IfAutoMode() == true) {
		return Mobi_Pre_Yaw;
	} else {	// 未开启自瞄不可能有预测
		return false;
	}
}

/**
 *	@brief	预测是否跟踪到位
 *	@return true - 跟踪到位可以打弹
 *			false - 跟踪未到位禁止打弹
 */
bool GIMBAL_AUTO_IfChaseReady(void)
{
	return Mobi_Pre_Yaw_Fire;
}

/**
 *	@brief	反馈底盘偏离设定的机械归中值的机械角度(-8191~+8191)
 *	@return 反馈值为机械角度差值(带正负号)
 *	@note	规定：以机械中值为轴，机械角度增加的方向为正方向
 */
float GIMBAL_GetTopGyroAngleOffset(void)
{
	float delta_angle, feedback;

	feedback = g_Gimbal_Motor_Info[YAW_205].angle;
	
	delta_angle = feedback - GIMBAL_TOP_YAW_MID_ANGLE;

	return delta_angle;
}

/**
 *	@brief	云台获取系统信息
 */
void GIMBAL_GetSysInfo(System_t *sys, Gimbal_Info_t *gim)
{
	/*----控制方式----*/
	/* 控制方式 - 遥控器 */
	if(sys->RemoteMode == RC) {
		gim->State.remote_mode = RC;
	} 
	/* 控制方式 - 键鼠 */
	else if(sys->RemoteMode == KEY) {
		gim->State.remote_mode = KEY;
	}
	
	/*----模式修改----*/
	switch(sys->Action)
	{
		case SYS_ACT_NORMAL: 
			{
				// 刚恢复常规模式
				if(gim->State.mode != GIMBAL_MODE_NORMAL) {
					Gimbal_PID[gim->State.pid_mode][YAW_205].Angle.target = Gimbal_PID[gim->State.pid_mode][YAW_205].Angle.feedback;
					Gimbal_PID[gim->State.pid_mode][PITCH_206].Angle.target = Gimbal_PID[gim->State.pid_mode][PITCH_206].Angle.feedback;
				}
				gim->State.mode = GIMBAL_MODE_NORMAL;
			}break;
		case SYS_ACT_AUTO: 
			{
				// 刚进入自瞄模式
				if(gim->State.mode != GIMBAL_MODE_AUTO)
					Gimbal.Auto.FLAG_first_into_auto = true;
				gim->State.mode = GIMBAL_MODE_AUTO;
			}break;
		case SYS_ACT_BUFF:
			{
				// 刚进入打符模式
				if(gim->State.mode != GIMBAL_MODE_SMALL_BUFF
					|| gim->State.mode != GIMBAL_MODE_BIG_BUFF)
					Gimbal.Buff.FLAG_first_into_buff = true;
				
				if(sys->BranchAction == BCH_ACT_SMALL_BUFF) {
					gim->State.mode = GIMBAL_MODE_SMALL_BUFF;
				}
				else if(sys->BranchAction == BCH_ACT_BIG_BUFF) {
					gim->State.mode = GIMBAL_MODE_BIG_BUFF;
				}
			}break;
		case SYS_ACT_PARK:
			{
				gim->State.mode = GIMBAL_MODE_RELOAD_BULLET;
			}break;
	}
	
	/*----Pid模式----*/
	if(sys->PidMode != gim->State.pid_mode)
	{
		/* GYRO -> MECH */
		if(sys->PidMode == MECH)
		{
			Gimbal_PID[MECH][YAW_205].Angle.target = Gimbal_PID[MECH][YAW_205].Angle.feedback;
			Gimbal_PID[MECH][PITCH_206].Angle.target = Gimbal_PID[MECH][PITCH_206].Angle.feedback;
		}
		/* MECH -> GYRO*/
		else if(sys->PidMode == GYRO)
		{
			Gimbal_PID[GYRO][YAW_205].Angle.target = Gimbal_PID[GYRO][YAW_205].Angle.feedback;
			Gimbal_PID[GYRO][PITCH_206].Angle.target = Gimbal_PID[GYRO][PITCH_206].Angle.feedback;			
		}
	}
	gim->State.pid_mode = sys->PidMode;
}

/**
 *	@brief	云台获取裁判系统信息
 */
void GIMBAL_GetJudgeInfo(Judge_Info_t *judge, Gimbal_Info_t *gim)
{
	/* 根据射速等级修改自瞄参数 */
	switch(judge->GameRobotStatus.robot_level)
	{
		case 0:
			k_level = K_LEVEL_0;	// 1/(12/30) = 2.5
		case 1:	
			k_level = K_LEVEL_1;	// 1/(15/30) = 2
			break;
		case 2:
			k_level = K_LEVEL_2;	// 1/(18/30) = 1.67
			break;
		case 3:
			k_level = K_LEVEL_3;	// 1/(30/30) = 1
			break;
		default:
			break;
	}
}


/* #应用层# ---------------------------------------------------------------------------------------------------------------------------------------*/
/* #遥控# -----------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	根据遥控值设置云台YAW位置环的期望值(累加值)
 *	@note	
 */
void REMOTE_SetGimbalAngle(void)
{
	float targetAngle;
	/* 机械模式 */
	if(GIMBAL_IfMechMode() == true) {	
		/* Yaw */
		/* 云台跟随底盘运动 */
		targetAngle = GIMBAL_GetMiddleAngle();
		Gimbal_PID[MECH][YAW_205].Angle.target = targetAngle;
		/* Pitch */
		targetAngle = RC_RIGH_CH_UD_VALUE * RC_GIMBAL_MECH_PITCH_SENSITIVY; // Pitch机械角为绝对角度
		Gimbal_PID[MECH][PITCH_206].Angle.target = constrain(Gimbal_PID[MECH][PITCH_206].Angle.target - targetAngle, // 抬头机械角度减小
																						GIMBAL_MECH_PITCH_UP_ANGLE, 
																						GIMBAL_MECH_PITCH_DOWN_ANGLE);	// 设置Pitch轴(绝对值)
	} 
	/* 陀螺仪模式 */
	else if(GIMBAL_IfGyroMode() == true) {	
		/* Yaw */
		targetAngle = RC_RIGH_CH_LR_VALUE * RC_GIMBAL_GYRO_YAW_SENSITIVY * (YAW_DIR);
		Gimbal_PID[GYRO][YAW_205].Angle.target += targetAngle;
		
		/* Pitch */		
		targetAngle = RC_RIGH_CH_UD_VALUE * RC_GIMBAL_GYRO_PITCH_SENSITIVY;
		Gimbal_PID[GYRO][PITCH_206].Angle.target = constrain(Gimbal_PID[GYRO][PITCH_206].Angle.target - targetAngle, // 抬头Pitch角度减小
																						GIMBAL_GYRO_PITCH_UP_ANGLE, 
																						GIMBAL_GYRO_PITCH_DOWN_ANGLE);	// 设置Pitch轴(绝对值)		
	}
}

/* #键盘鼠标# -------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	根据按键值设置云台YAW位置环的期望值(累加值)
 *	@note	
 *			鼠标右下为正
 */
float js_targetYaw;
float js_targetPitch;
void KEY_SetGimbalAngle(void)
{
	float targetAngle;
	
	js_targetYaw = MOUSE_X_MOVE_SPEED * KEY_GIMBAL_GYRO_YAW_SENSITIVY;
	js_targetPitch = MOUSE_Y_MOVE_SPEED * KEY_GIMBAL_GYRO_PITCH_SENSITIVY;
	
	/* Yaw */
	targetAngle = MOUSE_X_MOVE_SPEED * KEY_GIMBAL_GYRO_YAW_SENSITIVY * (YAW_DIR);	// 鼠标X反馈速度*灵敏度
	Gimbal_PID[GYRO][YAW_205].Angle.target += targetAngle;
	
	/* Pitch */		
	targetAngle = -MOUSE_Y_MOVE_SPEED * KEY_GIMBAL_GYRO_PITCH_SENSITIVY;// 鼠标Y反馈速度*灵敏度
	Gimbal_PID[GYRO][PITCH_206].Angle.target = constrain(Gimbal_PID[GYRO][PITCH_206].Angle.target - targetAngle, // 抬头Pitch角度减小
																					GIMBAL_GYRO_PITCH_UP_ANGLE, 
																					GIMBAL_GYRO_PITCH_DOWN_ANGLE);	// 设置Pitch轴(绝对值)	
}

/**
 *	@brief	根据按键值设置云台转头
 */
void KEY_SetGimbalTurn(void)
{
	float targetAngle = 0;
	static uint8_t keyQLockFlag = false;
	static uint8_t keyELockFlag = false;
	static uint8_t keyVLockFlag = false;
	/* 按键延时响应,防止手贱狂按 */
	static portTickType  keyCurrentTime = 0;	
	static uint32_t keyQLockTime = 0;
	static uint32_t keyELockTime = 0;
	static uint32_t keyVLockTime = 0;
		
	keyCurrentTime = xTaskGetTickCount();
	
	if(IF_KEY_PRESSED_Q) {	// 按下Q
		if(keyCurrentTime > keyQLockTime) {	// 250ms响应一次
			keyQLockTime = keyCurrentTime + TIME_STAMP_250MS;
			if(keyQLockFlag == false) {
				if(IF_KEY_PRESSED_E) {
					// 同时按下Q和E
				} else {	// 只按Q，没按E
					targetAngle = -90*GIMBAL_GYRO_ANGLE_ZOOM_INDEX * (YAW_DIR);
					Gimbal_PID[GYRO][YAW_205].AngleRampTarget += targetAngle;
				}
			}
			keyQLockFlag = true;
		}
	} else {	// 松开Q
		keyQLockFlag = false;
	}
	
	if(IF_KEY_PRESSED_E) {	// 按下E
		if(keyCurrentTime > keyELockTime) {	// 250ms响应一次
			keyELockTime = keyCurrentTime + TIME_STAMP_250MS;		
			if(keyELockFlag == false) {
				if(IF_KEY_PRESSED_Q) {
					// 同时按下Q和E
				} else {	// 只按E，没按Q
					targetAngle = +90*GIMBAL_GYRO_ANGLE_ZOOM_INDEX * (YAW_DIR);
					Gimbal_PID[GYRO][YAW_205].AngleRampTarget += targetAngle;
				}
			}
			keyELockFlag = true;
		}
	} else {
		keyELockFlag = false;
	}
	
	if(IF_KEY_PRESSED_V && !IF_KEY_PRESSED_CTRL) {	// 按下V
		if(keyCurrentTime > keyVLockTime) {	// 500ms响应一次
			keyVLockTime = keyCurrentTime + TIME_STAMP_500MS;
			
			/* 扭头就跑标志位 */
			Flag.Chassis.GoHome = true;
			
			if(keyVLockFlag == false) {
				if(IF_KEY_PRESSED_A) {
					// 同时按下AV 或 先按A再按V
					targetAngle = -180*GIMBAL_GYRO_ANGLE_ZOOM_INDEX * (YAW_DIR);
					Gimbal_PID[GYRO][YAW_205].AngleRampTarget += targetAngle;
				} else {	// 只按V，没按A
					targetAngle = +180*GIMBAL_GYRO_ANGLE_ZOOM_INDEX * (YAW_DIR);
					Gimbal_PID[GYRO][YAW_205].AngleRampTarget += targetAngle;
				}

			}
			keyVLockFlag = true;
		}
	} else {
		keyVLockFlag = false;
	}
	
	/* 斜坡函数给累加期望，防止突然增加很大的期望值 */
	Gimbal_PID[GYRO][YAW_205].AngleRampFeedback = RampFloat(Gimbal_PID[GYRO][YAW_205].AngleRampTarget, 
															Gimbal_PID[GYRO][YAW_205].AngleRampFeedback, 
															GIMBAL_GYRO_ANGLE_ZOOM_INDEX/1.5f);
	if(Gimbal_PID[GYRO][YAW_205].AngleRampFeedback < Gimbal_PID[GYRO][YAW_205].AngleRampTarget) // 正向累加
	{
		Gimbal_PID[GYRO][YAW_205].Angle.target = GIMBAL_GYRO_YawBoundaryProc(&Gimbal_PID[GYRO][YAW_205], 
																			GIMBAL_GYRO_ANGLE_ZOOM_INDEX/1.5f);
	} 
	else if(Gimbal_PID[GYRO][YAW_205].AngleRampFeedback > Gimbal_PID[GYRO][YAW_205].AngleRampTarget) // 反向累加
	{
		Gimbal_PID[GYRO][YAW_205].Angle.target = GIMBAL_GYRO_YawBoundaryProc(&Gimbal_PID[GYRO][YAW_205], 
																			-GIMBAL_GYRO_ANGLE_ZOOM_INDEX/1.5f);
	} 
	else // 缓冲池清零
	{
		/* 扭头到位标志位清零 */
		Flag.Chassis.GoHome = false;
		
		Gimbal_PID[GYRO][YAW_205].AngleRampTarget = 0;
		Gimbal_PID[GYRO][YAW_205].AngleRampFeedback = 0;
	}
}

/**
 *	@brief	根据按键值设置云台快速抬头
 */
void KEY_SetQuickPickUp(void)
{
	float targetAngle;
	static uint8_t keyGLockFlag = false;
	
	if(IF_KEY_PRESSED_G) {
		if(keyGLockFlag == false) {
			targetAngle = 15.f/360*8192;	// 快速抬头15°
			Gimbal_PID[GYRO][PITCH_206].Angle.target = constrain(Gimbal_PID[GYRO][PITCH_206].Angle.target - targetAngle, // 抬头Pitch角度减小
															 GIMBAL_GYRO_PITCH_UP_ANGLE, 
															 GIMBAL_GYRO_PITCH_DOWN_ANGLE);	// 设置Pitch轴(绝对值)
		}
		keyGLockFlag = true;
	} else {
		keyGLockFlag = false;
	}
}

/* #云台# -----------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	云台电机初始化
 *	@note		
 */
void GIMBAL_Init(void)
{
	GIMBAL_KalmanCreate();	// 创建卡尔曼滤波器
}

/**
 *	@brief	云台电机复位
 *	@note	# 先用机械模式归中
 *				# 具备5s的超时退出机制，防止云台没有复位到位一直卡死	
 */
void GIMBAL_Reset(void)
{
	float delta_angle;
	float target_angle;
	static portTickType time_prev;
	static portTickType time_now;
	
	target_angle = GIMBAL_GetMiddleAngle();
	
	/* 刚进入复位 */
	if(Flag.Gimbal.AngleRecordStart == true) {	
		// 记录上电时云台机械角度反馈值
		Gimbal_PID[MECH][YAW_205].Angle.target = Gimbal_PID[MECH][YAW_205].Angle.feedback;
		Gimbal_PID[MECH][PITCH_206].Angle.target = Gimbal_PID[MECH][PITCH_206].Angle.feedback;
		// 复位完成标志位和计数器清零
		Flag.Gimbal.ResetOk = false;
		Cnt.Gimbal.ResetOk = 0;
		// 开始计时
		time_now = xTaskGetTickCount();
		time_prev = time_now;
		
		/* 就近移动算法 */
		delta_angle = target_angle - Gimbal_PID[MECH][YAW_205].Angle.feedback;
		/* 没有越界取最近 */
		if(abs(delta_angle) < 4096) {
			Gimbal_PID[MECH][YAW_205].AngleRampTarget = delta_angle;
		} 
		/* 越界后作边界处理 */
		else {
			if(delta_angle >= 4096) {
				Gimbal_PID[MECH][YAW_205].AngleRampTarget = -8192 + delta_angle;
			} 
			else if(delta_angle <= -4096) {
				Gimbal_PID[MECH][YAW_205].AngleRampTarget = 8192 + delta_angle;
			}
		}		
		
		// 清除刚进入的标志位
		Flag.Gimbal.AngleRecordStart = false;
	}	
	
	/* 云台复位未完成 */
	if(Flag.Gimbal.ResetOk == false) {
		
		time_now = xTaskGetTickCount();	// 复位计时

		// 强制用机械模式复位
		Gimbal.State.pid_mode = MECH;

		/* 平缓地让云台移动到中间，防止上电狂甩
			 斜坡函数给累加期望，防止突然增加很大的期望值 */
		if(Gimbal_PID[MECH][YAW_205].AngleRampTarget > 0) {
			Gimbal_PID[MECH][YAW_205].Angle.target = GIMBAL_MECH_YawBoundaryProc(&Gimbal_PID[MECH][YAW_205], GIMBAL_RAMP_BEGIN_YAW);
		} 
		else if(Gimbal_PID[MECH][YAW_205].AngleRampTarget < 0) {
			Gimbal_PID[MECH][YAW_205].Angle.target = GIMBAL_MECH_YawBoundaryProc(&Gimbal_PID[MECH][YAW_205], -GIMBAL_RAMP_BEGIN_YAW);
		}
		
		Gimbal_PID[MECH][YAW_205].AngleRampTarget = RampFloat(0, Gimbal_PID[MECH][YAW_205].AngleRampTarget, GIMBAL_RAMP_BEGIN_YAW);
		
		/* 平缓地让云台移动到中间，防止上电狂甩 */
		Gimbal_PID[MECH][PITCH_206].Angle.target = RampFloat(GIMBAL_MECH_PITCH_MID_ANGLE, Gimbal_PID[MECH][PITCH_206].Angle.target, GIMBAL_RAMP_BEGIN_PITCH);
		
		/* 等待云台归中 */
		if(abs(Gimbal_PID[MECH][YAW_205].Angle.feedback - target_angle) <= 1 
			&& abs(Gimbal_PID[MECH][PITCH_206].Angle.feedback - GIMBAL_MECH_PITCH_MID_ANGLE) <= 1) {
			Cnt.Gimbal.ResetOk++;
		}
			
		/* 复位成功或者超时强制退出(5s) */
		if(Cnt.Gimbal.ResetOk > 250 || (time_now - time_prev) >= TIME_STAMP_5000MS) {	
			Cnt.Gimbal.ResetOk = 0;
			Flag.Gimbal.ResetOk = true;// 云台复位成功
			/* 记录起始陀螺仪角度 */
			start_gyro_yaw = Gimbal_PID[GYRO][YAW_205].Angle.feedback;
			last_gyro_yaw = start_gyro_yaw;
			delta_gyro_yaw = 0;
		}		
	} 
	/* 云台复位完成 */
	else if(Flag.Gimbal.ResetOk == true) {
		Flag.Gimbal.ResetOk = false;	// 清除状态标志位
		BM_Reset(BitMask.System.BM_Reset, BM_RESET_GIMBAL);
	}	
}

/**
 *	@brief	云台陀螺仪模式记录反馈数据
 *	@note
 *					# 处理Pitch角度的临界值问题
 *					1. 枪管向下 - -162.4°
 *					2. 枪管向上 - +145.8°
 *					先±180得到一个小于90°的数值
 *					规定; 
 *					下边为负( > -17°)
 *					上边为正( < +34°)
 *
 *					# 处理Yaw角度的临界值问题
 *						# 法①:
 *							Yaw每次上电都会不一样，本身的数据源自相对坐标系
 *							相同的规律是顺时针角度值一定是减小的
 *					1. 枪管向左 - -59.6°
 *					2. 枪管向右 - +115°
 *					先±180得到一个小于90°的数值
 *						# 法②：
 *					上述方法由于yaw轴陀螺仪数据的漂移导致效果不佳，因此采用机械模式下的角度
 *					反馈来作为限制云台分离的角度(即云台抵达限制角度后只能增加另一个方向的期望)。
 *					
 *				  # 上电后MPU6050数据会有一段时间不稳定(可做等待处理)
 *					# 机械模式下的速度反馈值利用IMU的角速度反馈值
 *						原因：在手动移动云台yaw和pitch方向时。
 *							  # Yaw
 *							  陀螺仪数据的反馈最大可达5000+(不用乘缩放系数)
 *							  而电机的转速反馈最大可达50+
 *							  # Pitch
 *							  陀螺仪数据的反馈最大可达3000+(不用乘缩放系数)
 *							  而电机的转速反馈最大可达30+
 *							  对比可以知道陀螺仪反馈的角速度值精度更高，因此
 *							  陀螺仪模式和机械模式下的速度环均采用IMU的反馈数据
 *
 *					# YAW轴电机	右摆 电机的out为 +
 *								左摆 电机的out为 -
 *					# PITCH电机 抬头 电机的out为 -
 *								低头 电机的out为 +
 *
 *	@direction
 *			IMU方向：
 *					抬头	- ratePitch => -
 *							-	Pitch 	=> + 
 *					低头 	- ratePitch => +
 *							- 	Pitch 	=> -
 *					左摆  	- rateYaw   => -
 *							- 	Yaw   	=> +
 *					右摆  	- rateYaw   => +
 *							- 	Yaw		=> -
 *			规定:
 *					抬头	- ratePitch => -
 *								- Pitch => -(陀螺仪模式) 
 *					低头 	- ratePitch => +
 *								- Pitch => +(陀螺仪模式)
 *					左摆  	- rateYaw   => +
 *								- Yaw   => +(陀螺仪模式)
 *					右摆  	- rateYaw   => -
 *								- Yaw	=> -(陀螺仪模式)
 */
float pitch;
float yaw;
float rateYaw;
float ratePitch;
void GIMBAL_GetImuInfo(Gimbal_PID_t pid[GIMBAL_MODE_COUNT][GIMBAL_MOTOR_COUNT])
{	
	static float last_yaw = 0.f;
	float delta_yaw;
	
	/* # Yaw # */
	yaw = Mpu_Info.yaw;
	delta_yaw = yaw - last_yaw;
	if(delta_yaw > +180.f) {
		delta_yaw = -360.f + delta_yaw;//(+360.f - delta_yaw);
	} else if(delta_yaw < -180.f) {
		delta_yaw = +360.f + delta_yaw;//(-360.f - delta_yaw);
	}
	last_yaw = yaw; 	
	
	/* # 陀螺仪模式下的角度反馈值利用IMU的角度值 */
	pid[GYRO][YAW_205].Angle.feedback += delta_yaw * GIMBAL_GYRO_ANGLE_ZOOM_INDEX;
	
	/* # Yaw 速度反馈 # */
	rateYaw = Mpu_Info.rateYaw - Mpu_Info.rateYawOffset;

	/* # 机械模式下的速度反馈值利用IMU的角速度值 */
	pid[MECH][YAW_205].Speed.feedback = (YAW_DIR) * rateYaw;	// GIMBAL_COMPENSATE_RATEYAW
	/* # 陀螺仪模式下的速度反馈值利用IMU的角速度值 */
	pid[GYRO][YAW_205].Speed.feedback = (YAW_DIR) * rateYaw;	// GIMBAL_COMPENSATE_RATEYAW
	
	/* # Pitch 速度反馈 # */
	ratePitch = Mpu_Info.ratePitch - Mpu_Info.ratePitchOffset;

	/* # Pitch # */
	pitch = -Mpu_Info.pitch;
	/* # 陀螺仪模式下的角度反馈值利用Pitch机械角度反馈值 */
	pid[GYRO][PITCH_206].Angle.feedback = g_Gimbal_Motor_Info[PITCH_206].angle;	
	/* # 机械模式下的速度反馈值利用IMU的角速度值 */
	pid[MECH][PITCH_206].Speed.feedback = ratePitch;	// 抬头速度为-	GIMBAL_COMPENSATE_RATEPITCH
	/* # 陀螺仪模式下的速度反馈值利用IMU的角速度值 */
	pid[GYRO][PITCH_206].Speed.feedback = ratePitch;	// 抬头速度为-	GIMBAL_COMPENSATE_RATEPITCH
}

/**
 *	@brief	机械模式下云台YAW期望值(累加值)边界处理
 */
float GIMBAL_MECH_YawBoundaryProc(Gimbal_PID_t *pid, float delta_target)
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
 *	@brief	陀螺仪模式下云台YAW期望值(累加值)边界处理
 */
float GIMBAL_GYRO_YawBoundaryProc(Gimbal_PID_t *pid, float delta_target)
{
	float target;
	target = pid->Angle.target + delta_target;
//	if(target >= 180.0f*GIMBAL_GYRO_ANGLE_ZOOM_INDEX) {	// 超过右边界
//		target = (-360.f*GIMBAL_GYRO_ANGLE_ZOOM_INDEX + target);
//	} else if(target <= -180.0f*GIMBAL_GYRO_ANGLE_ZOOM_INDEX){// 超过左边界
//		target = (+360.f*GIMBAL_GYRO_ANGLE_ZOOM_INDEX + target);
//	}
	return target;
}

/**
 *	@brief	目标速度解算
 *	@note		利用两帧之间的角度差算出目标的速度
 */
float speed_calculate(Speed_Calculate_t *info, uint32_t time, float position)
{
	info->delay_cnt++;
	if(time != info->last_time)
	{
		if((position - info->last_position) >= (180.f*GIMBAL_GYRO_ANGLE_ZOOM_INDEX)) {
			info->speed = (+360.f*GIMBAL_GYRO_ANGLE_ZOOM_INDEX - (position - info->last_position))/(time - info->last_time)*2;	// 帧差法计算速度(*2感觉是速度放大的意思)
		} else if((position - info->last_position) <= (-180.f*GIMBAL_GYRO_ANGLE_ZOOM_INDEX)) {
			info->speed = (-360.f*GIMBAL_GYRO_ANGLE_ZOOM_INDEX - (position - info->last_position))/(time - info->last_time)*2;	// 帧差法计算速度(*2感觉是速度放大的意思)
		} else {
			info->speed = (position - info->last_position)/(time - info->last_time)*2;	// 帧差法计算速度(*2感觉是速度放大的意思)
		}
		
		info->processed_speed = info->speed;
			
		info->last_time = time;
		info->last_position = position;
		info->last_speed = info->speed;
		info->delay_cnt = 0;
	}
	
	/* 由于时间更新是在视觉数据更新里面进行的，因此如果视觉失联超过一定时间后将反馈速度清零 */
	if(info->delay_cnt > 250)	// 500ms
	{
		info->processed_speed = 0;
	}
	
	return info->processed_speed;
}



/**
 *	@brief	自瞄模式PID计算视觉预测版
 *	@note		3.75f -> 射速提高之后需要重新计算一个补偿值
 */
float GIMBAL_AUTO_YAW_COMPENSATION = 0.f;	// 调试的时候用的
float GIMBAL_AUTO_PITCH_COMPENSATION = (0.f/360.0f*8192.0f);
float auto_yaw_ramp = 260;
float auto_pitch_ramp = 80;
void GIMBAL_VISION_AUTO_PidCalc(Gimbal_PID_t pid[GIMBAL_MODE_COUNT][GIMBAL_MOTOR_COUNT], Gimbal_Info_t *gimbal)
{
	/* 计算自瞄帧率 */
	gimbal->Auto.Time[NOW] = xTaskGetTickCount();
	gimbal->Auto.Time[DELTA] = gimbal->Auto.Time[NOW] - gimbal->Auto.Time[PREV];

//	/* >60ms 则认为丢失目标后再识别到 */
//	if(gimbal->Auto.Time[DELTA] > TIME_STAMP_60MS) {
//		pid[GYRO][YAW_205].Speed.integrate = 0;
//		pid[GYRO][PITCH_206].Speed.integrate = 0;
//	}
	
	if(gimbal->Auto.FLAG_first_into_auto == true) {
		gimbal->Auto.FLAG_first_into_auto = false;
		gimbal->Auto.Yaw.target = pid[GYRO][YAW_205].Angle.feedback;
		gimbal->Auto.Pitch.target = pid[GYRO][PITCH_206].Angle.feedback;	
	}
	
	/* 数据更新 */
	if(VISION_GetFlagStatus(VISION_FLAG_DATA_UPDATE) == true) {
		/* 清除数据更新标志位，防止未接收到新数据的情况下重复进入 */
		VISION_ClearFlagStatus(VISION_FLAG_DATA_UPDATE);	

		if(VISION_GetFlagStatus(VISION_FLAG_LOCK_TARGET) == true) {
			/* Yaw 误差计算 */
			gimbal->Auto.Yaw.erro = gimbal->Auto.Yaw.kp * VISION_GYRO_GetYawFeedback();
			//gimbal->Auto.Yaw.erro = KalmanFilter(&Gimbal_Auto_kalmanError[YAW_205], gimbal->Auto.Yaw.erro);
			gimbal->Auto.Yaw.target = pid[GYRO][YAW_205].Angle.feedback + gimbal->Auto.Yaw.erro;
			/* Pitch 误差计算 */
			gimbal->Auto.Pitch.erro = (gimbal->Auto.Pitch.kp * VISION_MECH_GetPitchFeedback()) - GIMBAL_AUTO_PITCH_COMPENSATION;
			//gimbal->Auto.Pitch.erro = KalmanFilter(&Gimbal_Auto_kalmanError[PITCH_206], gimbal->Auto.Pitch.erro);
			gimbal->Auto.Pitch.target = pid[GYRO][PITCH_206].Angle.feedback + gimbal->Auto.Pitch.erro;
		} else {
			/* 可能存在问题 */
			gimbal->Auto.Yaw.erro = 0;
			gimbal->Auto.Yaw.target = pid[GYRO][YAW_205].Angle.feedback;
			gimbal->Auto.Pitch.erro = 0;
			gimbal->Auto.Pitch.target = pid[GYRO][PITCH_206].Angle.feedback;
		}
	}
	
	/*----期望修改----*/
	pid[GYRO][YAW_205].Angle.target = RampFloat(gimbal->Auto.Yaw.target, pid[GYRO][YAW_205].Angle.feedback, auto_yaw_ramp);
	pid[GYRO][PITCH_206].Angle.target = RampFloat(gimbal->Auto.Pitch.target, pid[GYRO][PITCH_206].Angle.feedback, auto_pitch_ramp);	
//	pid[GYRO][YAW_205].Angle.target = pid[GYRO][YAW_205].Angle.feedback;
//	pid[GYRO][YAW_205].Angle.target = GIMBAL_GYRO_YawBoundaryProc(&pid[GYRO][YAW_205], gimbal->Auto.Yaw.erro);
//	pid[GYRO][PITCH_206].Angle.target = pid[GYRO][PITCH_206].Angle.feedback + gimbal->Auto.Pitch.erro;	
	
	gimbal->Auto.Time[PREV] = gimbal->Auto.Time[NOW];
}

#define AUTO_CTRL_WAY	2

/**
 *	@brief	自瞄模式电控预测版
 *	@note		云台右摆(目标右移) - 目标速度为正
 *					   									目标角度为正
 *					
 *					!use_zx
 *					ramp - 20
 *					yaw_predict_k 35		
 *
 *					!use_zx 						(效果较差)
 *					等步数 - 20步
 *					yaw_predict_k 1000
 *					
 *					use_zx 35
 *					ramp - 20
 *					
 *					use_zx 350					(感觉效果较好)
 *					step - 10
 */
Speed_Calculate_t	yaw_angle_speed_struct, pitch_angle_speed_struct;
float *yaw_kf_result, *pitch_kf_result;	// 二阶卡尔曼滤波结果,0角度 1速度
float yaw_angle_speed, pitch_angle_speed;
float yaw_angle_predict, pitch_angle_predict;
float yaw_predict_k = 0.f;
float pitch_predict_k = 0.f;

float auto_predict_start_delay = 0.f;
float auto_predict_start_delay_boundary = 150;

float auto_yaw_predict_max = 12.f * GIMBAL_GYRO_ANGLE_ZOOM_INDEX;
float auto_pitch_predict_max = 8;	// 这个待测试 8 ≈ 0.35°

float auto_yaw_speed_predict_low = 0.35;
float auto_yaw_speed_predict_high = 25;

float auto_pitch_speed_predict_low = 0.05; 	// 测试发现正常平移速度会在0.05~0.10，如果是步兵前装甲板正对则情况不一样
float auto_pitch_speed_predict_high = 2;		// 这个待测试
	
float auto_yaw_angle_predict_limit = 220;
float auto_yaw_angle_predict_ramp = 20;

float auto_pitch_angle_predict_limit = 120;	// 这个待测试

float js_yaw_angle = 0.f;
float js_yaw_speed = 0.f;
float js_pitch_angle = 0.f;
float js_pitch_speed = 0.f;

/* zx预测算法 */
float visionYawAngleRaw=0,cloudYawAngleRaw=0,targetYawAngleRaw=0,targetYawSpeedRaw=0;
float visionYawAngleKF=0,cloudYawAngleKF=0,targetYawAngleKF=0,targetYawSpeedKF=0;
float deathzoom = 0.f;
float ksc = 1.f;
float use_zx = 350.f;

/* 队列算法 */
uint8_t queue_num = 10;
uint8_t lost_num = 8;

/* 距离补偿 */
float kDis = 0.f;	// 距离影响因子(这里有bug)
float kPitch = 0.5f; //	抬头补偿因子 
float distance = 0.f;
float dis_temp = 0.f;
float distance_death = 2.5f;
float dis_compensation = 0.f;
float pitch_compensation =0.f;

/* 等步长算法 */
uint8_t test_ramp_step = 1;
float 	step = 10.f;
uint8_t step_cnt = 10.f;
float 	step_death = 0.f;

/* 开火延迟 */
float mobpre_yaw_boundary = 0.5f*GIMBAL_GYRO_ANGLE_ZOOM_INDEX;	// 移动预测yaw的边界
float stoppre_yaw_boundary = 1.f*GIMBAL_GYRO_ANGLE_ZOOM_INDEX;	// 停止预测yaw的边界
uint16_t mobpre_yaw_left_delay = 0;//向左预测延时判断可开火消抖
uint16_t mobpre_yaw_right_delay = 0;//向右预测延时判断可开火消抖
uint16_t mobpre_yaw_stop_delay = 0;//预测关闭延时判断可开火消抖
	
/* 掉帧处理后视觉 */
uint8_t vision_truly_lost = LOST;	
float kLost = 2.f;	// 掉帧补偿因子
float last_distance = 0.f;
float last_speed = 0.f;
float lost_compensation = 0.f;

#if (AUTO_CTRL_WAY == 1)
void GIMBAL_AUTO_PidCalc(Gimbal_PID_t pid[GIMBAL_MODE_COUNT][GIMBAL_MOTOR_COUNT], Gimbal_Info_t *gimbal)
{
	static float avrCalAngle,nowCalAngle=0;
	float none=0;
	
	float yaw_predict_temp, pitch_predict_temp;
	
	/* 首次进入自瞄 */
	if(gimbal->Auto.FLAG_first_into_auto == true) {
		gimbal->Auto.FLAG_first_into_auto = false;
		gimbal->Auto.Yaw.target = pid[GYRO][YAW_205].Angle.feedback;
		gimbal->Auto.Pitch.target = pid[GYRO][PITCH_206].Angle.feedback;	
		cQueue_fill(&LostQueue, lost_num, LOST);
	}
	
	/*----数据更新----*/
	if(VISION_GetFlagStatus(VISION_FLAG_DATA_UPDATE) == true) {
		/* 清除数据更新标志位，防止未接收到新数据的情况下重复进入 */
		VISION_ClearFlagStatus(VISION_FLAG_DATA_UPDATE);	

		/* 计算自瞄帧率 */
		gimbal->Auto.Time[NOW] = xTaskGetTickCount();
		gimbal->Auto.Time[DELTA] = gimbal->Auto.Time[NOW] - gimbal->Auto.Time[PREV];
		gimbal->Auto.Time[PREV] = gimbal->Auto.Time[NOW];
		
		/* 识别到目标 */
		if(VISION_GetFlagStatus(VISION_FLAG_LOCK_TARGET) == true) {
			/* Yaw 误差放大 */
			gimbal->Auto.Yaw.erro = gimbal->Auto.Yaw.kp * VISION_GYRO_GetYawFeedback() + GIMBAL_AUTO_YAW_COMPENSATION;
			/* Pitch 误差放大 */
			gimbal->Auto.Pitch.erro = (gimbal->Auto.Pitch.kp * VISION_MECH_GetPitchFeedback()) + GIMBAL_AUTO_PITCH_COMPENSATION;
			/* 掉帧处理 */
			cQueue_in(&LostQueue, lost_num, FOUND);
		} 
		/* 未识别到目标 */
		else {
			/* Yaw 误差为0 */
			gimbal->Auto.Yaw.erro = 0;
			/* Pitch 误差为0 */
			gimbal->Auto.Pitch.erro = 0;
			/* 掉帧处理 */
			cQueue_in(&LostQueue, lost_num, LOST);
		}
		/* Yaw 目标计算 */
		gimbal->Auto.Yaw.target = pid[GYRO][YAW_205].Angle.feedback + gimbal->Auto.Yaw.erro;
		/* Pitch 目标计算 */
		gimbal->Auto.Pitch.target = pid[GYRO][PITCH_206].Angle.feedback + gimbal->Auto.Pitch.erro;
		
		/* 更新步长 */
		step_cnt = step;
	}
	
	/* 掉帧判断 */
	if(cQueue_ifFilled(&LostQueue, lost_num, LOST)) {
		vision_truly_lost = LOST;
	} else if(cQueue_ifFilled(&LostQueue, lost_num, FOUND)) {
		vision_truly_lost = FOUND;
	}
	
	/*----更新二阶卡尔曼预测值----*/
	/* 识别到目标 */
	if(VISION_GetFlagStatus(VISION_FLAG_LOCK_TARGET) == true) {	
		/* 更新二阶卡尔曼速度先验估计值 */
		yaw_angle_speed = speed_calculate(&yaw_angle_speed_struct, gimbal->Auto.Time[NOW], gimbal->Auto.Yaw.target);
		pitch_angle_speed = speed_calculate(&pitch_angle_speed_struct, gimbal->Auto.Time[NOW], gimbal->Auto.Pitch.target);

		/* 对角度和速度进行二阶卡尔曼滤波融合,0位置,1速度 */
		yaw_kf_result = kalman_filter_calc(&yaw_kalman_filter, gimbal->Auto.Yaw.target, yaw_angle_speed);
		pitch_kf_result = kalman_filter_calc(&pitch_kalman_filter, gimbal->Auto.Pitch.target, pitch_angle_speed);
		
		/* ↓ ZX预测速度 ↓ */
		nowCalAngle = yaw_kf_result[KF_ANGLE];	// gimbal->Auto.Yaw.target
		
		VISION_UpdateInfo(nowCalAngle, queue_num, &VisionQueue, &avrCalAngle, &none);
		
		targetYawSpeedRaw = (nowCalAngle - avrCalAngle)*ksc;
		
		if(abs(targetYawSpeedRaw) < deathzoom)
			targetYawSpeedRaw = 0.f;
		
		targetYawSpeedKF = KalmanFilter(&kalman_speedYaw, targetYawSpeedRaw);
		/* ↑ ZX预测速度 ↑ */
		last_speed = targetYawSpeedKF;
		last_distance = VISION_GetDistance()/1000.f;	//换算成(m)
	} 
	/* 未识别到目标 */
	else {
		/* 更新二阶卡尔曼速度先验估计值 */
		yaw_angle_speed = speed_calculate(&yaw_angle_speed_struct, xTaskGetTickCount(), pid[GYRO][YAW_205].Angle.feedback);	
		pitch_angle_speed = speed_calculate(&pitch_angle_speed_struct, xTaskGetTickCount(), pid[GYRO][PITCH_206].Angle.feedback);

		/* 对角度和速度进行二阶卡尔曼滤波融合,0位置,1速度 */
		yaw_kf_result = kalman_filter_calc(&yaw_kalman_filter, pid[GYRO][YAW_205].Angle.feedback, 0);		// 识别不到时认为目标速度为0
		pitch_kf_result = kalman_filter_calc(&pitch_kalman_filter, pid[GYRO][PITCH_206].Angle.feedback, 0);	// 识别不到时认为目标速度为0
		
		/* ↓ ZX预测速度 ↓ */
		nowCalAngle = yaw_kf_result[KF_ANGLE];	// gimbal->Auto.Yaw.target
		
		VISION_UpdateInfo(nowCalAngle, queue_num, &VisionQueue, &avrCalAngle, &none);
		
		targetYawSpeedRaw = (nowCalAngle - avrCalAngle)*ksc;
		
		if(abs(targetYawSpeedRaw) < deathzoom)
			targetYawSpeedRaw = 0.f;
		
		targetYawSpeedKF = KalmanFilter(&kalman_speedYaw,0);
		/* ↑ ZX预测速度 ↑ */
	}
	
	js_yaw_speed = yaw_kf_result[KF_SPEED];
	js_yaw_angle = yaw_kf_result[KF_ANGLE];
	
	/*----加上距离补偿(假定是线性关系)----*/
	distance = VISION_GetDistance()/1000.f;	//换算成(m)
	
	pitch_compensation = distance * kPitch;
	
	if(myDeathZoom(0, distance_death, distance) > 0.f) {
		dis_temp -= distance_death;
	} else {
		dis_temp = 0.f;
	}	
	
	/*----预测量计算----*/
	/* 识别到目标 */
	if(VISION_GetFlagStatus(VISION_FLAG_LOCK_TARGET) == true) {	
		auto_predict_start_delay++;	// 滤波延时开启
		
		/* zx预测 */
		if(use_zx != 0)
		{
			/* 预测开启条件判断 */
			if(auto_predict_start_delay > auto_predict_start_delay_boundary 
					&& abs(gimbal->Auto.Yaw.erro) < auto_yaw_predict_max 
					&& abs(targetYawSpeedKF) > auto_yaw_speed_predict_low
					&& abs(targetYawSpeedKF) < auto_yaw_speed_predict_high)
			{
				auto_predict_start_delay = auto_predict_start_delay_boundary + 1;
				
				/* 计算距离补偿 */
				dis_compensation = dis_temp*kDis;
				
				/* 目标右移 -> 预测量为负(云台右移) */
				if(targetYawSpeedKF < 0) {
					yaw_predict_temp = use_zx*(targetYawSpeedKF + auto_yaw_speed_predict_low) - dis_compensation; // 时间常数2ms归入预测系数中
				} else if(targetYawSpeedKF >= 0) {
					yaw_predict_temp = use_zx*(targetYawSpeedKF - auto_yaw_speed_predict_low) + dis_compensation; // 时间常数2ms归入预测系数中
				}
				
				/* 预测量缓慢变化 */
				if(test_ramp_step == 0)
					yaw_angle_predict = RampFloat(yaw_predict_temp, yaw_angle_predict, auto_yaw_angle_predict_ramp);	// StepFloat();
				else 
					yaw_angle_predict = StepFloat(yaw_predict_temp, step, &step_cnt, step_death);
				/* 预测量限幅 */
				yaw_angle_predict = constrain(yaw_angle_predict, -auto_yaw_angle_predict_limit, auto_yaw_angle_predict_limit);
				
				/*----期望修改----*/
				pid[GYRO][YAW_205].Angle.target = yaw_kf_result[KF_ANGLE] + yaw_angle_predict;	// gimbal->Auto.Yaw.target 滤波前实时性稍微较好，但从识别不到进入到识别时会有阶跃期望
			
				/*----预测到位判断----*/
				/* 目标右移而且视觉误差表明目标在左边(表示已超前) */
				if(targetYawSpeedKF < 0 
					&& (gimbal->Auto.Yaw.erro - GIMBAL_AUTO_YAW_COMPENSATION) > mobpre_yaw_boundary)
				{
					mobpre_yaw_left_delay = 0;	// 向左预测开火延时重置
					
					mobpre_yaw_right_delay++;	// 累加向右移动预测
					if(mobpre_yaw_right_delay > 0) {
						Mobi_Pre_Yaw_Fire = true;
					} else {
						Mobi_Pre_Yaw_Fire = false;
					}
				}
				/* 目标左移而且视觉误差表明目标在右边(表示已超前) */
				else if(targetYawSpeedKF > 0
					&& (gimbal->Auto.Yaw.erro - GIMBAL_AUTO_YAW_COMPENSATION) < -mobpre_yaw_boundary)
				{
					mobpre_yaw_right_delay = 0;	// 向右预测开火延时重置
					
					mobpre_yaw_left_delay++;		// 累加向左移动预测
					if(mobpre_yaw_left_delay > 0) {
						Mobi_Pre_Yaw_Fire = true;
					} else {
						Mobi_Pre_Yaw_Fire = false;
					}		
				}
				/* 预测未到位 */
				else {
					Mobi_Pre_Yaw_Fire = false;
					
					mobpre_yaw_left_delay = 0;
					mobpre_yaw_right_delay = 0;
				}
				
				Mobi_Pre_Yaw = true;	// 标记已开启移动预测
				mobpre_yaw_stop_delay = 0;	// 重置静止时的开火延迟
				
			}
			/* 未达到预测开启条件 */
			else {
				pid[GYRO][YAW_205].Angle.target = gimbal->Auto.Yaw.target;//RampFloat(gimbal->Auto.Yaw.target, pid[GYRO][YAW_205].Angle.feedback, auto_yaw_ramp);
				
				Mobi_Pre_Yaw = false;	// 标记未开启移动预测
				mobpre_yaw_left_delay = 0;
				mobpre_yaw_right_delay = 0;
				
				if(abs(gimbal->Auto.Yaw.erro - GIMBAL_AUTO_YAW_COMPENSATION) < stoppre_yaw_boundary) 
				{
					mobpre_yaw_stop_delay++;
					if(mobpre_yaw_stop_delay > 25)	// 稳定50ms
					{
						Mobi_Pre_Yaw_Fire = true;
					}
					else
					{
						Mobi_Pre_Yaw_Fire = false;
					}
				}
			}
		}
		/* 19预测 */
		else {
			/* 预测开启条件判断 */
			if(auto_predict_start_delay > auto_predict_start_delay_boundary 
					&& abs(gimbal->Auto.Yaw.erro) < auto_yaw_predict_max 
					&& abs(yaw_kf_result[KF_SPEED]) > auto_yaw_speed_predict_low
					&& abs(yaw_kf_result[KF_SPEED]) < auto_yaw_speed_predict_high)
			{
				auto_predict_start_delay = auto_predict_start_delay_boundary + 1;
				
				/* 计算距离补偿 */
				dis_compensation = dis_temp*kDis;
				
				/* 目标右移 -> 预测量为负(云台右移) */
				if(yaw_kf_result[KF_SPEED] < 0) {
					yaw_predict_temp = yaw_predict_k*(yaw_kf_result[KF_SPEED] + auto_yaw_speed_predict_low) - dis_compensation; // 时间常数2ms归入预测系数中
				} else if(yaw_kf_result[KF_SPEED] >= 0) {
					yaw_predict_temp = yaw_predict_k*(yaw_kf_result[KF_SPEED] - auto_yaw_speed_predict_low) + dis_compensation; // 时间常数2ms归入预测系数中
				}
				
				/* 预测量缓慢变化 */
				if(test_ramp_step == 0)
					yaw_angle_predict = RampFloat(yaw_predict_temp, yaw_angle_predict, auto_yaw_angle_predict_ramp);	// StepFloat();
				else 
					yaw_angle_predict = StepFloat(yaw_predict_temp, step, &step_cnt, step_death);
				/* 预测量限幅 */
				yaw_angle_predict = constrain(yaw_angle_predict, -auto_yaw_angle_predict_limit, auto_yaw_angle_predict_limit);
				
				/*----期望修改----*/
				pid[GYRO][YAW_205].Angle.target = yaw_kf_result[KF_ANGLE] + yaw_angle_predict;	// gimbal->Auto.Yaw.target

				/*----预测到位判断----*/
				/* 目标右移而且视觉误差表明目标在左边(表示已超前) */
				if(yaw_kf_result[KF_SPEED] < 0 
					&& (gimbal->Auto.Yaw.erro - GIMBAL_AUTO_YAW_COMPENSATION) > mobpre_yaw_boundary)
				{
					mobpre_yaw_right_delay = 0;	// 向右预测开火延时重置
					
					mobpre_yaw_left_delay++;	// 累加向左移动预测
					if(mobpre_yaw_left_delay > 0) {
						Mobi_Pre_Yaw_Fire = true;
					} else {
						Mobi_Pre_Yaw_Fire = false;
					}
				}
				/* 目标左移而且视觉误差表明目标在右边(表示已超前) */
				else if(yaw_kf_result[KF_SPEED] > 0 
					&& (gimbal->Auto.Yaw.erro - GIMBAL_AUTO_YAW_COMPENSATION) < -mobpre_yaw_boundary)
				{
					mobpre_yaw_right_delay = 0;	// 向右预测开火延时重置
					
					mobpre_yaw_left_delay++;		// 累加向左移动预测
					if(mobpre_yaw_left_delay > 0) {
						Mobi_Pre_Yaw_Fire = true;
					} else {
						Mobi_Pre_Yaw_Fire = false;
					}		
				}
				/* 预测未到位 */
				else {
					Mobi_Pre_Yaw_Fire = false;
					
					mobpre_yaw_left_delay = 0;
					mobpre_yaw_right_delay = 0;
				}
				
				Mobi_Pre_Yaw = true;	// 标记已开启预测
				mobpre_yaw_stop_delay = 0;	// 重置静止时的开火延迟
				
			}
			/* 未达到预测开启条件 */
			else {
				/*----期望修改----*/
				pid[GYRO][YAW_205].Angle.target = gimbal->Auto.Yaw.target;//RampFloat(gimbal->Auto.Yaw.target, pid[GYRO][YAW_205].Angle.feedback, auto_yaw_ramp);

				Mobi_Pre_Yaw = false;	// 标记未开启预测
				mobpre_yaw_left_delay = 0;
				mobpre_yaw_right_delay = 0;
				
				if(abs(gimbal->Auto.Yaw.erro - GIMBAL_AUTO_YAW_COMPENSATION) < stoppre_yaw_boundary) 
				{
					mobpre_yaw_stop_delay++;
					if(mobpre_yaw_stop_delay > 25)	// 稳定50ms
					{
						Mobi_Pre_Yaw_Fire = true;
					}
					else
					{
						Mobi_Pre_Yaw_Fire = false;
					}
				}
			}
		}
		
		/* Pitch轴给个很小的预测 */
		if(auto_predict_start_delay > auto_predict_start_delay_boundary
				&& abs(gimbal->Auto.Pitch.erro) < auto_pitch_predict_max 
					&& abs(pitch_kf_result[KF_SPEED]) > auto_pitch_speed_predict_low
					&& abs(pitch_kf_result[KF_SPEED]) < auto_pitch_speed_predict_high)
		{
			/* 目标下移 -> 预测量为正(云台低头) */
			if(pitch_kf_result[KF_SPEED] >= 0) {
				pitch_predict_temp = pitch_predict_k * (pitch_kf_result[KF_SPEED] - auto_pitch_speed_predict_low);
			}
			/* 目标上移 -> 预测量为负(云台抬头) */
			else if(pitch_kf_result[KF_SPEED] < 0){
				pitch_predict_temp = pitch_predict_k * (pitch_kf_result[KF_SPEED] + auto_pitch_speed_predict_low);
			}
			
			/* 预测量限幅 */
			pitch_predict_temp = constrain(pitch_predict_temp, -auto_pitch_angle_predict_limit, auto_pitch_angle_predict_limit);
			pitch_angle_predict = pitch_predict_temp;
			
			/*----期望修改----*/
			pid[GYRO][PITCH_206].Angle.target = pitch_kf_result[KF_ANGLE] + pitch_angle_predict;	// pitch_kf_result[KF_ANGLE]
		}
		/* 未达到预测开启的条件 */
		else {
			/*----期望修改----*/
			pid[GYRO][PITCH_206].Angle.target = gimbal->Auto.Pitch.target;//RampFloat(gimbal->Auto.Pitch.target, pid[GYRO][PITCH_206].Angle.feedback, auto_pitch_ramp);
		}
	}
	/* 未识别到目标可以随意移动云台 */
	else {
		/* 重置延时 */
		auto_predict_start_delay = 0;	// 卡尔曼滤波器延时
		mobpre_yaw_left_delay = 0;		// 重置左预测开火延迟
		mobpre_yaw_right_delay = 0;		// 重置右预测开火延迟
		mobpre_yaw_stop_delay = 0;		// 重置停止预测开火延迟
		
		KEY_SetGimbalAngle();
		
//		if(vision_truly_lost == LOST) {
//			last_speed = 0;
//			last_distance = 0;
//			lost_compensation = 0;
//			/* # 这里先简化成普通的键盘移动，后续再优化 */
//			/*----期望修改----*/
//			KEY_SetGimbalAngle();
//		} else {
//			/* 掉帧补偿 */
//			if(last_distance != 0)
//				lost_compensation += kLost * last_speed / last_distance;	// 将时间因子归入kLost
//			else
//				lost_compensation = 0;
//			
//			Gimbal_PID[GYRO][YAW_205].Angle.target = gimbal->Auto.Yaw.target + lost_compensation;
//			Gimbal_PID[GYRO][PITCH_206].Angle.target = gimbal->Auto.Pitch.target;
//		}
	}
}
#endif

/**
 *	@brief	云台自瞄预测
 *	@author	Lzx
 */
uint8_t RAMP_MAX_CNT		= 10;
uint8_t ACTIVE_MAX_CNT		= 10;
uint8_t PREDICT_DELAY_CNT	= 100;
uint8_t LOST_MAX_CNT		= 10;

float cloud_yaw_raw;
float cloud_yaw_kf;
float cloud_degree;
float vision_yaw;
float vision_degree;
float vision_yaw_raw;
float update_cloud_yaw;
float vision_yaw_kf;
float vision_dis_raw;
float vision_dis_kf;
float target_yaw_raw;
float k_scale_vision;
float target_yaw_kf;
float now_cal_yaw;
float target_speed_raw;
float target_degree;
uint8_t queue_speed_length = 10;
uint8_t queue_accel_length = 10;
float k_speed;
float target_speed_kf;
float target_accel_raw;
float target_accel_kf;
float predict_angle_raw;
float predict_angle_degree;
float k_ff;
float feedforward_angle;
float k_pre;
float predict_angle;
uint8_t use_ff;
uint8_t predict_ramp_cnt = 10;
uint8_t vision_online_flag;
uint16_t vision_update_fps;

uint16_t predict_delay=0;/*预测的延时函数*/		


void VISION_AUTO_Predict(Gimbal_PID_t pid[GIMBAL_MODE_COUNT][GIMBAL_MOTOR_COUNT], Gimbal_Info_t *gimbal)
{
	static uint16_t ramp_cnt=0;/*斜坡计数 -- 用于缓和掉帧影响*/			
	
	static bool judge_flag=false;/*丢失标志位激活标志位,用于一次性操作*/
	
	
	static uint16_t active_cnt=0,lost_cnt=0;/*激活计数/丢失计数--用于识别和未识别到相互切换的过程*/
	
	static uint32_t vision_this_time,vision_last_time;/*视觉接受的时间*/					
	
	static uint8_t vision_identify_target=0, last_identify_flag=0;/*上一次的接收标志位*/
	
	static float now_vision_yaw=0;/*当前获得的视觉yaw角度--相对坐标角度*/				
	
	uint32_t outline_cnt=0;	/*离线计数*/
	
//	float now_gyro_yaw=0;/*当前和上次的机械yaw角度 -- 云台角度*/

//	static bool init=true;/*初始化标志位*/		
//	
//	if(init == true)
//	{
//		start_gyro_yaw = pid[GYRO][YAW_205].Angle.feedback;
//		last_gyro_yaw = start_gyro_yaw;
//		init = false;
//		//初始化过程
//	}	
	
	if(Gimbal.Auto.FLAG_first_into_auto == true)
	{
		//start_gyro_yaw = pid[GYRO][YAW_205].Angle.feedback;
		//last_gyro_yaw = start_gyro_yaw;
		//delta_gyro_yaw = 0;
		last_identify_flag = 0;
		predict_delay = PREDICT_DELAY_CNT;	/*刚进自瞄的时候延时预测超前，防止云台晃动*/
		Gimbal.Auto.FLAG_first_into_auto = false;
		//初始化过程		
	}
	
	now_gyro_yaw = pid[GYRO][YAW_205].Angle.feedback; /*当前的yaw轴陀螺仪角度值*/
	/*这里的pid是云台Yaw轴陀螺仪的结构体，这里主要是为了更新yaw轴云台电机的陀螺仪角度*/ 

	delta_gyro_yaw += now_gyro_yaw - last_gyro_yaw; 	/*计算两次采样的陀螺仪角度差值*/
	/*这里是计算两次采样的陀螺仪角度之间的差值*/ 
	
	
	cloud_yaw_raw = delta_gyro_yaw;  /*进入卡尔曼滤波*/
	/*这里是为了将差值进行卡尔曼滤波*/
	 
	last_gyro_yaw = now_gyro_yaw; /*记录上一次的角度值*/
	/*更新记录上一次采样的陀螺仪角度*/ 

	cloud_yaw_kf = KalmanFilter(&kalman_cloudYaw,cloud_yaw_raw)+start_gyro_yaw;	/*滤波*/       
	
	cloud_degree = cloud_yaw_kf / GIMBAL_GYRO_ANGLE_ZOOM_INDEX;  
	/*这里是将前后两次采样的差值进行卡尔曼滤波后再加上起始角度，来作为滤波后的云台陀螺仪角度*/ 
	
	/*
		这里这么做是为了避开机械角度的边界问题，这个方法同样可以解决陀螺仪产生的边界而导致无法直接对角度数据进行滤波的问题。 你细细品 
	*/	
	
	/*以上是对视觉原始数据的处理*/

	if(VISION_GetFlagStatus(VISION_FLAG_DATA_UPDATE) == true)
	{
		vision_identify_target = VISION_GetFlagStatus(VISION_FLAG_LOCK_TARGET);
			
		vision_yaw = VISION_GYRO_GetYawFeedback();

		
//		/*当视觉数据更新后进来*/	
//		if(last_identify_flag != vision_identify_target)	/*识别的状态发生了切换*/
//		{
//			/*若发生了掉帧或刚识别到目标或丢失了目标，则进到这里*/ 
//			
//			ramp_cnt = RAMP_MAX_CNT;	/*切换后斜坡过渡*/
//			/*这里原本的ramp_cnt为0.现在赋值为不为0的数，表示要对数据进行斜坡处理*/ 
//			
//			judge_flag = true;			/*进入判定过程*/
//			/*触发识别状态的判定条件，即当出现识别模式的切换时，开始进入判断过程，判断是否真的跟丢或者只是掉帧或者刚识别到目标*/ 
//			

//			//掉帧后重新判断识别状态
//		}		

//		if(judge_flag == true)		/*识别的状态发生了切换*/
//		{
//			/*
//				这里的内容是哨兵侦察和识别之间的切换代码，实际上其他兵种可以不看，根据需要写上自己的切换过程代码。不过也可以看一下 参考一下思路，你细细品。 
//			*/ 
//			
//			if(vision_identify_target == 1)		/*表示为发生了掉帧重新进入状态判定过程--这里不影响继续跟踪识别-因为模式没有改变*/
//			{
//				active_cnt++; 	/*活跃计数*/
//				
//				lost_cnt = 0;
//				
//				if(active_cnt >= ACTIVE_MAX_CNT) /*达到阈值，认定为识别到*/
//				{
//					judge_flag = false;
//					
//					//gimbal_mode = ATTACK;
//				
//					predict_delay = PREDICT_DELAY_CNT;  /*用于延迟超前角的加入时间*/
//					
//					active_cnt = 0;
//					
//					/*重新确认进来的判断*/
//				}	
//			}
//			else
//			{
//				lost_cnt++;	 		/*丢失计数*/
//				
//				active_cnt = 0;

//				if(lost_cnt >= LOST_MAX_CNT) /*达到阈值，认定为丢失*/
//				{
//					judge_flag = false;
//					
//					//gimbal_mode = SCOUT; 	
//					
//					
//					lost_cnt = 0;					
//					/*侦察模式的切换*/
//				}
//			}
//			/*进入判定后不侦察*/	
//			 
//			/*到这里是侦察和识别的切换*/ 
//		}	

		/*↑↑↑ 模式判定过程 ↑↑↑*/
		
		/*
			下面就是斜坡的过程了
			如果发生了切换的情况，则这里会进入斜坡并持续一小段时间，来缓和由于掉帧带来的冲击。具体的根据实际情况采用，可不用， 
		*/ 
//		if(ramp_cnt > 0)
//		{
//			/*斜坡过渡*/
//			now_vision_yaw = RampFloat(vision_yaw, now_vision_yaw, abs(vision_yaw - now_vision_yaw)/ramp_cnt);
//			ramp_cnt--;
//		}
//		else
			now_vision_yaw = vision_yaw;                          
		
		vision_degree = now_vision_yaw / GIMBAL_GYRO_ANGLE_ZOOM_INDEX;
		/* ↑↑↑获取当前的视觉角度↑↑↑ */ 
		
		update_cloud_yaw = cloud_yaw_raw+start_gyro_yaw; 			/*视觉数据更新时的云台角度*/
		/*视觉数据更新时的云台数据*/ 
		
		vision_yaw_raw = now_vision_yaw;	/*与陀螺仪数据尺度相同*/    
		/*视觉的最新数据，转换成和陀螺仪角度的同一个尺度*/ 
		
		
		vision_yaw_kf = KalmanFilter(&kalman_visionYaw,vision_yaw_raw); 	/*对视觉角度数据做卡尔曼滤波*/  
		/*视觉数据的卡尔曼滤波*/ 
		
		
		vision_dis_raw = VISION_GetDistance()/1000.f;	/*计算距离*/
		/*获取距离数据*/  
	
	
		vision_dis_kf = KalmanFilter(&kalman_dist,vision_dis_raw);	/*滤波*/
		/*距离数据卡尔曼滤波器*/ 
		
		
		vision_this_time = xTaskGetTickCount();		/*获取当前时间*/
		/*更新视觉更新的时间 -- 在最后有用到*/ 
		
		VISION_ClearFlagStatus(VISION_FLAG_DATA_UPDATE);	/*清除标志位*/	

		/*
			之所以将数据更新的内容放在更新标志位里，是为了保证预测数据可以不受云台自身运动的影响。
			仔细说了就是因为数据的更新频率远小于云台的角度数据，在视觉没更新的时间点里，云台的数据一直在更新。
			这导致了云台的运动会给预测的速度带来扰动数据。这里的处理就是为了尽可能地避免这里地噪声 
		*/ 
	}
	/*↑↑↑数据更新的处理↑↑↑*/
		
	target_yaw_raw = -vision_yaw_kf*k_scale_vision + cloud_yaw_kf;		/*对目标的期望角度*/
	/*目标的位置的yaw角度预测，这里的符号是因为器件安装包括云台和摄像头的安装之间的关系。具体看自己的安装关系。*/ 


	target_yaw_kf = KalmanFilter(&kalman_targetYaw,target_yaw_raw);		/*对目标的yaw角度进行定位*/   
	
	target_degree=target_yaw_kf / GIMBAL_GYRO_ANGLE_ZOOM_INDEX;  
	/*对目标角度进行滤波*/ 
	
	
	now_cal_yaw = -vision_yaw_raw*k_scale_vision + update_cloud_yaw;	 /*目标的角度 -- 用于计算目标速度*/
	/*开始预测速度，这里的k_scale_vision是摄像头角度变化的转化系数。
	
		举个例子，摄像头的视觉数据变化了10°，但是实际上云台转过了15°，那这个时候就需要给视觉的数据乘上一个1.5，此时的k_scale_vision就为1.5 
	*/

	
	
	target_speed_raw = Get_Target_Speed(queue_speed_length,now_cal_yaw)*k_speed; 	/*计算出对目标速度的预测*/
	/*开始对目标的速度进行预测，这个函数直接参考我给的代码。k_speed是缩放系数，因为得到的速度数据比较小，因此放大后放到卡尔曼比较合适*/ 
	
	
	if(vision_identify_target == 1)				/*计算速度值*/
		target_speed_kf = KalmanFilter(&kalman_speedYaw,target_speed_raw);
	else
		target_speed_kf = KalmanFilter(&kalman_speedYaw,0);			
	
	/*参考去年的做法。*/ 
	
	
	target_accel_raw = Get_Target_Accel(queue_accel_length,target_speed_kf);	 /*获取加速度*/
	
	target_accel_raw = myDeathZoom(0,0.1,target_accel_raw);		/*死区处理 - 滤除0点附近的噪声*/
	
	target_accel_kf = KalmanFilter(&kalman_accel,target_accel_raw);		/*卡尔曼滤波*/

	/*上面三行是解算加速度的数据，跟速度的解算原理一样，不多做解释*/		
		
	if(predict_delay>0)
	{
		/*预测的延时*/
		predict_delay--;
		predict_angle_raw=0;
		
		/*这里predict的延时跟去年的代码有点像，都是为了让云台先摆到目标附近在进行预测用的，
		否则会导致云台晃动，原因就是因为云台晃动会给预测的速度带来影响，从而形成正反馈的数据震荡过程。 */ 
	}
	else
	{
		/*延时结束后，进入预测过程*/
		float tmp1;	 /*临时变量*/
		
		if((target_accel_kf*target_speed_kf)>=0)
		{
			/*加速度和速度同向时*/
			tmp1 = 0.8f; 
		}
		else
		{
			tmp1 = -1.2f;
		}
		
		/*这里用来判断加速度和速度关系，来决定加速度得作用方向
			
			例如 加速度和速度反向时，说明减速运动
				加速度和速度同向时，说明加速运动。 
		*/ 
		
		feedforward_angle = k_ff * target_accel_kf; 	/*计算前馈角*/
		
		feedforward_angle = constrain(feedforward_angle,-4.f,4.f);	 /*前馈角*/
		
		/*前馈角度时通过加速度来对目标运动的提前反应。*/ 
		
		predict_angle_raw = k_level*(k_pre*target_speed_kf*vision_dis_kf + tmp1*feedforward_angle*use_ff);	/*计算预测角度*/
	}
	predict_angle_raw = constrain(predict_angle_raw,-8*GIMBAL_GYRO_ANGLE_ZOOM_INDEX,8*GIMBAL_GYRO_ANGLE_ZOOM_INDEX);	/*限幅*/
	
	predict_angle_degree = predict_angle_raw / GIMBAL_GYRO_ANGLE_ZOOM_INDEX;
	
	predict_angle = RampFloat(predict_angle_raw, predict_angle, (abs(predict_angle_raw - predict_angle)/predict_ramp_cnt)); 	/*斜坡处理*/
	
	/*这里是对预测的滤波*/ 
	
	//attack_pitch_offset = k_pitch * vision_dis_kf; 	/*抬头距离补偿*/
	/*对于距离的pitch角度抬头，但是没有加入对pitch方向的预测。后续补充上。*/ 
	
	
	/*以上是控制所需要的角度计算过程*/
	
	outline_cnt = xTaskGetTickCount() - vision_this_time; 	/*离线计数*/
	/*计算视觉最近一帧的更新时间，来判断是否已经离线*/ 

	if(outline_cnt >= 1000)
	{
		/*离线*/
		VISION_ClearRxPacket();
		vision_online_flag = false;		
	}
	else
	{
		vision_online_flag=true;
	}
	/*判定是否离线*/
	/*离线要有对应的处理，否则云台暴走。*/ 	
	
	
	if(vision_this_time != vision_last_time)
	{
		/*计算帧率*/
		vision_update_fps = vision_this_time - vision_last_time;
		
		vision_last_time = vision_this_time;
		
		/*计算帧率*/ 
	}
	
	last_identify_flag = vision_identify_target;
	//记录上一次标志位	
	
}

#if (AUTO_CTRL_WAY == 2)
void GIMBAL_AUTO_PidCalc(Gimbal_PID_t pid[GIMBAL_MODE_COUNT][GIMBAL_MOTOR_COUNT], Gimbal_Info_t *gimbal)
{
	VISION_AUTO_Predict(pid, gimbal);
	
	mobpre_yaw_boundary = (k_level/K_LEVEL_3) * (target_speed_kf/auto_yaw_speed_predict_high) * auto_yaw_predict_max;	// 
	
	
	if(VISION_GetFlagStatus(VISION_FLAG_LOCK_TARGET) == true)
	{
		pid[GYRO][YAW_205].Angle.target = target_yaw_kf + predict_angle;
		
		//pid[GYRO][PITCH_206].Angle.target = target_pitch_kf;
		
		/*----预测到位判断----*/
		if(predict_delay == 0 
				&& abs(vision_yaw_kf) < auto_yaw_predict_max 
				&& abs(target_speed_kf) > auto_yaw_speed_predict_low
				&& abs(target_speed_kf) < auto_yaw_speed_predict_high)
		{			
			/* 目标右移而且视觉误差表明目标在左边(表示已超前) */
			if(target_speed_kf < 0 
				&& vision_yaw_kf > mobpre_yaw_boundary)
			{
				mobpre_yaw_left_delay = 0;	// 向左预测开火延时重置
				
				mobpre_yaw_right_delay++;	// 累加向右移动预测
				if(mobpre_yaw_right_delay > 0) {
					Mobi_Pre_Yaw_Fire = true;
				} else {
					Mobi_Pre_Yaw_Fire = false;
				}
			}
			/* 目标左移而且视觉误差表明目标在右边(表示已超前) */
			else if(target_speed_kf > 0
				&& vision_yaw_kf < -mobpre_yaw_boundary)
			{
				mobpre_yaw_right_delay = 0;	// 向右预测开火延时重置
				
				mobpre_yaw_left_delay++;		// 累加向左移动预测
				if(mobpre_yaw_left_delay > 0) {
					Mobi_Pre_Yaw_Fire = true;
				} else {
					Mobi_Pre_Yaw_Fire = false;
				}		
			}
			/* 预测未到位 */
			else {
				Mobi_Pre_Yaw_Fire = false;
				
				mobpre_yaw_left_delay = 0;
				mobpre_yaw_right_delay = 0;
			}
			
			Mobi_Pre_Yaw = true;	// 标记已开启移动预测
			mobpre_yaw_stop_delay = 0;	// 重置静止时的开火延迟
		
		}
		/* 未达到预测开启条件 */
		else {
			pid[GYRO][YAW_205].Angle.target = target_yaw_kf;//RampFloat(gimbal->Auto.Yaw.target, pid[GYRO][YAW_205].Angle.feedback, auto_yaw_ramp);
			
			Mobi_Pre_Yaw = false;	// 标记未开启移动预测
			mobpre_yaw_left_delay = 0;
			mobpre_yaw_right_delay = 0;
			
			if(abs(vision_yaw_kf) < stoppre_yaw_boundary) 
			{
				mobpre_yaw_stop_delay++;
				if(mobpre_yaw_stop_delay > 25) {// 稳定50ms
					Mobi_Pre_Yaw_Fire = true;
				} else {
					Mobi_Pre_Yaw_Fire = false;
				}
			}
		}		
	}
	else
	{
		mobpre_yaw_left_delay = 0;		// 重置左预测开火延迟
		mobpre_yaw_right_delay = 0;		// 重置右预测开火延迟
		mobpre_yaw_stop_delay = 0;		// 重置停止预测开火延迟
		
		KEY_SetGimbalAngle();		
	}
}
#endif

/**
 *	@brief	计算速度和预测角度
 */
void GIMBAL_CalPredictInfo(void)
{
//	/* 非自瞄模式下也更新 */
//	if(!GIMBAL_IfAutoMode()) {
//		/* 更新二阶卡尔曼速度先验估计值 */
//		yaw_angle_speed = speed_calculate(&yaw_angle_speed_struct, xTaskGetTickCount(), Gimbal_PID[GYRO][YAW_205].Angle.feedback);	
//		pitch_angle_speed = speed_calculate(&pitch_angle_speed_struct, xTaskGetTickCount(), Gimbal_PID[GYRO][PITCH_206].Angle.feedback);

//		/* 对角度和速度进行二阶卡尔曼滤波融合,0位置,1速度 */
//		yaw_kf_result = kalman_filter_calc(&yaw_kalman_filter, Gimbal_PID[GYRO][YAW_205].Angle.feedback, 0);		// 识别不到时认为目标速度为0
//		pitch_kf_result = kalman_filter_calc(&pitch_kalman_filter, Gimbal_PID[GYRO][PITCH_206].Angle.feedback, 0);	// 识别不到时认为目标速度为0

//		js_yaw_speed = yaw_kf_result[KF_SPEED];
//		js_yaw_angle = yaw_kf_result[KF_ANGLE];
//	}

	
	if( !GIMBAL_IfAutoMode() ) 
	{
		now_gyro_yaw = Gimbal_PID[GYRO][YAW_205].Angle.feedback; /*当前的yaw轴陀螺仪角度值*/
		/*这里的pid是云台Yaw轴陀螺仪的结构体，这里主要是为了更新yaw轴云台电机的陀螺仪角度*/ 

		delta_gyro_yaw += now_gyro_yaw - last_gyro_yaw; 	/*计算两次采样的陀螺仪角度差值*/
		/*这里是计算两次采样的陀螺仪角度之间的差值*/ 
		
		cloud_yaw_raw = delta_gyro_yaw;  /*进入卡尔曼滤波*/
		/*这里是为了将差值进行卡尔曼滤波*/
		 
		last_gyro_yaw = now_gyro_yaw; /*记录上一次的角度值*/
		/*更新记录上一次采样的陀螺仪角度*/ 

		cloud_yaw_kf = KalmanFilter(&kalman_cloudYaw,cloud_yaw_raw)+start_gyro_yaw;	/*滤波*/       
		
		cloud_degree = cloud_yaw_kf / GIMBAL_GYRO_ANGLE_ZOOM_INDEX;  
		/*这里是将前后两次采样的差值进行卡尔曼滤波后再加上起始角度，来作为滤波后的云台陀螺仪角度*/ 
		
		target_speed_raw = Get_Target_Speed(queue_speed_length,now_cal_yaw)*k_speed; 	/*计算出对目标速度的预测*/
		/*开始对目标的速度进行预测，这个函数直接参考我给的代码。k_speed是缩放系数，因为得到的速度数据比较小，因此放大后放到卡尔曼比较合适*/ 
		
		target_speed_kf = KalmanFilter(&kalman_speedYaw,0);			
		
		/*参考去年的做法。*/ 
		
		
		target_accel_raw = Get_Target_Accel(queue_accel_length,target_speed_kf);	 /*获取加速度*/
		
		target_accel_raw = myDeathZoom(0,0.1,target_accel_raw);		/*死区处理 - 滤除0点附近的噪声*/
		
		target_accel_kf = KalmanFilter(&kalman_accel,target_accel_raw);		/*卡尔曼滤波*/

		/*上面三行是解算加速度的数据，跟速度的解算原理一样，不多做解释*/		
		
	}
}

/**
 *	@brief	打符模式PID计算
 *	@note	红方打红符、蓝方打蓝符
 *			两个阶段：
 *			①小符
 *				简介：5:59~4:00允许打符，打符成功则获得1.5倍攻击，持续1分钟
 *				运动策略：10RPM
 *			②大符
 *				简介：2:59~0:00允许打符，打符成功则获得2倍攻击，50%防御加成，持续1分钟
 *			
 *			地形信息：
 *			①桥面离地1米2,大风车中心离地2米283,内径70cm,外径80cm
 *			②能量机关激活点
 *				能量机关可激活状态时，己方机器人占领并停留3s，枪口热量每秒冷却值变为5倍
 *				2.5s未击中/击中其它装甲板 -》激活失败
 *
 *			控制思路：
 *			进入打符模式的时候，云台斜坡归中(以视觉反馈的坐标原点为中心)
 *
 *			# 在家测试条件：
 *			投影直线距离：7.40m
 *			大符中心距离地面：1.55m
 *			枪管距离大符中心：7.56m(勾股定理)
 *	
 *			# 摄像头在云台（摄像头在底盘的可参考往年代码）
 *
 *			# 根据地图的测绘图估计能量机关激活点距离大符7.50m
 */
float buff_yaw_ramp;
float buff_pitch_ramp;
void GIMBAL_BUFF_PidCalc(Gimbal_PID_t pid[GIMBAL_MODE_COUNT][GIMBAL_MOTOR_COUNT], Gimbal_Info_t *gimbal)
{
	static float yaw_gyro_mid, yaw_mech_mid;
	static float pitch_gyro_mid, pitch_mech_mid;
	
	static uint16_t into_buff_time = 0;
	static uint16_t lost_cnt = 0;
	
	/* 计算打符帧率 */
	gimbal->Buff.Time[NOW] = xTaskGetTickCount();
	gimbal->Buff.Time[DELTA] = gimbal->Auto.Time[NOW] - gimbal->Auto.Time[PREV];

//	/* 切换pitch补偿角 */
//	if((GIMBAL_MECH_PITCH_DOWN_ANGLE - pid[GYRO][PITCH_206].Angle.feedback) > 0)
//		GIMBAL_BUFF_PITCH_COMPENSATION = BUFF_PITCH_MODIFY_TABLE[(uint8_t)((GIMBAL_MECH_PITCH_DOWN_ANGLE - pid[GYRO][PITCH_206].Angle.feedback)/100)];
//	else
//		GIMBAL_BUFF_PITCH_COMPENSATION = 0;
	
	if(gimbal->Buff.FLAG_first_into_buff == true) {
		gimbal->Buff.FLAG_first_into_buff = false;
		into_buff_time = 0;
		/* 记录刚进入打符模式时的陀螺仪角度 */
		yaw_gyro_mid = pid[GYRO][YAW_205].Angle.feedback;
		pitch_gyro_mid = pid[GYRO][PITCH_206].Angle.feedback;
		/* 记录刚进入打符模式时的机械角度 */
		yaw_mech_mid = pid[MECH][YAW_205].Angle.feedback;
		pitch_mech_mid = pid[MECH][PITCH_206].Angle.feedback;
	} else {
		/* 防止加得太大 */
		if(into_buff_time < 250)	
			into_buff_time++;
	}
	
	/* 数据更新 */
	if(VISION_GetFlagStatus(VISION_FLAG_DATA_UPDATE) == true) {
		/* 清除数据更新标志位，防止未接收到新数据的情况下重复进入 */
		VISION_ClearFlagStatus(VISION_FLAG_DATA_UPDATE);	
		if(VISION_GetFlagStatus(VISION_FLAG_LOCK_BUFF) == true) {	// 识别到目标则更新误差
			lost_cnt = 0;
			if(into_buff_time > 100) {
				/* Yaw 误差计算 */
				gimbal->Buff.Yaw.erro = gimbal->Buff.Yaw.kp * VISION_BUFF_GetYawFeedback() + GIMBAL_BUFF_YAW_COMPENSATION;
				gimbal->Buff.Yaw.target = pid[GYRO][YAW_205].Angle.feedback + gimbal->Buff.Yaw.erro;
				/* Pitch 误差计算 */
				gimbal->Buff.Pitch.erro = (gimbal->Buff.Pitch.kp * VISION_BUFF_GetPitchFeedback()) + GIMBAL_BUFF_PITCH_COMPENSATION;
				gimbal->Buff.Pitch.target = pid[GYRO][PITCH_206].Angle.feedback + gimbal->Buff.Pitch.erro;
				/*----期望修改----*/
				pid[GYRO][YAW_205].Angle.target = RampFloat(gimbal->Buff.Yaw.target, pid[GYRO][YAW_205].Angle.feedback, GIMBAL_BUFF_YAW_RAMP);
				pid[GYRO][PITCH_206].Angle.target = RampFloat(gimbal->Buff.Pitch.target, pid[GYRO][PITCH_206].Angle.feedback, GIMBAL_BUFF_PITCH_RAMP);				
			} else {
				/*----期望修改----*/
				pid[GYRO][YAW_205].Angle.target = yaw_gyro_mid;
				pid[GYRO][PITCH_206].Angle.target = pitch_gyro_mid;
			}
		} else {	// 未识别到目标则保持原位置
			gimbal->Buff.Yaw.erro = 0;
			gimbal->Buff.Pitch.erro = 0;
			if(lost_cnt > 50) {	// 连续50帧丢失
				if(lost_cnt == 51) {	// 只赋值一次
					/*----期望修改----*/
					pid[GYRO][YAW_205].Angle.target = pid[GYRO][YAW_205].Angle.feedback;
					pid[GYRO][YAW_205].Angle.target = GIMBAL_GYRO_YawBoundaryProc(&Gimbal_PID[GYRO][YAW_205], 
													((yaw_mech_mid - pid[MECH][YAW_205].Angle.feedback)*360/8192.f));
					pid[GYRO][PITCH_206].Angle.target = pitch_mech_mid;	// Pitch采用机械模式
					lost_cnt++;
				}
			} else {
				lost_cnt++;
			}
		}
	}
	
	/*----期望修改----*/
	//..自动打弹
	
	gimbal->Buff.Time[PREV] = gimbal->Buff.Time[NOW];
}

/**
 *	@brief	云台常规控制
 */
void GIMBAL_NormalCtrl(void)
{
	KEY_SetGimbalAngle();
	KEY_SetGimbalTurn();
	KEY_SetQuickPickUp();	
}

/**
 *	@brief	云台自瞄控制
 */
void GIMBAL_AutoCtrl(void)
{
	/* 视觉数据可用 && 键盘模式下 */
	if( VISION_IsDataValid() ) 
	{
		/*----期望修改----*/
		/* 视觉预测版 */
		//GIMBAL_VISION_AUTO_PidCalc(Gimbal_PID, &Gimbal);
		/* 电控预测版 */
		GIMBAL_AUTO_PidCalc(Gimbal_PID, &Gimbal);
	}	
}

/**
 *	@brief	云台打符控制
 *	@note	
 *	1.先把一块装甲常亮放到最下面，调红外，调offsetx和offsety把红外点到装甲板中心。
 *
 *	2.旋转你的装甲板，看你的红外是不是一个完整的圆，就是说红外在你的距离给定的时候，不管装甲板旋转那边，红外点一直跟着。
 *
 *	3.调抬头补偿。
 *
 *	4.这个时候因为红外与枪管经常不平衡，所以打弹，把offsetx与offsety调到打中。
 *
 *	5.各个位置试着打。
 *
 *	6.加旋转。
 *	
 *	7.pid调试心得：pitch和yaw都要特别硬，可以加很大的积分ki（注释掉反向积分清零的操作会稳定一点）
 */
void GIMBAL_BuffCtrl(void)
{
	/* 按下WSADQEV(任意方向键)则退出打符模式 */
	if((IF_KEY_PRESSED_W || IF_KEY_PRESSED_S || IF_KEY_PRESSED_A || IF_KEY_PRESSED_D
		|| IF_KEY_PRESSED_Q || IF_KEY_PRESSED_E || IF_KEY_PRESSED_V)&&(!IF_KEY_PRESSED_CTRL)) 
	{
		Gimbal.State.mode = GIMBAL_MODE_NORMAL;	// 进入正常模式
		Gimbal.State.pid_mode = GYRO;		// 进入陀螺仪模式
		
		Gimbal_PID[GYRO][YAW_205].Angle.target = Gimbal_PID[GYRO][YAW_205].Angle.feedback;
		Gimbal_PID[GYRO][PITCH_206].Angle.target = Gimbal_PID[GYRO][PITCH_206].Angle.feedback;	// GIMBAL_GYRO_PITCH_MID_ANGLE

		/* 拨盘模式调整 */
		REVOLVER_SetAction(SHOOT_NORMAL);		// 变成常规控制模式
		/* 视觉模式调整 */
		VISION_SetMode(VISION_MODE_MANUAL);	// 手动模式
		/* 底盘模式调整 */
		CHASSIS_SetMode(CHAS_MODE_NORMAL);
		return;
	}
	
	/* 视觉数据可用 && 键盘模式下 */
	if( VISION_IsDataValid() ) 
	{
		/*----期望修改----*/
		GIMBAL_BUFF_PidCalc(Gimbal_PID, &Gimbal);	
	}
}

/**
 *	@brief	云台自动补弹
 *	@note		
 */
void GIMBAL_ReloadBulletCtrl(void)
{
//	/* 按下WSADQEV(任意方向键)则退出自动补弹模式 */
//	if((IF_KEY_PRESSED_W || IF_KEY_PRESSED_S || IF_KEY_PRESSED_A || IF_KEY_PRESSED_D
//		|| IF_KEY_PRESSED_Q || IF_KEY_PRESSED_E || IF_KEY_PRESSED_V))
//	{
//		GIMBAL_SetMode(GIMBAL_MODE_NORMAL);
//		Gimbal.State.pid_mode = GYRO;
//		GIMBAL_keyMech_To_keyGyro();
//		return;
//	}
//	
	/* 头定在中间 */
	Gimbal_PID[Gimbal.State.pid_mode][PITCH_206].Angle.target = GIMBAL_MECH_PITCH_MID_ANGLE;
}

/* #任务层# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	云台信息获取函数
 */
void GIMBAL_GetInfo(void)
{
	// 获取系统信息
	GIMBAL_GetSysInfo(&System, &Gimbal);
	// 获取裁判系统信息
	GIMBAL_GetJudgeInfo(&Judge, &Gimbal);
	// 获取IMU信息
	GIMBAL_GetImuInfo(Gimbal_PID);	
	// 计算自瞄预测信息
	GIMBAL_CalPredictInfo();
}

/**
 *	@brief	pid控制器最终输出
 */
uint8_t test_yaw_pid = 0;
uint8_t test_pitch_pid = 0;
float   test_yaw_speed_max_target = 8000;
float   test_pitch_speed_max_target = 4000;
void GIMBAL_PidCtrlTask(void)
{
	/* YAW 角度环 */
	GIMBAL_Angle_PidCalc(Gimbal_PID[Gimbal.State.pid_mode], YAW_205);
	/* YAW 速度环 */
	if(test_yaw_pid == 0) 
	{
		Gimbal_PID[Gimbal.State.pid_mode][YAW_205].Speed.target = Gimbal_PID[Gimbal.State.pid_mode][YAW_205].Angle.out;
	} 
	else 
	{
		Gimbal_PID[Gimbal.State.pid_mode][YAW_205].Speed.target = (RC_Ctl_Info.rc.ch0 - 1024)/660.f * test_yaw_speed_max_target;
	}
	GIMBAL_Speed_PidCalc(Gimbal_PID[Gimbal.State.pid_mode], YAW_205);
	
	/* PITCH 角度环 */
	GIMBAL_Angle_PidCalc(Gimbal_PID[Gimbal.State.pid_mode], PITCH_206);
	/* PITCH 速度环 */
	if(test_pitch_pid == 0) 
	{
		Gimbal_PID[Gimbal.State.pid_mode][PITCH_206].Speed.target = Gimbal_PID[Gimbal.State.pid_mode][PITCH_206].Angle.out;
	} 
	else 
	{
		Gimbal_PID[Gimbal.State.pid_mode][PITCH_206].Speed.target = -RC_RIGH_CH_UD_VALUE/660.f * test_pitch_speed_max_target;
	}
	GIMBAL_Speed_PidCalc(Gimbal_PID[Gimbal.State.pid_mode], PITCH_206);
	
	GIMBAL_PidOut(Gimbal_PID[Gimbal.State.pid_mode]);	
}

/**
 *	@brief	遥控控制云台
 */
void GIMBAL_RcCtrlTask(void)
{
	REMOTE_SetGimbalAngle();	
	REMOTE_SetTopGyro();
}

/**
 *	@brief	键盘控制云台
 */
void GIMBAL_KeyCtrlTask(void)
{
	switch(Gimbal.State.mode)
	{
		case GIMBAL_MODE_NORMAL:	
			GIMBAL_NormalCtrl();
			break;
		case GIMBAL_MODE_AUTO:
			GIMBAL_AutoCtrl();
			break;
		case GIMBAL_MODE_BIG_BUFF:
		case GIMBAL_MODE_SMALL_BUFF:
			GIMBAL_BuffCtrl();
			break;
		case GIMBAL_MODE_RELOAD_BULLET:
			GIMBAL_ReloadBulletCtrl();
			break;
		default:
			break;
	}
}

/**
 *	@brief	云台失控保护
 */
void GIMBAL_SelfProtect(void)
{	
	GIMBAL_Stop(Gimbal_PID[MECH]);
	GIMBAL_Stop(Gimbal_PID[GYRO]);
	GIMBAL_PidParamsInit(Gimbal_PID[MECH], GIMBAL_MOTOR_COUNT);
	GIMBAL_PidParamsInit(Gimbal_PID[GYRO], GIMBAL_MOTOR_COUNT);	
	
	GIMBAL_CalPredictInfo();
}

/**
 *	@brief	云台控制
 */
void GIMBAL_Ctrl(void)
{
	/*----信息读入----*/
	GIMBAL_GetInfo();
	/*----期望修改----*/
	if(BM_IfSet(BitMask.System.BM_Reset, BM_RESET_GIMBAL)) {	// 复位状态
		GIMBAL_Reset(); // 云台复位
	} 
	else {
		if(Gimbal.State.remote_mode == RC) {
			GIMBAL_RcCtrlTask();
		} 
		else if(Gimbal.State.remote_mode == KEY) {
			GIMBAL_KeyCtrlTask();
		}
	}

	/* 根据云台模式切换PID参数 */
	GIMBAL_PidParamsSwitch(Gimbal_PID, &Gimbal);

	/*----最终输出----*/
	GIMBAL_PidCtrlTask();
}
