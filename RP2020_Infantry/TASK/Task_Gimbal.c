/**
 * @file        Task_Gimbal.c
 * @author      MaWeiming
 * @Version     V1.0
 * @date        1-October-2019
 * @brief       This file includes the Gimbal(云台) external functions 
 *
 * @Verison			V1.1 (1-October-2019)
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

/* Private variables ---------------------------------------------------------*/
/* 卡尔曼滤波器 */
extKalman_t Gimbal_kalmanError[GIMBAL_MODE_COUNT][GIMBAL_MOTOR_COUNT];
extKalman_t Mpu_kalmanError[2];
extKalman_t Gimbal_Auto_kalmanError[GIMBAL_MOTOR_COUNT];

extKalman_t kalman_visionYaw,kalman_cloudYaw,kalman_targetYaw,kalman_speedYaw;

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
			.Angle.target = GIMBAL_MECH_YAW_ANGLE_MID_LIMIT,	// 归中
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
			.Angle.target = GIMBAL_MECH_PITCH_ANGLE_MID_LIMIT,
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
			.Angle.target = GIMBAL_MECH_PITCH_ANGLE_MID_LIMIT,
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
	.data = {0}
};

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* API functions -------------------------------------------------------------*/
/**
 *	@brief	云台电机PID参数初始化
 */
void GIMBAL_pidParamsInit(Gimbal_PID_t *pid, uint8_t motor_cnt)
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
			pid[i].Angle.target = GIMBAL_MECH_YAW_ANGLE_MID_LIMIT;
		} else if(i == PITCH_206) {
			pid[i].Angle.target = GIMBAL_MECH_PITCH_ANGLE_MID_LIMIT;
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

void GIMBAL_pidParamsSwitch(Gimbal_PID_t pid[GIMBAL_MODE_COUNT][GIMBAL_MOTOR_COUNT], Gimbal_Info_t *gimbal)
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
			if(VISION_getFlagStatus(VISION_FLAG_LOCK_TARGET) == true) // 锁定目标
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
void GIMBAL_stop(Gimbal_PID_t *pid)
{
	static int16_t pid_out[4] = {0, 0, 0, 0};
	
	/* 内环速度环最终输出 */
	pid[YAW_205].Speed.out = 0;
	pid[YAW_205].Out = 0;
	pid[PITCH_206].Speed.out = 0;
	pid[PITCH_206].Out = 0;
	
	CAN1_send(0x1FF, pid_out);	// 云台CAN总线标准标识符
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

void GIMBAL_Speed_pidCalculate(Gimbal_PID_t *pid, Gimbal_Motor_Names_t MOTORx)
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
void GIMBAL_Angle_pidCalculate(Gimbal_PID_t *pid, Gimbal_Motor_Names_t MOTORx)
{	
	js_gimbal_yaw_angle_feedback = pid[YAW_205].Angle.feedback;
	js_gimbal_yaw_angle_target = pid[YAW_205].Angle.target;
	js_gimbal_pitch_angle_feedback = pid[PITCH_206].Angle.feedback;
	js_gimbal_pitch_angle_target = pid[PITCH_206].Angle.target;	
	
	pid[MOTORx].Angle.erro = pid[MOTORx].Angle.target - pid[MOTORx].Angle.feedback;
	js_gimbal_yaw_angle_erro = pid[YAW_205].Angle.erro;
	js_gimbal_pitch_angle_erro = pid[PITCH_206].Angle.erro;
	
	if((Flag.Gimbal.FLAG_pidMode == MECH) && (MOTORx == YAW_205)) {	// 计算有效误差
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
	
//	if((Flag.Gimbal.FLAG_pidMode == GYRO) && (MOTORx == YAW_205)) {	// 计算有效误差
//		if(pid[MOTORx].Angle.erro >= 180.f*GIMBAL_GYRO_ANGLE_ZOOM_INDEX) {
//			pid[MOTORx].Angle.erro = -(360.f*GIMBAL_GYRO_ANGLE_ZOOM_INDEX - pid[MOTORx].Angle.erro);
//		} else if(pid[MOTORx].Angle.erro <= -180.f*GIMBAL_GYRO_ANGLE_ZOOM_INDEX) {
//			pid[MOTORx].Angle.erro = +(360.f*GIMBAL_GYRO_ANGLE_ZOOM_INDEX + pid[MOTORx].Angle.erro);
//		}
//	}
	
	/* 对误差进行卡尔曼滤波，消除低频等幅抖动 */
	if(Gimbal.State.mode == GIMBAL_MODE_NORMAL) {	// 正常模式下
		pid[MOTORx].Angle.erro = KalmanFilter(&Gimbal_kalmanError[Flag.Gimbal.FLAG_pidMode][MOTORx], pid[MOTORx].Angle.erro);
	}
	
	if(Gimbal.State.mode == GIMBAL_MODE_AUTO) {
		pid[MOTORx].Angle.erro = KalmanFilter(&Gimbal_kalmanError[Flag.Gimbal.FLAG_pidMode][MOTORx], pid[MOTORx].Angle.erro);
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
void GIMBAL_pidOut(Gimbal_PID_t *pid)
{
	int16_t pidOut[4] = {0, 0, 0, 0};
	
	/* CAN发送电压值 */
	pidOut[YAW_205] = (int16_t)pid[YAW_205].Out;		// 0x205
	pidOut[PITCH_206] = (int16_t)pid[PITCH_206].Out;	// 0x206
	
	CAN1_send(0x1FF, pidOut);
}

/**
 *	@brief	云台卡尔曼滤波器初始化
 */
float k_Auto_Yaw_R = 20;
float k_Auto_Pitch_R = 10;
float k_Zx_Yaw_Q = 1;
float k_Zx_Yaw_R = 1000;
void GIMBAL_kalmanCreate(void)
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
void GIMBAL_setMode(Gimbal_Mode_t mode)
{
	Gimbal.State.mode = mode;
}

/**
 *	@brief	根据底盘控制逻辑反馈机械中值
 */
float GIMBAL_getMiddleAngle(void)
{
	if(CHASSIS_getLogic() == CHAS_LOGIC_NORMAL) {
		return GIMBAL_TOP_YAW_ANGLE_MID_LIMIT;
	} 
	else if(CHASSIS_getLogic() == CHAS_LOGIC_REVERT) {
		return GIMBAL_REVERT_YAW_ANGLE_MID_LIMIT;
	}
	return GIMBAL_TOP_YAW_ANGLE_MID_LIMIT;
}

/**
 *	@brief	云台是否处于机械模式
 */
bool GIMBAL_ifMechMode(void)
{
	return (Flag.Gimbal.FLAG_pidMode == MECH);
}

/**
 *	@brief	云台是否处于陀螺仪模式
 */
bool GIMBAL_ifGyroMode(void)
{
	return (Flag.Gimbal.FLAG_pidMode == GYRO);
}

/**
 *	@brief	返回云台当前的模式
 */
Gimbal_Mode_t GIMBAL_getGimbalMode(void)
{
	return Gimbal.State.mode;
}

/**
 *	@brief	判断云台是否处于常规模式
 *	@return true - 常规模式
 *					false - 非常规模式
 */
bool GIMBAL_ifNormalMode(void)
{
	if(Gimbal.State.mode == GIMBAL_MODE_NORMAL) {
		return true;
	} else {
		return false;
	}
}

/**
 *	@brief	判断云台是否开启小陀螺
 *	@return true - 小陀螺模式
 *					false - 小陀螺模式
 */
bool GIMBAL_ifTopGyroOpen(void)
{
	if((GIMBAL_ifGyroMode() == true )&& 
		(Gimbal.State.FLAG_topGyroOpen == true)) {
		return true;
	} else {
		return false;
	}
}

/**
 *	@brief	判断云台是否处于自瞄模式
 *	@return true - 自瞄模式
 *					false - 非自瞄模式
 */
bool GIMBAL_ifAutoMode(void)
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
 *					false - 非打符模式
 */
bool GIMBAL_ifBuffMode(void)
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
 *					false - 没有瞄准哨兵
 */
bool GIMBAL_ifAimSentry(void)
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
 *					false - 没有正在补弹
 */
bool GIMBAL_ifReloadBullet(void)
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
 *					false - 跟踪未到位禁止打弹
 */
float debug_pix_yaw = 0;
float debug_pix_pitch = 0;
float debug_pix_yaw_ready = 35;
float debug_pix_pitch_ready = 35;
bool GIMBAL_BUFF_chaseReady(void)
{
	bool res = true;
	debug_pix_yaw = Gimbal.Buff.Yaw.erro;
	debug_pix_pitch = Gimbal.Buff.Pitch.erro;
	
	if((abs(debug_pix_yaw) < debug_pix_yaw_ready) && (VISION_getFlagStatus(VISION_FLAG_LOCK_BUFF) == true)) {
		res &= true;
	} else {
		res &= false;
	}
	
	if((abs(debug_pix_pitch) < debug_pix_pitch_ready) && (VISION_getFlagStatus(VISION_FLAG_LOCK_BUFF) == true)) {
		res &= true;
	} else {
		res &= false;
	}
	return res;
}

/**
 *	@brief	是否进入开火预测
 *	@return true - 已开启开火预测
 *					false - 未开启开火预测
 */
bool GIMBAL_ifAutoMobiPre(void)
{
	if(GIMBAL_ifAutoMode() == true) {
		return Mobi_Pre_Yaw;
	} else {	// 未开启自瞄不可能有预测
		return false;
	}
}

/**
 *	@brief	预测是否跟踪到位
 *	@return true - 跟踪到位可以打弹
 *					false - 跟踪未到位禁止打弹
 */
bool GIMBAL_AUTO_chaseReady(void)
{
	return Mobi_Pre_Yaw_Fire;
}

/**
 *	@brief	反馈底盘偏离设定的机械归中值的机械角度(0~8192)
 *	@return 反馈值为 >= 0 的机械角度
 *	@note	规定：以机械中值为轴，机械角度增加的方向为正方向
 */
float GIMBAL_getTopGyroAngleOffset(void)
{
	float delta_angle, feedback;

	feedback = g_Gimbal_Motor_Info[YAW_205].angle;
	

	
	delta_angle = feedback - GIMBAL_TOP_YAW_ANGLE_MID_LIMIT;

	
	
	return delta_angle;
}

/**
 *	@brief	云台信息获取函数
 */
void GIMBAL_getInfo(void)
{
	GIMBAL_calPredictInfo();
	
	if(CHASSIS_ifTopGyroOpen() == true) {
		Gimbal.State.FLAG_topGyroOpen = true;
	} else {
		Gimbal.State.FLAG_topGyroOpen = false;
	}
	
//	/* 弹仓打开之后 */
//	if(MAGZINE_ifOpen() == true) {
//		GIMBAL_setMode(GIMBAL_MODE_RELOAD_BULLET);
//	} 
//	/* 关闭弹仓 */
//	else {
//		if(GIMBAL_ifReloadBullet() == true) {
//			GIMBAL_setMode(GIMBAL_MODE_NORMAL);
//		}
//	}
	
}

/* #应用层# ---------------------------------------------------------------------------------------------------------------------------------------*/
/* #遥控# -----------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	根据遥控值设置云台YAW位置环的期望值(累加值)
 *	@note	
 */
void REMOTE_setGimbalAngle(void)
{
	float targetAngle;
	/* 机械模式 */
	if(GIMBAL_ifMechMode() == true) {	
		/* Yaw */
		/* 云台跟随底盘运动 */
		targetAngle = GIMBAL_getMiddleAngle();
		Gimbal_PID[MECH][YAW_205].Angle.target = targetAngle;
		/* Pitch */
		targetAngle = RC_RIGH_CH_UD_VALUE * RC_GIMBAL_MECH_PITCH_SENSITIVY; // Pitch机械角为绝对角度
		Gimbal_PID[MECH][PITCH_206].Angle.target = constrain(Gimbal_PID[MECH][PITCH_206].Angle.target - targetAngle, // 抬头机械角度减小
															 GIMBAL_MECH_PITCH_ANGLE_UP_LIMIT, 
															 GIMBAL_MECH_PITCH_ANGLE_DOWN_LIMIT);	// 设置Pitch轴(绝对值)
	} 
	/* 陀螺仪模式 */
	else if(GIMBAL_ifGyroMode() == true) {	
		/* Yaw */
		targetAngle = RC_RIGH_CH_LR_VALUE * RC_GIMBAL_GYRO_YAW_SENSITIVY * (YAW_DIR);
		Gimbal_PID[GYRO][YAW_205].Angle.target += targetAngle;
		
		/* Pitch */		
		targetAngle = RC_RIGH_CH_UD_VALUE * RC_GIMBAL_GYRO_PITCH_SENSITIVY;
		Gimbal_PID[GYRO][PITCH_206].Angle.target = constrain(Gimbal_PID[GYRO][PITCH_206].Angle.target - targetAngle, // 抬头Pitch角度减小
															 GIMBAL_GYRO_PITCH_ANGLE_UP_LIMIT, 
															 GIMBAL_GYRO_PITCH_ANGLE_DOWN_LIMIT);	// 设置Pitch轴(绝对值)		
	}
}

///**
// *	@brief	根据遥控值设置云台的小陀螺模式
// *	@note	
// */
//void REMOTE_setTopGyro(void)
//{
//	/* 处于陀螺仪模式可以开启 */
//	if(GIMBAL_ifGyroMode() == true) {
//		/* 拨轮向上开启 */
//		if((RC_THUMB_WHEEL_VALUE < -650)) {
//			if(Gimbal.State.FLAG_topGyroOpen == false) {
//				Gimbal.State.FLAG_topGyroOpen = true;
//			}
//		}		
//		/* 拨轮向下关闭 */
//		else if(RC_THUMB_WHEEL_VALUE > 650) {
//			Gimbal.State.FLAG_topGyroOpen = false;
//		}
//	}
//}

/* #键盘鼠标# -------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	根据按键值设置云台YAW位置环的期望值(累加值)
 *	@note	
 *			鼠标右下为正
 */
float js_targetYaw;
float js_targetPitch;
void KEY_setGimbalAngle(void)
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
														 GIMBAL_GYRO_PITCH_ANGLE_UP_LIMIT, 
														 GIMBAL_GYRO_PITCH_ANGLE_DOWN_LIMIT);	// 设置Pitch轴(绝对值)	
}

/**
 *	@brief	根据按键值设置云台转头
 */
void KEY_setGimbalTurn(void)
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
			Flag.Chassis.FLAG_goHome = true;
			
			if(keyVLockFlag == false) {
				if(IF_KEY_PRESSED_A) {
					// 同时按下AV 或 先按A再按V
					targetAngle = -180*GIMBAL_GYRO_ANGLE_ZOOM_INDEX * (YAW_DIR);
					Gimbal_PID[GYRO][YAW_205].AngleRampTarget += targetAngle;
				} else {	// 只按V，没按A
					targetAngle = +180*GIMBAL_GYRO_ANGLE_ZOOM_INDEX * (YAW_DIR);
					Gimbal_PID[GYRO][YAW_205].AngleRampTarget += targetAngle;
				}
				
//				/* 修改底盘逻辑 */
//				CHASSIS_logicRevert();
			}
			keyVLockFlag = true;
		}
	} else {
		keyVLockFlag = false;
	}
	
	/* 斜坡函数给累加期望，防止突然增加很大的期望值 */
	Gimbal_PID[GYRO][YAW_205].AngleRampFeedback = RAMP_float(Gimbal_PID[GYRO][YAW_205].AngleRampTarget, 
																														Gimbal_PID[GYRO][YAW_205].AngleRampFeedback, 
																														GIMBAL_GYRO_ANGLE_ZOOM_INDEX/1.5f);
	if(Gimbal_PID[GYRO][YAW_205].AngleRampFeedback < Gimbal_PID[GYRO][YAW_205].AngleRampTarget) // 正向累加
	{
		Gimbal_PID[GYRO][YAW_205].Angle.target = GIMBAL_GYRO_yawTargetBoundaryProcess(&Gimbal_PID[GYRO][YAW_205], 
																																									GIMBAL_GYRO_ANGLE_ZOOM_INDEX/1.5f);
	} 
	else if(Gimbal_PID[GYRO][YAW_205].AngleRampFeedback > Gimbal_PID[GYRO][YAW_205].AngleRampTarget) // 反向累加
	{
		Gimbal_PID[GYRO][YAW_205].Angle.target = GIMBAL_GYRO_yawTargetBoundaryProcess(&Gimbal_PID[GYRO][YAW_205], 
																																									-GIMBAL_GYRO_ANGLE_ZOOM_INDEX/1.5f);
	} 
	else // 缓冲池清零
	{
		/* 扭头到位标志位清零 */
		Flag.Chassis.FLAG_goHome = false;
		
		Gimbal_PID[GYRO][YAW_205].AngleRampTarget = 0;
		Gimbal_PID[GYRO][YAW_205].AngleRampFeedback = 0;
	}
}

/**
 *	@brief	根据按键值设置云台快速抬头
 */
void KEY_setQuickPickUp(void)
{
	float targetAngle;
	static uint8_t keyGLockFlag = false;
	
	if(IF_KEY_PRESSED_G) {
		if(keyGLockFlag == false) {
			targetAngle = 15.f/360*8192;	// 快速抬头15°
			Gimbal_PID[GYRO][PITCH_206].Angle.target = constrain(Gimbal_PID[GYRO][PITCH_206].Angle.target - targetAngle, // 抬头Pitch角度减小
															 GIMBAL_GYRO_PITCH_ANGLE_UP_LIMIT, 
															 GIMBAL_GYRO_PITCH_ANGLE_DOWN_LIMIT);	// 设置Pitch轴(绝对值)
		}
		keyGLockFlag = true;
	} else {
		keyGLockFlag = false;
	}
}

///**
// *	@brief	根据按键值设置云台小陀螺模式
// */
//void KEY_setTopGyro(void)
//{
//	static uint8_t keyFLockFlag = false;
//	
//	if(IF_KEY_PRESSED_F) {
//		if(keyFLockFlag == false) {
//			if(GIMBAL_ifGyroMode() == true) {
//				if(Gimbal.State.FLAG_topGyroOpen == false) {
//					Gimbal.State.FLAG_topGyroOpen = true;
//				}
//				else {
//					Gimbal.State.FLAG_topGyroOpen = false;
//				}
//			}
//		}
//		keyFLockFlag = true;
//	} else {
//		keyFLockFlag = false;
//	}	
//}

/**
 *	@brief	鼠标右键进入自瞄模式
 *			松开右键退出自瞄模式
 */
uint8_t test_auto_pid = 0;
static uint8_t mouseRLockFlag = false;	// 鼠标右键锁定
static uint8_t rcSw1LockFlag = false;
static uint8_t keyCtrlLockFlag = false;
static portTickType  keyCurrentTime = 0;	
static uint32_t keyCtrlFLockTime = 0;
static uint32_t keyCtrlVLockTime = 0;
void KEY_setGimbalMode(RC_Ctl_t *remoteInfo)
{
	keyCurrentTime = xTaskGetTickCount();
	
	if(test_auto_pid == 0) {
		/* 鼠标右键 */
		if(IF_MOUSE_PRESSED_RIGH) {
			if(mouseRLockFlag == false && GIMBAL_ifBuffMode() == false) {
				/* 云台模式调整 */
				Gimbal.State.mode = GIMBAL_MODE_AUTO;
				Gimbal.Auto.FLAG_first_into_auto = true;
				/* 视觉模式调整 */
				VISION_setMode(VISION_MODE_AUTO);
				/* 拨盘模式调整 */
				REVOLVER_setAction(SHOOT_AUTO);		// 变成常规控制模式
			}
			mouseRLockFlag = true;
		} else {
			if(GIMBAL_ifAutoMode() == true) { // 退出自瞄模式
				/* 云台模式调整 */
				Gimbal.State.mode = GIMBAL_MODE_NORMAL;
				Gimbal_PID[GYRO][YAW_205].Angle.target = Gimbal_PID[GYRO][YAW_205].Angle.feedback;
				Gimbal_PID[GYRO][PITCH_206].Angle.target = Gimbal_PID[GYRO][PITCH_206].Angle.feedback;
				/* 视觉模式调整 */
				VISION_setMode(VISION_MODE_MANUAL);	// 手动模式
				/* 拨盘模式调整 */
				REVOLVER_setAction(SHOOT_NORMAL);		// 变成常规控制模式
			}
			mouseRLockFlag = false;
		}
	} else {
		if(IF_RC_SW1_MID) {
			if(rcSw1LockFlag == false) {
				/* 云台模式调整 */
				Gimbal.State.mode = GIMBAL_MODE_AUTO;
				Gimbal.Auto.FLAG_first_into_auto = true;
				/* 视觉模式调整 */
				VISION_setMode(VISION_MODE_AUTO);
			}
			rcSw1LockFlag = true;
		} else {
			if(GIMBAL_ifAutoMode() == true) { // 退出自瞄模式
				/* 云台模式调整 */
				Gimbal.State.mode = GIMBAL_MODE_NORMAL;
				Gimbal_PID[GYRO][YAW_205].Angle.target = Gimbal_PID[GYRO][YAW_205].Angle.feedback;
				Gimbal_PID[GYRO][PITCH_206].Angle.target = Gimbal_PID[GYRO][PITCH_206].Angle.feedback;
				/* 视觉模式调整 */
				VISION_setMode(VISION_MODE_MANUAL);	// 手动模式
			}
			rcSw1LockFlag = false;
		}
	}
	
	/* Ctrl+V组合键 */
	if(IF_KEY_PRESSED_CTRL) {
		if(keyCtrlLockFlag == false) {
			/* 云台模式调整 */
			Gimbal.State.mode = GIMBAL_MODE_NORMAL;
			Flag.Gimbal.FLAG_pidMode = MECH;	// 强制进入机械模式
			GIMBAL_keyGyro_To_keyMech();
			/* 视觉模式调整 */
			VISION_setMode(VISION_MODE_MANUAL);	// 手动模式
			/* 拨盘模式调整 */
			REVOLVER_setAction(SHOOT_NORMAL);	
		}
		if(keyCurrentTime > keyCtrlVLockTime) {	
			keyCtrlVLockTime = keyCurrentTime + TIME_STAMP_400MS;
			if(IF_KEY_PRESSED_V) {	// Ctrl+V
				if(Gimbal.State.mode != GIMBAL_MODE_SMALL_BUFF) {
					/* 云台模式调整 */
					Gimbal.State.mode = GIMBAL_MODE_SMALL_BUFF;
					Gimbal.Buff.FLAG_first_into_buff = true;
					Flag.Gimbal.FLAG_pidMode = GYRO;	// 强制进入陀螺仪模式
					GIMBAL_keyMech_To_keyGyro();
					/* 视觉模式调整 */
					VISION_setMode(VISION_MODE_SMALL_BUFF);	// 击打小符
					/* 底盘模式调整 */
					CHASSIS_setMode(CHAS_MODE_BUFF);
				}
			}
		}
		if(keyCurrentTime > keyCtrlFLockTime) {
			keyCtrlFLockTime = keyCurrentTime + TIME_STAMP_400MS;
			if(IF_KEY_PRESSED_F) {	// Ctrl+F
				if(Gimbal.State.mode != GIMBAL_MODE_BIG_BUFF) {
					/* 云台模式调整 */
					Gimbal.State.mode = GIMBAL_MODE_BIG_BUFF;
					Gimbal.Buff.FLAG_first_into_buff = true;
					Flag.Gimbal.FLAG_pidMode = GYRO;	// 强制进入陀螺仪模式
					GIMBAL_keyMech_To_keyGyro();
					/* 视觉模式调整 */
					VISION_setMode(VISION_MODE_BIG_BUFF);	// 击打大符
					/* 底盘模式调整 */
					CHASSIS_setMode(CHAS_MODE_BUFF);
				}
			}
		}
		keyCtrlLockFlag = true;
	} else {
		if(keyCtrlLockFlag == true && Gimbal.State.mode == GIMBAL_MODE_NORMAL) {
			/* 云台模式调整 */
			Gimbal.State.mode = GIMBAL_MODE_NORMAL;
			Flag.Gimbal.FLAG_pidMode = GYRO;	// 强制进入陀螺仪模式
			GIMBAL_keyMech_To_keyGyro();
			/* 视觉模式调整 */
			VISION_setMode(VISION_MODE_MANUAL);	// 手动模式
			/* 底盘模式调整 */
			CHASSIS_setMode(CHAS_MODE_NORMAL);
		}
		keyCtrlLockFlag = false;
	}
}

/* #云台# -----------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	云台电机初始化
 *	@note		
 */
void GIMBAL_init(void)
{
	GIMBAL_kalmanCreate();	// 创建卡尔曼滤波器
	GIMBAL_GYRO_calAverageOffset(Mpu_Info);	// 陀螺仪角速度误差补偿值计算	
}

/**
 *	@brief	云台电机复位
 *	@note	# 先用机械模式归中
 *			# 具备5s的超时退出机制，防止云台没有复位到位一直卡死	
 */
void GIMBAL_reset(void)
{
	static uint8_t reset_start = 1;
	static uint32_t resetTime = 0;
	static portTickType tickTime_prev = 0;
	static portTickType tickTime_now = 0;
	
	int8_t reset_dir;
	float delta_angle;
	float target_angle;
	
	target_angle = GIMBAL_getMiddleAngle();
	
	tickTime_now = xTaskGetTickCount();
	if(tickTime_now  - tickTime_prev > TIME_STAMP_100MS) {	// 保证不断电情况下，下次可用
		Flag.Gimbal.FLAG_resetOK = false;
		Cnt.Gimbal.CNT_resetOK = 0;
		Flag.Gimbal.FLAG_angleRecordStart = 0;	// 重新记录
		//..feedback值是否需要清零?
	}
	tickTime_prev = tickTime_now;
	
	if(Flag.Gimbal.FLAG_angleRecordStart == 0) {	// 记录上电时云台机械角度和陀螺仪反馈值
		Gimbal_PID[MECH][YAW_205].Angle.target = Gimbal_PID[MECH][YAW_205].Angle.feedback;
		Gimbal_PID[MECH][PITCH_206].Angle.target = Gimbal_PID[MECH][PITCH_206].Angle.feedback;
		Flag.Gimbal.FLAG_angleRecordStart = 1;
	}	
	
	if(Flag.Gimbal.FLAG_resetOK == false) {	// 云台复位未完成
		resetTime++;	// 复位计时
		if(reset_start) {	// 刚进入复位
			reset_start = 0;
			delta_angle = target_angle - g_Gimbal_Motor_Info[YAW_205].angle;
			/* 就近移动算法 */
			/* 中值在[4096,8191] */
			if(target_angle > 4095) {	
				if(delta_angle >= 0 && delta_angle < 4096) {
					reset_dir = +1;
					Gimbal_PID[MECH][YAW_205].AngleRampTarget = reset_dir*delta_angle;
				} 
				else if(delta_angle >= 4096) {
					reset_dir = -1;
					Gimbal_PID[MECH][YAW_205].AngleRampTarget = reset_dir*(8192 - delta_angle);
				} 
				else if(delta_angle < 0){	// 误差为0
					reset_dir = -1;
					Gimbal_PID[MECH][YAW_205].AngleRampTarget = delta_angle;
				}
			} 
			/* 中值在[0,4095] */
//			else {
//				if(delta_angle > -4096) 
//					reset_dir = 1;
//				else
//					reset_dir = -1;
//			}			
		}
		
		if(Flag.Gimbal.FLAG_pidStart == 1) {
			Flag.Gimbal.FLAG_pidMode = MECH;

			/* 平缓地让云台移动到中间，防止上电狂甩 */
			/* 斜坡函数给累加期望，防止突然增加很大的期望值 */
			if(Gimbal_PID[MECH][YAW_205].AngleRampFeedback < Gimbal_PID[MECH][YAW_205].AngleRampTarget) // 正向累加
			{
				Gimbal_PID[MECH][YAW_205].Angle.target = GIMBAL_MECH_yawTargetBoundaryProcess(&Gimbal_PID[MECH][YAW_205], GIMBAL_RAMP_BEGIN_YAW);
			} 
			else if(Gimbal_PID[MECH][YAW_205].AngleRampFeedback > Gimbal_PID[MECH][YAW_205].AngleRampTarget) // 反向累加
			{
				Gimbal_PID[MECH][YAW_205].Angle.target = GIMBAL_MECH_yawTargetBoundaryProcess(&Gimbal_PID[MECH][YAW_205], -GIMBAL_RAMP_BEGIN_YAW);
			} 
			else // 缓冲池清零
			{
				Gimbal_PID[MECH][YAW_205].AngleRampTarget = 0;
				Gimbal_PID[MECH][YAW_205].AngleRampFeedback = 0;
			}
			Gimbal_PID[MECH][YAW_205].AngleRampFeedback = RAMP_float(Gimbal_PID[MECH][YAW_205].AngleRampTarget, Gimbal_PID[MECH][YAW_205].AngleRampFeedback, GIMBAL_RAMP_BEGIN_YAW);
			
			/* 平缓地让云台移动到中间，防止上电狂甩 */
			Gimbal_PID[MECH][PITCH_206].Angle.target = RAMP_float(GIMBAL_MECH_PITCH_ANGLE_MID_LIMIT, Gimbal_PID[MECH][PITCH_206].Angle.target, GIMBAL_RAMP_BEGIN_PITCH);
		}		
		
		/* 等待云台归中 */
		if(abs(Gimbal_PID[MECH][YAW_205].Angle.feedback - target_angle) <= 1 
			&& abs(Gimbal_PID[MECH][PITCH_206].Angle.feedback - GIMBAL_MECH_PITCH_ANGLE_MID_LIMIT) <= 1) {
			Cnt.Gimbal.CNT_resetOK++;
		}
			
		/* 复位成功或者超时强制退出(5s) */
		if(Cnt.Gimbal.CNT_resetOK > 250 || resetTime >= 2500) {	
			Cnt.Gimbal.CNT_resetOK = 0;
			Flag.Gimbal.FLAG_resetOK = true;// 云台复位成功
			resetTime = 0;					// 复位计时清零
		}		
	} else if(Flag.Gimbal.FLAG_resetOK == true) {	// 云台复位完成
		Flag.Gimbal.FLAG_resetOK = false;	// 清除状态标志位
		reset_start = 1;	// 等待下次调用复位云台函数
		BM_reset(BitMask.System.BM_reset, BM_RESET_GIMBAL);
		/* 模式切换(刚上电太快切成陀螺仪模式会因为反馈角度还未稳定造成云台甩头) */			
		if(RC_Ctl_Info.rc.s2 == RC_SW_UP) {
			Flag.Remote.FLAG_mode = KEY;
			Flag.Gimbal.FLAG_pidMode = GYRO;
			GIMBAL_rcMech_To_keyGyro();
		} else if(RC_Ctl_Info.rc.s2 == RC_SW_MID) {
			Flag.Remote.FLAG_mode = RC;
			Flag.Gimbal.FLAG_pidMode = MECH;					
		} else if(RC_Ctl_Info.rc.s2 == RC_SW_DOWN) {
			Flag.Remote.FLAG_mode = RC;
			Flag.Gimbal.FLAG_pidMode = GYRO;
			GIMBAL_rcMech_To_rcGyro();					
		}
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
void GIMBAL_IMU_recordFeedback(Gimbal_PID_t pid[GIMBAL_MODE_COUNT][GIMBAL_MOTOR_COUNT])
{	
	static float last_yaw = 0.f;
	float delta_yaw;
	
	/* 读取MPU6050传感器数据 */
	mpu_dmp_get_data( &Mpu_Info.roll, &Mpu_Info.pitch, &Mpu_Info.yaw);
	MPU_Get_Gyroscope( &Mpu_Info.ratePitch, &Mpu_Info.rateRoll, &Mpu_Info.rateYaw);		
	
	/* # Yaw # */
	yaw = Mpu_Info.yaw;
	delta_yaw = yaw - last_yaw;
	if(delta_yaw > +180.f) {
		delta_yaw = -360.f + delta_yaw;//(+360.f - delta_yaw);
	} else if(delta_yaw < -180.f) {
		delta_yaw = +360.f + delta_yaw;//(-360.f - delta_yaw);
	}
	last_yaw = yaw; 	
	
	/* 利用陀螺仪数据作扭头补偿 */
	if(GIMBAL_ifTopGyroOpen() == true) {
		Chassis_Z_PID.Angle.target = CHASSIS_MECH_yawTargetBoundaryProcess(&Chassis_Z_PID, 		
																	delta_yaw * 8192 / 360.f);
	}
	
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
 *	@brief	云台电机IMU计算误差补偿值
 */
void GIMBAL_GYRO_calAverageOffset(Mpu_Info_t mpuInfo)
{
	uint16_t i;
	for(i = 0; i < 50; i++) {
		delay_us(100);
		// 读取陀螺仪角度和角速度
		mpu_dmp_get_data( &Mpu_Info.roll, &Mpu_Info.pitch, &Mpu_Info.yaw);
		MPU_Get_Gyroscope( &Mpu_Info.ratePitch, &Mpu_Info.rateRoll, &Mpu_Info.rateYaw);		
	}
	for(i = 0; i < 500; i++) {
		delay_us(200);
		MPU_Get_Gyroscope( &Mpu_Info.ratePitch, &Mpu_Info.rateRoll, &Mpu_Info.rateYaw);
		Mpu_Info.ratePitchOffset += Mpu_Info.ratePitch;
		Mpu_Info.rateYawOffset   += Mpu_Info.rateYaw;
	}
	Mpu_Info.ratePitchOffset = (Mpu_Info.ratePitchOffset/500);
	Mpu_Info.rateYawOffset   = (Mpu_Info.rateYawOffset/500);
}

/**
 *	@brief	机械模式下云台YAW期望值(累加值)边界处理
 */
float GIMBAL_MECH_yawTargetBoundaryProcess(Gimbal_PID_t *pid, float delta_target)
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
float GIMBAL_GYRO_yawTargetBoundaryProcess(Gimbal_PID_t *pid, float delta_target)
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
 *	@brief	遥控机械模式 -> 遥控陀螺仪模式
 *	@note	异步切换
 */
void GIMBAL_rcMech_To_rcGyro(void)
{
	Gimbal_PID[GYRO][YAW_205].Angle.target = Gimbal_PID[GYRO][YAW_205].Angle.feedback;
	Gimbal_PID[GYRO][PITCH_206].Angle.target = Gimbal_PID[GYRO][PITCH_206].Angle.feedback;
	Gimbal_PID[MECH][YAW_205].Out = 0;
	Gimbal_PID[MECH][PITCH_206].Out = 0;	
	Gimbal.State.mode = GIMBAL_MODE_NORMAL;
}

/**
 *	@brief	遥控陀螺仪模式 -> 遥控机械模式
 *	@note	异步切换
 */
void GIMBAL_rcGyro_To_rcMech(void)
{
	Gimbal_PID[MECH][YAW_205].Angle.target = Gimbal_PID[MECH][YAW_205].Angle.feedback;
	Gimbal_PID[MECH][PITCH_206].Angle.target = Gimbal_PID[MECH][PITCH_206].Angle.feedback;
	Gimbal_PID[GYRO][YAW_205].Out = 0;
	Gimbal_PID[GYRO][PITCH_206].Out = 0;	
	Gimbal.State.FLAG_topGyroOpen = false;
	Gimbal.State.mode = GIMBAL_MODE_NORMAL;
}

/**
 *	@brief	遥控机械模式 -> 键盘模式
 *	@note	异步切换
 */
void GIMBAL_rcMech_To_keyGyro(void)
{
	Gimbal_PID[GYRO][YAW_205].Angle.target = Gimbal_PID[GYRO][YAW_205].Angle.feedback;
	Gimbal_PID[GYRO][PITCH_206].Angle.target = Gimbal_PID[GYRO][PITCH_206].Angle.feedback;
	Gimbal_PID[MECH][YAW_205].Out = 0;
	Gimbal_PID[MECH][PITCH_206].Out = 0;	
	Gimbal.State.mode = GIMBAL_MODE_NORMAL;
	//CHASSIS_setMode(CHAS_MODE_NORMAL);
}

/**
 *	@brief	键盘模式 -> 遥控机械模式
 *	@note	异步切换
 */
void GIMBAL_keyGyro_To_rcMech(void)
{
	Gimbal_PID[MECH][YAW_205].Angle.target = Gimbal_PID[MECH][YAW_205].Angle.feedback;
	Gimbal_PID[MECH][PITCH_206].Angle.target = Gimbal_PID[MECH][PITCH_206].Angle.feedback;
	Gimbal_PID[GYRO][YAW_205].Out = 0;
	Gimbal_PID[GYRO][PITCH_206].Out = 0;
	Gimbal.State.FLAG_topGyroOpen = false;
	//CHASSIS_setMode(CHAS_MODE_NORMAL);
	//Gimbal.State.mode = GIMBAL_MODE_NORMAL;
}

/**
 *	@brief	键盘陀螺仪模式 -> 键盘机械模式
 *	@note	异步切换
 */
void GIMBAL_keyGyro_To_keyMech(void)
{
	Gimbal_PID[MECH][YAW_205].Angle.target = GIMBAL_getMiddleAngle();
	Gimbal_PID[MECH][PITCH_206].Angle.target = Gimbal_PID[MECH][PITCH_206].Angle.feedback;
	Gimbal.State.FLAG_topGyroOpen = false;
	//Gimbal.State.mode = GIMBAL_MODE_NORMAL;
}

/**
 *	@brief	键盘机械模式 -> 键盘陀螺仪模式
 *	@note	异步切换
 */
void GIMBAL_keyMech_To_keyGyro(void)
{
	Gimbal_PID[GYRO][YAW_205].Angle.target = Gimbal_PID[GYRO][YAW_205].Angle.feedback;
	Gimbal_PID[GYRO][PITCH_206].Angle.target = Gimbal_PID[GYRO][PITCH_206].Angle.feedback;
	//Gimbal.State.mode = GIMBAL_MODE_NORMAL;
}

/**
 *	@brief	目标速度解算
 *	@note	利用两帧之间的角度差算出目标的速度
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
void GIMBAL_VISION_AUTO_pidCalculate(Gimbal_PID_t pid[GIMBAL_MODE_COUNT][GIMBAL_MOTOR_COUNT], Gimbal_Info_t *gimbal)
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
	if(VISION_getFlagStatus(VISION_FLAG_DATA_UPDATE) == true) {
		/* 清除数据更新标志位，防止未接收到新数据的情况下重复进入 */
		VISION_clearFlagStatus(VISION_FLAG_DATA_UPDATE);	

		if(VISION_getFlagStatus(VISION_FLAG_LOCK_TARGET) == true) {
			/* Yaw 误差计算 */
			gimbal->Auto.Yaw.erro = gimbal->Auto.Yaw.kp * VISION_GYRO_getYawFeedback();
			//gimbal->Auto.Yaw.erro = KalmanFilter(&Gimbal_Auto_kalmanError[YAW_205], gimbal->Auto.Yaw.erro);
			gimbal->Auto.Yaw.target = pid[GYRO][YAW_205].Angle.feedback + gimbal->Auto.Yaw.erro;
			/* Pitch 误差计算 */
			gimbal->Auto.Pitch.erro = (gimbal->Auto.Pitch.kp * VISION_MECH_getPitchFeedback()) - GIMBAL_AUTO_PITCH_COMPENSATION;
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
	pid[GYRO][YAW_205].Angle.target = RAMP_float(gimbal->Auto.Yaw.target, pid[GYRO][YAW_205].Angle.feedback, auto_yaw_ramp);
	pid[GYRO][PITCH_206].Angle.target = RAMP_float(gimbal->Auto.Pitch.target, pid[GYRO][PITCH_206].Angle.feedback, auto_pitch_ramp);	
//	pid[GYRO][YAW_205].Angle.target = pid[GYRO][YAW_205].Angle.feedback;
//	pid[GYRO][YAW_205].Angle.target = GIMBAL_GYRO_yawTargetBoundaryProcess(&pid[GYRO][YAW_205], gimbal->Auto.Yaw.erro);
//	pid[GYRO][PITCH_206].Angle.target = pid[GYRO][PITCH_206].Angle.feedback + gimbal->Auto.Pitch.erro;	
	
	gimbal->Auto.Time[PREV] = gimbal->Auto.Time[NOW];
}

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

void GIMBAL_AUTO_pidCalculate(Gimbal_PID_t pid[GIMBAL_MODE_COUNT][GIMBAL_MOTOR_COUNT], Gimbal_Info_t *gimbal)
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
	if(VISION_getFlagStatus(VISION_FLAG_DATA_UPDATE) == true) {
		/* 清除数据更新标志位，防止未接收到新数据的情况下重复进入 */
		VISION_clearFlagStatus(VISION_FLAG_DATA_UPDATE);	

		/* 计算自瞄帧率 */
		gimbal->Auto.Time[NOW] = xTaskGetTickCount();
		gimbal->Auto.Time[DELTA] = gimbal->Auto.Time[NOW] - gimbal->Auto.Time[PREV];
		gimbal->Auto.Time[PREV] = gimbal->Auto.Time[NOW];
		
		/* 识别到目标 */
		if(VISION_getFlagStatus(VISION_FLAG_LOCK_TARGET) == true) {
			/* Yaw 误差放大 */
			gimbal->Auto.Yaw.erro = gimbal->Auto.Yaw.kp * VISION_GYRO_getYawFeedback() + GIMBAL_AUTO_YAW_COMPENSATION;
			/* Pitch 误差放大 */
			gimbal->Auto.Pitch.erro = (gimbal->Auto.Pitch.kp * VISION_MECH_getPitchFeedback()) + GIMBAL_AUTO_PITCH_COMPENSATION;
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
	if(VISION_getFlagStatus(VISION_FLAG_LOCK_TARGET) == true) {	
		/* 更新二阶卡尔曼速度先验估计值 */
		yaw_angle_speed = speed_calculate(&yaw_angle_speed_struct, gimbal->Auto.Time[NOW], gimbal->Auto.Yaw.target);
		pitch_angle_speed = speed_calculate(&pitch_angle_speed_struct, gimbal->Auto.Time[NOW], gimbal->Auto.Pitch.target);

		/* 对角度和速度进行二阶卡尔曼滤波融合,0位置,1速度 */
		yaw_kf_result = kalman_filter_calc(&yaw_kalman_filter, gimbal->Auto.Yaw.target, yaw_angle_speed);
		pitch_kf_result = kalman_filter_calc(&pitch_kalman_filter, gimbal->Auto.Pitch.target, pitch_angle_speed);
		
		/* ↓ ZX预测速度 ↓ */
		nowCalAngle = yaw_kf_result[KF_ANGLE];	// gimbal->Auto.Yaw.target
		
		VISION_updateInfo(nowCalAngle, queue_num, &VisionQueue, &avrCalAngle, &none);
		
		targetYawSpeedRaw = (nowCalAngle - avrCalAngle)*ksc;
		
		if(abs(targetYawSpeedRaw) < deathzoom)
			targetYawSpeedRaw = 0.f;
		
		targetYawSpeedKF = KalmanFilter(&kalman_speedYaw, targetYawSpeedRaw);
		/* ↑ ZX预测速度 ↑ */
		last_speed = targetYawSpeedKF;
		last_distance = VISION_getDistance()/1000.f;	//换算成(m)
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
		
		VISION_updateInfo(nowCalAngle, queue_num, &VisionQueue, &avrCalAngle, &none);
		
		targetYawSpeedRaw = (nowCalAngle - avrCalAngle)*ksc;
		
		if(abs(targetYawSpeedRaw) < deathzoom)
			targetYawSpeedRaw = 0.f;
		
		targetYawSpeedKF = KalmanFilter(&kalman_speedYaw,0);
		/* ↑ ZX预测速度 ↑ */
	}
	
	js_yaw_speed = yaw_kf_result[KF_SPEED];
	js_yaw_angle = yaw_kf_result[KF_ANGLE];
	
	/*----加上距离补偿(假定是线性关系)----*/
	distance = VISION_getDistance()/1000.f;	//换算成(m)
	
	pitch_compensation = distance * kPitch;
	
	if(myDeathZoom(distance, distance_death) > 0.f) {
		dis_temp -= distance_death;
	} else {
		dis_temp = 0.f;
	}	
	
	/*----预测量计算----*/
	/* 识别到目标 */
	if(VISION_getFlagStatus(VISION_FLAG_LOCK_TARGET) == true) {	
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
					yaw_angle_predict = RAMP_float(yaw_predict_temp, yaw_angle_predict, auto_yaw_angle_predict_ramp);	// STEP_float();
				else 
					yaw_angle_predict = STEP_float(yaw_predict_temp, step, &step_cnt, step_death);
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
				
				Mobi_Pre_Yaw = true;	// 标记已开启预测
				mobpre_yaw_stop_delay = 0;	// 重置静止时的开火延迟
				
			}
			/* 未达到预测开启条件 */
			else {
				pid[GYRO][YAW_205].Angle.target = gimbal->Auto.Yaw.target;//RAMP_float(gimbal->Auto.Yaw.target, pid[GYRO][YAW_205].Angle.feedback, auto_yaw_ramp);
				
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
		/* 19预测 */
		else {
			/* 预测开启条件判断 */
			if(auto_predict_start_delay > auto_predict_start_delay_boundary 
					&& abs(gimbal->Auto.Yaw.erro) < auto_yaw_predict_max 
					&& abs(yaw_kf_result[KF_SPEED]) > auto_yaw_speed_predict_low
					&& abs(yaw_kf_result[KF_SPEED]) < auto_yaw_speed_predict_high)
			{
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
					yaw_angle_predict = RAMP_float(yaw_predict_temp, yaw_angle_predict, auto_yaw_angle_predict_ramp);	// STEP_float();
				else 
					yaw_angle_predict = STEP_float(yaw_predict_temp, step, &step_cnt, step_death);
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
				pid[GYRO][YAW_205].Angle.target = gimbal->Auto.Yaw.target;//RAMP_float(gimbal->Auto.Yaw.target, pid[GYRO][YAW_205].Angle.feedback, auto_yaw_ramp);

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
			pid[GYRO][PITCH_206].Angle.target = gimbal->Auto.Pitch.target;//RAMP_float(gimbal->Auto.Pitch.target, pid[GYRO][PITCH_206].Angle.feedback, auto_pitch_ramp);
		}
	}
	/* 未识别到目标可以随意移动云台 */
	else {
		/* 重置延时 */
		auto_predict_start_delay = 0;	// 卡尔曼滤波器延时
		mobpre_yaw_left_delay = 0;		// 重置左预测开火延迟
		mobpre_yaw_right_delay = 0;		// 重置右预测开火延迟
		mobpre_yaw_stop_delay = 0;		// 重置停止预测开火延迟
		
		KEY_setGimbalAngle();
		
//		if(vision_truly_lost == LOST) {
//			last_speed = 0;
//			last_distance = 0;
//			lost_compensation = 0;
//			/* # 这里先简化成普通的键盘移动，后续再优化 */
//			/*----期望修改----*/
//			KEY_setGimbalAngle();
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

/**
 *	@brief	计算速度和预测角度
 */
void GIMBAL_calPredictInfo(void)
{
	/* 非自瞄模式下也更新 */
	if(!GIMBAL_ifAutoMode()) {
		/* 更新二阶卡尔曼速度先验估计值 */
		yaw_angle_speed = speed_calculate(&yaw_angle_speed_struct, xTaskGetTickCount(), Gimbal_PID[GYRO][YAW_205].Angle.feedback);	
		pitch_angle_speed = speed_calculate(&pitch_angle_speed_struct, xTaskGetTickCount(), Gimbal_PID[GYRO][PITCH_206].Angle.feedback);

		/* 对角度和速度进行二阶卡尔曼滤波融合,0位置,1速度 */
		yaw_kf_result = kalman_filter_calc(&yaw_kalman_filter, Gimbal_PID[GYRO][YAW_205].Angle.feedback, 0);		// 识别不到时认为目标速度为0
		pitch_kf_result = kalman_filter_calc(&pitch_kalman_filter, Gimbal_PID[GYRO][PITCH_206].Angle.feedback, 0);	// 识别不到时认为目标速度为0

		js_yaw_speed = yaw_kf_result[KF_SPEED];
		js_yaw_angle = yaw_kf_result[KF_ANGLE];
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
void GIMBAL_BUFF_pidCalculate(Gimbal_PID_t pid[GIMBAL_MODE_COUNT][GIMBAL_MOTOR_COUNT], Gimbal_Info_t *gimbal)
{
	static float yaw_gyro_mid, yaw_mech_mid;
	static float pitch_gyro_mid, pitch_mech_mid;
	
	static uint16_t into_buff_time = 0;
	static uint16_t lost_cnt = 0;
	
	/* 计算打符帧率 */
	gimbal->Buff.Time[NOW] = xTaskGetTickCount();
	gimbal->Buff.Time[DELTA] = gimbal->Auto.Time[NOW] - gimbal->Auto.Time[PREV];

//	/* 切换pitch补偿角 */
//	if((GIMBAL_MECH_PITCH_ANGLE_DOWN_LIMIT - pid[GYRO][PITCH_206].Angle.feedback) > 0)
//		GIMBAL_BUFF_PITCH_COMPENSATION = BUFF_PITCH_MODIFY_TABLE[(uint8_t)((GIMBAL_MECH_PITCH_ANGLE_DOWN_LIMIT - pid[GYRO][PITCH_206].Angle.feedback)/100)];
//	else
//		GIMBAL_BUFF_PITCH_COMPENSATION = 0;
	
	if(gimbal->Buff.FLAG_first_into_buff == true) {
		gimbal->Buff.FLAG_first_into_buff = false;
		into_buff_time = 0;
		/* 记录刚进入打符模式时的陀螺仪角度 */
		yaw_gyro_mid = Gimbal_PID[GYRO][YAW_205].Angle.feedback;
		pitch_gyro_mid = Gimbal_PID[GYRO][PITCH_206].Angle.feedback;
		/* 记录刚进入打符模式时的机械角度 */
		yaw_mech_mid = Gimbal_PID[MECH][YAW_205].Angle.feedback;
		pitch_mech_mid = Gimbal_PID[MECH][PITCH_206].Angle.feedback;
	} else {
		/* 防止加得太大 */
		if(into_buff_time < 250)	
			into_buff_time++;
	}
	
	/* 数据更新 */
	if(VISION_getFlagStatus(VISION_FLAG_DATA_UPDATE) == true) {
		/* 清除数据更新标志位，防止未接收到新数据的情况下重复进入 */
		VISION_clearFlagStatus(VISION_FLAG_DATA_UPDATE);	
		if(VISION_getFlagStatus(VISION_FLAG_LOCK_BUFF) == true) {	// 识别到目标则更新误差
			lost_cnt = 0;
			if(into_buff_time > 100) {
				/* Yaw 误差计算 */
				gimbal->Buff.Yaw.erro = gimbal->Buff.Yaw.kp * VISION_BUFF_getYawFeedback() + GIMBAL_BUFF_YAW_COMPENSATION;
				gimbal->Buff.Yaw.target = pid[GYRO][YAW_205].Angle.feedback + gimbal->Buff.Yaw.erro;
				/* Pitch 误差计算 */
				gimbal->Buff.Pitch.erro = (gimbal->Buff.Pitch.kp * VISION_BUFF_getPitchFeedback()) + GIMBAL_BUFF_PITCH_COMPENSATION;
				gimbal->Buff.Pitch.target = pid[GYRO][PITCH_206].Angle.feedback + gimbal->Buff.Pitch.erro;
				/*----期望修改----*/
				pid[GYRO][YAW_205].Angle.target = RAMP_float(gimbal->Buff.Yaw.target, pid[GYRO][YAW_205].Angle.feedback, GIMBAL_BUFF_YAW_RAMP);
				pid[GYRO][PITCH_206].Angle.target = RAMP_float(gimbal->Buff.Pitch.target, pid[GYRO][PITCH_206].Angle.feedback, GIMBAL_BUFF_PITCH_RAMP);				
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
					pid[GYRO][YAW_205].Angle.target = GIMBAL_GYRO_yawTargetBoundaryProcess(&Gimbal_PID[GYRO][YAW_205], 
													((yaw_mech_mid - Gimbal_PID[MECH][YAW_205].Angle.feedback)*360/8192.f));
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
void GIMBAL_normalControl(void)
{
	KEY_setGimbalAngle();
	KEY_setGimbalTurn();
	KEY_setQuickPickUp();	
	//KEY_setTopGyro();
}

/**
 *	@brief	云台自瞄控制
 */
void GIMBAL_autoControl(void)
{
	/* 视觉数据可用 && 键盘模式下 */
	if( (VISION_isDataValid()) && (Flag.Remote.FLAG_mode == KEY) ) {
		/*----期望修改----*/
		/* 视觉预测版 */
		//GIMBAL_VISION_AUTO_pidCalculate(Gimbal_PID, &Gimbal);
		/* 电控预测版 */
		GIMBAL_AUTO_pidCalculate(Gimbal_PID, &Gimbal);
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
void GIMBAL_buffControl(void)
{
	/* 按下WSADQEV(任意方向键)则退出打符模式 */
	if((IF_KEY_PRESSED_W || IF_KEY_PRESSED_S || IF_KEY_PRESSED_A || IF_KEY_PRESSED_D
		|| IF_KEY_PRESSED_Q || IF_KEY_PRESSED_E || IF_KEY_PRESSED_V)&&(!IF_KEY_PRESSED_CTRL)) 
	{
		Gimbal.State.mode = GIMBAL_MODE_NORMAL;	// 进入正常模式
		Flag.Gimbal.FLAG_pidMode = GYRO;		// 进入陀螺仪模式
		
		Gimbal_PID[GYRO][YAW_205].Angle.target = Gimbal_PID[GYRO][YAW_205].Angle.feedback;
		Gimbal_PID[GYRO][PITCH_206].Angle.target = Gimbal_PID[GYRO][PITCH_206].Angle.feedback;	// GIMBAL_GYRO_PITCH_ANGLE_MID_LIMIT

		/* 拨盘模式调整 */
		REVOLVER_setAction(SHOOT_NORMAL);		// 变成常规控制模式
		/* 视觉模式调整 */
		VISION_setMode(VISION_MODE_MANUAL);	// 手动模式
		/* 底盘模式调整 */
		CHASSIS_setMode(CHAS_MODE_NORMAL);
	}
	
	/* 视觉数据可用 && 键盘模式下 */
	if( (VISION_isDataValid()) && (Flag.Remote.FLAG_mode == KEY) ) {
		/*----期望修改----*/
		GIMBAL_BUFF_pidCalculate(Gimbal_PID, &Gimbal);	
	}
}

/**
 *	@brief	云台自动补弹
 *	@note		
 */
void GIMBAL_reloadBullet(void)
{
	Gimbal_PID[Flag.Gimbal.FLAG_pidMode][PITCH_206].Angle.target = GIMBAL_MECH_PITCH_ANGLE_MID_LIMIT;
}

/* #任务层# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	pid控制器最终输出
 */
uint8_t test_yaw_pid = 0;
uint8_t test_pitch_pid = 0;
float   test_yaw_speed_max_target = 8000;
float   test_pitch_speed_max_target = 4000;
void GIMBAL_pidControlTask(void)
{
	/* YAW 角度环 */
	GIMBAL_Angle_pidCalculate(Gimbal_PID[Flag.Gimbal.FLAG_pidMode], YAW_205);
	/* YAW 速度环 */
	if(test_yaw_pid == 0) {
			Gimbal_PID[Flag.Gimbal.FLAG_pidMode][YAW_205].Speed.target = Gimbal_PID[Flag.Gimbal.FLAG_pidMode][YAW_205].Angle.out;
	} else {
		Gimbal_PID[Flag.Gimbal.FLAG_pidMode][YAW_205].Speed.target = (RC_Ctl_Info.rc.ch0 - 1024)/660.f * test_yaw_speed_max_target;
	}
	GIMBAL_Speed_pidCalculate(Gimbal_PID[Flag.Gimbal.FLAG_pidMode], YAW_205);
	
	/* PITCH 角度环 */
	GIMBAL_Angle_pidCalculate(Gimbal_PID[Flag.Gimbal.FLAG_pidMode], PITCH_206);
	/* PITCH 速度环 */
	if(test_pitch_pid == 0) {
		Gimbal_PID[Flag.Gimbal.FLAG_pidMode][PITCH_206].Speed.target = Gimbal_PID[Flag.Gimbal.FLAG_pidMode][PITCH_206].Angle.out;
	} 
	else {
		Gimbal_PID[Flag.Gimbal.FLAG_pidMode][PITCH_206].Speed.target = -RC_RIGH_CH_UD_VALUE/660.f * test_pitch_speed_max_target;
	}
	GIMBAL_Speed_pidCalculate(Gimbal_PID[Flag.Gimbal.FLAG_pidMode], PITCH_206);
	
	GIMBAL_pidOut(Gimbal_PID[Flag.Gimbal.FLAG_pidMode]);	
}

/**
 *	@brief	遥控控制云台
 */
void GIMBAL_rcControlTask(void)
{
	REMOTE_setGimbalAngle();	
	//REMOTE_setTopGyro();
}

/**
 *	@brief	键盘控制云台
 */
void GIMBAL_keyControlTask(void)
{
	/* 设置云台的模式 */
	KEY_setGimbalMode(&RC_Ctl_Info);
	switch(Gimbal.State.mode)
	{
		case GIMBAL_MODE_NORMAL:
			GIMBAL_normalControl();
			break;
		case GIMBAL_MODE_AUTO:
			GIMBAL_autoControl();
			break;
		case GIMBAL_MODE_BIG_BUFF:
		case GIMBAL_MODE_SMALL_BUFF:
			GIMBAL_buffControl();
			break;
		case GIMBAL_MODE_RELOAD_BULLET:
			GIMBAL_reloadBullet();
			break;
		default:
			break;
	}
}

/**
 *	@brief	云台失控保护
 */
void GIMBAL_selfProtect(void)
{	
	GIMBAL_stop(Gimbal_PID[MECH]);
	GIMBAL_stop(Gimbal_PID[GYRO]);
	GIMBAL_pidParamsInit(Gimbal_PID[MECH], GIMBAL_MOTOR_COUNT);
	GIMBAL_pidParamsInit(Gimbal_PID[GYRO], GIMBAL_MOTOR_COUNT);	
	
	GIMBAL_calPredictInfo();
}

/**
 *	@brief	云台控制
 */
void GIMBAL_control(void)
{
	/*----信息读入----*/
	//GIMBAL_IMU_recordFeedback(Gimbal_PID);
	GIMBAL_getInfo();
	/*----期望修改----*/
	if(BM_ifSet(BitMask.System.BM_reset, BM_RESET_GIMBAL)) {	// 复位状态
		Gimbal.State.mode = GIMBAL_MODE_NORMAL;
		GIMBAL_reset(); // 云台复位
	} else {
		if(Flag.Remote.FLAG_mode == RC) {
			GIMBAL_rcControlTask();
		} else if(Flag.Remote.FLAG_mode == KEY) {
			GIMBAL_keyControlTask();
		}
	}

	/* 根据云台模式切换PID参数 */
	GIMBAL_pidParamsSwitch(Gimbal_PID, &Gimbal);

	/*----最终输出----*/
	GIMBAL_pidControlTask();
}
