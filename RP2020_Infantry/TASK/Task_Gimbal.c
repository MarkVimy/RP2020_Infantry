/**
 * @file        Task_Gimbal.c
 * @author      MaWeiming
 * @Version     V1.0
 * @date        1-October-2019
 * @brief       This file includes the Gimbal(��̨) external functions 
 *
 * @Verison			V1.1 (1-October-2019)
 *							V1.2 (30-Jan-2020)		-	�Ż���̨��λ�ͽ��㷨
 */

/**
 *	 ����PID	�ο���ַ��https://www.jianshu.com/p/4b0fa85cd353
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

/* ���׿����� */
#define KF_ANGLE	0
#define KF_SPEED	1
#define KF_ACCEL	2

/* �Ӿ���֡ */
#define LOST	0
#define FOUND	1

#define K_LEVEL_0	2.5
#define K_LEVEL_1	2
#define K_LEVEL_2	1.67
#define K_LEVEL_3	1

/* Private variables ---------------------------------------------------------*/
/* �������˲��� */
extKalman_t Gimbal_kalmanError[GIMBAL_MODE_COUNT][GIMBAL_MOTOR_COUNT];
extKalman_t Mpu_kalmanError[2];
extKalman_t Gimbal_Auto_kalmanError[GIMBAL_MOTOR_COUNT];

extKalman_t kalman_visionYaw,kalman_cloudYaw,kalman_targetYaw,kalman_speedYaw, kalman_dist, kalman_accel;

kalman_filter_t yaw_kalman_filter;
kalman_filter_t pitch_kalman_filter;
kalman_filter_init_t yaw_kalman_filter_para = {
  .P_data = {2, 0, 0, 2},
  .A_data = {1, 0.002/*0.001*/, 0, 1},//����ʱ����
  .H_data = {1, 0, 0, 1},
  .Q_data = {1, 0, 0, 1},
  .R_data = {2000, 0, 0, 1500}//500 1000
};//��ʼ��yaw�Ĳ���kalman����

kalman_filter_init_t pitch_kalman_filter_para = {
  .P_data = {2, 0, 0, 2},
  .A_data = {1, 0.002/*0.001*/, 0, 1},//����ʱ����
  .H_data = {1, 0, 0, 1},
  .Q_data = {1, 0, 0, 1},
  .R_data = {200, 0, 0, 400}
};//��ʼ��pitch�Ĳ���kalman����

/* ������� */
float GIMBAL_BUFF_PITCH_COMPENSATION =	5;	// 0	5
float GIMBAL_BUFF_YAW_COMPENSATION = -15;	// -175		-35
float GIMBAL_BUFF_YAW_RAMP = 170;		// 185	160	185	180
float GIMBAL_BUFF_PITCH_RAMP = 120;	// 130	105	120	140

/* �����ʱ��pitch������ */
float BUFF_PITCH_MODIFY_TABLE[11] = 
{
	// �����λ�ÿ�ʼ��4.4��(��е�� ��-�� = 4472-3045 = 1067 �ֳ�11�ݣ�100Ϊ��е�ǶȲ���)Ϊ�ǶȲ���
	/* �����ʱ���� [3]90 ~ [7]80Ϊ��Ч��Χ */
	/* ͷ̧��Խ�� []�±�ֵԽ�� => MAX_UP -> [11] MAX_DOWN -> [0] */
	/* ����ֵԽ�� -> ͷѹ��Խ��(����Ϊ0��ʱ��ͷ��ƫ�ñȽϸ�) */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

/* Ԥ�⵽λ�㷨 */
bool Mobi_Pre_Yaw = false;//Ԥ���Ƿ�����־λ
bool Mobi_Pre_Yaw_Fire = false;//Ĭ��Ԥ��û��λ����ֹ��ǹ

/* �����㷨 */
float start_gyro_yaw=0;
float now_gyro_yaw=0;
float last_gyro_yaw=0;
float delta_gyro_yaw=0;

float k_level;	// ����Ӱ�����ӣ��������ʱΪ1��

/* ## Global variables ## ----------------------------------------------------*/
 
/**
 *	@pid �����ֲ���
 *	1. ����ģʽ	- ��е+������
 *	2. ����ģʽ - ������
 *	3. ���ģʽ - ��е+������
 *	4. �������ģʽ	- ��е
 */

// 34.40 46.25 10.45
/**
 *	@brief	��̨PID
 */
Gimbal_PID_t	Gimbal_PID[GIMBAL_MODE_COUNT][GIMBAL_MOTOR_COUNT] = {
	{	// MECH - ��еģʽ
		{	// YAW(��еģʽ)
			/* �ٶȻ� */
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
			/* λ�û� */
			.Angle.kp = 10.51,	
			.Angle.ki = 0,			
			.Angle.kd = 0,
			.Angle.target = GIMBAL_MECH_YAW_MID_ANGLE,	// ����
			.Angle.feedback = 0,
			.Angle.erro = 0,
			.Angle.last_erro = 0,
			.Angle.integrate = 0,
			.Angle.integrate_max = 0,
			.Angle.pout = 0,
			.Angle.iout = 0,
			.Angle.dout = 0,
			.Angle.out = 0,	
			/* б������ */
			.AngleRampTarget = 0,
			.AngleRampFeedback = 0,
			/* ��� */
			.Out = 0,
		},
		{	// PITCH(��еģʽ)
			/* �ٶȻ� */
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
			/* λ�û� */
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
			/* б������ */
			.AngleRampTarget = 0,
			.AngleRampFeedback = 0,
			/* ��� */
			.Out = 0,		
		},
	},	// MECH - ��еģʽ
	{	// GYRO - ������ģʽ
		{	// YAW(������ģʽ)
			/* �ٶȻ� */
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
			/* λ�û� */
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
			/* б������ */
			.AngleRampTarget = 0,
			.AngleRampFeedback = 0,
			/* ��� */
			.Out = 0,		
		},
		{	// PITCH(������ģʽ) - ���û�еģʽ��һ��
			/* �ٶȻ� */
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
			/* λ�û� */
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
			/* б������ */
			.AngleRampTarget = 0,
			.AngleRampFeedback = 0,
			/* ��� */
			.Out = 0,
		},
	}	// GYRO - ������ģʽ
};

/**
 *	@brief	��̨�ۺ���Ϣ
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

/* �Ӿ���֡������� */
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
 *	@brief	��̨���PID������ʼ��
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
 *	@brief	������̨��ǰģʽ����PID�������л�
 */
float auto_yaw_angle_kp = 9.25f;	// 7(ramp280)			7.5		7.75	8.5		9.25
float auto_yaw_speed_kp = 31.5f;	// 31.5			34.5	31.5	34.5	31.5	32.5
float auto_yaw_speed_ki = 575.f;	// 550			650		550		650		575		575

float auto_pitch_angle_kp = 8.5f;		// 10.5		10.5(ramp80)	10.5		8.5		8
float auto_pitch_speed_kp = 15.25f;	// 16.65	// 11.15	18.5			12.85		15.85	15.65
float auto_pitch_speed_ki = 275.f;	// 250		250				175			200		275

float buff_yaw_angle_kp = 6.15f;	// 3.65(ramp115)	4.5			3.75		3.85	9			6.15��ramp 3000)	6.15(ramp 170)
float buff_yaw_speed_kp = 28.5f;	// 31.5						30.5		30.5		33.5	30.5	29.5							28.5
float buff_yaw_speed_ki = 560.f;	// 500						300			350.		500		550		575								560

float buff_pitch_angle_kp = 7.f;		// 3			6			5.85	(ramp 125)	6.55(ramp140)	5.85	(ramp 140)	6			7
float buff_pitch_speed_kp = 12.f;	// 13.5		13.65	12.65							13.45					13.45								14.5	12
float buff_pitch_speed_ki = 250.f;	// 250.f	400		500								250						200								215		250

float normal_mech_yaw_angle_kp = 12.05f;	// 		10.35	10.65			11.25		11.55		13.05	14.45
float normal_mech_yaw_speed_kp = 28.5f;	// 34.32	18.75	26.5			27.5		28.5	33.5	33.5
float normal_mech_yaw_speed_ki = 350.f;	// 38.10	32.5	90				225			200		550	

float normal_gyro_yaw_angle_kp = 12.25f;// 9.5		9.5	6.5		7.15	7.25	7.25		8			6.5		12.25(�ۼ�ֵ)
float normal_gyro_yaw_speed_kp = 29.f;	// 26.75	20	13.5	14.25	14		14.25	14.35		20		29		
float normal_gyro_yaw_speed_ki = 510.f;	// 33.50	0		240		250		250		325			375		505		510

float normal_pitch_angle_kp = 12.f;	// 8.35			12.50	12		15			14.65
float normal_pitch_speed_kp = 12.65f;	// 15.10	16.65	14.25		12.85	14.75		14.65
float normal_pitch_speed_ki = 180.f;	// 39.5		55		100				150		180		180.

void GIMBAL_PidParamsSwitch(Gimbal_PID_t pid[GIMBAL_MODE_COUNT][GIMBAL_MOTOR_COUNT], Gimbal_Info_t *gimbal)
{
	switch(gimbal->State.mode)
	{
		case GIMBAL_MODE_RELOAD_BULLET:
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
			if(VISION_GetFlagStatus(VISION_FLAG_LOCK_TARGET) == true) // ����Ŀ��
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
			else	// δ����Ŀ���������ƶ���̨
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
 *	@brief	��̨���ж��
 */
void GIMBAL_Stop(Gimbal_PID_t *pid)
{
	static int16_t pid_out[4] = {0, 0, 0, 0};
	
	/* �ڻ��ٶȻ�������� */
	pid[YAW_205].Speed.out = 0;
	pid[YAW_205].Out = 0;
	pid[PITCH_206].Speed.out = 0;
	pid[PITCH_206].Out = 0;
	
	CAN1_Send(0x1FF, pid_out);	// ��̨CAN���߱�׼��ʶ��
}

/**
 *	@brief	��̨����ٶȻ�
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
	pid[MOTORx].Speed.iout = pid[MOTORx].Speed.ki * pid[MOTORx].Speed.integrate * 0.002f; // ģ��19��������(0.002f��2ms����˼)
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
	/* �ڻ��ٶȻ�������� */
	pid[MOTORx].Out = pid[MOTORx].Speed.out;
	
	js_gimbal_yaw_speed_out = pid[YAW_205].Speed.out;
	js_gimbal_pitch_speed_out = pid[PITCH_206].Speed.out;
}

/**
 *	@brief	��̨���λ�û�
 *	@note
 *			����PID
 *			�ڻ� ����ֵ �������ٶ�(ʵ���Ͼ����ٶȻ�)
 *					 ���ֵ �����Ǽ��ٶ�
 *
 *			�⻷ ����ֵ �����Ƕ�
 *					 ���ֵ �������ٶ�
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
	
	if((Gimbal.State.pid_mode == MECH) && (MOTORx == YAW_205)) {	// ������Ч���
		if(pid[MOTORx].Angle.erro >= 4096.f) {
			pid[MOTORx].Angle.erro = -8192.f + pid[MOTORx].Angle.erro;
		} else if(pid[MOTORx].Angle.erro <= -4096.f) {
			pid[MOTORx].Angle.erro = 8192.f + pid[MOTORx].Angle.erro;
		}
	}	
	
	if(MOTORx == PITCH_206) {	// ������Ч���
		if(pid[MOTORx].Angle.erro >= 4096.f) {
			pid[MOTORx].Angle.erro = -8192.f + pid[MOTORx].Angle.erro;
		} else if(pid[MOTORx].Angle.erro <= -4096.f) {
			pid[MOTORx].Angle.erro = 8192.f + pid[MOTORx].Angle.erro;
		}
	}		
	
//	if((Gimbal.State.pid_mode == GYRO) && (MOTORx == YAW_205)) {	// ������Ч���
//		if(pid[MOTORx].Angle.erro >= 180.f*GIMBAL_GYRO_ANGLE_ZOOM_INDEX) {
//			pid[MOTORx].Angle.erro = -(360.f*GIMBAL_GYRO_ANGLE_ZOOM_INDEX - pid[MOTORx].Angle.erro);
//		} else if(pid[MOTORx].Angle.erro <= -180.f*GIMBAL_GYRO_ANGLE_ZOOM_INDEX) {
//			pid[MOTORx].Angle.erro = +(360.f*GIMBAL_GYRO_ANGLE_ZOOM_INDEX + pid[MOTORx].Angle.erro);
//		}
//	}
	
	/* �������п������˲���������Ƶ�ȷ����� */
	if(Gimbal.State.mode == GIMBAL_MODE_NORMAL) {	// ����ģʽ��
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
	pid[MOTORx].Angle.iout = pid[MOTORx].Angle.ki * pid[MOTORx].Angle.integrate * 0.002f; // ģ��19��������(0.002f��2ms����˼)
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
 *	@brief	��̨���PID���������
 */
void GIMBAL_PidOut(Gimbal_PID_t *pid)
{
	int16_t pidOut[4] = {0, 0, 0, 0};
	
	/* CAN���͵�ѹֵ */
	if(BitMask.Gimbal.BM_rxReport & BM_RX_REPORT_205) {
		pidOut[YAW_205] = (int16_t)pid[YAW_205].Out;		// 0x205
	}
	else {
		pidOut[YAW_205] = 0;	// ʧ����ж��
	}
//	pidOut[YAW_205] = (int16_t)pid[YAW_205].Out;		// 0x205
	if(BitMask.Gimbal.BM_rxReport & BM_RX_REPORT_206) {
		pidOut[PITCH_206] = (int16_t)pid[PITCH_206].Out;	// 0x206
	} 
	else {
		pidOut[PITCH_206] = 0;	// ʧ����ж��
	}
	
	CAN1_QueueSend(0x1FF, pidOut);
}

/**
 *	@brief	��̨�������˲�����ʼ��
 */
float k_Auto_Yaw_R = 20;
float k_Auto_Pitch_R = 10;
float k_Zx_Yaw_Q = 1;
float k_Zx_Yaw_R = 1000;
void GIMBAL_KalmanCreate(void)
{
	/* �������˲�����ʼ�� */
	KalmanCreate(&Gimbal_kalmanError[MECH][YAW_205], 1, 40);
	KalmanCreate(&Gimbal_kalmanError[MECH][PITCH_206], 1, 60);
	KalmanCreate(&Gimbal_kalmanError[GYRO][YAW_205], 1, 40);
	KalmanCreate(&Gimbal_kalmanError[GYRO][PITCH_206], 1, 60);
	/* Mpu6050 */
	KalmanCreate(&Mpu_kalmanError[YAW_205], 1, 80);
	KalmanCreate(&Mpu_kalmanError[PITCH_206], 1, 10);	
	/* ���� */
	KalmanCreate(&Gimbal_Auto_kalmanError[YAW_205], 1, k_Auto_Yaw_R);
	KalmanCreate(&Gimbal_Auto_kalmanError[PITCH_206], 1, k_Auto_Pitch_R);
	/* ���鿨�����˲�,���� */
	mat_init(&yaw_kalman_filter.Q,2,2, yaw_kalman_filter_para.Q_data);
	mat_init(&yaw_kalman_filter.R,2,2, yaw_kalman_filter_para.R_data);
	kalman_filter_init(&yaw_kalman_filter, &yaw_kalman_filter_para);
	
	mat_init(&pitch_kalman_filter.Q,2,2, pitch_kalman_filter_para.Q_data);
	mat_init(&pitch_kalman_filter.R,2,2, pitch_kalman_filter_para.R_data);
	kalman_filter_init(&pitch_kalman_filter, &pitch_kalman_filter_para);
	
	/* ZX�������˲��� */
	KalmanCreate(&kalman_speedYaw, k_Zx_Yaw_Q, k_Zx_Yaw_R);
}

/* #��Ϣ��# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	��̨����ģʽ
 */
void GIMBAL_SetMode(Gimbal_Mode_t mode)
{
	Gimbal.State.mode = mode;
}

/**
 *	@brief	������̨��ǰ��ģʽ
 */
Gimbal_Mode_t GIMBAL_GetMode(void)
{
	return Gimbal.State.mode;
}

/**
 *	@brief	���ݵ��̿����߼�������е��ֵ
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
 *	@brief	��̨�Ƿ��ڻ�еģʽ
 */
bool GIMBAL_IfMechMode(void)
{
	return (Gimbal.State.pid_mode == MECH);
}

/**
 *	@brief	��̨�Ƿ���������ģʽ
 */
bool GIMBAL_IfGyroMode(void)
{
	return (Gimbal.State.pid_mode == GYRO);
}

/**
 *	@brief	�ж���̨�Ƿ��ڳ���ģʽ
 *	@return true - ����ģʽ
 *			false - �ǳ���ģʽ
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
 *	@brief	�ж���̨�Ƿ�������ģʽ
 *	@return true - ����ģʽ
 *			false - ������ģʽ
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
 *	@brief	�ж���̨�Ƿ��ڴ��ģʽ
 *	@return true - ���ģʽ
 *			false - �Ǵ��ģʽ
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
 *	@brief	�ж���̨�Ƿ�������׼�ڱ�
 *	@return true - ������׼�ڱ�
 *			false - û����׼�ڱ�
 */
bool GIMBAL_IfAimSentry(void)
{
	/* �ж��ڻ����ڱ�(̧ͷpitch��С) */
	if((Gimbal_PID[GYRO][PITCH_206].Angle.feedback <= GIMBAL_AUTO_LOCK_SENTRY_ANGLE || Gimbal_PID[MECH][PITCH_206].Angle.feedback <= GIMBAL_AUTO_LOCK_SENTRY_ANGLE ) 
		&& Gimbal.State.mode == GIMBAL_MODE_AUTO) {
		return true;
	} else {
		return false;
	}
}

/**
 *	@brief	�ж���̨�Ƿ����ڲ��ӵ�
 *	@return true - ���ڲ���
 *			false - û�����ڲ���
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
 *	@brief	�����̨�Ƿ���ٵ�λ
 *	@return true - ���ٵ�λ���Դ�
 *			false - ����δ��λ��ֹ��
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
 *	@brief	�Ƿ���뿪��Ԥ��
 *	@return true - �ѿ�������Ԥ��
 *			false - δ��������Ԥ��
 */
bool GIMBAL_IfAutoMobiPre(void)
{
	if(GIMBAL_IfAutoMode() == true) {
		return Mobi_Pre_Yaw;
	} else {	// δ�������鲻������Ԥ��
		return false;
	}
}

/**
 *	@brief	Ԥ���Ƿ���ٵ�λ
 *	@return true - ���ٵ�λ���Դ�
 *			false - ����δ��λ��ֹ��
 */
bool GIMBAL_AUTO_IfChaseReady(void)
{
	return Mobi_Pre_Yaw_Fire;
}

/**
 *	@brief	��������ƫ���趨�Ļ�е����ֵ�Ļ�е�Ƕ�(-8191~+8191)
 *	@return ����ֵΪ��е�ǶȲ�ֵ(��������)
 *	@note	�涨���Ի�е��ֵΪ�ᣬ��е�Ƕ����ӵķ���Ϊ������
 */
float GIMBAL_GetTopGyroAngleOffset(void)
{
	float delta_angle, feedback;

	feedback = g_Gimbal_Motor_Info[YAW_205].angle;
	
	delta_angle = feedback - GIMBAL_TOP_YAW_MID_ANGLE;

	return delta_angle;
}

/**
 *	@brief	��̨��ȡϵͳ��Ϣ
 */
void GIMBAL_GetSysInfo(System_t *sys, Gimbal_Info_t *gim)
{
	/*----���Ʒ�ʽ----*/
	/* ���Ʒ�ʽ - ң���� */
	if(sys->RemoteMode == RC) {
		gim->State.remote_mode = RC;
	} 
	/* ���Ʒ�ʽ - ���� */
	else if(sys->RemoteMode == KEY) {
		gim->State.remote_mode = KEY;
	}
	
	/*----ģʽ�޸�----*/
	switch(sys->Action)
	{
		case SYS_ACT_NORMAL: 
			{
				// �ջָ�����ģʽ
				if(gim->State.mode != GIMBAL_MODE_NORMAL) {
					Gimbal_PID[gim->State.pid_mode][YAW_205].Angle.target = Gimbal_PID[gim->State.pid_mode][YAW_205].Angle.feedback;
					Gimbal_PID[gim->State.pid_mode][PITCH_206].Angle.target = Gimbal_PID[gim->State.pid_mode][PITCH_206].Angle.feedback;
				}
				gim->State.mode = GIMBAL_MODE_NORMAL;
			}break;
		case SYS_ACT_AUTO: 
			{
				// �ս�������ģʽ
				if(gim->State.mode != GIMBAL_MODE_AUTO)
					Gimbal.Auto.FLAG_first_into_auto = true;
				gim->State.mode = GIMBAL_MODE_AUTO;
			}break;
		case SYS_ACT_BUFF:
			{
				// �ս�����ģʽ
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
	
	/*----Pidģʽ----*/
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
 *	@brief	��̨��ȡ����ϵͳ��Ϣ
 */
void GIMBAL_GetJudgeInfo(Judge_Info_t *judge, Gimbal_Info_t *gim)
{
	/* �������ٵȼ��޸�������� */
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


/* #Ӧ�ò�# ---------------------------------------------------------------------------------------------------------------------------------------*/
/* #ң��# -----------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	����ң��ֵ������̨YAWλ�û�������ֵ(�ۼ�ֵ)
 *	@note	
 */
void REMOTE_SetGimbalAngle(void)
{
	float targetAngle;
	/* ��еģʽ */
	if(GIMBAL_IfMechMode() == true) {	
		/* Yaw */
		/* ��̨��������˶� */
		targetAngle = GIMBAL_GetMiddleAngle();
		Gimbal_PID[MECH][YAW_205].Angle.target = targetAngle;
		/* Pitch */
		targetAngle = RC_RIGH_CH_UD_VALUE * RC_GIMBAL_MECH_PITCH_SENSITIVY; // Pitch��е��Ϊ���ԽǶ�
		Gimbal_PID[MECH][PITCH_206].Angle.target = constrain(Gimbal_PID[MECH][PITCH_206].Angle.target - targetAngle, // ̧ͷ��е�Ƕȼ�С
																						GIMBAL_MECH_PITCH_UP_ANGLE, 
																						GIMBAL_MECH_PITCH_DOWN_ANGLE);	// ����Pitch��(����ֵ)
	} 
	/* ������ģʽ */
	else if(GIMBAL_IfGyroMode() == true) {	
		/* Yaw */
		targetAngle = RC_RIGH_CH_LR_VALUE * RC_GIMBAL_GYRO_YAW_SENSITIVY * (YAW_DIR);
		Gimbal_PID[GYRO][YAW_205].Angle.target += targetAngle;
		
		/* Pitch */		
		targetAngle = RC_RIGH_CH_UD_VALUE * RC_GIMBAL_GYRO_PITCH_SENSITIVY;
		Gimbal_PID[GYRO][PITCH_206].Angle.target = constrain(Gimbal_PID[GYRO][PITCH_206].Angle.target - targetAngle, // ̧ͷPitch�Ƕȼ�С
																						GIMBAL_GYRO_PITCH_UP_ANGLE, 
																						GIMBAL_GYRO_PITCH_DOWN_ANGLE);	// ����Pitch��(����ֵ)		
	}
}

/* #�������# -------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	���ݰ���ֵ������̨YAWλ�û�������ֵ(�ۼ�ֵ)
 *	@note	
 *			�������Ϊ��
 */
float js_targetYaw;
float js_targetPitch;
void KEY_SetGimbalAngle(void)
{
	float targetAngle;
	
	js_targetYaw = MOUSE_X_MOVE_SPEED * KEY_GIMBAL_GYRO_YAW_SENSITIVY;
	js_targetPitch = MOUSE_Y_MOVE_SPEED * KEY_GIMBAL_GYRO_PITCH_SENSITIVY;
	
	/* Yaw */
	targetAngle = MOUSE_X_MOVE_SPEED * KEY_GIMBAL_GYRO_YAW_SENSITIVY * (YAW_DIR);	// ���X�����ٶ�*������
	Gimbal_PID[GYRO][YAW_205].Angle.target += targetAngle;
	
	/* Pitch */		
	targetAngle = -MOUSE_Y_MOVE_SPEED * KEY_GIMBAL_GYRO_PITCH_SENSITIVY;// ���Y�����ٶ�*������
	Gimbal_PID[GYRO][PITCH_206].Angle.target = constrain(Gimbal_PID[GYRO][PITCH_206].Angle.target - targetAngle, // ̧ͷPitch�Ƕȼ�С
																					GIMBAL_GYRO_PITCH_UP_ANGLE, 
																					GIMBAL_GYRO_PITCH_DOWN_ANGLE);	// ����Pitch��(����ֵ)	
}

/**
 *	@brief	���ݰ���ֵ������̨תͷ
 */
void KEY_SetGimbalTurn(void)
{
	float targetAngle = 0;
	static uint8_t keyQLockFlag = false;
	static uint8_t keyELockFlag = false;
	static uint8_t keyVLockFlag = false;
	/* ������ʱ��Ӧ,��ֹ�ּ��� */
	static portTickType  keyCurrentTime = 0;	
	static uint32_t keyQLockTime = 0;
	static uint32_t keyELockTime = 0;
	static uint32_t keyVLockTime = 0;
		
	keyCurrentTime = xTaskGetTickCount();
	
	if(IF_KEY_PRESSED_Q) {	// ����Q
		if(keyCurrentTime > keyQLockTime) {	// 250ms��Ӧһ��
			keyQLockTime = keyCurrentTime + TIME_STAMP_250MS;
			if(keyQLockFlag == false) {
				if(IF_KEY_PRESSED_E) {
					// ͬʱ����Q��E
				} else {	// ֻ��Q��û��E
					targetAngle = -90*GIMBAL_GYRO_ANGLE_ZOOM_INDEX * (YAW_DIR);
					Gimbal_PID[GYRO][YAW_205].AngleRampTarget += targetAngle;
				}
			}
			keyQLockFlag = true;
		}
	} else {	// �ɿ�Q
		keyQLockFlag = false;
	}
	
	if(IF_KEY_PRESSED_E) {	// ����E
		if(keyCurrentTime > keyELockTime) {	// 250ms��Ӧһ��
			keyELockTime = keyCurrentTime + TIME_STAMP_250MS;		
			if(keyELockFlag == false) {
				if(IF_KEY_PRESSED_Q) {
					// ͬʱ����Q��E
				} else {	// ֻ��E��û��Q
					targetAngle = +90*GIMBAL_GYRO_ANGLE_ZOOM_INDEX * (YAW_DIR);
					Gimbal_PID[GYRO][YAW_205].AngleRampTarget += targetAngle;
				}
			}
			keyELockFlag = true;
		}
	} else {
		keyELockFlag = false;
	}
	
	if(IF_KEY_PRESSED_V && !IF_KEY_PRESSED_CTRL) {	// ����V
		if(keyCurrentTime > keyVLockTime) {	// 500ms��Ӧһ��
			keyVLockTime = keyCurrentTime + TIME_STAMP_500MS;
			
			/* Ťͷ���ܱ�־λ */
			Flag.Chassis.GoHome = true;
			
			if(keyVLockFlag == false) {
				if(IF_KEY_PRESSED_A) {
					// ͬʱ����AV �� �Ȱ�A�ٰ�V
					targetAngle = -180*GIMBAL_GYRO_ANGLE_ZOOM_INDEX * (YAW_DIR);
					Gimbal_PID[GYRO][YAW_205].AngleRampTarget += targetAngle;
				} else {	// ֻ��V��û��A
					targetAngle = +180*GIMBAL_GYRO_ANGLE_ZOOM_INDEX * (YAW_DIR);
					Gimbal_PID[GYRO][YAW_205].AngleRampTarget += targetAngle;
				}

			}
			keyVLockFlag = true;
		}
	} else {
		keyVLockFlag = false;
	}
	
	/* б�º������ۼ���������ֹͻȻ���Ӻܴ������ֵ */
	Gimbal_PID[GYRO][YAW_205].AngleRampFeedback = RampFloat(Gimbal_PID[GYRO][YAW_205].AngleRampTarget, 
															Gimbal_PID[GYRO][YAW_205].AngleRampFeedback, 
															GIMBAL_GYRO_ANGLE_ZOOM_INDEX/1.5f);
	if(Gimbal_PID[GYRO][YAW_205].AngleRampFeedback < Gimbal_PID[GYRO][YAW_205].AngleRampTarget) // �����ۼ�
	{
		Gimbal_PID[GYRO][YAW_205].Angle.target = GIMBAL_GYRO_YawBoundaryProc(&Gimbal_PID[GYRO][YAW_205], 
																			GIMBAL_GYRO_ANGLE_ZOOM_INDEX/1.5f);
	} 
	else if(Gimbal_PID[GYRO][YAW_205].AngleRampFeedback > Gimbal_PID[GYRO][YAW_205].AngleRampTarget) // �����ۼ�
	{
		Gimbal_PID[GYRO][YAW_205].Angle.target = GIMBAL_GYRO_YawBoundaryProc(&Gimbal_PID[GYRO][YAW_205], 
																			-GIMBAL_GYRO_ANGLE_ZOOM_INDEX/1.5f);
	} 
	else // ���������
	{
		/* Ťͷ��λ��־λ���� */
		Flag.Chassis.GoHome = false;
		
		Gimbal_PID[GYRO][YAW_205].AngleRampTarget = 0;
		Gimbal_PID[GYRO][YAW_205].AngleRampFeedback = 0;
	}
}

/**
 *	@brief	���ݰ���ֵ������̨����̧ͷ
 */
void KEY_SetQuickPickUp(void)
{
	float targetAngle;
	static uint8_t keyGLockFlag = false;
	
	if(IF_KEY_PRESSED_G) {
		if(keyGLockFlag == false) {
			targetAngle = 15.f/360*8192;	// ����̧ͷ15��
			Gimbal_PID[GYRO][PITCH_206].Angle.target = constrain(Gimbal_PID[GYRO][PITCH_206].Angle.target - targetAngle, // ̧ͷPitch�Ƕȼ�С
															 GIMBAL_GYRO_PITCH_UP_ANGLE, 
															 GIMBAL_GYRO_PITCH_DOWN_ANGLE);	// ����Pitch��(����ֵ)
		}
		keyGLockFlag = true;
	} else {
		keyGLockFlag = false;
	}
}

/* #��̨# -----------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	��̨�����ʼ��
 *	@note		
 */
void GIMBAL_Init(void)
{
	GIMBAL_KalmanCreate();	// �����������˲���
}

/**
 *	@brief	��̨�����λ
 *	@note	# ���û�еģʽ����
 *				# �߱�5s�ĳ�ʱ�˳����ƣ���ֹ��̨û�и�λ��λһֱ����	
 */
void GIMBAL_Reset(void)
{
	float delta_angle;
	float target_angle;
	static portTickType time_prev;
	static portTickType time_now;
	
	target_angle = GIMBAL_GetMiddleAngle();
	
	/* �ս��븴λ */
	if(Flag.Gimbal.AngleRecordStart == true) {	
		// ��¼�ϵ�ʱ��̨��е�Ƕȷ���ֵ
		Gimbal_PID[MECH][YAW_205].Angle.target = Gimbal_PID[MECH][YAW_205].Angle.feedback;
		Gimbal_PID[MECH][PITCH_206].Angle.target = Gimbal_PID[MECH][PITCH_206].Angle.feedback;
		// ��λ��ɱ�־λ�ͼ���������
		Flag.Gimbal.ResetOk = false;
		Cnt.Gimbal.ResetOk = 0;
		// ��ʼ��ʱ
		time_now = xTaskGetTickCount();
		time_prev = time_now;
		
		/* �ͽ��ƶ��㷨 */
		delta_angle = target_angle - Gimbal_PID[MECH][YAW_205].Angle.feedback;
		/* û��Խ��ȡ��� */
		if(abs(delta_angle) < 4096) {
			Gimbal_PID[MECH][YAW_205].AngleRampTarget = delta_angle;
		} 
		/* Խ������߽紦�� */
		else {
			if(delta_angle >= 4096) {
				Gimbal_PID[MECH][YAW_205].AngleRampTarget = -8192 + delta_angle;
			} 
			else if(delta_angle <= -4096) {
				Gimbal_PID[MECH][YAW_205].AngleRampTarget = 8192 + delta_angle;
			}
		}		
		
		// ����ս���ı�־λ
		Flag.Gimbal.AngleRecordStart = false;
	}	
	
	/* ��̨��λδ��� */
	if(Flag.Gimbal.ResetOk == false) {
		
		time_now = xTaskGetTickCount();	// ��λ��ʱ

		// ǿ���û�еģʽ��λ
		Gimbal.State.pid_mode = MECH;

		/* ƽ��������̨�ƶ����м䣬��ֹ�ϵ��˦
			 б�º������ۼ���������ֹͻȻ���Ӻܴ������ֵ */
		if(Gimbal_PID[MECH][YAW_205].AngleRampTarget > 0) {
			Gimbal_PID[MECH][YAW_205].Angle.target = GIMBAL_MECH_YawBoundaryProc(&Gimbal_PID[MECH][YAW_205], GIMBAL_RAMP_BEGIN_YAW);
		} 
		else if(Gimbal_PID[MECH][YAW_205].AngleRampTarget < 0) {
			Gimbal_PID[MECH][YAW_205].Angle.target = GIMBAL_MECH_YawBoundaryProc(&Gimbal_PID[MECH][YAW_205], -GIMBAL_RAMP_BEGIN_YAW);
		}
		
		Gimbal_PID[MECH][YAW_205].AngleRampTarget = RampFloat(0, Gimbal_PID[MECH][YAW_205].AngleRampTarget, GIMBAL_RAMP_BEGIN_YAW);
		
		/* ƽ��������̨�ƶ����м䣬��ֹ�ϵ��˦ */
		Gimbal_PID[MECH][PITCH_206].Angle.target = RampFloat(GIMBAL_MECH_PITCH_MID_ANGLE, Gimbal_PID[MECH][PITCH_206].Angle.target, GIMBAL_RAMP_BEGIN_PITCH);
		
		/* �ȴ���̨���� */
		if(abs(Gimbal_PID[MECH][YAW_205].Angle.feedback - target_angle) <= 1 
			&& abs(Gimbal_PID[MECH][PITCH_206].Angle.feedback - GIMBAL_MECH_PITCH_MID_ANGLE) <= 1) {
			Cnt.Gimbal.ResetOk++;
		}
			
		/* ��λ�ɹ����߳�ʱǿ���˳�(5s) */
		if(Cnt.Gimbal.ResetOk > 250 || (time_now - time_prev) >= TIME_STAMP_5000MS) {	
			Cnt.Gimbal.ResetOk = 0;
			Flag.Gimbal.ResetOk = true;// ��̨��λ�ɹ�
			/* ��¼��ʼ�����ǽǶ� */
			start_gyro_yaw = Gimbal_PID[GYRO][YAW_205].Angle.feedback;
			last_gyro_yaw = start_gyro_yaw;
			delta_gyro_yaw = 0;
		}		
	} 
	/* ��̨��λ��� */
	else if(Flag.Gimbal.ResetOk == true) {
		Flag.Gimbal.ResetOk = false;	// ���״̬��־λ
		BM_Reset(BitMask.System.BM_Reset, BM_RESET_GIMBAL);
	}	
}

/**
 *	@brief	��̨������ģʽ��¼��������
 *	@note
 *					# ����Pitch�Ƕȵ��ٽ�ֵ����
 *					1. ǹ������ - -162.4��
 *					2. ǹ������ - +145.8��
 *					�ȡ�180�õ�һ��С��90�����ֵ
 *					�涨; 
 *					�±�Ϊ��( > -17��)
 *					�ϱ�Ϊ��( < +34��)
 *
 *					# ����Yaw�Ƕȵ��ٽ�ֵ����
 *						# ����:
 *							Yawÿ���ϵ綼�᲻һ�������������Դ���������ϵ
 *							��ͬ�Ĺ�����˳ʱ��Ƕ�ֵһ���Ǽ�С��
 *					1. ǹ������ - -59.6��
 *					2. ǹ������ - +115��
 *					�ȡ�180�õ�һ��С��90�����ֵ
 *						# ���ڣ�
 *					������������yaw�����������ݵ�Ư�Ƶ���Ч�����ѣ���˲��û�еģʽ�µĽǶ�
 *					��������Ϊ������̨����ĽǶ�(����̨�ִ����ƽǶȺ�ֻ��������һ�����������)��
 *					
 *				  # �ϵ��MPU6050���ݻ���һ��ʱ�䲻�ȶ�(�����ȴ�����)
 *					# ��еģʽ�µ��ٶȷ���ֵ����IMU�Ľ��ٶȷ���ֵ
 *						ԭ�����ֶ��ƶ���̨yaw��pitch����ʱ��
 *							  # Yaw
 *							  ���������ݵķ������ɴ�5000+(���ó�����ϵ��)
 *							  �������ת�ٷ������ɴ�50+
 *							  # Pitch
 *							  ���������ݵķ������ɴ�3000+(���ó�����ϵ��)
 *							  �������ת�ٷ������ɴ�30+
 *							  �Աȿ���֪�������Ƿ����Ľ��ٶ�ֵ���ȸ��ߣ����
 *							  ������ģʽ�ͻ�еģʽ�µ��ٶȻ�������IMU�ķ�������
 *
 *					# YAW����	�Ұ� �����outΪ +
 *								��� �����outΪ -
 *					# PITCH��� ̧ͷ �����outΪ -
 *								��ͷ �����outΪ +
 *
 *	@direction
 *			IMU����
 *					̧ͷ	- ratePitch => -
 *							-	Pitch 	=> + 
 *					��ͷ 	- ratePitch => +
 *							- 	Pitch 	=> -
 *					���  	- rateYaw   => -
 *							- 	Yaw   	=> +
 *					�Ұ�  	- rateYaw   => +
 *							- 	Yaw		=> -
 *			�涨:
 *					̧ͷ	- ratePitch => -
 *								- Pitch => -(������ģʽ) 
 *					��ͷ 	- ratePitch => +
 *								- Pitch => +(������ģʽ)
 *					���  	- rateYaw   => +
 *								- Yaw   => +(������ģʽ)
 *					�Ұ�  	- rateYaw   => -
 *								- Yaw	=> -(������ģʽ)
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
	
	/* # ������ģʽ�µĽǶȷ���ֵ����IMU�ĽǶ�ֵ */
	pid[GYRO][YAW_205].Angle.feedback += delta_yaw * GIMBAL_GYRO_ANGLE_ZOOM_INDEX;
	
	/* # Yaw �ٶȷ��� # */
	rateYaw = Mpu_Info.rateYaw - Mpu_Info.rateYawOffset;

	/* # ��еģʽ�µ��ٶȷ���ֵ����IMU�Ľ��ٶ�ֵ */
	pid[MECH][YAW_205].Speed.feedback = (YAW_DIR) * rateYaw;	// GIMBAL_COMPENSATE_RATEYAW
	/* # ������ģʽ�µ��ٶȷ���ֵ����IMU�Ľ��ٶ�ֵ */
	pid[GYRO][YAW_205].Speed.feedback = (YAW_DIR) * rateYaw;	// GIMBAL_COMPENSATE_RATEYAW
	
	/* # Pitch �ٶȷ��� # */
	ratePitch = Mpu_Info.ratePitch - Mpu_Info.ratePitchOffset;

	/* # Pitch # */
	pitch = -Mpu_Info.pitch;
	/* # ������ģʽ�µĽǶȷ���ֵ����Pitch��е�Ƕȷ���ֵ */
	pid[GYRO][PITCH_206].Angle.feedback = g_Gimbal_Motor_Info[PITCH_206].angle;	
	/* # ��еģʽ�µ��ٶȷ���ֵ����IMU�Ľ��ٶ�ֵ */
	pid[MECH][PITCH_206].Speed.feedback = ratePitch;	// ̧ͷ�ٶ�Ϊ-	GIMBAL_COMPENSATE_RATEPITCH
	/* # ������ģʽ�µ��ٶȷ���ֵ����IMU�Ľ��ٶ�ֵ */
	pid[GYRO][PITCH_206].Speed.feedback = ratePitch;	// ̧ͷ�ٶ�Ϊ-	GIMBAL_COMPENSATE_RATEPITCH
}

/**
 *	@brief	��еģʽ����̨YAW����ֵ(�ۼ�ֵ)�߽紦��
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
 *	@brief	������ģʽ����̨YAW����ֵ(�ۼ�ֵ)�߽紦��
 */
float GIMBAL_GYRO_YawBoundaryProc(Gimbal_PID_t *pid, float delta_target)
{
	float target;
	target = pid->Angle.target + delta_target;
//	if(target >= 180.0f*GIMBAL_GYRO_ANGLE_ZOOM_INDEX) {	// �����ұ߽�
//		target = (-360.f*GIMBAL_GYRO_ANGLE_ZOOM_INDEX + target);
//	} else if(target <= -180.0f*GIMBAL_GYRO_ANGLE_ZOOM_INDEX){// ������߽�
//		target = (+360.f*GIMBAL_GYRO_ANGLE_ZOOM_INDEX + target);
//	}
	return target;
}

/**
 *	@brief	Ŀ���ٶȽ���
 *	@note		������֮֡��ĽǶȲ����Ŀ����ٶ�
 */
float speed_calculate(Speed_Calculate_t *info, uint32_t time, float position)
{
	info->delay_cnt++;
	if(time != info->last_time)
	{
		if((position - info->last_position) >= (180.f*GIMBAL_GYRO_ANGLE_ZOOM_INDEX)) {
			info->speed = (+360.f*GIMBAL_GYRO_ANGLE_ZOOM_INDEX - (position - info->last_position))/(time - info->last_time)*2;	// ֡������ٶ�(*2�о����ٶȷŴ����˼)
		} else if((position - info->last_position) <= (-180.f*GIMBAL_GYRO_ANGLE_ZOOM_INDEX)) {
			info->speed = (-360.f*GIMBAL_GYRO_ANGLE_ZOOM_INDEX - (position - info->last_position))/(time - info->last_time)*2;	// ֡������ٶ�(*2�о����ٶȷŴ����˼)
		} else {
			info->speed = (position - info->last_position)/(time - info->last_time)*2;	// ֡������ٶ�(*2�о����ٶȷŴ����˼)
		}
		
		info->processed_speed = info->speed;
			
		info->last_time = time;
		info->last_position = position;
		info->last_speed = info->speed;
		info->delay_cnt = 0;
	}
	
	/* ����ʱ����������Ӿ����ݸ���������еģ��������Ӿ�ʧ������һ��ʱ��󽫷����ٶ����� */
	if(info->delay_cnt > 250)	// 500ms
	{
		info->processed_speed = 0;
	}
	
	return info->processed_speed;
}



/**
 *	@brief	����ģʽPID�����Ӿ�Ԥ���
 *	@note		3.75f -> �������֮����Ҫ���¼���һ������ֵ
 */
float GIMBAL_AUTO_YAW_COMPENSATION = 0.f;	// ���Ե�ʱ���õ�
float GIMBAL_AUTO_PITCH_COMPENSATION = (0.f/360.0f*8192.0f);
float auto_yaw_ramp = 260;
float auto_pitch_ramp = 80;
void GIMBAL_VISION_AUTO_PidCalc(Gimbal_PID_t pid[GIMBAL_MODE_COUNT][GIMBAL_MOTOR_COUNT], Gimbal_Info_t *gimbal)
{
	/* ��������֡�� */
	gimbal->Auto.Time[NOW] = xTaskGetTickCount();
	gimbal->Auto.Time[DELTA] = gimbal->Auto.Time[NOW] - gimbal->Auto.Time[PREV];

//	/* >60ms ����Ϊ��ʧĿ�����ʶ�� */
//	if(gimbal->Auto.Time[DELTA] > TIME_STAMP_60MS) {
//		pid[GYRO][YAW_205].Speed.integrate = 0;
//		pid[GYRO][PITCH_206].Speed.integrate = 0;
//	}
	
	if(gimbal->Auto.FLAG_first_into_auto == true) {
		gimbal->Auto.FLAG_first_into_auto = false;
		gimbal->Auto.Yaw.target = pid[GYRO][YAW_205].Angle.feedback;
		gimbal->Auto.Pitch.target = pid[GYRO][PITCH_206].Angle.feedback;	
	}
	
	/* ���ݸ��� */
	if(VISION_GetFlagStatus(VISION_FLAG_DATA_UPDATE) == true) {
		/* ������ݸ��±�־λ����ֹδ���յ������ݵ�������ظ����� */
		VISION_ClearFlagStatus(VISION_FLAG_DATA_UPDATE);	

		if(VISION_GetFlagStatus(VISION_FLAG_LOCK_TARGET) == true) {
			/* Yaw ������ */
			gimbal->Auto.Yaw.erro = gimbal->Auto.Yaw.kp * VISION_GYRO_GetYawFeedback();
			//gimbal->Auto.Yaw.erro = KalmanFilter(&Gimbal_Auto_kalmanError[YAW_205], gimbal->Auto.Yaw.erro);
			gimbal->Auto.Yaw.target = pid[GYRO][YAW_205].Angle.feedback + gimbal->Auto.Yaw.erro;
			/* Pitch ������ */
			gimbal->Auto.Pitch.erro = (gimbal->Auto.Pitch.kp * VISION_MECH_GetPitchFeedback()) - GIMBAL_AUTO_PITCH_COMPENSATION;
			//gimbal->Auto.Pitch.erro = KalmanFilter(&Gimbal_Auto_kalmanError[PITCH_206], gimbal->Auto.Pitch.erro);
			gimbal->Auto.Pitch.target = pid[GYRO][PITCH_206].Angle.feedback + gimbal->Auto.Pitch.erro;
		} else {
			/* ���ܴ������� */
			gimbal->Auto.Yaw.erro = 0;
			gimbal->Auto.Yaw.target = pid[GYRO][YAW_205].Angle.feedback;
			gimbal->Auto.Pitch.erro = 0;
			gimbal->Auto.Pitch.target = pid[GYRO][PITCH_206].Angle.feedback;
		}
	}
	
	/*----�����޸�----*/
	pid[GYRO][YAW_205].Angle.target = RampFloat(gimbal->Auto.Yaw.target, pid[GYRO][YAW_205].Angle.feedback, auto_yaw_ramp);
	pid[GYRO][PITCH_206].Angle.target = RampFloat(gimbal->Auto.Pitch.target, pid[GYRO][PITCH_206].Angle.feedback, auto_pitch_ramp);	
//	pid[GYRO][YAW_205].Angle.target = pid[GYRO][YAW_205].Angle.feedback;
//	pid[GYRO][YAW_205].Angle.target = GIMBAL_GYRO_YawBoundaryProc(&pid[GYRO][YAW_205], gimbal->Auto.Yaw.erro);
//	pid[GYRO][PITCH_206].Angle.target = pid[GYRO][PITCH_206].Angle.feedback + gimbal->Auto.Pitch.erro;	
	
	gimbal->Auto.Time[PREV] = gimbal->Auto.Time[NOW];
}

#define AUTO_CTRL_WAY	2

/**
 *	@brief	����ģʽ���Ԥ���
 *	@note		��̨�Ұ�(Ŀ������) - Ŀ���ٶ�Ϊ��
 *					   									Ŀ��Ƕ�Ϊ��
 *					
 *					!use_zx
 *					ramp - 20
 *					yaw_predict_k 35		
 *
 *					!use_zx 						(Ч���ϲ�)
 *					�Ȳ��� - 20��
 *					yaw_predict_k 1000
 *					
 *					use_zx 35
 *					ramp - 20
 *					
 *					use_zx 350					(�о�Ч���Ϻ�)
 *					step - 10
 */
Speed_Calculate_t	yaw_angle_speed_struct, pitch_angle_speed_struct;
float *yaw_kf_result, *pitch_kf_result;	// ���׿������˲����,0�Ƕ� 1�ٶ�
float yaw_angle_speed, pitch_angle_speed;
float yaw_angle_predict, pitch_angle_predict;
float yaw_predict_k = 0.f;
float pitch_predict_k = 0.f;

float auto_predict_start_delay = 0.f;
float auto_predict_start_delay_boundary = 150;

float auto_yaw_predict_max = 12.f * GIMBAL_GYRO_ANGLE_ZOOM_INDEX;
float auto_pitch_predict_max = 8;	// ��������� 8 �� 0.35��

float auto_yaw_speed_predict_low = 0.35;
float auto_yaw_speed_predict_high = 25;

float auto_pitch_speed_predict_low = 0.05; 	// ���Է�������ƽ���ٶȻ���0.05~0.10������ǲ���ǰװ�װ������������һ��
float auto_pitch_speed_predict_high = 2;		// ���������
	
float auto_yaw_angle_predict_limit = 220;
float auto_yaw_angle_predict_ramp = 20;

float auto_pitch_angle_predict_limit = 120;	// ���������

float js_yaw_angle = 0.f;
float js_yaw_speed = 0.f;
float js_pitch_angle = 0.f;
float js_pitch_speed = 0.f;

/* zxԤ���㷨 */
float visionYawAngleRaw=0,cloudYawAngleRaw=0,targetYawAngleRaw=0,targetYawSpeedRaw=0;
float visionYawAngleKF=0,cloudYawAngleKF=0,targetYawAngleKF=0,targetYawSpeedKF=0;
float deathzoom = 0.f;
float ksc = 1.f;
float use_zx = 350.f;

/* �����㷨 */
uint8_t queue_num = 10;
uint8_t lost_num = 8;

/* ���벹�� */
float kDis = 0.f;	// ����Ӱ������(������bug)
float kPitch = 0.5f; //	̧ͷ�������� 
float distance = 0.f;
float dis_temp = 0.f;
float distance_death = 2.5f;
float dis_compensation = 0.f;
float pitch_compensation =0.f;

/* �Ȳ����㷨 */
uint8_t test_ramp_step = 1;
float 	step = 10.f;
uint8_t step_cnt = 10.f;
float 	step_death = 0.f;

/* �����ӳ� */
float mobpre_yaw_boundary = 0.5f*GIMBAL_GYRO_ANGLE_ZOOM_INDEX;	// �ƶ�Ԥ��yaw�ı߽�
float stoppre_yaw_boundary = 1.f*GIMBAL_GYRO_ANGLE_ZOOM_INDEX;	// ֹͣԤ��yaw�ı߽�
uint16_t mobpre_yaw_left_delay = 0;//����Ԥ����ʱ�жϿɿ�������
uint16_t mobpre_yaw_right_delay = 0;//����Ԥ����ʱ�жϿɿ�������
uint16_t mobpre_yaw_stop_delay = 0;//Ԥ��ر���ʱ�жϿɿ�������
	
/* ��֡������Ӿ� */
uint8_t vision_truly_lost = LOST;	
float kLost = 2.f;	// ��֡��������
float last_distance = 0.f;
float last_speed = 0.f;
float lost_compensation = 0.f;

#if (AUTO_CTRL_WAY == 1)
void GIMBAL_AUTO_PidCalc(Gimbal_PID_t pid[GIMBAL_MODE_COUNT][GIMBAL_MOTOR_COUNT], Gimbal_Info_t *gimbal)
{
	static float avrCalAngle,nowCalAngle=0;
	float none=0;
	
	float yaw_predict_temp, pitch_predict_temp;
	
	/* �״ν������� */
	if(gimbal->Auto.FLAG_first_into_auto == true) {
		gimbal->Auto.FLAG_first_into_auto = false;
		gimbal->Auto.Yaw.target = pid[GYRO][YAW_205].Angle.feedback;
		gimbal->Auto.Pitch.target = pid[GYRO][PITCH_206].Angle.feedback;	
		cQueue_fill(&LostQueue, lost_num, LOST);
	}
	
	/*----���ݸ���----*/
	if(VISION_GetFlagStatus(VISION_FLAG_DATA_UPDATE) == true) {
		/* ������ݸ��±�־λ����ֹδ���յ������ݵ�������ظ����� */
		VISION_ClearFlagStatus(VISION_FLAG_DATA_UPDATE);	

		/* ��������֡�� */
		gimbal->Auto.Time[NOW] = xTaskGetTickCount();
		gimbal->Auto.Time[DELTA] = gimbal->Auto.Time[NOW] - gimbal->Auto.Time[PREV];
		gimbal->Auto.Time[PREV] = gimbal->Auto.Time[NOW];
		
		/* ʶ��Ŀ�� */
		if(VISION_GetFlagStatus(VISION_FLAG_LOCK_TARGET) == true) {
			/* Yaw ���Ŵ� */
			gimbal->Auto.Yaw.erro = gimbal->Auto.Yaw.kp * VISION_GYRO_GetYawFeedback() + GIMBAL_AUTO_YAW_COMPENSATION;
			/* Pitch ���Ŵ� */
			gimbal->Auto.Pitch.erro = (gimbal->Auto.Pitch.kp * VISION_MECH_GetPitchFeedback()) + GIMBAL_AUTO_PITCH_COMPENSATION;
			/* ��֡���� */
			cQueue_in(&LostQueue, lost_num, FOUND);
		} 
		/* δʶ��Ŀ�� */
		else {
			/* Yaw ���Ϊ0 */
			gimbal->Auto.Yaw.erro = 0;
			/* Pitch ���Ϊ0 */
			gimbal->Auto.Pitch.erro = 0;
			/* ��֡���� */
			cQueue_in(&LostQueue, lost_num, LOST);
		}
		/* Yaw Ŀ����� */
		gimbal->Auto.Yaw.target = pid[GYRO][YAW_205].Angle.feedback + gimbal->Auto.Yaw.erro;
		/* Pitch Ŀ����� */
		gimbal->Auto.Pitch.target = pid[GYRO][PITCH_206].Angle.feedback + gimbal->Auto.Pitch.erro;
		
		/* ���²��� */
		step_cnt = step;
	}
	
	/* ��֡�ж� */
	if(cQueue_ifFilled(&LostQueue, lost_num, LOST)) {
		vision_truly_lost = LOST;
	} else if(cQueue_ifFilled(&LostQueue, lost_num, FOUND)) {
		vision_truly_lost = FOUND;
	}
	
	/*----���¶��׿�����Ԥ��ֵ----*/
	/* ʶ��Ŀ�� */
	if(VISION_GetFlagStatus(VISION_FLAG_LOCK_TARGET) == true) {	
		/* ���¶��׿������ٶ��������ֵ */
		yaw_angle_speed = speed_calculate(&yaw_angle_speed_struct, gimbal->Auto.Time[NOW], gimbal->Auto.Yaw.target);
		pitch_angle_speed = speed_calculate(&pitch_angle_speed_struct, gimbal->Auto.Time[NOW], gimbal->Auto.Pitch.target);

		/* �ԽǶȺ��ٶȽ��ж��׿������˲��ں�,0λ��,1�ٶ� */
		yaw_kf_result = kalman_filter_calc(&yaw_kalman_filter, gimbal->Auto.Yaw.target, yaw_angle_speed);
		pitch_kf_result = kalman_filter_calc(&pitch_kalman_filter, gimbal->Auto.Pitch.target, pitch_angle_speed);
		
		/* �� ZXԤ���ٶ� �� */
		nowCalAngle = yaw_kf_result[KF_ANGLE];	// gimbal->Auto.Yaw.target
		
		VISION_UpdateInfo(nowCalAngle, queue_num, &VisionQueue, &avrCalAngle, &none);
		
		targetYawSpeedRaw = (nowCalAngle - avrCalAngle)*ksc;
		
		if(abs(targetYawSpeedRaw) < deathzoom)
			targetYawSpeedRaw = 0.f;
		
		targetYawSpeedKF = KalmanFilter(&kalman_speedYaw, targetYawSpeedRaw);
		/* �� ZXԤ���ٶ� �� */
		last_speed = targetYawSpeedKF;
		last_distance = VISION_GetDistance()/1000.f;	//�����(m)
	} 
	/* δʶ��Ŀ�� */
	else {
		/* ���¶��׿������ٶ��������ֵ */
		yaw_angle_speed = speed_calculate(&yaw_angle_speed_struct, xTaskGetTickCount(), pid[GYRO][YAW_205].Angle.feedback);	
		pitch_angle_speed = speed_calculate(&pitch_angle_speed_struct, xTaskGetTickCount(), pid[GYRO][PITCH_206].Angle.feedback);

		/* �ԽǶȺ��ٶȽ��ж��׿������˲��ں�,0λ��,1�ٶ� */
		yaw_kf_result = kalman_filter_calc(&yaw_kalman_filter, pid[GYRO][YAW_205].Angle.feedback, 0);		// ʶ�𲻵�ʱ��ΪĿ���ٶ�Ϊ0
		pitch_kf_result = kalman_filter_calc(&pitch_kalman_filter, pid[GYRO][PITCH_206].Angle.feedback, 0);	// ʶ�𲻵�ʱ��ΪĿ���ٶ�Ϊ0
		
		/* �� ZXԤ���ٶ� �� */
		nowCalAngle = yaw_kf_result[KF_ANGLE];	// gimbal->Auto.Yaw.target
		
		VISION_UpdateInfo(nowCalAngle, queue_num, &VisionQueue, &avrCalAngle, &none);
		
		targetYawSpeedRaw = (nowCalAngle - avrCalAngle)*ksc;
		
		if(abs(targetYawSpeedRaw) < deathzoom)
			targetYawSpeedRaw = 0.f;
		
		targetYawSpeedKF = KalmanFilter(&kalman_speedYaw,0);
		/* �� ZXԤ���ٶ� �� */
	}
	
	js_yaw_speed = yaw_kf_result[KF_SPEED];
	js_yaw_angle = yaw_kf_result[KF_ANGLE];
	
	/*----���Ͼ��벹��(�ٶ������Թ�ϵ)----*/
	distance = VISION_GetDistance()/1000.f;	//�����(m)
	
	pitch_compensation = distance * kPitch;
	
	if(myDeathZoom(0, distance_death, distance) > 0.f) {
		dis_temp -= distance_death;
	} else {
		dis_temp = 0.f;
	}	
	
	/*----Ԥ��������----*/
	/* ʶ��Ŀ�� */
	if(VISION_GetFlagStatus(VISION_FLAG_LOCK_TARGET) == true) {	
		auto_predict_start_delay++;	// �˲���ʱ����
		
		/* zxԤ�� */
		if(use_zx != 0)
		{
			/* Ԥ�⿪�������ж� */
			if(auto_predict_start_delay > auto_predict_start_delay_boundary 
					&& abs(gimbal->Auto.Yaw.erro) < auto_yaw_predict_max 
					&& abs(targetYawSpeedKF) > auto_yaw_speed_predict_low
					&& abs(targetYawSpeedKF) < auto_yaw_speed_predict_high)
			{
				auto_predict_start_delay = auto_predict_start_delay_boundary + 1;
				
				/* ������벹�� */
				dis_compensation = dis_temp*kDis;
				
				/* Ŀ������ -> Ԥ����Ϊ��(��̨����) */
				if(targetYawSpeedKF < 0) {
					yaw_predict_temp = use_zx*(targetYawSpeedKF + auto_yaw_speed_predict_low) - dis_compensation; // ʱ�䳣��2ms����Ԥ��ϵ����
				} else if(targetYawSpeedKF >= 0) {
					yaw_predict_temp = use_zx*(targetYawSpeedKF - auto_yaw_speed_predict_low) + dis_compensation; // ʱ�䳣��2ms����Ԥ��ϵ����
				}
				
				/* Ԥ���������仯 */
				if(test_ramp_step == 0)
					yaw_angle_predict = RampFloat(yaw_predict_temp, yaw_angle_predict, auto_yaw_angle_predict_ramp);	// StepFloat();
				else 
					yaw_angle_predict = StepFloat(yaw_predict_temp, step, &step_cnt, step_death);
				/* Ԥ�����޷� */
				yaw_angle_predict = constrain(yaw_angle_predict, -auto_yaw_angle_predict_limit, auto_yaw_angle_predict_limit);
				
				/*----�����޸�----*/
				pid[GYRO][YAW_205].Angle.target = yaw_kf_result[KF_ANGLE] + yaw_angle_predict;	// gimbal->Auto.Yaw.target �˲�ǰʵʱ����΢�Ϻã�����ʶ�𲻵����뵽ʶ��ʱ���н�Ծ����
			
				/*----Ԥ�⵽λ�ж�----*/
				/* Ŀ�����ƶ����Ӿ�������Ŀ�������(��ʾ�ѳ�ǰ) */
				if(targetYawSpeedKF < 0 
					&& (gimbal->Auto.Yaw.erro - GIMBAL_AUTO_YAW_COMPENSATION) > mobpre_yaw_boundary)
				{
					mobpre_yaw_left_delay = 0;	// ����Ԥ�⿪����ʱ����
					
					mobpre_yaw_right_delay++;	// �ۼ������ƶ�Ԥ��
					if(mobpre_yaw_right_delay > 0) {
						Mobi_Pre_Yaw_Fire = true;
					} else {
						Mobi_Pre_Yaw_Fire = false;
					}
				}
				/* Ŀ�����ƶ����Ӿ�������Ŀ�����ұ�(��ʾ�ѳ�ǰ) */
				else if(targetYawSpeedKF > 0
					&& (gimbal->Auto.Yaw.erro - GIMBAL_AUTO_YAW_COMPENSATION) < -mobpre_yaw_boundary)
				{
					mobpre_yaw_right_delay = 0;	// ����Ԥ�⿪����ʱ����
					
					mobpre_yaw_left_delay++;		// �ۼ������ƶ�Ԥ��
					if(mobpre_yaw_left_delay > 0) {
						Mobi_Pre_Yaw_Fire = true;
					} else {
						Mobi_Pre_Yaw_Fire = false;
					}		
				}
				/* Ԥ��δ��λ */
				else {
					Mobi_Pre_Yaw_Fire = false;
					
					mobpre_yaw_left_delay = 0;
					mobpre_yaw_right_delay = 0;
				}
				
				Mobi_Pre_Yaw = true;	// ����ѿ����ƶ�Ԥ��
				mobpre_yaw_stop_delay = 0;	// ���þ�ֹʱ�Ŀ����ӳ�
				
			}
			/* δ�ﵽԤ�⿪������ */
			else {
				pid[GYRO][YAW_205].Angle.target = gimbal->Auto.Yaw.target;//RampFloat(gimbal->Auto.Yaw.target, pid[GYRO][YAW_205].Angle.feedback, auto_yaw_ramp);
				
				Mobi_Pre_Yaw = false;	// ���δ�����ƶ�Ԥ��
				mobpre_yaw_left_delay = 0;
				mobpre_yaw_right_delay = 0;
				
				if(abs(gimbal->Auto.Yaw.erro - GIMBAL_AUTO_YAW_COMPENSATION) < stoppre_yaw_boundary) 
				{
					mobpre_yaw_stop_delay++;
					if(mobpre_yaw_stop_delay > 25)	// �ȶ�50ms
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
		/* 19Ԥ�� */
		else {
			/* Ԥ�⿪�������ж� */
			if(auto_predict_start_delay > auto_predict_start_delay_boundary 
					&& abs(gimbal->Auto.Yaw.erro) < auto_yaw_predict_max 
					&& abs(yaw_kf_result[KF_SPEED]) > auto_yaw_speed_predict_low
					&& abs(yaw_kf_result[KF_SPEED]) < auto_yaw_speed_predict_high)
			{
				auto_predict_start_delay = auto_predict_start_delay_boundary + 1;
				
				/* ������벹�� */
				dis_compensation = dis_temp*kDis;
				
				/* Ŀ������ -> Ԥ����Ϊ��(��̨����) */
				if(yaw_kf_result[KF_SPEED] < 0) {
					yaw_predict_temp = yaw_predict_k*(yaw_kf_result[KF_SPEED] + auto_yaw_speed_predict_low) - dis_compensation; // ʱ�䳣��2ms����Ԥ��ϵ����
				} else if(yaw_kf_result[KF_SPEED] >= 0) {
					yaw_predict_temp = yaw_predict_k*(yaw_kf_result[KF_SPEED] - auto_yaw_speed_predict_low) + dis_compensation; // ʱ�䳣��2ms����Ԥ��ϵ����
				}
				
				/* Ԥ���������仯 */
				if(test_ramp_step == 0)
					yaw_angle_predict = RampFloat(yaw_predict_temp, yaw_angle_predict, auto_yaw_angle_predict_ramp);	// StepFloat();
				else 
					yaw_angle_predict = StepFloat(yaw_predict_temp, step, &step_cnt, step_death);
				/* Ԥ�����޷� */
				yaw_angle_predict = constrain(yaw_angle_predict, -auto_yaw_angle_predict_limit, auto_yaw_angle_predict_limit);
				
				/*----�����޸�----*/
				pid[GYRO][YAW_205].Angle.target = yaw_kf_result[KF_ANGLE] + yaw_angle_predict;	// gimbal->Auto.Yaw.target

				/*----Ԥ�⵽λ�ж�----*/
				/* Ŀ�����ƶ����Ӿ�������Ŀ�������(��ʾ�ѳ�ǰ) */
				if(yaw_kf_result[KF_SPEED] < 0 
					&& (gimbal->Auto.Yaw.erro - GIMBAL_AUTO_YAW_COMPENSATION) > mobpre_yaw_boundary)
				{
					mobpre_yaw_right_delay = 0;	// ����Ԥ�⿪����ʱ����
					
					mobpre_yaw_left_delay++;	// �ۼ������ƶ�Ԥ��
					if(mobpre_yaw_left_delay > 0) {
						Mobi_Pre_Yaw_Fire = true;
					} else {
						Mobi_Pre_Yaw_Fire = false;
					}
				}
				/* Ŀ�����ƶ����Ӿ�������Ŀ�����ұ�(��ʾ�ѳ�ǰ) */
				else if(yaw_kf_result[KF_SPEED] > 0 
					&& (gimbal->Auto.Yaw.erro - GIMBAL_AUTO_YAW_COMPENSATION) < -mobpre_yaw_boundary)
				{
					mobpre_yaw_right_delay = 0;	// ����Ԥ�⿪����ʱ����
					
					mobpre_yaw_left_delay++;		// �ۼ������ƶ�Ԥ��
					if(mobpre_yaw_left_delay > 0) {
						Mobi_Pre_Yaw_Fire = true;
					} else {
						Mobi_Pre_Yaw_Fire = false;
					}		
				}
				/* Ԥ��δ��λ */
				else {
					Mobi_Pre_Yaw_Fire = false;
					
					mobpre_yaw_left_delay = 0;
					mobpre_yaw_right_delay = 0;
				}
				
				Mobi_Pre_Yaw = true;	// ����ѿ���Ԥ��
				mobpre_yaw_stop_delay = 0;	// ���þ�ֹʱ�Ŀ����ӳ�
				
			}
			/* δ�ﵽԤ�⿪������ */
			else {
				/*----�����޸�----*/
				pid[GYRO][YAW_205].Angle.target = gimbal->Auto.Yaw.target;//RampFloat(gimbal->Auto.Yaw.target, pid[GYRO][YAW_205].Angle.feedback, auto_yaw_ramp);

				Mobi_Pre_Yaw = false;	// ���δ����Ԥ��
				mobpre_yaw_left_delay = 0;
				mobpre_yaw_right_delay = 0;
				
				if(abs(gimbal->Auto.Yaw.erro - GIMBAL_AUTO_YAW_COMPENSATION) < stoppre_yaw_boundary) 
				{
					mobpre_yaw_stop_delay++;
					if(mobpre_yaw_stop_delay > 25)	// �ȶ�50ms
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
		
		/* Pitch�������С��Ԥ�� */
		if(auto_predict_start_delay > auto_predict_start_delay_boundary
				&& abs(gimbal->Auto.Pitch.erro) < auto_pitch_predict_max 
					&& abs(pitch_kf_result[KF_SPEED]) > auto_pitch_speed_predict_low
					&& abs(pitch_kf_result[KF_SPEED]) < auto_pitch_speed_predict_high)
		{
			/* Ŀ������ -> Ԥ����Ϊ��(��̨��ͷ) */
			if(pitch_kf_result[KF_SPEED] >= 0) {
				pitch_predict_temp = pitch_predict_k * (pitch_kf_result[KF_SPEED] - auto_pitch_speed_predict_low);
			}
			/* Ŀ������ -> Ԥ����Ϊ��(��̨̧ͷ) */
			else if(pitch_kf_result[KF_SPEED] < 0){
				pitch_predict_temp = pitch_predict_k * (pitch_kf_result[KF_SPEED] + auto_pitch_speed_predict_low);
			}
			
			/* Ԥ�����޷� */
			pitch_predict_temp = constrain(pitch_predict_temp, -auto_pitch_angle_predict_limit, auto_pitch_angle_predict_limit);
			pitch_angle_predict = pitch_predict_temp;
			
			/*----�����޸�----*/
			pid[GYRO][PITCH_206].Angle.target = pitch_kf_result[KF_ANGLE] + pitch_angle_predict;	// pitch_kf_result[KF_ANGLE]
		}
		/* δ�ﵽԤ�⿪�������� */
		else {
			/*----�����޸�----*/
			pid[GYRO][PITCH_206].Angle.target = gimbal->Auto.Pitch.target;//RampFloat(gimbal->Auto.Pitch.target, pid[GYRO][PITCH_206].Angle.feedback, auto_pitch_ramp);
		}
	}
	/* δʶ��Ŀ����������ƶ���̨ */
	else {
		/* ������ʱ */
		auto_predict_start_delay = 0;	// �������˲�����ʱ
		mobpre_yaw_left_delay = 0;		// ������Ԥ�⿪���ӳ�
		mobpre_yaw_right_delay = 0;		// ������Ԥ�⿪���ӳ�
		mobpre_yaw_stop_delay = 0;		// ����ֹͣԤ�⿪���ӳ�
		
		KEY_SetGimbalAngle();
		
//		if(vision_truly_lost == LOST) {
//			last_speed = 0;
//			last_distance = 0;
//			lost_compensation = 0;
//			/* # �����ȼ򻯳���ͨ�ļ����ƶ����������Ż� */
//			/*----�����޸�----*/
//			KEY_SetGimbalAngle();
//		} else {
//			/* ��֡���� */
//			if(last_distance != 0)
//				lost_compensation += kLost * last_speed / last_distance;	// ��ʱ�����ӹ���kLost
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
 *	@brief	��̨����Ԥ��
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

uint16_t predict_delay=0;/*Ԥ�����ʱ����*/		


void VISION_AUTO_Predict(Gimbal_PID_t pid[GIMBAL_MODE_COUNT][GIMBAL_MOTOR_COUNT], Gimbal_Info_t *gimbal)
{
	static uint16_t ramp_cnt=0;/*б�¼��� -- ���ڻ��͵�֡Ӱ��*/			
	
	static bool judge_flag=false;/*��ʧ��־λ�����־λ,����һ���Բ���*/
	
	
	static uint16_t active_cnt=0,lost_cnt=0;/*�������/��ʧ����--����ʶ���δʶ���໥�л��Ĺ���*/
	
	static uint32_t vision_this_time,vision_last_time;/*�Ӿ����ܵ�ʱ��*/					
	
	static uint8_t vision_identify_target=0, last_identify_flag=0;/*��һ�εĽ��ձ�־λ*/
	
	static float now_vision_yaw=0;/*��ǰ��õ��Ӿ�yaw�Ƕ�--�������Ƕ�*/				
	
	uint32_t outline_cnt=0;	/*���߼���*/
	
//	float now_gyro_yaw=0;/*��ǰ���ϴεĻ�еyaw�Ƕ� -- ��̨�Ƕ�*/

//	static bool init=true;/*��ʼ����־λ*/		
//	
//	if(init == true)
//	{
//		start_gyro_yaw = pid[GYRO][YAW_205].Angle.feedback;
//		last_gyro_yaw = start_gyro_yaw;
//		init = false;
//		//��ʼ������
//	}	
	
	if(Gimbal.Auto.FLAG_first_into_auto == true)
	{
		//start_gyro_yaw = pid[GYRO][YAW_205].Angle.feedback;
		//last_gyro_yaw = start_gyro_yaw;
		//delta_gyro_yaw = 0;
		last_identify_flag = 0;
		predict_delay = PREDICT_DELAY_CNT;	/*�ս������ʱ����ʱԤ�ⳬǰ����ֹ��̨�ζ�*/
		Gimbal.Auto.FLAG_first_into_auto = false;
		//��ʼ������		
	}
	
	now_gyro_yaw = pid[GYRO][YAW_205].Angle.feedback; /*��ǰ��yaw�������ǽǶ�ֵ*/
	/*�����pid����̨Yaw�������ǵĽṹ�壬������Ҫ��Ϊ�˸���yaw����̨����������ǽǶ�*/ 

	delta_gyro_yaw += now_gyro_yaw - last_gyro_yaw; 	/*�������β����������ǽǶȲ�ֵ*/
	/*�����Ǽ������β����������ǽǶ�֮��Ĳ�ֵ*/ 
	
	
	cloud_yaw_raw = delta_gyro_yaw;  /*���뿨�����˲�*/
	/*������Ϊ�˽���ֵ���п������˲�*/
	 
	last_gyro_yaw = now_gyro_yaw; /*��¼��һ�εĽǶ�ֵ*/
	/*���¼�¼��һ�β����������ǽǶ�*/ 

	cloud_yaw_kf = KalmanFilter(&kalman_cloudYaw,cloud_yaw_raw)+start_gyro_yaw;	/*�˲�*/       
	
	cloud_degree = cloud_yaw_kf / GIMBAL_GYRO_ANGLE_ZOOM_INDEX;  
	/*�����ǽ�ǰ�����β����Ĳ�ֵ���п������˲����ټ�����ʼ�Ƕȣ�����Ϊ�˲������̨�����ǽǶ�*/ 
	
	/*
		������ô����Ϊ�˱ܿ���е�Ƕȵı߽����⣬�������ͬ�����Խ�������ǲ����ı߽�������޷�ֱ�ӶԽǶ����ݽ����˲������⡣ ��ϸϸƷ 
	*/	
	
	/*�����Ƕ��Ӿ�ԭʼ���ݵĴ���*/

	if(VISION_GetFlagStatus(VISION_FLAG_DATA_UPDATE) == true)
	{
		vision_identify_target = VISION_GetFlagStatus(VISION_FLAG_LOCK_TARGET);
			
		vision_yaw = VISION_GYRO_GetYawFeedback();

		
//		/*���Ӿ����ݸ��º����*/	
//		if(last_identify_flag != vision_identify_target)	/*ʶ���״̬�������л�*/
//		{
//			/*�������˵�֡���ʶ��Ŀ���ʧ��Ŀ�꣬���������*/ 
//			
//			ramp_cnt = RAMP_MAX_CNT;	/*�л���б�¹���*/
//			/*����ԭ����ramp_cntΪ0.���ڸ�ֵΪ��Ϊ0��������ʾҪ�����ݽ���б�´���*/ 
//			
//			judge_flag = true;			/*�����ж�����*/
//			/*����ʶ��״̬���ж���������������ʶ��ģʽ���л�ʱ����ʼ�����жϹ��̣��ж��Ƿ���ĸ�������ֻ�ǵ�֡���߸�ʶ��Ŀ��*/ 
//			

//			//��֡�������ж�ʶ��״̬
//		}		

//		if(judge_flag == true)		/*ʶ���״̬�������л�*/
//		{
//			/*
//				������������ڱ�����ʶ��֮����л����룬ʵ�����������ֿ��Բ�����������Ҫд���Լ����л����̴��롣����Ҳ���Կ�һ�� �ο�һ��˼·����ϸϸƷ�� 
//			*/ 
//			
//			if(vision_identify_target == 1)		/*��ʾΪ�����˵�֡���½���״̬�ж�����--���ﲻӰ���������ʶ��-��Ϊģʽû�иı�*/
//			{
//				active_cnt++; 	/*��Ծ����*/
//				
//				lost_cnt = 0;
//				
//				if(active_cnt >= ACTIVE_MAX_CNT) /*�ﵽ��ֵ���϶�Ϊʶ��*/
//				{
//					judge_flag = false;
//					
//					//gimbal_mode = ATTACK;
//				
//					predict_delay = PREDICT_DELAY_CNT;  /*�����ӳٳ�ǰ�ǵļ���ʱ��*/
//					
//					active_cnt = 0;
//					
//					/*����ȷ�Ͻ������ж�*/
//				}	
//			}
//			else
//			{
//				lost_cnt++;	 		/*��ʧ����*/
//				
//				active_cnt = 0;

//				if(lost_cnt >= LOST_MAX_CNT) /*�ﵽ��ֵ���϶�Ϊ��ʧ*/
//				{
//					judge_flag = false;
//					
//					//gimbal_mode = SCOUT; 	
//					
//					
//					lost_cnt = 0;					
//					/*���ģʽ���л�*/
//				}
//			}
//			/*�����ж������*/	
//			 
//			/*������������ʶ����л�*/ 
//		}	

		/*������ ģʽ�ж����� ������*/
		
		/*
			�������б�µĹ�����
			����������л������������������б�²�����һС��ʱ�䣬���������ڵ�֡�����ĳ��������ĸ���ʵ��������ã��ɲ��ã� 
		*/ 
//		if(ramp_cnt > 0)
//		{
//			/*б�¹���*/
//			now_vision_yaw = RampFloat(vision_yaw, now_vision_yaw, abs(vision_yaw - now_vision_yaw)/ramp_cnt);
//			ramp_cnt--;
//		}
//		else
			now_vision_yaw = vision_yaw;                          
		
		vision_degree = now_vision_yaw / GIMBAL_GYRO_ANGLE_ZOOM_INDEX;
		/* ��������ȡ��ǰ���Ӿ��Ƕȡ����� */ 
		
		update_cloud_yaw = cloud_yaw_raw+start_gyro_yaw; 			/*�Ӿ����ݸ���ʱ����̨�Ƕ�*/
		/*�Ӿ����ݸ���ʱ����̨����*/ 
		
		vision_yaw_raw = now_vision_yaw;	/*�����������ݳ߶���ͬ*/    
		/*�Ӿ����������ݣ�ת���ɺ������ǽǶȵ�ͬһ���߶�*/ 
		
		
		vision_yaw_kf = KalmanFilter(&kalman_visionYaw,vision_yaw_raw); 	/*���Ӿ��Ƕ��������������˲�*/  
		/*�Ӿ����ݵĿ������˲�*/ 
		
		
		vision_dis_raw = VISION_GetDistance()/1000.f;	/*�������*/
		/*��ȡ��������*/  
	
	
		vision_dis_kf = KalmanFilter(&kalman_dist,vision_dis_raw);	/*�˲�*/
		/*�������ݿ������˲���*/ 
		
		
		vision_this_time = xTaskGetTickCount();		/*��ȡ��ǰʱ��*/
		/*�����Ӿ����µ�ʱ�� -- ��������õ�*/ 
		
		VISION_ClearFlagStatus(VISION_FLAG_DATA_UPDATE);	/*�����־λ*/	

		/*
			֮���Խ����ݸ��µ����ݷ��ڸ��±�־λ���Ϊ�˱�֤Ԥ�����ݿ��Բ�����̨�����˶���Ӱ�졣
			��ϸ˵�˾�����Ϊ���ݵĸ���Ƶ��ԶС����̨�ĽǶ����ݣ����Ӿ�û���µ�ʱ������̨������һֱ�ڸ��¡�
			�⵼������̨���˶����Ԥ����ٶȴ����Ŷ����ݡ�����Ĵ������Ϊ�˾����ܵر������������ 
		*/ 
	}
	/*���������ݸ��µĴ��������*/
		
	target_yaw_raw = -vision_yaw_kf*k_scale_vision + cloud_yaw_kf;		/*��Ŀ��������Ƕ�*/
	/*Ŀ���λ�õ�yaw�Ƕ�Ԥ�⣬����ķ�������Ϊ������װ������̨������ͷ�İ�װ֮��Ĺ�ϵ�����忴�Լ��İ�װ��ϵ��*/ 


	target_yaw_kf = KalmanFilter(&kalman_targetYaw,target_yaw_raw);		/*��Ŀ���yaw�ǶȽ��ж�λ*/   
	
	target_degree=target_yaw_kf / GIMBAL_GYRO_ANGLE_ZOOM_INDEX;  
	/*��Ŀ��ǶȽ����˲�*/ 
	
	
	now_cal_yaw = -vision_yaw_raw*k_scale_vision + update_cloud_yaw;	 /*Ŀ��ĽǶ� -- ���ڼ���Ŀ���ٶ�*/
	/*��ʼԤ���ٶȣ������k_scale_vision������ͷ�Ƕȱ仯��ת��ϵ����
	
		�ٸ����ӣ�����ͷ���Ӿ����ݱ仯��10�㣬����ʵ������̨ת����15�㣬�����ʱ�����Ҫ���Ӿ������ݳ���һ��1.5����ʱ��k_scale_vision��Ϊ1.5 
	*/

	
	
	target_speed_raw = Get_Target_Speed(queue_speed_length,now_cal_yaw)*k_speed; 	/*�������Ŀ���ٶȵ�Ԥ��*/
	/*��ʼ��Ŀ����ٶȽ���Ԥ�⣬�������ֱ�Ӳο��Ҹ��Ĵ��롣k_speed������ϵ������Ϊ�õ����ٶ����ݱȽ�С����˷Ŵ��ŵ��������ȽϺ���*/ 
	
	
	if(vision_identify_target == 1)				/*�����ٶ�ֵ*/
		target_speed_kf = KalmanFilter(&kalman_speedYaw,target_speed_raw);
	else
		target_speed_kf = KalmanFilter(&kalman_speedYaw,0);			
	
	/*�ο�ȥ���������*/ 
	
	
	target_accel_raw = Get_Target_Accel(queue_accel_length,target_speed_kf);	 /*��ȡ���ٶ�*/
	
	target_accel_raw = myDeathZoom(0,0.1,target_accel_raw);		/*�������� - �˳�0�㸽��������*/
	
	target_accel_kf = KalmanFilter(&kalman_accel,target_accel_raw);		/*�������˲�*/

	/*���������ǽ�����ٶȵ����ݣ����ٶȵĽ���ԭ��һ��������������*/		
		
	if(predict_delay>0)
	{
		/*Ԥ�����ʱ*/
		predict_delay--;
		predict_angle_raw=0;
		
		/*����predict����ʱ��ȥ��Ĵ����е��񣬶���Ϊ������̨�Ȱڵ�Ŀ�긽���ڽ���Ԥ���õģ�
		����ᵼ����̨�ζ���ԭ�������Ϊ��̨�ζ����Ԥ����ٶȴ���Ӱ�죬�Ӷ��γ��������������𵴹��̡� */ 
	}
	else
	{
		/*��ʱ�����󣬽���Ԥ�����*/
		float tmp1;	 /*��ʱ����*/
		
		if((target_accel_kf*target_speed_kf)>=0)
		{
			/*���ٶȺ��ٶ�ͬ��ʱ*/
			tmp1 = 0.8f; 
		}
		else
		{
			tmp1 = -1.2f;
		}
		
		/*���������жϼ��ٶȺ��ٶȹ�ϵ�����������ٶȵ����÷���
			
			���� ���ٶȺ��ٶȷ���ʱ��˵�������˶�
				���ٶȺ��ٶ�ͬ��ʱ��˵�������˶��� 
		*/ 
		
		feedforward_angle = k_ff * target_accel_kf; 	/*����ǰ����*/
		
		feedforward_angle = constrain(feedforward_angle,-4.f,4.f);	 /*ǰ����*/
		
		/*ǰ���Ƕ�ʱͨ�����ٶ�����Ŀ���˶�����ǰ��Ӧ��*/ 
		
		predict_angle_raw = k_level*(k_pre*target_speed_kf*vision_dis_kf + tmp1*feedforward_angle*use_ff);	/*����Ԥ��Ƕ�*/
	}
	predict_angle_raw = constrain(predict_angle_raw,-8*GIMBAL_GYRO_ANGLE_ZOOM_INDEX,8*GIMBAL_GYRO_ANGLE_ZOOM_INDEX);	/*�޷�*/
	
	predict_angle_degree = predict_angle_raw / GIMBAL_GYRO_ANGLE_ZOOM_INDEX;
	
	predict_angle = RampFloat(predict_angle_raw, predict_angle, (abs(predict_angle_raw - predict_angle)/predict_ramp_cnt)); 	/*б�´���*/
	
	/*�����Ƕ�Ԥ����˲�*/ 
	
	//attack_pitch_offset = k_pitch * vision_dis_kf; 	/*̧ͷ���벹��*/
	/*���ھ����pitch�Ƕ�̧ͷ������û�м����pitch�����Ԥ�⡣���������ϡ�*/ 
	
	
	/*�����ǿ�������Ҫ�ĽǶȼ������*/
	
	outline_cnt = xTaskGetTickCount() - vision_this_time; 	/*���߼���*/
	/*�����Ӿ����һ֡�ĸ���ʱ�䣬���ж��Ƿ��Ѿ�����*/ 

	if(outline_cnt >= 1000)
	{
		/*����*/
		VISION_ClearRxPacket();
		vision_online_flag = false;		
	}
	else
	{
		vision_online_flag=true;
	}
	/*�ж��Ƿ�����*/
	/*����Ҫ�ж�Ӧ�Ĵ���������̨���ߡ�*/ 	
	
	
	if(vision_this_time != vision_last_time)
	{
		/*����֡��*/
		vision_update_fps = vision_this_time - vision_last_time;
		
		vision_last_time = vision_this_time;
		
		/*����֡��*/ 
	}
	
	last_identify_flag = vision_identify_target;
	//��¼��һ�α�־λ	
	
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
		
		/*----Ԥ�⵽λ�ж�----*/
		if(predict_delay == 0 
				&& abs(vision_yaw_kf) < auto_yaw_predict_max 
				&& abs(target_speed_kf) > auto_yaw_speed_predict_low
				&& abs(target_speed_kf) < auto_yaw_speed_predict_high)
		{			
			/* Ŀ�����ƶ����Ӿ�������Ŀ�������(��ʾ�ѳ�ǰ) */
			if(target_speed_kf < 0 
				&& vision_yaw_kf > mobpre_yaw_boundary)
			{
				mobpre_yaw_left_delay = 0;	// ����Ԥ�⿪����ʱ����
				
				mobpre_yaw_right_delay++;	// �ۼ������ƶ�Ԥ��
				if(mobpre_yaw_right_delay > 0) {
					Mobi_Pre_Yaw_Fire = true;
				} else {
					Mobi_Pre_Yaw_Fire = false;
				}
			}
			/* Ŀ�����ƶ����Ӿ�������Ŀ�����ұ�(��ʾ�ѳ�ǰ) */
			else if(target_speed_kf > 0
				&& vision_yaw_kf < -mobpre_yaw_boundary)
			{
				mobpre_yaw_right_delay = 0;	// ����Ԥ�⿪����ʱ����
				
				mobpre_yaw_left_delay++;		// �ۼ������ƶ�Ԥ��
				if(mobpre_yaw_left_delay > 0) {
					Mobi_Pre_Yaw_Fire = true;
				} else {
					Mobi_Pre_Yaw_Fire = false;
				}		
			}
			/* Ԥ��δ��λ */
			else {
				Mobi_Pre_Yaw_Fire = false;
				
				mobpre_yaw_left_delay = 0;
				mobpre_yaw_right_delay = 0;
			}
			
			Mobi_Pre_Yaw = true;	// ����ѿ����ƶ�Ԥ��
			mobpre_yaw_stop_delay = 0;	// ���þ�ֹʱ�Ŀ����ӳ�
		
		}
		/* δ�ﵽԤ�⿪������ */
		else {
			pid[GYRO][YAW_205].Angle.target = target_yaw_kf;//RampFloat(gimbal->Auto.Yaw.target, pid[GYRO][YAW_205].Angle.feedback, auto_yaw_ramp);
			
			Mobi_Pre_Yaw = false;	// ���δ�����ƶ�Ԥ��
			mobpre_yaw_left_delay = 0;
			mobpre_yaw_right_delay = 0;
			
			if(abs(vision_yaw_kf) < stoppre_yaw_boundary) 
			{
				mobpre_yaw_stop_delay++;
				if(mobpre_yaw_stop_delay > 25) {// �ȶ�50ms
					Mobi_Pre_Yaw_Fire = true;
				} else {
					Mobi_Pre_Yaw_Fire = false;
				}
			}
		}		
	}
	else
	{
		mobpre_yaw_left_delay = 0;		// ������Ԥ�⿪���ӳ�
		mobpre_yaw_right_delay = 0;		// ������Ԥ�⿪���ӳ�
		mobpre_yaw_stop_delay = 0;		// ����ֹͣԤ�⿪���ӳ�
		
		KEY_SetGimbalAngle();		
	}
}
#endif

/**
 *	@brief	�����ٶȺ�Ԥ��Ƕ�
 */
void GIMBAL_CalPredictInfo(void)
{
//	/* ������ģʽ��Ҳ���� */
//	if(!GIMBAL_IfAutoMode()) {
//		/* ���¶��׿������ٶ��������ֵ */
//		yaw_angle_speed = speed_calculate(&yaw_angle_speed_struct, xTaskGetTickCount(), Gimbal_PID[GYRO][YAW_205].Angle.feedback);	
//		pitch_angle_speed = speed_calculate(&pitch_angle_speed_struct, xTaskGetTickCount(), Gimbal_PID[GYRO][PITCH_206].Angle.feedback);

//		/* �ԽǶȺ��ٶȽ��ж��׿������˲��ں�,0λ��,1�ٶ� */
//		yaw_kf_result = kalman_filter_calc(&yaw_kalman_filter, Gimbal_PID[GYRO][YAW_205].Angle.feedback, 0);		// ʶ�𲻵�ʱ��ΪĿ���ٶ�Ϊ0
//		pitch_kf_result = kalman_filter_calc(&pitch_kalman_filter, Gimbal_PID[GYRO][PITCH_206].Angle.feedback, 0);	// ʶ�𲻵�ʱ��ΪĿ���ٶ�Ϊ0

//		js_yaw_speed = yaw_kf_result[KF_SPEED];
//		js_yaw_angle = yaw_kf_result[KF_ANGLE];
//	}

	
	if( !GIMBAL_IfAutoMode() ) 
	{
		now_gyro_yaw = Gimbal_PID[GYRO][YAW_205].Angle.feedback; /*��ǰ��yaw�������ǽǶ�ֵ*/
		/*�����pid����̨Yaw�������ǵĽṹ�壬������Ҫ��Ϊ�˸���yaw����̨����������ǽǶ�*/ 

		delta_gyro_yaw += now_gyro_yaw - last_gyro_yaw; 	/*�������β����������ǽǶȲ�ֵ*/
		/*�����Ǽ������β����������ǽǶ�֮��Ĳ�ֵ*/ 
		
		cloud_yaw_raw = delta_gyro_yaw;  /*���뿨�����˲�*/
		/*������Ϊ�˽���ֵ���п������˲�*/
		 
		last_gyro_yaw = now_gyro_yaw; /*��¼��һ�εĽǶ�ֵ*/
		/*���¼�¼��һ�β����������ǽǶ�*/ 

		cloud_yaw_kf = KalmanFilter(&kalman_cloudYaw,cloud_yaw_raw)+start_gyro_yaw;	/*�˲�*/       
		
		cloud_degree = cloud_yaw_kf / GIMBAL_GYRO_ANGLE_ZOOM_INDEX;  
		/*�����ǽ�ǰ�����β����Ĳ�ֵ���п������˲����ټ�����ʼ�Ƕȣ�����Ϊ�˲������̨�����ǽǶ�*/ 
		
		target_speed_raw = Get_Target_Speed(queue_speed_length,now_cal_yaw)*k_speed; 	/*�������Ŀ���ٶȵ�Ԥ��*/
		/*��ʼ��Ŀ����ٶȽ���Ԥ�⣬�������ֱ�Ӳο��Ҹ��Ĵ��롣k_speed������ϵ������Ϊ�õ����ٶ����ݱȽ�С����˷Ŵ��ŵ��������ȽϺ���*/ 
		
		target_speed_kf = KalmanFilter(&kalman_speedYaw,0);			
		
		/*�ο�ȥ���������*/ 
		
		
		target_accel_raw = Get_Target_Accel(queue_accel_length,target_speed_kf);	 /*��ȡ���ٶ�*/
		
		target_accel_raw = myDeathZoom(0,0.1,target_accel_raw);		/*�������� - �˳�0�㸽��������*/
		
		target_accel_kf = KalmanFilter(&kalman_accel,target_accel_raw);		/*�������˲�*/

		/*���������ǽ�����ٶȵ����ݣ����ٶȵĽ���ԭ��һ��������������*/		
		
	}
}

/**
 *	@brief	���ģʽPID����
 *	@note	�췽����������������
 *			�����׶Σ�
 *			��С��
 *				��飺5:59~4:00������������ɹ�����1.5������������1����
 *				�˶����ԣ�10RPM
 *			�ڴ��
 *				��飺2:59~0:00������������ɹ�����2��������50%�����ӳɣ�����1����
 *			
 *			������Ϣ��
 *			���������1��2,��糵�������2��283,�ھ�70cm,�⾶80cm
 *			���������ؼ����
 *				�������ؿɼ���״̬ʱ������������ռ�첢ͣ��3s��ǹ������ÿ����ȴֵ��Ϊ5��
 *				2.5sδ����/��������װ�װ� -������ʧ��
 *
 *			����˼·��
 *			������ģʽ��ʱ����̨б�¹���(���Ӿ�����������ԭ��Ϊ����)
 *
 *			# �ڼҲ���������
 *			ͶӰֱ�߾��룺7.40m
 *			������ľ�����棺1.55m
 *			ǹ�ܾ��������ģ�7.56m(���ɶ���)
 *	
 *			# ����ͷ����̨������ͷ�ڵ��̵Ŀɲο�������룩
 *
 *			# ���ݵ�ͼ�Ĳ��ͼ�����������ؼ���������7.50m
 */
float buff_yaw_ramp;
float buff_pitch_ramp;
void GIMBAL_BUFF_PidCalc(Gimbal_PID_t pid[GIMBAL_MODE_COUNT][GIMBAL_MOTOR_COUNT], Gimbal_Info_t *gimbal)
{
	static float yaw_gyro_mid, yaw_mech_mid;
	static float pitch_gyro_mid, pitch_mech_mid;
	
	static uint16_t into_buff_time = 0;
	static uint16_t lost_cnt = 0;
	
	/* ������֡�� */
	gimbal->Buff.Time[NOW] = xTaskGetTickCount();
	gimbal->Buff.Time[DELTA] = gimbal->Auto.Time[NOW] - gimbal->Auto.Time[PREV];

//	/* �л�pitch������ */
//	if((GIMBAL_MECH_PITCH_DOWN_ANGLE - pid[GYRO][PITCH_206].Angle.feedback) > 0)
//		GIMBAL_BUFF_PITCH_COMPENSATION = BUFF_PITCH_MODIFY_TABLE[(uint8_t)((GIMBAL_MECH_PITCH_DOWN_ANGLE - pid[GYRO][PITCH_206].Angle.feedback)/100)];
//	else
//		GIMBAL_BUFF_PITCH_COMPENSATION = 0;
	
	if(gimbal->Buff.FLAG_first_into_buff == true) {
		gimbal->Buff.FLAG_first_into_buff = false;
		into_buff_time = 0;
		/* ��¼�ս�����ģʽʱ�������ǽǶ� */
		yaw_gyro_mid = pid[GYRO][YAW_205].Angle.feedback;
		pitch_gyro_mid = pid[GYRO][PITCH_206].Angle.feedback;
		/* ��¼�ս�����ģʽʱ�Ļ�е�Ƕ� */
		yaw_mech_mid = pid[MECH][YAW_205].Angle.feedback;
		pitch_mech_mid = pid[MECH][PITCH_206].Angle.feedback;
	} else {
		/* ��ֹ�ӵ�̫�� */
		if(into_buff_time < 250)	
			into_buff_time++;
	}
	
	/* ���ݸ��� */
	if(VISION_GetFlagStatus(VISION_FLAG_DATA_UPDATE) == true) {
		/* ������ݸ��±�־λ����ֹδ���յ������ݵ�������ظ����� */
		VISION_ClearFlagStatus(VISION_FLAG_DATA_UPDATE);	
		if(VISION_GetFlagStatus(VISION_FLAG_LOCK_BUFF) == true) {	// ʶ��Ŀ����������
			lost_cnt = 0;
			if(into_buff_time > 100) {
				/* Yaw ������ */
				gimbal->Buff.Yaw.erro = gimbal->Buff.Yaw.kp * VISION_BUFF_GetYawFeedback() + GIMBAL_BUFF_YAW_COMPENSATION;
				gimbal->Buff.Yaw.target = pid[GYRO][YAW_205].Angle.feedback + gimbal->Buff.Yaw.erro;
				/* Pitch ������ */
				gimbal->Buff.Pitch.erro = (gimbal->Buff.Pitch.kp * VISION_BUFF_GetPitchFeedback()) + GIMBAL_BUFF_PITCH_COMPENSATION;
				gimbal->Buff.Pitch.target = pid[GYRO][PITCH_206].Angle.feedback + gimbal->Buff.Pitch.erro;
				/*----�����޸�----*/
				pid[GYRO][YAW_205].Angle.target = RampFloat(gimbal->Buff.Yaw.target, pid[GYRO][YAW_205].Angle.feedback, GIMBAL_BUFF_YAW_RAMP);
				pid[GYRO][PITCH_206].Angle.target = RampFloat(gimbal->Buff.Pitch.target, pid[GYRO][PITCH_206].Angle.feedback, GIMBAL_BUFF_PITCH_RAMP);				
			} else {
				/*----�����޸�----*/
				pid[GYRO][YAW_205].Angle.target = yaw_gyro_mid;
				pid[GYRO][PITCH_206].Angle.target = pitch_gyro_mid;
			}
		} else {	// δʶ��Ŀ���򱣳�ԭλ��
			gimbal->Buff.Yaw.erro = 0;
			gimbal->Buff.Pitch.erro = 0;
			if(lost_cnt > 50) {	// ����50֡��ʧ
				if(lost_cnt == 51) {	// ֻ��ֵһ��
					/*----�����޸�----*/
					pid[GYRO][YAW_205].Angle.target = pid[GYRO][YAW_205].Angle.feedback;
					pid[GYRO][YAW_205].Angle.target = GIMBAL_GYRO_YawBoundaryProc(&Gimbal_PID[GYRO][YAW_205], 
													((yaw_mech_mid - pid[MECH][YAW_205].Angle.feedback)*360/8192.f));
					pid[GYRO][PITCH_206].Angle.target = pitch_mech_mid;	// Pitch���û�еģʽ
					lost_cnt++;
				}
			} else {
				lost_cnt++;
			}
		}
	}
	
	/*----�����޸�----*/
	//..�Զ���
	
	gimbal->Buff.Time[PREV] = gimbal->Buff.Time[NOW];
}

/**
 *	@brief	��̨�������
 */
void GIMBAL_NormalCtrl(void)
{
	KEY_SetGimbalAngle();
	KEY_SetGimbalTurn();
	KEY_SetQuickPickUp();	
}

/**
 *	@brief	��̨�������
 */
void GIMBAL_AutoCtrl(void)
{
	/* �Ӿ����ݿ��� && ����ģʽ�� */
	if( VISION_IsDataValid() ) 
	{
		/*----�����޸�----*/
		/* �Ӿ�Ԥ��� */
		//GIMBAL_VISION_AUTO_PidCalc(Gimbal_PID, &Gimbal);
		/* ���Ԥ��� */
		GIMBAL_AUTO_PidCalc(Gimbal_PID, &Gimbal);
	}	
}

/**
 *	@brief	��̨�������
 *	@note	
 *	1.�Ȱ�һ��װ�׳����ŵ������棬�����⣬��offsetx��offsety�Ѻ���㵽װ�װ����ġ�
 *
 *	2.��ת���װ�װ壬����ĺ����ǲ���һ��������Բ������˵��������ľ��������ʱ�򣬲���װ�װ���ת�Ǳߣ������һֱ���š�
 *
 *	3.��̧ͷ������
 *
 *	4.���ʱ����Ϊ������ǹ�ܾ�����ƽ�⣬���Դ򵯣���offsetx��offsety�������С�
 *
 *	5.����λ�����Ŵ�
 *
 *	6.����ת��
 *	
 *	7.pid�����ĵã�pitch��yaw��Ҫ�ر�Ӳ�����ԼӺܴ�Ļ���ki��ע�͵������������Ĳ������ȶ�һ�㣩
 */
void GIMBAL_BuffCtrl(void)
{
	/* ����WSADQEV(���ⷽ���)���˳����ģʽ */
	if((IF_KEY_PRESSED_W || IF_KEY_PRESSED_S || IF_KEY_PRESSED_A || IF_KEY_PRESSED_D
		|| IF_KEY_PRESSED_Q || IF_KEY_PRESSED_E || IF_KEY_PRESSED_V)&&(!IF_KEY_PRESSED_CTRL)) 
	{
		Gimbal.State.mode = GIMBAL_MODE_NORMAL;	// ��������ģʽ
		Gimbal.State.pid_mode = GYRO;		// ����������ģʽ
		
		Gimbal_PID[GYRO][YAW_205].Angle.target = Gimbal_PID[GYRO][YAW_205].Angle.feedback;
		Gimbal_PID[GYRO][PITCH_206].Angle.target = Gimbal_PID[GYRO][PITCH_206].Angle.feedback;	// GIMBAL_GYRO_PITCH_MID_ANGLE

		/* ����ģʽ���� */
		REVOLVER_SetAction(SHOOT_NORMAL);		// ��ɳ������ģʽ
		/* �Ӿ�ģʽ���� */
		VISION_SetMode(VISION_MODE_MANUAL);	// �ֶ�ģʽ
		/* ����ģʽ���� */
		CHASSIS_SetMode(CHAS_MODE_NORMAL);
		return;
	}
	
	/* �Ӿ����ݿ��� && ����ģʽ�� */
	if( VISION_IsDataValid() ) 
	{
		/*----�����޸�----*/
		GIMBAL_BUFF_PidCalc(Gimbal_PID, &Gimbal);	
	}
}

/**
 *	@brief	��̨�Զ�����
 *	@note		
 */
void GIMBAL_ReloadBulletCtrl(void)
{
//	/* ����WSADQEV(���ⷽ���)���˳��Զ�����ģʽ */
//	if((IF_KEY_PRESSED_W || IF_KEY_PRESSED_S || IF_KEY_PRESSED_A || IF_KEY_PRESSED_D
//		|| IF_KEY_PRESSED_Q || IF_KEY_PRESSED_E || IF_KEY_PRESSED_V))
//	{
//		GIMBAL_SetMode(GIMBAL_MODE_NORMAL);
//		Gimbal.State.pid_mode = GYRO;
//		GIMBAL_keyMech_To_keyGyro();
//		return;
//	}
//	
	/* ͷ�����м� */
	Gimbal_PID[Gimbal.State.pid_mode][PITCH_206].Angle.target = GIMBAL_MECH_PITCH_MID_ANGLE;
}

/* #�����# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	��̨��Ϣ��ȡ����
 */
void GIMBAL_GetInfo(void)
{
	// ��ȡϵͳ��Ϣ
	GIMBAL_GetSysInfo(&System, &Gimbal);
	// ��ȡ����ϵͳ��Ϣ
	GIMBAL_GetJudgeInfo(&Judge, &Gimbal);
	// ��ȡIMU��Ϣ
	GIMBAL_GetImuInfo(Gimbal_PID);	
	// ��������Ԥ����Ϣ
	GIMBAL_CalPredictInfo();
}

/**
 *	@brief	pid�������������
 */
uint8_t test_yaw_pid = 0;
uint8_t test_pitch_pid = 0;
float   test_yaw_speed_max_target = 8000;
float   test_pitch_speed_max_target = 4000;
void GIMBAL_PidCtrlTask(void)
{
	/* YAW �ǶȻ� */
	GIMBAL_Angle_PidCalc(Gimbal_PID[Gimbal.State.pid_mode], YAW_205);
	/* YAW �ٶȻ� */
	if(test_yaw_pid == 0) 
	{
		Gimbal_PID[Gimbal.State.pid_mode][YAW_205].Speed.target = Gimbal_PID[Gimbal.State.pid_mode][YAW_205].Angle.out;
	} 
	else 
	{
		Gimbal_PID[Gimbal.State.pid_mode][YAW_205].Speed.target = (Remote.rc.ch0 - 1024)/660.f * test_yaw_speed_max_target;
	}
	GIMBAL_Speed_PidCalc(Gimbal_PID[Gimbal.State.pid_mode], YAW_205);
	
	/* PITCH �ǶȻ� */
	GIMBAL_Angle_PidCalc(Gimbal_PID[Gimbal.State.pid_mode], PITCH_206);
	/* PITCH �ٶȻ� */
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
 *	@brief	ң�ؿ�����̨
 */
void GIMBAL_RcCtrlTask(void)
{
	REMOTE_SetGimbalAngle();	
	REMOTE_SetTopGyro();
}

/**
 *	@brief	���̿�����̨
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
 *	@brief	��̨ʧ�ر���
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
 *	@brief	��̨����
 */
void GIMBAL_Ctrl(void)
{
	/*----��Ϣ����----*/
	GIMBAL_GetInfo();
	/*----�����޸�----*/
	if(BM_IfSet(BitMask.System.BM_Reset, BM_RESET_GIMBAL)) {	// ��λ״̬
		GIMBAL_Reset(); // ��̨��λ
	} 
	else {
		if(Gimbal.State.remote_mode == RC) {
			GIMBAL_RcCtrlTask();
		} 
		else if(Gimbal.State.remote_mode == KEY) {
			GIMBAL_KeyCtrlTask();
		}
	}

	/* ������̨ģʽ�л�PID���� */
	GIMBAL_PidParamsSwitch(Gimbal_PID, &Gimbal);

	/*----�������----*/
	GIMBAL_PidCtrlTask();
}
