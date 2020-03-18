/**
 * @file        Task_Chassis.c
 * @author      MaWeiming
 * @Version     V1.0
 * @date        30-September-2019
 * @brief       This file includes the Chassis(����) external functions 
 *
 * @Verison			V1.1 (1-October-2019)
 */

/**
 *	PID���οھ�
 *	kp�� -> ��Ӧ�� -> ����
 *	ki�� -> �������� -> 
 *	�ο���ַ��https://www.jianshu.com/p/4b0fa85cd353
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

/* 19���������� */
#define RADIUS     76      //���ְ뾶
#define PERIMETER  478     //�����ܳ�
#define WHEELTRACK 360//398     //�����־�
#define WHEELBASE  300     //ǰ�����
#define GIMBAL_X_OFFSET 0//75  //��̨��Ե������ĵ�ǰ��ƫ����
#define GIMBAL_Y_OFFSET 0 //��̨��Ե������ĵ�����ƫ����
#define CHASSIS_DECELE_RATIO (1.0f/19.0f)  //���������
#define RADIAN_COEF 57.3f  

/* ��ͬģʽ�µ�������ٶ� */
#define STANDARD_MAX_NORMAL		CHASSIS_PID_OUT_MAX
#define SPIN_MAX_NORMAL			CHASSIS_PID_OUT_MAX

#define STANDARD_MAX_SZUPUP		3000
#define SPIN_MAX_SZUPUP			9000

/* ��ͬģʽ�µ���б�� */
#define TIME_INC_NORMAL	2
#define TIME_DEC_NORMAL	500

#define TIME_INC_SZUPUP	3

#define SPIN_ANGLE	(35)

/* Private variables ---------------------------------------------------------*/
/* �������˲��� */
extKalman_t Chassis_Kalman_Error;

/* �޷� */
float Chas_Spin_Move_Max = CHASSIS_PID_OUT_MAX;			// ������ת-����
float Chas_Standard_Move_Max = CHASSIS_PID_OUT_MAX;	// ����ƽ��-����

float Chas_Top_Spin_Max = CHASSIS_PID_OUT_MAX;
float Chas_Top_Move_Max = CHASSIS_PID_OUT_MAX;

float Chas_Top_Spin_scale = 0.8;	// С������ת���ܷ�����ٶȱ���
float Chas_Top_Move_scale = 0.2;	// С�����н����ܷ�����ٶȱ���

/* б�� */
uint16_t Time_Inc_Normal;			// ����б��������
uint16_t Time_Inc_Saltation = 1;	// ǰ����ͻ��б��������

/* ����ģʽ�µ�ǰ������Լ���תб�±��� */
float	Chas_Slope_Move_Fron, Chas_Slope_Move_Back, Chas_Slope_Move_Left, Chas_Slope_Move_Righ;
float Chas_Slope_Spin_Move;

/* �����ٶ� */
float Chas_Target_Speed[4];

/* ҡ�������� */
//��еģʽ�µ��̱���ϵ��,����ҡ����Ӧ�ٶ�,�����СҲ���������ת��,max = ��ϵ�� *660
float kRc_Mech_Chas_Standard, kRc_Mech_Chas_Spin;//ƽ�ƣ���ת,

//������ģʽ�µ��̱���ϵ��,����ҡ����Ӧ�ٶ�,�����СҲ���������ת��,max = ��ϵ�� *660
float kRc_Gyro_Chas_Standard, kRc_Gyro_Chas_Spin;//ƽ�ƣ���ת

//��еģʽ�µ��̱���ϵ��,���Ƽ���б�±仯��
float kKey_Mech_Chas_Standard, kKey_Mech_Chas_Spin;//ƽ�ƣ���ת

//������ģʽ�µ��̱���ϵ��,���Ƽ���б�±仯��
float kKey_Gyro_Chas_Standard, kKey_Gyro_Chas_Spin;//ƽ�ƣ���ת

//С����ģʽ�µ��̱���ϵ��
float k_Gyro_Chas_Top;

//Ť��ģʽ�µ��̱���ϵ��
float k_Gyro_Chas_Twist;

//С������תת�ٲ���
float Chas_Top_Gyro_Step;
uint16_t Chas_Top_Gyro_Period = 0;	// ����������(TΪ3~7s�������)

//Ť����ת����
float Chas_Twist_Step;

/* ## Global variables ## ----------------------------------------------------*/
/* #������# ---------------------------------------------------------------------------------------------------------------------------------------*/
float speed_target_203;
float speed_feedback_203;
float angle_target_203;
float angle_feedback_203;

/**
 *	@brief	����PID
 */
Chassis_PID_t Chassis_PID[CHASSIS_MOTOR_COUNT] = {
	{	// LEFT_FRON_201
		/* �ٶȻ� */
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
		/* λ�û� */
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
		/* �ٶȻ� */
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
		/* λ�û� */
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
		/* �ٶȻ� */
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
		/* λ�û� */
		.Angle.kp = 1.172,		// 1.15					1.16				1.172		(���ײ����ʺϴ�Ƕ�ת��)
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
		/* �ٶȻ� */
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
		/* λ�û� */
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
 *	@brief	����Z����Pid
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
 *	@brief	���̶�λPID
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
 *	@brief	������Ϣ�ṹ��
 */
Chassis_Info_t Chassis = {
	// ���Ʒ�ʽ
	.RemoteMode = RC,
	// Pidģʽ
	.PidMode = MECH,
	// ģʽ
	.Mode = CHAS_MODE_NORMAL,
	// ����
	.Power.max_limit = CHASSIS_MAX_CURRENT_LIMIT,
	.Power.cur_limit = 0,
	// �߼�
	.Logic = CHAS_LOGIC_NORMAL,
	// С���ݿ�����־λ
	.TopGyro = false,
	// Ť��������־λ
	.Twist = false,
};

Chassis_Handler_t hChassis = {
	/* ��� �ٶȻ�+�ǶȻ� */
	.Pid = {
		{	// LEFT_FRON_201
			/* �ٶȻ� */
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
			/* λ�û� */
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
			/* �ٶȻ� */
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
			/* λ�û� */
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
			/* �ٶȻ� */
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
			/* λ�û� */
			.Angle.kp = 1.172,		// 1.15					1.16				1.172		(���ײ����ʺϴ�Ƕ�ת��)
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
			/* �ٶȻ� */
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
			/* λ�û� */
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
	
	/* Z�ǶȻ� */
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
	
	/* ��λPid */
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

/* #Ӧ�ò�# ---------------------------------------------------------------------------------------------------------------------------------------*/

/* #�����# ---------------------------------------------------------------------------------------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* API functions -------------------------------------------------------------*/
/* #������# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	���̲�����ʼ��
 */
void CHASSIS_InitParams(void)
{
	/* ƽ��ҡ�������� */
	kRc_Mech_Chas_Standard = 14;	// 14*660 = 9240 -> 9000
	kRc_Gyro_Chas_Standard = 14;	// 14*660 = 9240 -> 9000
	
	/* ��תҡ�������� */
	kRc_Mech_Chas_Spin = 11.4;	// 11.4*660 = 7524
	kKey_Mech_Chas_Spin = 40;		// ���̻�еģʽŤͷ��Ӧ���� #δ��
	
	/* ������ģʽ����ϵ�� */
	kRc_Gyro_Chas_Spin = -4.8;	// (����)����
	kKey_Gyro_Chas_Spin = kRc_Gyro_Chas_Spin;
	
	/* С���ݵ���ϵ�� */
	k_Gyro_Chas_Top = -4.8;
	
	/* Ť������ϵ�� */
	k_Gyro_Chas_Twist = -4.8;
	
	/* С����ת�ٲ��� */
	Chas_Top_Gyro_Step = 10;
	
	/* Ť����ת���� */
	Chas_Twist_Step = 4;
}

/**
 *	@brief	���̵��PID������ʼ��
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
 *	@brief	����������ģʽ��������
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
 *	@brief	����������ģʽ�������˲�����ʼ��
 */
void CHASSIS_KalmanCreate(void)
{
	/* �ٶ���е�Ƕȵķ���û��������� */
	KalmanCreate(&Chassis_Kalman_Error, 1, 0);
}	

/**
 *	@brief	���̵������ʵʱ�ٶ�
 */
void CHASSIS_UpdateMotorSpeed(Chassis_PID_t *pid, Chassis_Motor_Names MOTORx)
{
	pid[MOTORx].Speed.feedback = g_Chassis_Motor_Info[MOTORx].speed;
}

/**
 *	@brief	���̵������ʵʱ�Ƕ�(�ۻ�ת���ĽǶ�)
 */
void CHASSIS_UpdateMotorAngle(Chassis_PID_t *pid, Chassis_Motor_Names MOTORx)
{
  //pid[MOTORx].Angle.feedback = g_Chassis_Motor_Info[MOTORx].angle;
	pid[MOTORx].Angle.feedback = g_Chassis_Motor_Info[MOTORx].angle_sum;
}

/**
 *	@brief	���̵������ʵʱ����ֵ
 */
void CHASSIS_UpdateMotorCurrent(Chassis_PID_t *pid, Chassis_Motor_Names MOTORx)
{
	//pid[MOTORx].Speed.feedback = g_Chassis_Motor_Info[MOTORx].speed;
}

/**
 *	@brief	���̵������ɲ��
 */
void CHASSIS_Stop(Chassis_PID_t *pid)
{
	static int16_t pid_out[4] = {0, 0, 0, 0};
	
	/* �ڻ��ٶȻ�������� */
	pid[LEFT_FRON_201].Out = 0;
	pid[RIGH_FRON_202].Out = 0;
	pid[LEFT_BACK_203].Out = 0;
	pid[RIGH_BACK_204].Out = 0;
	
	CAN1_Send(0x200, pid_out);	
}

/**
 *	@brief	���̵���ٶȻ�
 */
void CHASSIS_Speed_PidCalc(Chassis_PID_t *pid, Chassis_Motor_Names MOTORx)
{
	pid[MOTORx].Speed.erro = pid[MOTORx].Speed.target - pid[MOTORx].Speed.feedback;
	pid[MOTORx].Speed.integrate += pid[MOTORx].Speed.erro;
	
	//pid[MOTORx].Speed.integrate = constrain(pid[MOTORx].Speed.integrate, -3000, 3000);
	// #integrate�Ƿ��޷�?
	
	pid[MOTORx].Speed.integrate = constrain(pid[MOTORx].Speed.integrate, -pid[MOTORx].Speed.integrate_max, pid[MOTORx].Speed.integrate_max);
	
	/* Pout */
	pid[MOTORx].Speed.pout = pid[MOTORx].Speed.kp * pid[MOTORx].Speed.erro;
	/* Iout */
	pid[MOTORx].Speed.iout = pid[MOTORx].Speed.ki * pid[MOTORx].Speed.integrate * 0.002f; // ģ��19��������(0.002f��2ms����˼)
	/* Iout Limits */
	pid[MOTORx].Speed.iout = constrain(pid[MOTORx].Speed.iout, -CHASSIS_SPEED_IOUT_MAX, CHASSIS_SPEED_IOUT_MAX);
	/* Dout*/
	pid[MOTORx].Speed.dout = pid[MOTORx].Speed.kd * (pid[MOTORx].Speed.erro - pid[MOTORx].Speed.last_erro)/0.002f;
	/* Record Last Error */
	pid[MOTORx].Speed.last_erro = pid[MOTORx].Speed.erro;
	
	/* 19�������� - ���������С����ֹ���Ϊ0ʱͻȻʧ�� */
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
// *	@brief	���̵���ٶȻ�
// */
//void CHASSIS_Speed_PidCalcTest(Chassis_PID_t *pid, Chassis_Motor_Names MOTORx)
//{
//	pid[MOTORx].Speed.Err = pid[MOTORx].Speed.Target - pid[MOTORx].Speed.Feedback;
//	pid[MOTORx].Speed.Integrate += pid[MOTORx].Speed.Err;
//	
//	//pid[MOTORx].Speed.integrate = constrain(pid[MOTORx].Speed.integrate, -3000, 3000);
//	// #integrate�Ƿ��޷�?
//	
//	pid[MOTORx].Speed.Integrate = constrain(pid[MOTORx].Speed.Integrate, -18000, 18000);
//	
//	/* Pout */
//	pid[MOTORx].Speed.Pout = pid[MOTORx].Speed.Kp * pid[MOTORx].Speed.Err;
//	/* Iout */
//	pid[MOTORx].Speed.Iout = pid[MOTORx].Speed.Ki * pid[MOTORx].Speed.Integrate * 0.002f; // ģ��19��������(0.002f��2ms����˼)
//	/* Iout Limits */
//	pid[MOTORx].Speed.Iout = constrain(pid[MOTORx].Speed.Iout, -CHASSIS_SPEED_IOUT_MAX, CHASSIS_SPEED_IOUT_MAX);
//	/* Dout*/
//	pid[MOTORx].Speed.Dout = pid[MOTORx].Speed.Kd * (pid[MOTORx].Speed.Err - pid[MOTORx].Speed.LastErr)/0.002f;
//	/* Record Last Error */
//	pid[MOTORx].Speed.LastErr = pid[MOTORx].Speed.Err;
//	
//	/* 19�������� - ���������С����ֹ���Ϊ0ʱͻȻʧ�� */
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
 *	@brief	���̵����λ�û�
 *	@note
 *			����PID
 *			�ڻ� ����ֵ �������ٶ�(ʵ���Ͼ����ٶȻ�)
 *					 ���ֵ �����Ǽ��ٶ�
 *
 *			�⻷ ����ֵ �����Ƕ�
 *					 ���ֵ �������ٶ�
 */
void CHASSIS_Angle_PidCalc(Chassis_PID_t *pid, Chassis_Motor_Names MOTORx)
{
	pid[MOTORx].Angle.erro = pid[MOTORx].Angle.target - pid[MOTORx].Angle.feedback;
	pid[MOTORx].Angle.integrate += pid[MOTORx].Angle.erro;
	pid[MOTORx].Angle.integrate = constrain(pid[MOTORx].Angle.integrate, -pid[MOTORx].Angle.integrate_max, pid[MOTORx].Angle.integrate_max);
	
	/* Pout */
	pid[MOTORx].Angle.pout = pid[MOTORx].Angle.kp * pid[MOTORx].Angle.erro;
	/* Iout */
	pid[MOTORx].Angle.iout = pid[MOTORx].Angle.ki * pid[MOTORx].Angle.integrate * 0.002f; // ģ��19��������(0.002f��2ms����˼)
	/* Iout Limits */
	pid[MOTORx].Angle.iout = constrain(pid[MOTORx].Angle.iout, -CHASSIS_ANGLE_IOUT_MAX, CHASSIS_ANGLE_IOUT_MAX);
	/* Dout*/
	pid[MOTORx].Angle.dout = pid[MOTORx].Angle.kd * (pid[MOTORx].Angle.erro - pid[MOTORx].Angle.last_erro)/0.002f;
	/* Record Last Error */
	pid[MOTORx].Angle.last_erro = pid[MOTORx].Angle.erro;
	
//	/* 19�������� - ���������С����ֹ���Ϊ0ʱͻȻʧ�� */
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
 *	@brief	����������ģʽ����Z������ٶȻ�(ʵ������λ�û�)
 *	@note
 *			����ǰYAW��е�Ǻ���ֵYAW��е�ǵĲ�ֵ��Ϊ����ͽ� Z_PID��������
 *			���������Ľ��ٶ� tar_spd_z
 */
float CHASSIS_Z_Angle_PidCalc(PID_Object_t *pid, float kp)
{
	pid->kp = kp;
	pid->erro = pid->target - pid->feedback;
	
	/* �ٽ紦�� */
	
	/*
	
		0 - 8191 

		���=����-����
	
		������4096
	
		������������� -> ȷ�������ķ��� 0->8191 --�� ������������
	
		ʵ����� = -��8191 - ���� + ������ = -8191 + ���
	
		���С��-4096
	
		������������� -> ȷ���������� 8191->0  --�� �������ڷ���
	
		ʵ����� = 8191 + ���� -���� = 8191 + (���� - ����) = 8191 + ��� 
	*/
	
	
	if(pid->erro >= 4096) {
		pid->erro = -8192 + pid->erro;
	} 
	else if(pid->erro <= -4096) {
		pid->erro = 8192 + pid->erro;
	}
	
	pid->erro = KalmanFilter(&Chassis_Kalman_Error, pid->erro);
	
	/* ���̴�����ͨ����ģʽ */
	if(CHASSIS_IfTopGyroOpen() == false) {
		// ���������㷨
		if(abs(pid->erro) < 15) {
			pid->erro = 0;
		}
	}
	/* ���̴���С����ģʽ */
	else {
		//..������������
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
 *	@brief	�Զ���λģʽ�µ�PID����
 */
float CHASSIS_Locate_PidCalc(PID_Object_t* pid)
{
	pid->erro = pid->target - pid->feedback;
	
	pid->integrate += pid->erro;
	
	pid->integrate = constrain(pid->integrate, -pid->integrate_max, pid->integrate_max);
	
	pid->pout = pid->kp * pid->erro;
	
	pid->iout = pid->ki * pid->integrate * 0.002f;	// 2ms��������
	
	pid->dout = pid->kd * (pid->erro - pid->last_erro) / 0.002f;	// 2ms��������
	
	pid->last_erro = pid->erro;
	
	pid->out = pid->pout + pid->iout + pid->dout;
	
	pid->out = constrain(pid->out, -pid->out_max, pid->out_max);
	
	return pid->out;
}

/**
 *	@brief	�����ۼӽǶ�����
 */
void CHASSIS_Angle_ClearSum(Chassis_PID_t *pid)
{
	/* �ۼӻ�е�Ƕ�ֵ���� */
	g_Chassis_Motor_Info[LEFT_FRON_201].angle_sum = 0;
	g_Chassis_Motor_Info[RIGH_FRON_202].angle_sum = 0;
	g_Chassis_Motor_Info[LEFT_BACK_203].angle_sum = 0;
	g_Chassis_Motor_Info[RIGH_BACK_204].angle_sum = 0;

	/* ���̽ǶȻ�PID�������� */
	pid[LEFT_FRON_201].Angle.feedback = 0;
	pid[RIGH_FRON_202].Angle.feedback = 0;
	pid[LEFT_BACK_203].Angle.feedback = 0;
	pid[RIGH_BACK_204].Angle.feedback = 0;	
}

/**
 *	@brief	���̵��PID���������
 */
void CHASSIS_PidOut(Chassis_PID_t *pid)
{
	int16_t pidOut[4];
	
	if(BitMask.Chassis.BM_rxReport & BM_RX_REPORT_201) {
		pidOut[LEFT_FRON_201] = (int16_t)pid[LEFT_FRON_201].Out;
	} else {
		pidOut[LEFT_FRON_201] = 0;	// ʧ����ж��
	}
	
	if(BitMask.Chassis.BM_rxReport & BM_RX_REPORT_202) {
		pidOut[RIGH_FRON_202] = (int16_t)pid[RIGH_FRON_202].Out;
	} else {
		pidOut[LEFT_FRON_201] = 0;	// ʧ����ж��
	}
	
	if(BitMask.Chassis.BM_rxReport & BM_RX_REPORT_203) {
		pidOut[LEFT_BACK_203] = (int16_t)pid[LEFT_BACK_203].Out;
	} else {
		pidOut[LEFT_BACK_203] = 0;	// ʧ����ж��
	}
	
	if(BitMask.Chassis.BM_rxReport & BM_RX_REPORT_204) {
		pidOut[RIGH_BACK_204] = (int16_t)pid[RIGH_BACK_204].Out;
	} else {
		pidOut[RIGH_BACK_204] = 0;	// ʧ����ж��
	}
	
	CAN1_QueueSend(0x200, pidOut);
}

/* #��Ϣ��# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**	
 *	@breif	��������ģʽ
 */
Chassis_Mode_Names_t CHASSIS_GetMode(void)
{
	return Chassis.Mode;
}

/**	
 *	@breif	���������߼�
 */
Chassis_Logic_Names_t CHASSIS_GetLogic(void)
{
	return Chassis.Logic;
}

/**	
 *	@breif	�������̻�е��ֵĿ��
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
 *	@brief	���õ��̵�ģʽ
 */
void CHASSIS_SetMode(Chassis_Mode_Names_t mode)
{
	Chassis.Mode = mode;
}

/**
 *	@brief	�����߼�ȡ��
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
 *	@brief	�����Ƿ��ڻ�еģʽ
 */
bool CHASSIS_IfMechMode(void)
{
	if(Chassis.PidMode == MECH)
		return true;
	else
		return false;
}

/**
 *	@brief	�����Ƿ���������ģʽ
 */
bool CHASSIS_IfGyroMode(void)
{
	if(Chassis.PidMode == GYRO)
		return true;
	else
		return false;
}

/**
 *	@brief	�����Ƿ��ڳ���ģʽ
 */
bool CHASSIS_IfNormalMode(void)
{
	if(Chassis.Mode == CHAS_MODE_NORMAL)
		return true;
	else 
		return false;
}

/**
 * @brief		�����Ƿ���С����ģʽ
 */
bool CHASSIS_IfTopGyroOpen(void)
{
	if(Chassis.PidMode == GYRO
			&& Chassis.TopGyro == true)
		return true;

	return false;
}

/**
 *	@brief	�����Ƿ���Ť��ģʽ
 */
bool CHASSIS_IfTwistOpen(void)
{
	if(Chassis.PidMode == GYRO
			&& Chassis.Twist == true)
		return true;
			
	return false;
}

/**	
 *	@breif	�����߼��Ƿ�ȡ��
 */
bool CHASSIS_IfLogicRevert(void)
{
	static Chassis_Logic_Names_t prev_logic = CHAS_LOGIC_NORMAL;
	
	/* �����߼�ȡ����� */
	if(prev_logic != Chassis.Logic) {
		return true;
	}
	prev_logic = Chassis.Logic;
	
	return false;
}

/**
 *	@brief	���̻�ȡϵͳ��Ϣ
 */
void CHASSIS_GetSysInfo(System_t *sys, Chassis_Info_t *chas)
{
	/*----���Ʒ�ʽ----*/
	/* ���Ʒ�ʽ - ң���� */
	if(sys->RemoteMode == RC) {
		chas->RemoteMode = RC;
	} 
	/* ���Ʒ�ʽ - ���� */
	else if(sys->RemoteMode == KEY) {
		chas->RemoteMode = KEY;
	}
	
	/*----ģʽ�޸�----*/
	switch(sys->Action)
	{
		case SYS_ACT_NORMAL: 
			{
				// �ֶ����¿��ڳ���ģʽ�½���
				if(chas->Mode != CHAS_MODE_SZUPUP) {
					chas->Mode = CHAS_MODE_NORMAL;
				}
			}break;
		case SYS_ACT_AUTO: 
			{
				// ����ʱ��Ϊ����ģʽ
				if(chas->Mode != CHAS_MODE_NORMAL) {
				}
				chas->Mode = CHAS_MODE_NORMAL;
			}break;
		case SYS_ACT_BUFF:
			{
				// �ս�����ģʽ
				if(chas->Mode != CHAS_MODE_BUFF) {
				}
				chas->Mode = CHAS_MODE_BUFF;
			}break;
		case SYS_ACT_PARK:
			{
				// �ս����Զ���λģʽ
				if(chas->Mode != CHAS_MODE_RELOAD_BULLET) {
				}
				chas->Mode = CHAS_MODE_RELOAD_BULLET;
			}break;
	}
	
	/*----Pidģʽ----*/
	if(sys->PidMode != chas->PidMode)
	{
		/* GYRO -> MECH */
		if(sys->PidMode == MECH)
		{
			Chassis.TopGyro = false;	// ������
			Chassis.Twist = false;		// ��Ť��
		}
		/* MECH -> GYRO*/
		else if(sys->PidMode == GYRO)
		{
		}
	}
	chas->PidMode = sys->PidMode;
}

/**
 *	@brief	���̶�ȡ����ϵͳ��Ϣ
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
 *	@brief	���̶�ȡIMU����
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

	/* ����������������Ťͷ����(ʹ����̨����̶����˶�) */
	if(CHASSIS_IfTopGyroOpen() == true) {
		zpid->Angle.target = CHASSIS_MECH_YawBoundaryProc(&Chassis_Z_PID, 		
												yaw[DELTA] * 8192 / 360.f);
	}	
}

/**
 *	@brief	������С����
 */
void CHASSIS_GetTopGyroInfo(void)
{
	static portTickType ulCurrentTime;
	
	ulCurrentTime = xTaskGetTickCount();
	ulCurrentTime %= Chas_Top_Gyro_Period;
	
	if(CHASSIS_IfTopGyroOpen() == true) {
		// -7.5sin(2pi*t/T)+22.5
		Chas_Top_Gyro_Step = -7.5*sin(6.28f*(float)ulCurrentTime/Chas_Top_Gyro_Period) + 22.5;	// ����С����
	}
}

/**
 *	@brief	���̶�ȡң������
 */
static uint8_t RcLockFlag_Wheel = false;
static uint8_t RcUnlockFlag_Wheel = false;
static uint8_t KeyLockFlag_F = false;
static uint8_t KeyLockFlag_C = false;

void CHASSIS_GetRemoteInfo(System_t *sys, RC_Ctl_t *remote, Chassis_Info_t *chas)
{
	/* ϵͳ���� */
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
	/* ϵͳ�쳣 */
	else
	{
		// ��λ�ɳ�ʼֵ
		KeyLockFlag_F = false;
		KeyLockFlag_C = false;
		RcLockFlag_Wheel = false;
		RcUnlockFlag_Wheel = false;
	}	
}

/* #Ӧ�ò�# ---------------------------------------------------------------------------------------------------------------------------------------*/
/* #�������# -------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	����ң��ֵ�����ٶȻ�������ֵ(��С���ݰ汾)
 *	@note		�漰�����ķ�ֵ��˶��ϳ�
 *					target��Ϊ+ʱ:
 *					LF201	��	��	RF202	
 *					
 *					LB203	��	��	RB204
 *					
 *					-660 ~ +660 ң�ز�ֵ
 *					
 *					��Ҫ����ҡ�˵ķ���ֵ����֤�ٶȵ�Ŀ��ֵ���ΪCHASSIS_PID_OUT_MAX(9000)
 */

void REMOTE_SetChassisSpeed(void)
{
	float k_rc_z;

	/* ��еģʽ */
	if(Chassis.PidMode == MECH) {
		
		Chas_Target_Speed[ X ] = RC_LEFT_CH_LR_VALUE * kRc_Mech_Chas_Standard;
		Chas_Target_Speed[ X ] = constrain(Chas_Target_Speed[ X ], -Chas_Standard_Move_Max, Chas_Standard_Move_Max);
		
		Chas_Target_Speed[ Y ] = RC_LEFT_CH_UD_VALUE * kRc_Mech_Chas_Standard;
		Chas_Target_Speed[ Y ] = constrain(Chas_Target_Speed[ Y ], -Chas_Standard_Move_Max, Chas_Standard_Move_Max);
		
		Chas_Target_Speed[ Z ] = RC_RIGH_CH_LR_VALUE * kRc_Mech_Chas_Spin;
		Chas_Target_Speed[ Z ] = constrain(Chas_Target_Speed[ Z ], -Chas_Spin_Move_Max, Chas_Spin_Move_Max);
		
	}
	/* ������ģʽ */
	else if(Chassis.PidMode == GYRO) {

		Chas_Target_Speed[ X ] = RC_LEFT_CH_LR_VALUE * kRc_Gyro_Chas_Standard;
		Chas_Target_Speed[ X ] = constrain(Chas_Target_Speed[ X ], -Chas_Standard_Move_Max, Chas_Standard_Move_Max);
		
		Chas_Target_Speed[ Y ] = RC_LEFT_CH_UD_VALUE * kRc_Gyro_Chas_Standard;
		Chas_Target_Speed[ Y ] = constrain(Chas_Target_Speed[ Y ], -Chas_Standard_Move_Max, Chas_Standard_Move_Max);
		
		Chassis_Z_PID.Angle.target = CHASSIS_GetMiddleAngle();	// �ָ����̸���
		Chas_Target_Speed[ Z ] = CHASSIS_Z_Angle_PidCalc(&Chassis_Z_PID.Angle, (YAW_DIR) * kRc_Gyro_Chas_Spin);
		Chas_Target_Speed[ Z ] = constrain(Chas_Target_Speed[ Z ], -Chas_Spin_Move_Max, Chas_Spin_Move_Max);
	}
	
	/* ������ת����(ͨ����������������������ת��ƽ�Ƶķ������) */
	if(abs(Chas_Target_Speed[ Z ]) > 800) {	
		// Ťͷ�ٶ�Խ�죬ƽ���ٶ�Խ��
		k_rc_z = ((Chas_Spin_Move_Max - (abs(Chas_Target_Speed[ Z ]) - 800)) * (Chas_Spin_Move_Max - (abs(Chas_Target_Speed[ Z ]) - 800)))
							/ (Chas_Spin_Move_Max * Chas_Spin_Move_Max);	// ���ۼ��㷶Χ (0.008, 1)
		
		k_rc_z = constrain(k_rc_z, 0.f, 1.f);
	} else {
		k_rc_z = 1.f;
	}		
	
	/* ������ת�����·����ٶ�����ֵ */
	Chas_Target_Speed[ X ] = Chas_Target_Speed[ X ] * k_rc_z;
	Chas_Target_Speed[ Y ] = Chas_Target_Speed[ Y ] * k_rc_z;
	
	/* ���㵱ǰ�������ٶ� */
	Chas_Target_Speed[ ALL ] = abs(Chas_Target_Speed[ X ]) + abs(Chas_Target_Speed[ Y ]) + abs(Chas_Target_Speed[ Z ]);
	
	/* ȫ������㷨 */
	CHASSIS_OmniSpeedCalc();
}

/**
 *	@brief	����ң��ֵ�����ٶȻ�������ֵ(С���ݰ汾)
 *	@note		�漰�����ķ�ֵ��˶��ϳ�
 *					target��Ϊ+ʱ:
 *					LF201	��	��	RF202	
 *					
 *					LB203	��	��	RB204
 *					
 *					-660 ~ +660 ң�ز�ֵ
 *					
 *					��Ҫ����ҡ�˵ķ���ֵ����֤�ٶȵ�Ŀ��ֵ���ΪCHASSIS_PID_OUT_MAX(9000)
 *
 *			��̨����ϵ(����С����֮ǰ���Ի�е��ֵ)��
 *				��ǹ�ܳ��� Y
 *				��
 *				��	  
 *				������	  X
 *
 *			��������ϵ��
 *			^\.		^/.	
 *				�� 		  y	˳ʱ��Ƕ�ƫ������(+)
 *				�� 
 *				������	  x
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
	
	/* ��еģʽ */
	if(CHASSIS_IfMechMode() == true) {
		
		Chas_Target_Speed[ X ] = RC_LEFT_CH_LR_VALUE * kRc_Mech_Chas_Standard;
		Chas_Target_Speed[ X ] = constrain(Chas_Target_Speed[ X ], -Chas_Standard_Move_Max, Chas_Standard_Move_Max);
		
		Chas_Target_Speed[ Y ] = RC_LEFT_CH_UD_VALUE * kRc_Mech_Chas_Standard;
		Chas_Target_Speed[ Y ] = constrain(Chas_Target_Speed[ Y ], -Chas_Standard_Move_Max, Chas_Standard_Move_Max);
		
		Chas_Target_Speed[ Z ] = RC_RIGH_CH_LR_VALUE * kRc_Mech_Chas_Spin;
		Chas_Target_Speed[ Z ] = constrain(Chas_Target_Speed[ Z ], -Chas_Spin_Move_Max, Chas_Spin_Move_Max);
		
	}
	/* ������ģʽ */
	else if(CHASSIS_IfGyroMode() == true) {

		Chas_Target_Speed[ X ] = RC_LEFT_CH_LR_VALUE * kRc_Gyro_Chas_Standard;
		Chas_Target_Speed[ X ] = constrain(Chas_Target_Speed[ X ], -Chas_Standard_Move_Max, Chas_Standard_Move_Max);
		
		Chas_Target_Speed[ Y ] = RC_LEFT_CH_UD_VALUE * kRc_Gyro_Chas_Standard;
		Chas_Target_Speed[ Y ] = constrain(Chas_Target_Speed[ Y ], -Chas_Standard_Move_Max, Chas_Standard_Move_Max);
		
		/* ������С���� */
		if(CHASSIS_IfTopGyroOpen() == true) {

			// �⻷����б�±仯
			Chassis_Z_PID.Angle.target = CHASSIS_MECH_YawBoundaryProc(&Chassis_Z_PID, top_gyro_dir * Chas_Top_Gyro_Step);
			// Z�����ٶ�pid����
			Chas_Target_Speed[ Z ] = CHASSIS_Z_Angle_PidCalc(&Chassis_Z_PID.Angle, (YAW_DIR) * k_Gyro_Chas_Top);
			
		}
		/* û��С���� */
		else {
			
			// �ָ����̸���
			Chassis_Z_PID.Angle.target = CHASSIS_GetMiddleAngle() ;	
			// Z�����ٶ�pid����
			Chas_Target_Speed[ Z ] = CHASSIS_Z_Angle_PidCalc(&Chassis_Z_PID.Angle, (YAW_DIR) * kRc_Gyro_Chas_Spin);
			
		}
		
		// Z�����ٶ��޷�
		Chas_Target_Speed[ Z ] = constrain(Chas_Target_Speed[ Z ], -Chas_Spin_Move_Max, Chas_Spin_Move_Max);
	}
	
	/* ������ģʽ�µ�����ϵת�� */
	if(CHASSIS_IfGyroMode() == true) {
		
		// ����ƫ���е��ֵ�ĽǶȲ�
		delta_angle = GIMBAL_GetTopGyroAngleOffset() * MECH_2_ANGLE_RADIAN;	//  (8192.f-GIMBAL_GetTopGyroAngleOffset()) * MECH_2_ANGLE_RADIAN
		target_speed_x = Chas_Target_Speed[ X ];
		target_speed_y = Chas_Target_Speed[ Y ];
		// ��ת���� ����̨����ϵ�������ٶ� ת���� ��������ϵ�������ٶ�
		Chas_Target_Speed[ X ] = cos(delta_angle) * target_speed_x - sin(delta_angle) * target_speed_y;
		Chas_Target_Speed[ Y ] = sin(delta_angle) * target_speed_x + cos(delta_angle) * target_speed_y;
		
	}
	
	/* ������ת����(ͨ����������������������ת��ƽ�Ƶķ������) */
	if(abs(Chas_Target_Speed[ Z ]) > 800) {	
		// Ťͷ�ٶ�Խ�죬ƽ���ٶ�Խ��
		k_rc_z = ((Chas_Spin_Move_Max - (abs(Chas_Target_Speed[ Z ]) - 800)) * (Chas_Spin_Move_Max - (abs(Chas_Target_Speed[ Z ]) - 800)))
							/ (Chas_Spin_Move_Max * Chas_Spin_Move_Max);	// ���ۼ��㷶Χ (0.008, 1)
		
		k_rc_z = constrain(k_rc_z, 0.f, 1.f);
	} else {
		k_rc_z = 1.f;
	}		
	
	/* ������ת�����·����ٶ�����ֵ */
	Chas_Target_Speed[ X ] = Chas_Target_Speed[ X ] * k_rc_z;
	Chas_Target_Speed[ Y ] = Chas_Target_Speed[ Y ] * k_rc_z;
	
	/* ���㵱ǰ�������ٶ� */
	Chas_Target_Speed[ ALL ] = abs(Chas_Target_Speed[ X ]) + abs(Chas_Target_Speed[ Y ]) + abs(Chas_Target_Speed[ Z ]);
	
	/* ȫ������㷨 */
	CHASSIS_OmniSpeedCalc();	
}

/**
 *	@brief	����ң�ز���������̨С����ģʽ
 */
void REMOTE_SetTopGyro(void)
{
	/* �������Ͽ��� */
	if(RC_THUMB_WHEEL_VALUE < -650) {
		if(RcLockFlag_Wheel == false) {
			/* ������ģʽ */
			if(CHASSIS_IfGyroMode() == true) {
				Chassis_Z_PID.Angle.target = Chassis_Z_PID.Angle.feedback;
				Chassis.TopGyro = true;
				Chassis.Twist = false;
				top_gyro_dir = -1;
				//���ݵ�����Ҫ����������Ϊ3~7s
				Chas_Top_Gyro_Period = RNG_ucGetRandomNum(3, 7)*1000;
			}
		}
		RcLockFlag_Wheel = true;
	} else {
		RcLockFlag_Wheel = false;
	}
	
	/* �������¹ر� */
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
 *	@brief	����б�º���
 *	@note	�����������ϵ��1
 *			��ʱδ��������Сϵ��0
 */
#define KEY_RAMP_SCALE	500
float KEY_RampChasSpeed(int8_t key_state, int16_t *time, uint16_t inc_ramp_step, uint16_t dec_ramp_step)
{
	#if 0
	float fac;

	if(key_state == 1) {	// ��������
		if(fac < 1) {
			*time += inc_ramp_step;
		}
	} else {	// �����ɿ�
		if(fac > 0) {
			*time -= dec_ramp_step;
			if(*time < 0) {
				*time = 0;
			}
		}
	}

	fac = 0.15 * sqrt( 0.15 * (*time) );	// time�ۼӵ�296.3б�¾����
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
 *	@brief	���ݰ���ֵ������ת�ƶ��ٶ�
 */
void KEY_SetChasSpinSpeed(int16_t sSpinMax)
{
	Chas_Spin_Move_Max = sSpinMax;

	/* ��еģʽ */
	if(CHASSIS_IfMechMode() == true) {
		
		Chas_Target_Speed[ Z ] = MOUSE_X_MOVE_SPEED * kKey_Mech_Chas_Spin;
		Chas_Target_Speed[ Z ] = constrain(Chas_Target_Speed[ Z ], -Chas_Spin_Move_Max, Chas_Spin_Move_Max);
		
	}
	/* ������ģʽ */
	else if(CHASSIS_IfGyroMode() == true) {
		/* ������С���� */
		if(CHASSIS_IfTopGyroOpen() == true) {
			
			// �⻷����б�±仯
			Chassis_Z_PID.Angle.target = CHASSIS_MECH_YawBoundaryProc(&Chassis_Z_PID, top_gyro_dir * Chas_Top_Gyro_Step);
			// Z�����ٶ�pid����
			Chas_Target_Speed[ Z ] = CHASSIS_Z_Angle_PidCalc(&Chassis_Z_PID.Angle, (YAW_DIR) * k_Gyro_Chas_Top);
			
		}
		/* ������Ť�� */
		else if(CHASSIS_IfTwistOpen() == true) {
			
			if(Chassis_Z_PID.Angle.target >= CHASSIS_GetMiddleAngle() + 800) {
				twist_dir = -1;
			} 
			else if(Chassis_Z_PID.Angle.target <= CHASSIS_GetMiddleAngle() - 800) {
				twist_dir = +1;
			}
			Chassis_Z_PID.Angle.target = CHASSIS_MECH_YawBoundaryProc(&Chassis_Z_PID, twist_dir * Chas_Twist_Step);
			
			// Z�����ٶ�pid����
			Chas_Target_Speed[ Z ] = CHASSIS_Z_Angle_PidCalc(&Chassis_Z_PID.Angle, (YAW_DIR) * k_Gyro_Chas_Twist);
			
		}
		/* û��С����/Ť�� */
		else {
			if(Flag.Chassis.GoHome == true) {
				/* ���ֵ���״̬
					 ���ε��̸���
					 �޸ĵ����߼� */
				CHASSIS_LogicRevert();// �޸ĵ����߼�
			}
			
			/* ���õ��̸����߼� */
			Chassis_Z_PID.Angle.target = CHASSIS_GetMiddleAngle();
			
			if(Flag.Chassis.GoHome == false)
				Chas_Target_Speed[ Z ] = CHASSIS_Z_Angle_PidCalc(&Chassis_Z_PID.Angle, (YAW_DIR) * kKey_Gyro_Chas_Spin);
			else 
				Chas_Target_Speed[ Z ] = 0;	// ���ε��̸���
		}
		
		/* Z�����ٶ��޷� */
		Chas_Target_Speed[ Z ] = constrain(Chas_Target_Speed[ Z ], -Chas_Spin_Move_Max, Chas_Spin_Move_Max);
	}
}

/**
 *	@brief	���ݰ���ֵ����ˮƽ�ƶ��ٶ�
 */
void KEY_SetChasMoveSpeed(int16_t sMoveMax, int16_t sMoveRamp)
{
	static int16_t time_fron_y, time_back_y, time_left_x, time_righ_x;
	float target_speed_x, target_speed_y;
	float k_rc_z;
	
	Chas_Standard_Move_Max = sMoveMax;
	Time_Inc_Normal = sMoveRamp;
	
	if(IF_KEY_PRESSED_W) {
		time_back_y = 0;	// ����б������
	} 
	if(IF_KEY_PRESSED_S) {
		time_fron_y = 0;	// ǰ��б������
	}
	if(IF_KEY_PRESSED_A) {
		time_righ_x = 0;	// ����б������
	}
	if(IF_KEY_PRESSED_D) {
		time_left_x = 0;	// ����б������
	}	
	
	if(Chassis.Mode == CHAS_MODE_NORMAL) {
		
		/* ǰ����ͻ�䣬�տ�ʼһС�λ���б�£���ֹ���Ӵ��˷ѹ��� */
		if(abs(Chas_Target_Speed[ Y ]) < (Chas_Standard_Move_Max/2.5f)) {
			Chas_Slope_Move_Fron = Chas_Standard_Move_Max * KEY_RampChasSpeed(IF_KEY_PRESSED_W, &time_fron_y, Time_Inc_Saltation, TIME_DEC_NORMAL);
			Chas_Slope_Move_Back = Chas_Standard_Move_Max * KEY_RampChasSpeed(IF_KEY_PRESSED_S, &time_back_y, Time_Inc_Saltation, TIME_DEC_NORMAL);
		} else {
			Chas_Slope_Move_Fron = Chas_Standard_Move_Max * KEY_RampChasSpeed(IF_KEY_PRESSED_W, &time_fron_y, Time_Inc_Normal, TIME_DEC_NORMAL);
			Chas_Slope_Move_Back = Chas_Standard_Move_Max * KEY_RampChasSpeed(IF_KEY_PRESSED_S, &time_back_y, Time_Inc_Normal, TIME_DEC_NORMAL);
		}
		
		/* ���ҷ���ͻ�䣬�տ�ʼһС�λ���б�£���ֹ���Ӵ��˷ѹ��� */
		if(abs(Chas_Target_Speed[ X ]) < (Chas_Standard_Move_Max/2.5f)) {
			Chas_Slope_Move_Fron = Chas_Standard_Move_Max * KEY_RampChasSpeed(IF_KEY_PRESSED_W, &time_fron_y, Time_Inc_Saltation, TIME_DEC_NORMAL);
			Chas_Slope_Move_Back = Chas_Standard_Move_Max * KEY_RampChasSpeed(IF_KEY_PRESSED_S, &time_back_y, Time_Inc_Saltation, TIME_DEC_NORMAL);
		} else {
			Chas_Slope_Move_Left = Chas_Standard_Move_Max * KEY_RampChasSpeed(IF_KEY_PRESSED_A, &time_left_x, Time_Inc_Normal, TIME_DEC_NORMAL);
			Chas_Slope_Move_Righ = Chas_Standard_Move_Max * KEY_RampChasSpeed(IF_KEY_PRESSED_D, &time_righ_x, Time_Inc_Normal, TIME_DEC_NORMAL);
		}
		
	} else {
		/* ȫ��б�������� */
		Chas_Slope_Move_Fron = Chas_Standard_Move_Max * KEY_RampChasSpeed(IF_KEY_PRESSED_W, &time_fron_y, Time_Inc_Normal, TIME_DEC_NORMAL);
		Chas_Slope_Move_Back = Chas_Standard_Move_Max * KEY_RampChasSpeed(IF_KEY_PRESSED_S, &time_back_y, Time_Inc_Normal, TIME_DEC_NORMAL);
		Chas_Slope_Move_Left = Chas_Standard_Move_Max * KEY_RampChasSpeed(IF_KEY_PRESSED_A, &time_left_x, Time_Inc_Normal, TIME_DEC_NORMAL);
		Chas_Slope_Move_Righ = Chas_Standard_Move_Max * KEY_RampChasSpeed(IF_KEY_PRESSED_D, &time_righ_x, Time_Inc_Normal, TIME_DEC_NORMAL);
	}
	
	/* ������ת����(ͨ����������������������ת��ƽ�Ƶķ������) */
	if(abs(Chas_Target_Speed[ Z ]) > 800) {
		// Ťͷ�ٶ�Խ�죬ƽ���ٶ�Խ��
		k_rc_z = ((Chas_Spin_Move_Max - (abs(Chas_Target_Speed[ Z ]) - 800)) * (Chas_Spin_Move_Max - (abs(Chas_Target_Speed[ Z ]) - 800)))
							/ (Chas_Spin_Move_Max * Chas_Spin_Move_Max);
		
		k_rc_z = constrain(k_rc_z, 0.f, 1.f);
	} else {
		k_rc_z = 1.f;
	}		
	
	/* �����ٶȼ��� */
	Chas_Target_Speed[ Y ] = (Chas_Slope_Move_Fron - Chas_Slope_Move_Back) * k_rc_z;	
	Chas_Target_Speed[ X ] = (Chas_Slope_Move_Righ - Chas_Slope_Move_Left) * k_rc_z;
	
	/* ������ģʽ�µ�����ϵת�� */
	if(CHASSIS_IfGyroMode() == true) {
		/* ����ƫ���е��ֵ�ĽǶȲ� */
		delta_angle = GIMBAL_GetTopGyroAngleOffset() * MECH_2_ANGLE_RADIAN;	//  (8192.f-GIMBAL_GetTopGyroAngleOffset()) * MECH_2_ANGLE_RADIAN
		target_speed_x = Chas_Target_Speed[ X ];
		target_speed_y = Chas_Target_Speed[ Y ];
		
		if(Flag.Chassis.GoHome == true)
			delta_angle = 0;	// ���ֵ���״̬
		
		/* ��ת���� ����̨����ϵ�������ٶ� ת���� ��������ϵ�������ٶ� */
		Chas_Target_Speed[ X ] = cos(delta_angle) * target_speed_x - sin(delta_angle) * target_speed_y;
		Chas_Target_Speed[ Y ] = sin(delta_angle) * target_speed_x + cos(delta_angle) * target_speed_y;
	}	
}

/**
 *	@brief	���ݰ���ֵ�����ٶȻ�������ֵ
 *  @note		Loop Time: 2ms
 *					�������ֵ����ٶ�(rpm)
 *					
 *					\		/			
 *						8	------	��ת����	(��תʱ�����������ٶ���ͬ)
 *					/		\
 *	
 *					\	8 	/	----	��ת����	(��תʱ�����ֵ������ٶȲ�ͬ���뾶С�������ٶ�ҪС���뾶��������ٶ�Ҫ��)
 *					
 *					/		\
 *		
 *					��֪����������ת���˶��������ϸ���Ľ��ٶ�(��)��ͬ�����ٶ�(v)��ͬ
 *					v = ��*r;	// �뾶�� -> ���ٶȡ�
 *					�� = Chas_Target_Speed[ Z ];	// ��е�Ƕ�/ms
 *					vz = ��*r;
 *
 *					�������ǵĵ��̵��PID���Ƶ�Ŀ�������Ƕ���ת��(rpm)���ٶȷ���ֵΪת��ת��(rpm)
 *					# Chas_Target_Speed[ Z ] - ����һ���̶��Ϸ�Ӧת�ٵ���������ȫ�ȼ�
 *
 *					��(��е�Ƕ�/ms) => (��е�Ƕ�/8192*360/ms) => (��/ms) = (1000��/s) = (1000/57.3 rad/s)
 *					��(rad/s) = Chas_Target_Speed[ Z ] / 8192 * 360 *1000 / 57.3
 *					r(mm) => (1/1000 m)
 *					r(m) = ���Ӿ����ĵľ���/1000;
 *					����v(m/s) = ��*r = Chas_Target_Speed[ Z ] / 8192 * 360 *1000 / 57.3 * ���Ӿ����ĵľ���/1000
 *									 					= Chas_Target_Speed[ Z ] / 8192 * 360 / 57.3 * ���Ӿ����ĵľ���;
 *
 *					����rpm(r/min) = ���ٱ� * ת��rpm => (�����ܳ�(mm)*���ٱ�*r/min) => (mm/min)
 *					����v(m/s) = �����ܳ�(mm)*���ٱ�*ת��rpm(r/min)
 *
 *					����v(m/s) => ((mm/1000)(min/60)) => (60/1000*mm/min)
 *
 *					# �����ܳ�(mm)*���ٱ�*ת��rpm(r/min) = 60/1000*����v(m/s)
 *
 *					ת��rpm = 60/1000*����v(m/s) / (�����ܳ�(mm)*���ٱ�)
 *	
 *					ת��rpm(r/min) = 60/1000/(�����ܳ�(mm)*���ٱ�) * (Chas_Target_Speed[ Z ] / 8192 * 360 / 57.3 * ���Ӿ����ĵľ���)
 */
float rotate_ratio_fr;
float rotate_ratio_fl;
float rotate_ratio_bl;
float rotate_ratio_br;
float wheel_ratio_rpm;
void KEY_SetChassisSpeed(int16_t sSpinMax, int16_t sMoveMax, int16_t sMoveRamp)
{
	/* ��ת */
	KEY_SetChasSpinSpeed(sSpinMax);
	
	/* ƽ�� */
	KEY_SetChasMoveSpeed(sMoveMax, sMoveRamp);
	
	/* ȫ������㷨 */
	CHASSIS_OmniSpeedCalc();		
}

/**
 *	@brief	���ݰ���ֵ���õ���ģʽ
 */
void KEY_SetChassisMode(void)
{
	/* Ctrl+W */
	if(IF_KEY_PRESSED_W && IF_KEY_PRESSED_CTRL) {
		CHASSIS_SetMode(CHAS_MODE_SZUPUP);
	}
}

/**
 *	@brief	���ݰ���ֵ���õ���С����ģʽ
 */
void KEY_SetTopGyro(void)
{
	if(IF_KEY_PRESSED_F) {
		if(KeyLockFlag_F == false) {
			if(CHASSIS_IfGyroMode() == true) {
				if(Chassis.TopGyro == false) {
					Chassis_Z_PID.Angle.target = Chassis_Z_PID.Angle.feedback;	// ���¸տ���С����ʱ��Ŀ��Ƕ�
					Chassis.TopGyro = true;	// ������
					Chassis.Twist = false;	// ��Ť��
					top_gyro_dir = -1;	// ����������������ʹ��С���ݷ������
					//���ݵ�����Ҫ����������Ϊ3~7s
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
 *	@brief	���ݰ���ֵ���õ���Ť��ģʽ
 */
void KEY_SetTwist(void)
{
	if(IF_KEY_PRESSED_C) {
		if(KeyLockFlag_C == false) {
			if(CHASSIS_IfGyroMode() == true) {
				if(Chassis.Twist == false) {
					Chassis_Z_PID.Angle.target = Chassis_Z_PID.Angle.feedback;	// ���¸տ���Ť��ʱ��Ŀ��Ƕ�
					Chassis.Twist = true;		// ��Ť��
					Chassis.TopGyro = false;	// ������
					twist_dir = -1;	// ����������������ʹ��Ť���������
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

/* #����# -------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	���̳�ʼ��
 */
void CHASSIS_Init(void)
{
	CHASSIS_KalmanCreate();	// �����������˲���
	CHASSIS_InitParams();	// ��ʼ�����̲���
}

/**
 *	@brief	���̹�������(�����ٷ���)
 *	@note	�洫�㷨����Ҫ�Ǳ������㷨,ICRA
 *			# �迼�Ƿ��������������Ļ��������ӳɣ�
 */
void CHASSIS_PowerLimit(Chassis_Info_t *chas, Chassis_PID_t *pid, Judge_Info_t *judge)
{
	float kLimit;
	float fTotalOutput;
	float fRemainJ;
	static uint16_t usJudgeErrCnt = 0;
	
	// ��ȡ���役������
	fRemainJ = JUDGE_fGetChassisPowerBuffer();//judge->PowerHeatData.chassis_power_buffer;	
	fTotalOutput = abs(pid[LEFT_FRON_201].Out) + 
				   abs(pid[RIGH_FRON_202].Out) + 
				   abs(pid[LEFT_BACK_203].Out) + 
				   abs(pid[RIGH_BACK_204].Out);
	
	if(JUDGE_IfDataValid() == false) {
		usJudgeErrCnt++;
		if(usJudgeErrCnt > 100) {
			usJudgeErrCnt = 0;	// ��ֹ���
			chas->Power.cur_limit = 9000;	// ����1/4
		}
	} else {
		usJudgeErrCnt = 0;
		// ʣ�ཹ������С,��ʼ�������,����ϵ��Ϊƽ����ϵ
		if(fRemainJ < WARNING_REMAIN_POWER) {
			kLimit = (float)(fRemainJ / WARNING_REMAIN_POWER)
						* (float)(fRemainJ / WARNING_REMAIN_POWER);
			chas->Power.cur_limit =  kLimit * chas->Power.max_limit;
		} else {	// ���������ָ���һ����ֵ
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
 *	@brief	����Ť���ֲڰ�
 */
float CHASSIS_TwistTargetCalc(int16_t maxTarget, int16_t rampTarget)
{
	static uint8_t dir = 1;
	static float target_z_speed = 0;
	static portTickType tickTime_prev = 0;
	static portTickType tickTime_now = 0;	

	tickTime_now = xTaskGetTickCount();
	if(tickTime_now  - tickTime_prev > TIME_STAMP_200MS) {	// ��ʱδ����Ť������������
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
 *	@brief	����ȫ���˶��㷨
 */
void CHASSIS_OmniSpeedCalc(void)
{
	float kx, ky, kz;
	/* ���㵱ǰ�������ٶ� */
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
 *	@brief	��еģʽ����̨YAW����ֵ�߽紦��
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
 *	@brief	���̳������
 */
void CHASSIS_NormalCtrl(void)
{
	KEY_SetChassisMode();
	KEY_SetTopGyro();
	KEY_SetTwist();
	KEY_SetChassisSpeed(SPIN_MAX_NORMAL, STANDARD_MAX_NORMAL, TIME_INC_NORMAL);
}

/**
 *	@brief	���̴��ʱ��Ŀ���
 *	@note	�����ʱ����̲���
 */
void CHASSIS_BuffCtrl(void)
{
	/*----�����޸�----*/
	Chassis_PID[LEFT_FRON_201].Speed.target = 0;
	Chassis_PID[RIGH_FRON_202].Speed.target = 0;
	Chassis_PID[LEFT_BACK_203].Speed.target = 0;
	Chassis_PID[RIGH_BACK_204].Speed.target = 0;	
}

/**
 *	@brief	�ֶ����¿���
 */
void CHASSIS_SzuPupCtrl(void)
{
	/* �ɿ�W����Ctrl */
	if(!IF_KEY_PRESSED_W || !IF_KEY_PRESSED_CTRL) {
		CHASSIS_SetMode(CHAS_MODE_NORMAL);
	}
	
	KEY_SetChassisSpeed(SPIN_MAX_SZUPUP, STANDARD_MAX_SZUPUP, TIME_INC_SZUPUP);
}

/**
 *	@brief	�Զ���λ����
 *	@note	x��������ܴ�С����վӰ��
 */
#define R_B_CASE	5

#define RELOAD_BULLET_DIS_Y				200	// ��λmm
#define RELOAD_BULLET_DIS_JUDGE_CNT		3	// �жϼ���
#define RELOAD_BULLET_DIS_JUDGE_SMALL	450	// С����վx�����о�
#define RELOAD_BULLET_DIS_JUDGE_BIG		750	// �󲹸�վx�����о�
#define RELOAD_BULLET_DIS_JUDGE_DEATH	100	// 10cm������Χ
#define RELOAD_BULLET_DIS_X_SMALL		300	// �ж�ΪС����վʱ��x�����ƶ�����
#define RELOAD_BULLET_DIS_X_BIG			500	// �ж�Ϊ�󲹸�վʱ��x�����ƶ�����

float tar_angle_sum_y = -((RELOAD_BULLET_DIS_Y / PERIMETER) * 8192) / CHASSIS_DECELE_RATIO;	// ����ȡ��
float ramp_angle_y = 1024;

portTickType ulDetectTime[TIME_STATE_COUNT];	// ���Գ���������֡��

void CHASSIS_ReloadBulletCtrl(void)
{
	// �������Ƚ����������Ϸ���סһ����
	#if (R_B_CASE == 1)
		// �޷���������ʱģ��
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
			tx = TIME_STAMP_500MS/2;	// ��������Ϊ2ms
			ty = TIME_STAMP_500MS/2;	// ��������Ϊ2ms
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
		
		/* ȫ������㷨 */
		CHASSIS_OmniSpeedCalc();	
		
	#elif (R_B_CASE == 2)
		// ������������ʱģ��
		/*
			����ֱ����y�ϣ����õ����е�ǶȺ����ְ뾶���Զ�λ
			��ˮƽ����x�ϣ������ٶ�+��ʱ
		*/
		/* 
			Branch_1
				���ĸ�����ķ����Ƕ�ȡƽ����Ϊ����ֵ�������Ļ�ֻ��һ���⻷pid������.
				tar_angle_sum_y - fed_angle_sum_y = err_y -> PID -> Chas_Target_Speed[ Y ]
				tx ��ʱ -> Chas_Target_Speed[ X ]
				����ȫ������㷨
		*/
		static uint16_t tx;
		float fed_angle_sum_y;
		
		if(Flag.Chassis.ReloadBulletStart == true) {
			Flag.Chassis.ReloadBulletStart = false;
			/* Y */
			// ����ۼӻ�е�Ƕ�����
			CHASSIS_Angle_ClearSum(Chassis_PID);
			// y����Ŀ���ۼӽǶ�����
			Chas_Locate_PID[ Y ].target = 0;
			/* X */
			tx = TIME_STAMP_500MS/2;	// ��������2ms
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
		
		/* ȫ������㷨 */
		CHASSIS_OmniSpeedCalc();	
		
	#elif (R_B_CASE == 3)
		// ���������Զ���λģ��
		/*
				
		*/
	#elif (R_B_CASE == 4)
		// ˫����PID����ģ��
		
		static float dis_judge_x = 0;	// ����λ�þ�����Ϣ�жϴ�С����վ
		float tar_angle_sum_y = -((RELOAD_BULLET_DIS_Y / PERIMETER) * 8192) / CHASSIS_DECELE_RATIO;	// ����ȡ��
		
		if(Flag.Chassis.ReloadBulletStart == true) {
			// ���������־λ
			Flag.Chassis.ReloadBulletStart = false;
			// �����ж���С����վ
			Flag.Chassis.ReloadBulletJudge = false;
			// ��װ���жϼ���
			Cnt.Chassis.ReloadBulletJudge = RELOAD_BULLET_DIS_JUDGE_CNT;
			// ��ʼ��������
			dis_judge_x = 0.f;
			// ����ۼӻ�е�Ƕ�����
			CHASSIS_Angle_ClearSum(Chassis_PID);
			// y����Ŀ���ۼӽǶ�����
			Chas_Locate_PID[ Y ].target = 0;
			// x���������ٶ�����
			Chas_Target_Speed[ X ] = 0;
			// y���������ٶ�����
			Chas_Target_Speed[ Y ] = 0;
			// ����һ�γ�����̽��
			Ultra.update = false;
			ULTRA_Detect();
		}
		
		/* �����жϴ�С����վ */
		if(Cnt.Chassis.ReloadBulletJudge) {
			
			// �ȴ����������ݸ���
			if(Ultra.update == true) {
				
				Ultra.update = false;	// ������±�־

				ULTRA_Detect();	// �ٿ���һ�γ�����̽��

				ulDetectTime[NOW] = xTaskGetTickCount();
				
				ulDetectTime[DELTA] = ulDetectTime[NOW] - ulDetectTime[PREV];

				ulDetectTime[PREV] = ulDetectTime[NOW];
				
				Cnt.Chassis.ReloadBulletJudge--;
				
				dis_judge_x += Ultra.dis;
				
				/* �жϽ��� */
				if(Cnt.Chassis.ReloadBulletJudge == 0) {
				
					dis_judge_x /= RELOAD_BULLET_DIS_JUDGE_CNT;	// ��β���ȡƽ��
					
					if(myDeathZoom(RELOAD_BULLET_DIS_JUDGE_SMALL, RELOAD_BULLET_DIS_JUDGE_DEATH, dis_judge_x) == true) {
					
						Chas_Locate_PID[ X ].target = RELOAD_BULLET_DIS_X_SMALL;	// �ж�ΪС����վ
						
						Chas_Locate_PID[ Y ].target = tar_angle_sum_y;
						
						Flag.Chassis.ReloadBulletJudge = true;
						
					}
					else if(myDeathZoom(RELOAD_BULLET_DIS_JUDGE_BIG, RELOAD_BULLET_DIS_JUDGE_DEATH, dis_judge_x) == true) {
					
						Chas_Locate_PID[ X ].target = RELOAD_BULLET_DIS_X_BIG;		// �ж�Ϊ�󲹸�վ
						
						Chas_Locate_PID[ Y ].target = tar_angle_sum_y;
						
						Flag.Chassis.ReloadBulletJudge = true;
						
					}
					else {
					
						// ..��������
						Flag.Chassis.ReloadBulletJudge = false;
					}
					
				}
			}
		}
		
		/* �ж��ɹ� */
		if(Flag.Chassis.ReloadBulletJudge == true) {
		
			if(Ultra.update == true) {
			
				Ultra.update = false;
				
				ULTRA_Detect();	// �ٿ���һ�γ�����̽��
				
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
//				ULTRA_Detect();	// �ٿ���һ�γ�����̽��
//				
//				ulDetectTime[NOW] = xTaskGetTickCount();
//				
//				ulDetectTime[DELTA] = ulDetectTime[NOW] - ulDetectTime[PREV];

//				ulDetectTime[PREV] = ulDetectTime[NOW];
//			
//			}
			
		}
		
		/* ȫ������㷨 */
		CHASSIS_OmniSpeedCalc();
		
	#elif (R_B_CASE == 5)
		// ˫����PID����ģ��
		
		Supply_Id_t supply_id;
		
		if(Flag.Chassis.ReloadBulletStart == true) {
			// ���������־λ
			Flag.Chassis.ReloadBulletStart = false;
			// �����ж���С����վ
			Flag.Chassis.ReloadBulletJudge = false;
			// ��װ���жϼ���
			Cnt.Chassis.ReloadBulletJudge = RELOAD_BULLET_DIS_JUDGE_CNT;
			// ����ۼӻ�е�Ƕ�����
			CHASSIS_Angle_ClearSum(Chassis_PID);
			// y����Ŀ���ۼӽǶ�����
			Chas_Locate_PID[ Y ].target = tar_angle_sum_y;
			// x���������ٶ�����
			Chas_Target_Speed[ X ] = 0;
			// y���������ٶ�����
			Chas_Target_Speed[ Y ] = 0;
			// ����һ�γ�����̽��
			Ultra.update = true;
		}
		
		/* �����жϴ�С����վ */
		if(Cnt.Chassis.ReloadBulletJudge) {
			
			supply_id = JUDGE_eGetSupplyId();
			
			if(supply_id == SUPPLY_ID_1) {
				
				Chas_Locate_PID[ X ].target = RELOAD_BULLET_DIS_X_SMALL;	// �ж�ΪС����վ
				
				Flag.Chassis.ReloadBulletJudge = true;

				Cnt.Chassis.ReloadBulletJudge = 0;
				
			} else if(supply_id == SUPPLY_ID_2) {
				
				Chas_Locate_PID[ X ].target = RELOAD_BULLET_DIS_X_BIG;		// �ж�Ϊ�󲹸�վ
				
				Flag.Chassis.ReloadBulletJudge = true;
						
				Cnt.Chassis.ReloadBulletJudge = 0;

			}
			
		}
		
		/* �ж��ɹ� */
		if(Flag.Chassis.ReloadBulletJudge == true) {
		
			#if (ULTRA_USE_USART)
				if(Ultra.update == true) {
				
					Ultra.update = false;
					
					ULTRA_Detect();	// �ٿ���һ�γ�����̽��
					
					ulDetectTime[NOW] = xTaskGetTickCount();
					
					ulDetectTime[DELTA] = ulDetectTime[NOW] - ulDetectTime[PREV];

					ulDetectTime[PREV] = ulDetectTime[NOW];

					Chas_Locate_PID[ X ].feedback = Ultra.dis;
					
					Chas_Target_Speed[ X ] = CHASSIS_Locate_PidCalc(&Chas_Locate_PID[ X ]);
				
				}
			#else
				if(Ultra.update == true) {
					
					Ultra.update = false;
					
					ULTRA_Detect();	// �ٿ���һ�γ�����̽��

					ulDetectTime[PREV] = xTaskGetTickCount();
					
				} 
				else {
				
					// ��ȡSCL�����ж��Ƿ�̽�����
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
		/* �ж�ʧ�� */
		else {
		
			Chas_Target_Speed[ X ] = 0;
			
			Chas_Target_Speed[ Y ] = CHASSIS_Locate_PidCalc(&Chas_Locate_PID[ Y ]);
			
		}
		
		/* ȫ������㷨 */
		CHASSIS_OmniSpeedCalc();		
	
	#endif
}

/* #�����# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	���̻�ȡpidģʽ��Ϣ
 */
void CHASSIS_GetInfo(void)
{
	// ��ȡϵͳ��Ϣ
	CHASSIS_GetSysInfo(&System, &Chassis);
	// ��ȡ����ϵͳ��Ϣ
	CHASSIS_GetJudgeInfo(&Judge, &Chassis);
	// ��ȡIMU��Ϣ
	CHASSIS_GetImuInfo(&Chassis_Z_PID);	
	// ��ȡС������Ϣ
	CHASSIS_GetTopGyroInfo();
	// ��ȡң����Ϣ
	CHASSIS_GetRemoteInfo(&System, &Remote, &Chassis);
}

/**
 *	@brief	pid�������������
 */
void CHASSIS_PidCtrlTask(void)
{
	/* PID�ٶȻ����� */
	CHASSIS_Speed_PidCalc(Chassis_PID, LEFT_FRON_201); 	// ��ǰ - �ٶȻ�
	CHASSIS_Speed_PidCalc(Chassis_PID, RIGH_FRON_202); 	// ��ǰ - �ٶȻ�
	CHASSIS_Speed_PidCalc(Chassis_PID, LEFT_BACK_203); 	// ��� - �ٶȻ�
	CHASSIS_Speed_PidCalc(Chassis_PID, RIGH_BACK_204); 	// �Һ� - �ٶȻ�
	/* �������� */
	//CHASSIS_PowerLimit(&Chassis_Power, Chassis_PID, &Judge);
	/* ������� */
	CHASSIS_PidOut(Chassis_PID);	
}

/**
 *	@brief	ң�ؿ��Ƶ�������
 */
void CHASSIS_RcCtrlTask(void)
{
	//REMOTE_SetChassisSpeed();
	REMOTE_TOP_SetChassisSpeed();
	REMOTE_SetTopGyro();
}

/**
 *	@brief	���̿��Ƶ�������
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
 *	@brief	����ʧ�ر���
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
 *	@brief	���̿�������
 */
void CHASSIS_Ctrl(void)
{
	/*----��Ϣ����----*/
	CHASSIS_GetInfo();
	/*----�����޸�----*/
	if(Chassis.RemoteMode == RC) {
		CHASSIS_RcCtrlTask();
	}
	else if(Chassis.RemoteMode == KEY) {
		CHASSIS_KeyCtrlTask();
	}
	/*----�������----*/
	CHASSIS_PidCtrlTask();
}
