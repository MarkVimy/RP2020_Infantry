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
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#define X	0
#define Y 1
#define Z 2
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

/* б�� */
#define TIME_INC_NORMAL	2
#define TIME_DEC_NORMAL	500

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
uint16_t Time_IncSaltation;	// ǰ����ͻ��б��������

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
float kKey_Mech_Chas_Standard, kKey_Mech_Chas_Revolve;//ƽ�ƣ���ת

//������ģʽ�µ��̱���ϵ��,���Ƽ���б�±仯��
float kKey_Gyro_Chas_Standard, kKey_Gyro_Chas_Revolve;//ƽ�ƣ���ת

//С����ģʽ�µ��̱���ϵ��
float k_Gyro_Chas_Top;

//С������תת�ٲ���
float Chas_Top_Gyro_Step;

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
 *	@brief	���̹���
 */
Chassis_Power_t Chassis_Power = {
	.maxLimit = CHASSIS_MAX_CURRENT_LIMIT,
	.currentLimit = 0,
};

/**
 *	@brief	������Ϣ�ṹ��
 */
Chassis_Info_t Chassis = {
	.pid_mode = MECH,
	.mode = CHAS_MODE_NORMAL,
	.logic = CHAS_LOGIC_NORMAL,
	.top_gyro = false,
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
void CHASSIS_initParameter(void)
{
	/* ƽ��ҡ�������� */
	kRc_Mech_Chas_Standard = 14;	// 14*660 = 9240 -> 9000
	kRc_Gyro_Chas_Standard = 14;	// 14*660 = 9240 -> 9000
	
	/* ��תҡ�������� */
	kRc_Mech_Chas_Spin = 11.4;	// 11.4*660 = 7524
	kRc_Gyro_Chas_Spin = -4.8;	// (����)����
	
	/* С���ݵ���ϵ�� */
	k_Gyro_Chas_Top = -4.8;
	
	/* С����ת�ٲ��� */
	Chas_Top_Gyro_Step = 10;

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
 *	@brief	����������ģʽ�������˲�����ʼ��
 */
void CHASSIS_kalmanCreate(void)
{
	/* �ٶ���е�Ƕȵķ���û��������� */
	KalmanCreate(&Chassis_Kalman_Error, 1, 0);
}	

/**
 *	@brief	���̵������ʵʱ�ٶ�
 */
void CHASSIS_updateMotorSpeed(Chassis_PID_t *pid, Chassis_Motor_Names MOTORx)
{
	pid[MOTORx].Speed.feedback = g_Chassis_Motor_Info[MOTORx].speed;
}

/**
 *	@brief	���̵������ʵʱ�Ƕ�(�ۻ�ת���ĽǶ�)
 */
void CHASSIS_updateMotorAngle(Chassis_PID_t *pid, Chassis_Motor_Names MOTORx)
{
  //pid[MOTORx].Angle.feedback = g_Chassis_Motor_Info[MOTORx].angle;
	pid[MOTORx].Angle.feedback = g_Chassis_Motor_Info[MOTORx].angle_sum;
}

/**
 *	@brief	���̵������ʵʱ����ֵ
 */
void CHASSIS_updateMotorCurrent(Chassis_PID_t *pid, Chassis_Motor_Names MOTORx)
{
	//pid[MOTORx].Speed.feedback = g_Chassis_Motor_Info[MOTORx].speed;
}

/**
 *	@brief	���̵������ɲ��
 */
void CHASSIS_stop(Chassis_PID_t *pid)
{
	static int16_t pid_out[4] = {0, 0, 0, 0};
	
	/* �ڻ��ٶȻ�������� */
	pid[LEFT_FRON_201].Out = 0;
	pid[RIGH_FRON_202].Out = 0;
	pid[LEFT_BACK_203].Out = 0;
	pid[RIGH_BACK_204].Out = 0;
	
	CAN1_send(0x200, pid_out);	
}

/**
 *	@brief	���̵���ٶȻ�
 */
void CHASSIS_Speed_pidCalculate(Chassis_PID_t *pid, Chassis_Motor_Names MOTORx)
{
	pid[MOTORx].Speed.erro = pid[MOTORx].Speed.target - pid[MOTORx].Speed.feedback;
	pid[MOTORx].Speed.integrate += pid[MOTORx].Speed.erro;
	
	//pid[MOTORx].Speed.integrate = constrain(pid[MOTORx].Speed.integrate, -3000, 3000);
	// #integrate�Ƿ��޷�?
	
	pid[MOTORx].Speed.integrate = constrain(pid[MOTORx].Speed.integrate, -18000, 18000);
	
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
	pid[MOTORx].Speed.out = constrain(pid[MOTORx].Speed.out, -CHASSIS_PID_OUT_MAX, CHASSIS_PID_OUT_MAX);
	pid[MOTORx].Out = pid[MOTORx].Speed.out;
	
	/* test */
	speed_target_203 = pid[LEFT_BACK_203].Speed.target;
	speed_feedback_203 = pid[LEFT_BACK_203].Speed.feedback;
}

/**
 *	@brief	���̵��λ�û�
 *	@note
 *			����PID
 *			�ڻ� ����ֵ �������ٶ�(ʵ���Ͼ����ٶȻ�)
 *					 ���ֵ �����Ǽ��ٶ�
 *
 *			�⻷ ����ֵ �����Ƕ�
 *					 ���ֵ �������ٶ�
 */
void CHASSIS_Angle_pidCalculate(Chassis_PID_t *pid, Chassis_Motor_Names MOTORx)
{
	pid[MOTORx].Angle.erro = pid[MOTORx].Angle.target - pid[MOTORx].Angle.feedback;
	pid[MOTORx].Angle.integrate += pid[MOTORx].Angle.erro;
	//pid[MOTORx].Angle.integrate = constrain(pid[MOTORx].Angle.integrate, -3000, 3000);
	// #integrate�Ƿ��޷�?
	
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
	pid[MOTORx].Angle.out = constrain(pid[MOTORx].Angle.out, -CHASSIS_PID_OUT_MAX, CHASSIS_PID_OUT_MAX);
	
	/* test */
	angle_target_203 = pid[LEFT_BACK_203].Angle.target;
	angle_feedback_203 = pid[LEFT_BACK_203].Angle.feedback;
}

/**
 *	@brief	����������ģʽ����Z������ٶȻ�(ʵ������λ�û�)
 *	@note
 *				����ǰYAW��е�Ǻ���ֵYAW��е�ǵĲ�ֵ��Ϊ����ͽ� Z_Speed PID��������
 *				���������Ľ��ٶ�
 */
float CHASSIS_Z_Speed_pidCalculate(PID_Object_t *pid, float kp)
{
	pid->kp = kp;
	pid->erro = pid->target - pid->feedback;
	
	/* �ٽ紦�� */
	
	/*
	
		0 - 8191 

		���=����-����
	
		������4096
	
		����������������� --> ȷ�������ķ������Ϊ������0-��8191 --��--- ������������
	
		ʵ����� = -��8191 - ���� + ������ = -8191 + ���
	
		���С��-4096
	
		������������� -> ȷ���������� 8191->0  == �������ڷ���
	
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
	if(CHASSIS_ifTopGyroOpen() == false) {
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
 *	@brief	���̵��PID���������
 */
void CHASSIS_PID_out(Chassis_PID_t *pid)
{
	int16_t pidOut[4];
	
	pidOut[LEFT_FRON_201] = (int16_t)pid[LEFT_FRON_201].Out;
	pidOut[RIGH_FRON_202] = (int16_t)pid[RIGH_FRON_202].Out;
	pidOut[LEFT_BACK_203] = (int16_t)pid[LEFT_BACK_203].Out;
	pidOut[RIGH_BACK_204] = (int16_t)pid[RIGH_BACK_204].Out;
	
	CAN1_send(0x200, pidOut);
}

/* #��Ϣ��# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	���̻�ȡpidģʽ��Ϣ
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
	
	/* ����������������Ťͷ���� */
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
	
//	/* С����ģʽ���� */
//	if(GIMBAL_ifTopGyroMode() == true) {
//			CHASSIS_setMode(CHAS_MODE_TOP_GYRO);
//	} else {
//		/* ����С����ģʽ -> ���̳������ģʽ */
//		if(CHASSIS_ifTopGyroMode() == true) {
//			CHASSIS_setMode(CHAS_MODE_NORMAL);
//		}
//	}
	
//	/* �����߼�ȡ�� */
//	if(CHASSIS_ifLogicRevert() == true) {
//		CHASSIS_setMode(CHAS_MODE_REVERT);
//	}
	
	/* ���ִ򿪺����� */
//	if(MAGZINE_ifOpen() == true) {
//		CHASSIS_setMode(CHAS_MODE_SLOW);
//	}
}

/**	
 *	@breif	��������ģʽ
 */
Chassis_Mode_Names_t CHASSIS_getMode(void)
{
	return Chassis.mode;
}

/**	
 *	@breif	���������߼�
 */
Chassis_Logic_Names_t CHASSIS_getLogic(void)
{
	return Chassis.logic;
}

/**	
 *	@breif	�������̻�е��ֵĿ��
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
 *	@brief	���õ��̵�ģʽ
 */
void CHASSIS_setMode(Chassis_Mode_Names_t mode)
{
	Chassis.mode = mode;
}

/**
 *	@brief	�����߼�ȡ��
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
 *	@brief	�����Ƿ��ڻ�еģʽ
 */
bool CHASSIS_ifMechMode(void)
{
	if(Chassis.pid_mode == MECH)
		return true;
	else
		return false;
}

/**
 *	@brief	�����Ƿ���������ģʽ
 */
bool CHASSIS_ifGyroMode(void)
{
	if(Chassis.pid_mode == GYRO)
		return true;
	else
		return false;
}

/**
 *	@brief	�����Ƿ��ڳ���ģʽ
 */
bool CHASSIS_ifNormalMode(void)
{
	if(Chassis.mode == CHAS_MODE_NORMAL)
		return true;
	else 
		return false;
}

/**
 * @brief		�����Ƿ���С����ģʽ
 */
bool CHASSIS_ifTopGyroOpen(void)
{
	if(Chassis.pid_mode == GYRO
			&& Chassis.top_gyro == true)
		return true;

	return false;
}

/**	
 *	@breif	�����߼��Ƿ�ȡ��
 */
bool CHASSIS_ifLogicRevert(void)
{
	static Chassis_Logic_Names_t prev_logic = CHAS_LOGIC_NORMAL;
	
	/* �����߼�ȡ����� */
	if(prev_logic != Chassis.logic) {
		return true;
	}
	prev_logic = Chassis.logic;
	
	return false;
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

void REMOTE_setChassisSpeed(void)
{
	float k_rc_z;

	/* ��еģʽ */
	if(Chassis.pid_mode == MECH) {
		
		Chas_Target_Speed[ X ] = RC_LEFT_CH_LR_VALUE * kRc_Mech_Chas_Standard;
		Chas_Target_Speed[ X ] = constrain(Chas_Target_Speed[ X ], -Chas_Standard_Move_Max, Chas_Standard_Move_Max);
		
		Chas_Target_Speed[ Y ] = RC_LEFT_CH_UD_VALUE * kRc_Mech_Chas_Standard;
		Chas_Target_Speed[ Y ] = constrain(Chas_Target_Speed[ Y ], -Chas_Standard_Move_Max, Chas_Standard_Move_Max);
		
		Chas_Target_Speed[ Z ] = RC_RIGH_CH_LR_VALUE * kRc_Mech_Chas_Spin;
		Chas_Target_Speed[ Z ] = constrain(Chas_Target_Speed[ Z ], -Chas_Spin_Move_Max, Chas_Spin_Move_Max);
		
	}
	/* ������ģʽ */
	else if(Chassis.pid_mode == GYRO) {

		Chas_Target_Speed[ X ] = RC_LEFT_CH_LR_VALUE * kRc_Gyro_Chas_Standard;
		Chas_Target_Speed[ X ] = constrain(Chas_Target_Speed[ X ], -Chas_Standard_Move_Max, Chas_Standard_Move_Max);
		
		Chas_Target_Speed[ Y ] = RC_LEFT_CH_UD_VALUE * kRc_Gyro_Chas_Standard;
		Chas_Target_Speed[ Y ] = constrain(Chas_Target_Speed[ Y ], -Chas_Standard_Move_Max, Chas_Standard_Move_Max);
		
		Chassis_Z_PID.Angle.target = CHASSIS_getMiddleAngle();	// �ָ����̸���
		Chas_Target_Speed[ Z ] = CHASSIS_Z_Speed_pidCalculate(&Chassis_Z_PID.Angle, (YAW_DIR) * kRc_Gyro_Chas_Spin);
		Chas_Target_Speed[ Z ] = constrain(Chas_Target_Speed[ Z ], -Chas_Spin_Move_Max, Chas_Spin_Move_Max);
	}
	
	/* ������ת����(ͨ����������������������ת��ƽ�Ƶķ������) */
	if(abs(Chas_Target_Speed[ Z ]) > 800) {	// Ťͷ�ٶ�Խ�죬ƽ���ٶ�Խ��
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
	CHASSIS_omniSpeedCalculate();
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

void REMOTE_TOP_setChassisSpeed(void)
{
	float k_rc_z;
	float target_speed_x;
	float target_speed_y;
	
	/* ��еģʽ */
	if(CHASSIS_ifMechMode() == true) {
		
		Chas_Target_Speed[ X ] = RC_LEFT_CH_LR_VALUE * kRc_Mech_Chas_Standard;
		Chas_Target_Speed[ X ] = constrain(Chas_Target_Speed[ X ], -Chas_Standard_Move_Max, Chas_Standard_Move_Max);
		
		Chas_Target_Speed[ Y ] = RC_LEFT_CH_UD_VALUE * kRc_Mech_Chas_Standard;
		Chas_Target_Speed[ Y ] = constrain(Chas_Target_Speed[ Y ], -Chas_Standard_Move_Max, Chas_Standard_Move_Max);
		
		Chas_Target_Speed[ Z ] = RC_RIGH_CH_LR_VALUE * kRc_Mech_Chas_Spin;
		Chas_Target_Speed[ Z ] = constrain(Chas_Target_Speed[ Z ], -Chas_Spin_Move_Max, Chas_Spin_Move_Max);
		
		/* С���ݻ�������� */
		top_gyro_dir = 0;
		Chassis_Z_PID.AngleRampTarget = 0;
		Chassis_Z_PID.AngleRampFeedback = 0;
	}
	/* ������ģʽ */
	else if(CHASSIS_ifGyroMode() == true) {

		Chas_Target_Speed[ X ] = RC_LEFT_CH_LR_VALUE * kRc_Gyro_Chas_Standard;
		Chas_Target_Speed[ X ] = constrain(Chas_Target_Speed[ X ], -Chas_Standard_Move_Max, Chas_Standard_Move_Max);
		
		Chas_Target_Speed[ Y ] = RC_LEFT_CH_UD_VALUE * kRc_Gyro_Chas_Standard;
		Chas_Target_Speed[ Y ] = constrain(Chas_Target_Speed[ Y ], -Chas_Standard_Move_Max, Chas_Standard_Move_Max);
		
		/* ������С���� */
		if(CHASSIS_ifTopGyroOpen() == true) {
			top_gyro_dir = -1;
			Chassis_Z_PID.AngleRampTarget += top_gyro_dir * Chas_Top_Gyro_Step;
			if(Chassis_Z_PID.AngleRampFeedback < Chassis_Z_PID.AngleRampTarget) // �����ۼ�
			{
				Chassis_Z_PID.Angle.target = CHASSIS_MECH_yawTargetBoundaryProcess(&Chassis_Z_PID, Chas_Top_Gyro_Step);
			} 
			else if(Chassis_Z_PID.AngleRampFeedback > Chassis_Z_PID.AngleRampTarget) // �����ۼ�
			{
				Chassis_Z_PID.Angle.target = CHASSIS_MECH_yawTargetBoundaryProcess(&Chassis_Z_PID, -Chas_Top_Gyro_Step);
			} 
			else // ���������
			{
				Chassis_Z_PID.AngleRampTarget = 0;
				Chassis_Z_PID.AngleRampFeedback = 0;
			}			
			/* б�º������ۼ���������ֹͻȻ���Ӻܴ������ֵ */
			Chassis_Z_PID.AngleRampFeedback = RAMP_float(Chassis_Z_PID.AngleRampTarget, Chassis_Z_PID.AngleRampFeedback, Chas_Top_Gyro_Step);
			
			Chas_Target_Speed[ Z ] = CHASSIS_Z_Speed_pidCalculate(&Chassis_Z_PID.Angle, (YAW_DIR) * k_Gyro_Chas_Top);
		}
		/* û��С���� */
		else {
			/* С���ݻ�������� */
			top_gyro_dir = 0;
			Chassis_Z_PID.AngleRampTarget = 0;
			Chassis_Z_PID.AngleRampFeedback = 0;
			
			Chassis_Z_PID.Angle.target = CHASSIS_getMiddleAngle() ;	// �ָ����̸���
			Chas_Target_Speed[ Z ] = CHASSIS_Z_Speed_pidCalculate(&Chassis_Z_PID.Angle, (YAW_DIR) * kRc_Gyro_Chas_Spin);
			
		}
		
		/* Z�����ٶ��޷� */
		Chas_Target_Speed[ Z ] = constrain(Chas_Target_Speed[ Z ], -Chas_Spin_Move_Max, Chas_Spin_Move_Max);
	}
	
	/* ������ģʽ�µ�����ϵת�� */
	if(CHASSIS_ifGyroMode() == true) {
		/* ����ƫ���е��ֵ�ĽǶȲ� */
		delta_angle = GIMBAL_getTopGyroAngleOffset() * MECH_2_ANGLE_RADIAN;	//  (8192.f-GIMBAL_getTopGyroAngleOffset()) * MECH_2_ANGLE_RADIAN
		target_speed_x = Chas_Target_Speed[ X ];
		target_speed_y = Chas_Target_Speed[ Y ];
		/* ��ת���� ����̨����ϵ�������ٶ� ת���� ��������ϵ�������ٶ� */
		Chas_Target_Speed[ X ] = cos(delta_angle) * target_speed_x - sin(delta_angle) * target_speed_y;
		Chas_Target_Speed[ Y ] = sin(delta_angle) * target_speed_x + cos(delta_angle) * target_speed_y;
	}
	
	/* ������ת����(ͨ����������������������ת��ƽ�Ƶķ������) */
	if(abs(Chas_Target_Speed[ Z ]) > 800) {	// Ťͷ�ٶ�Խ�죬ƽ���ٶ�Խ��
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
	CHASSIS_omniSpeedCalculate();	
}

/**
 *	@brief	����ң�ز���������̨С����ģʽ
 */
void REMOTE_setTopGyro(void)
{
	static uint8_t rcWheelLockOnFlag = false;
	static uint8_t rcWheelLockOffFlag = false;
	
	/* �������Ͽ��� */
	if(RC_THUMB_WHEEL_VALUE < -650) {
		if(rcWheelLockOnFlag == false) {
			/* ������ģʽ */
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
	
	/* �������¹ر� */
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
 *	@brief	����б�º���
 *	@note	�����������ϵ��1
 *			��ʱδ��������Сϵ��0
 */
float KEY_rampChassisSpeed(int8_t key_state, int16_t *time, uint16_t inc_ramp_step, uint16_t dec_ramp_step)
{
	float fac;
	fac = 0.15 * sqrt( 0.15 * (*time) );	// time�ۼӵ�296.3б�¾����
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
	fac = constrain(fac, 0, 1);
	return fac;
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
 *					\	8 /	----	��ת����	(��תʱ�����ֵ������ٶȲ�ͬ���뾶С�������ٶ�ҪС���뾶��������ٶ�Ҫ��)
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
			//���ֵ���״̬
			//���ε��̸���
			//�޸ĵ����߼�
		}		
		
		/* ������С���� */
		if(CHASSIS_ifTopGyroOpen() == true) {
			Chassis_Z_PID.AngleRampTarget += top_gyro_dir * Chas_Top_Gyro_Step;
			if(Chassis_Z_PID.AngleRampFeedback < Chassis_Z_PID.AngleRampTarget) // �����ۼ�
			{
				Chassis_Z_PID.Angle.target = CHASSIS_MECH_yawTargetBoundaryProcess(&Chassis_Z_PID, Chas_Top_Gyro_Step);
			} 
			else if(Chassis_Z_PID.AngleRampFeedback > Chassis_Z_PID.AngleRampTarget) // �����ۼ�
			{
				Chassis_Z_PID.Angle.target = CHASSIS_MECH_yawTargetBoundaryProcess(&Chassis_Z_PID, -Chas_Top_Gyro_Step);
			} 
			else // ���������
			{
				Chassis_Z_PID.AngleRampTarget = 0;
				Chassis_Z_PID.AngleRampFeedback = 0;
			}			
			/* б�º������ۼ���������ֹͻȻ���Ӻܴ������ֵ */
			Chassis_Z_PID.AngleRampFeedback = RAMP_float(Chassis_Z_PID.AngleRampTarget, Chassis_Z_PID.AngleRampFeedback, Chas_Top_Gyro_Step);
			
			Chas_Target_Speed[ Z ] = CHASSIS_Z_Speed_pidCalculate(&Chassis_Z_PID.Angle, (YAW_DIR) * k_Gyro_Chas_Top);
		}
		/* û��С���� */
		else {
			/* С���ݻ�������� */
			top_gyro_dir = 0;
			Chassis_Z_PID.AngleRampTarget = 0;
			Chassis_Z_PID.AngleRampFeedback = 0;
			
			if(Flag.Chassis.FLAG_goHome == true){
				Chassis.logic = (Chassis_Logic_Names_t)!Chassis.logic;
			}
			
			/* ���õ��̸����߼� */
			Chassis_Z_PID.Angle.target = CHASSIS_getMiddleAngle() ;	
			
			if(Flag.Chassis.FLAG_goHome == false)
				Chas_Target_Speed[ Z ] = CHASSIS_Z_Speed_pidCalculate(&Chassis_Z_PID.Angle, (YAW_DIR) * kRc_Gyro_Chas_Spin);
			else 
				Chas_Target_Speed[ Z ] = 0;
		}
		
		
		/* Z�����ٶ��޷� */
		Chas_Target_Speed[ Z ] = constrain(Chas_Target_Speed[ Z ], -Chas_Spin_Move_Max, Chas_Spin_Move_Max);
	}
	
	/* ������ת����(ͨ����������������������ת��ƽ�Ƶķ������) */
	if(abs(Chas_Target_Speed[ Z ]) > 800) {
		k_rc_z = ((Chas_Spin_Move_Max - (abs(Chas_Target_Speed[ Z ]) - 800)) * (Chas_Spin_Move_Max - (abs(Chas_Target_Speed[ Z ]) - 800)))
							/ (Chas_Spin_Move_Max * Chas_Spin_Move_Max);
		
		k_rc_z = constrain(k_rc_z, 0.f, 1.f);
	} else {
		k_rc_z = 1.f;
	}		
	
	if(IF_KEY_PRESSED_W) {
		timeYBack = 0;	// ����б������
	} 
	if(IF_KEY_PRESSED_S) {
		timeYFront = 0;	// ǰ��б������
	}
	if(IF_KEY_PRESSED_A) {
		timeXRight = 0;	// ����б������
	}
	if(IF_KEY_PRESSED_D) {
		timeXLeft = 0;	// ����б������
	}	
	
	/* ȫ��б�������� */
	Chas_Slope_Move_Fron = Chas_Standard_Move_Max * KEY_rampChassisSpeed(IF_KEY_PRESSED_W, &timeYFront, Time_Inc_Normal, TIME_DEC_NORMAL);
	Chas_Slope_Move_Back = Chas_Standard_Move_Max * KEY_rampChassisSpeed(IF_KEY_PRESSED_S, &timeYBack,  Time_Inc_Normal, TIME_DEC_NORMAL);
	Chas_Slope_Move_Left = Chas_Standard_Move_Max * KEY_rampChassisSpeed(IF_KEY_PRESSED_A, &timeXLeft,  Time_Inc_Normal, TIME_DEC_NORMAL);
	Chas_Slope_Move_Righ = Chas_Standard_Move_Max * KEY_rampChassisSpeed(IF_KEY_PRESSED_D, &timeXRight, Time_Inc_Normal, TIME_DEC_NORMAL);
	
	/* �����ٶȼ��� */
	Chas_Target_Speed[ Y ] = (Chas_Slope_Move_Fron - Chas_Slope_Move_Back)*k_rc_z;	
	Chas_Target_Speed[ X ] = (Chas_Slope_Move_Righ - Chas_Slope_Move_Left)*k_rc_z;
	
	/* ������ģʽ�µ�����ϵת�� */
	if(CHASSIS_ifGyroMode() == true) {
		/* ����ƫ���е��ֵ�ĽǶȲ� */
		delta_angle = GIMBAL_getTopGyroAngleOffset() * MECH_2_ANGLE_RADIAN;	//  (8192.f-GIMBAL_getTopGyroAngleOffset()) * MECH_2_ANGLE_RADIAN
		target_speed_x = Chas_Target_Speed[ X ];
		target_speed_y = Chas_Target_Speed[ Y ];
		if(Flag.Chassis.FLAG_goHome==true)
			delta_angle=0;
		/* ��ת���� ����̨����ϵ�������ٶ� ת���� ��������ϵ�������ٶ� */
		Chas_Target_Speed[ X ] = cos(delta_angle) * target_speed_x - sin(delta_angle) * target_speed_y;
		Chas_Target_Speed[ Y ] = sin(delta_angle) * target_speed_x + cos(delta_angle) * target_speed_y;
	}
	
	/* ���㵱ǰ�������ٶ� */
	Chas_Target_Speed[ ALL ] = abs(Chas_Target_Speed[ X ]) + abs(Chas_Target_Speed[ Y ]) + abs(Chas_Target_Speed[ Z ]);
	
	/* ȫ������㷨 */
	CHASSIS_omniSpeedCalculate();		
}

/**
 *	@brief	���ݰ���ֵ������̨С����ģʽ
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

/* #����# -------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	���̳�ʼ��
 */
void CHASSIS_init(void)
{
	CHASSIS_kalmanCreate();	// �����������˲���
	CHASSIS_initParameter();// ��ʼ�����̲���
}

/**
 *	@brief	���̹�������(�����ٷ���)
 *	@note		�洫�㷨����Ҫ�Ǳ������㷨,ICRA
 */
void CHASSIS_powerLimit(Chassis_Power_t *power, Chassis_PID_t *pid, Judge_Info_t *judge_info)
{
	float kLimit;
	float totalOutput;
	float remain_J;
	static uint16_t judge_err_cnt;
	
	// ��ȡ���役������
	remain_J = judge_info->PowerHeatData.chassis_power_buffer;	
	totalOutput = abs(pid[LEFT_FRON_201].Out) + 
				  abs(pid[RIGH_FRON_202].Out) + 
				  abs(pid[LEFT_BACK_203].Out) + 
				  abs(pid[RIGH_BACK_204].Out);
	
	if(judge_info->data_valid == false) {
		judge_err_cnt++;
		if(judge_err_cnt > 100) {
			power->currentLimit = 9000;	// ����1/4
		}
	} else {
		judge_err_cnt = 0;
		// ʣ�ཹ������С,��ʼ�������,����ϵ��Ϊƽ����ϵ
		if(remain_J < WARNING_REMAIN_POWER) {
			kLimit = (float)(remain_J / WARNING_REMAIN_POWER)
						* (float)(remain_J / WARNING_REMAIN_POWER);
			power->currentLimit =  kLimit * power->maxLimit;
		} else {	// ���������ָ���һ����ֵ
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
 *	@brief	����Ť���ֲڰ�
 */
float CHASSIS_twistTargetCalculate(int16_t maxTarget, int16_t rampTarget)
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
		target_z_speed = RAMP_float(maxTarget, target_z_speed, rampTarget);
	} else if(dir == 0) {
		target_z_speed = RAMP_float((-maxTarget), target_z_speed, rampTarget);
	}
	return target_z_speed;
}

/**
 *	@brief	����ȫ���˶��㷨
 */
void CHASSIS_omniSpeedCalculate(void)
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
 *	@brief	��еģʽ����̨YAW����ֵ(�ۼ�ֵ)�߽紦��
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
 *	@brief	���̳������
 */
void CHASSIS_normalControl(void)
{
	KEY_setChassisSpeed(&RC_Ctl_Info, Chas_Standard_Move_Max, TIME_INC_NORMAL);
	KEY_setTopGyro();
}

/**
 *	@brief	���̴��ʱ��Ŀ���
 *	@note		�����ʱ����̲���
 */
void CHASSIS_buffControl(void)
{
	/*----�����޸�----*/
	Chassis_PID[LEFT_FRON_201].Speed.target = 0;
	Chassis_PID[RIGH_FRON_202].Speed.target = 0;
	Chassis_PID[LEFT_BACK_203].Speed.target = 0;
	Chassis_PID[RIGH_BACK_204].Speed.target = 0;	
}

/* #�����# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	pid�������������
 */
void CHASSIS_pidControlTask(void)
{
	/* PID�ٶȻ����� */
	CHASSIS_Speed_pidCalculate(Chassis_PID, LEFT_FRON_201); 	// ��ǰ - �ٶȻ�
	CHASSIS_Speed_pidCalculate(Chassis_PID, RIGH_FRON_202); 	// ��ǰ - �ٶȻ�
	CHASSIS_Speed_pidCalculate(Chassis_PID, LEFT_BACK_203); 	// ��� - �ٶȻ�
	CHASSIS_Speed_pidCalculate(Chassis_PID, RIGH_BACK_204); 	// �Һ� - �ٶȻ�
	/* �������� */
	//CHASSIS_powerLimit(&Chassis_Power, Chassis_PID, &Judge_Info);
	/* ������� */
	CHASSIS_PID_out(Chassis_PID);	
}

/**
 *	@brief	ң�ؿ��Ƶ�������
 */
void CHASSIS_rcControlTask(void)
{
	//REMOTE_setChassisSpeed();
	REMOTE_TOP_setChassisSpeed();
	REMOTE_setTopGyro();
}

/**
 *	@brief	���̿��Ƶ�������
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
 *	@brief	����ʧ�ر���
 */
void CHASSIS_selfProtect(void)
{
	CHASSIS_stop(Chassis_PID);
	CHASSIS_PID_ParamsInit(Chassis_PID, CHASSIS_MOTOR_COUNT);
	CHASSIS_Z_PID_ParamsInit(&Chassis_Z_PID.Angle);	
}

/**
 *	@brief	���̿�������
 */
void CHASSIS_control(void)
{
	/*----��Ϣ����----*/
	//..can.c��
	CHASSIS_getInfo();
	/*----�����޸�----*/
	if(Flag.Remote.FLAG_mode == RC) {
		CHASSIS_rcControlTask();
	} else if(Flag.Remote.FLAG_mode == KEY) {
		CHASSIS_keyControlTask();
	}
	/*----�������----*/
	CHASSIS_pidControlTask();
}