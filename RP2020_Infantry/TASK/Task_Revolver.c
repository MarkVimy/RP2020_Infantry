/**
 * @file        Task_Revolver.c
 * @author      MaWeiming
 * @Version     V1.0
 * @date        10-October-2019
 * @brief       This file includes the Revolver(���̵��) external functions 
 */
 
/**
 *	# m2006 P36 ���̵��
 *	���������ת�ٴ����20800����
 */
 
/* Includes ------------------------------------------------------------------*/
#include "Task_Revolver.h"

#include "can.h"
#include "my_app.h"

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#define REVOLVER_SPEED_PID_IOUT_MAX		250
#define REVOLVER_SPEED_PID_OUT_MAX		9999
#define REVOLVER_ANGLE_PID_IOUT_MAX		2000
#define REVOLVER_ANGLE_PID_OUT_MAX		6000

#define REVOLVER_STUCK_PID_OUT			4000
#define REVOLVER_STUCK_SPEED				60
#define REVOLVER_STUCK_TIME					100
#define REVOLVER_TURNBACK_TIME			100

#define	TRIPLE_SHOOT_PRESSED_TIME		TIME_STAMP_250MS

#define LEVEL_COUNT		3
#define LEVEL_1				0
#define LEVEL_2				1
#define LEVEL_3				2


/* ## Global variables ## ----------------------------------------------------*/
Revolver_PID_t Revolver_PID = {
	/* �ٶȻ� */
	.Speed.kp = 9.2,		
	.Speed.ki = 0,
	.Speed.kd = 0,	
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
	.Angle.kp = 0.26,		
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
};

/* # Revolver # */
Revolver_Info_t Revolver = {
	.state = REVO_STATE_OFF,
	.pidMode = REVO_POSI_MODE,
	.action = SHOOT_NORMAL,
	.Shoot.freq = 8,	// Ĭ����Ƶ8
	.Shoot.num = 0,	// �����䵯��
	.Shoot.heat_cooling_rate = 0,	// 17mmǹ��������ȴֵ
	.Shoot.heat_real = 0, // 17mmǹ�ܳ�ʼ����
	.Shoot.heat_limit = 240,	// Ĭ��ǹ����������Ϊ240
	.Shoot.stuck_count = 0,// ��������
	
	.buff.lost_time = 0,
	.buff.lost_time_boundary = TIME_STAMP_70MS,
	.buff.lock_time = 0,
	.buff.lock_time_boundary = TIME_STAMP_80MS,
	.buff.change_armor_delay = 0,
	.buff.change_armor_delay_boundary = TIME_STAMP_50MS,
	.buff.respond_interval = TIME_STAMP_800MS,	// TIME_STAMP_800MS ���Ե�ʱ����Ҫ
	.buff.change_armor = false,
	.buff.fire = false,
};

/* ǹ���������� */
uint16_t REVO_HEAT_LIMIT[LEVEL_COUNT] = {180, 240, 300};

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* API functions -------------------------------------------------------------*/
/* #������# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	���̵��PID������ʼ��
 */
void REVOLVER_pidParamsInit(Revolver_PID_t *pid)
{
	/* �ٶȻ��������� */
	pid->Speed.target = 0;
	pid->Speed.feedback = 0,
	pid->Speed.erro = 0;
	pid->Speed.last_erro = 0;
	pid->Speed.integrate = 0;
	pid->Speed.pout = 0;
	pid->Speed.dout = 0;
	pid->Speed.iout = 0;
	pid->Speed.out = 0;

	/* λ�û��������� */
	pid->Angle.target = 0;
	pid->Angle.feedback = 0,
	pid->Angle.erro = 0;
	pid->Angle.last_erro = 0;
	pid->Angle.integrate = 0;
	pid->Angle.pout = 0;
	pid->Angle.dout = 0;
	pid->Angle.iout = 0;
	pid->Angle.out = 0;
	
	pid->AngleRampBuffer = 0;
	pid->Out = 0;
}

/**
 *	@brief	���̵��ֹͣ���
 */
void REVOLVER_stop(Revolver_PID_t *pid)
{
	int16_t pidOut[4] = {0, 0, 0, 0};
	
	/* �ڻ��ٶȻ�������� */
	Revolver_PID.Speed.out = 0;
	Revolver_PID.Out = 0;
	
#if REVOLVER_ID > 0x204	
	CAN2_send(0x1FF, pidOut);
#else
	CAN2_send(0x200, pidOut);
#endif
}

/**
 *	@brief	���̵���ٶȻ�
 */
void REVOLVER_Speed_pidCalculate(Revolver_PID_t *pid)
{
	pid->Speed.erro = pid->Speed.target - pid->Speed.feedback;
	pid->Speed.integrate += pid->Speed.erro;
	/* # ���Բ��� */
	pid->Speed.integrate = constrain(pid->Speed.integrate, -REVOLVER_SPEED_PID_IOUT_MAX, REVOLVER_SPEED_PID_IOUT_MAX);
		
	/* Pout */
	pid->Speed.pout = pid->Speed.kp * pid->Speed.erro;
	/* Iout */
	pid->Speed.iout = pid->Speed.ki * pid->Speed.integrate * 0.002f; // ģ��19��������(0.002f��2ms����˼)
	/* Iout Limits */
	pid->Speed.iout = constrain(pid->Speed.iout, -REVOLVER_SPEED_PID_IOUT_MAX, REVOLVER_SPEED_PID_IOUT_MAX);
	/* Dout*/
	pid->Speed.dout = pid->Speed.kd * (pid->Speed.erro - pid->Speed.last_erro)/0.002f;
	/* Record Last Error */
	pid->Speed.last_erro = pid->Speed.erro;		
	
	/* Total PID Output*/
	pid->Speed.out = pid->Speed.pout + pid->Speed.iout + pid->Speed.dout;
	/* Total PID Output Limits */
	pid->Speed.out = constrain(pid->Speed.out, -REVOLVER_SPEED_PID_OUT_MAX, REVOLVER_SPEED_PID_OUT_MAX);
	/* �ڻ��ٶȻ�������� */
	pid->Out = pid->Speed.out;
}

/**
 *	@brief	���̵��λ�û�
 */
void REVOLVER_Angle_pidCalculate(Revolver_PID_t *pid)
{	
	pid->Angle.erro = pid->Angle.target - pid->Angle.feedback;
	/* �������п������˲���������Ƶ�ط����� */
	//pid->Angle.erro = KalmanFilter(&Gimbal_kalmanError[Flag.Gimbal.FLAG_pidMode][MOTORx], pid->Angle.erro);
	pid->Angle.integrate += pid->Angle.erro;

	/* Pout */
	pid->Angle.pout = pid->Angle.kp * pid->Angle.erro;
	/* Iout */
	pid->Angle.iout = pid->Angle.ki * pid->Angle.integrate * 0.002f; // ģ��19��������(0.002f��2ms����˼)
	/* Iout Limits */
	//pid->Angle.iout = constrain(pid->Angle.iout, -REVOLVER_ANGLE_PID_IOUT_MAX, REVOLVER_ANGLE_PID_IOUT_MAX);
	/* Dout*/
	pid->Angle.dout = pid->Angle.kd * (pid->Angle.erro - pid->Angle.last_erro)/0.002f;
	/* Record Last Error */
	pid->Angle.last_erro = pid->Angle.erro;

	/* Total PID Output*/
	pid->Angle.out = pid->Angle.pout + pid->Angle.iout + pid->Angle.dout;
	/* Total PID Output Limits */
	//pid->Angle.out = constrain(pid->Angle.out, -REVOLVER_ANGLE_PID_OUT_MAX, REVOLVER_ANGLE_PID_OUT_MAX);
}

/**
 *	@brief	���̵��PID���������
 *	@note		# ���ID�Ÿ�����ʱ����Ҫ�޸ı�ʶ���ͷ��͵�λ��(����ʵ��)
 *					��ʱ����ת��е�ǶȻ�����
 *	@direction
 *					��ʱ�� -> ��е�Ƕȡ�,ת��+
 *					˳ʱ�� -> ��е�Ƕȡ�,ת��-
 */
void REVOLVER_pidOut(Revolver_PID_t *pid)
{
	int16_t pidOut[4] = {0, 0, 0, 0};
	uint8_t pidSn = ((REVOLVER_ID - 0x201) % 4);	// ((0x207-0x201)%4)
	
	/* CAN���͵���ֵ */
	if(BitMask.Revolver.BM_rxReport & REVOLVER_BM_RX_REPORT) {
		pidOut[pidSn] = (int16_t)pid->Out;
	} else {
		pidOut[pidSn] = 0;	// ʧ������ж��
	}
	
#if REVOLVER_ID > 0x204
	CAN2_queueSend(0x1FF, pidOut);
#else
	CAN2_queueSend(0x200, pidOut);
#endif	
}

/**
 *	@brief	�ٶȻ���������ʽ(��ʱ�ж�+��ת����)
 */
void REVOLVER_Speed_bulletStuck(Revolver_PID_t *pid, Revolver_Info_t *info)
{
	static uint16_t stuck_time = 0;
	static uint16_t turnback_time = 0;
	static uint8_t	stuck_recover_flag = 0;
	
	if(stuck_recover_flag == 0) {	// û�����������ָ�����
		if(pid->Speed.out > REVOLVER_STUCK_PID_OUT
		   && pid->Speed.feedback < REVOLVER_STUCK_SPEED) {	// PID���������ٶȷ�����С����Ϊ����
			stuck_time++;	
		} else {
			stuck_time = 0;
		}
		if(stuck_time > REVOLVER_STUCK_TIME) {	// ������������60ms
			stuck_time = 0;
			info->Shoot.stuck_count++;
			stuck_recover_flag = 1;
			info->Shoot.num = 0;	// ���δ�ָ��
		}
	} else {	// �����ָ�
		pid->Speed.target = -4000;
		turnback_time++;
		if(turnback_time > REVOLVER_TURNBACK_TIME) {	// ��ת���
			turnback_time = 0;
			stuck_recover_flag = 0;
		}
	}
}

/**
 *	@brief	λ�û���������ʽ(��ʱ�ж�+��ת����)
 */
void REVOLVER_Angle_bulletStuck(Revolver_PID_t *pid, Revolver_Info_t *info)
{
	static uint16_t stuck_time = 0;
	static uint16_t turnback_time = 0;
	static uint8_t	stuck_recover_flag = 0;
	
	if(stuck_recover_flag == 0) {	// û�����������ָ�����
		if(pid->Speed.out > REVOLVER_STUCK_PID_OUT
		   && pid->Speed.feedback < REVOLVER_STUCK_SPEED) {	// PID���������ٶȷ�����С����Ϊ����
			stuck_time++;	
		} else {
			stuck_time = 0;
		}
		if(stuck_time > REVOLVER_STUCK_TIME) {	// ������������60ms
			stuck_time = 0;
			info->Shoot.stuck_count++;
			stuck_recover_flag = 1;
			info->Shoot.num = 0;	// ���δ�ָ��
			//pid->Angle.target = ANGLE_AN_BULLET;	// ��ת1��
			pid->AngleRampBuffer = pid->Angle.target - ANGLE_AN_BULLET;
		}
	} else {	// �����ָ�
		turnback_time++;
		if(turnback_time > REVOLVER_TURNBACK_TIME) {	// ��ת���
			pid->Angle.target = pid->Angle.feedback;
			pid->AngleRampBuffer = pid->Angle.target;
			turnback_time = 0;
			stuck_recover_flag = 0;
		}
	}
}

/* #��Ϣ��# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief ���ò��̵Ŀ�����Ϊ
 */
void REVOLVER_setAction(Revolver_Action_t action)
{
	Revolver.action = action;
}

/**
 *	@brief	����ʵʱ���ʱ��
 */
uint32_t REVOLVER_getRealShootTime(void)
{
	return Revolver.Shoot.real_time;
}

/**
 *	@brief	������+1
 */
void REVOLVER_addShootCount(void)
{
	Revolver.Shoot.total_count++;
}


/* #Ӧ�ò�# ---------------------------------------------------------------------------------------------------------------------------------------*/
/* #ң��# -----------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	ң�����ò��̵��ת��(����������)
 *	@note	SW1_DOWN(ǰ����Ħ���ֿ���)
 */
void REMOTE_setRevolverShoot(RC_Ctl_t *remoteInfo)
{
	static uint8_t prev_sw1 = RC_SW_DOWN;
	//static uint8_t prev_sw2 = RC_SW_MID;
	uint8_t	sw1, sw2;
	sw1 = remoteInfo->rc.s1;
	sw2 = remoteInfo->rc.s2;
	
	if(FRICTION_ifReady()) {
		if(Revolver.pidMode == REVO_POSI_MODE) {	// ����ģʽ
			if(sw1 == RC_SW_DOWN && prev_sw1 != RC_SW_DOWN) 
			{
				Revolver.Shoot.freq = 8;
				Revolver.Shoot.num += REVOLVER_heatLimit(&Revolver, 1);
			}
		} else if(Revolver.pidMode == REVO_SPEED_MODE) {	// ����ģʽ
			if(sw1 == RC_SW_DOWN) {
				Revolver.state = REVO_STATE_ON;
				Revolver.Shoot.freq = 15;
				Revolver_PID.Speed.target = SPEED_AN_BULLET_PER_SECOND * Revolver.Shoot.freq;	// ��/s
			} else {
				Revolver.state = REVO_STATE_OFF;
				Revolver_PID.Speed.target = 0;
			}
		}
	}
	prev_sw1 = sw1;
	
	if(sw2 == RC_SW_MID) {
		/* �ٶȻ�->λ�û�*/
		if(Revolver.pidMode == REVO_SPEED_MODE) {
			Revolver.pidMode = REVO_POSI_MODE;
			Revolver_PID.Angle.target = Revolver_PID.Angle.feedback;	// ��ֹ�л�ģʽ��ʱ��һ��
			Revolver_PID.AngleRampBuffer = Revolver_PID.Angle.target;
		}
	} else if(sw2 == RC_SW_DOWN) {
		/* λ�û�->�ٶȻ� */
		if(Revolver.pidMode == REVO_POSI_MODE) {
			Revolver.pidMode = REVO_SPEED_MODE;
			Revolver.state = REVO_STATE_OFF;
			Revolver_PID.Speed.target = 0;
			//Revolver_PID.Speed.out = 0;	// s1_down,s2_down -> s2_mid -> s1_mid -> s2_down => ���̻���תһС��(С��һ��)
		}
	}
}

/* #�������# -------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	�������ò��̵��ת��(����������)
 *	@note	    ���������
 *			���������������(�൱�ڿ������ε���)
 *			
 *			# Loop Time:5ms
 *			timePressed�����ֵΪ +32767(32767*5 = 163.835s)
 */

/*
	����		- ������
	������� - ����
	������� - ����
	���		- �Զ�����
	����		- �ֶ�����
*/

void KEY_setRevolverShoot(RC_Ctl_t *remoteInfo)
{	
//	/* Ħ���ֿ�����ʱ�����Ӧ���� */
//	if(FRICTION_ifReady()) 
//	{		
//		/* ����������ʱ�ۼ�ʱ�� */
//		if(IF_MOUSE_PRESSED_LEFT)
//		{
//			if(Time_Pressed_Shoot < TIME_STAMP_6000MS)	// ��ֹ��ʱ�䰴�������
//					Time_Pressed_Shoot++;
//		}
//		
//		if(IF_KEY_PRESSED_B && !IF_MOUSE_PRESSED_LEFT & !IF_KEY_PRESSED_B)
//		{
//			/* �Ƽ�ģʽ */
//			Revolver.pidMode = REVO_POSI_MODE;
//			Revolver.action = SHOOT_HIGH_F_LOW_S;
//		}
//		else if(GIMBAL_ifBuffMode())
//		{
//			/* ���ģʽ */
//			Revolver.pidMode = REVO_POSI_MODE;
//			Revolver.action = SHOOT_BUFF;
//			/* �л�ģʽʱ��������Ӧʱ�� */
//			Time_Pressed_Shoot = 0;
//		}
//		else if(GIMBAL_ifAutoMode())
//		{
//			/* ����ģʽ */
//			Revolver.pidMode = REVO_POSI_MODE;
//			Revolver.action = SHOOT_AUTO;
//			/* �л�ģʽʱ��������Ӧʱ�� */
//			Time_Pressed_Shoot = 0;
//		}
//		else if(IF_MOUSE_PRESSED_LEFT && !IF_KEY_PRESSED_B && !IF_KEY_PRESSED_Z) 
//		{
//			/* ������ģʽ */
//			if(Time_Pressed_Shoot >= TRIPLE_SHOOT_PRESSED_TIME) 
//			{	// �״�������Ҫ�������250ms
//				//Time_Pressed_Shoot = 0;
//				Revolver.pidMode = REVO_SPEED_MODE;	// �ٶȻ�
//				Revolver.action = SHOOT_TRIPLE;		// ������
//				Revolver.state = REVO_STATE_ON;		// ����������ת
//			} 
//		} 
//		else if((Time_Pressed_Shoot > TIME_STAMP_10MS && Time_Pressed_Shoot < TRIPLE_SHOOT_PRESSED_TIME)
//				&& !IF_MOUSE_PRESSED_LEFT && !IF_KEY_PRESSED_B && !IF_KEY_PRESSED_Z)
//		{
//			/* ����ģʽ */
//			Time_Pressed_Shoot = 0;
//			Revolver.pidMode = REVO_POSI_MODE;
//			Revolver.action = SHOOT_SINGLE;
//		}
//		else 
//		{
//			/* �������ģʽ */
//			/* �ٶȻ���λ�û� */
//			if(Revolver.pidMode == REVO_SPEED_MODE) {
//				Revolver_PID.Angle.target = Revolver_PID.Angle.feedback;	// ��ֹ�л�ģʽ��ʱ��һ��
//				Revolver_PID.AngleRampBuffer = Revolver_PID.Angle.target;
//			}
//			Time_Pressed_Shoot = 0;
//			Revolver.pidMode = REVO_POSI_MODE;
//			Revolver.action = SHOOT_NORMAL;
//			Revolver.state = REVO_STATE_OFF;		// ֹͣ������ת			
//		}
//	}
//	/* δ����Ħ��������ָ��*/
//	else	
//	{
//		Time_Pressed_Shoot = 0;
//		Revolver.pidMode = REVO_POSI_MODE;
//		Revolver.action = SHOOT_NORMAL;		
//	}
}

/* #����# -----------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief 	��ϵ�ǰ�������������󷢵���(0<=allow_num<=target_num)
 *	@return allow_num( 0 ~ target_num)
 */
uint8_t REVOLVER_heatLimit(Revolver_Info_t *info, uint8_t target_num)
{
	uint16_t remain_heat;	// ʣ������
	uint8_t  allow_num;		// ����ķ�������
	
	if(target_num == 0)	// ��ֹ����
		return 0;
	
	remain_heat = info->Shoot.heat_limit - info->Shoot.heat_real;
	allow_num = remain_heat / HEAT_AN_BULLET;	// ��ǰ�������������󷢵���
	if(allow_num > 0) {
		while(target_num) {
			if(allow_num >= target_num) {
				allow_num = target_num;
				break;
			}
			target_num--;
		}
	}
	return allow_num;
}

/**
 *	@brief	���̵��PID������
 *	@note	(ǰ����Ħ���ֿ���)
 */
uint16_t js_shoot_num = 0;
void REVOLVER_pidControlTask(void)
{
	/* δ��Ħ���� */
	if(FRICTION_ifOpen() == false) {
		g_Revolver_Motor_Info.angle_sum = 0;	// �ۼӽǶ�ֵ����
		REVOLVER_pidParamsInit(&Revolver_PID);
		REVOLVER_stop(&Revolver_PID);
	}
	else {
		if(Revolver.pidMode ==  REVO_SPEED_MODE) // ����ģʽ
		{	
			if(Revolver.state == REVO_STATE_ON) {	// ����������ת
				Revolver_PID.Speed.target = SPEED_AN_BULLET_PER_SECOND * Revolver.Shoot.freq;	// ÿ��һ�ŵ�ת�ٳ�����Ƶ
			} else if(Revolver.state == REVO_STATE_OFF) {	// �رղ�����ת
				Revolver_PID.Speed.target = 0;
			}
			/* �����ж� */
			REVOLVER_Speed_bulletStuck(&Revolver_PID, &Revolver);
			/* �ٶȻ����� */
			REVOLVER_Speed_pidCalculate(&Revolver_PID);
		} 
		else if(Revolver.pidMode ==  REVO_POSI_MODE) // ����ģʽ
		{	
			/* �����ж� */
			REVOLVER_Angle_bulletStuck(&Revolver_PID, &Revolver);	
			if(Revolver.Shoot.num > 0) {	// ��Ҫ������
				js_shoot_num++;
				Revolver.Shoot.num--;
				Revolver_PID.AngleRampBuffer += ANGLE_AN_BULLET;
				Revolver.Shoot.real_time = xTaskGetTickCount();
			}
			/* λ�û�����(��Ŀ�껺���ƽ�����Ŀ��) */
			Revolver_PID.Angle.target = RAMP_float(Revolver_PID.AngleRampBuffer, Revolver_PID.Angle.target, ANGLE_AN_BULLET_RAMP);
			REVOLVER_Angle_pidCalculate(&Revolver_PID);
			Revolver_PID.Speed.target = Revolver_PID.Angle.out;
			REVOLVER_Speed_pidCalculate(&Revolver_PID);
		}
		/*----�������----*/
		REVOLVER_pidOut(&Revolver_PID);	
	} 
}

/**
 *	@brief	���̵���������
 *	@note		Ĭ�ϲ���
 */
static uint16_t Time_Pressed_Shoot = 0;	
void REVOLVER_normalControl(Revolver_Info_t *info)
{
	/* Ħ����û�о������δ�ָ�� */
	if(FRICTION_ifReady() == false) {
		Time_Pressed_Shoot = 0;
		Revolver.pidMode = REVO_POSI_MODE;
		Revolver.action = SHOOT_NORMAL;
		return;
	}
	
	/* ����������ʱ�ۼ�ʱ�� */
	if(IF_MOUSE_PRESSED_LEFT)
	{
		if(Time_Pressed_Shoot < TIME_STAMP_6000MS)	// ��ֹ��ʱ�䰴�������
				Time_Pressed_Shoot++;
	}

	if( !IF_MOUSE_PRESSED_LEFT
			&& (Time_Pressed_Shoot > TIME_STAMP_10MS)
			&& (Time_Pressed_Shoot < TRIPLE_SHOOT_PRESSED_TIME) ) 
	{
		/* ���� */
		Revolver.pidMode = REVO_POSI_MODE;	// λ�û�
		Revolver.state = REVO_STATE_OFF;		// �ز�����ת
		REVOLVER_singleShootControl(info);
	}
	else if( (Time_Pressed_Shoot >= TRIPLE_SHOOT_PRESSED_TIME) )
	{
		/* ���� */
		Revolver.pidMode = REVO_SPEED_MODE;	// �ٶȻ�
		Revolver.state = REVO_STATE_ON;			// ��������ת
		REVOLVER_tripleShootControl(info);
	}
	else
	{
		/* ��ֹ���� */
		/* �ٶȻ���λ�û� */
		if(Revolver.pidMode == REVO_SPEED_MODE) {
			Revolver_PID.Angle.target = Revolver_PID.Angle.feedback;	// ��ֹ�л�ģʽ��ʱ��һ��
			Revolver_PID.AngleRampBuffer = Revolver_PID.Angle.target;
		}
		Time_Pressed_Shoot = 0;
		Revolver.pidMode = REVO_POSI_MODE;
		Revolver.action = SHOOT_NORMAL;
		Revolver.state = REVO_STATE_OFF;		// ֹͣ������ת
	}	
}

/**
 *	@brief	���̵����������
 *	@note		����
 */
void REVOLVER_singleShootControl(Revolver_Info_t *info)
{
	portTickType  current_time = 0;
	static uint32_t  respond_time = 0;	//��Ӧ�����ʱ
	
	current_time = xTaskGetTickCount();
	
	/* # ��ô�������ܵ������Ƶ? */
	info->Shoot.interval = TIME_STAMP_1000MS / info->Shoot.freq;
	
	if((respond_time < current_time)
			&& (info->Shoot.num == 0))
	{
		respond_time = current_time + info->Shoot.interval;
		info->Shoot.num += REVOLVER_heatLimit(&Revolver, 1);
	}

	/* �ɿ�����󷵻س������ */
	if(!IF_MOUSE_PRESSED_LEFT) {
		Time_Pressed_Shoot = 0;
		Revolver.action = SHOOT_NORMAL;
	}
}

/**
 *	@brief	���̵������������
 *	@note		������
 */
void REVOLVER_tripleShootControl(Revolver_Info_t *info)
{
//	/* # ��ô�������ܵ������Ƶ? */
//	if(info->Shoot.heat_real >= 160) {
//		info->Shoot.freq = 14;
//	} else {
//		info->Shoot.freq = 8;
//	}	
	
	/* �ɿ�����󷵻س������ */
	if(!IF_MOUSE_PRESSED_LEFT) {	
		Time_Pressed_Shoot = 0;
		Revolver.action = SHOOT_NORMAL;
	}
}

/**
 *	@brief	�ؼ�֮�����滨
 *	@note		# RM2020�Կ�����������ǹ���������ӵ����ٶ��޹أ��̶�һ��Ϊ10����
 */
void REVOLVER_comboShootControl(Revolver_Info_t *info)
{
	portTickType  current_time = 0;
	static uint32_t  respond_time = 0;	//��Ӧ�����ʱ	
	
	current_time = xTaskGetTickCount();
	
	/* # ��ô�������ܵ������Ƶ? */
	info->Shoot.interval = TIME_STAMP_1000MS/15;	//���һ��15��
	
	if((respond_time < current_time) 
			&& (info->Shoot.num == 0))
	{
		respond_time = current_time + info->Shoot.interval;
		info->Shoot.num += REVOLVER_heatLimit(&Revolver, 1);
	}	
	
	/* �ɿ�B���󷵻س������ */
	if(!IF_KEY_PRESSED_B) {	
		Time_Pressed_Shoot = 0;
		Revolver.action = SHOOT_NORMAL;
	}
}

/**
 *	@brief	���̵���������
 *	@note	���
 *			0.5s��ʱ���л�װ�װ�
 */
uint8_t test_buff_bullet = 0;
portTickType first_into_armor_4 = 0;
uint32_t	 respond_into_armor_4 = 0;
void REVOLVER_buffShootControl(Revolver_Info_t *info)
{
	portTickType  current_time = 0;
	static uint32_t  respond_time = 0;	// ��Ӧ�����ʱ
	static uint16_t manual_pressed_left = 0;	// �ֶ��������Ӧ

	/* #λ�û� */
	Revolver.pidMode = REVO_POSI_MODE;
	Revolver.state = REVO_STATE_OFF;
	
	current_time = xTaskGetTickCount();

	/* �Ƿ����װ�װ� */
	if(VISION_getFlagStatus(VISION_FLAG_CHANGE_ARMOR) == true) {
		info->buff.change_armor = true;
	}
	
	/* �Ƿ��л������Ŀ�װ�װ� */
	info->buff.change_armor_4 = VISION_getFlagStatus(VISION_FLAG_CHANGE_ARMOR_4);
	
	if(test_buff_bullet == 0)
	{
		/* �ɿ��Ҽ��Զ��� */
		if(!IF_MOUSE_PRESSED_RIGH)
		{
			manual_pressed_left = 0;
			/* δʶ�𵽷� */
			if(VISION_getFlagStatus(VISION_FLAG_LOCK_BUFF) == false)
			{
				info->buff.lost_time++;
				if(info->buff.lost_time > info->buff.lost_time_boundary) {
					info->buff.fire = false;
					info->buff.lock_time = 0;
				}
				
	//			if(info->buff.change_armor == true)
	//			{
	//				info->buff.lock_time = 0;
	//				info->buff.lost_time = 0;
	//			}
			}
			/* ʶ�𵽷� */
			else
			{
				info->buff.lost_time = 0;
				
				/* ���л���װ�װ��ȶ�һ��ʱ�䲻�� */
				if(info->buff.change_armor_delay > 0)
				{
					info->buff.change_armor_delay--;
				}

				if(info->buff.change_armor_delay == 0)		
				{					
					/* ���л���װ�װ� */
					if(info->buff.change_armor == true)
					{
						info->buff.change_armor = false;
						info->buff.fire = false;	// ��ֹ���
						info->buff.lock_time = 0;	// ����װ�װ�ʱ������
						respond_time = current_time-1;// ˢ�����ʱ��
						info->buff.change_armor_delay = info->buff.change_armor_delay_boundary;	// �л������ȶ�һ��ʱ��
					} 
					/* δ�л�װ�װ� */
					else if(info->buff.change_armor == false)
					{
						/* ��̨�Ѹ���Ŀ�� */
						if(GIMBAL_BUFF_chaseReady()) 
						{
							info->buff.lock_time++;
							/* ��̨�ȶ�����װ�װ� */
							if(info->buff.lock_time > info->buff.lock_time_boundary)
							{
								/* ���Կ��� */
								info->buff.fire = true;
							}
						}
						/* ��̨���Ŀ�� */
						else
						{
							/* ��ֹ���� */
							info->buff.lock_time = 0;
							info->buff.fire = false;
						}
					}
					
					/* ��Ӧʱ���ѵ� & ���Կ��� & �Ǹ��л�װ�װ� & ��һ���ӵ��Ѵ��ȥ */
					if((respond_time < current_time) &&
						info->buff.fire == true && 
						info->buff.change_armor == false &&
						info->Shoot.num == 0) 
					{
						respond_time = current_time + info->buff.respond_interval;
						info->Shoot.num += REVOLVER_heatLimit(&Revolver, 1);
					}
				}
			}
			
			/* # ���Ե�ʱ��Ĭ�����ﲦһ���ʹ�һ�ų�ȥ(֮��������ò���ϵͳ�����ٷ������ж��Ƿ����) */
			if(info->buff.change_armor_4 == true) {
				if(info->Shoot.num != 0) {
					if(first_into_armor_4 == 0) {
						first_into_armor_4 = xTaskGetTickCount();
					}
				}
				if(first_into_armor_4 != 0) {
					respond_into_armor_4 = xTaskGetTickCount();
					if(respond_into_armor_4 > first_into_armor_4 + TIME_STAMP_100MS) {	// �ʵ���ʱģ�ⷢ���ӳ�
						VISION_setFlagStatus(VISION_FLAG_SHOOT_ARMOR_4);
					}
				}
			} else {
				VISION_clearFlagStatus(VISION_FLAG_SHOOT_ARMOR_4);
				first_into_armor_4 = 0;
			}
		}
		/* ��ס�Ҽ��ӹܴ� */
		else
		{
			/* ��ֹ�ֶ��л����Զ���ʱ�����ϴ� */
			info->buff.fire = false;	// ��ֹ�Զ����
			info->buff.lock_time = 0;	// ����װ�װ�ʱ������
			if(IF_MOUSE_PRESSED_LEFT) 
			{
				if(manual_pressed_left < TIME_STAMP_10S)	// ��ֹ��ʱ�䰴�������
					manual_pressed_left++;
			}
			else
			{
				if(manual_pressed_left > TIME_STAMP_10MS) {	// ���̧���Ŵ��
					info->Shoot.num += REVOLVER_heatLimit(&Revolver, 1);
				}
				
				/* # ���Ե�ʱ��Ĭ�����ﲦһ���ʹ�һ�ų�ȥ(֮��������ò���ϵͳ�����ٷ������ж��Ƿ����) */
				if(info->buff.change_armor_4 == true) {
					if(info->Shoot.num != 0) {
						if(first_into_armor_4 == 0) {
							first_into_armor_4 = xTaskGetTickCount();
						}
					}
					if(first_into_armor_4 != 0) {
						respond_into_armor_4 = xTaskGetTickCount();
						if(respond_into_armor_4 > first_into_armor_4 + TIME_STAMP_100MS) {// �ʵ���ʱģ�ⷢ���ӳ�
							VISION_setFlagStatus(VISION_FLAG_SHOOT_ARMOR_4);
						}
					}
				} else {
					VISION_clearFlagStatus(VISION_FLAG_SHOOT_ARMOR_4);
					first_into_armor_4 = 0;
				}
				
				manual_pressed_left = 0;
			}
		}
	}
	else
	{
		/* ��ֹ�ֶ��л����Զ���ʱ�����ϴ� */
		info->buff.fire = false;	// ��ֹ�Զ����
		info->buff.lock_time = 0;	// ����װ�װ�ʱ������
		if(IF_MOUSE_PRESSED_LEFT) 
		{
			if(manual_pressed_left < TIME_STAMP_10S)	// ��ֹ��ʱ�䰴�������
				manual_pressed_left++;
		}
		else
		{
			if(manual_pressed_left > TIME_STAMP_10MS) {	// ���̧���Ŵ��
				info->Shoot.num += REVOLVER_heatLimit(&Revolver, 1);
			}
			manual_pressed_left = 0;
		}		
	}
}

/**
 *	@brief	���̵���������
 *	@note		����Ԥ��
 */
void REVOLVER_autoShootControl(Revolver_Info_t *info)
{
	static portTickType current_time     = 0;
	static uint32_t respond_time_Stop    = 0;//��Ӧ�����ʱ����ֹ
	static uint32_t respond_time_MobiPre = 0;//��Ӧ�����ʱ���ƶ�Ԥ��	

	/* #λ�û� */
	Revolver.pidMode = REVO_POSI_MODE;
	Revolver.state = REVO_STATE_OFF;
	
	current_time = xTaskGetTickCount();
	
	/* ������Ƶ������Ӧ��� */
	info->Shoot.interval = TIME_STAMP_1000MS / info->Shoot.freq;
	
	/* �����ƶ�Ԥ���ѿ��� */
	if( GIMBAL_ifAutoMobiPre() == true) {
		//info->Shoot.interval = TIME_STAMP_1000MS/10;
		/* ���鿪������ */
		if(GIMBAL_AUTO_chaseReady() == true				// Ԥ�⵽λ
				&& respond_time_MobiPre < current_time	// �����Ӧ
					&& info->Shoot.num == 0							// ��һ���Ѵ��
						&& IF_MOUSE_PRESSED_LEFT)					// �������
		{
			respond_time_MobiPre = current_time + info->Shoot.interval;
			info->Shoot.num += REVOLVER_heatLimit(&Revolver, 1);
		}
		else{
			info->Shoot.num = 0;	// Ԥ��δ��λ����ֹ��
		}
	}
	/* �����ƶ�Ԥ��δ���� */
	else if( GIMBAL_ifAutoMobiPre() == false) {
		//info->Shoot.interval = TIME_STAMP_1000MS/5;
		/* ���鿪������ */
		if(GIMBAL_AUTO_chaseReady() == true				// Ԥ�⵽λ
				&& respond_time_Stop < current_time			// �����Ӧ
					&& info->Shoot.num == 0							// ��һ���Ѵ��
						&& IF_MOUSE_PRESSED_LEFT)					// �������
		{
			respond_time_Stop = current_time + info->Shoot.interval;
			info->Shoot.num += REVOLVER_heatLimit(&Revolver, 1);
		}
		else{
			info->Shoot.num = 0;	// Ԥ��δ��λ����ֹ��
		}		
	}
}

/* #�����# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	���̵����ȡ����ϵͳ��Ϣ
 */
void REVOLVER_recordJudgeInfo(Revolver_Info_t *revo_info, Judge_Info_t *judge_info)
{
	if(judge_info->data_valid == true) {
		revo_info->Shoot.heat_real = judge_info->PowerHeatData.shooter_heat0;
		revo_info->Shoot.heat_limit = judge_info->GameRobotStatus.shooter_heat0_cooling_limit;
		revo_info->Shoot.heat_cooling_rate = judge_info->GameRobotStatus.shooter_heat0_cooling_rate;	// ����<20%HP,��ȴ���ʷ���
	} else {
		//..�ݲ�����
	}
	
	switch(judge_info->GameRobotStatus.robot_level)
	{
		// 1������
		case 1:
			revo_info->Shoot.freq = 8;
			break;
		// 2������
		case 2:
			revo_info->Shoot.freq = 10;
			break;
		// 3������
		case 3:
			revo_info->Shoot.freq = 12;
			break;
		// ��ֹ��bug
		default:
			revo_info->Shoot.freq = 8;
			break;
	}
}

/**
 *	@brief	������̨ģʽ��������ģʽ
 *	@note		# ��ԭ�еĿ����߼����ڳ�ͻ����ʱ����
 */
void REVOLVER_getInfo(void)
{
	static Gimbal_Mode_t now_mode, prev_mode;
	
	/* ��ȡ����ϵͳ��Ϣ */
	REVOLVER_recordJudgeInfo(&Revolver, &Judge_Info);
	
	/* ��������ģʽ */
	now_mode = GIMBAL_getMode();
	switch(now_mode)
	{
		case GIMBAL_MODE_NORMAL:
		case GIMBAL_MODE_RELOAD_BULLET:
			/*
				��̨����Ͷ�λʱ�ָ��������
			*/
			REVOLVER_setAction(SHOOT_NORMAL);
			break;
		case GIMBAL_MODE_AUTO:
			/*
				��̨����ʱ���̽�������ģʽ
			*/
			REVOLVER_setAction(SHOOT_AUTO);
			break;
		case GIMBAL_MODE_SMALL_BUFF:
		case GIMBAL_MODE_BIG_BUFF:
			/*
				��̨���ʱ���̽�����ģʽ
			*/
			REVOLVER_setAction(SHOOT_BUFF);
			break;
		default:
			break;
	}
	prev_mode = now_mode;
}

/**
 *	@brief	���̵��ң�ؿ�������
 */
void REVOLVER_rcControlTask(void)
{
	REMOTE_setRevolverShoot(&RC_Ctl_Info);
}

/**
 *	@brief	���̵�����̿�������
 */
void REVOLVER_keyControlTask(void)
{
	/* ���������Ʋ��������Ϊ */
	//KEY_setRevolverShoot(&RC_Ctl_Info);	
	switch(Revolver.action)
	{
		case SHOOT_NORMAL:
			REVOLVER_normalControl(&Revolver);
			break;
		case SHOOT_BUFF:
			REVOLVER_buffShootControl(&Revolver);
			break;
		case SHOOT_AUTO:
			REVOLVER_autoShootControl(&Revolver);
			break;
		default:
			break;
	}	
}

/**
 *	@brief	���̵��ʧ�ر���
 */
void REVOLVER_selfProtect(void)
{
	g_Revolver_Motor_Info.angle_sum = 0;	// �ۼӽǶ�ֵ����
	REVOLVER_stop(&Revolver_PID);
	REVOLVER_pidParamsInit(&Revolver_PID);	
}

/**
 *	@brief	���̵������
 */
void REVOLVER_control(void)
{
	/*----��Ϣ����----*/
	REVOLVER_getInfo();
	/*----�����޸�----*/
	if(Flag.Remote.FLAG_mode == RC) {
		REVOLVER_rcControlTask();
	} else if(Flag.Remote.FLAG_mode == KEY) {
		REVOLVER_keyControlTask();
	}	
	
	/*----�������----*/
	REVOLVER_pidControlTask();
}

