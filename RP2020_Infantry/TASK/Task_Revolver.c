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
#define REVOLVER_STUCK_SPEED			60
#define REVOLVER_STUCK_TIME				100
#define REVOLVER_TURNBACK_TIME			100

#define	TRIPLE_SHOOT_PRESSED_TIME		TIME_STAMP_250MS

#define LEVEL_COUNT			3
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
	.RemoteMode = RC,
	
	.State = REVO_STATE_OFF,
	.PidMode = REVO_POSI_MODE,
	.Action = SHOOT_NORMAL,
	
	.Shoot.freq = 8,	// Ĭ����Ƶ8
	.Shoot.num = 0,	// �����䵯��
	.Shoot.heat_cooling_rate = 0,	// 17mmǹ��������ȴֵ
	.Shoot.heat_real = 0, // 17mmǹ�ܳ�ʼ����
	.Shoot.heat_limit = 240,	// Ĭ��ǹ����������Ϊ240
	.Shoot.stuck_count = 0,// ��������
	
	.Buff.lost_time = 0,
	.Buff.lost_time_boundary = TIME_STAMP_70MS,
	.Buff.lock_time = 0,
	.Buff.lock_time_boundary = TIME_STAMP_80MS,
	.Buff.change_armor_delay = 0,
	.Buff.change_armor_delay_boundary = TIME_STAMP_50MS,
	.Buff.respond_interval = TIME_STAMP_800MS,	// TIME_STAMP_800MS ���Ե�ʱ����Ҫ
	.Buff.change_armor = false,
	.Buff.fire = false,
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
void REVOLVER_PidParamsInit(Revolver_PID_t *pid)
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
void REVOLVER_Stop(Revolver_PID_t *pid)
{
	int16_t pidOut[4] = {0, 0, 0, 0};
	
	/* �ڻ��ٶȻ�������� */
	Revolver_PID.Speed.out = 0;
	Revolver_PID.Out = 0;
	
#if REVOLVER_ID > 0x204	
	CAN2_Send(0x1FF, pidOut);
#else
	CAN2_Send(0x200, pidOut);
#endif
}

/**
 *	@brief	���̵���ٶȻ�
 */
void REVOLVER_Speed_PidCalc(Revolver_PID_t *pid)
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
void REVOLVER_Angle_PidCalc(Revolver_PID_t *pid)
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
void REVOLVER_PidOut(Revolver_PID_t *pid)
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
	CAN2_QueueSend(0x1FF, pidOut);
#else
	CAN2_QueueSend(0x200, pidOut);
#endif	
}

/**
 *	@brief	�ٶȻ���������ʽ(��ʱ�ж�+��ת����)
 */
void REVOLVER_Speed_BulletStuck(Revolver_PID_t *pid, Revolver_Info_t *revo)
{
	static uint16_t stuck_time = 0;
	static uint16_t turnback_time = 0;
	static uint8_t	stuck_recover_flag = 0;
	
	if(stuck_recover_flag == false) {	// û�����������ָ�����
		if(pid->Speed.out > REVOLVER_STUCK_PID_OUT
		   && pid->Speed.feedback < REVOLVER_STUCK_SPEED) {	// PID���������ٶȷ�����С����Ϊ����
			stuck_time++;	
		} else {
			stuck_time = 0;
		}
		if(stuck_time > REVOLVER_STUCK_TIME) {	// ������������60ms
			stuck_time = 0;
			revo->Shoot.stuck_count++;
			stuck_recover_flag = true;
			revo->Shoot.num = 0;	// ���δ�ָ��
		}
	} else {	// �����ָ�
		pid->Speed.target = -4000;
		turnback_time++;
		if(turnback_time > REVOLVER_TURNBACK_TIME) {	// ��ת���
			turnback_time = 0;
			stuck_recover_flag = false;
		}
	}
}

/**
 *	@brief	λ�û���������ʽ(��ʱ�ж�+��ת����)
 */
void REVOLVER_Angle_BulletStuck(Revolver_PID_t *pid, Revolver_Info_t *revo)
{
	static uint16_t stuck_time = 0;
	static uint16_t turnback_time = 0;
	static uint8_t	stuck_recover_flag = 0;
	
	if(stuck_recover_flag == false) {	// û�����������ָ�����
		if(pid->Speed.out > REVOLVER_STUCK_PID_OUT
		   && pid->Speed.feedback < REVOLVER_STUCK_SPEED) {	// PID���������ٶȷ�����С����Ϊ����
			stuck_time++;	
		} else {
			stuck_time = 0;
		}
		if(stuck_time > REVOLVER_STUCK_TIME) {	// ������������60ms
			stuck_time = 0;
			revo->Shoot.stuck_count++;
			stuck_recover_flag = true;
			revo->Shoot.num = 0;	// ���δ�ָ��
			//pid->Angle.target = ANGLE_AN_BULLET;	// ��ת1��
			pid->AngleRampBuffer = pid->Angle.target - ANGLE_AN_BULLET;
		}
	} else {	// �����ָ�
		turnback_time++;
		if(turnback_time > REVOLVER_TURNBACK_TIME) {	// ��ת���
			pid->Angle.target = pid->Angle.feedback;
			pid->AngleRampBuffer = pid->Angle.target;
			turnback_time = 0;
			stuck_recover_flag = false;
		}
	}
}

/* #��Ϣ��# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief ���ò��̵Ŀ�����Ϊ
 */
void REVOLVER_SetAction(Revolver_Action_t action)
{
	Revolver.Action = action;
}

/**
 *	@brief	������+1
 */
void REVOLVER_AddShootCount(void)
{
	Revolver.Shoot.total_count++;
}

/**
 *	@brief	����ʵʱ���ʱ��
 */
uint32_t REVOLVER_GetRealShootTime(void)
{
	return Revolver.Shoot.real_time;
}

/**
 *	@brief	���̻�ȡϵͳ��Ϣ
 */
void REVOLVER_GetSysInfo(System_t *sys, Revolver_Info_t *revo)
{
	/*----���Ʒ�ʽ----*/
	/* ���Ʒ�ʽ - ң���� */
	if(sys->RemoteMode == RC) {
		revo->RemoteMode = RC;
	} 
	/* ���Ʒ�ʽ - ���� */
	else if(sys->RemoteMode == KEY) {
		revo->RemoteMode = KEY;
	}

	/*----ģʽ�޸�----*/
	switch(sys->Action)
	{
		case SYS_ACT_NORMAL: 
			{
				revo->Action = SHOOT_NORMAL;// ����
			}break;
		case SYS_ACT_AUTO: 
			{
				revo->Action = SHOOT_AUTO;	// ����
			}break;
		case SYS_ACT_BUFF:
			{
				revo->Action = SHOOT_BUFF;	// ����
			}break;
		case SYS_ACT_PARK:
			{
				revo->Action = SHOOT_BAN;// ����
			}break;
	}	
}

/**
 *	@brief	���̶�ȡ����ϵͳ��Ϣ
 */
void REVOLVER_GetJudgeInfo(Judge_Info_t *judge, Revolver_Info_t *revo)
{
	if(judge->data_valid == true) {
		revo->Shoot.heat_real = JUDGE_usGetShooterRealHeat17();//judge->PowerHeatData.shooter_heat0;
		revo->Shoot.heat_limit = JUDGE_usGetShooterLimitHeat17();//judge->GameRobotStatus.shooter_heat0_cooling_limit;
		revo->Shoot.heat_cooling_rate = JUDGE_usGetShooterHeatCoolingRate17();//judge->GameRobotStatus.shooter_heat0_cooling_rate;	// ����<20%HP,��ȴ���ʷ���	
		
		//revo->Shoot.num_buffer = 
	} else {
		//..�ݲ�����
	}
	
	switch(judge->GameRobotStatus.robot_level)
	{
		// 1������
		case 1:
			revo->Shoot.freq = 8;
			break;
		// 2������
		case 2:
			revo->Shoot.freq = 10;
			break;
		// 3������
		case 3:
			revo->Shoot.freq = 12;
			break;
		// ��ֹ��bug
		default:
			revo->Shoot.freq = 8;
			break;
	}
}

/**
 *	@brief	���̻�ȡң����Ϣ
 *	@note	��ֹ����
 */
static uint8_t prev_sw1 = RC_SW_DOWN;
static uint16_t MouseLockTime_L = 0;

void REVOLVER_GetRemoteInfo(System_t *sys, RC_Ctl_t *remote, Revolver_Info_t *revo)
{
	/* ϵͳ���� */
	if(sys->State == SYSTEM_STATE_NORMAL)
	{
		if(sys->RemoteMode == RC) {
			MouseLockTime_L = 0;
		}
		else if(sys->RemoteMode == KEY) {
			prev_sw1 = RC_SW1_VALUE;
		}
	}
	/* ϵͳ�쳣 */
	else
	{
		// ��λ�ɳ�ʼֵ
		prev_sw1 = RC_SW_DOWN;
		MouseLockTime_L = 0;
	}	
}

/* #Ӧ�ò�# ---------------------------------------------------------------------------------------------------------------------------------------*/
/* #ң��# -----------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	ң�����ò��̵��ת��(����������)
 *	@note	SW1_DOWN(ǰ����Ħ���ֿ���)
 */
void REMOTE_SetRevolverShoot(RC_Ctl_t *remote)
{
	uint8_t	sw1, sw2;
	sw1 = RC_SW1_VALUE;
	sw2 = RC_SW2_VALUE;
	
	if(FRICTION_IfReady()) {
		if(Revolver.PidMode == REVO_POSI_MODE) {	// ����ģʽ
			if(sw1 == RC_SW_DOWN && prev_sw1 != RC_SW_DOWN) 
			{
				Revolver.Shoot.freq = 8;
				Revolver.Shoot.num += REVOLVER_HeatLimit(&Revolver, 1);
			}
		} else if(Revolver.PidMode == REVO_SPEED_MODE) {	// ����ģʽ
			if(sw1 == RC_SW_DOWN) {
				Revolver.State = REVO_STATE_ON;
				Revolver.Shoot.freq = 12;
				Revolver_PID.Speed.target = SPEED_AN_BULLET_PER_SECOND * Revolver.Shoot.freq;	// ��/s
			} else {
				Revolver.State = REVO_STATE_OFF;
				Revolver_PID.Speed.target = 0;
			}
		}
	}
	prev_sw1 = sw1;
	
	if(sw2 == RC_SW_MID) {
		/* �ٶȻ�->λ�û�*/
		if(Revolver.PidMode == REVO_SPEED_MODE) {
			Revolver.PidMode = REVO_POSI_MODE;
			Revolver.State = REVO_STATE_OFF;
			Revolver_PID.Angle.target = Revolver_PID.Angle.feedback;	// ��ֹ�л�ģʽ��ʱ��һ��
			Revolver_PID.AngleRampBuffer = Revolver_PID.Angle.target;
		}
	} else if(sw2 == RC_SW_DOWN) {
		/* λ�û�->�ٶȻ� */
		if(Revolver.PidMode == REVO_POSI_MODE) {
			Revolver.PidMode = REVO_SPEED_MODE;
			Revolver.State = REVO_STATE_OFF;
			Revolver_PID.Speed.target = 0;	// s1_down,s2_down -> s2_mid -> s1_mid -> s2_down => ���̻���תһС��(С��һ��)
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

void KEY_SetRevolverShoot(RC_Ctl_t *remote)
{	
//	/* Ħ���ֿ�����ʱ�����Ӧ���� */
//	if(FRICTION_IfReady()) 
//	{		
//		/* ����������ʱ�ۼ�ʱ�� */
//		if(IF_MOUSE_PRESSED_LEFT)
//		{
//			if(MouseLockTime_L < TIME_STAMP_6000MS)	// ��ֹ��ʱ�䰴�������
//					MouseLockTime_L++;
//		}
//		
//		if(IF_KEY_PRESSED_B && !IF_MOUSE_PRESSED_LEFT & !IF_KEY_PRESSED_B)
//		{
//			/* �Ƽ�ģʽ */
//			Revolver.PidMode = REVO_POSI_MODE;
//			Revolver.Action = SHOOT_HIGH_F_LOW_S;
//		}
//		else if(GIMBAL_IfBuffMode())
//		{
//			/* ���ģʽ */
//			Revolver.PidMode = REVO_POSI_MODE;
//			Revolver.Action = SHOOT_BUFF;
//			/* �л�ģʽʱ��������Ӧʱ�� */
//			MouseLockTime_L = 0;
//		}
//		else if(GIMBAL_IfAutoMode())
//		{
//			/* ����ģʽ */
//			Revolver.PidMode = REVO_POSI_MODE;
//			Revolver.Action = SHOOT_AUTO;
//			/* �л�ģʽʱ��������Ӧʱ�� */
//			MouseLockTime_L = 0;
//		}
//		else if(IF_MOUSE_PRESSED_LEFT && !IF_KEY_PRESSED_B && !IF_KEY_PRESSED_Z) 
//		{
//			/* ������ģʽ */
//			if(MouseLockTime_L >= TRIPLE_SHOOT_PRESSED_TIME) 
//			{	// �״�������Ҫ�������250ms
//				//MouseLockTime_L = 0;
//				Revolver.PidMode = REVO_SPEED_MODE;	// �ٶȻ�
//				Revolver.Action = SHOOT_TRIPLE;		// ������
//				Revolver.State = REVO_STATE_ON;		// ����������ת
//			} 
//		} 
//		else if((MouseLockTime_L > TIME_STAMP_10MS && MouseLockTime_L < TRIPLE_SHOOT_PRESSED_TIME)
//				&& !IF_MOUSE_PRESSED_LEFT && !IF_KEY_PRESSED_B && !IF_KEY_PRESSED_Z)
//		{
//			/* ����ģʽ */
//			MouseLockTime_L = 0;
//			Revolver.PidMode = REVO_POSI_MODE;
//			Revolver.Action = SHOOT_SINGLE;
//		}
//		else 
//		{
//			/* �������ģʽ */
//			/* �ٶȻ���λ�û� */
//			if(Revolver.PidMode == REVO_SPEED_MODE) {
//				Revolver_PID.Angle.target = Revolver_PID.Angle.feedback;	// ��ֹ�л�ģʽ��ʱ��һ��
//				Revolver_PID.AngleRampBuffer = Revolver_PID.Angle.target;
//			}
//			MouseLockTime_L = 0;
//			Revolver.PidMode = REVO_POSI_MODE;
//			Revolver.Action = SHOOT_NORMAL;
//			Revolver.State = REVO_STATE_OFF;		// ֹͣ������ת			
//		}
//	}
//	/* δ����Ħ��������ָ��*/
//	else	
//	{
//		MouseLockTime_L = 0;
//		Revolver.PidMode = REVO_POSI_MODE;
//		Revolver.Action = SHOOT_NORMAL;		
//	}
}

/* #����# -----------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief 	��ϵ�ǰ�������������󷢵���(0<=allow_num<=target_num)
 *	@return allow_num( 0 ~ target_num)
 */
uint8_t REVOLVER_HeatLimit(Revolver_Info_t *revo, uint8_t target_num)
{
	uint16_t remain_heat;	// ʣ������
	uint8_t  allow_num;		// ����ķ�������
	
	if(target_num == 0)	// ��ֹ����
		return 0;
	
	if(revo->Shoot.suicide_mode == true)	// �������޿�Ѫ��ȡ��Ƶ
		return target_num;
	
	remain_heat = revo->Shoot.heat_limit - revo->Shoot.heat_real;
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
 *	@brief	���̵���������
 *	@note	Ĭ�ϲ���
 */
void REVOLVER_NormalCtrl(Revolver_Info_t *revo)
{
	/* ����������ʱ�ۼ�ʱ�� */
	if(IF_MOUSE_PRESSED_LEFT)
	{
		if(MouseLockTime_L < TIME_STAMP_6000MS)	// ��ֹ��ʱ�䰴�������
			MouseLockTime_L++;
	}

	/* �̰� */
	if( !IF_MOUSE_PRESSED_LEFT
			&& (MouseLockTime_L > TIME_STAMP_10MS)
			&& (MouseLockTime_L < TRIPLE_SHOOT_PRESSED_TIME) ) 
	{
		/* ���� */
		REVOLVER_SingleShootCtrl(revo);
	}
	/* ���� */
	else if( 
			(MouseLockTime_L >= TRIPLE_SHOOT_PRESSED_TIME) )
	{
		/* ���� */
		REVOLVER_TripleShootCtrl(revo);
	}
	/* �ɿ� */
	else
	{
		MouseLockTime_L = 0;
		/* ��ֹ���� */
		/* �ٶȻ���λ�û� */
		if(revo->PidMode == REVO_SPEED_MODE) {
			Revolver_PID.Angle.target = Revolver_PID.Angle.feedback;	// ��ֹ�л�ģʽ��ʱ��һ��
			Revolver_PID.AngleRampBuffer = Revolver_PID.Angle.target;
		}
		revo->PidMode = REVO_POSI_MODE;
		revo->State = REVO_STATE_OFF;		// ֹͣ������ת
	}	
}

/**
 *	@brief	���̵����������
 *	@note	����
 */
void REVOLVER_SingleShootCtrl(Revolver_Info_t *revo)
{
	portTickType  current_time = 0;
	static uint32_t  respond_time = 0;	//��Ӧ�����ʱ
	
	current_time = xTaskGetTickCount();

	revo->PidMode = REVO_POSI_MODE;	// λ�û�
	revo->State = REVO_STATE_OFF;	// �ز�����ת
	
	/* # ��ô�������ܵ������Ƶ? */
	revo->Shoot.interval = TIME_STAMP_1000MS / revo->Shoot.freq;
	
	if((respond_time < current_time)
			&& (revo->Shoot.num == 0))
	{
		respond_time = current_time + revo->Shoot.interval;
		revo->Shoot.num += REVOLVER_HeatLimit(&Revolver, 1);
	}

	/* �ɿ�����󷵻س������ */
	if(!IF_MOUSE_PRESSED_LEFT) {
		MouseLockTime_L = 0;
	}
}

/**
 *	@brief	���̵������������
 *	@note	������
 */
void REVOLVER_TripleShootCtrl(Revolver_Info_t *revo)
{
//	/* # ��ô�������ܵ������Ƶ? */
//	if(revo->Shoot.heat_real >= 160) {
//		revo->Shoot.freq = 14;
//	} else {
//		revo->Shoot.freq = 8;
//	}	
	
	#if	0
	/* ��-���� */
	revo->PidMode = REVO_SPEED_MODE;	// �ٶȻ�
	revo->State = REVO_STATE_ON;		// ��������ת
	
	/* �ɿ�����󷵻س������ */
	if(!IF_MOUSE_PRESSED_LEFT) {	
		MouseLockTime_L = 0;
	}
	
	#else
	/* �������� */
	portTickType  current_time = 0;
	static uint32_t  respond_time = 0;	//��Ӧ�����ʱ
		
	current_time = xTaskGetTickCount();
	
	revo->PidMode = REVO_POSI_MODE;	// λ�û�
	revo->State = REVO_STATE_OFF;	// �ز�����ת
	revo->Shoot.interval = TIME_STAMP_1000MS / revo->Shoot.freq;
	
	if(respond_time < current_time
		&& (revo->Shoot.num == 0)) 
	{
		respond_time = current_time + revo->Shoot.interval;
		revo->Shoot.num += REVOLVER_HeatLimit(revo, 1);
	}
	
	/* �ɿ�����󷵻س������ */
	if(!IF_MOUSE_PRESSED_LEFT) {	
		MouseLockTime_L = 0;
	}	
	#endif
}

/**
 *	@brief	�ؼ�֮�����滨
 *	@note		# RM2020�Կ�����������ǹ���������ӵ����ٶ��޹أ��̶�һ��Ϊ10����
 */
void REVOLVER_ComboShootCtrl(Revolver_Info_t *revo)
{
	portTickType  current_time = 0;
	static uint32_t  respond_time = 0;	//��Ӧ�����ʱ	
	
	current_time = xTaskGetTickCount();
	
	/* # ��ô�������ܵ������Ƶ? */
	revo->Shoot.interval = TIME_STAMP_1000MS/15;	//���һ��15��
	
	if((respond_time < current_time) 
			&& (revo->Shoot.num == 0))
	{
		respond_time = current_time + revo->Shoot.interval;
		revo->Shoot.num += REVOLVER_HeatLimit(&Revolver, 1);
	}	
	
	/* �ɿ�B���󷵻س������ */
	if(!IF_KEY_PRESSED_B) {	
		MouseLockTime_L = 0;
		revo->Action = SHOOT_NORMAL;
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
void REVOLVER_BuffShootCtrl(Revolver_Info_t *revo)
{
	portTickType  current_time = 0;
	static uint32_t  respond_time = 0;	// ��Ӧ�����ʱ
	static uint16_t manual_pressed_left = 0;	// �ֶ��������Ӧ

	/* #λ�û� */
	revo->PidMode = REVO_POSI_MODE;
	revo->State = REVO_STATE_OFF;
	
	current_time = xTaskGetTickCount();

	/* �Ƿ����װ�װ� */
	if(VISION_GetFlagStatus(VISION_FLAG_CHANGE_ARMOR) == true) {
		revo->Buff.change_armor = true;
	}
	
	/* �Ƿ��л������Ŀ�װ�װ� */
	revo->Buff.change_armor_4 = VISION_GetFlagStatus(VISION_FLAG_CHANGE_ARMOR_4);
	
	if(test_buff_bullet == 0)
	{
		/* �ɿ��Ҽ��Զ��� */
		if(!IF_MOUSE_PRESSED_RIGH)
		{
			manual_pressed_left = 0;
			/* δʶ�𵽷� */
			if(VISION_GetFlagStatus(VISION_FLAG_LOCK_BUFF) == false)
			{
				revo->Buff.lost_time++;
				if(revo->Buff.lost_time > revo->Buff.lost_time_boundary) {
					revo->Buff.fire = false;
					revo->Buff.lock_time = 0;
				}
				
	//			if(revo->Buff.change_armor == true)
	//			{
	//				revo->Buff.lock_time = 0;
	//				revo->Buff.lost_time = 0;
	//			}
			}
			/* ʶ�𵽷� */
			else
			{
				revo->Buff.lost_time = 0;
				
				/* ���л���װ�װ��ȶ�һ��ʱ�䲻�� */
				if(revo->Buff.change_armor_delay > 0)
				{
					revo->Buff.change_armor_delay--;
				}

				if(revo->Buff.change_armor_delay == 0)		
				{					
					/* ���л���װ�װ� */
					if(revo->Buff.change_armor == true)
					{
						revo->Buff.change_armor = false;
						revo->Buff.fire = false;	// ��ֹ���
						revo->Buff.lock_time = 0;	// ����װ�װ�ʱ������
						respond_time = current_time-1;// ˢ�����ʱ��
						revo->Buff.change_armor_delay = revo->Buff.change_armor_delay_boundary;	// �л������ȶ�һ��ʱ��
					} 
					/* δ�л�װ�װ� */
					else if(revo->Buff.change_armor == false)
					{
						/* ��̨�Ѹ���Ŀ�� */
						if(GIMBAL_BUFF_IfChaseReady()) 
						{
							revo->Buff.lock_time++;
							/* ��̨�ȶ�����װ�װ� */
							if(revo->Buff.lock_time > revo->Buff.lock_time_boundary)
							{
								/* ���Կ��� */
								revo->Buff.fire = true;
							}
						}
						/* ��̨���Ŀ�� */
						else
						{
							/* ��ֹ���� */
							revo->Buff.lock_time = 0;
							revo->Buff.fire = false;
						}
					}
					
					/* ��Ӧʱ���ѵ� & ���Կ��� & �Ǹ��л�װ�װ� & ��һ���ӵ��Ѵ��ȥ */
					if((respond_time < current_time) &&
						revo->Buff.fire == true && 
						revo->Buff.change_armor == false &&
						revo->Shoot.num == 0) 
					{
						respond_time = current_time + revo->Buff.respond_interval;
						// ���ʱ��������ȴ�ӳɣ�һ�㲻���������ޡ��������޳�����Ӱ������
						revo->Shoot.num += 1;	//revo->Shoot.num += REVOLVER_HeatLimit(&Revolver, 1);
					}
				}
			}
			
			/* # ���Ե�ʱ��Ĭ�����ﲦһ���ʹ�һ�ų�ȥ(֮��������ò���ϵͳ�����ٷ������ж��Ƿ����) */
			if(revo->Buff.change_armor_4 == true) {
				if(revo->Shoot.num != 0) {
					if(first_into_armor_4 == 0) {
						first_into_armor_4 = xTaskGetTickCount();
					}
				}
				if(first_into_armor_4 != 0) {
					respond_into_armor_4 = xTaskGetTickCount();
					if(respond_into_armor_4 > first_into_armor_4 + TIME_STAMP_100MS) {	// �ʵ���ʱģ�ⷢ���ӳ�
						VISION_SetFlagStatus(VISION_FLAG_SHOOT_ARMOR_4);
					}
				}
			} else {
				VISION_ClearFlagStatus(VISION_FLAG_SHOOT_ARMOR_4);
				first_into_armor_4 = 0;
			}
		}
		/* ��ס�Ҽ��ӹܴ� */
		else
		{
			/* ��ֹ�ֶ��л����Զ���ʱ�����ϴ� */
			revo->Buff.fire = false;	// ��ֹ�Զ����
			revo->Buff.lock_time = 0;	// ����װ�װ�ʱ������
			if(IF_MOUSE_PRESSED_LEFT) 
			{
				if(manual_pressed_left < TIME_STAMP_10S)	// ��ֹ��ʱ�䰴�������
					manual_pressed_left++;
			}
			else
			{
				if(manual_pressed_left > TIME_STAMP_10MS) {	// ���̧���Ŵ��
					// ���ʱ��������ȴ�ӳɣ�һ�㲻�����������ޡ��������޳�����Ӱ������
					revo->Shoot.num += 1;	//revo->Shoot.num += REVOLVER_HeatLimit(&Revolver, 1);
				}
				
				/* # ���Ե�ʱ��Ĭ�����ﲦһ���ʹ�һ�ų�ȥ(֮��������ò���ϵͳ�����ٷ������ж��Ƿ����) */
				if(revo->Buff.change_armor_4 == true) {
					if(revo->Shoot.num != 0) {
						if(first_into_armor_4 == 0) {
							first_into_armor_4 = xTaskGetTickCount();
						}
					}
					if(first_into_armor_4 != 0) {
						respond_into_armor_4 = xTaskGetTickCount();
						if(respond_into_armor_4 > first_into_armor_4 + TIME_STAMP_100MS) {// �ʵ���ʱģ�ⷢ���ӳ�
							VISION_SetFlagStatus(VISION_FLAG_SHOOT_ARMOR_4);
						}
					}
				} else {
					VISION_ClearFlagStatus(VISION_FLAG_SHOOT_ARMOR_4);
					first_into_armor_4 = 0;
				}
				
				manual_pressed_left = 0;
			}
		}
	}
	else
	{
		/* ��ֹ�ֶ��л����Զ���ʱ�����ϴ� */
		revo->Buff.fire = false;	// ��ֹ�Զ����
		revo->Buff.lock_time = 0;	// ����װ�װ�ʱ������
		if(IF_MOUSE_PRESSED_LEFT) 
		{
			if(manual_pressed_left < TIME_STAMP_10S)	// ��ֹ��ʱ�䰴�������
				manual_pressed_left++;
		}
		else
		{
			if(manual_pressed_left > TIME_STAMP_10MS) {	// ���̧���Ŵ��
				revo->Shoot.num += REVOLVER_HeatLimit(&Revolver, 1);
			}
			manual_pressed_left = 0;
		}		
	}
}

/**
 *	@brief	���̵���������
 *	@note		����Ԥ��
 */
void REVOLVER_AutoShootCtrl(Revolver_Info_t *revo)
{
	static portTickType current_time     = 0;
	static uint32_t respond_time_Stop    = 0;//��Ӧ�����ʱ����ֹ
	static uint32_t respond_time_MobiPre = 0;//��Ӧ�����ʱ���ƶ�Ԥ��	

	/* #λ�û� */
	revo->PidMode = REVO_POSI_MODE;
	revo->State = REVO_STATE_OFF;
	
	current_time = xTaskGetTickCount();
	
	/* �Ա�ģʽ(������������������޿�Ѫ��ȡ��Ƶ */
	if(IF_KEY_PRESSED_B) {
		revo->Shoot.freq = 20;
		revo->Shoot.suicide_mode = true;
	} else {
		revo->Shoot.suicide_mode = false;
	}

	/* ������Ƶ������Ӧ��� */
	revo->Shoot.interval = TIME_STAMP_1000MS / revo->Shoot.freq;
	
	/* �����ƶ�Ԥ���ѿ��� */
	if( GIMBAL_IfAutoMobiPre() == true) {
		//revo->Shoot.interval = TIME_STAMP_1000MS/10;
		/* ���鿪������ */
		if(GIMBAL_AUTO_IfChaseReady() == true			// Ԥ�⵽λ
				&& respond_time_MobiPre < current_time	// �����Ӧ
					&& revo->Shoot.num == 0				// ��һ���Ѵ��
						&& IF_MOUSE_PRESSED_LEFT)		// �������
		{
			respond_time_MobiPre = current_time + revo->Shoot.interval;
			revo->Shoot.num += REVOLVER_HeatLimit(&Revolver, 1);
		}
		else{
			revo->Shoot.num = 0;	// Ԥ��δ��λ����ֹ��
		}
	}
	/* �����ƶ�Ԥ��δ���� */
	else if( GIMBAL_IfAutoMobiPre() == false) {
		//revo->Shoot.interval = TIME_STAMP_1000MS/5;
		/* ���鿪������ */
		if(GIMBAL_AUTO_IfChaseReady() == true			// Ԥ�⵽λ
				&& respond_time_Stop < current_time		// �����Ӧ
					&& revo->Shoot.num == 0				// ��һ���Ѵ��
						&& IF_MOUSE_PRESSED_LEFT)		// �������
		{
			respond_time_Stop = current_time + revo->Shoot.interval;
			revo->Shoot.num += REVOLVER_HeatLimit(&Revolver, 1);
		}
		else{
			revo->Shoot.num = 0;	// Ԥ��δ��λ����ֹ��
		}		
	}
}

/**
 *	@brief	���̵����ֹ�������
 */
void REVOLVER_BanShootCtrl(Revolver_Info_t *revo)
{
	revo->PidMode = REVO_POSI_MODE;	// λ�û�
	revo->State = REVO_STATE_OFF;	// �ز�����ת
	
	revo->Shoot.num = 0;	// ���δ�ָ��
}

/* #�����# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	������̨ģʽ��������ģʽ
 *	@note		# ��ԭ�еĿ����߼����ڳ�ͻ����ʱ����
 */
void REVOLVER_GetInfo(void)
{
	// ��ȡϵͳ��Ϣ
	REVOLVER_GetSysInfo(&System, &Revolver);
	// ��ȡң����Ϣ
	REVOLVER_GetRemoteInfo(&System, &RC_Ctl_Info, &Revolver);
	// ��ȡ����ϵͳ��Ϣ
	REVOLVER_GetJudgeInfo(&Judge, &Revolver);	
}

/**
 *	@brief	���̵��PID������
 *	@note	(ǰ����Ħ���ֿ���)
 */
uint16_t js_shoot_num = 0;
void REVOLVER_PidCtrlTask(void)
{
	/* δ��Ħ���� */
	if(FRICTION_IfOpen() == false) {
		g_Revolver_Motor_Info.angle_sum = 0;	// �ۼӽǶ�ֵ����
		REVOLVER_PidParamsInit(&Revolver_PID);
		REVOLVER_Stop(&Revolver_PID);
	}
	else {
		if(Revolver.PidMode ==  REVO_SPEED_MODE) // ����ģʽ
		{	
			if(Revolver.State == REVO_STATE_ON) {	// ����������ת
				if(REVOLVER_HeatLimit(&Revolver, 1))
					Revolver_PID.Speed.target = SPEED_AN_BULLET_PER_SECOND * Revolver.Shoot.freq;	// ÿ��һ�ŵ�ת�ٳ�����Ƶ
				else
					Revolver_PID.Speed.target = 0;
			} else if(Revolver.State == REVO_STATE_OFF) {	// �رղ�����ת
				Revolver_PID.Speed.target = 0;
			}
			/* �����ж� */
			REVOLVER_Speed_BulletStuck(&Revolver_PID, &Revolver);
			/* �ٶȻ����� */
			REVOLVER_Speed_PidCalc(&Revolver_PID);
		} 
		else if(Revolver.PidMode ==  REVO_POSI_MODE) // ����ģʽ
		{	
			/* �����ж� */
			REVOLVER_Angle_BulletStuck(&Revolver_PID, &Revolver);	
			if(Revolver.Shoot.num > 0) {	// ��Ҫ������
				js_shoot_num++;
				Revolver.Shoot.num--;
				Revolver_PID.AngleRampBuffer += ANGLE_AN_BULLET;
				Revolver.Shoot.real_time = xTaskGetTickCount();	// 
			}
			/* λ�û�����(��Ŀ�껺���ƽ�����Ŀ��) */
			Revolver_PID.Angle.target = RampFloat(Revolver_PID.AngleRampBuffer, Revolver_PID.Angle.target, ANGLE_AN_BULLET_RAMP);
			REVOLVER_Angle_PidCalc(&Revolver_PID);
			Revolver_PID.Speed.target = Revolver_PID.Angle.out;
			REVOLVER_Speed_PidCalc(&Revolver_PID);
		}
		/*----�������----*/
		REVOLVER_PidOut(&Revolver_PID);	
	} 
}

/**
 *	@brief	���̵��ң�ؿ�������
 */
void REVOLVER_RcCtrlTask(void)
{
	REMOTE_SetRevolverShoot(&RC_Ctl_Info);
}

/**
 *	@brief	���̵�����̿�������
 */
void REVOLVER_KeyCtrlTask(void)
{
	/* Ħ����û�о������δ�ָ�� */
	if(FRICTION_IfReady() == false) {
		MouseLockTime_L = 0;
		Revolver.Shoot.num = 0;
		Revolver.State = REVO_STATE_OFF;
		Revolver.PidMode = REVO_POSI_MODE;
		return;
	}

	switch(Revolver.Action)
	{
		case SHOOT_NORMAL:
			REVOLVER_NormalCtrl(&Revolver);
			break;
		case SHOOT_BUFF:
			REVOLVER_BuffShootCtrl(&Revolver);
			break;
		case SHOOT_AUTO:
			REVOLVER_AutoShootCtrl(&Revolver);
			break;
		case SHOOT_BAN:
			REVOLVER_BanShootCtrl(&Revolver);
		default:
			break;
	}	
}

/**
 *	@brief	���̵��ʧ�ر���
 */
void REVOLVER_SelfProtect(void)
{
	g_Revolver_Motor_Info.angle_sum = 0;	// �ۼӽǶ�ֵ����
	REVOLVER_Stop(&Revolver_PID);
	REVOLVER_PidParamsInit(&Revolver_PID);	
}

/**
 *	@brief	���̵������
 */
void REVOLVER_Ctrl(void)
{
	/*----��Ϣ����----*/
	REVOLVER_GetInfo();
	/*----�����޸�----*/
	if(Revolver.RemoteMode == RC) {
		REVOLVER_RcCtrlTask();
	} else if(Revolver.RemoteMode == KEY) {
		REVOLVER_KeyCtrlTask();
	}	
	
	/*----�������----*/
	REVOLVER_PidCtrlTask();
}

