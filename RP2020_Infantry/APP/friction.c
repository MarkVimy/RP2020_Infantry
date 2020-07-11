#include "pwm.h"

/**
 * @file        pwm.c
 * @author      MaWeiming
 * @Version     V1.0
 * @date        17-October-2019
 * @brief       This file includes the FRICTION WHEEL(Ħ����) external functions 
 * 				(based on ST Peripherals Libaray Keil.STM32F4xx_DFP.2.13.0)
 * @Version		
 */

/**
 *	# Ħ���ֵ�� ������
 *	  
 *	# Ħ���ֵ�� С�۷���
 *
 *	# ���
 */

/* Includes ------------------------------------------------------------------*/
#include "friction.h"

#include "pwm.h"

#include "remote.h"
#include "laser.h"
#include "delay.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* ## Global variables ## ----------------------------------------------------*/
Friction_Info_t Friction = {
	.RemoteMode = RC,
	.State = FRIC_STATE_OFF,
	.Speed = 0,
	.SpeedLevel = FRIC_SPEED_OFF,
	.SpeedTarget = 0,
	.SpeedFeedback = 0,
};

uint16_t Friction_Pwm_Output[FRIC_SPEED_COUNT] = {0, 460, 505, 563, 695, 685};	// 0, 460, 505, 583, 695, 685
												// �ر�  ����  ����  ����  ��  �ڱ�
uint16_t Friction_Pwm_Speed[FRIC_SPEED_COUNT] = {0, 0, 2190, 2510, 2850, 2800};

uint8_t test_fric_off = 0;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* API functions -------------------------------------------------------------*/
/* #������# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	Ħ���ֲ�����ʼ��
 */
void FRICTION_ParamsInit(Friction_Info_t *fric)
{
	fric->State = FRIC_STATE_OFF;
	fric->SpeedLevel = FRIC_SPEED_OFF;
	fric->SpeedTarget = Friction_Pwm_Output[FRIC_SPEED_OFF];
	//fric->SpeedFeedback = 0;
}

/**
 *	@brief	Ħ����б�¼�С�����ֹͣ
 */
void FRICTION_Stop(Friction_Info_t *fric)
{
//	fric->SpeedLevel = FRIC_SPEED_OFF;
//	fric->SpeedTarget = Friction_Pwm_Output[FRIC_SPEED_OFF];
	FRICTION_ParamsInit(fric);
	if(fric->SpeedFeedback > 0) {
		fric->SpeedFeedback -= 5;
	} else if(fric->SpeedFeedback < 0) {
		fric->SpeedFeedback  = 0;
	}
	
	if(test_fric_off == 0) {
		FRICTION_PwmOut(fric->SpeedFeedback, fric->SpeedFeedback);
	}
}

/**
 *	@brief	Ħ����б�����
 */
void FRICTION_PwmCalc(Friction_Info_t *fric)
{
	if(fric->SpeedFeedback < fric->SpeedTarget) {	// ����
		fric->SpeedFeedback += 5;
		if(fric->SpeedFeedback > fric->SpeedTarget) {
			fric->SpeedFeedback = fric->SpeedTarget;
		}
	} else if(fric->SpeedFeedback > fric->SpeedTarget) {	// ����
		fric->SpeedFeedback -= 5;
		if(fric->SpeedFeedback < fric->SpeedTarget) {
			fric->SpeedFeedback = fric->SpeedTarget;
		}
	}
	
	if(fric->SpeedFeedback < 0) {
		fric->SpeedFeedback = 0;
	}
}

/**
 *	@brief	Ħ���ֵ��У׼
 */
void FRICTION_SelfCalibrate(void)
{
	PWM1 = 1500;
	PWM2 = 1500;
	delay_ms(1000);
	delay_ms(1000);
	delay_ms(1000);
	delay_ms(1000);
	delay_ms(1000);
	delay_ms(1000);
	delay_ms(1000);
}

/* #��Ϣ��# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	�ж�Ħ�����Ƿ���
 */
bool FRICTION_IfOpen(void)
{
	if(Friction.State == FRIC_STATE_OFF)
		return false;
	else
		return true;
}

/**
 *	@brief	�ж�Ħ�����Ƿ����/���ٵ������ٶ�
 */
bool FRICTION_IfReady(void)
{
	if(Friction.State == FRIC_STATE_OFF)
		return false;
	return (abs(Friction.SpeedTarget - Friction.SpeedFeedback)<=20)?true:false;
}

/**
 *	@brief	Ħ���ֻ�ȡϵͳ��Ϣ
 */
void FRICTION_GetSysInfo(System_t *sys, Friction_Info_t *fric)
{
	/*----���Ʒ�ʽ----*/
	/* ���Ʒ�ʽ - ң���� */
	if(sys->RemoteMode == RC) {
		fric->RemoteMode = RC;
	} 
	/* ���Ʒ�ʽ - ���� */
	else if(sys->RemoteMode == KEY) {
		fric->RemoteMode = KEY;
	}	
}

/**
 *	@brief	Ħ���ֻ�ȡϵͳ��Ϣ
 */
void FRICTION_GetJudgeInfo(Judge_Info_t *judge, Friction_Info_t *fric)
{
	uint8_t speed_limit = JUDGE_ucGetBulletLimitSpeed17();
	
	/* #��������������Ƶ���Ħ���ֵ��ٶ� */
	switch(speed_limit)
	{
		case 12: {
				Friction.SpeedLevel = FRIC_SPEED_LOW;
			}break;
		
		case 15: {
				Friction.SpeedLevel = FRIC_SPEED_MID;
			}break;
		
		case 18: {
				Friction.SpeedLevel = FRIC_SPEED_HIGH;
			}break;
		
		case 30: {
				Friction.SpeedLevel = FRIC_SPEED_VERYHIGH;
			}break;
		
		default:
			break;
	}	
}

/**
 *	@brief	Ħ����ң����Ϣ
 *	@note	��ֹ����
 */
static uint8_t prev_sw1 = RC_SW_UP;

void FRICTION_GetRemoteInfo(System_t *sys, RC_Ctl_t *remote, Friction_Info_t *fric)
{
	/* ϵͳ���� */
	if(sys->State == SYSTEM_STATE_NORMAL)
	{
		if(sys->RemoteMode == KEY) {
			prev_sw1 = RC_SW1_VALUE;
		}
	}
	/* ϵͳ�쳣 */
	else
	{
		// ��λ�ɳ�ʼֵ
		prev_sw1 = RC_SW_UP;
	}	
}

/* #Ӧ�ò�# ---------------------------------------------------------------------------------------------------------------------------------------*/
/* #ң��# -----------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	ң������Ħ���ֿ���/�ر�
 *	@note	SW2_DOWN + SW1_UP
 */
uint16_t test_speed = 350;
void REMOTE_SetFrictionState(RC_Ctl_t *remote)
{
	uint8_t	sw1, sw2;
	sw1 = RC_SW1_VALUE;
	sw2 = RC_SW2_VALUE;
	
	if(BM_IfReset(BitMask.System.BM_Reset, BM_RESET_FRIC)) {	// Ħ���ָ�λ���
		if((sw1 == RC_SW_UP && prev_sw1 != RC_SW_UP) && (sw2 == RC_SW_DOWN)) {
			if(Friction.State == FRIC_STATE_OFF) {
				Friction.State = FRIC_STATE_ON;			// ��Ħ����
				Friction.SpeedLevel = FRIC_SPEED_HIGH;	// ������
				Friction.SpeedTarget = test_speed;//Friction_Pwm_Output[Friction.SpeedLevel];
				LASER_ON();
			} else if(Friction.State == FRIC_STATE_ON){
				Friction.State = FRIC_STATE_OFF;		// �ر�Ħ����
				Friction.SpeedLevel = FRIC_SPEED_OFF;	// ������
				Friction.SpeedTarget = Friction_Pwm_Output[Friction.SpeedLevel];
				LASER_OFF();
			}
		}
	}
	prev_sw1 = sw1;
}

/* #�������# -----------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	ң������Ħ���ֿ���/�ر�
 *	@note	����ģʽ��Ħ���ֿ����󲻹ر�(���������������)
 */
void KEY_SetFrictionState(RC_Ctl_t *remote)
{
	if(BM_IfReset(BitMask.System.BM_Reset, BM_RESET_FRIC)) {	// Ħ���ָ�λ���
		if(Friction.State == FRIC_STATE_OFF && IF_MOUSE_PRESSED_LEFT) {
			Friction.State = FRIC_STATE_ON;			// ��Ħ����
			Friction.SpeedLevel = FRIC_SPEED_HIGH;	// ������
			Friction.SpeedTarget = Friction_Pwm_Output[Friction.SpeedLevel];
			LASER_ON();
		}
	}
}

/* #Ħ����# -----------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	Ħ���ָ�λ
 */
void FRICTION_Reset(void)
{
	if(Friction.SpeedFeedback != 0) {
		FRICTION_Stop(&Friction);
	} else {
		BM_Reset(BitMask.System.BM_Reset, BM_RESET_FRIC);	// ��λ���
	}	
}

/**
 *	@brief	Ħ�����ٶ��л�
 */
void FRICTION_SpeedSwitch(Friction_Info_t *fric)
{
	if(FRICTION_IfReady()) {
		/* �ж��ڻ����ڱ�(̧ͷpitch��С) */
		if(GIMBAL_IfAimSentry()) {
			fric->SpeedLevel = FRIC_SPEED_SENTRY;
		}
		/* �����ʱ����ó������� */
		else if(GIMBAL_IfBuffMode()) {
			fric->SpeedLevel = FRIC_SPEED_VERYHIGH;
		}
		/* ����ģʽ�²��ø����� */
		else if(GIMBAL_IfNormalMode()) {
			fric->SpeedLevel = FRIC_SPEED_SENTRY;
		}
		fric->SpeedTarget =  Friction_Pwm_Output[Friction.SpeedLevel];
	}
}

/* #�����# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	��ȡ�ⲿ��Ϣ
 *	@note		��Ҫ�Ǹ������ܵ�������Ħ��������
 */
void FRICTION_GetInfo(void)
{
	// ��ȡϵͳ��Ϣ
	FRICTION_GetSysInfo(&System, &Friction);
	// ��ȡ����ϵͳ��Ϣ
	FRICTION_GetJudgeInfo(&Judge, &Friction);
	// ��ȡң����Ϣ
	FRICTION_GetRemoteInfo(&System, &Remote, &Friction);
	// ����pwm-���ٱ��ȡ������Ϣ
	Friction.Speed = Friction_Pwm_Speed[Friction.SpeedLevel];
}

/**
 *	@brief	ң������Ħ���ֿ���/�ر�
 *	@note	Loop Time: 10ms
 */
void FRICTION_RcCtrlTask(void)
{
	REMOTE_SetFrictionState(&Remote);
}

/**
 *	@brief	��������Ħ���ֿ���/�ر�
 *	@note	Loop Time: 10ms
 */
void FRICTION_KeyCtrlTask(void)
{
	KEY_SetFrictionState(&Remote);
}

/**
 *	@brief	Ħ����ʧ�ر���
 */
void FRICTION_SelfProtect(void)
{
	FRICTION_Stop(&Friction);
	FRICTION_GetRemoteInfo(&System, &Remote, &Friction);
	LASER_OFF();
}

/**
 *	@brief	Ħ���ֿ���
 */
void FRICTION_Ctrl(void)
{
	/*----��Ϣ����----*/
	FRICTION_GetInfo();
	
	/*----�����޸�----*/
	if(BM_IfSet(BitMask.System.BM_Reset, BM_RESET_FRIC)) {	// ��λ״̬
		FRICTION_Reset(); // Ħ���ָ�λ
	} else {
		if(Friction.RemoteMode == RC) {
			FRICTION_RcCtrlTask();
		} else if(Friction.RemoteMode == KEY) {
			FRICTION_KeyCtrlTask();
		}	
	}
	
	/* ������̨ģʽ�л�Ħ���ֲ��� */
	//FRICTION_SpeedSwitch(&Friction);	
	
	/*----�������----*/
	if(test_fric_off == 0) {
		FRICTION_PwmCalc(&Friction);	// б�¼���
		FRICTION_PwmOut(Friction.SpeedFeedback, Friction.SpeedFeedback);
	}
}
