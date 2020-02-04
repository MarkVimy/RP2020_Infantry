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
	.state = FRIC_STATE_OFF,
	.speedLevel = FRIC_SPEED_OFF,
	.speedTarget = 0,
	.speedFeedback = 0,
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
	fric->state = FRIC_STATE_OFF;
	fric->speedLevel = FRIC_SPEED_OFF;
	fric->speedTarget = Friction_Pwm_Output[FRIC_SPEED_OFF];
	//fric->speedFeedback = 0;
}

/**
 *	@brief	Ħ����б�¼�С�����ֹͣ
 */
void FRICTION_stop(Friction_Info_t *fric)
{
//	fric->speedLevel = FRIC_SPEED_OFF;
//	fric->speedTarget = Friction_Pwm_Output[FRIC_SPEED_OFF];
	FRICTION_ParamsInit(fric);
	if(Friction.speedFeedback > 0) {
		Friction.speedFeedback -= 5;
	} else if(Friction.speedFeedback < 0) {
		Friction.speedFeedback = 0;
	}
	
	if(test_fric_off == 0) {
		FRICTION_pwmOut(Friction.speedFeedback, Friction.speedFeedback);
	}
}

/**
 *	@brief	Ħ����б�����
 */
void FRICTION_pwmCalculate(Friction_Info_t *fric)
{
	if(fric->speedFeedback < fric->speedTarget) {	// ����
		fric->speedFeedback += 5;
		if(fric->speedFeedback > fric->speedTarget) {
			fric->speedFeedback = fric->speedTarget;
		}
	} else if(fric->speedFeedback > fric->speedTarget) {	// ����
		fric ->speedFeedback -= 5;
		if(fric->speedFeedback < fric->speedTarget) {
			fric->speedFeedback = fric->speedTarget;
		}
	}
	
	if(fric->speedFeedback < 0) {
		fric->speedFeedback = 0;
	}
}

/**
 *	@brief	Ħ���ֵ��У׼
 */
void FRICTION_selfCalibration(void)
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

/* #Ӧ�ò�# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	Ħ���ָ�λ
 */
void FRICTION_reset(void)
{
	if(Friction.speedFeedback != 0) {
		FRICTION_stop(&Friction);
	} else {
		BM_reset(BitMask.System.BM_reset, BM_RESET_FRIC);	// ��λ���
	}	
}

/**
 *	@brief	�ж�Ħ�����Ƿ���
 */
bool FRICTION_ifOpen(void)
{
	if(Friction.state == FRIC_STATE_OFF)
		return false;
	else
		return true;
}

/**
 *	@brief	�ж�Ħ�����Ƿ����/���ٵ������ٶ�
 */
bool FRICTION_ifReady(void)
{
	if(Friction.state == FRIC_STATE_OFF)
		return 0;
	return (abs(Friction.speedTarget - Friction.speedFeedback)<=20)?1:0;
}

/**
 *	@brief	Ħ�����ٶ��л�
 */
void FRICTION_speedSwitch(Friction_Info_t *fric)
{
	if(FRICTION_ifReady()) {
		/* �ж��ڻ����ڱ�(̧ͷpitch��С) */
		if(GIMBAL_ifAimSentry()) {
			fric->speedLevel = FRIC_SPEED_SENTRY;
		}
		/* �����ʱ����ó������� */
		else if(GIMBAL_ifBuffMode()) {
			fric->speedLevel = FRIC_SPEED_VERYHIGH;
		}
		/* ����ģʽ�²��ø����� */
		else if(GIMBAL_ifNormalMode()) {
			fric->speedLevel = FRIC_SPEED_SENTRY;
		}
		fric->speedTarget =  Friction_Pwm_Output[Friction.speedLevel];
	}
}

/**
 *	@brief	ң������Ħ���ֿ���/�ر�
 *	@note	SW2_DOWN + SW1_UP
 */
void REMOTE_setFrictionState(RC_Ctl_t *remoteInfo)
{
	static uint8_t prev_sw1 = RC_SW_UP;
	uint8_t	sw1, sw2;
	sw1 = remoteInfo->rc.s1;
	sw2 = remoteInfo->rc.s2;
	
	if(BM_ifReset(BitMask.System.BM_reset, BM_RESET_FRIC)) {	// Ħ���ָ�λ���
		if((sw1 == RC_SW_UP && prev_sw1 != RC_SW_UP) && (sw2 == RC_SW_DOWN)) {
			if(Friction.state == FRIC_STATE_OFF) {
				Friction.state = FRIC_STATE_ON;			// ��Ħ����
				Friction.speedLevel = FRIC_SPEED_HIGH;	// ������
				Friction.speedTarget = Friction_Pwm_Output[Friction.speedLevel];
				LASER_ON();
			} else if(Friction.state == FRIC_STATE_ON){
				Friction.state = FRIC_STATE_OFF;		// �ر�Ħ����
				Friction.speedLevel = FRIC_SPEED_OFF;	// ������
				Friction.speedTarget = Friction_Pwm_Output[Friction.speedLevel];
				LASER_OFF();
			}
		}
	}
	prev_sw1 = sw1;
}

/**
 *	@brief	ң������Ħ���ֿ���/�ر�
 *	@note	����ģʽ��Ħ���ֿ����󲻹ر�(���������������)
 */
void KEY_setFrictionState(RC_Ctl_t *remoteInfo)
{
	if(BM_ifReset(BitMask.System.BM_reset, BM_RESET_FRIC)) {	// Ħ���ָ�λ���
		if(Friction.state == FRIC_STATE_OFF && IF_MOUSE_PRESSED_LEFT) {
			Friction.state = FRIC_STATE_ON;			// ��Ħ����
			Friction.speedLevel = FRIC_SPEED_HIGH;	// ������
			Friction.speedTarget = Friction_Pwm_Output[Friction.speedLevel];
			LASER_ON();
		}
//		/* #������Ҫ(��������Ҽ������ر�) */
//		if(IF_MOUSE_PRESSED_RIGH) {
//			Friction.state = FRIC_STATE_OFF;		// �ر�Ħ����
//			Friction.speedLevel = FRIC_SPEED_OFF;	// ������
//			Friction.speedTarget = Friction_Pwm_Output[Friction.speedLevel];
//			LASER_OFF();
//		}
	}
}

/* #�����# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	ң������Ħ���ֿ���/�ر�
 *	@note	Loop Time: 10ms
 */
void FRICTION_rcControlTask(void)
{
	REMOTE_setFrictionState(&RC_Ctl_Info);
}

/**
 *	@brief	��������Ħ���ֿ���/�ر�
 *	@note	Loop Time: 10ms
 */
void FRICTION_keyControlTask(void)
{
	KEY_setFrictionState(&RC_Ctl_Info);
}

/**
 *	@brief	Ħ����ʧ�ر���
 */
void FRICTION_selfProtect(void)
{
	FRICTION_stop(&Friction);
	LASER_OFF();
}

/**
 *	@brief	Ħ���ֿ���
 */
void FRICTION_control(void)
{
	/*----��Ϣ����----*/
	
	/*----�����޸�----*/
	if(BM_ifSet(BitMask.System.BM_reset, BM_RESET_FRIC)) {	// ��λ״̬
		FRICTION_reset(); // Ħ���ָ�λ
	} else {
		if(Flag.Remote.FLAG_mode == RC) {
			FRICTION_rcControlTask();
		} else if(Flag.Remote.FLAG_mode == KEY) {
			FRICTION_keyControlTask();
		}	
	}
	
	/* ������̨ģʽ�л�Ħ���ֲ��� */
	FRICTION_speedSwitch(&Friction);	
	
	/*----�������----*/
	if(test_fric_off == 0) {
		FRICTION_pwmCalculate(&Friction);	// б�¼���
		FRICTION_pwmOut(Friction.speedFeedback, Friction.speedFeedback);
	}
}
