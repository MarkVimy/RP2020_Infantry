/**
 * @file        magzine.c
 * @author      MaWeiming
 * @Version     V1.0
 * @date        18-October-2019
 * @brief       This file includes the SERVO(���) external functions 
 * 				(based on ST Peripherals Libaray Keil.STM32F4xx_DFP.2.13.0)
 * @Version		
 */

/**
 *	# ���
 */

/* Includes ------------------------------------------------------------------*/
#include "magzine.h"

#include "pwm.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* ## Global variables ## ----------------------------------------------------*/
Magzine_Info_t Magzine = {
	.state = MAGZ_STATE_CLOSE,
	.angleTarget = MAGZ_ANGLE_CLOSE,
	.angleFeedback = MAGZ_ANGLE_CLOSE
};

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* API functions -------------------------------------------------------------*/
/* #������# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	���ֶ��������ʼ��
 */
void MAGZINE_ParamsInit(Magzine_Info_t *magz)
{
	magz->state = MAGZ_STATE_CLOSE;
	magz->angleTarget = MAGZ_ANGLE_CLOSE;
	//magz->angleFeedback = MAGZ_ANGLE_CLOSE;
}

/**
 *	@brief	���ֶ��ֹͣת��(��ʱ���ֶ��򿪵���)
 */
void MAGZINE_stop(Magzine_Info_t *magz)
{
	MAGZINE_ParamsInit(magz);
	MAGZINE_pwmOut(0);
}

/**
 *	@brief	����б�����
 */
void MAGZINE_output(Magzine_Info_t *magz)
{
	if(magz->angleFeedback < magz->angleTarget) {	// ����
		magz->angleFeedback += 5;
		if(magz->angleFeedback > magz->angleTarget) {
			magz->angleFeedback = magz->angleTarget;
		}
	} else if(magz->angleFeedback > magz->angleTarget) {	// ����
		magz ->angleFeedback -= 5;
		if(magz->angleFeedback < magz->angleTarget) {
			magz->angleFeedback = magz->angleTarget;
		}
	}
	
	if(magz->angleFeedback < 0) {
		magz->angleFeedback = 0;
	}

	MAGZINE_pwmOut(Magzine.angleFeedback);
}

/* #��Ϣ��# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	�Ƿ���˵���
 */
bool MAGZINE_ifOpen(void)
{
	if((Magzine.state == MAGZ_STATE_OPEN) && (Magzine.angleFeedback == Magzine.angleTarget))
		return true;
	
	return false;
}

/* #Ӧ�ò�# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	ң�����õ��ֿ���/�ر�
 *	@note	SW2_MID + SW1_UP
 */
void REMOTE_setMagzineState(RC_Ctl_t *remoteInfo)
{
	static uint8_t prev_sw1 = RC_SW_UP;
	uint8_t	sw1, sw2;
	sw1 = remoteInfo->rc.s1;
	sw2 = remoteInfo->rc.s2;
	
	if((sw1 == RC_SW_UP && prev_sw1 != RC_SW_UP) && (sw2 == RC_SW_MID)) {
		if(Magzine.state == MAGZ_STATE_CLOSE) {
			Magzine.state = MAGZ_STATE_OPEN;
			Magzine.angleTarget = MAGZ_ANGLE_OPEN;	// �򿪵���
		} else if(Magzine.state == MAGZ_STATE_OPEN){
			Magzine.state = MAGZ_STATE_CLOSE;
			Magzine.angleTarget = MAGZ_ANGLE_CLOSE;	// �رյ���
		}
	}
	prev_sw1 = sw1;
}

/**
 *	@brief	ң�����õ��ֿ���/�ر�
 *	@note	���� R
 */
void KEY_setMagzineState(RC_Ctl_t *remoteInfo)
{
//	static uint16_t timeToClose = 0;
//	
//		
//	if(IF_KEY_PRESSED_R) {	// ����R
//		Magzine.state = MAGZ_STATE_OPEN;
//		Magzine.angleTarget = MAGZ_ANGLE_OPEN;	// �򿪵���
//		timeToClose = 500;	// 500*20ms = 10s�Զ��ر�
//	} else {	// �ɿ�R
//		if(timeToClose) {	// ��ʼ����ʱ
//			timeToClose--;
//		} else {
//			Magzine.state = MAGZ_STATE_CLOSE;
//			Magzine.angleTarget = MAGZ_ANGLE_CLOSE;	// �رյ���
//		}
//	}
	
	static uint8_t keyRLockFlag = false;
	if(IF_KEY_PRESSED_R) {	// ����R
		if(keyRLockFlag == false) {
			if((Magzine.state == MAGZ_STATE_CLOSE) && (Magzine.angleFeedback == Magzine.angleTarget)) {
				Magzine.state = MAGZ_STATE_OPEN;
				Magzine.angleTarget = MAGZ_ANGLE_OPEN;	// �򿪵���
			} else if((Magzine.state == MAGZ_STATE_OPEN) && (Magzine.angleFeedback == Magzine.angleTarget)) {
				Magzine.state = MAGZ_STATE_CLOSE;
				Magzine.angleTarget = MAGZ_ANGLE_CLOSE;	// �رյ���
			}
		}
		keyRLockFlag = true;
	} else {	// �ɿ�R
		keyRLockFlag = false;
	}
}

/* #�����# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	ң�����õ��ֿ���/�ر�
 *	@note	Loop Time: 20ms
 */
void MAGZINE_rcControlTask(void)
{
	REMOTE_setMagzineState(&RC_Ctl_Info);
}

/**
 *	@brief	�������õ��ֿ���/�ر�
 *	@note	Loop Time: 20ms
 */
void MAGZINE_keyControlTask(void)
{
	KEY_setMagzineState(&RC_Ctl_Info);
}

/**
 *	@brief	����ʧ�ر���
 */
void MAGZINE_selfProtect(void)
{
	MAGZINE_stop(&Magzine);
}

/**
 *	@brief	���ֿ���
 */
void MAGZINE_control(void)
{
	/*----��Ϣ����----*/
	
	/*----�����޸�----*/
	if(Flag.Remote.FLAG_mode == RC) {
		MAGZINE_rcControlTask();
	} else if(Flag.Remote.FLAG_mode == KEY) {
		MAGZINE_keyControlTask();
	}	
	/*----�������----*/
	MAGZINE_output(&Magzine);
}
