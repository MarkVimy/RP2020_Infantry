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
Magzine_Handler_t Magzine = {
	.Angle.target = MAGZ_ANGLE_CLOSE,
	.Angle.feedback = MAGZ_ANGLE_CLOSE,
	
	.State = MAGZ_STATE_CLOSE,
	.RemoteMode = RC,
};

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* API functions -------------------------------------------------------------*/
/* #������# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	���ֶ��������ʼ��
 */
void MAGZINE_ParamsInit(Magzine_Handler_t *Magz)
{
	Magz->State = MAGZ_STATE_CLOSE;
	Magz->Angle.target = MAGZ_ANGLE_CLOSE;
	//Magz->Angle.feedback = MAGZ_ANGLE_CLOSE;
}

/**
 *	@brief	���ֶ��ֹͣת��(��ʱ���ֶ��򿪵���)
 */
void MAGZINE_Stop(Magzine_Handler_t *Magz)
{
	MAGZINE_ParamsInit(Magz);
	MAGZINE_PwmOut(0);
}

/**
 *	@brief	����б�����
 */
void MAGZINE_Output(Magzine_Handler_t *Magz)
{
	/* �ز� */
	if(Magz->Angle.feedback < Magz->Angle.target) 
	{	
		Magz->Angle.feedback += 5;
		if(Magz->Angle.feedback > Magz->Angle.target) {
			Magz->Angle.feedback = Magz->Angle.target;
		}
	}
	/* ���� */
	else if(Magz->Angle.feedback > Magz->Angle.target) 
	{
		Magz->Angle.feedback  -= 5;
		if(Magz->Angle.feedback < Magz->Angle.target) {
			Magz->Angle.feedback = Magz->Angle.target;
		}
	}
	
	if(Magz->Angle.feedback < 0) {
		Magz->Angle.feedback = 0;
	}

	MAGZINE_PwmOut(Magz->Angle.feedback);
}

/* #��Ϣ��# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	�Ƿ���˵���
 */
bool MAGZINE_IfOpen(void)
{
	if((Magzine.State == MAGZ_STATE_OPEN) && (Magzine.Angle.feedback == Magzine.Angle.target))
		return true;
	
	return false;
}

/**
 *	@brief	�����ѵ���Ŀ��Ƕ�(ʵ��������û�з���������ֻ�ǹ���)
 */
bool MAGZINE_IfReady(void)
{
	if(Magzine.Angle.feedback == Magzine.Angle.target)
		return true;
	
	return false;
}

/**
 *	@brief	���ֻ�ȡϵͳ��Ϣ
 */
void MAGZINE_GetSysInfo(System_t *sys, Magzine_Handler_t *magz)
{
	/*----���Ʒ�ʽ----*/
	/* ���Ʒ�ʽ - ң���� */
	if(sys->RemoteMode == RC) {
		magz->RemoteMode = RC;
	} 
	/* ���Ʒ�ʽ - ���� */
	else if(sys->RemoteMode == KEY) {
		magz->RemoteMode = KEY;
	}
	
	/*----ģʽ�޸�----*/
	switch(sys->Action)
	{
		case SYS_ACT_NORMAL: 
			{
				if(magz->RemoteMode == RC)
					magz->Mode = MAGZ_MODE_FREE;// ����
				else if(magz->RemoteMode == KEY)
					magz->Mode = MAGZ_MODE_NC;	// ����
			}break;
		case SYS_ACT_AUTO: 
			{
				magz->Mode = MAGZ_MODE_NC;	// ����
			}break;
		case SYS_ACT_BUFF:
			{
				magz->Mode = MAGZ_MODE_NC;	// ����
			}break;
		case SYS_ACT_PARK:
			{
				magz->Mode = MAGZ_MODE_NO;	// ����
			}break;
	}
}

/**
 *	@brief	���ֻ�ȡң����Ϣ
 *	@note	ʵʱ����ң����Ϣ����������
 */
static uint8_t prev_sw1 = RC_SW_UP;
static uint8_t KeyLockFlag_R = false;

void MAGZINE_GetRemoteInfo(System_t *sys, RC_Ctl_t *rc, Magzine_Handler_t *magz)
{
	/* ϵͳ���� */
	if(sys->State == SYSTEM_STATE_NORMAL)
	{
		if(sys->RemoteMode == RC) {
			KeyLockFlag_R = false;
		}
		else if(sys->RemoteMode == KEY) {
			prev_sw1 = RC_SW1_VALUE;
		}
	}
	/* ϵͳ�쳣 */
	else
	{
		// ��λ�ɳ�ʼֵ
		KeyLockFlag_R = false;
		prev_sw1 = RC_SW_UP;
	}
}


/* #Ӧ�ò�# ---------------------------------------------------------------------------------------------------------------------------------------*/
/* #ң��# -----------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	ң�����õ��ֿ���/�ر�
 *	@note		SW2_MID + SW1_UP
 */
void REMOTE_SetMagzineState(Magzine_Handler_t *Magz)
{
	uint8_t	sw1, sw2;
	sw1 = RC_SW1_VALUE;
	sw2 = RC_SW2_VALUE;
	
	if((sw1 == RC_SW_UP && prev_sw1 != RC_SW_UP) && (sw2 == RC_SW_MID)) {
		/* �رյ��� -> �򿪵��� */
		if(Magz->State == MAGZ_STATE_CLOSE) {
			Magz->State = MAGZ_STATE_OPEN;
			Magz->Angle.target = MAGZ_ANGLE_OPEN;	
		} 
		/* �򿪵��� -> �رյ���*/
		else if(Magz->State == MAGZ_STATE_OPEN){
			Magz->State = MAGZ_STATE_CLOSE;
			Magz->Angle.target = MAGZ_ANGLE_CLOSE;	
		}
	}
	prev_sw1 = sw1;
}

/* #�������# -----------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	�������õ��ֿ���/�ر�
 *	@note		���� R
 */
void KEY_SetMagzineState(Magzine_Handler_t *Magz)
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
	
	// #Ϊ�˵��Է��㣬��ʱ�޸ĳ�����
	if(IF_KEY_PRESSED_R) {	// ����R
		if(KeyLockFlag_R == false) {
			/* �رյ��� -> �򿪵��� */
			if((Magz->State == MAGZ_STATE_CLOSE) && MAGZINE_IfReady()) {
				Magz->State = MAGZ_STATE_OPEN;
				Magz->Angle.target = MAGZ_ANGLE_OPEN;	// �򿪵���
			} 
			/* �򿪵��� -> �رյ���*/
			else if((Magz->State == MAGZ_STATE_OPEN) && MAGZINE_IfReady()) {
				Magz->State = MAGZ_STATE_CLOSE;
				Magz->Angle.target = MAGZ_ANGLE_CLOSE;	// �رյ���
			}
		}
		KeyLockFlag_R = true;
	} else {	// �ɿ�R
		KeyLockFlag_R = false;
	}
}

/* #�����# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	���ֻ�ȡ�ⲿ��Ϣ�Ĵ���
 */
void MAGZINE_GetInfo(void)
{
	// ��ȡϵͳ��Ϣ
	MAGZINE_GetSysInfo(&System, &Magzine);
	// ��ȡң����Ϣ
	MAGZINE_GetRemoteInfo(&System, &Remote, &Magzine);
}

/**
 *	@brief	ң�����õ��ֿ���/�ر�
 *	@note		Loop Time: 10ms
 */
void MAGZINE_RcCtrlTask(void)
{
	// ң�ؿ�����Ĭ���ǵ���״̬
	REMOTE_SetMagzineState(&Magzine);
}

/**
 *	@brief	�������õ��ֿ���/�ر�
 *	@note		Loop Time: 10ms
 */
void MAGZINE_KeyCtrlTask(void)
{
	switch(Magzine.Mode)
	{
		case MAGZ_MODE_FREE:
			{
				// ����
				KEY_SetMagzineState(&Magzine);
			}break;
		case MAGZ_MODE_NC:
			{
				// ����
				Magzine.State = MAGZ_STATE_CLOSE;
				Magzine.Angle.target = MAGZ_ANGLE_CLOSE;				
			}break;
		case MAGZ_MODE_NO:
			{
				// ����
				Magzine.State = MAGZ_STATE_OPEN;
				Magzine.Angle.target = MAGZ_ANGLE_OPEN;			
			}break;
	}
}

/**
 *	@brief	����ʧ�ر���
 */
void MAGZINE_SelfProtect(void)
{
	MAGZINE_Stop(&Magzine);
	MAGZINE_GetRemoteInfo(&System, &Remote, &Magzine);
}

/**
 *	@brief	���ֿ���
 */
void MAGZINE_Ctrl(void)
{
	/*----��Ϣ����----*/
	MAGZINE_GetInfo();
	/*----�����޸�----*/
	if(Magzine.RemoteMode == RC) {
		MAGZINE_RcCtrlTask();
	} else if(Magzine.RemoteMode == KEY) {
		MAGZINE_KeyCtrlTask();
	}	
	/*----�������----*/
	MAGZINE_Output(&Magzine);
}
