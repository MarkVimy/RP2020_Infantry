/**
 * @file        magzine.c
 * @author      MaWeiming
 * @Version     V1.0
 * @date        18-October-2019
 * @brief       This file includes the SERVO(舵机) external functions 
 * 				(based on ST Peripherals Libaray Keil.STM32F4xx_DFP.2.13.0)
 * @Version		
 */

/**
 *	# 舵机
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
/* #驱动层# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	弹仓舵机参数初始化
 */
void MAGZINE_ParamsInit(Magzine_Handler_t *Magz)
{
	Magz->State = MAGZ_STATE_CLOSE;
	Magz->Angle.target = MAGZ_ANGLE_CLOSE;
	//Magz->Angle.feedback = MAGZ_ANGLE_CLOSE;
}

/**
 *	@brief	弹仓舵机停止转动(此时可手动打开弹仓)
 */
void MAGZINE_Stop(Magzine_Handler_t *Magz)
{
	MAGZINE_ParamsInit(Magz);
	MAGZINE_PwmOut(0);
}

/**
 *	@brief	弹仓斜坡输出
 */
void MAGZINE_Output(Magzine_Handler_t *Magz)
{
	/* 关仓 */
	if(Magz->Angle.feedback < Magz->Angle.target) 
	{	
		Magz->Angle.feedback += 5;
		if(Magz->Angle.feedback > Magz->Angle.target) {
			Magz->Angle.feedback = Magz->Angle.target;
		}
	}
	/* 开仓 */
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

/* #信息层# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	是否打开了弹仓
 */
bool MAGZINE_IfOpen(void)
{
	if((Magzine.State == MAGZ_STATE_OPEN) && (Magzine.Angle.feedback == Magzine.Angle.target))
		return true;
	
	return false;
}

/**
 *	@brief	弹仓已到达目标角度(实际上由于没有反馈，这里只是估计)
 */
bool MAGZINE_IfReady(void)
{
	if(Magzine.Angle.feedback == Magzine.Angle.target)
		return true;
	
	return false;
}

/**
 *	@brief	弹仓获取系统信息
 */
void MAGZINE_GetSysInfo(System_t *sys, Magzine_Handler_t *magz)
{
	/*----控制方式----*/
	/* 控制方式 - 遥控器 */
	if(sys->RemoteMode == RC) {
		magz->RemoteMode = RC;
	} 
	/* 控制方式 - 键鼠 */
	else if(sys->RemoteMode == KEY) {
		magz->RemoteMode = KEY;
	}
	
	/*----模式修改----*/
	switch(sys->Action)
	{
		case SYS_ACT_NORMAL: 
			{
				if(magz->RemoteMode == RC)
					magz->Mode = MAGZ_MODE_FREE;// 调试
				else if(magz->RemoteMode == KEY)
					magz->Mode = MAGZ_MODE_NC;	// 常闭
			}break;
		case SYS_ACT_AUTO: 
			{
				magz->Mode = MAGZ_MODE_NC;	// 常闭
			}break;
		case SYS_ACT_BUFF:
			{
				magz->Mode = MAGZ_MODE_NC;	// 常闭
			}break;
		case SYS_ACT_PARK:
			{
				magz->Mode = MAGZ_MODE_NO;	// 常开
			}break;
	}
}

/**
 *	@brief	弹仓获取遥控信息
 *	@note	实时更新遥控信息，防出错用
 */
static uint8_t prev_sw1 = RC_SW_UP;
static uint8_t KeyLockFlag_R = false;

void MAGZINE_GetRemoteInfo(System_t *sys, RC_Ctl_t *rc, Magzine_Handler_t *magz)
{
	/* 系统正常 */
	if(sys->State == SYSTEM_STATE_NORMAL)
	{
		if(sys->RemoteMode == RC) {
			KeyLockFlag_R = false;
		}
		else if(sys->RemoteMode == KEY) {
			prev_sw1 = RC_SW1_VALUE;
		}
	}
	/* 系统异常 */
	else
	{
		// 复位成初始值
		KeyLockFlag_R = false;
		prev_sw1 = RC_SW_UP;
	}
}


/* #应用层# ---------------------------------------------------------------------------------------------------------------------------------------*/
/* #遥控# -----------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	遥控设置弹仓开启/关闭
 *	@note		SW2_MID + SW1_UP
 */
void REMOTE_SetMagzineState(Magzine_Handler_t *Magz)
{
	uint8_t	sw1, sw2;
	sw1 = RC_SW1_VALUE;
	sw2 = RC_SW2_VALUE;
	
	if((sw1 == RC_SW_UP && prev_sw1 != RC_SW_UP) && (sw2 == RC_SW_MID)) {
		/* 关闭弹仓 -> 打开弹仓 */
		if(Magz->State == MAGZ_STATE_CLOSE) {
			Magz->State = MAGZ_STATE_OPEN;
			Magz->Angle.target = MAGZ_ANGLE_OPEN;	
		} 
		/* 打开弹仓 -> 关闭弹仓*/
		else if(Magz->State == MAGZ_STATE_OPEN){
			Magz->State = MAGZ_STATE_CLOSE;
			Magz->Angle.target = MAGZ_ANGLE_CLOSE;	
		}
	}
	prev_sw1 = sw1;
}

/* #键盘鼠标# -----------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	键盘设置弹仓开启/关闭
 *	@note		按键 R
 */
void KEY_SetMagzineState(Magzine_Handler_t *Magz)
{
//	static uint16_t timeToClose = 0;
//	
//		
//	if(IF_KEY_PRESSED_R) {	// 按下R
//		Magzine.state = MAGZ_STATE_OPEN;
//		Magzine.angleTarget = MAGZ_ANGLE_OPEN;	// 打开弹仓
//		timeToClose = 500;	// 500*20ms = 10s自动关闭
//	} else {	// 松开R
//		if(timeToClose) {	// 开始倒计时
//			timeToClose--;
//		} else {
//			Magzine.state = MAGZ_STATE_CLOSE;
//			Magzine.angleTarget = MAGZ_ANGLE_CLOSE;	// 关闭弹仓
//		}
//	}
	
	if(IF_KEY_PRESSED_R) {	// 按下R
		if(KeyLockFlag_R == false) {
			/* 关闭弹仓 -> 打开弹仓 */
			if((Magz->State == MAGZ_STATE_CLOSE) && MAGZINE_IfReady()) {
				Magz->State = MAGZ_STATE_OPEN;
				Magz->Angle.target = MAGZ_ANGLE_OPEN;	// 打开弹仓
			} 
			/* 打开弹仓 -> 关闭弹仓*/
			else if((Magz->State == MAGZ_STATE_OPEN) && MAGZINE_IfReady()) {
				Magz->State = MAGZ_STATE_CLOSE;
				Magz->Angle.target = MAGZ_ANGLE_CLOSE;	// 关闭弹仓
			}
		}
		KeyLockFlag_R = true;
	} else {	// 松开R
		KeyLockFlag_R = false;
	}
}

/* #任务层# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	弹仓获取外部信息的窗口
 */
void MAGZINE_GetInfo(void)
{
	// 获取系统信息
	MAGZINE_GetSysInfo(&System, &Magzine);
	// 获取遥控信息
	MAGZINE_GetRemoteInfo(&System, &Remote, &Magzine);
}

/**
 *	@brief	遥控设置弹仓开启/关闭
 *	@note		Loop Time: 10ms
 */
void MAGZINE_RcCtrlTask(void)
{
	// 遥控控制下默认是调试状态
	REMOTE_SetMagzineState(&Magzine);
}

/**
 *	@brief	按键设置弹仓开启/关闭
 *	@note		Loop Time: 10ms
 */
void MAGZINE_KeyCtrlTask(void)
{
	switch(Magzine.Mode)
	{
		case MAGZ_MODE_FREE:
			{
				// 调试
				KEY_SetMagzineState(&Magzine);
			}break;
		case MAGZ_MODE_NC:
			{
				// 常闭
				Magzine.State = MAGZ_STATE_CLOSE;
				Magzine.Angle.target = MAGZ_ANGLE_CLOSE;				
			}break;
		case MAGZ_MODE_NO:
			{
				// 常开
				Magzine.State = MAGZ_STATE_OPEN;
				Magzine.Angle.target = MAGZ_ANGLE_OPEN;			
			}break;
	}
}

/**
 *	@brief	弹仓失控保护
 */
void MAGZINE_SelfProtect(void)
{
	MAGZINE_Stop(&Magzine);
	MAGZINE_GetRemoteInfo(&System, &Remote, &Magzine);
}

/**
 *	@brief	弹仓控制
 */
void MAGZINE_Ctrl(void)
{
	/*----信息读入----*/
	MAGZINE_GetInfo();
	/*----期望修改----*/
	if(Magzine.RemoteMode == RC) {
		MAGZINE_RcCtrlTask();
	} else if(Magzine.RemoteMode == KEY) {
		MAGZINE_KeyCtrlTask();
	}	
	/*----最终输出----*/
	MAGZINE_Output(&Magzine);
}
