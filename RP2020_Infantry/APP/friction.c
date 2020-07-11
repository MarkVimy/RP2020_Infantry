#include "pwm.h"

/**
 * @file        pwm.c
 * @author      MaWeiming
 * @Version     V1.0
 * @date        17-October-2019
 * @brief       This file includes the FRICTION WHEEL(摩擦轮) external functions 
 * 				(based on ST Peripherals Libaray Keil.STM32F4xx_DFP.2.13.0)
 * @Version		
 */

/**
 *	# 摩擦轮电机 朗宇电机
 *	  
 *	# 摩擦轮电调 小蜜蜂电调
 *
 *	# 舵机
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
												// 关闭  低速  中速  高速  狂暴  哨兵
uint16_t Friction_Pwm_Speed[FRIC_SPEED_COUNT] = {0, 0, 2190, 2510, 2850, 2800};

uint8_t test_fric_off = 0;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* API functions -------------------------------------------------------------*/
/* #驱动层# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	摩擦轮参数初始化
 */
void FRICTION_ParamsInit(Friction_Info_t *fric)
{
	fric->State = FRIC_STATE_OFF;
	fric->SpeedLevel = FRIC_SPEED_OFF;
	fric->SpeedTarget = Friction_Pwm_Output[FRIC_SPEED_OFF];
	//fric->SpeedFeedback = 0;
}

/**
 *	@brief	摩擦轮斜坡减小输出至停止
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
 *	@brief	摩擦轮斜坡输出
 */
void FRICTION_PwmCalc(Friction_Info_t *fric)
{
	if(fric->SpeedFeedback < fric->SpeedTarget) {	// 加速
		fric->SpeedFeedback += 5;
		if(fric->SpeedFeedback > fric->SpeedTarget) {
			fric->SpeedFeedback = fric->SpeedTarget;
		}
	} else if(fric->SpeedFeedback > fric->SpeedTarget) {	// 减速
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
 *	@brief	摩擦轮电调校准
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

/* #信息层# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	判断摩擦轮是否开启
 */
bool FRICTION_IfOpen(void)
{
	if(Friction.State == FRIC_STATE_OFF)
		return false;
	else
		return true;
}

/**
 *	@brief	判断摩擦轮是否加速/减速到期望速度
 */
bool FRICTION_IfReady(void)
{
	if(Friction.State == FRIC_STATE_OFF)
		return false;
	return (abs(Friction.SpeedTarget - Friction.SpeedFeedback)<=20)?true:false;
}

/**
 *	@brief	摩擦轮获取系统信息
 */
void FRICTION_GetSysInfo(System_t *sys, Friction_Info_t *fric)
{
	/*----控制方式----*/
	/* 控制方式 - 遥控器 */
	if(sys->RemoteMode == RC) {
		fric->RemoteMode = RC;
	} 
	/* 控制方式 - 键鼠 */
	else if(sys->RemoteMode == KEY) {
		fric->RemoteMode = KEY;
	}	
}

/**
 *	@brief	摩擦轮获取系统信息
 */
void FRICTION_GetJudgeInfo(Judge_Info_t *judge, Friction_Info_t *fric)
{
	uint8_t speed_limit = JUDGE_ucGetBulletLimitSpeed17();
	
	/* #根据最高射速限制调整摩擦轮的速度 */
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
 *	@brief	摩擦轮遥控信息
 *	@note	防止出错
 */
static uint8_t prev_sw1 = RC_SW_UP;

void FRICTION_GetRemoteInfo(System_t *sys, RC_Ctl_t *remote, Friction_Info_t *fric)
{
	/* 系统正常 */
	if(sys->State == SYSTEM_STATE_NORMAL)
	{
		if(sys->RemoteMode == KEY) {
			prev_sw1 = RC_SW1_VALUE;
		}
	}
	/* 系统异常 */
	else
	{
		// 复位成初始值
		prev_sw1 = RC_SW_UP;
	}	
}

/* #应用层# ---------------------------------------------------------------------------------------------------------------------------------------*/
/* #遥控# -----------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	遥控设置摩擦轮开启/关闭
 *	@note	SW2_DOWN + SW1_UP
 */
uint16_t test_speed = 350;
void REMOTE_SetFrictionState(RC_Ctl_t *remote)
{
	uint8_t	sw1, sw2;
	sw1 = RC_SW1_VALUE;
	sw2 = RC_SW2_VALUE;
	
	if(BM_IfReset(BitMask.System.BM_Reset, BM_RESET_FRIC)) {	// 摩擦轮复位完成
		if((sw1 == RC_SW_UP && prev_sw1 != RC_SW_UP) && (sw2 == RC_SW_DOWN)) {
			if(Friction.State == FRIC_STATE_OFF) {
				Friction.State = FRIC_STATE_ON;			// 打开摩擦轮
				Friction.SpeedLevel = FRIC_SPEED_HIGH;	// 高射速
				Friction.SpeedTarget = test_speed;//Friction_Pwm_Output[Friction.SpeedLevel];
				LASER_ON();
			} else if(Friction.State == FRIC_STATE_ON){
				Friction.State = FRIC_STATE_OFF;		// 关闭摩擦轮
				Friction.SpeedLevel = FRIC_SPEED_OFF;	// 零射速
				Friction.SpeedTarget = Friction_Pwm_Output[Friction.SpeedLevel];
				LASER_OFF();
			}
		}
	}
	prev_sw1 = sw1;
}

/* #键盘鼠标# -----------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	遥控设置摩擦轮开启/关闭
 *	@note	键盘模式下摩擦轮开启后不关闭(由鼠标左键点击触发)
 */
void KEY_SetFrictionState(RC_Ctl_t *remote)
{
	if(BM_IfReset(BitMask.System.BM_Reset, BM_RESET_FRIC)) {	// 摩擦轮复位完成
		if(Friction.State == FRIC_STATE_OFF && IF_MOUSE_PRESSED_LEFT) {
			Friction.State = FRIC_STATE_ON;			// 打开摩擦轮
			Friction.SpeedLevel = FRIC_SPEED_HIGH;	// 高射速
			Friction.SpeedTarget = Friction_Pwm_Output[Friction.SpeedLevel];
			LASER_ON();
		}
	}
}

/* #摩擦轮# -----------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	摩擦轮复位
 */
void FRICTION_Reset(void)
{
	if(Friction.SpeedFeedback != 0) {
		FRICTION_Stop(&Friction);
	} else {
		BM_Reset(BitMask.System.BM_Reset, BM_RESET_FRIC);	// 复位完成
	}	
}

/**
 *	@brief	摩擦轮速度切换
 */
void FRICTION_SpeedSwitch(Friction_Info_t *fric)
{
	if(FRICTION_IfReady()) {
		/* 判断在击打哨兵(抬头pitch减小) */
		if(GIMBAL_IfAimSentry()) {
			fric->SpeedLevel = FRIC_SPEED_SENTRY;
		}
		/* 打符的时候采用超高射速 */
		else if(GIMBAL_IfBuffMode()) {
			fric->SpeedLevel = FRIC_SPEED_VERYHIGH;
		}
		/* 正常模式下采用高射速 */
		else if(GIMBAL_IfNormalMode()) {
			fric->SpeedLevel = FRIC_SPEED_SENTRY;
		}
		fric->SpeedTarget =  Friction_Pwm_Output[Friction.SpeedLevel];
	}
}

/* #任务层# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	读取外部信息
 *	@note		主要是根据性能点分配调整摩擦轮射速
 */
void FRICTION_GetInfo(void)
{
	// 获取系统信息
	FRICTION_GetSysInfo(&System, &Friction);
	// 获取裁判系统信息
	FRICTION_GetJudgeInfo(&Judge, &Friction);
	// 获取遥控信息
	FRICTION_GetRemoteInfo(&System, &Remote, &Friction);
	// 根据pwm-射速表获取射速信息
	Friction.Speed = Friction_Pwm_Speed[Friction.SpeedLevel];
}

/**
 *	@brief	遥控设置摩擦轮开启/关闭
 *	@note	Loop Time: 10ms
 */
void FRICTION_RcCtrlTask(void)
{
	REMOTE_SetFrictionState(&Remote);
}

/**
 *	@brief	按键设置摩擦轮开启/关闭
 *	@note	Loop Time: 10ms
 */
void FRICTION_KeyCtrlTask(void)
{
	KEY_SetFrictionState(&Remote);
}

/**
 *	@brief	摩擦轮失控保护
 */
void FRICTION_SelfProtect(void)
{
	FRICTION_Stop(&Friction);
	FRICTION_GetRemoteInfo(&System, &Remote, &Friction);
	LASER_OFF();
}

/**
 *	@brief	摩擦轮控制
 */
void FRICTION_Ctrl(void)
{
	/*----信息读入----*/
	FRICTION_GetInfo();
	
	/*----期望修改----*/
	if(BM_IfSet(BitMask.System.BM_Reset, BM_RESET_FRIC)) {	// 复位状态
		FRICTION_Reset(); // 摩擦轮复位
	} else {
		if(Friction.RemoteMode == RC) {
			FRICTION_RcCtrlTask();
		} else if(Friction.RemoteMode == KEY) {
			FRICTION_KeyCtrlTask();
		}	
	}
	
	/* 根据云台模式切换摩擦轮参数 */
	//FRICTION_SpeedSwitch(&Friction);	
	
	/*----最终输出----*/
	if(test_fric_off == 0) {
		FRICTION_PwmCalc(&Friction);	// 斜坡计算
		FRICTION_PwmOut(Friction.SpeedFeedback, Friction.SpeedFeedback);
	}
}
