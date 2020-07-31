/**
 * @file        supercap.c
 * @author      MaWeiming
 * @Version     V1.0
 * @date        25-July-2020
 * @brief       This file includes the SuperCap external functions 
 * 				(based on ST Peripherals Libaray Keil.STM32F4xx_DFP.2.13.0)
 * @Version		
 */

/**
 *	# 超级电容
 */

/* Includes ------------------------------------------------------------------*/
#include "supercap.h"

#include "adda.h"
#include "remote.h"
#include "anoc.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
// 充电
#define Charge_On()		(CHARGE_PIN=1)
#define Charge_Off()	(CHARGE_PIN=0)

// 放电
// 电容放电
#define DisCharge_On()	(DISCHARGE_PIN=0)
// 电池放电
#define DisCharge_Off()	(DISCHARGE_PIN=1)

/* Private variables ---------------------------------------------------------*/
// ADC测量电容电压的分压比例
float MEASURE_DIV_RATE=1;//#define MEASURE_DIV_RATE	1
float DISCHARGE_LOWEST_VOL = 12;
float CHARGE_HIGHEST_VOL = 24;
float MAX_POWER = 50;

/* ## Global variables ## ----------------------------------------------------*/
SuperCap_Ctrl_t SuperCap_Ctrl = {
	.kp = 40,
	.ki = 10,
	.iout_max = 2000,
//	.out_max = 
};

SuperCap_Info_t SuperCap_Info;

SuperCap_t SuperCap;

/* Private function prototypes -----------------------------------------------*/
static void SuperCAP_GPIO_Init(void);

/* Private functions ---------------------------------------------------------*/
/**
 *	@brief	超级电容控制引脚初始化
 */
static void SuperCAP_GPIO_Init(void)
{
	GPIO_InitTypeDef	GPIO_InitStructure;
	
	// 使能 GPIOA
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
	// 初始化 充电控制引脚 - PA2 放电控制引脚 - PA5
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// 初始时不充电也不放电
	Charge_Off();
	DisCharge_Off();
}

/**
 *	@brief	设置超级电容充电电压(对应充电电流)
 *	@param	uint16_t vol 0~3300，代表0~3.3V
 */
static void SuperCAP_DAC_SetChargeVol(uint16_t vol)
{
	double dac_val;
	dac_val = vol/3300.f*4096;
	DAC_SetChannel1Data(DAC_Align_12b_R, dac_val);
}

/**
 *	@brief	获取超级电容的电压
 *	@return	uint16_t vol 0~3300，代表0~3.3V
 */
static uint16_t SuperCAP_ADC_GetCapVol(void)
{
	// 设置 \ADC1 \通道10 \1个序列 \480个采样周期(增大可提高精度)
	ADC_RegularChannelConfig(ADC1, 10, 1, ADC_SampleTime_480Cycles);
	// 软件触发ADC1转换
	ADC_SoftwareStartConv(ADC1);
	// 等待转换结束
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
	
	// 返回最近一次ADC1规则组的转换结果
	return ADC_GetConversionValue(ADC1);
}

/**
 *	@brief	超级电容充电PID控制算法
 */
void SuperCAP_PidCalc(SuperCap_Ctrl_t *cap_ctrl)
{
	cap_ctrl->err = cap_ctrl->target - cap_ctrl->feedback;
	// Pout
	cap_ctrl->pout = cap_ctrl->kp * cap_ctrl->err;
	// Iout
	cap_ctrl->iout += cap_ctrl->ki * cap_ctrl->err;
	cap_ctrl->iout = constrain(cap_ctrl->iout, -cap_ctrl->iout_max, cap_ctrl->iout_max);
	// Out
	cap_ctrl->out = cap_ctrl->pout + cap_ctrl->iout;
	cap_ctrl->out = constrain(cap_ctrl->out, 2400, 2900);

}

/* API functions -------------------------------------------------------------*/
/* #驱动层# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	超级电容初始化
 */
void SuperCAP_Init(void)
{
	SuperCAP_ADC_Init();
	SuperCAP_DAC_Init();
	SuperCAP_GPIO_Init();
	
	SuperCap.info = &SuperCap_Info;
	SuperCap.ctrl = &SuperCap_Ctrl;
}

/* #信息层# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	超级电容获取系统信息
 */
void SuperCAP_GetSysInfo(System_t *sys, SuperCap_Info_t *cap_info)
{
	/*----控制方式----*/
	/* 控制方式 - 遥控器 */
	if(sys->RemoteMode == RC) {
		cap_info->remote_mode = RC;
	}
	/* 控制方式 - 键鼠 */
	else if (sys->RemoteMode == KEY) {
		cap_info->remote_mode = KEY;
	}
}

/**
 *	@brief	超级电容获取裁判系统信息
 */
void SuperCAP_GetJudgeInfo(Judge_Info_t *judge, SuperCap_t *cap)
{
	if(judge->data_valid) {
		cap->info->real_power = JUDGE_fGetChassisRealPower();
		cap->info->real_power_buff = judge->PowerHeatData.chassis_power_buffer;
		cap->info->max_power = judge->GameRobotStatus.max_chassis_power;
	}
	else {
		cap->info->max_power = 50;
	}
	
	// 保险起见，充电最大功率为最大功率限制-5
	cap->ctrl->target = cap->info->max_power - 5;
	cap->ctrl->feedback = cap->info->real_power;
}

/**
 *	@brief	超级电容获取实际电压
 */
void SuperCAP_GetVolInfo(SuperCap_Info_t *cap_info)
{
	float tmp;
	
	// 多次测量取平均值
	for(uint8_t i = 0; i < 5; i++) {
		tmp += SuperCAP_ADC_GetCapVol();
	}
	tmp /= 5;
	
	cap_info->real_vol = 3.3f*(float)tmp/4096*MEASURE_DIV_RATE;
}

void SuperCAP_GetChassisInfo(Chassis_Info_t *chas)
{
	
}

/**
 *	@brief	超级电容获取全部输入信息
 */
void SuperCAP_GetInfo(void)
{
	SuperCAP_GetSysInfo(&System, &SuperCap_Info);
	SuperCAP_GetJudgeInfo(&Judge, &SuperCap);
	SuperCAP_GetVolInfo(&SuperCap_Info);
}

/* #应用层# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	放电
 */
void SuperCAP_Out(SuperCap_Info_t *cap_info)
{
	// 不充电
	cap_info->charge_status = OFF;
	/* 未放完则继续放电 */
	if(cap_info->real_vol > DISCHARGE_LOWEST_VOL && cap_info->real_power_buff >= 60) {
		cap_info->discharge_status = ON;
	}
	/* 已放完则不放电 */
	else {
		cap_info->discharge_status = OFF;
	}
}

/**
 *	@brief	充电
 */
void SuperCAP_In(SuperCap_Info_t *cap_info)
{
	// 不放电
	cap_info->discharge_status = OFF;
	/* 未充满则继续充电 */
	if(cap_info->real_vol < CHARGE_HIGHEST_VOL) {
		cap_info->charge_status = ON;
	} 
	/* 已充满则不充电 */
	else {
		cap_info->charge_status = OFF;
	}
}

/**
 *	@brief	既不充电也不放电
 */
void SuperCAP_Off(SuperCap_Info_t *cap_info)
{
	cap_info->charge_status = OFF;
	cap_info->discharge_status = OFF;	
}

/**
 *	@brief	充电PID控制
 */
void SuperCAP_PidCtrlTask(SuperCap_t *cap)
{
	/* 开启放电 */
	if(cap->info->discharge_status == ON) {
		DisCharge_On();
	}
	/* 关闭放电 */
	else if(cap->info->discharge_status == OFF) {
		DisCharge_Off();
	}	
	
	/* 开启充电 */
	if(cap->info->charge_status == ON) {
		Charge_On();
		SuperCAP_PidCalc(cap->ctrl);
	}
	/* 关闭充电 */
	else if(cap->info->charge_status == OFF) {
		Charge_Off();
		cap->ctrl->out = 0;
	}
	// 设置充电电压
//	SuperCAP_DAC_SetChargeVol(cap->ctrl->out);
	

}

/* #任务层# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	超级电容断电保护(不充电不放电)
 */
void SuperCAP_SelfProtect(void)
{
	SuperCAP_Off(&SuperCap_Info);
	SuperCAP_DAC_SetChargeVol(0);
}

/**
 *	@brief	超级电容遥控控制
 */
void SuperCAP_RcCtrlTask(void)
{
	uint8_t sw1;
	sw1 = RC_SW1_VALUE;
	
	switch(sw1) {
		case RC_SW_UP:	SuperCAP_In(&SuperCap_Info);
						break;
		case RC_SW_MID:	SuperCAP_Off(&SuperCap_Info);
						break;
		case RC_SW_DOWN:SuperCAP_Out(&SuperCap_Info);
						break;
		default:		SuperCAP_Off(&SuperCap_Info);	
	}
}

/**
 *	@brief	超级电容键盘控制
 */
void SuperCAP_KeyCtrlTask(void)
{
	
}

/**
 *	@brief	超级电容控制任务
 */
void SuperCAP_Ctrl(void)
{
	/*----信息读入----*/
	SuperCAP_GetInfo();

	/*----信息输出----*/	
	//RP_SendToPc()
	
	/*----期望修改----*/
	if(SuperCap.info->remote_mode== RC) {
		SuperCAP_RcCtrlTask();
	} else if(SuperCap.info->remote_mode == KEY) {
		SuperCAP_KeyCtrlTask();
	}	
	
	/*----最终输出----*/
	SuperCAP_PidCtrlTask(&SuperCap);
}
