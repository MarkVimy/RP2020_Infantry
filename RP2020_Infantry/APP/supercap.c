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
 *	# ��������
 */

/* Includes ------------------------------------------------------------------*/
#include "supercap.h"

#include "adda.h"
#include "remote.h"
#include "anoc.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
// ���
#define Charge_On()		(CHARGE_PIN=1)
#define Charge_Off()	(CHARGE_PIN=0)

// �ŵ�
// ���ݷŵ�
#define DisCharge_On()	(DISCHARGE_PIN=0)
// ��طŵ�
#define DisCharge_Off()	(DISCHARGE_PIN=1)

/* Private variables ---------------------------------------------------------*/
// ADC�������ݵ�ѹ�ķ�ѹ����
float MEASURE_DIV_RATE=11;//#define MEASURE_DIV_RATE	1
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
 *	@brief	�������ݿ������ų�ʼ��
 */
static void SuperCAP_GPIO_Init(void)
{
	GPIO_InitTypeDef	GPIO_InitStructure;
	
	// ʹ�� GPIOA C
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	
	// ��ʼ�� ���������� - PC3 �ŵ�������� - PA5
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	// ��ʼʱ�����Ҳ���ŵ�
	Charge_Off();
	DisCharge_Off();
}

/**
 *	@brief	���ó������ݳ���ѹ(��Ӧ������)
 *	@param	uint16_t vol 0~3300������0~3.3V
 */
static void SuperCAP_DAC_SetChargeVol(uint16_t vol)
{
	double dac_val;
	dac_val = vol/3300.f*4096;
	DAC_SetChannel1Data(DAC_Align_12b_R, dac_val);
}

/**
 *	@brief	��ȡ�������ݵĵ�ѹ
 *	@return	uint16_t vol 0~3300������0~3.3V
 */
static uint16_t SuperCAP_ADC_GetCapVol(void)
{
	// ���� \ADC1 \ͨ��10 \1������ \480����������(�������߾���)
	ADC_RegularChannelConfig(ADC1, 10, 1, ADC_SampleTime_480Cycles);
	// �������ADC1ת��
	ADC_SoftwareStartConv(ADC1);
	// �ȴ�ת������
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
	
	// �������һ��ADC1�������ת�����
	return ADC_GetConversionValue(ADC1);
}

/**
 *	@brief	�������ݳ��PID�����㷨
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
	cap_ctrl->out = constrain(cap_ctrl->out, 2400, 2500);

}

/* API functions -------------------------------------------------------------*/
/* #������# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	�������ݳ�ʼ��
 */
void SuperCAP_Init(void)
{
	SuperCAP_ADC_Init();
	SuperCAP_DAC_Init();
	SuperCAP_GPIO_Init();
	
	SuperCap.info = &SuperCap_Info;
	SuperCap.ctrl = &SuperCap_Ctrl;
}

/* #��Ϣ��# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	�������ݻ�ȡϵͳ��Ϣ
 */
void SuperCAP_GetSysInfo(System_t *sys, SuperCap_Info_t *cap_info)
{
	/*----���Ʒ�ʽ----*/
	/* ���Ʒ�ʽ - ң���� */
	if(sys->RemoteMode == RC) {
		cap_info->remote_mode = RC;
	}
	/* ���Ʒ�ʽ - ���� */
	else if (sys->RemoteMode == KEY) {
		cap_info->remote_mode = KEY;
	}
}

/**
 *	@brief	�������ݻ�ȡ����ϵͳ��Ϣ
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
	
	// �����������������Ϊ���������-5
	cap->ctrl->target = cap->info->max_power - 5;
	cap->ctrl->feedback = cap->info->real_power;
}

/**
 *	@brief	�������ݻ�ȡʵ�ʵ�ѹ
 */
void SuperCAP_GetVolInfo(SuperCap_Info_t *cap_info)
{
	float tmp;
	
	// ��β���ȡƽ��ֵ
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
 *	@brief	�������ݻ�ȡȫ��������Ϣ
 */
void SuperCAP_GetInfo(void)
{
	SuperCAP_GetSysInfo(&System, &SuperCap_Info);
	SuperCAP_GetJudgeInfo(&Judge, &SuperCap);
	SuperCAP_GetVolInfo(&SuperCap_Info);
}

/* #Ӧ�ò�# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	�ŵ�
 */
void SuperCAP_Out(SuperCap_Info_t *cap_info)
{
	// �����
	cap_info->charge_status = OFF;
	/* δ����������ŵ� */
	if(cap_info->real_vol > DISCHARGE_LOWEST_VOL && cap_info->real_power_buff >= 60) {
		cap_info->discharge_status = ON;
	}
	/* �ѷ����򲻷ŵ� */
	else {
		cap_info->discharge_status = OFF;
	}
}

/**
 *	@brief	���
 */
void SuperCAP_In(SuperCap_Info_t *cap_info)
{
	// ���ŵ�
	cap_info->discharge_status = OFF;
	/* δ������������ */
	if(cap_info->real_vol < CHARGE_HIGHEST_VOL) {
		cap_info->charge_status = ON;
	} 
	/* �ѳ����򲻳�� */
	else {
		cap_info->charge_status = OFF;
	}
}

/**
 *	@brief	�Ȳ����Ҳ���ŵ�
 */
void SuperCAP_Off(SuperCap_Info_t *cap_info)
{
	cap_info->charge_status = OFF;
	cap_info->discharge_status = OFF;	
}

/**
 *	@brief	���PID����
 */
void SuperCAP_PidCtrlTask(SuperCap_t *cap)
{
	/* �����ŵ� */
	if(cap->info->discharge_status == ON) {
		DisCharge_On();
	}
	/* �رշŵ� */
	else if(cap->info->discharge_status == OFF) {
		DisCharge_Off();
	}	
	
	/* ������� */
	if(cap->info->charge_status == ON) {
		Charge_On();
		SuperCAP_PidCalc(cap->ctrl);
	}
	/* �رճ�� */
	else if(cap->info->charge_status == OFF) {
		Charge_Off();
		cap->ctrl->out = 0;
	}
	// ���ó���ѹ
	SuperCAP_DAC_SetChargeVol(cap->ctrl->out);
	

}

/* #�����# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	�������ݶϵ籣��(����粻�ŵ�)
 */
void SuperCAP_SelfProtect(void)
{
	SuperCAP_Off(&SuperCap_Info);
	SuperCAP_DAC_SetChargeVol(0);
}

/**
 *	@brief	��������ң�ؿ���
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
 *	@brief	�������ݼ��̿���
 */
void SuperCAP_KeyCtrlTask(void)
{
	
}

/**
 *	@brief	�������ݿ�������
 */
void SuperCAP_Ctrl(void)
{
	/*----��Ϣ����----*/
	SuperCAP_GetInfo();

	/*----��Ϣ���----*/	
	//RP_SendToPc()
	
	/*----�����޸�----*/
	if(SuperCap.info->remote_mode== RC) {
		SuperCAP_RcCtrlTask();
	} else if(SuperCap.info->remote_mode == KEY) {
		SuperCAP_KeyCtrlTask();
	}	
	
	/*----�������----*/
	SuperCAP_PidCtrlTask(&SuperCap);
}
