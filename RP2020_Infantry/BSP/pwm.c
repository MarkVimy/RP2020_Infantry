/**
 * @file        pwm.c
 * @author      MaWeiming
 * @Version     V1.0
 * @date        16-October-2019
 * @brief       This file includes the PWM driver external functions 
 * 							(based on ST Peripherals Libaray Keil.STM32F4xx_DFP.2.13.0)
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
#include "pwm.h"

#include "delay.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* ## Global variables ## ----------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void TIM3_GPIO_init(void);
static void TIM1_GPIO_init(void);

/* Private functions ---------------------------------------------------------*/
/**
 *	@brief	Ħ���ֵ��GPIO����
 *	@note	PWM1 - PA6
 *			PWM2 - PA7
 */
static void TIM3_GPIO_init(void)
{
	GPIO_InitTypeDef	GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_TIM3);
}

/**
 *	@brief	���GPIO����
 *	@note	SERVO - PE11
 */
static void TIM1_GPIO_init(void)
{
	GPIO_InitTypeDef	GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1);
}

/* API functions -------------------------------------------------------------*/
/**
 *	@brief	Ħ����
 */
uint8_t test_fric_cali = 0;
void TIM3_init(void)
{
	TIM_TimeBaseInitTypeDef	tim;
	TIM_OCInitTypeDef		oc;
	
	/* GPIO��ʼ�� */
	TIM3_GPIO_init();
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	
	tim.TIM_Prescaler = 84-1;	// 1Mhz
	tim.TIM_CounterMode = TIM_CounterMode_Up;
	tim.TIM_Period = 2499;	// 25ms(ÿ����) +1����+1us
	tim.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM3, &tim);
	
	oc.TIM_OCMode = TIM_OCMode_PWM2;
	oc.TIM_OutputState = TIM_OutputState_Enable;
	oc.TIM_OutputNState = TIM_OutputState_Disable;
	oc.TIM_Pulse = 0;
	oc.TIM_OCPolarity = TIM_OCPolarity_Low;		// ������Ե�
	oc.TIM_OCNPolarity = TIM_OCPolarity_High;	
	oc.TIM_OCIdleState = TIM_OCIdleState_Reset;
	oc.TIM_OCNIdleState = TIM_OCIdleState_Set;
	
	/* PWM1 */
	TIM_OC1Init(TIM3, &oc);
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
	/* PWM2 */
	TIM_OC2Init(TIM3, &oc);
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
	TIM_ARRPreloadConfig(TIM3, ENABLE);
	
	TIM_CtrlPWMOutputs(TIM3, ENABLE);
	
	TIM_Cmd(TIM3, ENABLE);
	
	if(test_fric_cali == 0) {
		PWM1 = 1000;	// ����Ħ����(>640)
		PWM2 = 1000;
	} else {
		PWM1 = 2500;
		PWM2 = 2500;
		delay_ms(1000);
		delay_ms(1000);
		delay_ms(1000);
		delay_ms(1000);
		delay_ms(1000);
		delay_ms(1000);
		delay_ms(1000);
		delay_ms(1000);	// ����Ħ����У׼����
		PWM1 = 1000;
		PWM2 = 1000;
	}
}

/**
 *	@brief	���
 */
void TIM1_init(void)
{
	TIM_TimeBaseInitTypeDef	tim;
	TIM_OCInitTypeDef		oc;
	
	/* GPIO��ʼ�� */
	TIM1_GPIO_init();
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	
	tim.TIM_Prescaler = 3360-1;	// 50Hz
	tim.TIM_CounterMode = TIM_CounterMode_Up;
	tim.TIM_Period = 1000-1;	// 20ms(ÿ����)
	tim.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM1, &tim);
	
	oc.TIM_OCMode = TIM_OCMode_PWM2;
	oc.TIM_OutputState = TIM_OutputState_Enable;
	oc.TIM_OutputNState = TIM_OutputState_Disable;
	oc.TIM_Pulse = 0;
	oc.TIM_OCPolarity = TIM_OCPolarity_Low;		// ������Ե�
	oc.TIM_OCNPolarity = TIM_OCPolarity_High;	
	oc.TIM_OCIdleState = TIM_OCIdleState_Reset;
	oc.TIM_OCNIdleState = TIM_OCIdleState_Set;
	
	/* SERVO - TIM1->CCR2 */
	TIM_OC2Init(TIM1, &oc);
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
	
	TIM_ARRPreloadConfig(TIM1, ENABLE);
	
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
	
	TIM_Cmd(TIM1, ENABLE);
	
	SERVO = 0;
}

/**
 *	@brief	Ħ����PWM�������
 *	@note	���Ҷ�����,�Ժ���ߵ�ʱ��ע�������ߵĽӷ�	
 */
void FRICTION_pwmOut(int16_t pwm1, int16_t pwm2)
{
	PWM1 = pwm1+1000;
	PWM2 = pwm2+1000;
}

/**
 *	@brief	���PWM�������
 *	@note	
 */
void MAGZINE_pwmOut(int16_t pwm)
{
	SERVO = pwm;
}
