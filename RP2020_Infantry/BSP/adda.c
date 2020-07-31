/**
 * @file        adda.c
 * @author      MaWeiming
 * @Version     V1.0
 * @date        25-July-2020
 * @brief       This file includes the ADC/DAC driver external functions 
 * 				(based on ST Peripherals Libaray Keil.STM32F4xx_DFP.2.9.0)
 * @Version		
 */
 
/* Includes ------------------------------------------------------------------*/
#include "adda.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* ## Global variables ## ----------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/**
 *	@brief	ADC GPIO����
 *	@note	ADC1ͨ��10 - PC0
 */
static void ADC_GPIO_Init(void)
{
	GPIO_InitTypeDef	GPIO_InitStructure;
	
	// ʹ��GPIOCʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);	
	
	// ��ʼ��ADC1ͨ��10IO�� - PC0
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;	// ģ������
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;// ��������
	
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}

/**
 *	@brief	DAC GPIO����
 *	@note	DACͨ��1 - PA4
 */
static void DAC_GPIO_Init(void)
{
	GPIO_InitTypeDef	GPIO_InitStructure;
	
	// ʹ��GPIOAʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	
	
	// ��ʼ��DACͨ��10IO�� - PA4
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;	// ģ������
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	// ����
	
	GPIO_Init(GPIOA, &GPIO_InitStructure);	
}

/* Private functions ---------------------------------------------------------*/
/* API functions -------------------------------------------------------------*/
/**
 *	@brief	�������� ADC����
 */
void SuperCAP_ADC_Init(void)
{
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_InitTypeDef       ADC_InitStructure;	
	
	// ͨ��GPIO�ڳ�ʼ��
	ADC_GPIO_Init();
	// ʹ��ADC1ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); 
	// ��λADC
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,ENABLE);	  
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,DISABLE);
	// ����ADC
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;	// ����ģʽ
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;	// ���������׶�֮����ӳ�5��ʱ��
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;	// DMAʧ�� 
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;	// Ԥ��Ƶ4��Ƶ��ADCCLK=PCLK2/4=84/4=21Mhz,ADCʱ����ò�Ҫ����36Mhz 
	ADC_CommonInit(&ADC_CommonInitStructure);	// ͨ�����ó�ʼ��
	
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;	// 12λģʽ
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;			// ��ɨ��ģʽ	
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;		// �ر�����ת��
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;	// ��ֹ������⣬ʹ���������
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	// �Ҷ���
	ADC_InitStructure.ADC_NbrOfConversion = 1;	// 1��ת���ڹ��������� Ҳ����ֻת����������1 
	ADC_Init(ADC1, &ADC_InitStructure);			// ADC���ó�ʼ��
	// ����ADת����
	ADC_Cmd(ADC1, ENABLE);
}

/**
 *	@brief	�������� ADC����
 */
void SuperCAP_DAC_Init(void)
{
	DAC_InitTypeDef   DAC_InitStructure;
	
	// ͨ��GPIO�ڳ�ʼ��
	DAC_GPIO_Init();
	// ����DAC
	DAC_InitStructure.DAC_Trigger = DAC_Trigger_None;	// ��ʹ�ô�������(TEN1=0)
	DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;	// �����ò��η���
	DAC_InitStructure.DAC_LFSRUnmask_TriangleAmplitude = DAC_LFSRUnmask_Bit0;	// ���Ρ���ֵ����
	DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Disable;	// DAC1�������ر�(BOFF=1)
	DAC_Init(DAC_Channel_1, &DAC_InitStructure);
	// ʹ��DACͨ��1
	DAC_Cmd(DAC_Channel_1, ENABLE);
	// ����12λ�Ҷ������ݸ�ʽ
	DAC_SetChannel1Data(DAC_Align_12b_R, 0);
}
