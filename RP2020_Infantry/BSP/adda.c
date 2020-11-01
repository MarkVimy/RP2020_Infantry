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
 *	@brief	ADC GPIO配置
 *	@note	ADC1通道10 - PC0
 */
static void ADC_GPIO_Init(void)
{
	GPIO_InitTypeDef	GPIO_InitStructure;
	
	// 使能GPIOC时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);	
	
	// 初始化ADC1通道10IO口 - PC0
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;	// 模拟输入
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;// 不上下拉
	
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}

/**
 *	@brief	DAC GPIO配置
 *	@note	DAC通道1 - PA4
 */
static void DAC_GPIO_Init(void)
{
	GPIO_InitTypeDef	GPIO_InitStructure;
	
	// 使能GPIOA时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	
	
	// 初始化DAC通道10IO口 - PA4
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;	// 模拟输入
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	// 上拉
	
	GPIO_Init(GPIOA, &GPIO_InitStructure);	
}

/* Private functions ---------------------------------------------------------*/
/* API functions -------------------------------------------------------------*/
/**
 *	@brief	超级电容 ADC配置
 */
void SuperCAP_ADC_Init(void)
{
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_InitTypeDef       ADC_InitStructure;	
	
	// 通道GPIO口初始化
	ADC_GPIO_Init();
	// 使能ADC1时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); 
	// 复位ADC
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,ENABLE);	  
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,DISABLE);
	// 配置ADC
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;	// 独立模式
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;	// 两个采样阶段之间的延迟5个时钟
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;	// DMA失能 
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;	// 预分频4分频。ADCCLK=PCLK2/4=84/4=21Mhz,ADC时钟最好不要超过36Mhz 
	ADC_CommonInit(&ADC_CommonInitStructure);	// 通用配置初始化
	
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;	// 12位模式
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;			// 非扫描模式	
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;		// 关闭连续转换
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;	// 禁止触发检测，使用软件触发
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	// 右对齐
	ADC_InitStructure.ADC_NbrOfConversion = 1;	// 1个转换在规则序列中 也就是只转换规则序列1 
	ADC_Init(ADC1, &ADC_InitStructure);			// ADC配置初始化
	// 开启AD转换器
	ADC_Cmd(ADC1, ENABLE);
}

/**
 *	@brief	超级电容 ADC配置
 */
void SuperCAP_DAC_Init(void)
{
	DAC_InitTypeDef   DAC_InitStructure;
		
	// 通道GPIO口初始化
	DAC_GPIO_Init();
	// 使能DAC时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);
	// 配置DAC
	DAC_InitStructure.DAC_Trigger = DAC_Trigger_None;	// 不使用触发功能(TEN1=0)
	DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;	// 不适用波形发生
	DAC_InitStructure.DAC_LFSRUnmask_TriangleAmplitude = DAC_LFSRUnmask_Bit0;	// 屏蔽、赋值设置
	DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Disable;	// DAC1输出缓存关闭(BOFF=1)
	DAC_Init(DAC_Channel_1, &DAC_InitStructure);
	// 使能DAC通道1
	DAC_Cmd(DAC_Channel_1, ENABLE);
	// 设置12位右对齐数据格式
	DAC_SetChannel1Data(DAC_Align_12b_R, 0);
}
