#include "sys.h"  
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//ϵͳʱ�ӳ�ʼ��	
//����ʱ������/�жϹ���/GPIO���õ�
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/5/2
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved
//********************************************************************************
//�޸�˵��
//��
//////////////////////////////////////////////////////////////////////////////////  


//THUMBָ�֧�ֻ������
//�������·���ʵ��ִ�л��ָ��WFI  
__asm void WFI_SET(void)
{
	WFI;		  
}
//�ر������ж�(���ǲ�����fault��NMI�ж�)
__asm void INTX_DISABLE(void)
{
	CPSID   I
	BX      LR	  
}
//���������ж�
__asm void INTX_ENABLE(void)
{
	CPSIE   I
	BX      LR  
}
//����ջ����ַ
//addr:ջ����ַ
__asm void MSR_MSP(u32 addr) 
{
	MSR MSP, r0 			//set Main Stack value
	BX r14
}

float myDeathZoom(float input, float death)
{
	if(abs(input) < death)
		return 0.f;
	return input;
}

/**
  * @brief  б�º���,ʹĿ�����ֵ������������ֵ
  * @param  �����������,��ǰ���,�仯�ٶ�(Խ��Խ��)
  * @retval ��ǰ���
  * @attention  
  */
float RAMP_float(float final, float now, float ramp )
{
	  float buffer = 0;
	
	
	  buffer = final - now;
	
		if (buffer > 0)
		{
				if (buffer > ramp)
				{  
						now += ramp;
				}   
				else
				{
						now += buffer;
				}
		}
		else
		{
				if (buffer < -ramp)
				{
						now += -ramp;
				}
				else
				{
						now += buffer;
				}
		}
		
		return now;
}

/**
 *	@brief	�Ȳ��������㷨
 */
float STEP_float(float err, float step, uint8_t *stepCnt, float death)
{
	if(*stepCnt>0) {
		(*stepCnt)--;
		if(abs(err)<death)
			return 0.f;
		else
			return err/step;
	}
	return 0.f;
}

void Bubble_sort_float(uint16_t num, float *arr)
{
	uint16_t i, j;
	float tmp = 0;
	bool isSorted = true;
	uint16_t lastExchangeIndex = 0;	// ��һ�ν���Ԫ�������±�
	uint16_t sortBorder = num - 1;	// �������б߽�
	for(i = 0; i < num - 1; i++) {
		isSorted = true;
		for(j = 0; j < sortBorder; j++) {
			if(arr[j] > arr[j + 1]) {
				tmp = arr[j];
				arr[j] = arr[j + 1];
				arr[j + 1] = tmp;
				isSorted = false;
				lastExchangeIndex = j;
			}
		}
		sortBorder = lastExchangeIndex;
		
		if(isSorted) {	// һ�˱Ƚ�����û�н���Ԫ����˵�������Ѿ�����
			break;
		}
	}	
}

/*************��λֵƽ�������˲�***************/
float Median_filter_float(float data,int measureNum,float *Filterdata)
{
	unsigned int i = 0;
	unsigned int MAX_error_targe = 0;
	float MAX_error1;
	Filterdata[measureNum-1] = data;
	for(i=0;i<measureNum-1;i++)	//--������������
	{
	 Filterdata[i]=Filterdata[i+1];
	}
	Bubble_sort_float(measureNum, Filterdata);	// ����ð������[0]��С��
	MAX_error1 = abs(Filterdata[1] - Filterdata[0]);	//--�������΢�ֳ�ֵ��б�ʣ�
	for(i = 1 ; i < measureNum-1 ; i++)
	{
			if(MAX_error1 < abs(Filterdata[i+1] - Filterdata[i]) )
			{
					MAX_error1 =  abs(Filterdata[i+1] - Filterdata[i]);
					MAX_error_targe = i; //--������΢��ֵ�±�
			}
	}
	float Average_data=0;
	if(MAX_error_targe+1 > (measureNum+1)/2)	//--���������������ǰ�ߵ����ݣ�Ȼ���ʣ��������ƽ��ֵ
	{
			for(i = 0 ; i <= MAX_error_targe ; i++)
			{
					Average_data += Filterdata[i];
			}
			Average_data /= (MAX_error_targe+1);
	}
	else
	{
			for(i = MAX_error_targe + 1 ; i < measureNum ; i++)
			{
					Average_data += Filterdata[i];
			}
			Average_data /= (measureNum - MAX_error_targe -1);
	}
	return Average_data;	
}


