/**
 * @file        usart.c
 * @author      MaWeiming
 * @Version     V1.0
 * @date        26-March-2019
 * @brief       This file includes the USART driver external functions 
 * 							(based on ST Peripherals Libaray F10x V3.5)
 *
 * @Verison			V1.1 (5-June-2019)
 *								- Add USART DMA(receive random length)
 *							V1.2 (15-June-2019)
 *								- Add USARTx send
 *							V1.3 (3-August-2019)
 *								- Finished USART1~3 API functions
 */

/* Includes ------------------------------------------------------------------*/
#include "usart.h"

 //////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32������
//����1��ʼ��		   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2012/8/18
//�汾��V1.5
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved
//********************************************************************************
#define DEBUG

//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
#if 1

#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
void _sys_exit(int x) 
{ 
	x = x; 
} 

//�ض���fputc���� 
int fputc(int ch, FILE *f)
{      
//#ifdef DEBUG
//	while((USART3->SR&0X40)==0);//ѭ������,ֱ���������   
//    USART3->DR = (uint8_t) ch;      
//	return ch;
//#endif
	return ch;
}

#endif 





