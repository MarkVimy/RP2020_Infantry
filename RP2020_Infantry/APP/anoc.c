/**
 * @file        anoc.c
 * @author      MaWeiming
 * @Version     V1.0
 * @date        26-February-2020
 * @brief       This file includes the ANOC Protocol external functions 
 * 				(based on ST Peripherals Libaray Keil.STM32F4xx_DFP.2.9.0)
 * @Version		
 */

/**
 *	@Zigbee\anoc
 *	Zigbee结合匿名上位机无线调试
 */
 
/* Includes ------------------------------------------------------------------*/
#include "anoc.h"

#include "usart1.h"
#include "usart3.h"

/* Private typedef -----------------------------------------------------------*/
//typedef struct {
//	uint8_t sof[2];
//	uint8_t cmd_id;
//	uint8_t data_length;
//}Anoc_Frame_Header_t;

//typedef struct {
//	uint8_t buf[ANOC_MAX_DATA_LENGTH];
//}Anoc_Data_t;

//typedef struct {
//	uint8_t check_sum;
//}Anoc_Frame_Tailer_t;

//typedef struct {
//	Anoc_Frame_Header_t	FrameHeader;
//	Anoc_Data_t			Data;
//	Anoc_Frame_Tailer_t	FrameTailer;
//}Anoc_TxPacket_t;

//typedef enum {
//	ANOC_CMD_SENSER = 0x02
//}Anoc_Cmd_Names_t;

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
//static Anoc_TxPacket_t	Anoc_TxPacket = {
//	{
//		.sof[0] = 0xAA,
//		.sof[1] = 0xAA,
//		.cmd_id = ANOC_CMD_SENSER,
//		.data_length = 18,
//	},
//};

/* ## Global variables ## ----------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/* API functions -------------------------------------------------------------*/
void ANOC_SendToPc1(int16_t rol, int16_t pit, int16_t yaw)
{
	uint16_t static send_cnt = 0;
	uint8_t i;
	uint16_t check_sum = 0;
	uint8_t data_to_pc[17];
	
	data_to_pc[0] = 0xAA;
	data_to_pc[1] = 0xAA;
	data_to_pc[2] = 0x01;
	data_to_pc[3] = 12;
	
	data_to_pc[4] = (rol & 0xff00) >> 8;
	data_to_pc[5] = (rol & 0xff);
	data_to_pc[6] = (pit & 0xff00) >> 8;
	data_to_pc[7] = (pit & 0xff);
	data_to_pc[8] = (yaw & 0xff00) >> 8;
	data_to_pc[9] = (yaw & 0xff);
	
	data_to_pc[10] = 0;
	data_to_pc[11] = 0;
	data_to_pc[12] = 0;
	data_to_pc[13] = 0;
	data_to_pc[14] = 0;
	data_to_pc[15] = 0;
		
	for(i = 0; i < 16; i++) {
		check_sum += data_to_pc[i];
	}
	data_to_pc[16] = check_sum & 0xff;
	
	if(send_cnt < 5000) {
		for(i = 0; i < 17; i++) {
			USART1_SendChar(data_to_pc[i]);
		}
		send_cnt++;
	}
}

void ANOC_SendToPc(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz, int16_t mx, int16_t my, int16_t mz)
{
	uint8_t i;
	uint16_t check_sum = 0;
	uint8_t data_to_pc[23];
	
	data_to_pc[0] = 0xAA;
	data_to_pc[1] = 0xAA;
	data_to_pc[2] = 0x02;
	data_to_pc[3] = 18;
	
	data_to_pc[4] = (ax & 0xff00) >> 8;
	data_to_pc[5] = (ax & 0xff);
	data_to_pc[6] = (ay & 0xff00) >> 8;
	data_to_pc[7] = (ay & 0xff);
	data_to_pc[8] = (az & 0xff00) >> 8;
	data_to_pc[9] = (az & 0xff);
	
	data_to_pc[10] = (gx & 0xff00) >> 8;
	data_to_pc[11] = (gx & 0xff);
	data_to_pc[12] = (gy & 0xff00) >> 8;
	data_to_pc[13] = (gy & 0xff);
	data_to_pc[14] = (gz & 0xff00) >> 8;
	data_to_pc[15] = (gz & 0xff);
	
	data_to_pc[16] = (mx & 0xff00) >> 8;
	data_to_pc[17] = (mx & 0xff);
	data_to_pc[18] = (my & 0xff00) >> 8;
	data_to_pc[19] = (my & 0xff);
	data_to_pc[20] = (mz & 0xff00) >> 8;
	data_to_pc[21] = (mz & 0xff);
	
	for(i = 0; i < 22; i++) {
		check_sum += data_to_pc[i];
	}
	data_to_pc[22] = check_sum & 0xff;
	
	for(i = 0; i < 23; i++) {
		USART1_SendChar(data_to_pc[i]);
	}
}


void RP_SendToPc(float yaw, float pitch, float roll, int16_t rateYaw, int16_t ratePitch, int16_t rateRoll)
{
	static uint16_t send_cnt = 1;
	uint8_t i;
	uint16_t check_sum = 0;
	uint8_t data_to_pc[23];
	
	data_to_pc[0] = 0xAA;
	data_to_pc[1] = 0xAA;
	data_to_pc[2] = 0x01;
	data_to_pc[3] = 18;
	
	/* 以默认的小端模式发送数据 */
	memcpy(&data_to_pc[4], (uint8_t*)&yaw, 4);
	memcpy(&data_to_pc[8], (uint8_t*)&pitch, 4);
	memcpy(&data_to_pc[12], (uint8_t*)&roll, 4);
	memcpy(&data_to_pc[16], (uint8_t*)&rateYaw, 2);
	memcpy(&data_to_pc[18], (uint8_t*)&ratePitch, 2);
	memcpy(&data_to_pc[20], (uint8_t*)&rateRoll, 2);
	
//	send_cnt++;
//	以下操作会将数据转化成大端模式发送出去
//	data_to_pc[16] = (rateYaw & 0xff00) >> 8;
//	data_to_pc[17] = (rateYaw & 0xff);
//	data_to_pc[18] = (ratePitch & 0xff00) >> 8;
//	data_to_pc[19] = (ratePitch & 0xff);
//	data_to_pc[20] = (rateRoll & 0xff00) >> 8;
//	data_to_pc[21] = (rateRoll & 0xff);
	
	for(i = 0; i < 22; i++) {
		check_sum += data_to_pc[i];
	}
	data_to_pc[22] = check_sum & 0xff;
	
//	USART1_DMA_SendBuf(data_to_pc, 23);
	for(i = 0; i < 23; i++) {
		USART1_SendChar(data_to_pc[i]);
	}
}

void RP_SendToPc2(uint8_t shoot_freq, uint16_t shoot_ping, uint16_t shoot_heat, uint16_t shoot_pwm, float shoot_speed)
{
	uint8_t i;
	uint16_t check_sum = 0;
	uint8_t data_to_pc[16];	
	
	data_to_pc[0] = 0xAA;
	data_to_pc[1] = 0xAA;
	data_to_pc[2] = 0x02;
	data_to_pc[3] = 11;	
	
	memcpy(&data_to_pc[4], (uint8_t*)&shoot_freq, 1);
	memcpy(&data_to_pc[5], (uint8_t*)&shoot_ping, 2);
	memcpy(&data_to_pc[7], (uint8_t*)&shoot_heat, 2);
	memcpy(&data_to_pc[9], (uint8_t*)&shoot_pwm, 2);
	memcpy(&data_to_pc[11], (uint8_t*)&shoot_speed, 4);
	
	for(i = 0; i < 15; i++) {
		check_sum += data_to_pc[i];
	}
	data_to_pc[15] = check_sum & 0xff;	
	
	for(i = 0; i < 16; i++) {
		USART1_SendChar(data_to_pc[i]);
	}
}

void RP_SendToPc3(void)
{
	
}
