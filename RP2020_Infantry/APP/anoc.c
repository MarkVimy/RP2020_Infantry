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
