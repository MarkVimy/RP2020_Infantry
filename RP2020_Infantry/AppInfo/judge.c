/**
 * @file        judge.c
 * @author      MaWeiming
 * @Version     V1.0
 * @date        19-October-2019
 * @brief       This file includes the JUDGE Protocols(����ϵͳЭ��) external functions 
 * 				(based on ST Peripherals Libaray Keil.STM32F4xx_DFP.2.13.0)
 * @Version		
 */

/**
 *	# ����ϵͳʹ��USART5��ΪͨѶ�ӿ�
 *	ͨ��Э���ʽ:
 *				֡ͷ		�����			���ݶ�		֡β
 *	��		frame_header 	cmd_id 			data  		frame_tail(CRC16,����У��)
 *	�ֽ�		5			  2		  		n			2
 *	
 *	֡ͷ��ʽ:
 *		  ��ʼ�ֽ�(0xA5)	����֡����		�����		֡ͷCRC8У��
 *	��			SOF 	 	data_length    	seq 	 	CRC8
 *	�ֽ�		0				2			1			1
 */

/* Includes ------------------------------------------------------------------*/
#include "judge.h"

#include "uart5.h"
#include "crc.h"

#include "Task_Revolver.h"

/* Private typedef -----------------------------------------------------------*/
/*------Cmd ID-------*/
// (0x0001)����״̬				����Ƶ��: 1Hz
// (0x0002)������� 			����Ƶ��: ������������
// (0x0003)������Ѫ������  		����Ƶ��: 1Hz
// (0x0101)�����¼�����			����Ƶ��: �¼��ı����
// (0x0102)����վ������ʶ		����Ƶ��: �����ı����
// (0x0103)���󲹸�վ�����ӵ�	����Ƶ��: ����10Hz(RM�Կ�����δ����)
// (0x0104)���о�����Ϣ			����Ƶ��: ���淢������
// (0x0201)����������״̬		����Ƶ��: 10Hz
// (0x0202)ʵʱ��������			����Ƶ��: 50Hz
// (0x0203)������λ��			����Ƶ��: 10Hz
// (0x0204)����������			����Ƶ��: ״̬�ı����
// (0x0205)���л���������״̬	����Ƶ��: 10Hz
// (0x0206)�˺�״̬				����Ƶ��: �˺���������
// (0x0207)ʵʱ�����Ϣ			����Ƶ��: �������
// (0x0208)�ӵ�ʣ�෢����		����Ƶ��: 1Hz����(���л����˺��ڱ����������ط���)
// (0x0301)�����˼佻������		����Ƶ��: ���ͷ���������(����10Hz)
typedef enum {
	ID_GAME_STATUS 					= 0x0001,	// ����״̬			
	ID_GAME_RESULT 					= 0x0002,	// ������� 		
	ID_GAME_ROBOT_HP 				= 0x0003,	// ������Ѫ������  	
	ID_EVENT_DATA 					= 0x0101,	// �����¼�����		
	ID_SUPPLY_PROJECTILE_ACTION 	= 0x0102,	// ����վ������ʶ	
	ID_SUPPLY_PROJECTILE_BOOKING 	= 0x0103,	// ���󲹸�վ�����ӵ�	
	ID_REFEREE_WARNING 				= 0x0104,	// ���о�����Ϣ
	ID_GAME_ROBOT_STATUS 			= 0x0201,	// ����������״̬
	ID_POWER_HEAT_DATA 				= 0x0202,	// ʵʱ������������
	ID_GAME_ROBOT_POS				= 0x0203,	// ������λ��
	ID_BUFF							= 0x0204,	// ����������
	ID_AERIAL_ROBOT_ENERGY			= 0x0205,	// ���л���������״̬
	ID_ROBOT_HURT					= 0x0206,	// �������˺�״̬
	ID_SHOOT_DATA					= 0x0207,	// ʵʱ�����Ϣ
	ID_BULLET_REMAINING				= 0x0208,	// �ӵ�ʣ�෢����
	ID_COMMUNICATION				= 0x0301,	// �����˼佻������(���ͷ���������)
} Judge_Cmd_ID_t;

/* ֡�ֽ�ƫ�� */
typedef enum {
	FRAME_HEADER	= 0,
	CMD_ID			= 5,
	DATA_SEG		= 7
} Judge_Frame_Offset_t;

/* ֡ͷ�ֽ�ƫ�� */
typedef enum {
	SOF			= 0,
	DATA_LENGTH	= 1,
	SEQ			= 3,
	CRC8		= 4
} Judge_Frame_Header_Offset_t;

typedef enum {
	/* Std */
	LEN_FRAME_HEAD 	= 5,	// ֡ͷ����
	LEN_CMD_ID 		= 2,	// �����볤��
	LEN_FRAME_TAIL 	= 2,	// ֡βCRC16
	/* Ext */
	LEN_GAME_STATUS 				= 3,
	LEN_GAME_RESULT 				= 1,
	LEN_GAME_ROBOT_HP 				= 28,
	LEN_EVENT_DATA					= 4,
	LEN_SUPPLY_PROJECTILE_ACTION	= 3,
	LEN_SUPPLY_PROJECTILE_BOOKING	= 2,
	LEN_REFEREE_WARNING				= 2,
	LEN_GAME_ROBOT_STATUS			= 15,
	LEN_POWER_HEAT_DATA 			= 14,
	LEN_GAME_ROBOT_POS				= 16,
	LEN_BUFF		 				= 1,
	LEN_AERIAL_ROBOT_ENERGY 		= 3,
	LEN_ROBOT_HURT					= 1,
	LEN_SHOOT_DATA					= 6,
	LEN_BULLET_REMAINING	 		= 2,
} Judge_Data_Length_t;

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#define JUDGE_FRAME_HEADER		(0xA5)

/* Private variables ---------------------------------------------------------*/
//ext_game_status_t 				GameStatus;					// 0x0001
//ext_game_result_t 				GameResult;					// 0x0002
//ext_game_robot_HP_t 			GameRobotHP;				// 0x0003
//ext_event_data_t				EventData;					// 0x0101
//ext_supply_projectile_action_t	SupplyProjectileAction;		// 0x0102
//ext_supply_projectile_booking_t SupplyProjectileBooking;	// 0x0103
//ext_referee_warning_t			RefereeWarning;				// 0x0104
//ext_game_robot_status_t			GameRobotStatus;			// 0x0201
//ext_power_heat_data_t			PowerHeatData;				// 0x0202
//ext_game_robot_pos_t			GameRobotPos;				// 0x0203
//ext_buff_t						Buff;						// 0x0204
//ext_aerial_robot_energy_t		AerialRobotEnergy;			// 0x0205
//ext_robot_hurt_t				RobotHurt;					// 0x0206
//ext_shoot_data_t				ShootData;					// 0x0207
//ext_bullet_remaining_t			BulletRemaining;			// 0x0208

//std_frame_header_t				FrameHeader;	// ֡ͷ��Ϣ

/* ## Global variables ## ----------------------------------------------------*/
Judge_Info_t Judge_Info = 
{
	.frame_length = 0,
	.cmd_id = 0,
	.err_cnt = 0,
	.data_valid = false,
};	// ������Ե�ʱ����

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* API functions -------------------------------------------------------------*/
/**
 *	@brief	����ϵͳ��ȡʵʱ����
 *	@note
 *			# ���ݹ���һ����(�ж�+����)�ĳ�����ķ�ʽ�Ͳ������HardFault
 *				���ھ���ԭ��û�����
 *			# ����DMA�Ļػ����ݴ�С(Buffer Size)����ʹ�ô������ӽϻ���
 */
bool JUDGE_readData(uint8_t *rxBuf)
{
	uint8_t  res = false;
	uint16_t frame_length;
	uint16_t cmd_id;	

	if( rxBuf == NULL )
	{
		return -1;
	}
	
	memcpy(&Judge_Info.FrameHeader, rxBuf, LEN_FRAME_HEAD);
	
	/* ֡���ֽ��Ƿ�Ϊ0xA5 */
	if(rxBuf[SOF] == JUDGE_FRAME_HEADER) 
	{
		/* ֡ͷCRC8У�� */
		if(Verify_CRC8_Check_Sum(rxBuf, LEN_FRAME_HEAD) == true) 
		{
			/* ͳ��һ֡�������ݳ��ȣ�����CRC16У�� */
			frame_length = LEN_FRAME_HEAD + LEN_CMD_ID + Judge_Info.FrameHeader.data_length + LEN_FRAME_TAIL;
			Judge_Info.frame_length = frame_length;
			
			if(Verify_CRC16_Check_Sum(rxBuf, frame_length) == true)
			{
				res = true;
				
				cmd_id = (rxBuf[CMD_ID+1] << 8 | rxBuf[CMD_ID]);
				Judge_Info.cmd_id = cmd_id;
				
				switch(cmd_id)
				{
					case ID_GAME_STATUS:
						memcpy(&Judge_Info.GameStatus, (rxBuf+DATA_SEG), LEN_GAME_STATUS);
						break;
					case ID_GAME_RESULT:
						memcpy(&Judge_Info.GameResult,  (rxBuf+DATA_SEG), LEN_GAME_RESULT);
						break;
					case ID_GAME_ROBOT_HP:
						memcpy(&Judge_Info.GameRobotHP, (rxBuf+DATA_SEG), LEN_GAME_ROBOT_HP);
						break;
					case ID_EVENT_DATA:
						memcpy(&Judge_Info.EventData, (rxBuf+DATA_SEG), LEN_EVENT_DATA);
						break;
					case ID_SUPPLY_PROJECTILE_ACTION:
						memcpy(&Judge_Info.SupplyProjectileAction, (rxBuf+DATA_SEG), LEN_SUPPLY_PROJECTILE_ACTION);
						break;
					case ID_SUPPLY_PROJECTILE_BOOKING:
						memcpy(&Judge_Info.SupplyProjectileBooking, (rxBuf+DATA_SEG), LEN_SUPPLY_PROJECTILE_BOOKING);
						break;
					case ID_REFEREE_WARNING:
						memcpy(&Judge_Info.RefereeWarning, (rxBuf+DATA_SEG), LEN_REFEREE_WARNING);
						break;
					case ID_GAME_ROBOT_STATUS:
						memcpy(&Judge_Info.GameRobotStatus, (rxBuf+DATA_SEG), LEN_GAME_ROBOT_STATUS);
						break;
					case ID_POWER_HEAT_DATA:
						memcpy(&Judge_Info.PowerHeatData, (rxBuf+DATA_SEG), LEN_POWER_HEAT_DATA);
						break;
					case ID_GAME_ROBOT_POS:
						memcpy(&Judge_Info.GameRobotPos, (rxBuf+DATA_SEG), LEN_GAME_ROBOT_POS);
						break;
					case ID_BUFF:
						memcpy(&Judge_Info.Buff, (rxBuf+DATA_SEG), LEN_BUFF);
						break;
					case ID_AERIAL_ROBOT_ENERGY:
						memcpy(&Judge_Info.AerialRobotEnergy, (rxBuf+DATA_SEG), LEN_AERIAL_ROBOT_ENERGY);
						break;
					case ID_ROBOT_HURT:
						memcpy(&Judge_Info.RobotHurt, (rxBuf+DATA_SEG), LEN_ROBOT_HURT);
						break;
					case ID_SHOOT_DATA:
						memcpy(&Judge_Info.ShootData, (rxBuf+DATA_SEG), LEN_SHOOT_DATA);
						break;
					case ID_BULLET_REMAINING:
						memcpy(&Judge_Info.BulletRemaining, (rxBuf+DATA_SEG), LEN_BULLET_REMAINING);
						break;
					case ID_COMMUNICATION:
						break;
				}
			}
		}

		/* ֡βCRC16��һ�ֽ��Ƿ�Ϊ0xA5 */
		if(rxBuf[ frame_length ] == JUDGE_FRAME_HEADER)
		{
			/* ���һ�����ݰ������˶�֡���ݾ��ٴζ�ȡ */
			JUDGE_readData( &rxBuf[frame_length] );
		}
	}
	
	Judge_Info.data_valid = res;
	if(Judge_Info.data_valid != true)
	{
		Judge_Info.err_cnt++;
		Judge_Info.data_valid = false;
	}
	else
	{
		Judge_Info.data_valid = true;
	}
		
	return res;
}

/**
 *	@brief	�����жϴ�ʱ����ϵͳ���ݵ���ȷ��
 */
bool JUDEG_ifDataValid(void)
{
	return Judge_Info.data_valid;
}

/**
  * @brief  ͳ�Ʒ�����
  * @param  void
  * @retval void
  * @attention  
  */
portTickType shoot_time;//������ʱ����
portTickType shoot_ping;//����������շ����ӳ�
float Shoot_Speed_Now = 0;
float Shoot_Speed_Last = 0;
void JUDGE_shootNumCount(void)
{
	Shoot_Speed_Now = Judge_Info.ShootData.bullet_speed;
	if(Shoot_Speed_Last != Shoot_Speed_Now)//��Ϊ��float�ͣ�������������ȫ���,�����ٶȲ���ʱ˵��������һ�ŵ�
	{
		REVOLVER_addShootCount();
		Shoot_Speed_Last = Shoot_Speed_Now;
	}
	shoot_time = xTaskGetTickCount();//��ȡ���跢��ʱ��ϵͳʱ��
	shoot_ping = shoot_time - REVOLVER_getRealShootTime();//�����ӳ�
}
