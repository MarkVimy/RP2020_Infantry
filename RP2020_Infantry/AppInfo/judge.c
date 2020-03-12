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
 *		  ��ʼ�ֽ�(0xA5)		����֡����		�����		֡ͷCRC8У��
 *	��			SOF 	 	data_length    	seq 	 	CRC8
 *	�ֽ�		0				2			1			1
 */

/*------Cmd ID-------*/
/**
 *	-- =>	��һ���Ѵ���
 *	++ =>	��������
 *	xx =>	����ȡ��
 *	// =>	�����޸�
 */
/*
	--19	(0x0001)����״̬				����Ƶ��: 1Hz
	--19	(0x0002)������� 			����Ƶ��: ������������
	--19	(0x0003)������Ѫ������  		����Ƶ��: 1Hz
	++20	(0x0004)���ڷ���״̬			����Ƶ��: ���ڷ���ʱ����
	++20	(0x0005)�˹�������ս���ӳ���ͷ���״̬	����Ƶ��: 1Hz
	
	--19	(0x0101)�����¼�����			����Ƶ��: �¼��ı����
	--19	(0x0102)����վ������ʶ		����Ƶ��: �����ı����
	--19	(0x0103)���󲹸�վ�����ӵ�	����Ƶ��: ����10Hz(RM�Կ�����δ����)
	xx20	(0x0103)��ȡ��
	--19	(0x0104)���о�����Ϣ			����Ƶ��: ���淢������
	++20	(0x0105)���ڷ���ڵ���ʱ		����Ƶ��: 1Hz
	
	--19	(0x0201)����������״̬		����Ƶ��: 10Hz
	--19	(0x0202)ʵʱ��������			����Ƶ��: 50Hz
	--19	(0x0203)������λ��			����Ƶ��: 10Hz
	--19	(0x0204)����������			����Ƶ��: ״̬�ı����
	//19	(0x0204)����������			����Ƶ��: 1Hz
	--19	(0x0205)���л���������״̬	����Ƶ��: 10Hz
	--19	(0x0206)�˺�״̬				����Ƶ��: �˺���������
	--19	(0x0207)ʵʱ�����Ϣ			����Ƶ��: �������
	--19	(0x0208)�ӵ�ʣ�෢����		����Ƶ��: 1Hz����(�����л����ˡ��ڱ������˺�ICRA���������ط���)
	++20	(0x0209)������RFID״̬		����Ƶ��: 1Hz���ڷ���
	
	--19	(0x0301)�����˼佻������		����Ƶ��: ���ͷ���������(����10Hz)
*/

/* Includes ------------------------------------------------------------------*/
#include "judge.h"

#include "uart5.h"
#include "crc.h"

#include "Task_Revolver.h"

/*----------------------------------------------------------------------------*/
/*-----------------------------����19����ϵͳ����------------------------------*/
/*----------------------------------------------------------------------------*/
#if (JUDGE_VERSION == JUDGE_VERSION_19)

/* Private typedef -----------------------------------------------------------*/
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

typedef enum {
	/* Std */
	LEN_FRAME_HEAD 					= 5,	// ֡ͷ����
	LEN_CMD_ID 						= 2,	// �����볤��
	LEN_FRAME_TAIL 					= 2,	// ֡βCRC16
	/* Ext */
	LEN_GAME_STATUS 				= 3,
	LEN_GAME_RESULT 				= 1,
	LEN_GAME_ROBOT_HP 				= 28,
	LEN_EVENT_DATA					= 4,
	LEN_SUPPLY_PROJECTILE_ACTION	= 4,
	LEN_SUPPLY_PROJECTILE_BOOKING	= 3,
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
	
/* ## Global variables ## ----------------------------------------------------*/
Judge_Info_t Judge = 
{
	.frame_length = 0,
	.cmd_id = 0,
	.err_cnt = 0,
	.data_valid = false,
	.hurt_data_update = false,
};	// ������Ե�ʱ����

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* API functions -------------------------------------------------------------*/
/* #������# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	����ϵͳ��ȡʵʱ����
 *	@note
 *			# ���ݹ���һ����(�ж�+����)�ĳ�����ķ�ʽ�Ͳ������HardFault
 *				���ھ���ԭ��û�����
 *			# ����DMA�Ļػ����ݴ�С(Buffer Size)����ʹ�ô������ӽϻ���
 */
bool JUDGE_ReadData(uint8_t *rxBuf)
{
	uint8_t  res = false;
	uint16_t frame_length;
	uint16_t cmd_id;	

	if( rxBuf == NULL )
	{
		return -1;
	}
	
	memcpy(&Judge.FrameHeader, rxBuf, LEN_FRAME_HEAD);
	
	/* ֡���ֽ��Ƿ�Ϊ0xA5 */
	if(rxBuf[SOF] == JUDGE_FRAME_HEADER) 
	{
		/* ֡ͷCRC8У�� */
		if(Verify_CRC8_Check_Sum(rxBuf, LEN_FRAME_HEAD) == true) 
		{
			/* ͳ��һ֡�������ݳ��ȣ�����CRC16У�� */
			frame_length = LEN_FRAME_HEAD + LEN_CMD_ID + Judge.FrameHeader.data_length + LEN_FRAME_TAIL;
			Judge.frame_length = frame_length;
			
			if(Verify_CRC16_Check_Sum(rxBuf, frame_length) == true)
			{
				res = true;
				
				cmd_id = (rxBuf[CMD_ID+1] << 8 | rxBuf[CMD_ID]);
				Judge.cmd_id = cmd_id;
				
				switch(cmd_id)
				{
					case ID_GAME_STATUS:
						memcpy(&Judge.GameStatus, (rxBuf+DATA_SEG), LEN_GAME_STATUS);
						break;
					case ID_GAME_RESULT:
						memcpy(&Judge.GameResult,  (rxBuf+DATA_SEG), LEN_GAME_RESULT);
						break;
					case ID_GAME_ROBOT_HP:
						memcpy(&Judge.GameRobotHP, (rxBuf+DATA_SEG), LEN_GAME_ROBOT_HP);
						break;
					case ID_EVENT_DATA:
						memcpy(&Judge.EventData, (rxBuf+DATA_SEG), LEN_EVENT_DATA);
						break;
					case ID_SUPPLY_PROJECTILE_ACTION:
						memcpy(&Judge.SupplyProjectileAction, (rxBuf+DATA_SEG), LEN_SUPPLY_PROJECTILE_ACTION);
						break;
					case ID_SUPPLY_PROJECTILE_BOOKING:
						//memcpy(&Judge.SupplyProjectileBooking, (rxBuf+DATA_SEG), LEN_SUPPLY_PROJECTILE_BOOKING);
						break;
					case ID_REFEREE_WARNING:
						memcpy(&Judge.RefereeWarning, (rxBuf+DATA_SEG), LEN_REFEREE_WARNING);
						break;
					case ID_GAME_ROBOT_STATUS:
						memcpy(&Judge.GameRobotStatus, (rxBuf+DATA_SEG), LEN_GAME_ROBOT_STATUS);
						break;
					case ID_POWER_HEAT_DATA:
						memcpy(&Judge.PowerHeatData, (rxBuf+DATA_SEG), LEN_POWER_HEAT_DATA);
						break;
					case ID_GAME_ROBOT_POS:
						memcpy(&Judge.GameRobotPos, (rxBuf+DATA_SEG), LEN_GAME_ROBOT_POS);
						break;
					case ID_BUFF:
						memcpy(&Judge.Buff, (rxBuf+DATA_SEG), LEN_BUFF);
						break;
					case ID_AERIAL_ROBOT_ENERGY:
						memcpy(&Judge.AerialRobotEnergy, (rxBuf+DATA_SEG), LEN_AERIAL_ROBOT_ENERGY);
						break;
					case ID_ROBOT_HURT:
						memcpy(&Judge.RobotHurt, (rxBuf+DATA_SEG), LEN_ROBOT_HURT);
						Judge.hurt_data_update = true;	// �˺����ݸ���
						break;
					case ID_SHOOT_DATA:
						memcpy(&Judge.ShootData, (rxBuf+DATA_SEG), LEN_SHOOT_DATA);
						JUDGE_ShootNumCount();	// ���㷢����
						break;
					case ID_BULLET_REMAINING:
						memcpy(&Judge.BulletRemaining, (rxBuf+DATA_SEG), LEN_BULLET_REMAINING);
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
			JUDGE_ReadData( &rxBuf[frame_length] );
		}
	}
	
	Judge.data_valid = res;
	if(Judge.data_valid != true)
	{
		Judge.err_cnt++;
		Judge.data_valid = false;
	}
	else
	{
		Judge.data_valid = true;
	}
		
	return res;
}

/* #��Ϣ��# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	�����жϴ�ʱ����ϵͳ���ݵ���ȷ��
 */
bool JUDGE_IfDataValid(void)
{
	return Judge.data_valid;
}

/**
 *	@brief	��������ʵʱ����
 */
float JUDGE_fGetChassisRealPower(void)
{
	return (Judge.PowerHeatData.chassis_power);
}

/**
 *	@brief	�������̻�������
 */
float JUDGE_fGetChassisPowerBuffer(void)
{
	return (Judge.PowerHeatData.chassis_power_buffer);
}

/**
 *	@brief	���������˵ȼ�
 */
uint8_t JUDGE_ucGetRobotLevel(void)
{
	return (Judge.GameRobotStatus.robot_level);
}

/**
 *	@brief	����������17mmǹ��ʵʱ����
 */
uint16_t JUDGE_usGetShooterRealHeat17(void)
{
	return (Judge.PowerHeatData.shooter_heat0);
}

/**
 *	@brief	����������17mmǹ����������
 */
uint16_t JUDGE_usGetShooterLimitHeat17(void)
{
	return (Judge.GameRobotStatus.shooter_heat0_cooling_limit);
}

/**
 *	@brief	����������17mmǹ��������ȴ����
 */
uint16_t JUDGE_usGetShooterHeatCoolingRate17(void)
{
	return (Judge.GameRobotStatus.shooter_heat0_cooling_rate);
}

/**
 *	@brief	����������17mm��������
 */
float JUDGE_fGetBulletSpeed17(void)
{
	return (Judge.ShootData.bullet_speed);
}

/**
 *	@brief	�����������ҷ���ɫ
 */
Color_t JUDGE_eGeyMyColor(void)
{
	uint8_t my_id;
	my_id = Judge.GameRobotStatus.robot_id;
	if(my_id > 10) {
		return BLUE;
	} else {
		return RED;
	}	
}

/**
 *	@brief	�������������˺���װ�װ�
 */
uint8_t JUDGE_eGetArmorHurt(void)
{
	static portTickType ulCurrent = 0;
	static portTickType ulDelay = 0;
	static bool bIfHurt = false;
	
	ulCurrent = xTaskGetTickCount();
	
	if(Judge.hurt_data_update == true) {
		Judge.hurt_data_update = false;	// ��֤���жϵ��˺����ݵĸ���
		if(Judge.RobotHurt.hurt_type == 0x0) {	// �˺����� - װ���˺�
			ulDelay = ulCurrent + TIME_STAMP_200MS;	//
			bIfHurt = true;
		}
	}
	
	if(ulCurrent > ulDelay) {
		bIfHurt = false;
	}
	
	if(bIfHurt == true) {
		return Judge.RobotHurt.armor_id;
	} else {
		return ARMOR_NONE;
	}
}

/* #Ӧ�ò�# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
  * @brief  ͳ�Ʒ�����
  * @param  void
  * @retval void
  * @attention  �ж��е��ã���������˫ǹ��(��׼)
  */
portTickType shoot_ping;//����������շ����ӳ�
float Shoot_Speed_Now = 0;
float Shoot_Speed_Last = 0;
void JUDGE_ShootNumCount(void)
{
	portTickType shoot_time;//������ʱ����

	Shoot_Speed_Now = JUDGE_fGetBulletSpeed17();
	if(Shoot_Speed_Last != Shoot_Speed_Now)//��Ϊ��float�ͣ�������������ȫ���,�����ٶȲ���ʱ˵��������һ�ŵ�
	{
		REVOLVER_AddShootCount();
		Shoot_Speed_Last = Shoot_Speed_Now;
	}
	shoot_time = xTaskGetTickCount();//��ȡ���跢��ʱ��ϵͳʱ��
	shoot_ping = shoot_time - REVOLVER_GetRealShootTime();//�����ӳ�
}

/* #������# ---------------------------------------------------------------------------------------------------------------------------------------*/

#endif	// Judge Version --19
/*----------------------------------------------------------------------------*/
/*-----------------------------����19����ϵͳ����------------------------------*/
/*----------------------------------------------------------------------------*/















/*----------------------------------------------------------------------------*/
/*-----------------------------����20����ϵͳ����------------------------------*/
/*----------------------------------------------------------------------------*/
#if (JUDGE_VERSION == JUDGE_VERSION_20)

/* Private typedef -----------------------------------------------------------*/
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
	ID_GAME_STATUS 					= 0x0001,	// ����״̬			
	ID_GAME_RESULT 					= 0x0002,	// ������� 		
	ID_GAME_ROBOT_HP 				= 0x0003,	// ������Ѫ������  	
	ID_DART_STATUS					= 0x0004,	// ���ڷ���״̬
	ID_ICRA_BUFF_DEBUFF_ZONE_STATUS = 0x0005,	// �˹�������ս���ӳ���ͷ���״̬
	
	ID_EVENT_DATA 					= 0x0101,	// �����¼�����		
	ID_SUPPLY_PROJECTILE_ACTION 	= 0x0102,	// ����վ������ʶ	
	//ID_SUPPLY_PROJECTILE_BOOKING 	= 0x0103,	// ���󲹸�վ�����ӵ�
	ID_REFEREE_WARNING 				= 0x0104,	// ���о�����Ϣ
	ID_DART_REMAINING_TIME			= 0x0105,	// ���ڷ���ڵ���ʱ
	
	ID_GAME_ROBOT_STATUS 			= 0x0201,	// ����������״̬
	ID_POWER_HEAT_DATA 				= 0x0202,	// ʵʱ������������
	ID_GAME_ROBOT_POS				= 0x0203,	// ������λ��
	ID_BUFF							= 0x0204,	// ����������
	ID_AERIAL_ROBOT_ENERGY			= 0x0205,	// ���л���������״̬
	ID_ROBOT_HURT					= 0x0206,	// �������˺�״̬
	ID_SHOOT_DATA					= 0x0207,	// ʵʱ�����Ϣ
	ID_BULLET_REMAINING				= 0x0208,	// �ӵ�ʣ�෢����
	ID_RFID_STATUS					= 0x0209,	// ������RFID״̬
	
	ID_COMMUNICATION				= 0x0301,	// �����˼佻������(���ͷ���������)
} Judge_Cmd_ID_t;

typedef enum {
	/* Std */
	LEN_FRAME_HEAD 	= 5,	// ֡ͷ����
	LEN_CMD_ID 		= 2,	// �����볤��
	LEN_FRAME_TAIL 	= 2,	// ֡βCRC16
	/* Ext */
	// 0x000x
	LEN_GAME_STATUS 				= 3,
	LEN_GAME_RESULT 				= 1,
	LEN_GAME_ROBOT_HP 				= 32,
	LEN_DART_STATUS					= 3,
	LEN_ICRA_BUFF_DEBUFF_ZONE_STATUS= 3,
	
	// 0x010x
	LEN_EVENT_DATA					= 4,
	LEN_SUPPLY_PROJECTILE_ACTION	= 4,
	//LEN_SUPPLY_PROJECTILE_BOOKING	= 3,
	LEN_REFEREE_WARNING				= 2,
	LEN_DART_REMAINING_TIME			= 1,
	
	// 0x020x
	LEN_GAME_ROBOT_STATUS			= 18,
	LEN_POWER_HEAT_DATA 			= 16,
	LEN_GAME_ROBOT_POS				= 16,
	LEN_BUFF		 				= 1,
	LEN_AERIAL_ROBOT_ENERGY 		= 3,
	LEN_ROBOT_HURT					= 1,
	LEN_SHOOT_DATA					= 6,
	LEN_BULLET_REMAINING	 		= 2,
	LEN_RFID_STATUS					= 4,
} Judge_Data_Length_t;

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#define JUDGE_FRAME_HEADER		(0xA5)

/* Private variables ---------------------------------------------------------*/
	
/* ## Global variables ## ----------------------------------------------------*/
Judge_Info_t Judge = 
{
	.frame_length = 0,
	.cmd_id = 0,
	.err_cnt = 0,
	.data_valid = false,
	.hurt_data_update = false,
};	// ������Ե�ʱ����

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* API functions -------------------------------------------------------------*/
/* #������# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	����ϵͳ��ȡʵʱ����
 *	@note
 *			# ���ݹ���һ����(�ж�+����)�ĳ�����ķ�ʽ�Ͳ������HardFault
 *				���ھ���ԭ��û�����
 *			# ����DMA�Ļػ����ݴ�С(Buffer Size)����ʹ�ô������ӽϻ���
 */
bool JUDGE_ReadData(uint8_t *rxBuf)
{
	uint8_t  res = false;
	uint16_t frame_length;
	uint16_t cmd_id;	

	if( rxBuf == NULL )
	{
		return -1;
	}
	
	memcpy(&Judge.FrameHeader, rxBuf, LEN_FRAME_HEAD);
	
	/* ֡���ֽ��Ƿ�Ϊ0xA5 */
	if(rxBuf[SOF] == JUDGE_FRAME_HEADER) 
	{
		/* ֡ͷCRC8У�� */
		if(Verify_CRC8_Check_Sum(rxBuf, LEN_FRAME_HEAD) == true) 
		{
			/* ͳ��һ֡�������ݳ��ȣ�����CRC16У�� */
			frame_length = LEN_FRAME_HEAD + LEN_CMD_ID + Judge.FrameHeader.data_length + LEN_FRAME_TAIL;
			Judge.frame_length = frame_length;
			
			if(Verify_CRC16_Check_Sum(rxBuf, frame_length) == true)
			{
				res = true;
				
				cmd_id = (rxBuf[CMD_ID+1] << 8 | rxBuf[CMD_ID]);
				Judge.cmd_id = cmd_id;
				
				switch(cmd_id)
				{
					case ID_GAME_STATUS: {
							memcpy(&Judge.GameStatus, (rxBuf+DATA_SEG), LEN_GAME_STATUS);
						}break;
					
					case ID_GAME_RESULT: {
							memcpy(&Judge.GameResult,  (rxBuf+DATA_SEG), LEN_GAME_RESULT);
						}break;
					
					case ID_GAME_ROBOT_HP: {
							memcpy(&Judge.GameRobotHP, (rxBuf+DATA_SEG), LEN_GAME_ROBOT_HP);
						}break;
					
					case ID_DART_STATUS: {
							memcpy(&Judge.DartStatus, (rxBuf+DATA_SEG), LEN_DART_STATUS);
							Judge.dart_data_update = true;	// �������ݸ���
						}break;
					
					case ID_EVENT_DATA: {
							memcpy(&Judge.EventData, (rxBuf+DATA_SEG), LEN_EVENT_DATA);
						}break;
					
					case ID_SUPPLY_PROJECTILE_ACTION: {
							memcpy(&Judge.SupplyProjectileAction, (rxBuf+DATA_SEG), LEN_SUPPLY_PROJECTILE_ACTION);
							Judge.supply_data_update = true;	// ����վ���ݸ���
						}break;
					
					case ID_REFEREE_WARNING: {
							memcpy(&Judge.RefereeWarning, (rxBuf+DATA_SEG), LEN_REFEREE_WARNING);
						}break;
					
					case ID_DART_REMAINING_TIME: {
							memcpy(&Judge.DartRemainingTime, (rxBuf+DATA_SEG), LEN_DART_REMAINING_TIME);
						}break;
						
					case ID_GAME_ROBOT_STATUS: {
							memcpy(&Judge.GameRobotStatus, (rxBuf+DATA_SEG), LEN_GAME_ROBOT_STATUS);
						}break;
					
					case ID_POWER_HEAT_DATA: {
							memcpy(&Judge.PowerHeatData, (rxBuf+DATA_SEG), LEN_POWER_HEAT_DATA);
						}break;
					
					case ID_GAME_ROBOT_POS: {
							memcpy(&Judge.GameRobotPos, (rxBuf+DATA_SEG), LEN_GAME_ROBOT_POS);
						}break;
					
					case ID_BUFF: {
							memcpy(&Judge.Buff, (rxBuf+DATA_SEG), LEN_BUFF);
						}break;
					
					case ID_AERIAL_ROBOT_ENERGY: {
							memcpy(&Judge.AerialRobotEnergy, (rxBuf+DATA_SEG), LEN_AERIAL_ROBOT_ENERGY);
						}break;
					
					case ID_ROBOT_HURT: {
							memcpy(&Judge.RobotHurt, (rxBuf+DATA_SEG), LEN_ROBOT_HURT);
							Judge.hurt_data_update = true;	// �˺����ݸ���
						}break;
					
					case ID_SHOOT_DATA: {
							memcpy(&Judge.ShootData, (rxBuf+DATA_SEG), LEN_SHOOT_DATA);
							JUDGE_ShootNumCount();	// ���㷢����
						}break;
					
					case ID_BULLET_REMAINING: {
							memcpy(&Judge.BulletRemaining, (rxBuf+DATA_SEG), LEN_BULLET_REMAINING);
						}break;
					
					case ID_RFID_STATUS: {
							memcpy(&Judge.RfidStatus, (rxBuf+DATA_SEG), LEN_RFID_STATUS);
						}break;
						
					case ID_COMMUNICATION: {
						}break;
				}
			}
		}

		/* ֡βCRC16��һ�ֽ��Ƿ�Ϊ0xA5 */
		if(rxBuf[ frame_length ] == JUDGE_FRAME_HEADER)
		{
			/* ���һ�����ݰ������˶�֡���ݾ��ٴζ�ȡ */
			JUDGE_ReadData( &rxBuf[frame_length] );
		}
	}
	
	Judge.data_valid = res;
	if(Judge.data_valid != true) {
		Judge.err_cnt++;
		Judge.data_valid = false;
	}
	else {
		Judge.data_valid = true;
	}
		
	return res;
}

/* #��Ϣ��# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	�����жϴ�ʱ����ϵͳ���ݵ���ȷ��
 */
bool JUDGE_IfDataValid(void)
{
	return Judge.data_valid;
}

/**
 *	@brief	��������ʵʱ����
 */
float JUDGE_fGetChassisRealPower(void)
{
	return (Judge.PowerHeatData.chassis_power);
}

/**
 *	@brief	�������̻�������
 */
float JUDGE_fGetChassisPowerBuffer(void)
{
	return (Judge.PowerHeatData.chassis_power_buffer);
}

/**
 *	@brief	���������˵ȼ�
 */
uint8_t JUDGE_ucGetRobotLevel(void)
{
	return (Judge.GameRobotStatus.robot_level);
}

/**
 *	@brief	����������17mmǹ��ʵʱ����
 */
uint16_t JUDGE_usGetShooterRealHeat17(void)
{
	return (Judge.PowerHeatData.shooter_heat0);
}

/**
 *	@brief	����������17mmǹ����������
 */
uint16_t JUDGE_usGetShooterLimitHeat17(void)
{
	return (Judge.GameRobotStatus.shooter_heat0_cooling_limit);
}

/**
 *	@brief	����������17mmǹ��������ȴ����
 */
uint16_t JUDGE_usGetShooterHeatCoolingRate17(void)
{
	return (Judge.GameRobotStatus.shooter_heat0_cooling_rate);
}

/**
 *	@brief	����������17mm��������
 */
float JUDGE_fGetBulletSpeed17(void)
{
	return (Judge.ShootData.bullet_speed);
}

/**
 *	@brief	����������17mm������������
 */
uint8_t JUDGE_ucGetBulletLimitSpeed17(void)
{
	return (Judge.GameRobotStatus.shooter_heat0_speed_limit);
}

/**
 *	@brief	���������������̹���
 */
uint8_t JUDGE_ucGetMaxPower(void)
{
	return (Judge.GameRobotStatus.max_chassis_power);
}

/**
 *	@brief	�����������ҷ���ɫ
 */
Color_t JUDGE_eGeyMyColor(void)
{
	uint8_t my_id;
	my_id = Judge.GameRobotStatus.robot_id;
	if(my_id > 10) {
		return BLUE;
	} else {
		return RED;
	}	
}

/**
 *	@brief	�������������˺���װ�װ�
 */
uint8_t JUDGE_eGetArmorHurt(void)
{
	static portTickType ulCurrent = 0;
	static portTickType ulDelay = 0;
	static bool bIfHurt = false;
	
	ulCurrent = xTaskGetTickCount();
	
	if(Judge.hurt_data_update == true) {
		Judge.hurt_data_update = false;	// ��֤���жϵ��˺����ݵĸ���
		if(Judge.RobotHurt.hurt_type == 0x0) {	// �˺����� - װ���˺�
			ulDelay = ulCurrent + TIME_STAMP_200MS;	//
			bIfHurt = true;
		}
	}
	
	if(ulCurrent > ulDelay) {
		bIfHurt = false;
	}
	
	if(bIfHurt == true) {
		return Judge.RobotHurt.armor_id;
	} else {
		return ARMOR_NONE;
	}
}

/**
 *	@brief	�����Ƿ��з��ڷ���
 */
Color_t JUDGE_eGetDartBelong(void)
{
	Color_t eDartBelong = COLOR_NONE;
	
	if(Judge.dart_data_update == true) {
		Judge.dart_data_update = false;
		if(Judge.DartStatus.dart_belong == 1)
			eDartBelong = RED;
		else if(Judge.DartStatus.dart_belong == 2)
			eDartBelong = BLUE;
	}
	
	return eDartBelong;
}

/**
 *	@brief	��������վID
 */
Supply_Id_t JUDGE_eGetSupplyId(void)
{
	Supply_Id_t eSupplyId = SUPPLY_ID_NONE;
	
	if(Judge.supply_data_update == true) {
		Judge.supply_data_update = false;
		if(Judge.SupplyProjectileAction.supply_projectile_id == 1)
			eSupplyId = SUPPLY_ID_1;
		else if(Judge.SupplyProjectileAction.supply_projectile_id == 2)
			eSupplyId = SUPPLY_ID_2;
	}
	
	return eSupplyId;
}

/**
 *	@brief	����������ǹ��λ��
 */
float JUDGE_fGetShooterYaw(void)
{
	return (Judge.GameRobotPos.yaw);
}

/* #Ӧ�ò�# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
  * @brief  ͳ�Ʒ�����
  * @param  void
  * @retval void
  * @attention  �ж��е��ã���������˫ǹ��(��׼)
  */
portTickType shoot_ping;//����������շ����ӳ�
float Shoot_Speed_Now = 0;
float Shoot_Speed_Last = 0;
void JUDGE_ShootNumCount(void)
{
	portTickType shoot_time;//������ʱ����

	Shoot_Speed_Now = JUDGE_fGetBulletSpeed17();
	if(Shoot_Speed_Last != Shoot_Speed_Now)//��Ϊ��float�ͣ�������������ȫ���,�����ٶȲ���ʱ˵��������һ�ŵ�
	{
		REVOLVER_AddShootCount();
		Shoot_Speed_Last = Shoot_Speed_Now;
	}
	shoot_time = xTaskGetTickCount();//��ȡ���跢��ʱ��ϵͳʱ��
	shoot_ping = shoot_time - REVOLVER_GetRealShootTime();//�����ӳ�
}

/* #������# ---------------------------------------------------------------------------------------------------------------------------------------*/

#endif	// Judge Version --20
/*----------------------------------------------------------------------------*/
/*-----------------------------����20����ϵͳ����------------------------------*/
/*----------------------------------------------------------------------------*/
