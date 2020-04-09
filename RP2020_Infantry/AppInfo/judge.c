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
/* ����ϵͳ������Ϣ */
Judge_Info_t Judge = 
{
	.frame_length = 0,
	.cmd_id = 0,
	.err_cnt = 0,
	.data_valid = false,
	.hurt_data_update = false,
};	// ������Ե�ʱ����

/* ����ϵͳ�ͻ�����Ϣ */
Judge_Client_Data_t Judge_Client =
{
	.FrameHeader.sof = 0xA5
};

/* ����ϵͳ�����˽�����Ϣ */
Judge_Interact_Data_t Judge_Interact = 
{
	.FrameHeader.sof = 0xA5
};

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
							//JUDGE_ReadFromCom();
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

/* 
	������ ID��					�ͻ��� ID��
	1��Ӣ��(��)��				0x0101 ΪӢ�۲����ֿͻ���( ��) ��
	2������(��)��				0x0102 �����̲����ֿͻ��� ((�� )��
	3/4/5������(��)��			0x0103/0x0104/0x0105�����������ֿͻ���(��)��
	6������(��)��				0x0106�����в����ֿͻ���((��)�� 
	7���ڱ�(��)��

	101��Ӣ��(��)��				0x0165��Ӣ�۲����ֿͻ���(��)��
	102������(��)��				0x0166�����̲����ֿͻ���(��)��
	103/104/105������(��)��		0x0167/0x0168/0x0169�����������ֿͻ���(��)��
	106������(��)��				0x016A�����в����ֿͻ���(��)�� 
	107���ڱ�(��)��
*/
void JUDGE_DetermineClientId(void)
{
	Color_t color;
	
	color = JUDGE_eGeyMyColor();
	if(color == RED) {
		Judge.self_client_id = 0x0100 + Judge.GameRobotStatus.robot_id;
	} 
	else if(color == BLUE) {
		Judge.self_client_id = 0x0164 + Judge.GameRobotStatus.robot_id;
	}
}

/**
 *	@brief	�ϴ��Զ����������ͻ��˽���
 */
#define CLIENT_FRAME_LEN	(15+105)
static uint8_t txClientBuf[CLIENT_FRAME_LEN];
void JUDGE_SendToClient(void)
{
	uint8_t frame_length;
	
	JUDGE_DetermineClientId();
	
	// ֡ͷ�������
	Judge_Client.FrameHeader.sof = 0xA5;
	Judge_Client.FrameHeader.data_length = sizeof(Judge_Client.DataFrameHeader) + sizeof(Judge_Client.ClientData);
	Judge_Client.FrameHeader.seq = 0;

	memcpy(txClientBuf, &Judge_Client.FrameHeader, sizeof(Judge_Client.FrameHeader));
	// д��֡ͷCRC8У����
	Append_CRC8_Check_Sum(txClientBuf, sizeof(Judge_Client.FrameHeader));
	
	// ������
	Judge_Client.CmdId = ID_COMMUNICATION;
	
	// ����7��ͼ��
	Judge_Client.DataFrameHeader.data_cmd_id = 0x0104;
	
	// ������ID
	Judge_Client.DataFrameHeader.send_ID = Judge.GameRobotStatus.robot_id;
	// �����߿ͻ���ID
	Judge_Client.DataFrameHeader.receiver_ID = Judge.self_client_id;
	
	/*----���ݶ�����----*/
	//...
	
	// �������
	memcpy(	
			txClientBuf + CMD_ID, 
			(uint8_t*)&Judge_Client.CmdId, 
			(sizeof(Judge_Client.CmdId)+ sizeof(Judge_Client.DataFrameHeader)+ sizeof(Judge_Client.ClientData))
		  );			
	
	frame_length = sizeof(Judge_Client);
	//д�����ݶ�CRC16У����		
	Append_CRC16_Check_Sum(txClientBuf, frame_length);	

	for(uint8_t i = 0; i < frame_length; i++) {
		UART5_SendChar(txClientBuf[i]);
	}
	
	/* �������ݰ����� */
	memset(txClientBuf, 0, CLIENT_FRAME_LEN);	
}

/**
 *	@brief	�ϴ��Զ�������������
 */
#define INTERACT_FRAME_LEN	(15+INTERACT_DATA_LEN)
static uint8_t txInteractBuf[INTERACT_FRAME_LEN];
void JUDGE_SendToTeammate(uint8_t teammate_id)
{
	uint8_t frame_length;

	// ֡ͷ�������
	Judge_Interact.FrameHeader.sof = 0xA5;
	Judge_Interact.FrameHeader.data_length = sizeof(Judge_Interact.DataFrameHeader) + sizeof(Judge_Interact.InteractData);
	Judge_Interact.FrameHeader.seq = 0;	
	
	memcpy(txInteractBuf, &Judge_Interact.FrameHeader, sizeof(Judge_Interact.FrameHeader));
	// д��֡ͷCRC8У����
	Append_CRC8_Check_Sum(txInteractBuf, sizeof(Judge_Interact.FrameHeader));
	
	// ������
	Judge_Interact.CmdId = ID_COMMUNICATION;
	
	// �Զ���������
	Judge_Interact.DataFrameHeader.data_cmd_id = 0x0200;	// ��0x0200-0x02ff֮��ѡ��
	
	// ������ID
	Judge_Interact.DataFrameHeader.send_ID = Judge.GameRobotStatus.robot_id;
	// ������ID
	Judge_Interact.DataFrameHeader.receiver_ID = teammate_id;
	
	/*----���ݶ�����----*/
	//...
	
	// �������
	memcpy(	
			txClientBuf + CMD_ID, 
			(uint8_t*)&Judge_Interact.CmdId, 
			(sizeof(Judge_Interact.CmdId)+ sizeof(Judge_Interact.DataFrameHeader)+ sizeof(Judge_Interact.InteractData))
		  );			
	
	frame_length = sizeof(Judge_Interact);
	// д�����ݶ�CRC16У����		
	Append_CRC16_Check_Sum(txInteractBuf, frame_length);	

	for(uint8_t i = 0; i < frame_length; i++) {
		UART5_SendChar(txInteractBuf[i]);
	}

	/* �������ݰ����� */
	memset(txInteractBuf, 0, INTERACT_FRAME_LEN);	
}

/**
 *	@brief	���ջ����˼��ͨ������
 */
void JUDGE_ReadFromCom()
{
	
}

/* #������# ---------------------------------------------------------------------------------------------------------------------------------------*/
/* #�ͻ���# ---------------------------------------------------------------------------------------------------------------------------------------*/
///**
//* @brief ����һ������
//* @param  graphic *ͼ����Ϣ
//					name	ͼ����
//					layer ͼ����
//					color ͼ����ɫ
//					width �߿�
//					start_x ��ʼ��x����
//					start_y ��ʼ��y����
//					diagonal_x �Խǵ�x����
//					diagonal_y �Խǵ�y����
//* @return NONE
//*/
//void Draw_Rectangle(graphic_data_struct_t* graphic,
//										const uint8_t *name,
//										uint8_t layer,
//										Graphic_Color color,
//										uint16_t width,
//										uint16_t start_x,
//										uint16_t start_y,
//										uint16_t diagonal_x,
//										uint16_t diagonal_y)
//{
//	Add_Graphic(graphic,name,layer,RECTANGLE,color,0,0,width,start_x,start_y,0,diagonal_x,diagonal_y);
//}

///**
//* @brief ����һ��ֱ��
//* @param  graphic *ͼ����Ϣ
//					name	ͼ����
//					layer ͼ����
//					color ͼ����ɫ
//					width �߿�
//					start_x ��ʼ��x����
//					start_y ��ʼ��y����
//					end_x �յ�x����
//					end_y �յ�y����
//* @return NONE
//*/
//void Draw_Line(graphic_data_struct_t* graphic,
//							const uint8_t *name,
//							uint8_t layer,
//							Graphic_Color color,
//							uint16_t width,
//							uint16_t start_x,
//							uint16_t start_y,
//							uint16_t end_x,
//							uint16_t end_y)
//{
//	Add_Graphic(graphic,name,layer,LINE,color,0,0,width,start_x,start_y,0,end_x,end_y);
//}
// 
///**
//* @brief ����һ��Բ
//* @param  graphic *ͼ����Ϣ
//					name	ͼ����
//					layer ͼ����
//					color ͼ����ɫ
//					width �߿�
//					center_x Բ��x����
//					center_y Բ��y����
//					radius �뾶
//* @return NONE
//*/
//void Draw_Circle(graphic_data_struct_t* graphic,
//							const uint8_t *name,
//							uint8_t layer,
//							Graphic_Color color,
//							uint16_t width,
//							uint16_t center_x,
//							uint16_t center_y,
//							uint16_t radius)
//{
//	Add_Graphic(graphic,name,layer,CIRCLE,color,0,0,width,center_x,center_y,radius,0,0);
//}

///**
//* @brief ����һ����Բ
//* @param  graphic *ͼ����Ϣ
//					name	ͼ����
//					layer ͼ����
//					color ͼ����ɫ
//					width �߿�
//					center_x Բ��x����
//					center_y Բ��y����
//					axis_x x����
//					axis_y y����
//* @return NONE
//*/
//void Draw_Oval(graphic_data_struct_t* graphic,
//							const uint8_t *name,
//							uint8_t layer,
//							Graphic_Color color,
//							uint16_t width,
//							uint16_t center_x,
//							uint16_t center_y,
//							uint16_t axis_x,
//							uint16_t axis_y)
//{	
//	Add_Graphic(graphic,name,layer,OVAL,color,0,0,width,center_x,center_y,0,axis_x,axis_y);
//}


///**
//* @brief ����һ��Բ��
//* @param  graphic *ͼ����Ϣ
//					name	ͼ����
//					layer ͼ����
//					color ͼ����ɫ
//					start_angle ��ʼ�Ƕ�
//					end_angle 	��ֹ�Ƕ�
//					width �߿�
//					center_x Բ��x����
//					center_y Բ��y����
//					axis_x x����
//					axis_y y����
//* @return NONE
//*/
//void Draw_ARC(graphic_data_struct_t* graphic,
//							const uint8_t *name,
//							uint8_t layer,
//							Graphic_Color color,
//							uint16_t start_angle,
//							uint16_t end_angle,
//							uint16_t width,
//							uint16_t center_x,
//							uint16_t center_y,
//							uint16_t axis_x,
//							uint16_t axis_y)
//{
//	Add_Graphic(graphic,name,layer,ARC,color,start_angle,end_angle,width,center_x,center_y,0,axis_x,axis_y);
//}

///**
//* @brief ���Ƹ�����
//* @param  graphic *ͼ����Ϣ
//					name	ͼ����
//					layer ͼ����
//					color ͼ����ɫ
//					font_size	�����С
//					accuracy С����λ��
//					width �߿�
//					start_x	��ʼx����
//					start_y ��ʼy����
//					number ��ʾ������
//* @return NONE
//*/
//void Draw_Float(graphic_data_struct_t* graphic,
//								const uint8_t *name,
//								uint8_t layer,
//								Graphic_Color color,
//								uint16_t font_size,
//								uint16_t accuracy,
//								uint16_t width,
//								uint16_t start_x,
//								uint16_t start_y,
//								float number)
//{
//	float num_tmp_f = number;
//	uint32_t num_tmp_u=0x00;
//	memcpy(&num_tmp_u,&num_tmp_f,4);
//	Add_Graphic(graphic,name,layer,FLOAT,color,font_size,accuracy,width,start_x,start_y,(num_tmp_u>>22)&0x3ff,(num_tmp_u>>11)&0x7ff,num_tmp_u&0x7ff);

//}

///**
//* @brief ��������
//* @param  graphic *ͼ����Ϣ
//					name	ͼ����
//					layer ͼ����
//					color ͼ����ɫ
//					font_size	�����С
//					width �߿�
//					start_x	��ʼx����
//					start_y ��ʼy����
//					number ��ʾ������
//* @return NONE
//*/
//void Draw_Int(graphic_data_struct_t* graphic,
//							const uint8_t *name,
//							uint8_t layer,
//							Graphic_Color color,
//							uint16_t font_size,
//							uint16_t width,
//							uint16_t start_x,
//							uint16_t start_y,
//							int32_t number)
//{
//	int32_t num_tmp_i = number;
//	uint32_t num_tmp_u=0x00;
//	memcpy(&num_tmp_u,&num_tmp_i,4);
//	Add_Graphic(graphic,name,layer,INT,color,font_size,0,width,start_x,start_y,(num_tmp_i>>22)&0x3ff,(num_tmp_i>>11)&0x7ff,num_tmp_i&0x7ff);
//}



///**
//* @brief �����ַ�
//* @param  graphic *�ַ�ͼ����Ϣ
//					name	ͼ����
//					layer ͼ����
//					color ͼ����ɫ
//					font_size	�����С
//					length	�ַ�����
//					width �߿�
//					start_x	��ʼx����
//					start_y ��ʼy����
//					character �ַ�����Ϣ
//* @return NONE
//*/
//void Draw_Char(ext_client_custom_character_t* graphic,
//							const uint8_t *name,
//							uint8_t layer,
//							Graphic_Color color,
//							uint16_t font_size,
//							uint16_t length,
//							uint16_t width,
//							uint16_t start_x,
//							uint16_t start_y,
//							const uint8_t *character)
//{
//	Add_Graphic(&(graphic->grapic_data_struct),name,layer,CHAR,color,font_size,length,width,start_x,start_y,0,0,0);
//	memcpy(graphic->data,character,length);
//}



///**
//* @brief ����һ��ͼ�㣬���ͼ����Ϣ
//* @param  graphic *ͼ����Ϣ
//					name	ͼ����
//					layer ͼ����
//					type  ͼ������
//					color ͼ����ɫ
//					�ȵ�
//* @return NONE
//*/
//void Add_Graphic(graphic_data_struct_t* graphic,
//									const uint8_t* name,
//									uint8_t layer,
//									uint8_t type,
//									uint8_t color,
//									uint16_t start_angle,
//									uint16_t end_angle,
//									uint16_t width,
//									uint16_t start_x,
//									uint16_t start_y,
//									uint16_t radius,
//									uint16_t end_x,
//									uint16_t end_y)
//{
//	graphic->color = color;
//	for(uint8_t i=0;i<3;i++)
//		graphic->graphic_name[i] = name[i];
//	graphic->layer = layer;
//	graphic->graphic_tpye = type;
//	graphic->operate_tpye = ADD;
//	
//	graphic->start_angle = start_angle;
//	graphic->end_angle = end_angle;
//	graphic->width = width;
//	graphic->start_x = start_x;
//	graphic->start_y = start_y;
//	graphic->radius = radius;
//	graphic->end_x = end_x;
//	graphic->end_y = end_y;
//}

///**
//* @brief  �޸�һ��ͼ�㣬���ͼ����Ϣ
//* @param  graphic *ͼ����Ϣ
//					name	ͼ����
//					layer ͼ����
//					type  ͼ������
//					color ͼ����ɫ
//					�ȵ�
//* @return NONE
//*/
//void Modify_Graphic(graphic_data_struct_t* graphic,
//								  const uint8_t* name,
//									uint8_t layer,
//									uint8_t type,
//									uint8_t color,
//									uint16_t start_angle,
//									uint16_t end_angle,
//									uint16_t width,
//									uint16_t start_x,
//									uint16_t start_y,
//									uint16_t radius,
//									uint16_t end_x,
//									uint16_t end_y)
//{
//	graphic->color = color;
//	for(uint8_t i=0;i<3;i++)
//		graphic->graphic_name[i] = name[i];	
//	graphic->layer = layer;
//	graphic->graphic_tpye = type;
//	graphic->operate_tpye = MODIFY;
//	
//	graphic->start_angle = start_angle;
//	graphic->end_angle = end_angle;
//	graphic->width = width;
//	graphic->start_x = start_x;
//	graphic->start_y = start_y;
//	graphic->radius = radius;
//	graphic->end_x = end_x;
//	graphic->end_y = end_y;
//	
//	/*��Ϊ��֪���ͻ����޸�δ���Ӵ�����ͼ�λ᲻��bug������ڴ������жϣ��Ƿ���Ҫ����ͼ��*/
//}


///**
//* @brief  ɾ��һ��ͼ�Σ����ͼ����Ϣ
//* @param  graphic *ͼ����Ϣ
//* @return NONE
//*/
//void Delete_Graphic(graphic_data_struct_t* graphic)
//{
//	graphic->operate_tpye = DELETE;
//	/*ֱ��ɾ������*/
//}


///**
//* @brief ɾ��Ŀ��ͻ���ĳһͼ���ȫ������
//* @param Interact_Target Ŀ��ͻ���ID
//					layer ɾ����ͼ����
//* @return NONE
//*/
//void Delete_Graphic_Layer(Interact_Target target,uint8_t layer)
//{
//	uint8_t data[17]={0};	/*6+2+9*/  	/*��ʱ����*/
//	uint16_t data_length = LEN_INTERACT_delete_graphic;	 /*data�ֶεĳ���*/
//	uint8_t CRC8=0x00;
//	uint16_t CRC16=0x00;
//	/*����У�����*/
//	
//	data[0] = 0xA5;
//	data[1] = data_length>>8;
//	data[2] = data_length;
//	data[3] = 1;
//	CRC8 = Get_CRC8_Check_Sum(data,4,0xff);
//	data[4] = CRC8;
//	/*SOF*/
//	
//	data[5] = ID_interactive_header_data>>8;
//	data[6] = (uint8_t)ID_interactive_header_data;
//	/*��佻����cmd-ID*/
//	
//	interactive_data_header.data_cmd_id = INTERACT_ID_delete_graphic;
//	interactive_data_header.receiver_ID = target;
//	interactive_data_header.send_ID = self_id;
//	memcpy(data+7,&interactive_data_header,6);
//	/*���data��ͷ*/
//	
//	graphic_delete.operate_type = 1;
//	graphic_delete.layer = layer;
//	memcpy(data+13,&graphic_delete,2);
//	/*���data����*/
//	
//	
//	CRC16 = Get_CRC16_Check_Sum(data,15,0xffff);
//	data[15] = CRC16>>8;
//	data[16] = CRC16;
//	/*��ȡCRC16ֵ*/
//	
//	memcpy(&send_interactive_buff,data,17);
//	Send_Interact_Data(&send_interactive_buff,17);
//	/*ͨ�����ڷ���*/
//}



///**
//* @brief ɾ��Ŀ��ͻ�������ͼ���API����
//* @param Interact_Target Ŀ��ͻ���ID
//* @return NONE
//*/
//void Delete_All_Graphic_Layer(Interact_Target target)
//{
//	uint8_t data[17]={0};	/*6+2+9*/  	/*��ʱ����*/
//	uint16_t data_length = LEN_INTERACT_delete_graphic;	 /*data�ֶεĳ���*/
//	uint8_t CRC8=0x00;
//	uint16_t CRC16=0x00;
//	/*����У�����*/
//	
//	data[0] = 0xA5;
//	data[1] = data_length>>8;
//	data[2] = data_length;
//	data[3] = 1;
//	CRC8 = Get_CRC8_Check_Sum(data,4,0xff);
//	data[4] = CRC8;
//	/*SOF*/
//	
//	data[5] = ID_interactive_header_data>>8;
//	data[6] = (uint8_t)ID_interactive_header_data;
//	/*��佻����cmd-ID*/
//	
//	interactive_data_header.data_cmd_id = INTERACT_ID_delete_graphic;
//	interactive_data_header.receiver_ID = target;
//	interactive_data_header.send_ID = self_id;
//	memcpy(data+7,&interactive_data_header,6);
//	/*���data��ͷ*/
//	
//	graphic_delete.operate_type = 2;
//	graphic_delete.layer = 9;
//	memcpy(data+13,&graphic_delete,2);
//	/*���data����*/
//	
//	
//	CRC16 = Get_CRC16_Check_Sum(data,15,0xffff);
//	data[15] = CRC16>>8;
//	data[16] = CRC16;
//	/*��ȡCRC16ֵ*/
//	
//	memcpy(&send_interactive_buff,data,17);
//	Send_Interact_Data(&send_interactive_buff,17);
//	/*ͨ�����ڷ���*/
//}



///**
//* @brief ����ϵͳ�������ݵ����շ��ͺ���
//* @param uint8_t * ���͵�����
//* @return ���ͽ��
//*/
//bool Send_Interact_Data(robot_interactive_data_t *str,uint16_t length)
//{
//	if(length > 113)
//		return false;
//	/*��������˳�*/
//	for(uint16_t i=0;i<length;i++)
//	{
//		UART5_SendChar(str->data[i]);   
//	}
//	return true;
//}




///**
//* @brief �޸����еĸ�����ͼ��
//* @param  graphic *ͼ����Ϣ
//					number ��ʾ������
//* @return NONE
//*/
//void Modify_Float(graphic_data_struct_t* graphic,float number)
//{
//	float num_tmp_f = number;
//	uint32_t num_tmp_u=0x00;
//	memcpy(&num_tmp_u,&num_tmp_f,4);
//	Modify_Graphic(graphic,graphic->graphic_name,graphic->layer,FLOAT,graphic->color,graphic->start_angle,graphic->end_angle,graphic->width,graphic->start_x,graphic->start_y,(num_tmp_u>>22)&0x3ff,(num_tmp_u>>11)&0x7ff,num_tmp_u&0x7ff);

//}

///**
//* @brief �޸����е�����ͼ��
//* @param  graphic *ͼ����Ϣ
//					number ��ʾ������
//* @return NONE
//*/
//void Modify_Int(graphic_data_struct_t* graphic,int32_t number)
//{
//	int32_t num_tmp_i = number;
//	uint32_t num_tmp_u=0x00;
//	memcpy(&num_tmp_u,&num_tmp_i,4);
//	Modify_Graphic(graphic,graphic->graphic_name,graphic->layer,INT,graphic->color,graphic->start_angle,graphic->end_angle,graphic->width,graphic->start_x,graphic->start_y,(num_tmp_u>>22)&0x3ff,(num_tmp_u>>11)&0x7ff,num_tmp_u&0x7ff);
//}



///**
//* @brief �޸����е��ַ�ͼ��
//* @param  graphic *�ַ�ͼ����Ϣ
//					length	�ַ�����
//					character �ַ�����Ϣ
//* @return NONE
//*/
//void Modify_Char(ext_client_custom_character_t* graphic,const uint8_t *character,uint16_t length)
//{
//	Modify_Graphic(&(graphic->grapic_data_struct),graphic->grapic_data_struct.graphic_name,\
//			graphic->grapic_data_struct.layer,CHAR,graphic->grapic_data_struct.color,graphic->grapic_data_struct.start_angle,\
//				graphic->grapic_data_struct.end_angle,graphic->grapic_data_struct.width,graphic->grapic_data_struct.start_x,\
//					graphic->grapic_data_struct.start_y,0,0,0);
//	memcpy(graphic->data,character,length);
//}

///* #����ͨ��# ---------------------------------------------------------------------------------------------------------------------------------------*/


#endif	// Judge Version --20
///*----------------------------------------------------------------------------*/
///*-----------------------------����20����ϵͳ����------------------------------*/
///*----------------------------------------------------------------------------*/
