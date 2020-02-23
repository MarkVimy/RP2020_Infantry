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
// (0x0002)������� 				����Ƶ��: ������������
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
						memcpy(&Judge.SupplyProjectileBooking, (rxBuf+DATA_SEG), LEN_SUPPLY_PROJECTILE_BOOKING);
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
