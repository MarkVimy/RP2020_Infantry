/**
 * @file        judge.c
 * @author      MaWeiming
 * @Version     V1.0
 * @date        19-October-2019
 * @brief       This file includes the JUDGE Protocols(裁判系统协议) external functions 
 * 				(based on ST Peripherals Libaray Keil.STM32F4xx_DFP.2.13.0)
 * @Version		
 */

/**
 *	# 裁判系统使用USART5作为通讯接口
 *	通信协议格式:
 *				帧头		命令段			数据段		帧尾
 *	域		frame_header 	cmd_id 			data  		frame_tail(CRC16,整包校验)
 *	字节		5			  2		  		n			2
 *	
 *	帧头格式:
 *		  起始字节(0xA5)		数据帧长度		包序号		帧头CRC8校验
 *	域			SOF 	 	data_length    	seq 	 	CRC8
 *	字节		0				2			1			1
 */

/*------Cmd ID-------*/
/**
 *	-- =>	上一年已存在
 *	++ =>	今年新增
 *	xx =>	今年取消
 *	// =>	今年修改
 */
/*
	--19	(0x0001)比赛状态				发送频率: 1Hz
	--19	(0x0002)比赛结果 			发送频率: 比赛结束后发送
	--19	(0x0003)机器人血量数据  		发送频率: 1Hz
	++20	(0x0004)飞镖发射状态			发送频率: 飞镖发射时发送
	++20	(0x0005)人工智能挑战赛加成与惩罚区状态	发送频率: 1Hz
	
	--19	(0x0101)场地事件数据			发送频率: 事件改变后发送
	--19	(0x0102)补给站动作标识		发送频率: 动作改变后发送
	--19	(0x0103)请求补给站补弹子弹	发送频率: 上限10Hz(RM对抗赛尚未开放)
	xx20	(0x0103)已取消
	--19	(0x0104)裁判警告信息			发送频率: 警告发生后发送
	++20	(0x0105)飞镖发射口倒计时		发送频率: 1Hz
	
	--19	(0x0201)比赛机器人状态		发送频率: 10Hz
	--19	(0x0202)实时功率热量			发送频率: 50Hz
	--19	(0x0203)机器人位置			发送频率: 10Hz
	--19	(0x0204)机器人增益			发送频率: 状态改变后发送
	//19	(0x0204)机器人增益			发送频率: 1Hz
	--19	(0x0205)空中机器人能量状态	发送频率: 10Hz
	--19	(0x0206)伤害状态				发送频率: 伤害发生后发送
	--19	(0x0207)实时射击信息			发送频率: 射击后发送
	--19	(0x0208)子弹剩余发射数		发送频率: 1Hz发送(仅空中机器人、哨兵机器人和ICRA机器人主控发送)
	++20	(0x0209)机器人RFID状态		发送频率: 1Hz周期发送
	
	--19	(0x0301)机器人间交互数据		发送频率: 发送方触发发送(上线10Hz)
*/

/* Includes ------------------------------------------------------------------*/
#include "judge.h"

#include "uart5.h"
#include "crc.h"

#include "Task_Revolver.h"

/*----------------------------------------------------------------------------*/
/*-----------------------------↓↓19裁判系统↓↓------------------------------*/
/*----------------------------------------------------------------------------*/
#if (JUDGE_VERSION == JUDGE_VERSION_19)

/* Private typedef -----------------------------------------------------------*/
/* 帧字节偏移 */
typedef enum {
	FRAME_HEADER	= 0,
	CMD_ID			= 5,
	DATA_SEG		= 7
} Judge_Frame_Offset_t;

/* 帧头字节偏移 */
typedef enum {
	SOF			= 0,
	DATA_LENGTH	= 1,
	SEQ			= 3,
	CRC8		= 4
} Judge_Frame_Header_Offset_t;

typedef enum {
	ID_GAME_STATUS 					= 0x0001,	// 比赛状态			
	ID_GAME_RESULT 					= 0x0002,	// 比赛结果 		
	ID_GAME_ROBOT_HP 				= 0x0003,	// 机器人血量数据  	
	ID_EVENT_DATA 					= 0x0101,	// 场地事件数据		
	ID_SUPPLY_PROJECTILE_ACTION 	= 0x0102,	// 补给站动作标识	
	ID_SUPPLY_PROJECTILE_BOOKING 	= 0x0103,	// 请求补给站补弹子弹	
	ID_REFEREE_WARNING 				= 0x0104,	// 裁判警告信息
	ID_GAME_ROBOT_STATUS 			= 0x0201,	// 比赛机器人状态
	ID_POWER_HEAT_DATA 				= 0x0202,	// 实时功率热量数据
	ID_GAME_ROBOT_POS				= 0x0203,	// 机器人位置
	ID_BUFF							= 0x0204,	// 机器人增益
	ID_AERIAL_ROBOT_ENERGY			= 0x0205,	// 空中机器人能量状态
	ID_ROBOT_HURT					= 0x0206,	// 机器人伤害状态
	ID_SHOOT_DATA					= 0x0207,	// 实时射击信息
	ID_BULLET_REMAINING				= 0x0208,	// 子弹剩余发射数
	ID_COMMUNICATION				= 0x0301,	// 机器人间交互数据(发送方触发发送)
} Judge_Cmd_ID_t;

typedef enum {
	/* Std */
	LEN_FRAME_HEAD 					= 5,	// 帧头长度
	LEN_CMD_ID 						= 2,	// 命令码长度
	LEN_FRAME_TAIL 					= 2,	// 帧尾CRC16
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
};	// 方便调试的时候用

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* API functions -------------------------------------------------------------*/
/* #驱动层# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	裁判系统读取实时数据
 *	@note
 *			# 将递归那一部分(判断+调用)改成数组的方式就不会进入HardFault
 *				至于具体原因还没想清楚
 *			# 增大DMA的回环数据大小(Buffer Size)可以使得错误增加较缓慢
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
	
	/* 帧首字节是否为0xA5 */
	if(rxBuf[SOF] == JUDGE_FRAME_HEADER) 
	{
		/* 帧头CRC8校验 */
		if(Verify_CRC8_Check_Sum(rxBuf, LEN_FRAME_HEAD) == true) 
		{
			/* 统计一帧的总数据长度，用于CRC16校验 */
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
						Judge.hurt_data_update = true;	// 伤害数据更新
						break;
					case ID_SHOOT_DATA:
						memcpy(&Judge.ShootData, (rxBuf+DATA_SEG), LEN_SHOOT_DATA);
						JUDGE_ShootNumCount();	// 计算发弹量
						break;
					case ID_BULLET_REMAINING:
						memcpy(&Judge.BulletRemaining, (rxBuf+DATA_SEG), LEN_BULLET_REMAINING);
						break;
					case ID_COMMUNICATION:
						break;
				}
			}
		}

		/* 帧尾CRC16下一字节是否为0xA5 */
		if(rxBuf[ frame_length ] == JUDGE_FRAME_HEADER)
		{
			/* 如果一个数据包出现了多帧数据就再次读取 */
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

/* #信息层# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	用来判断此时裁判系统数据的正确性
 */
bool JUDGE_IfDataValid(void)
{
	return Judge.data_valid;
}

/**
 *	@brief	反馈底盘实时功率
 */
float JUDGE_fGetChassisRealPower(void)
{
	return (Judge.PowerHeatData.chassis_power);
}

/**
 *	@brief	反馈底盘缓冲能量
 */
float JUDGE_fGetChassisPowerBuffer(void)
{
	return (Judge.PowerHeatData.chassis_power_buffer);
}

/**
 *	@brief	反馈机器人等级
 */
uint8_t JUDGE_ucGetRobotLevel(void)
{
	return (Judge.GameRobotStatus.robot_level);
}

/**
 *	@brief	反馈机器人17mm枪口实时热量
 */
uint16_t JUDGE_usGetShooterRealHeat17(void)
{
	return (Judge.PowerHeatData.shooter_heat0);
}

/**
 *	@brief	反馈机器人17mm枪口热量上限
 */
uint16_t JUDGE_usGetShooterLimitHeat17(void)
{
	return (Judge.GameRobotStatus.shooter_heat0_cooling_limit);
}

/**
 *	@brief	反馈机器人17mm枪口热量冷却速率
 */
uint16_t JUDGE_usGetShooterHeatCoolingRate17(void)
{
	return (Judge.GameRobotStatus.shooter_heat0_cooling_rate);
}

/**
 *	@brief	反馈机器人17mm弹丸射速
 */
float JUDGE_fGetBulletSpeed17(void)
{
	return (Judge.ShootData.bullet_speed);
}

/**
 *	@brief	反馈机器人我方颜色
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
 *	@brief	反馈机器人受伤害的装甲板
 */
uint8_t JUDGE_eGetArmorHurt(void)
{
	static portTickType ulCurrent = 0;
	static portTickType ulDelay = 0;
	static bool bIfHurt = false;
	
	ulCurrent = xTaskGetTickCount();
	
	if(Judge.hurt_data_update == true) {
		Judge.hurt_data_update = false;	// 保证能判断到伤害数据的更新
		if(Judge.RobotHurt.hurt_type == 0x0) {	// 伤害类型 - 装甲伤害
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

/* #应用层# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
  * @brief  统计发弹量
  * @param  void
  * @retval void
  * @attention  中断中调用，不适用于双枪管(不准)
  */
portTickType shoot_ping;//计算出的最终发弹延迟
float Shoot_Speed_Now = 0;
float Shoot_Speed_Last = 0;
void JUDGE_ShootNumCount(void)
{
	portTickType shoot_time;//发射延时测试

	Shoot_Speed_Now = JUDGE_fGetBulletSpeed17();
	if(Shoot_Speed_Last != Shoot_Speed_Now)//因为是float型，几乎不可能完全相等,所以速度不等时说明发射了一颗弹
	{
		REVOLVER_AddShootCount();
		Shoot_Speed_Last = Shoot_Speed_Now;
	}
	shoot_time = xTaskGetTickCount();//获取弹丸发射时的系统时间
	shoot_ping = shoot_time - REVOLVER_GetRealShootTime();//计算延迟
}

/* #交互层# ---------------------------------------------------------------------------------------------------------------------------------------*/

#endif	// Judge Version --19
/*----------------------------------------------------------------------------*/
/*-----------------------------↑↑19裁判系统↑↑------------------------------*/
/*----------------------------------------------------------------------------*/















/*----------------------------------------------------------------------------*/
/*-----------------------------↓↓20裁判系统↓↓------------------------------*/
/*----------------------------------------------------------------------------*/
#if (JUDGE_VERSION == JUDGE_VERSION_20)

/* Private typedef -----------------------------------------------------------*/
/* 帧字节偏移 */
typedef enum {
	FRAME_HEADER	= 0,
	CMD_ID			= 5,
	DATA_SEG		= 7
} Judge_Frame_Offset_t;

/* 帧头字节偏移 */
typedef enum {
	SOF			= 0,
	DATA_LENGTH	= 1,
	SEQ			= 3,
	CRC8		= 4
} Judge_Frame_Header_Offset_t;

typedef enum {
	ID_GAME_STATUS 					= 0x0001,	// 比赛状态			
	ID_GAME_RESULT 					= 0x0002,	// 比赛结果 		
	ID_GAME_ROBOT_HP 				= 0x0003,	// 机器人血量数据  	
	ID_DART_STATUS					= 0x0004,	// 飞镖发射状态
	ID_ICRA_BUFF_DEBUFF_ZONE_STATUS = 0x0005,	// 人工智能挑战赛加成与惩罚区状态
	
	ID_EVENT_DATA 					= 0x0101,	// 场地事件数据		
	ID_SUPPLY_PROJECTILE_ACTION 	= 0x0102,	// 补给站动作标识	
	//ID_SUPPLY_PROJECTILE_BOOKING 	= 0x0103,	// 请求补给站补弹子弹
	ID_REFEREE_WARNING 				= 0x0104,	// 裁判警告信息
	ID_DART_REMAINING_TIME			= 0x0105,	// 飞镖发射口倒计时
	
	ID_GAME_ROBOT_STATUS 			= 0x0201,	// 比赛机器人状态
	ID_POWER_HEAT_DATA 				= 0x0202,	// 实时功率热量数据
	ID_GAME_ROBOT_POS				= 0x0203,	// 机器人位置
	ID_BUFF							= 0x0204,	// 机器人增益
	ID_AERIAL_ROBOT_ENERGY			= 0x0205,	// 空中机器人能量状态
	ID_ROBOT_HURT					= 0x0206,	// 机器人伤害状态
	ID_SHOOT_DATA					= 0x0207,	// 实时射击信息
	ID_BULLET_REMAINING				= 0x0208,	// 子弹剩余发射数
	ID_RFID_STATUS					= 0x0209,	// 机器人RFID状态
	
	ID_COMMUNICATION				= 0x0301,	// 机器人间交互数据(发送方触发发送)
} Judge_Cmd_ID_t;

typedef enum {
	/* Std */
	LEN_FRAME_HEAD 	= 5,	// 帧头长度
	LEN_CMD_ID 		= 2,	// 命令码长度
	LEN_FRAME_TAIL 	= 2,	// 帧尾CRC16
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
/* 裁判系统本地信息 */
Judge_Info_t Judge = 
{
	.frame_length = 0,
	.cmd_id = 0,
	.err_cnt = 0,
	.data_valid = false,
	.hurt_data_update = false,
};	// 方便调试的时候用

/* 裁判系统客户端信息 */
Judge_Client_Data_t Judge_Client =
{
	.FrameHeader.sof = 0xA5
};

/* 裁判系统机器人交互信息 */
Judge_Interact_Data_t Judge_Interact = 
{
	.FrameHeader.sof = 0xA5
};

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* API functions -------------------------------------------------------------*/
/* #驱动层# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	裁判系统读取实时数据
 *	@note
 *			# 将递归那一部分(判断+调用)改成数组的方式就不会进入HardFault
 *				至于具体原因还没想清楚
 *			# 增大DMA的回环数据大小(Buffer Size)可以使得错误增加较缓慢
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
	
	/* 帧首字节是否为0xA5 */
	if(rxBuf[SOF] == JUDGE_FRAME_HEADER) 
	{
		/* 帧头CRC8校验 */
		if(Verify_CRC8_Check_Sum(rxBuf, LEN_FRAME_HEAD) == true) 
		{
			/* 统计一帧的总数据长度，用于CRC16校验 */
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
							Judge.dart_data_update = true;	// 飞镖数据更新
						}break;
					
					case ID_EVENT_DATA: {
							memcpy(&Judge.EventData, (rxBuf+DATA_SEG), LEN_EVENT_DATA);
						}break;
					
					case ID_SUPPLY_PROJECTILE_ACTION: {
							memcpy(&Judge.SupplyProjectileAction, (rxBuf+DATA_SEG), LEN_SUPPLY_PROJECTILE_ACTION);
							Judge.supply_data_update = true;	// 补给站数据更新
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
							Judge.hurt_data_update = true;	// 伤害数据更新
						}break;
					
					case ID_SHOOT_DATA: {
							memcpy(&Judge.ShootData, (rxBuf+DATA_SEG), LEN_SHOOT_DATA);
							JUDGE_ShootNumCount();	// 计算发弹量
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

		/* 帧尾CRC16下一字节是否为0xA5 */
		if(rxBuf[ frame_length ] == JUDGE_FRAME_HEADER)
		{
			/* 如果一个数据包出现了多帧数据就再次读取 */
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

/* #信息层# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	用来判断此时裁判系统数据的正确性
 */
bool JUDGE_IfDataValid(void)
{
	return Judge.data_valid;
}

/**
 *	@brief	反馈底盘实时功率
 */
float JUDGE_fGetChassisRealPower(void)
{
	return (Judge.PowerHeatData.chassis_power);
}

/**
 *	@brief	反馈底盘缓冲能量
 */
float JUDGE_fGetChassisPowerBuffer(void)
{
	return (Judge.PowerHeatData.chassis_power_buffer);
}

/**
 *	@brief	反馈机器人等级
 */
uint8_t JUDGE_ucGetRobotLevel(void)
{
	return (Judge.GameRobotStatus.robot_level);
}

/**
 *	@brief	反馈机器人17mm枪口实时热量
 */
uint16_t JUDGE_usGetShooterRealHeat17(void)
{
	return (Judge.PowerHeatData.shooter_heat0);
}

/**
 *	@brief	反馈机器人17mm枪口热量上限
 */
uint16_t JUDGE_usGetShooterLimitHeat17(void)
{
	return (Judge.GameRobotStatus.shooter_heat0_cooling_limit);
}

/**
 *	@brief	反馈机器人17mm枪口热量冷却速率
 */
uint16_t JUDGE_usGetShooterHeatCoolingRate17(void)
{
	return (Judge.GameRobotStatus.shooter_heat0_cooling_rate);
}

/**
 *	@brief	反馈机器人17mm弹丸射速
 */
float JUDGE_fGetBulletSpeed17(void)
{
	return (Judge.ShootData.bullet_speed);
}

/**
 *	@brief	反馈机器人17mm弹丸射速上限
 */
uint8_t JUDGE_ucGetBulletLimitSpeed17(void)
{
	return (Judge.GameRobotStatus.shooter_heat0_speed_limit);
}

/**
 *	@brief	反馈机器人最大底盘功率
 */
uint8_t JUDGE_ucGetMaxPower(void)
{
	return (Judge.GameRobotStatus.max_chassis_power);
}

/**
 *	@brief	反馈机器人我方颜色
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
 *	@brief	反馈机器人受伤害的装甲板
 */
uint8_t JUDGE_eGetArmorHurt(void)
{
	static portTickType ulCurrent = 0;
	static portTickType ulDelay = 0;
	static bool bIfHurt = false;
	
	ulCurrent = xTaskGetTickCount();
	
	if(Judge.hurt_data_update == true) {
		Judge.hurt_data_update = false;	// 保证能判断到伤害数据的更新
		if(Judge.RobotHurt.hurt_type == 0x0) {	// 伤害类型 - 装甲伤害
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
 *	@brief	反馈是否有飞镖发射
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
 *	@brief	反馈补给站ID
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
 *	@brief	反馈机器人枪口位置
 */
float JUDGE_fGetShooterYaw(void)
{
	return (Judge.GameRobotPos.yaw);
}

/* #应用层# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
  * @brief  统计发弹量
  * @param  void
  * @retval void
  * @attention  中断中调用，不适用于双枪管(不准)
  */
portTickType shoot_ping;//计算出的最终发弹延迟
float Shoot_Speed_Now = 0;
float Shoot_Speed_Last = 0;
void JUDGE_ShootNumCount(void)
{
	portTickType shoot_time;//发射延时测试

	Shoot_Speed_Now = JUDGE_fGetBulletSpeed17();
	if(Shoot_Speed_Last != Shoot_Speed_Now)//因为是float型，几乎不可能完全相等,所以速度不等时说明发射了一颗弹
	{
		REVOLVER_AddShootCount();
		Shoot_Speed_Last = Shoot_Speed_Now;
	}
	shoot_time = xTaskGetTickCount();//获取弹丸发射时的系统时间
	shoot_ping = shoot_time - REVOLVER_GetRealShootTime();//计算延迟
}

/* #交互层# ---------------------------------------------------------------------------------------------------------------------------------------*/
/* 
	机器人 ID：					客户端 ID：
	1，英雄(红)；				0x0101 为英雄操作手客户端( 红) ；
	2，工程(红)；				0x0102 ，工程操作手客户端 ((红 )；
	3/4/5，步兵(红)；			0x0103/0x0104/0x0105，步兵操作手客户端(红)；
	6，空中(红)；				0x0106，空中操作手客户端((红)； 
	7，哨兵(红)；

	101，英雄(蓝)；				0x0165，英雄操作手客户端(蓝)；
	102，工程(蓝)；				0x0166，工程操作手客户端(蓝)；
	103/104/105，步兵(蓝)；		0x0167/0x0168/0x0169，步兵操作手客户端(蓝)；
	106，空中(蓝)；				0x016A，空中操作手客户端(蓝)。 
	107，哨兵(蓝)；
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
 *	@brief	上传自定义数据至客户端界面
 */
#define CLIENT_FRAME_LEN	(15+105)
static uint8_t txClientBuf[CLIENT_FRAME_LEN];
void JUDGE_SendToClient(void)
{
	uint8_t frame_length;
	
	JUDGE_DetermineClientId();
	
	// 帧头数据填充
	Judge_Client.FrameHeader.sof = 0xA5;
	Judge_Client.FrameHeader.data_length = sizeof(Judge_Client.DataFrameHeader) + sizeof(Judge_Client.ClientData);
	Judge_Client.FrameHeader.seq = 0;

	memcpy(txClientBuf, &Judge_Client.FrameHeader, sizeof(Judge_Client.FrameHeader));
	// 写入帧头CRC8校验码
	Append_CRC8_Check_Sum(txClientBuf, sizeof(Judge_Client.FrameHeader));
	
	// 命令码
	Judge_Client.CmdId = ID_COMMUNICATION;
	
	// 发送7个图形
	Judge_Client.DataFrameHeader.data_cmd_id = 0x0104;
	
	// 发送者ID
	Judge_Client.DataFrameHeader.send_ID = Judge.GameRobotStatus.robot_id;
	// 发送者客户端ID
	Judge_Client.DataFrameHeader.receiver_ID = Judge.self_client_id;
	
	/*----数据段内容----*/
	//...
	
	// 数据填充
	memcpy(	
			txClientBuf + CMD_ID, 
			(uint8_t*)&Judge_Client.CmdId, 
			(sizeof(Judge_Client.CmdId)+ sizeof(Judge_Client.DataFrameHeader)+ sizeof(Judge_Client.ClientData))
		  );			
	
	frame_length = sizeof(Judge_Client);
	//写入数据段CRC16校验码		
	Append_CRC16_Check_Sum(txClientBuf, frame_length);	

	for(uint8_t i = 0; i < frame_length; i++) {
		UART5_SendChar(txClientBuf[i]);
	}
	
	/* 发送数据包清零 */
	memset(txClientBuf, 0, CLIENT_FRAME_LEN);	
}

/**
 *	@brief	上传自定义数据至队友
 */
#define INTERACT_FRAME_LEN	(15+INTERACT_DATA_LEN)
static uint8_t txInteractBuf[INTERACT_FRAME_LEN];
void JUDGE_SendToTeammate(uint8_t teammate_id)
{
	uint8_t frame_length;

	// 帧头数据填充
	Judge_Interact.FrameHeader.sof = 0xA5;
	Judge_Interact.FrameHeader.data_length = sizeof(Judge_Interact.DataFrameHeader) + sizeof(Judge_Interact.InteractData);
	Judge_Interact.FrameHeader.seq = 0;	
	
	memcpy(txInteractBuf, &Judge_Interact.FrameHeader, sizeof(Judge_Interact.FrameHeader));
	// 写入帧头CRC8校验码
	Append_CRC8_Check_Sum(txInteractBuf, sizeof(Judge_Interact.FrameHeader));
	
	// 命令码
	Judge_Interact.CmdId = ID_COMMUNICATION;
	
	// 自定义命令码
	Judge_Interact.DataFrameHeader.data_cmd_id = 0x0200;	// 在0x0200-0x02ff之间选择
	
	// 发送者ID
	Judge_Interact.DataFrameHeader.send_ID = Judge.GameRobotStatus.robot_id;
	// 接收者ID
	Judge_Interact.DataFrameHeader.receiver_ID = teammate_id;
	
	/*----数据段内容----*/
	//...
	
	// 数据填充
	memcpy(	
			txClientBuf + CMD_ID, 
			(uint8_t*)&Judge_Interact.CmdId, 
			(sizeof(Judge_Interact.CmdId)+ sizeof(Judge_Interact.DataFrameHeader)+ sizeof(Judge_Interact.InteractData))
		  );			
	
	frame_length = sizeof(Judge_Interact);
	// 写入数据段CRC16校验码		
	Append_CRC16_Check_Sum(txInteractBuf, frame_length);	

	for(uint8_t i = 0; i < frame_length; i++) {
		UART5_SendChar(txInteractBuf[i]);
	}

	/* 发送数据包清零 */
	memset(txInteractBuf, 0, INTERACT_FRAME_LEN);	
}

/**
 *	@brief	接收机器人间的通信数据
 */
void JUDGE_ReadFromCom()
{
	
}
	
#endif	// Judge Version --20
/*----------------------------------------------------------------------------*/
/*-----------------------------↑↑20裁判系统↑↑------------------------------*/
/*----------------------------------------------------------------------------*/
