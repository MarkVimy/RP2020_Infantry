#ifndef __JUDGE_H
#define __JUDGE_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h" 
#include "stdbool.h"

/* Global macro --------------------------------------------------------------*/
// 2019/07/11(V2.0)
#define JUDGE_VERSION_19	19
// 2020/02/25(V1.0)
#define JUDGE_VERSION_20	20

#define JUDGE_VERSION		JUDGE_VERSION_20

// 机器人间交互数据段长度
#define INTERACT_DATA_LEN	10

/* Global TypeDef ------------------------------------------------------------*/
typedef enum {
	ARMOR_0 = 0,
	ARMOR_1 = 1,
	ARMOR_2 = 2,
	ARMOR_3 = 3,
	ARMOR_COUNT = 4,
	ARMOR_NONE = 0xff,
} Armor_Id_Names_t;

typedef enum {
	RED = 0,
	BLUE = 1,
	COLOR_NONE = 0xff,
} Color_t;

typedef enum {
	SUPPLY_ID_1 = 1,
	SUPPLY_ID_2 = 2,
	SUPPLY_ID_NONE = 0xff,
} Supply_Id_t;

typedef enum
{
	//0x200-0x02ff 	自定义命令 格式  INTERACT_ID_XXXX
	INTERACT_ID_delete_graphic 		= 0x0100,	/*客户端删除图形*/
	INTERACT_ID_draw_one_graphic 	= 0x0101,	/*客户端绘制一个图形*/
	INTERACT_ID_draw_two_graphic 	= 0x0102,	/*客户端绘制2个图形*/
	INTERACT_ID_draw_five_graphic 	= 0x0103,	/*客户端绘制5个图形*/
	INTERACT_ID_draw_seven_graphic 	= 0x0104,	/*客户端绘制7个图形*/
	INTERACT_ID_draw_char_graphic 	= 0x0105	/*客户端绘制字符图形*/
} Interact_Id;

/*自定义交互ID*/

typedef enum
{
	LEN_INTERACT_delete_graphic = 8,	/*删除图层*/
	LEN_INTERACT_draw_one_graphic = 21,
	LEN_INTERACT_draw_two_graphic = 36,
	LEN_INTERACT_draw_five_graphic = 81,
	LEN_INTERACT_draw_seven_graphic = 111,
	LEN_INTERACT_draw_char_graphic = 51
} Len_Interact;

typedef enum
{
	RED_HERO = 1, 
	RED_ENGINE = 2,
	RED_INFANTRY_1 = 3,
	RED_INFANTRY_2 = 4,
	RED_INFANTRY_3 = 5,
	RED_AERIAL = 6,
	RED_SENTRY = 7,
	/*红方机器人ID*/
	BLUE_HERO = 101,
	BLUE_ENGINE = 102,
	BLUE_INFANTRY_1 = 103,
	BLUE_INFANTRY_2 = 104,
	BLUE_INFANTRY_3 = 105,
	BLUE_AERIAL = 106,
	BLUE_SENTRY = 107,
	/*蓝方机器人ID*/
	RED_HERO_CLIENT = 0x101,
	RED_ENGINE_CLIENT = 0x102,
	RED_INFANTRY_CLIENT_1 = 0x103,
	RED_INFANTRY_CLIENT_2 = 0x104,
	RED_INFANTRY_CLIENT_3 = 0x105,
	RED_AERIAL_CLIENT = 0x106,
	/*红方操作手客户端ID*/
	BLUE_HERO_CLIENT = 0x165,
	BLUE_ENGINE_CLIENT = 0x166,
	BLUE_INFANTRY_CLIENT_1 = 0x167,
	BLUE_INFANTRY_CLIENT_2 = 0x168,
	BLUE_INFANTRY_CLIENT_3 = 0x169,
	BLUE_AERIAL_CLIENT = 0x16A
	/*蓝方操作手客户端ID*/
}Interact_Target;
/*机器人及各客户端的发送接收ID信息*/


typedef enum
{
	RED_BLUE = 0,	
	YELLOW = 1,
	GREEN = 2,
	ORANGE = 3,
	FUCHSIA = 4,	/*紫红色*/
	PINK = 5,
	CYAN_BLUE = 6,	/*青色*/
	BLACK = 7,
	WHITE = 8
}Graphic_Color;
/*图层颜色类型*/


typedef enum
{
	LINE = 0,
	RECTANGLE = 1,
	CIRCLE = 2,
	OVAL = 3, /*椭圆*/
	ARC = 4, /*圆弧*/
	FLOAT = 5, 
	INT = 6,
	CHAR = 7
}Graphic_Type;
/*图层类型*/

typedef enum
{
	NONE = 0,
	ADD = 1,	/*增加图层*/
	MODIFY = 2,		/*修改图层*/
	DELETE = 3		/*删除图层*/
}Graphic_Operate;
/*图层操作*/

/*
·获取比赛时间
·获取血量信息 -- 可以用来判断是否打中，以及基地和前哨站的血量状态
·飞镖发射时间&&发射口的倒计时 -- 可以用来触发反导模式和结束反导模式
·场地事件信息 -- 可以用来影响打击和防守的决策
·热量功率信息 -- 可以用来限热量和功率
·枪口位置 -- 参考
·buff -- 影响打击防守策略
·射击速度和剩余子弹数 -- 影响打击策略
*/

/*----------------------------------------------------------------------------*/
/*-----------------------------↓↓20裁判系统↓↓------------------------------*/
/*----------------------------------------------------------------------------*/
#if (JUDGE_VERSION == JUDGE_VERSION_20)

/* 自定义帧头 Byte: 5 */
typedef __packed struct
{
	uint8_t  sof;
	uint16_t data_length;
	uint8_t  seq;
	uint8_t  crc8;
} std_frame_header_t;


/* ID: 0x0001	Byte: 	3	比赛状态数据 */
typedef __packed struct 
{ 
	uint8_t game_type : 4;			// 比赛类型
	uint8_t game_progress : 4;		// 比赛阶段
	uint16_t stage_remain_time;		// 当前阶段剩余时间(单位:s)
} ext_game_status_t; 


/* ID: 0x0002	Byte:	1	比赛结果数据 */
typedef __packed struct 
{ 
	uint8_t winner;
} ext_game_result_t; 


/* ID: 0x0003	Byte:	32	机器人血量数据数据 */
typedef __packed struct 
{ 
	uint16_t red_1_robot_HP;	// 红1英雄机器人血量(未上场及罚下血量为0)
	uint16_t red_2_robot_HP;	// 红2工程机器人血量
	uint16_t red_3_robot_HP;	// 红3步兵机器人血量
	uint16_t red_4_robot_HP;	// 红4步兵机器人血量
	uint16_t red_5_robot_HP;	// 红5步兵机器人血量
	uint16_t red_7_robot_HP;	// 红7哨兵机器人血量
	uint16_t red_outpost_HP;	// 红方前哨站血量
	uint16_t red_base_HP;		// 红方基地血量
	uint16_t blue_1_robot_HP;	// 蓝1英雄机器人血量
	uint16_t blue_2_robot_HP;	// 蓝2工程机器人血量
	uint16_t blue_3_robot_HP;	// 蓝3步兵机器人血量
	uint16_t blue_4_robot_HP;	// 蓝4步兵机器人血量
	uint16_t blue_5_robot_HP;	// 蓝5步兵机器人血量
	uint16_t blue_7_robot_HP;	// 蓝7哨兵机器人血量
	uint16_t blue_outpost_HP;	// 蓝方前哨站血量
	uint16_t blue_base_HP;		// 蓝方基地血量	
} ext_game_robot_HP_t; 


/* ID: 0x0004 	Byte:	3	飞镖发射状态 */
typedef __packed struct
{
	uint8_t dart_belong;
	uint16_t stage_remaining_time;
} ext_dart_status_t;


/* ID: 0x0005 	Byte:	3	人工智能挑战赛加成与惩罚区状态 */
typedef __packed struct
{
	uint8_t F1_zone_status:1;
	uint8_t F1_zone_buff_debuff_status:3;
	uint8_t F2_zone_status:1;
	uint8_t F2_zone_buff_debuff_status:3;
	uint8_t F3_zone_status:1;
	uint8_t F3_zone_buff_debuff_status:3;
	uint8_t F4_zone_status:1;
	uint8_t F4_zone_buff_debuff_status:3;
	uint8_t F5_zone_status:1;
	uint8_t F5_zone_buff_debuff_status:3;
	uint8_t F6_zone_status:1;
	uint8_t F6_zone_buff_debuff_status:3;
} ext_ICRA_buff_debuff_zone_status_t;


/* ID: 0x0101  Byte:  4    场地事件数据 */
typedef __packed struct 
{ 
	uint32_t event_type;
} ext_event_data_t; 


/* ID: 0x0102  Byte:  4    补给站动作标识数据 */
typedef __packed struct 
{ 
	uint8_t supply_projectile_id;
	uint8_t supply_robot_id;
	uint8_t supply_projectile_step;
	uint8_t supply_projectile_num;
} ext_supply_projectile_action_t; 


///* ID: 0X0103  Byte:  3    请求补给站补弹子弹数据 */
//typedef __packed struct 
//{ 
//	uint8_t supply_projectile_id;
//	uint8_t supply_robot_id;
//	uint8_t supply_num;  
//} ext_supply_projectile_booking_t; 


/* ID: 0X0104  Byte:  2	   裁判警告信息 */
typedef __packed struct
{
	uint8_t level;
	uint8_t foul_robot_id;
} ext_referee_warning_t;


/* ID: 0X0105  Byte:  1	   飞镖发射口倒计时 */
typedef __packed struct
{
	uint8_t dart_remaining_time;
} ext_dart_remaining_time_t;


/* ID: 0X0201  Byte: 18    机器人状态数据 */
typedef __packed struct 
{ 
	uint8_t robot_id;   					// 机器人ID，可用来校验发送
	uint8_t robot_level;  					// 1一级，2二级，3三级
	uint16_t remain_HP;  					// 机器人剩余血量
	uint16_t max_HP; 						// 机器人满血量
	uint16_t shooter_heat0_cooling_rate;  	// 机器人 17mm 枪口每秒冷却值
	uint16_t shooter_heat0_cooling_limit;   // 机器人 17mm 枪口热量上限
	uint16_t shooter_heat1_cooling_rate;	// 机器人 42mm 枪口每秒冷却值
	uint16_t shooter_heat1_cooling_limit;   // 机器人 42mm 枪口热量上限
	uint8_t shooter_heat0_speed_limit;		// 机器人 17mm 枪口上限速度(m/s)
	uint8_t shooter_heat1_speed_limit; 		// 机器人 42mm 枪口上限速度(m/s)
	uint8_t max_chassis_power;				// 机器人最大底盘功率(W)
	uint8_t mains_power_gimbal_output : 1;  // gimbal口输出	
	uint8_t mains_power_chassis_output : 1; // chassis口输出
	uint8_t mains_power_shooter_output : 1; // shooter口输出
} ext_game_robot_status_t; 


/* ID: 0X0202  Byte: 16    实时功率热量数据 */
typedef __packed struct 
{ 
	uint16_t chassis_volt;   		// 底盘输出电压，单位：mV
	uint16_t chassis_current;		// 底盘输出电流，单位：mA
	float chassis_power;   			// 瞬时功率，单位：W
	uint16_t chassis_power_buffer;	// 底盘功率缓冲，单位：60J焦耳缓冲能量(飞坡根据规则增加至250J)
	uint16_t shooter_heat0;			// 17mm 枪口热量
	uint16_t shooter_heat1;  		// 42mm 枪口热量
	uint16_t mobile_shooter_heat2;	// 机动 17mm 枪口热量
} ext_power_heat_data_t; 


/* ID: 0x0203  Byte: 16    机器人位置数据 */
typedef __packed struct 
{   
	float x;   
	float y;   
	float z;   
	float yaw; 
} ext_game_robot_pos_t; 


/* ID: 0x0204  Byte:  1    机器人增益数据 */
typedef __packed struct 
{ 
	uint8_t power_rune_buff; 
} ext_buff_t; 


/* ID: 0x0205  Byte:  3    空中机器人能量状态数据 */
typedef __packed struct 
{ 
	uint16_t energy_point;
	uint8_t attack_time;
} ext_aerial_robot_energy_t; 


/* ID: 0x0206  Byte:  1    伤害状态数据 */
typedef __packed struct 
{ 
	uint8_t armor_id : 4; 	// 装甲伤害时代表装甲ID
	uint8_t hurt_type : 4; 	// 0x0装甲伤害 0x1模块掉线 0x2超射速 0x3超热量 0x4超功率 0x5撞击
} ext_robot_hurt_t; 


/* ID: 0x0207  Byte:  6    实时射击数据 */
typedef __packed struct 
{ 
	uint8_t bullet_type; 	// 子弹类型(1-17mm, 2-42mm)
	uint8_t bullet_freq;  	// 子弹射频(Hz)
	float bullet_speed;		// 子弹射速(m/s)
} ext_shoot_data_t; 


/* ID: 0x0208  Byte:  2    子弹剩余发射数数据 */
typedef __packed struct
{
	uint16_t bullet_remaining_num;
} ext_bullet_remaining_t;


/* ID: 0x0209  Byte:  4 	机器人RFID状态 */
typedef __packed struct
{
	uint32_t rfid_status;
} ext_rfid_status_t;

/**
 *	-----------------------------------
 *	# 机器人间交互数据
 *	-----------------------------------
 */

/* 
	交互数据，包括一个统一的数据段头结构，
	包含了内容 ID，发送者以及接收者的 ID 和内容数据段，
	整个交互数据的包总共长最大为 128 个字节，
	减去 frame_header,cmd_id,frame_tail 共9个字节以及数据段头结构的 6 个字节，
	故而发送的内容数据段最大为 113。
	整个交互数据 0x0301 的包上行频率为 10Hz。

	机器人 ID：
	1，英雄(红)；
	2，工程(红)；
	3/4/5，步兵(红)；
	6，空中(红)；
	7，哨兵(红)；

	101，英雄(蓝)；
	102，工程(蓝)；
	103/104/105，步兵(蓝)；
	106，空中(蓝)；
	107，哨兵(蓝)；

	客户端 ID： 
	0x0101 为英雄操作手客户端( 红) ；
	0x0102 ，工程操作手客户端 ((红 )；
	0x0103/0x0104/0x0105，步兵操作手客户端(红)；
	0x0106，空中操作手客户端((红)； 

	0x0165，英雄操作手客户端(蓝)；
	0x0166，工程操作手客户端(蓝)；
	0x0167/0x0168/0x0169，步兵操作手客户端(蓝)；
	0x016A，空中操作手客户端(蓝)。 
*/
/* 交互数据接收信息：0x0301  */
typedef __packed struct 
{ 
	uint16_t data_cmd_id;    
	uint16_t send_ID;    
	uint16_t receiver_ID; 
} ext_student_interactive_header_data_t; 


/* 
	学生机器人间通信 cmd_id 0x0301，内容 ID:0x0200~0x02FF
	交互数据 机器人间通信：0x0301。
	发送频率：上限 10Hz  

	字节偏移量 	大小 	说明 			备注 
	0 			2 		数据的内容 ID 	0x0200~0x02FF 
										可以在以上 ID 段选取，具体 ID 含义由参赛队自定义 
	
	2 			2 		发送者的 ID 		需要校验发送者的 ID 正确性， 
	
	4 			2 		接收者的 ID 		需要校验接收者的 ID 正确性，
										例如不能发送到敌对机器人的ID 
	
	6 			n 		数据段 			n 需要小于 113 

*/
/*
			最大上行每秒字节量	最大下行每秒字节量
	步兵		3720Bytes/s			3720Bytes/s
*/
typedef __packed struct 
{ 
	uint8_t data[INTERACT_DATA_LEN]; //数据段,n需要小于113
} robot_interactive_data_t;

typedef __packed struct 
{ 
	uint8_t operate_type;  
	uint8_t layer;  
} ext_client_custom_graphic_delete_t;

typedef __packed struct 
{  
	uint8_t graphic_name[3];
	uint32_t operate_tpye:3;
	uint32_t graphic_tpye:3;
	uint32_t layer:4;
	uint32_t color:4;
	uint32_t start_angle:9; 
	uint32_t end_angle:9; 
	uint32_t width:10;  
	uint32_t start_x:11;  
	uint32_t start_y:11;  
	uint32_t radius:10;  
	uint32_t end_x:11;  
	uint32_t end_y:11;  
} graphic_data_struct_t; 

typedef __packed struct 
{   
	graphic_data_struct_t  grapic_data_struct; 
} ext_client_custom_graphic_single_t;
 
typedef __packed struct 
{ 
	graphic_data_struct_t  grapic_data_struct[2]; 
} ext_client_custom_graphic_double_t; 

typedef __packed struct 
{ 
	graphic_data_struct_t  grapic_data_struct[5];
} ext_client_custom_graphic_five_t; 

typedef __packed struct 
{ 
	graphic_data_struct_t  grapic_data_struct[7]; 
} ext_client_custom_graphic_seven_t; 

typedef __packed struct 
{
	graphic_data_struct_t  grapic_data_struct;
	uint8_t data[30]; 
} ext_client_custom_character_t; 

typedef __packed struct
{
	std_frame_header_t						FrameHeader;	// 帧头
	uint16_t 								CmdId;			// 命令码
	ext_student_interactive_header_data_t	DataFrameHeader;// 数据段头结构
	ext_client_custom_graphic_single_t		ClientData;		// 数据(1个图形) <- 根据图形数修改
	uint16_t								FrameTail;		// 帧尾
} Judge_Client_Data_t;

typedef __packed struct
{
	std_frame_header_t						FrameHeader;	// 帧头
	uint16_t 								CmdId;			// 命令码
	ext_student_interactive_header_data_t	DataFrameHeader;// 数据段头结构
	robot_interactive_data_t				InteractData;	// 数据
	uint16_t								FrameTail;		// 帧尾	
} Judge_Interact_Data_t;

typedef struct 
{
	uint16_t frame_length;	// 整帧长度(调试时使用)
	uint16_t cmd_id;		// 命令码(调试时使用)
	uint16_t err_cnt;		// 错帧数(调试时使用)
	bool	 data_valid;	// 数据有效性
	uint16_t self_client_id;// 发送者机器人对应的客户端ID
	bool	 hurt_data_update;	// 伤害数据更新
	bool	dart_data_update;	// 飞镖数据更新
	bool	supply_data_update;	// 补给站数据更新
	std_frame_header_t				FrameHeader;				// 帧头信息
	ext_game_status_t 				GameStatus;					// 0x0001
	ext_game_result_t 				GameResult;					// 0x0002
	ext_game_robot_HP_t 			GameRobotHP;				// 0x0003
	ext_dart_status_t				DartStatus;					// 0x0004
	//ext_ICRA_buff_debuff_zone_status_t	
	
	ext_event_data_t				EventData;					// 0x0101
	ext_supply_projectile_action_t	SupplyProjectileAction;		// 0x0102
	//ext_supply_projectile_booking_t SupplyProjectileBooking;	// 0x0103
	ext_referee_warning_t			RefereeWarning;				// 0x0104
	ext_dart_remaining_time_t		DartRemainingTime;			// 0x0105
	
	ext_game_robot_status_t			GameRobotStatus;			// 0x0201
	ext_power_heat_data_t			PowerHeatData;				// 0x0202
	ext_game_robot_pos_t			GameRobotPos;				// 0x0203
	ext_buff_t						Buff;						// 0x0204
	ext_aerial_robot_energy_t		AerialRobotEnergy;			// 0x0205
	ext_robot_hurt_t				RobotHurt;					// 0x0206
	ext_shoot_data_t				ShootData;					// 0x0207
	ext_bullet_remaining_t			BulletRemaining;			// 0x0208	
	ext_rfid_status_t				RfidStatus;					// 0x0209
	
}Judge_Info_t;

/*----------------------------------------------------------------------------*/
/*---------------------------↑↑裁判系统数据结构↑↑---------------------------*/
/*----------------------------------------------------------------------------*/

/* ## Global Variables Prototypes ## -----------------------------------------*/
extern Judge_Info_t Judge;

/* API functions Prototypes --------------------------------------------------*/
/* #驱动层# ---------------------------------------------------------------------------------------------------------------------------------------*/
bool JUDGE_ReadData(uint8_t *rxBuf);

/* #信息层# ---------------------------------------------------------------------------------------------------------------------------------------*/
bool 		JUDGE_IfDataValid(void);
float 		JUDGE_fGetChassisRealPower(void);
float 		JUDGE_fGetChassisPowerBuffer(void);
uint8_t 	JUDGE_ucGetRobotLevel(void);
uint16_t 	JUDGE_usGetShooterRealHeat17(void);
uint16_t 	JUDGE_usGetShooterLimitHeat17(void);
uint16_t 	JUDGE_usGetShooterHeatCoolingRate17(void);
float 		JUDGE_fGetBulletSpeed17(void);
uint8_t 	JUDGE_ucGetBulletLimitSpeed17(void);
uint8_t 	JUDGE_ucGetMaxPower(void);
Color_t 	JUDGE_eGeyMyColor(void);
uint8_t 	JUDGE_eGetArmorHurt(void);
Color_t 	JUDGE_eGetDartBelong(void);
Supply_Id_t JUDGE_eGetSupplyId(void);
float 		JUDGE_fGetShooterYaw(void);

/* #应用层# ---------------------------------------------------------------------------------------------------------------------------------------*/
void JUDGE_ShootNumCount(void);
void JUDGE_SendToClient(void);
void JUDGE_SendToTeammate(uint8_t teammate_id);

/* #交互层# ---------------------------------------------------------------------------------------------------------------------------------------*/
void Draw_Rectangle(graphic_data_struct_t* graphic,
										const uint8_t *name,
										uint8_t layer,
										Graphic_Color color,
										uint16_t width,
										uint16_t start_x,
										uint16_t start_y,
										uint16_t diagonal_x,
										uint16_t diagonal_y);
void Draw_Line(graphic_data_struct_t* graphic,
							const uint8_t *name,
							uint8_t layer,
							Graphic_Color color,
							uint16_t width,
							uint16_t start_x,
							uint16_t start_y,
							uint16_t end_x,
							uint16_t end_y);
void Draw_Circle(graphic_data_struct_t* graphic,
							const uint8_t *name,
							uint8_t layer,
							Graphic_Color color,
							uint16_t width,
							uint16_t center_x,
							uint16_t center_y,
							uint16_t radius);
void Draw_Oval(graphic_data_struct_t* graphic,
							const uint8_t *name,
							uint8_t layer,
							Graphic_Color color,
							uint16_t width,
							uint16_t center_x,
							uint16_t center_y,
							uint16_t axis_x,
							uint16_t axis_y);
void Draw_ARC(graphic_data_struct_t* graphic,
							const uint8_t *name,
							uint8_t layer,
							Graphic_Color color,
							uint16_t start_angle,
							uint16_t end_angle,
							uint16_t width,
							uint16_t center_x,
							uint16_t center_y,
							uint16_t axis_x,
							uint16_t axis_y);
void Draw_Float(graphic_data_struct_t* graphic,
								const uint8_t *name,
								uint8_t layer,
								Graphic_Color color,
								uint16_t font_size,
								uint16_t accuracy,
								uint16_t width,
								uint16_t start_x,
								uint16_t start_y,
								float number);
void Draw_Int(graphic_data_struct_t* graphic,
							const uint8_t *name,
							uint8_t layer,
							Graphic_Color color,
							uint16_t font_size,
							uint16_t width,
							uint16_t start_x,
							uint16_t start_y,
							int32_t number);
void Draw_Char(ext_client_custom_character_t* graphic,
							const uint8_t *name,
							uint8_t layer,
							Graphic_Color color,
							uint16_t font_size,
							uint16_t length,
							uint16_t width,
							uint16_t start_x,
							uint16_t start_y,
							const uint8_t *character);
void Add_Graphic(graphic_data_struct_t* graphic,
									const uint8_t* name,
									uint8_t layer,
									uint8_t type,
									uint8_t color,
									uint16_t start_angle,
									uint16_t end_angle,
									uint16_t width,
									uint16_t start_x,
									uint16_t start_y,
									uint16_t radius,
									uint16_t end_x,
									uint16_t end_y);
void Modify_Graphic(graphic_data_struct_t* graphic,
								  const uint8_t* name,
									uint8_t layer,
									uint8_t type,
									uint8_t color,
									uint16_t start_angle,
									uint16_t end_angle,
									uint16_t width,
									uint16_t start_x,
									uint16_t start_y,
									uint16_t radius,
									uint16_t end_x,
									uint16_t end_y);
void Delete_Graphic(graphic_data_struct_t* graphic);
void Delete_Graphic_Layer(Interact_Target target,uint8_t layer);
void Delete_All_Graphic_Layer(Interact_Target target);
bool Send_Interact_Data(robot_interactive_data_t *str,uint16_t length);
void Modify_Float(graphic_data_struct_t* graphic,float number);
void Modify_Int(graphic_data_struct_t* graphic,int32_t number);
void Modify_Char(ext_client_custom_character_t* graphic,const uint8_t *character,uint16_t length);	

								  
#endif	// Judge Version --20
/*----------------------------------------------------------------------------*/
/*-----------------------------↑↑20裁判系统↑↑------------------------------*/
/*----------------------------------------------------------------------------*/

#endif	// __JUDGE_H

