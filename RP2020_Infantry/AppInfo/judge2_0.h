#ifndef __JUDGE_H
#define __JUDGE_H

#include "sys.h"
#include "uart5.h"

void Read_Judge_Info(uint8_t *rx_buf);

void Judge_Task(void);

bool Send_Interact_Data(robot_interactive_data_t *str,uint16_t length);

void Delete_All_Graphic_Layer(Interact_Target target);

void Delete_Graphic_Layer(Interact_Target target,uint8_t layer);

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

void Modify_Float(graphic_data_struct_t* graphic,float number);


void Modify_Int(graphic_data_struct_t* graphic,int32_t number);


void Modify_Char(ext_client_custom_character_t* graphic,const uint8_t *character,uint16_t length);

/*      judge system     */

//裁判系统控制

#define JUDGE_BUF_LENGTH 200
	//裁判系统缓冲区长度
#define JUDGE_FRAME_HEADER  0xA5
//帧头



enum
{ 
	ID_game_state       						= 0x0001,//比赛状态数据，1Hz
	ID_game_result 	   							= 0x0002,//比赛结果数据，比赛结束发送
	ID_game_robot_HP       					= 0x0003,//比赛机器人血量数据，1Hz发送
	ID_dart_status									= 0x0004,//飞镖发射状态，飞镖发射时发送
	ID_ICRA_buff_debuff_zone_status = 0x0005,//人工智能挑战赛加成与惩罚区状态，1Hz
	ID_event_data  									= 0x0101,//场地事件数据，1Hz
	ID_supply_projectile_action   	= 0x0102,//场地补给站动作标识数据
	ID_referee_warning					 		= 0x0104,//裁判警告数据，警告后发送
	ID_dart_remaining_time					= 0x0105,//飞镖发射口倒计时，1Hz
	ID_game_robot_state    					= 0x0201,//机器人状态数据，10Hz
	ID_power_heat_data    					= 0x0202,//实时功率热量数据，50Hz
	ID_game_robot_pos        				= 0x0203,//机器人位置数据，10Hz
	ID_buff_musk										= 0x0204,//机器人增益数据，1Hz
	ID_aerial_robot_energy					= 0x0205,//空中机器人能量状态数据，10Hz，只有空中机器人主控发送
	ID_robot_hurt										= 0x0206,//伤害状态数据，伤害发生后发送
	ID_shoot_data										= 0x0207,//实时射击数据，子弹发射后发送
	ID_bullet_remaining							= 0x0208,//弹丸剩余发送数，仅空中机器人，哨兵机器人以及ICRA机器人发送，1Hz
	ID_rfid_status									= 0x0209,//机器人RFID状态，1Hz
	ID_interactive_header_data			= 0x0301 //机器人交互数据，发送方触发发送
};
//命令码枚举

enum
{
	LEN_game_state       						= 3,			//0x0001,//比赛状态数据，1Hz
	LEN_game_result 	   						= 1,			//0x0002,//比赛结果数据，比赛结束发送
	LEN_game_robot_HP       				= 32,			//0x0003,//比赛机器人血量数据，1Hz发送
	LEN_dart_status									= 3,			//0x0004,//飞镖发射状态，飞镖发射时发送
	LEN_ICRA_buff_debuff_zone_status= 3,			//0x0005,//人工智能挑战赛加成与惩罚区状态，1Hz
	LEN_event_data  								= 4,			//0x0101,//场地事件数据，1Hz
	LEN_supply_projectile_action   	= 4,			//0x0102,//场地补给站动作标识数据
	LEN_referee_warning					 		= 2,			//0x0104,//裁判警告数据，警告后发送
	LEN_dart_remaining_time					= 1,			//0x0105,//飞镖发射口倒计时，1Hz
	LEN_game_robot_state    				= 18,			//0x0201,//机器人状态数据，10Hz
	LEN_power_heat_data    					= 16,			//0x0202,//实时功率热量数据，50Hz
	LEN_game_robot_pos        			= 16,			//0x0203,//机器人位置数据，10Hz
	LEN_buff_musk										= 1,			//0x0204,//机器人增益数据，1Hz
	LEN_aerial_robot_energy					= 3,			//0x0205,//空中机器人能量状态数据，10Hz，只有空中机器人主控发送
	LEN_robot_hurt									= 1,			//0x0206,//伤害状态数据，伤害发生后发送
	LEN_shoot_data									= 6,			//0x0207,//实时射击数据，子弹发射后发送
	LEN_bullet_remaining						= 2,			//0x0208,//弹丸剩余发送数，仅空中机器人，哨兵机器人以及ICRA机器人发送，1Hz
	LEN_rfid_status									= 4				//0x0209,//机器人RFID状态，1Hz
//	LEN_interactive_header_data			= n			//0x0301 //机器人交互数据，发送方触发发送
};
//长度枚举


enum
{
	//0x200-0x02ff 	自定义命令 格式  INTERACT_ID_XXXX
	INTERACT_ID_delete_graphic 			= 0x0100,	/*客户端删除图形*/
	INTERACT_ID_draw_one_graphic 		= 0x0101,	/*客户端绘制一个图形*/
	INTERACT_ID_draw_two_graphic 		= 0x0102,	/*客户端绘制2个图形*/
	INTERACT_ID_draw_five_graphic 	= 0x0103,	/*客户端绘制5个图形*/
	INTERACT_ID_draw_seven_graphic 	= 0x0104,	/*客户端绘制7个图形*/
	INTERACT_ID_draw_char_graphic 	= 0x0105	/*客户端绘制字符图形*/
};

/*自定义交互ID*/

enum
{
	LEN_INTERACT_delete_graphic = 8,	/*删除图层*/
	LEN_INTERACT_draw_one_graphic = 21,
	LEN_INTERACT_draw_two_graphic = 36,
	LEN_INTERACT_draw_five_graphic = 81,
	LEN_INTERACT_draw_seven_graphic = 111,
	LEN_INTERACT_draw_char_graphic = 51
};

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


typedef __packed struct
{
	uint8_t game_type:4;		
	/*1:大师赛	2：单项赛	3：ICRA*/
	uint8_t game_process:4;		
	/*0：未开始比赛	1：准备阶段	2：自检阶段	3：5s倒计时	4：对战中	5：比赛结算中*/
	uint16_t stage_remain_time;
	/*当前阶段剩余时间，单位s*/
}ext_game_status_t;
/*比赛状态信息*/

typedef __packed struct
{
	uint8_t winner;
	/*0：平局	1：红方胜利	2：蓝方胜利*/
}ext_game_result_t;
/*比赛结果信息*/

typedef __packed struct
{
	/*未上场以及罚下血量均为0*/
	uint16_t red_1_robot_HP;/*红1英雄机器人*/
	uint16_t red_2_robot_HP;/*红2工程机器人*/
	uint16_t red_3_robot_HP;/*红3步兵机器人*/
	uint16_t red_4_robot_HP;/*红4步兵机器人*/ 
	uint16_t red_5_robot_HP;/*红5步兵机器人*/
	uint16_t red_7_robot_HP;/*红7哨兵机器人*/ 
	uint16_t red_outpost_HP;/*红方前哨站*/
	uint16_t red_base_HP; 	/*红方基地*/
	uint16_t blue_1_robot_HP;	/*蓝1英雄机器人*/ 
	uint16_t blue_2_robot_HP; /*蓝2工程机器人*/
	uint16_t blue_3_robot_HP; /*蓝3步兵机器人*/
	uint16_t blue_4_robot_HP; /*蓝3步兵机器人*/
	uint16_t blue_5_robot_HP; /*蓝3步兵机器人*/
	uint16_t blue_7_robot_HP; /*蓝7哨兵机器人*/ 
	uint16_t blue_outpost_HP;	/*蓝方前哨站*/
	uint16_t blue_base_HP;		/*蓝方基地*/
}ext_game_robot_HP_t;	
/*机器人血量数据*/

typedef __packed struct
{
	uint8_t dart_belong; 
	/*1：红方飞镖	2：蓝方飞镖*/
	uint16_t stage_remaining_time; 
	/*发射时的剩余比赛时间，单位s*/
}ext_dart_status_t;
/*飞镖发射状态*/

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
}ext_ICRA_buff_debuff_zone_status_t;
/*人工智能挑战赛加成与惩罚区状态*/

typedef __packed struct
{
	uint32_t event_type;
	/*
	bit0-1：己方停机坪占领状态
		0为无机器人占领
		1为空中机器人已占领但未停桨
		2为空中机器人已占领并停桨
	bit2-3：己方能量机关状态
		bit2为小能量机关，1为激活
		bit3为大能量机关，1为激活
	bit4：己方基地虚拟护盾状态
		1为基地有虚拟护盾血量
		0为无
	bit5-31：保留
	*/
}ext_event_data_t;
/*场地事件数据*/

typedef __packed struct
{
	uint8_t supply_projectile_id;
	/*补给站口ID	1为1号 2为2号*/
	uint8_t supply_robot_id;
	/*补弹机器人ID：0为无机器人补弹，12345分别为红方英雄工程步兵	101-105分别为蓝方英雄工程步兵*/
	uint8_t supply_projectile_step;
	/*出弹口开闭状态	0为关闭	1为子弹准备中	2为子弹下落*/
	uint8_t supply_projectile_num;
	/*补弹数量 50 100 150 200*/
}ext_supply_projectile_action_t;
/*补给站动作标识*/

typedef __packed struct
{
	uint8_t level;
	/*警告等级*/
	uint8_t foul_robot_id; 
	/*1级以及5级警告时，机器人ID为0	234级警告时，ID为犯规机器人ID*/
}ext_referee_warning_t;
/*裁判警告信息*/


typedef __packed struct
{
	uint8_t dart_remaining_time;
	/*15s倒计时*/
}ext_dart_remaining_time_t;
/*飞镖发射口倒计时*/

typedef __packed struct
{
	uint8_t robot_id;
	/*1-9分别为红方英雄工程步兵无人机哨兵飞镖雷达  101-109分别为蓝方*/
	uint8_t robot_level;
	/*等级*/
	uint16_t remain_HP;
	/*剩余血量*/
	uint16_t max_HP;
	/*最大血量*/
	uint16_t shooter_heat0_cooling_rate;
	uint16_t shooter_heat0_cooling_limit;
	uint16_t shooter_heat1_cooling_rate;
	uint16_t shooter_heat1_cooling_limit;
	uint8_t shooter_heat0_speed_limit;
	uint8_t shooter_heat1_speed_limit;
	uint8_t max_chassis_power;
	uint8_t mains_power_gimbal_output : 1;
	uint8_t mains_power_chassis_output : 1;
	uint8_t mains_power_shooter_output : 1;
}ext_game_robot_status_t;
/*比赛机器人状态*/

typedef __packed struct
{
	uint16_t chassis_volt; 
	//毫伏
	uint16_t chassis_current; 
	//毫安
	float chassis_power; 
	//瓦特
	uint16_t chassis_power_buffer; 
	//焦耳
	uint16_t shooter_heat0; 
	//17mm热量
	uint16_t shooter_heat1; 
	//42mm热量
	uint16_t mobile_shooter_heat2;
	//机动枪管热量
}ext_power_heat_data_t;
//功率热量信息。

typedef __packed struct
{
	float x;//m
	float y;
	float z;
	float yaw;//度
}ext_game_robot_pos_t;
/*机器人位置*/

typedef __packed struct
{
	uint8_t power_rune_buff;
	/*
	bit0：机器人血量补血状态
	bit1：枪口热量冷却加速
	bit2：机器人防御加成
	bit3：机器人攻击加成
	保留
	*/
}ext_buff_t;
/*机器人增益*/

typedef __packed struct
{
	uint16_t energy_point; /*积累的能量*/
	uint8_t attack_time;	/*可攻击事件*/
}ext_aerial_robot_energy_t;
/*空中机器人能量状态*/

typedef __packed struct
{
	uint8_t armor_id:4;
	/*血量为装甲类型时的装甲板ID*/
	uint8_t hurt_type:4;
	/*
	0x0：装甲板伤害
	0x1：模块掉线扣血
	0x2：超射速扣血
	0x3：超枪口热量扣血
	0x4：超底盘功率扣血
	0x5：装甲板撞击扣血
	*/
}ext_robot_hurt_t;
/*伤害状态*/



typedef __packed struct
{
	uint8_t bullet_type;
	/*子弹类型 1为17mm 2为42mm*/
	uint8_t bullet_freq;
	/*hz*/
	float bullet_speed;
	/*m/s*/
}ext_shoot_data_t;
/*实时射击状态*/

typedef __packed struct
{
	uint16_t bullet_remaining_num;/*子弹剩余数目*/
}ext_bullet_remaining_t;
/*子弹剩余发射数*/

typedef __packed struct
{
	uint32_t rfid_status;
	/*
	bit0：基地增益点RFID状态
	bit1：高低增益点RFID状态
	bit2：能量机关激活点RFID状态
	bit3：飞坡增益点RFID状态
	bit4：前哨岗增益点RFID增益点
	bit5：资源到增益点RFID增益点
	bit6：补血点增益点RFID状态
	bit7：工程机器人补血卡RFID状态
	bit8-25：保留
	bit26-31：ICRA状态
	状态不完全代表对应的增益或处罚状态，例如敌方已占领的高地增益点，不能获取对应的增益效果。
	*/
}ext_rfid_status_t;
/*机器人RFID状态*/

typedef __packed struct
{
 uint16_t data_cmd_id;//命令ID
 uint16_t send_ID;		//发送这ID
 uint16_t receiver_ID;	//接收者ID
}ext_student_interactive_header_data_t;
/*交互数据的数据结构体*/


typedef __packed struct
{
	uint8_t data[113];		/*数据包最大为 128-6-9=113*/
}robot_interactive_data_t;
/*交互数据包*/

typedef __packed struct
{
	uint8_t operate_type; 
	/*
	0：空操作
	1：删除图层
	2：删除所有
	*/
	uint8_t layer;
	/*
	图层数  0-9
	*/
}ext_client_custom_graphic_delete_t;
/*客户端删除图形*/

typedef __packed struct
{ 
	uint8_t graphic_name[3]; 
	/*图形名，作为客户端的索引*/
	uint32_t operate_tpye:3; 
	/*
	0：空操作
	1：增加
	2：修改
	3：删除
	*/
	uint32_t graphic_tpye:3;
	/*
	0直线
	1矩形
	2整圆
	3椭圆
	4圆弧
	5浮点数
	6整形数
	7字符
	*/
	uint32_t layer:4; 
	/*图层数 0-9*/
	uint32_t color:4; 
	/*0红蓝 1黄色 2绿色 3橙色 4紫红色 5粉色 6青色 7黑色 8白色*/
	uint32_t start_angle:9;
	uint32_t end_angle:9;
	uint32_t width:10; 
	uint32_t start_x:11; 
	uint32_t start_y:11; 
	uint32_t radius:10; 
	uint32_t end_x:11; 
	uint32_t end_y:11; 
	/*后面根据图形类型的定义不同
	类型 	start_angle 	end_angle 		width 	start_x 	start_y 	radius 	end_x 	end_y
	直线  	空     					空 		 		 线宽  		起点x  	 起点y     空   		终点x  	终点y
	矩形		空							空				 线宽			起点x  	 起点y     空   		对角x  	对角y
	正圆		空							空				 线宽			圆心x  	 圆心y     半径   	空  		空
	椭圆		空							空				 线宽			圆心x  	 圆心y     空   		x半轴  	y半轴
	圆弧		起始角度					终止角度		 线宽			圆心x  	 圆心y     空   		x半轴 	y半轴
	浮点		字体大小					有效位数		 线宽			起点x  	 起点y     32bit浮点数--float
	整形		字体大小					空				 线宽			起点x  	 起点y     32bit整形数--int32_t
	字符		字体大小					字符长度		 线宽			起点x  	 起点y     空   		空  		空
	*/
}graphic_data_struct_t;
/*绘制图形*/


typedef __packed struct
{
	graphic_data_struct_t grapic_data_struct;
}ext_client_custom_graphic_single_t;
/*客户端绘制一个图形*/

typedef __packed struct
{
	graphic_data_struct_t grapic_data_struct[2];
}ext_client_custom_graphic_double_t;
/*客户端绘制两个图形*/


typedef __packed struct
{
	graphic_data_struct_t grapic_data_struct[5];
}ext_client_custom_graphic_five_t;
/*客户端绘制五个图形*/

typedef __packed struct
{
	graphic_data_struct_t grapic_data_struct[7];
}ext_client_custom_graphic_seven_t;
/*客户端绘制七个图形*/

typedef __packed struct
{
	graphic_data_struct_t grapic_data_struct;
	uint8_t data[30];
}ext_client_custom_character_t;
/*客户端绘制字符*/

//      divind line      */

							
#endif


