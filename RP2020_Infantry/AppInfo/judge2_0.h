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

//����ϵͳ����

#define JUDGE_BUF_LENGTH 200
	//����ϵͳ����������
#define JUDGE_FRAME_HEADER  0xA5
//֡ͷ



enum
{ 
	ID_game_state       						= 0x0001,//����״̬���ݣ�1Hz
	ID_game_result 	   							= 0x0002,//����������ݣ�������������
	ID_game_robot_HP       					= 0x0003,//����������Ѫ�����ݣ�1Hz����
	ID_dart_status									= 0x0004,//���ڷ���״̬�����ڷ���ʱ����
	ID_ICRA_buff_debuff_zone_status = 0x0005,//�˹�������ս���ӳ���ͷ���״̬��1Hz
	ID_event_data  									= 0x0101,//�����¼����ݣ�1Hz
	ID_supply_projectile_action   	= 0x0102,//���ز���վ������ʶ����
	ID_referee_warning					 		= 0x0104,//���о������ݣ��������
	ID_dart_remaining_time					= 0x0105,//���ڷ���ڵ���ʱ��1Hz
	ID_game_robot_state    					= 0x0201,//������״̬���ݣ�10Hz
	ID_power_heat_data    					= 0x0202,//ʵʱ�����������ݣ�50Hz
	ID_game_robot_pos        				= 0x0203,//������λ�����ݣ�10Hz
	ID_buff_musk										= 0x0204,//�������������ݣ�1Hz
	ID_aerial_robot_energy					= 0x0205,//���л���������״̬���ݣ�10Hz��ֻ�п��л��������ط���
	ID_robot_hurt										= 0x0206,//�˺�״̬���ݣ��˺���������
	ID_shoot_data										= 0x0207,//ʵʱ������ݣ��ӵ��������
	ID_bullet_remaining							= 0x0208,//����ʣ�෢�����������л����ˣ��ڱ��������Լ�ICRA�����˷��ͣ�1Hz
	ID_rfid_status									= 0x0209,//������RFID״̬��1Hz
	ID_interactive_header_data			= 0x0301 //�����˽������ݣ����ͷ���������
};
//������ö��

enum
{
	LEN_game_state       						= 3,			//0x0001,//����״̬���ݣ�1Hz
	LEN_game_result 	   						= 1,			//0x0002,//����������ݣ�������������
	LEN_game_robot_HP       				= 32,			//0x0003,//����������Ѫ�����ݣ�1Hz����
	LEN_dart_status									= 3,			//0x0004,//���ڷ���״̬�����ڷ���ʱ����
	LEN_ICRA_buff_debuff_zone_status= 3,			//0x0005,//�˹�������ս���ӳ���ͷ���״̬��1Hz
	LEN_event_data  								= 4,			//0x0101,//�����¼����ݣ�1Hz
	LEN_supply_projectile_action   	= 4,			//0x0102,//���ز���վ������ʶ����
	LEN_referee_warning					 		= 2,			//0x0104,//���о������ݣ��������
	LEN_dart_remaining_time					= 1,			//0x0105,//���ڷ���ڵ���ʱ��1Hz
	LEN_game_robot_state    				= 18,			//0x0201,//������״̬���ݣ�10Hz
	LEN_power_heat_data    					= 16,			//0x0202,//ʵʱ�����������ݣ�50Hz
	LEN_game_robot_pos        			= 16,			//0x0203,//������λ�����ݣ�10Hz
	LEN_buff_musk										= 1,			//0x0204,//�������������ݣ�1Hz
	LEN_aerial_robot_energy					= 3,			//0x0205,//���л���������״̬���ݣ�10Hz��ֻ�п��л��������ط���
	LEN_robot_hurt									= 1,			//0x0206,//�˺�״̬���ݣ��˺���������
	LEN_shoot_data									= 6,			//0x0207,//ʵʱ������ݣ��ӵ��������
	LEN_bullet_remaining						= 2,			//0x0208,//����ʣ�෢�����������л����ˣ��ڱ��������Լ�ICRA�����˷��ͣ�1Hz
	LEN_rfid_status									= 4				//0x0209,//������RFID״̬��1Hz
//	LEN_interactive_header_data			= n			//0x0301 //�����˽������ݣ����ͷ���������
};
//����ö��


enum
{
	//0x200-0x02ff 	�Զ������� ��ʽ  INTERACT_ID_XXXX
	INTERACT_ID_delete_graphic 			= 0x0100,	/*�ͻ���ɾ��ͼ��*/
	INTERACT_ID_draw_one_graphic 		= 0x0101,	/*�ͻ��˻���һ��ͼ��*/
	INTERACT_ID_draw_two_graphic 		= 0x0102,	/*�ͻ��˻���2��ͼ��*/
	INTERACT_ID_draw_five_graphic 	= 0x0103,	/*�ͻ��˻���5��ͼ��*/
	INTERACT_ID_draw_seven_graphic 	= 0x0104,	/*�ͻ��˻���7��ͼ��*/
	INTERACT_ID_draw_char_graphic 	= 0x0105	/*�ͻ��˻����ַ�ͼ��*/
};

/*�Զ��彻��ID*/

enum
{
	LEN_INTERACT_delete_graphic = 8,	/*ɾ��ͼ��*/
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
	/*�췽������ID*/
	BLUE_HERO = 101,
	BLUE_ENGINE = 102,
	BLUE_INFANTRY_1 = 103,
	BLUE_INFANTRY_2 = 104,
	BLUE_INFANTRY_3 = 105,
	BLUE_AERIAL = 106,
	BLUE_SENTRY = 107,
	/*����������ID*/
	RED_HERO_CLIENT = 0x101,
	RED_ENGINE_CLIENT = 0x102,
	RED_INFANTRY_CLIENT_1 = 0x103,
	RED_INFANTRY_CLIENT_2 = 0x104,
	RED_INFANTRY_CLIENT_3 = 0x105,
	RED_AERIAL_CLIENT = 0x106,
	/*�췽�����ֿͻ���ID*/
	BLUE_HERO_CLIENT = 0x165,
	BLUE_ENGINE_CLIENT = 0x166,
	BLUE_INFANTRY_CLIENT_1 = 0x167,
	BLUE_INFANTRY_CLIENT_2 = 0x168,
	BLUE_INFANTRY_CLIENT_3 = 0x169,
	BLUE_AERIAL_CLIENT = 0x16A
	/*���������ֿͻ���ID*/
}Interact_Target;
/*�����˼����ͻ��˵ķ��ͽ���ID��Ϣ*/


typedef enum
{
	RED_BLUE = 0,	
	YELLOW = 1,
	GREEN = 2,
	ORANGE = 3,
	FUCHSIA = 4,	/*�Ϻ�ɫ*/
	PINK = 5,
	CYAN_BLUE = 6,	/*��ɫ*/
	BLACK = 7,
	WHITE = 8
}Graphic_Color;
/*ͼ����ɫ����*/


typedef enum
{
	LINE = 0,
	RECTANGLE = 1,
	CIRCLE = 2,
	OVAL = 3, /*��Բ*/
	ARC = 4, /*Բ��*/
	FLOAT = 5, 
	INT = 6,
	CHAR = 7
}Graphic_Type;
/*ͼ������*/

typedef enum
{
	NONE = 0,
	ADD = 1,	/*����ͼ��*/
	MODIFY = 2,		/*�޸�ͼ��*/
	DELETE = 3		/*ɾ��ͼ��*/
}Graphic_Operate;
/*ͼ�����*/
/*

����ȡ����ʱ��
����ȡѪ����Ϣ -- ���������ж��Ƿ���У��Լ����غ�ǰ��վ��Ѫ��״̬
�����ڷ���ʱ��&&����ڵĵ���ʱ -- ����������������ģʽ�ͽ�������ģʽ
�������¼���Ϣ -- ��������Ӱ�����ͷ��صľ���
������������Ϣ -- ���������������͹���
��ǹ��λ�� -- �ο�
��buff -- Ӱ�������ز���
������ٶȺ�ʣ���ӵ��� -- Ӱ��������
*/


typedef __packed struct
{
	uint8_t game_type:4;		
	/*1:��ʦ��	2��������	3��ICRA*/
	uint8_t game_process:4;		
	/*0��δ��ʼ����	1��׼���׶�	2���Լ�׶�	3��5s����ʱ	4����ս��	5������������*/
	uint16_t stage_remain_time;
	/*��ǰ�׶�ʣ��ʱ�䣬��λs*/
}ext_game_status_t;
/*����״̬��Ϣ*/

typedef __packed struct
{
	uint8_t winner;
	/*0��ƽ��	1���췽ʤ��	2������ʤ��*/
}ext_game_result_t;
/*���������Ϣ*/

typedef __packed struct
{
	/*δ�ϳ��Լ�����Ѫ����Ϊ0*/
	uint16_t red_1_robot_HP;/*��1Ӣ�ۻ�����*/
	uint16_t red_2_robot_HP;/*��2���̻�����*/
	uint16_t red_3_robot_HP;/*��3����������*/
	uint16_t red_4_robot_HP;/*��4����������*/ 
	uint16_t red_5_robot_HP;/*��5����������*/
	uint16_t red_7_robot_HP;/*��7�ڱ�������*/ 
	uint16_t red_outpost_HP;/*�췽ǰ��վ*/
	uint16_t red_base_HP; 	/*�췽����*/
	uint16_t blue_1_robot_HP;	/*��1Ӣ�ۻ�����*/ 
	uint16_t blue_2_robot_HP; /*��2���̻�����*/
	uint16_t blue_3_robot_HP; /*��3����������*/
	uint16_t blue_4_robot_HP; /*��3����������*/
	uint16_t blue_5_robot_HP; /*��3����������*/
	uint16_t blue_7_robot_HP; /*��7�ڱ�������*/ 
	uint16_t blue_outpost_HP;	/*����ǰ��վ*/
	uint16_t blue_base_HP;		/*��������*/
}ext_game_robot_HP_t;	
/*������Ѫ������*/

typedef __packed struct
{
	uint8_t dart_belong; 
	/*1���췽����	2����������*/
	uint16_t stage_remaining_time; 
	/*����ʱ��ʣ�����ʱ�䣬��λs*/
}ext_dart_status_t;
/*���ڷ���״̬*/

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
/*�˹�������ս���ӳ���ͷ���״̬*/

typedef __packed struct
{
	uint32_t event_type;
	/*
	bit0-1������ͣ��ƺռ��״̬
		0Ϊ�޻�����ռ��
		1Ϊ���л�������ռ�쵫δͣ��
		2Ϊ���л�������ռ�첢ͣ��
	bit2-3��������������״̬
		bit2ΪС�������أ�1Ϊ����
		bit3Ϊ���������أ�1Ϊ����
	bit4�������������⻤��״̬
		1Ϊ���������⻤��Ѫ��
		0Ϊ��
	bit5-31������
	*/
}ext_event_data_t;
/*�����¼�����*/

typedef __packed struct
{
	uint8_t supply_projectile_id;
	/*����վ��ID	1Ϊ1�� 2Ϊ2��*/
	uint8_t supply_robot_id;
	/*����������ID��0Ϊ�޻����˲�����12345�ֱ�Ϊ�췽Ӣ�۹��̲���	101-105�ֱ�Ϊ����Ӣ�۹��̲���*/
	uint8_t supply_projectile_step;
	/*�����ڿ���״̬	0Ϊ�ر�	1Ϊ�ӵ�׼����	2Ϊ�ӵ�����*/
	uint8_t supply_projectile_num;
	/*�������� 50 100 150 200*/
}ext_supply_projectile_action_t;
/*����վ������ʶ*/

typedef __packed struct
{
	uint8_t level;
	/*����ȼ�*/
	uint8_t foul_robot_id; 
	/*1���Լ�5������ʱ��������IDΪ0	234������ʱ��IDΪ���������ID*/
}ext_referee_warning_t;
/*���о�����Ϣ*/


typedef __packed struct
{
	uint8_t dart_remaining_time;
	/*15s����ʱ*/
}ext_dart_remaining_time_t;
/*���ڷ���ڵ���ʱ*/

typedef __packed struct
{
	uint8_t robot_id;
	/*1-9�ֱ�Ϊ�췽Ӣ�۹��̲������˻��ڱ������״�  101-109�ֱ�Ϊ����*/
	uint8_t robot_level;
	/*�ȼ�*/
	uint16_t remain_HP;
	/*ʣ��Ѫ��*/
	uint16_t max_HP;
	/*���Ѫ��*/
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
/*����������״̬*/

typedef __packed struct
{
	uint16_t chassis_volt; 
	//����
	uint16_t chassis_current; 
	//����
	float chassis_power; 
	//����
	uint16_t chassis_power_buffer; 
	//����
	uint16_t shooter_heat0; 
	//17mm����
	uint16_t shooter_heat1; 
	//42mm����
	uint16_t mobile_shooter_heat2;
	//����ǹ������
}ext_power_heat_data_t;
//����������Ϣ��

typedef __packed struct
{
	float x;//m
	float y;
	float z;
	float yaw;//��
}ext_game_robot_pos_t;
/*������λ��*/

typedef __packed struct
{
	uint8_t power_rune_buff;
	/*
	bit0��������Ѫ����Ѫ״̬
	bit1��ǹ��������ȴ����
	bit2�������˷����ӳ�
	bit3�������˹����ӳ�
	����
	*/
}ext_buff_t;
/*����������*/

typedef __packed struct
{
	uint16_t energy_point; /*���۵�����*/
	uint8_t attack_time;	/*�ɹ����¼�*/
}ext_aerial_robot_energy_t;
/*���л���������״̬*/

typedef __packed struct
{
	uint8_t armor_id:4;
	/*Ѫ��Ϊװ������ʱ��װ�װ�ID*/
	uint8_t hurt_type:4;
	/*
	0x0��װ�װ��˺�
	0x1��ģ����߿�Ѫ
	0x2�������ٿ�Ѫ
	0x3����ǹ��������Ѫ
	0x4�������̹��ʿ�Ѫ
	0x5��װ�װ�ײ����Ѫ
	*/
}ext_robot_hurt_t;
/*�˺�״̬*/



typedef __packed struct
{
	uint8_t bullet_type;
	/*�ӵ����� 1Ϊ17mm 2Ϊ42mm*/
	uint8_t bullet_freq;
	/*hz*/
	float bullet_speed;
	/*m/s*/
}ext_shoot_data_t;
/*ʵʱ���״̬*/

typedef __packed struct
{
	uint16_t bullet_remaining_num;/*�ӵ�ʣ����Ŀ*/
}ext_bullet_remaining_t;
/*�ӵ�ʣ�෢����*/

typedef __packed struct
{
	uint32_t rfid_status;
	/*
	bit0�����������RFID״̬
	bit1���ߵ������RFID״̬
	bit2���������ؼ����RFID״̬
	bit3�����������RFID״̬
	bit4��ǰ�ڸ������RFID�����
	bit5����Դ�������RFID�����
	bit6����Ѫ�������RFID״̬
	bit7�����̻����˲�Ѫ��RFID״̬
	bit8-25������
	bit26-31��ICRA״̬
	״̬����ȫ�����Ӧ������򴦷�״̬������з���ռ��ĸߵ�����㣬���ܻ�ȡ��Ӧ������Ч����
	*/
}ext_rfid_status_t;
/*������RFID״̬*/

typedef __packed struct
{
 uint16_t data_cmd_id;//����ID
 uint16_t send_ID;		//������ID
 uint16_t receiver_ID;	//������ID
}ext_student_interactive_header_data_t;
/*�������ݵ����ݽṹ��*/


typedef __packed struct
{
	uint8_t data[113];		/*���ݰ����Ϊ 128-6-9=113*/
}robot_interactive_data_t;
/*�������ݰ�*/

typedef __packed struct
{
	uint8_t operate_type; 
	/*
	0���ղ���
	1��ɾ��ͼ��
	2��ɾ������
	*/
	uint8_t layer;
	/*
	ͼ����  0-9
	*/
}ext_client_custom_graphic_delete_t;
/*�ͻ���ɾ��ͼ��*/

typedef __packed struct
{ 
	uint8_t graphic_name[3]; 
	/*ͼ��������Ϊ�ͻ��˵�����*/
	uint32_t operate_tpye:3; 
	/*
	0���ղ���
	1������
	2���޸�
	3��ɾ��
	*/
	uint32_t graphic_tpye:3;
	/*
	0ֱ��
	1����
	2��Բ
	3��Բ
	4Բ��
	5������
	6������
	7�ַ�
	*/
	uint32_t layer:4; 
	/*ͼ���� 0-9*/
	uint32_t color:4; 
	/*0���� 1��ɫ 2��ɫ 3��ɫ 4�Ϻ�ɫ 5��ɫ 6��ɫ 7��ɫ 8��ɫ*/
	uint32_t start_angle:9;
	uint32_t end_angle:9;
	uint32_t width:10; 
	uint32_t start_x:11; 
	uint32_t start_y:11; 
	uint32_t radius:10; 
	uint32_t end_x:11; 
	uint32_t end_y:11; 
	/*�������ͼ�����͵Ķ��岻ͬ
	���� 	start_angle 	end_angle 		width 	start_x 	start_y 	radius 	end_x 	end_y
	ֱ��  	��     					�� 		 		 �߿�  		���x  	 ���y     ��   		�յ�x  	�յ�y
	����		��							��				 �߿�			���x  	 ���y     ��   		�Խ�x  	�Խ�y
	��Բ		��							��				 �߿�			Բ��x  	 Բ��y     �뾶   	��  		��
	��Բ		��							��				 �߿�			Բ��x  	 Բ��y     ��   		x����  	y����
	Բ��		��ʼ�Ƕ�					��ֹ�Ƕ�		 �߿�			Բ��x  	 Բ��y     ��   		x���� 	y����
	����		�����С					��Чλ��		 �߿�			���x  	 ���y     32bit������--float
	����		�����С					��				 �߿�			���x  	 ���y     32bit������--int32_t
	�ַ�		�����С					�ַ�����		 �߿�			���x  	 ���y     ��   		��  		��
	*/
}graphic_data_struct_t;
/*����ͼ��*/


typedef __packed struct
{
	graphic_data_struct_t grapic_data_struct;
}ext_client_custom_graphic_single_t;
/*�ͻ��˻���һ��ͼ��*/

typedef __packed struct
{
	graphic_data_struct_t grapic_data_struct[2];
}ext_client_custom_graphic_double_t;
/*�ͻ��˻�������ͼ��*/


typedef __packed struct
{
	graphic_data_struct_t grapic_data_struct[5];
}ext_client_custom_graphic_five_t;
/*�ͻ��˻������ͼ��*/

typedef __packed struct
{
	graphic_data_struct_t grapic_data_struct[7];
}ext_client_custom_graphic_seven_t;
/*�ͻ��˻����߸�ͼ��*/

typedef __packed struct
{
	graphic_data_struct_t grapic_data_struct;
	uint8_t data[30];
}ext_client_custom_character_t;
/*�ͻ��˻����ַ�*/

//      divind line      */

							
#endif


