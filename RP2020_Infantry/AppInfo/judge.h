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

#define JUDGE_VERSION		20

// �����˼佻�����ݶγ���
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

/*----------------------------------------------------------------------------*/
/*-----------------------------����19����ϵͳ����------------------------------*/
/*----------------------------------------------------------------------------*/
#if (JUDGE_VERSION == JUDGE_VERSION_19)

/* �Զ���֡ͷ	Byte: 5 */
typedef __packed struct
{
	uint8_t  sof;
	uint16_t data_length;
	uint8_t  seq;
	uint8_t  crc8;
} std_frame_header_t;


/* ID: 0x0001	Byte: 	3	����״̬���� */
typedef __packed struct 
{ 
	uint8_t game_type : 4;			// ��������
	uint8_t game_progress : 4;		// �����׶�
	uint16_t stage_remain_time;		// ��ǰ�׶�ʣ��ʱ��(��λ:s)
} ext_game_status_t; 


/* ID: 0x0002	Byte:	1	����������� */
typedef __packed struct 
{ 
	uint8_t winner;
} ext_game_result_t; 


/* ID: 0x0003	Byte:	28	������Ѫ���������� */
typedef __packed struct 
{ 
	uint16_t red_1_robot_HP;	// ��1Ӣ�ۻ�����Ѫ��(δ�ϳ�������Ѫ��Ϊ0)
	uint16_t red_2_robot_HP;	// ��2���̻�����Ѫ��
	uint16_t red_3_robot_HP;	// ��3����������Ѫ��
	uint16_t red_4_robot_HP;	// ��4����������Ѫ��
	uint16_t red_5_robot_HP;	// ��5����������Ѫ��
	uint16_t red_7_robot_HP;	// ��7�ڱ�������Ѫ��
	uint16_t red_base_HP;		// �췽����Ѫ��
	uint16_t blue_1_robot_HP;	// ��1Ӣ�ۻ�����Ѫ��
	uint16_t blue_2_robot_HP;	// ��2���̻�����Ѫ��
	uint16_t blue_3_robot_HP;	// ��3����������Ѫ��
	uint16_t blue_4_robot_HP;	// ��4����������Ѫ��
	uint16_t blue_5_robot_HP;	// ��5����������Ѫ��
	uint16_t blue_7_robot_HP;	// ��7�ڱ�������Ѫ��
	uint16_t blue_base_HP;		// ��������Ѫ��
} ext_game_robot_HP_t; 


/* ID: 0x0101  Byte:  4    �����¼����� */
typedef __packed struct 
{ 
	uint32_t event_type;
} ext_event_data_t; 


/* ID: 0x0102  Byte:  4    ����վ������ʶ���� */
typedef __packed struct 
{ 
	uint8_t supply_projectile_id;
	uint8_t supply_robot_id;
	uint8_t supply_projectile_step;
	uint8_t supply_projectile_num;
} ext_supply_projectile_action_t; 


/* ID: 0X0103  Byte:  3    ���󲹸�վ�����ӵ����� */
typedef __packed struct 
{ 
	uint8_t supply_projectile_id;
	uint8_t supply_robot_id;
	uint8_t supply_num;  
} ext_supply_projectile_booking_t; 


/* ID: 0X0104  Byte:  2	   ���о�����Ϣ */
typedef __packed struct
{
	uint8_t level;
	uint8_t foul_robot_id;
} ext_referee_warning_t;


/* ID: 0X0201  Byte: 15    ������״̬���� */
typedef __packed struct 
{ 
	uint8_t robot_id;   //������ID��������У�鷢��
	uint8_t robot_level;  //1һ����2������3����
	uint16_t remain_HP;  //������ʣ��Ѫ��
	uint16_t max_HP; //��������Ѫ��
	uint16_t shooter_heat0_cooling_rate;  //������ 17mm �ӵ�������ȴ�ٶ� ��λ /s
	uint16_t shooter_heat0_cooling_limit;   // ������ 17mm �ӵ���������
	uint16_t shooter_heat1_cooling_rate;   
	uint16_t shooter_heat1_cooling_limit;   
	uint8_t mains_power_gimbal_output : 1;  
	uint8_t mains_power_chassis_output : 1;  
	uint8_t mains_power_shooter_output : 1; 
} ext_game_robot_status_t; 


/* ID: 0X0202  Byte: 14    ʵʱ������������ */
typedef __packed struct 
{ 
	uint16_t chassis_volt;   	// ���������ѹ����λ��mV
	uint16_t chassis_current;	// ���������������λ��mA
	float chassis_power;   		// ˲ʱ���ʣ���λ��W
	uint16_t chassis_power_buffer;// ���̹��ʻ��壬��λ��J60������������(���¸��ݹ���������250J)
	uint16_t shooter_heat0;		// 17mmǹ������
	uint16_t shooter_heat1;  	// 42mmǹ������
} ext_power_heat_data_t; 


/* ID: 0x0203  Byte: 16    ������λ������ */
typedef __packed struct 
{   
	float x;   
	float y;   
	float z;   
	float yaw; 
} ext_game_robot_pos_t; 


/* ID: 0x0204  Byte:  1    �������������� */
typedef __packed struct 
{ 
	uint8_t power_rune_buff; 
} ext_buff_t; 


/* ID: 0x0205  Byte:  3    ���л���������״̬���� */
typedef __packed struct 
{ 
	uint8_t energy_point;
	uint16_t attack_time; // �ֲ����涨���uint8_t
} ext_aerial_robot_energy_t; 


/* ID: 0x0206  Byte:  1    �˺�״̬���� */
typedef __packed struct 
{ 
	uint8_t armor_id : 4; 	// װ���˺�ʱ����װ��ID
	uint8_t hurt_type : 4; 	// 0x0װ���˺� 0x1ģ����� 0x2������ 0x3������ 0x4������ 0x5ײ��
} ext_robot_hurt_t; 


/* ID: 0x0207  Byte:  6    ʵʱ������� */
typedef __packed struct 
{ 
	uint8_t bullet_type;   
	uint8_t bullet_freq;   
	float bullet_speed;  
} ext_shoot_data_t; 


/* ID: 0x0208  Byte:  2    �ӵ�ʣ�෢�������� */
typedef __packed struct
{
	uint16_t bullet_remaining_num;
} ext_bullet_remaining_t;


/**
 *	-----------------------------------
 *	# �����˼佻��������ʱ������
 *	-----------------------------------
 */

/* 
	�������ݣ�����һ��ͳһ�����ݶ�ͷ�ṹ��
	���������� ID���������Լ������ߵ� ID ���������ݶΣ�
	�����������ݵİ��ܹ������Ϊ 128 ���ֽڣ�
	��ȥ frame_header,cmd_id,frame_tail �Լ����ݶ�ͷ�ṹ�� 6 ���ֽڣ�
	�ʶ����͵��������ݶ����Ϊ 113��
	������������ 0x0301 �İ�����Ƶ��Ϊ 10Hz��

	������ ID��
	1��Ӣ��(��)��
	2������(��)��
	3/4/5������(��)��
	6������(��)��
	7���ڱ�(��)��
	11��Ӣ��(��)��
	12������(��)��
	13/14/15������(��)��
	16������(��)��
	17���ڱ�(��)�� 
	�ͻ��� ID�� 
	0x0101 ΪӢ�۲����ֿͻ���( ��) ��
	0x0102 �����̲����ֿͻ��� ((�� )��
	0x0103/0x0104/0x0105�����������ֿͻ���(��)��
	0x0106�����в����ֿͻ���((��)�� 
	0x0111��Ӣ�۲����ֿͻ���(��)��
	0x0112�����̲����ֿͻ���(��)��
	0x0113/0x0114/0x0115�������ֿͻ��˲���(��)��
	0x0116�����в����ֿͻ���(��)�� 
*/
/* �������ݽ�����Ϣ��0x0301  */
typedef __packed struct 
{ 
	uint16_t data_cmd_id;    
	uint16_t send_ID;    
	uint16_t receiver_ID; 
} ext_student_interactive_header_data_t; 


/* 
	�ͻ��� �ͻ����Զ������ݣ�cmd_id:0x0301������ ID:0xD180
	����Ƶ�ʣ����� 10Hz


	1.	�ͻ��� �ͻ����Զ������ݣ�cmd_id:0x0301������ ID:0xD180������Ƶ�ʣ����� 10Hz 
	�ֽ�ƫ���� 	��С 	˵�� 				��ע 
	0 			2 		���ݵ����� ID 		0xD180 
	2 			2 		���ߵ� ID 			��ҪУ�鷢���߻����˵� ID ��ȷ�� 
	4 			2 		�ͻ��˵� ID 		ֻ��Ϊ�����߻����˶�Ӧ�Ŀͻ��� 
	6 			4 		�Զ��帡������ 1 	 
	10 			4 		�Զ��帡������ 2 	 
	14 			4 		�Զ��帡������ 3 	 
	18 			1 		�Զ��� 8 λ���� 4 	 

*/
typedef __packed struct 
{ 
	float data1; 
	float data2; 
	float data3; 
	uint8_t masks; 
} client_custom_data_t;


/* 
	ѧ�������˼�ͨ�� cmd_id 0x0301������ ID:0x0200~0x02FF
	�������� �����˼�ͨ�ţ�0x0301��
	����Ƶ�ʣ����� 10Hz  

	�ֽ�ƫ���� 	��С 	˵�� 			��ע 
	0 			2 		���ݵ����� ID 	0x0200~0x02FF 
										���������� ID ��ѡȡ������ ID �����ɲ������Զ��� 
	
	2 			2 		�����ߵ� ID 	��ҪУ�鷢���ߵ� ID ��ȷ�ԣ� 
	
	4 			2 		�����ߵ� ID 	��ҪУ������ߵ� ID ��ȷ�ԣ�
										���粻�ܷ��͵��жԻ����˵�ID 
	
	6 			n 		���ݶ� 			n ��ҪС�� 113 

*/
typedef __packed struct 
{ 
	uint8_t data[10]; //���ݶ�,n��ҪС��113
} robot_interactive_data_t;

typedef struct 
{
	uint16_t frame_length;	// ��֡����(����ʱʹ��)
	uint16_t cmd_id;		// ������(����ʱʹ��)
	uint16_t err_cnt;		// ��֡��(����ʱʹ��)
	bool	 data_valid;	// ������Ч��
	bool	 hurt_data_update;	// �˺����ݸ���
	std_frame_header_t				FrameHeader;	// ֡ͷ��Ϣ
	ext_game_status_t 				GameStatus;					// 0x0001
	ext_game_result_t 				GameResult;					// 0x0002
	ext_game_robot_HP_t 			GameRobotHP;				// 0x0003
	ext_event_data_t				EventData;					// 0x0101
	ext_supply_projectile_action_t	SupplyProjectileAction;		// 0x0102
	ext_supply_projectile_booking_t SupplyProjectileBooking;	// 0x0103
	ext_referee_warning_t			RefereeWarning;				// 0x0104
	ext_game_robot_status_t			GameRobotStatus;			// 0x0201
	ext_power_heat_data_t			PowerHeatData;				// 0x0202
	ext_game_robot_pos_t			GameRobotPos;				// 0x0203
	ext_buff_t						Buff;						// 0x0204
	ext_aerial_robot_energy_t		AerialRobotEnergy;			// 0x0205
	ext_robot_hurt_t				RobotHurt;					// 0x0206
	ext_shoot_data_t				ShootData;					// 0x0207
	ext_bullet_remaining_t			BulletRemaining;			// 0x0208	
}Judge_Info_t;

/* ## Global Variables Prototypes ## -----------------------------------------*/
extern Judge_Info_t Judge;

/* API functions Prototypes --------------------------------------------------*/
/* #������# ---------------------------------------------------------------------------------------------------------------------------------------*/
bool JUDGE_ReadData(uint8_t *rxBuf);

/* #��Ϣ��# ---------------------------------------------------------------------------------------------------------------------------------------*/
bool 		JUDGE_IfDataValid(void);
float 		JUDGE_fGetChassisRealPower(void);
float 		JUDGE_fGetChassisPowerBuffer(void);
uint8_t 	JUDGE_ucGetRobotLevel(void);
uint16_t 	JUDGE_usGetShooterRealHeat17(void);
uint16_t 	JUDGE_usGetShooterLimitHeat17(void);
uint16_t 	JUDGE_usGetShooterHeatCoolingRate17(void);
float 		JUDGE_fGetBulletSpeed17(void);
Color_t 	JUDGE_eGeyMyColor(void);
uint8_t 	JUDGE_eGetArmorHurt(void);

/* #Ӧ�ò�# ---------------------------------------------------------------------------------------------------------------------------------------*/
void JUDGE_ShootNumCount(void);

/* #������# ---------------------------------------------------------------------------------------------------------------------------------------*/
#endif	// Judge Version --19

/*----------------------------------------------------------------------------*/
/*-----------------------------����19����ϵͳ����------------------------------*/
/*----------------------------------------------------------------------------*/














/*----------------------------------------------------------------------------*/
/*-----------------------------����20����ϵͳ����------------------------------*/
/*----------------------------------------------------------------------------*/
#if (JUDGE_VERSION == JUDGE_VERSION_20)

/* �Զ���֡ͷ Byte: 5 */
typedef __packed struct
{
	uint8_t  sof;
	uint16_t data_length;
	uint8_t  seq;
	uint8_t  crc8;
} std_frame_header_t;


/* ID: 0x0001	Byte: 	3	����״̬���� */
typedef __packed struct 
{ 
	uint8_t game_type : 4;			// ��������
	uint8_t game_progress : 4;		// �����׶�
	uint16_t stage_remain_time;		// ��ǰ�׶�ʣ��ʱ��(��λ:s)
} ext_game_status_t; 


/* ID: 0x0002	Byte:	1	����������� */
typedef __packed struct 
{ 
	uint8_t winner;
} ext_game_result_t; 


/* ID: 0x0003	Byte:	32	������Ѫ���������� */
typedef __packed struct 
{ 
	uint16_t red_1_robot_HP;	// ��1Ӣ�ۻ�����Ѫ��(δ�ϳ�������Ѫ��Ϊ0)
	uint16_t red_2_robot_HP;	// ��2���̻�����Ѫ��
	uint16_t red_3_robot_HP;	// ��3����������Ѫ��
	uint16_t red_4_robot_HP;	// ��4����������Ѫ��
	uint16_t red_5_robot_HP;	// ��5����������Ѫ��
	uint16_t red_7_robot_HP;	// ��7�ڱ�������Ѫ��
	uint16_t red_outpost_HP;	// �췽ǰ��վѪ��
	uint16_t red_base_HP;		// �췽����Ѫ��
	uint16_t blue_1_robot_HP;	// ��1Ӣ�ۻ�����Ѫ��
	uint16_t blue_2_robot_HP;	// ��2���̻�����Ѫ��
	uint16_t blue_3_robot_HP;	// ��3����������Ѫ��
	uint16_t blue_4_robot_HP;	// ��4����������Ѫ��
	uint16_t blue_5_robot_HP;	// ��5����������Ѫ��
	uint16_t blue_7_robot_HP;	// ��7�ڱ�������Ѫ��
	uint16_t blue_outpost_HP;	// ����ǰ��վѪ��
	uint16_t blue_base_HP;		// ��������Ѫ��	
} ext_game_robot_HP_t; 


/* ID: 0x0004 	Byte:	3	���ڷ���״̬ */
typedef __packed struct
{
	uint8_t dart_belong;
	uint16_t stage_remaining_time;
} ext_dart_status_t;


/* ID: 0x0005 	Byte:	3	�˹�������ս���ӳ���ͷ���״̬ */
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


/* ID: 0x0101  Byte:  4    �����¼����� */
typedef __packed struct 
{ 
	uint32_t event_type;
} ext_event_data_t; 


/* ID: 0x0102  Byte:  4    ����վ������ʶ���� */
typedef __packed struct 
{ 
	uint8_t supply_projectile_id;
	uint8_t supply_robot_id;
	uint8_t supply_projectile_step;
	uint8_t supply_projectile_num;
} ext_supply_projectile_action_t; 


///* ID: 0X0103  Byte:  3    ���󲹸�վ�����ӵ����� */
//typedef __packed struct 
//{ 
//	uint8_t supply_projectile_id;
//	uint8_t supply_robot_id;
//	uint8_t supply_num;  
//} ext_supply_projectile_booking_t; 


/* ID: 0X0104  Byte:  2	   ���о�����Ϣ */
typedef __packed struct
{
	uint8_t level;
	uint8_t foul_robot_id;
} ext_referee_warning_t;


/* ID: 0X0105  Byte:  1	   ���ڷ���ڵ���ʱ */
typedef __packed struct
{
	uint8_t dart_remaining_time;
} ext_dart_remaining_time_t;


/* ID: 0X0201  Byte: 18    ������״̬���� */
typedef __packed struct 
{ 
	uint8_t robot_id;   					// ������ID��������У�鷢��
	uint8_t robot_level;  					// 1һ����2������3����
	uint16_t remain_HP;  					// ������ʣ��Ѫ��
	uint16_t max_HP; 						// ��������Ѫ��
	uint16_t shooter_heat0_cooling_rate;  	// ������ 17mm ǹ��ÿ����ȴֵ
	uint16_t shooter_heat0_cooling_limit;   // ������ 17mm ǹ����������
	uint16_t shooter_heat1_cooling_rate;	// ������ 42mm ǹ��ÿ����ȴֵ
	uint16_t shooter_heat1_cooling_limit;   // ������ 42mm ǹ����������
	uint8_t shooter_heat0_speed_limit;		// ������ 17mm ǹ�������ٶ�(m/s)
	uint8_t shooter_heat1_speed_limit; 		// ������ 42mm ǹ�������ٶ�(m/s)
	uint8_t max_chassis_power;				// �����������̹���(W)
	uint8_t mains_power_gimbal_output : 1;  // gimbal�����	
	uint8_t mains_power_chassis_output : 1; // chassis�����
	uint8_t mains_power_shooter_output : 1; // shooter�����
} ext_game_robot_status_t; 


/* ID: 0X0202  Byte: 16    ʵʱ������������ */
typedef __packed struct 
{ 
	uint16_t chassis_volt;   		// ���������ѹ����λ��mV
	uint16_t chassis_current;		// ���������������λ��mA
	float chassis_power;   			// ˲ʱ���ʣ���λ��W
	uint16_t chassis_power_buffer;	// ���̹��ʻ��壬��λ��60J������������(���¸��ݹ���������250J)
	uint16_t shooter_heat0;			// 17mm ǹ������
	uint16_t shooter_heat1;  		// 42mm ǹ������
	uint16_t mobile_shooter_heat2;	// ���� 17mm ǹ������
} ext_power_heat_data_t; 


/* ID: 0x0203  Byte: 16    ������λ������ */
typedef __packed struct 
{   
	float x;   
	float y;   
	float z;   
	float yaw; 
} ext_game_robot_pos_t; 


/* ID: 0x0204  Byte:  1    �������������� */
typedef __packed struct 
{ 
	uint8_t power_rune_buff; 
} ext_buff_t; 


/* ID: 0x0205  Byte:  3    ���л���������״̬���� */
typedef __packed struct 
{ 
	uint16_t energy_point;
	uint8_t attack_time;
} ext_aerial_robot_energy_t; 


/* ID: 0x0206  Byte:  1    �˺�״̬���� */
typedef __packed struct 
{ 
	uint8_t armor_id : 4; 	// װ���˺�ʱ����װ��ID
	uint8_t hurt_type : 4; 	// 0x0װ���˺� 0x1ģ����� 0x2������ 0x3������ 0x4������ 0x5ײ��
} ext_robot_hurt_t; 


/* ID: 0x0207  Byte:  6    ʵʱ������� */
typedef __packed struct 
{ 
	uint8_t bullet_type; 	// �ӵ�����(1-17mm, 2-42mm)
	uint8_t bullet_freq;  	// �ӵ���Ƶ(Hz)
	float bullet_speed;		// �ӵ�����(m/s)
} ext_shoot_data_t; 


/* ID: 0x0208  Byte:  2    �ӵ�ʣ�෢�������� */
typedef __packed struct
{
	uint16_t bullet_remaining_num;
} ext_bullet_remaining_t;


/* ID: 0x0209  Byte:  4 	������RFID״̬ */
typedef __packed struct
{
	uint32_t rfid_status;
} ext_rfid_status_t;

/**
 *	-----------------------------------
 *	# �����˼佻������
 *	-----------------------------------
 */

/* 
	�������ݣ�����һ��ͳһ�����ݶ�ͷ�ṹ��
	���������� ID���������Լ������ߵ� ID ���������ݶΣ�
	�����������ݵİ��ܹ������Ϊ 128 ���ֽڣ�
	��ȥ frame_header,cmd_id,frame_tail ��9���ֽ��Լ����ݶ�ͷ�ṹ�� 6 ���ֽڣ�
	�ʶ����͵��������ݶ����Ϊ 113��
	������������ 0x0301 �İ�����Ƶ��Ϊ 10Hz��

	������ ID��
	1��Ӣ��(��)��
	2������(��)��
	3/4/5������(��)��
	6������(��)��
	7���ڱ�(��)��

	101��Ӣ��(��)��
	102������(��)��
	103/104/105������(��)��
	106������(��)��
	107���ڱ�(��)��

	�ͻ��� ID�� 
	0x0101 ΪӢ�۲����ֿͻ���( ��) ��
	0x0102 �����̲����ֿͻ��� ((�� )��
	0x0103/0x0104/0x0105�����������ֿͻ���(��)��
	0x0106�����в����ֿͻ���((��)�� 

	0x0165��Ӣ�۲����ֿͻ���(��)��
	0x0166�����̲����ֿͻ���(��)��
	0x0167/0x0168/0x0169�����������ֿͻ���(��)��
	0x016A�����в����ֿͻ���(��)�� 
*/
/* �������ݽ�����Ϣ��0x0301  */
typedef __packed struct 
{ 
	uint16_t data_cmd_id;    
	uint16_t send_ID;    
	uint16_t receiver_ID; 
} ext_student_interactive_header_data_t; 


/* 
	ѧ�������˼�ͨ�� cmd_id 0x0301������ ID:0x0200~0x02FF
	�������� �����˼�ͨ�ţ�0x0301��
	����Ƶ�ʣ����� 10Hz  

	�ֽ�ƫ���� 	��С 	˵�� 			��ע 
	0 			2 		���ݵ����� ID 	0x0200~0x02FF 
										���������� ID ��ѡȡ������ ID �����ɲ������Զ��� 
	
	2 			2 		�����ߵ� ID 		��ҪУ�鷢���ߵ� ID ��ȷ�ԣ� 
	
	4 			2 		�����ߵ� ID 		��ҪУ������ߵ� ID ��ȷ�ԣ�
										���粻�ܷ��͵��жԻ����˵�ID 
	
	6 			n 		���ݶ� 			n ��ҪС�� 113 

*/
/*
			�������ÿ���ֽ���	�������ÿ���ֽ���
	����		3720Bytes/s			3720Bytes/s
*/
typedef __packed struct 
{ 
	uint8_t data[INTERACT_DATA_LEN]; //���ݶ�,n��ҪС��113
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
	std_frame_header_t						FrameHeader;	// ֡ͷ
	uint16_t 								CmdId;			// ������
	ext_student_interactive_header_data_t	DataFrameHeader;// ���ݶ�ͷ�ṹ
	ext_client_custom_graphic_seven_t		ClientData;		// ����
	uint16_t								FrameTail;		// ֡β
} Judge_Client_Data_t;

typedef __packed struct
{
	std_frame_header_t						FrameHeader;	// ֡ͷ
	uint16_t 								CmdId;			// ������
	ext_student_interactive_header_data_t	DataFrameHeader;// ���ݶ�ͷ�ṹ
	robot_interactive_data_t				InteractData;	// ����
	uint16_t								FrameTail;		// ֡β	
} Judge_Interact_Data_t;

typedef struct 
{
	uint16_t frame_length;	// ��֡����(����ʱʹ��)
	uint16_t cmd_id;		// ������(����ʱʹ��)
	uint16_t err_cnt;		// ��֡��(����ʱʹ��)
	bool	 data_valid;	// ������Ч��
	uint16_t self_client_id;// �����߻����˶�Ӧ�Ŀͻ���ID
	bool	 hurt_data_update;	// �˺����ݸ���
	bool	dart_data_update;	// �������ݸ���
	bool	supply_data_update;	// ����վ���ݸ���
	std_frame_header_t				FrameHeader;				// ֡ͷ��Ϣ
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
/*---------------------------��������ϵͳ���ݽṹ����---------------------------*/
/*----------------------------------------------------------------------------*/

/* ## Global Variables Prototypes ## -----------------------------------------*/
extern Judge_Info_t Judge;

/* API functions Prototypes --------------------------------------------------*/
/* #������# ---------------------------------------------------------------------------------------------------------------------------------------*/
bool JUDGE_ReadData(uint8_t *rxBuf);

/* #��Ϣ��# ---------------------------------------------------------------------------------------------------------------------------------------*/
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

/* #Ӧ�ò�# ---------------------------------------------------------------------------------------------------------------------------------------*/
void JUDGE_ShootNumCount(void);

/* #������# ---------------------------------------------------------------------------------------------------------------------------------------*/
#endif	// Judge Version --20
/*----------------------------------------------------------------------------*/
/*-----------------------------����20����ϵͳ����------------------------------*/
/*----------------------------------------------------------------------------*/

#endif	// __JUDGE_H

