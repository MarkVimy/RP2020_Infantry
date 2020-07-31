/**
 * @file        can.c
 * @author      MaWeiming
 * @Version     V1.0
 * @date        22-September-2019
 * @brief       This file includes the CAN driver external functions 
 * 							(based on ST Peripherals Libaray Keil.STM32F4xx_DFP.2.13.0)
 * @Version		V1.1(10-October-2019)
 *				1. ����CAN2
 */

/**
 *	# CAN
 *	@brief
 *		���Ե�ƽ - �߼�0
 *		���Ե�ƽ - �߼�1
 *
 *	@note
 *	������֡
 *				֡��ʼ �ٲö� 	���ƶ� 	���ݶ� 	CRC�� 	ACK�� ֡����
 *	��׼֡		1		11		1+1+4	0-64	15+1	4		7
 *	��չ֡		1		29		1+1+4	
 *		# ֡��ʼ
 *			1��λ�����Ե�ƽ
 *
 *		# �ٲö�
 *			1. ��ʾ�������ȼ��Ķ�
 *			2. ID(��λ��ǰ����λ�ں�)
 *				����ID��ֹ��7λΪ����(���� ID = 1111111xxxxb
 *			3. RTR
 *				Զ������λ	
 *				0 - ����֡
 *				1 - Զ��֡
 *			4. SRR
 *				���Զ������֡(����Ϊ1(���Ե�ƽ))
 *			5. IDE
 *				��ʶ��ѡ��λ
 *				0 - ��׼��ʶ��
 *				1 - ��չ��ʶ��
 *			6. ��ʽ
 *				��׼ - 11λ����ID + RTR
 *				��չ - 11λ����ID + SRR + IDE
 *	
 *		# ���ƶ�
 *			1. r0, r1
 *				����λ
 *				���ͱ��������Ե�ƽ�����տ��������Ե�ƽ
 *			2. DLC
 *				���ݳ�����
 *				0 - 8(��ʾ�շ������ֽڳ���)
 *
 *		# ���ݶ�
 *			0 - 8 �ֽڣ�MSB(���λ)��ʼ���
 *		
 *		# CRC��
 *			���֡�������
 *			15λCRC˳�� + 1λCRC�綨��(�ָ�λ)
 *			���㷶Χ��֡��ʼ���ٲöΡ����ƶΡ����ݶ�
 *	
 *		# ACK��
 *			4λACK��(ACK Slot) + 1λACK�綨��
 *			1. ���͵�ԪACK��
 *				����2������λ
 *			2. ���յ�ԪACK��
 *				���յ���ȷ��Ϣ�ĵ�Ԫ��
 *				��ACK�۷�������λ��֪ͨ���͵�Ԫ���������ս�������֮Ϊ����ACK/����ACK��
 *		
 *		# ֡����
 *			7��λ�����Ե�ƽ
 *
 *	�������ٲ�
 *		1. �ȷ������Ե�ƽ����ռ����Ȩ
 *		2. ͬʱ�������Ե�ƽ�����ٲö���ѡ�����ȼ������
 *			 ��λ�Ƚϣ����Ե�ƽ��ʤ�������Ե�ƽ����̭(������һ�־���)
 *
 *	�� λʱ��
 *		1. λ����
 *			���͵�Ԫ�ڷ�ͬ�������ÿ���ӷ��͵�λ��
 *		2. λ�ֶ�
 *			1��λ = ͬ����(SS) + ����ʱ���(PTS) + ��λ�����1(PBS1) + ��λ�����2(PBS2)
 *			�������ֿ��� Time Quantum(tq) - ��Сʱ�䵥λ����
 *			λ<��<<tq>>> ��Ϊλʱ��
 *		3. λʱ��
 *			λʱ�� = 1/������
 *			ͨ���趨λʱ�򣬶����Ԫ��ͬʱ������Ҳ�������趨������
 *		4. ������
 *			������ = 1/������λʱ��
 *			������λʱ�� = 1*tq + tBS1 + tBS2
 *				tBS1 = tq * (TS1[3:0] + 1)
 *				tBS2 = tq * (TS2[2:0] + 1)
 *				tq = (BRP[9:0] + 1) * tPCLK
 *				tPCLK = APBʱ�ӵ�ʱ������
 *				BRP[9:0]��TS1[3:0]��TS2[2:0] �� CAN_BTR�Ĵ����ж���
 *	
 *				BRP[9:0] = 2, APB1 - 42Mhz
 *					tq = 3 / 42 (us) = 1/14(us)
 *				tBS1 = 9tq, tBS2 = 4tq,
 *					����λʱ�� = tq + 9tq + 4tq = 14tq
 *				  ������ = 1 / 14tq = 14/14(us) = 1MHz (�պ�ƥ������豸�����߱�����)
 *				
 */

/**
 *	# �����豸
 *	ֱ����ˢ���M3508 + ���C620
 *	1. CAN���߱�����Ϊ1Mbps
 *	2. C620֧�����ֿ��Ʒ�ʽ(��ѡһ + �ϵ��л�)
 *		 CAN / PWM(�ɽ��й̼�����)
 *	
 *		# ���C620
 *		�� SET��������		
 *			1. ��������
 *			2. ��������(<=8��)
 *			3. ���У׼
 *		�� CANͨ��Э��
 *			1. ������ձ��ĸ�ʽ(��׼֡)
 *				- ����ָ��(���� -> ���)
 *				0x200 - ���ID 1 - 4
 *				0x1FF	- ���ID 5 - 8
 *				�������ֵ��Χ�� -16384 ~ 0 ~ 16384
 *													-20A  ~ 0 ~ 20A
 *						��������ֵ�� -8192	~ 0 ~ 8192
 *													-10A		0		10A
 *				- ������ʽ
 *				0x200 + ���ID
 *				eg. ID1 -> 0x201
 *				DATA[0] - ת�ӻ�е�Ƕȸ�8λ
 *				DATA[1] - ת�ӻ�е�Ƕȵ�8λ
 *				DATA[2] - ת��ת�ٸ�8λ
 *				DATA[3] - ת��ת�ٵ�8λ
 *				DATA[4] - ת��ת�ص�����8λ
 *				DATA[5] - ת��ת�ص�����8λ
 *				DATA[6] - ����¶�
 *				DATA[7] - Null
 *	
 *				����Ƶ�� - 1Khz(RoboMaster Assistant������޸�)	
 *				ת�ӻ�е�Ƕȷ�Χ: 0 ~ 8191(0��~ 360��) 
 *										����: 0.0439453125�� = 0.044
 *				ת��ת��ֵ��λ: RPM
 *				����¶ȵ�λ����
 *			2. 
 *	
 *		# ֱ����ˢ���M3508
 *			����C620���ʵ����������
 *			��������ٱ�19.2:1
 *			λ�÷��� + �¶ȼ��
 *	
 *			1. ��������
 *			- ����C620
 *			���ѹ	24V
 *			����� 10A
 */

/**
 *	# �����豸
 *	ֱ����ˢ���M3510 + ���820R
 *	1. CAN���߱�����Ϊ1Mbps
 *
 *		# ���820R
 *		�� �̼�����		
 *			- SYSϵͳָʾ��
 *			1. �̵Ƴ���						��������RM3510���
 *			2. �̵�ÿ2����˸1��		��������RM2310���
 *			3. �̵�ÿ2����˸1��		��������RM2006���
 *			- ��ʾ��
 *			1. 1234567						RM3510���ٵ���̼�
 *			2. 7654321						RM2310���ٵ���̼�
 *			3. 1313221						RM2006���ٵ���̼� 
 *
 *		�� CANͨ��Э��
 *			1. ������ձ��ĸ�ʽ(��׼֡)
 *				- ����ָ��(���� -> ���)
 *				0x200 - ���ID 1 - 4
 *				�������ֵ��Χ�� -32768 ~ 0 ~ 32767
 *								-  A  ~ 0 ~   A
 *				����ڲ����˵������ƣ�ʵ����Ч�������뷶Χ:
 *				RM3510_V1_2_5_0.bin.enc: [ -16384  : +16384]
 *				RM2310_V1_2_5_1.bin.enc: [ -8000   :  +8000]
 *				RM2006_V1_2_5_2.bin.enc: [ -8000   :  +8000]
 *	
 *				- ������ʽ
 *				0x200 + ���ID
 *				eg. ID1 -> 0x201
 *				DATA[0] - ת�ӻ�е�Ƕȸ�8λ
 *				DATA[1] - ת�ӻ�е�Ƕȵ�8λ
 *				DATA[2] - ת��ת�ٸ�8λ
 *				DATA[3] - ת��ת�ٵ�8λ
 *				DATA[4] - Null
 *				DATA[5] - Null
 *				DATA[6] - Null
 *				DATA[7] - Null
 *
 *				����Ƶ�� - 1Khz(RoboMaster Assistant������޸�)	
 *				ת�ӻ�е�Ƕȷ�Χ: 0 ~ 8191(0��~ 360��) 
 *				����: 0.0439453125�� = 0.044
 *				ת��ת��ֵ��λ: RPM
 *
 *		�۵��У׼
 *			- ���뿪��
 * 			ON
 *			��������
 *			��������
 *			1	2	3	4
 *			1	- Bit0	�궨�豸ID
 *			2 - Bit1	�궨�豸ID
 *			3 - Bit2	�궨�豸ID
 *			4 - Bit3	�Ƿ����CAN����120���ն˵���
 *
 *			[Bit2:Bit0]
 *			000b - �������У׼ģʽ		
 *			001b - ID1
 *			010b - ID2
 *			011b - ID3
 *			100b - ID4
 *
 *		# ֱ����ˢ���M3510
 *			��������ٱ�19:1
 *			λ�÷���
 *	
 *			1. ��������
 *			- ����820R
 *			���ѹ	24V
 *			����� 	
 */

/**
 *	# ��̨�豸
 *	ֱ����ˢ���GM6020
 *	1. �ڲ������������ĸ߿ɿ���ֱ����ˢ���
 *	2. ��ת�١���Ť�ء�
 *	3. ���������ôų��������(FOC)�㷨����ϸ߾��ȵĽǶȴ�������
 *		 ��ʵ�־�ȷ�����غ�λ�ÿ��ơ�
 *	4. ����߱��쳣��ʾ�ͱ������ܣ�֧�ֶ���ͨ�ŷ�ʽ��������ƺ�ʡ������
 *	5. CAN���߱�����Ϊ1Mbps
 *
 *		# �ڲ����ɵ��
 *		�� ϵͳ״̬����		
 *			- ���ָʾ��
 *			1. �̵�ÿ��1����N��		������������ǰ������IDΪN
 *			2. �̵�����						PWMͨ������
 *			3. �̵Ƴ���						PWM�г�У׼��
 *
 *		�� CANͨ��Э��
 *			1. ������ձ��ĸ�ʽ(��׼֡)
 *				- ����ָ��(���� -> ���) - ��ѹ����
 *				0x1FF - ���ID 1 - 4
 *				0x2FF	- ���ID 5 - 7
 *				�����ѹֵ��Χ�� -30000 ~ 0 ~ 30000
 *								-24V  ~ 0 ~  24V
 *	
 *											0x1FF	0x2FF
 *				DATA[0] - ��ѹ����ֵ��8λ - 	ID1		ID5
 *				DATA[1] - ��ѹ����ֵ��8λ - 	ID1		ID5
 *				DATA[2] - ��ѹ����ֵ��8λ - 	ID2		ID6
 *				DATA[3] - ��ѹ����ֵ��8λ - 	ID2		ID6
 *				DATA[4] - ��ѹ����ֵ��8λ - 	ID3		ID7
 *				DATA[5] - ��ѹ����ֵ��8λ - 	ID3		ID7
 *				DATA[6] - ��ѹ����ֵ��8λ - 	ID4		Null
 *				DATA[7] - ��ѹ����ֵ��8λ - 	ID4		Null
 *				
 *				- ������ʽ
 *				0x204 + ���ID
 *				eg. ID1 -> 0x205
 *				DATA[0] - ת�ӻ�е�Ƕȸ�8λ
 *				DATA[1] - ת�ӻ�е�Ƕȵ�8λ
 *				DATA[2] - ת��ת�ٸ�8λ
 *				DATA[3] - ת��ת�ٵ�8λ
 *				DATA[4] - ʵ��ת�ص�����8λ
 *				DATA[5] - ʵ��ת�ص�����8λ
 *				DATA[6] - ����¶�
 *				DATA[7] - Null
 *
 *				����Ƶ�� - 1Khz(RoboMaster Assistant������޸�)	
 *				ת�ӻ�е�Ƕȷ�Χ: 0 ~ 8191(0��~ 360��) 
 *				����: 0.0439453125�� = 0.044
 *				ת��ת��ֵ��λ: RPM
 *
 *		�۵��У׼
 *			- ���뿪��
 * 			ON
 *			��������
 *			��������
 *			1	2	3	4
 *			1 - Bit0	�궨�豸ID
 *			2 - Bit1	�궨�豸ID
 *			3 - Bit2	�궨�豸ID
 *			4 - Bit3	�Ƿ����CAN����120���ն˵���
 *
 *			[Bit2:Bit0]
 *			000b - ��Ч	
 *			001b - ID1
 *			010b - ID2
 *			011b - ID3
 *			100b - ID4
 *			101b - ID5
 *			110b - ID6
 *			111b - ID7	
 *
 */
 
/**
 *	# ��������豸
 *	ֱ����ˢ���M2006 + ���C610
 *	1. ���֧��10A�ĳ�������
 *	2. ���������ôų��������(FOC)�㷨��ʵ�ֶԵ��ת�صľ�ȷ���ơ�
 *	3. ����߱��쳣��ʾ�ͱ������ܣ�CAN����ָ����ơ�
 *	4. CAN���߱�����Ϊ1Mbps
 *
 *	# ���C610
 *		�� ϵͳ״̬����		
 *			- ���ָʾ��
 *			1. �̵�ÿ��1����N��		������������ǰ������IDΪN
 *			2. �ȵƳ���						��ǰ������ڿ�������ID״̬
 *			3. �̵ƿ���						��ǰ�������У׼ģʽ
 *			4. ...
 *
 *		�� CANͨ��Э��
 *			1. ������ձ��ĸ�ʽ(��׼֡)
 *				- ����ָ��(���� -> ���) - ��ѹ����
 *				0x200 - ���ID 1 - 4
 *				0x1FF - ���ID 5 - 7
 *				�������ֵ��Χ�� -10000 ~ 0 ~ 10000
 *								-10A ~ 0 ~  	10A
 *	
 *											0x200	0x1FF
 *				DATA[0] - ���Ƶ���ֵ��8λ - 	ID1		ID5
 *				DATA[1] - ���Ƶ���ֵ��8λ - 	ID1		ID5
 *				DATA[2] - ���Ƶ���ֵ��8λ - 	ID2		ID6
 *				DATA[3] - ���Ƶ���ֵ��8λ - 	ID2		ID6
 *				DATA[4] - ���Ƶ���ֵ��8λ - 	ID3		ID7
 *				DATA[5] - ���Ƶ���ֵ��8λ - 	ID3		ID7
 *				DATA[6] - ���Ƶ���ֵ��8λ - 	ID4		ID8
 *				DATA[7] - ���Ƶ���ֵ��8λ - 	ID4		ID8
 *				
 *				- ������ʽ
 *				0x200 + ���ID
 *				eg. ID1 -> 0x201
 *				DATA[0] - ת�ӻ�е�Ƕȸ�8λ
 *				DATA[1] - ת�ӻ�е�Ƕȵ�8λ
 *				DATA[2] - ת��ת�ٸ�8λ
 *				DATA[3] - ת��ת�ٵ�8λ
 *				DATA[4] - ʵ�����ת�ظ�8λ
 *				DATA[5] - ʵ�����ת�ص�8λ
 *				DATA[6] - Null
 *				DATA[7] - Null
 *
 *				����Ƶ�� - 1Khz(RoboMaster Assistant������޸�)	
 *				ת�ӻ�е�Ƕȷ�Χ: 0 ~ 8191(0��~ 360��) 
 *				����: 0.0439453125�� = 0.044
 *				ת��ת��ֵ��λ: RPM
 *
 *		�۵��У׼
 *			- SET����
 *			
 */
/* Includes ------------------------------------------------------------------*/
#include "can.h"

#include "exti.h"
#include "led.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/**
 *	@var	CAN_DefaultParams 
 *	@brief
 */
static CAN_InitTypeDef CAN_DefaultParams = 
{
	.CAN_TTCM = DISABLE,	// ��ʱ�䴥��ͨ��ģʽ
	.CAN_ABOM = DISABLE,	// ����Զ����߹���
	.CAN_AWUM = DISABLE,	// ˯��ģʽͨ���������(���CAN->MCR��SLEEPλ)
	.CAN_NART = DISABLE,	// ��ֹ�����Զ����� ���߸������һ��CAN ��Ӱ�췢�� ��ʱ�ɸ�ΪENABLE
	.CAN_RFLM = DISABLE,	// ���Ĳ��������µĸ��Ǿɵ�
	.CAN_TXFP = ENABLE,		// ���ȼ��ɱ��ı�ʶ������
	.CAN_BS1	= CAN_BS1_9tq,
	.CAN_BS2	= CAN_BS2_4tq,
	.CAN_Mode = CAN_Mode_Normal,
	.CAN_Prescaler = 3,
	.CAN_SJW	= CAN_SJW_1tq,
};

static CAN_FilterInitTypeDef CAN_Filter_DefaultParams =
{
	.CAN_FilterNumber = 0,  					// ������0
	.CAN_FilterMode = CAN_FilterMode_IdMask,   	// ����ģʽ
	.CAN_FilterScale = CAN_FilterScale_32bit,   // 32λ��
	.CAN_FilterFIFOAssignment = 0,              // ������0������FIFO0
	.CAN_FilterActivation = ENABLE,   			// ���������
	.CAN_FilterIdHigh = 0x0000,                 // 32λID
	.CAN_FilterIdLow = 0x0000,
	.CAN_FilterMaskIdHigh = 0x0000,             // 32λMask
	.CAN_FilterMaskIdLow = 0x0000,	
};

/* ## Global variables ## ----------------------------------------------------*/
MOTOR_Info_t g_Chassis_Motor_Info[CHASSIS_MOTOR_COUNT];
MOTOR_Info_t g_Gimbal_Motor_Info[GIMBAL_MOTOR_COUNT];
MOTOR_Info_t g_Revolver_Motor_Info;

extern QueueHandle_t CAN1_Queue;
extern QueueHandle_t CAN2_Queue;
	
/* Private function prototypes -----------------------------------------------*/
static void CAN1_GPIO_Init(void);
static void CAN2_GPIO_Init(void);
static int16_t CAN_GetMotorAngle(CanRxMsg *rxMsg);
static int16_t CAN_GetMotorSpeed(CanRxMsg *rxMsg);
//static int16_t CAN_GetMotorCurrent(CanRxMsg *rxMsg);
static uint8_t CAN_GetMotorTemperature(CanRxMsg *rxMsg);
static void CalcMotorAngleSum(int16_t angle_now, int16_t angle_pre, int32_t *angle_sum);

/* Private functions ---------------------------------------------------------*/
/**
 *	@brief	CAN1 GPIO��ʼ��
 *	@note	PA11 - CAN1_RX
 *			PA12 - CAN1_TX
 */
static void CAN1_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
	// ��ʼ��GPIO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;// ���ù���
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;// �������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;// 100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
	GPIO_Init(GPIOA, &GPIO_InitStructure);// ��ʼ��PA11,PA12	
	
	// ���Ÿ���ӳ������
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_CAN1); // GPIOA11����ΪCAN1
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_CAN1); // GPIOA12����ΪCAN1	
}

/**
 *	@brief	CAN2 GPIO��ʼ��
 *	@note	PB12 - CAN2_RX
 *			PB13 - CAN2_TX
 */
static void CAN2_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
	// ��ʼ��GPIO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;// ���ù���
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;// �������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;// 100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
	GPIO_Init(GPIOB, &GPIO_InitStructure);// ��ʼ��PB12,PB13	
	
	// ���Ÿ���ӳ������
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_CAN2); // GPIOB12����ΪCAN2
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_CAN2); // GPIOB13����ΪCAN2	
}

/**
 *	@brief	��CAN�����з�������Ļ�е�Ƕ�
 */
static int16_t CAN_GetMotorAngle(CanRxMsg *rxMsg)
{
	int16_t angle;
	angle = ((int16_t)rxMsg->Data[0] << 8 | rxMsg->Data[1]);
	return angle;
}

/**
 *	@brief	��CAN�����з��������ת��ת��
 */
static int16_t CAN_GetMotorSpeed(CanRxMsg *rxMsg)
{
	int16_t speed;
	speed = ((int16_t)rxMsg->Data[2] << 8 | rxMsg->Data[3]);
	return speed;
}

/**
 *	@brief	��CAN�����з��������ʵ��ת�ص���
 */
static int16_t CAN_GetMotorCurrent(CanRxMsg *rxMsg)
{
	int16_t current;
	current = ((int16_t)rxMsg->Data[4] << 8 | rxMsg->Data[5]);
	return current;
}

/**
 *	@brief	��CAN�����з���������¶�
 */
static uint8_t CAN_GetMotorTemperature(CanRxMsg *rxMsg)
{
	uint8_t temperature;
	temperature = (rxMsg->Data[6]);
	return temperature;
}

/**
 *	@brief	��������е�Ƕȵ��ۼ�ֵ
 */
static void CalcMotorAngleSum(int16_t angle_now, int16_t angle_pre, int32_t *angle_sum)
{
	if(abs(angle_now - angle_pre) > 4095) {	// ת����Ȧ
		if(angle_now < angle_pre) {	// ת����Ȧ + ת�����(������)
			*angle_sum += 8191 - angle_pre + angle_now;
		} else {	// ת����Ȧ + ת�����(������)
			*angle_sum -= 8191 - angle_now + angle_pre;
		}
	} else {
		*angle_sum += angle_now - angle_pre;
	}
}

/* API functions -------------------------------------------------------------*/
/**
 *	@brief	CAN����Ĭ�����ó�ʼ��
 *	@param	(CAN_InitTypeDef*)CAN_InitStructure
 */
void CAN_ParamsInit(CAN_InitTypeDef* CAN_InitStructure)
{
	CAN_InitStructure->CAN_ABOM = CAN_DefaultParams.CAN_ABOM;
	CAN_InitStructure->CAN_AWUM = CAN_DefaultParams.CAN_AWUM;
	CAN_InitStructure->CAN_BS1  = CAN_DefaultParams.CAN_BS1;
	CAN_InitStructure->CAN_BS2  = CAN_DefaultParams.CAN_BS2;
	CAN_InitStructure->CAN_Mode = CAN_DefaultParams.CAN_Mode;
	CAN_InitStructure->CAN_NART = CAN_DefaultParams.CAN_NART;
	CAN_InitStructure->CAN_Prescaler = CAN_DefaultParams.CAN_Prescaler;
	CAN_InitStructure->CAN_RFLM = CAN_DefaultParams.CAN_RFLM;
	CAN_InitStructure->CAN_SJW	= CAN_DefaultParams.CAN_SJW;
	CAN_InitStructure->CAN_TTCM	= CAN_DefaultParams.CAN_TTCM;
	CAN_InitStructure->CAN_TXFP = CAN_DefaultParams.CAN_TXFP;
}

/**
 *	@brief	CAN�˲�������Ĭ�����ó�ʼ��
 *	@param	(CAN_FilterInitTypeDef*)CAN_FilterInitStructure
 */
void CAN_Filter_ParamsInit(CAN_FilterInitTypeDef* CAN_FilterInitStructure)
{
	CAN_FilterInitStructure->CAN_FilterActivation = CAN_Filter_DefaultParams.CAN_FilterActivation;
	CAN_FilterInitStructure->CAN_FilterFIFOAssignment = CAN_Filter_DefaultParams.CAN_FilterFIFOAssignment;
	CAN_FilterInitStructure->CAN_FilterIdHigh = CAN_Filter_DefaultParams.CAN_FilterIdHigh;
	CAN_FilterInitStructure->CAN_FilterIdLow = CAN_Filter_DefaultParams.CAN_FilterIdLow;
	CAN_FilterInitStructure->CAN_FilterMaskIdHigh = CAN_Filter_DefaultParams.CAN_FilterMaskIdHigh;
	CAN_FilterInitStructure->CAN_FilterMaskIdLow = CAN_Filter_DefaultParams.CAN_FilterMaskIdLow;
	CAN_FilterInitStructure->CAN_FilterMode = CAN_Filter_DefaultParams.CAN_FilterMode;
	CAN_FilterInitStructure->CAN_FilterNumber = CAN_Filter_DefaultParams.CAN_FilterNumber;
	CAN_FilterInitStructure->CAN_FilterScale = CAN_Filter_DefaultParams.CAN_FilterScale;
}

/**
 *	@brief	CAN1��ʼ��
 *	@note		ֻ�����жϽ���
 */
void CAN1_Init(void)
{
	CAN_InitTypeDef CAN_InitStructure;
	CAN_FilterInitTypeDef CAN_FilterInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);	// ʹ��CAN1ʱ��
	
	CAN1_GPIO_Init();

	/* CAN NVIC �ж����� */
	NVICx_init(CAN1_RX0_IRQn, CAN1_RX0_PRIO_PRE, CAN1_RX0_PRIO_SUB);
	
	/* CAN�������� */
	CAN_ParamsInit(&CAN_InitStructure);
	CAN_Init(CAN1, &CAN_InitStructure);
	
	/* CAN�˲����������� */
	CAN_Filter_ParamsInit(&CAN_FilterInitStructure);
	CAN_FilterInit(&CAN_FilterInitStructure);
	
	/* FIFO0��Ϣ�Һ��ж����� */
	CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
}

/**
 *	@brief	CAN2��ʼ��
 *	@note		�����ж��շ�
 */
void CAN2_Init(void)
{
	CAN_InitTypeDef CAN_InitStructure;
	CAN_FilterInitTypeDef CAN_FilterInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);	// ʹ��CAN2ʱ��
	
	CAN2_GPIO_Init();

	/* CAN NVIC �ж����� */
	NVICx_init(CAN2_RX0_IRQn, CAN2_RX0_PRIO_PRE, CAN2_RX0_PRIO_SUB);
	//NVICx_init(CAN2_TX_IRQn, CAN2_TX_PRIO_PRE, CAN2_TX_PRIO_SUB);
	
	/* CAN�������� */
	CAN_ParamsInit(&CAN_InitStructure);
	CAN_Init(CAN2, &CAN_InitStructure);
	
	/* CAN�˲����������� */
	CAN_Filter_ParamsInit(&CAN_FilterInitStructure);
	CAN_FilterInitStructure.CAN_FilterNumber = 14;	// ģ��19��������
	CAN_FilterInit(&CAN_FilterInitStructure);
	
	/* FIFO0��Ϣ�Һ��ж� �� ������������ж� ���� */
	CAN_ITConfig(CAN2, CAN_IT_FMP0, ENABLE);
	CAN_ITConfig(CAN2, CAN_IT_TME,  ENABLE);
}

/**
 *	@brief	CAN1 ���ͺ���
 *	@param	uint32_t stdID	- ��׼��ʶ��
 *					int16_t* dat - ���ݻ�����
 *	@note		Ĭ��8���ֽڵ�����
 *	@debug	2019/09/22
 *						��ͷ�ϵ��豸IDΪ0x203
 */
void CAN1_Send(uint32_t stdID, int16_t *dat)
{
	CanTxMsg	txMsg;
	
	txMsg.StdId = stdID;		// ʹ�ñ�׼��ʶ��
	txMsg.IDE = CAN_ID_STD;	// ʹ�ñ�׼ģʽ
	txMsg.RTR = CAN_RTR_DATA;	// 0 - ����֡
	txMsg.DLC = 8;	// ���ݳ���

	// �ȷ���8λ���ݣ��ٷ���8λ����
	txMsg.Data[0] = (uint8_t)((int16_t)dat[0] >> 8);
	txMsg.Data[1] = (uint8_t)((int16_t)dat[0]);
	txMsg.Data[2] = (uint8_t)((int16_t)dat[1] >> 8);
	txMsg.Data[3] = (uint8_t)((int16_t)dat[1]);
	txMsg.Data[4] = (uint8_t)((int16_t)dat[2] >> 8);
	txMsg.Data[5] = (uint8_t)((int16_t)dat[2]);
	txMsg.Data[6] = (uint8_t)((int16_t)dat[3] >> 8);
	txMsg.Data[7] = (uint8_t)((int16_t)dat[3]);
	
	CAN_Transmit(CAN1, &txMsg);
	//xQueueSend(CAN1_Queue, &txMsg, 1);
}

/**
 *	@brief	CAN1 ������亯��
 *	@param	uint32_t stdID	- ��׼��ʶ��
 *					int16_t* dat - ���ݻ�����
 *	@note		Ĭ��8���ֽڵ�����
 */
void CAN1_QueueSend(uint32_t stdID, int16_t *dat)
{
	CanTxMsg	txMsg;
	
	txMsg.StdId = stdID;		// ʹ�ñ�׼��ʶ��
	txMsg.IDE = CAN_ID_STD;	// ʹ�ñ�׼ģʽ
	txMsg.RTR = CAN_RTR_DATA;	// 0 - ����֡
	txMsg.DLC = 8;	// ���ݳ���

	// �ȷ���8λ���ݣ��ٷ���8λ����
	txMsg.Data[0] = (uint8_t)((int16_t)dat[0] >> 8);
	txMsg.Data[1] = (uint8_t)((int16_t)dat[0]);
	txMsg.Data[2] = (uint8_t)((int16_t)dat[1] >> 8);
	txMsg.Data[3] = (uint8_t)((int16_t)dat[1]);
	txMsg.Data[4] = (uint8_t)((int16_t)dat[2] >> 8);
	txMsg.Data[5] = (uint8_t)((int16_t)dat[2]);
	txMsg.Data[6] = (uint8_t)((int16_t)dat[3] >> 8);
	txMsg.Data[7] = (uint8_t)((int16_t)dat[3]);
	
	//CAN_Transmit(CAN1, &txMsg);
	xQueueSend(CAN1_Queue, &txMsg, 1);
}

/**
 *	@brief	CAN2 ���ͺ���
 *	@param	uint32_t stdID	- ��׼��ʶ��
 *					int16_t* dat - ���ݻ�����
 *	@note		Ĭ��8���ֽڵ�����
 *	@debug	
 */
void CAN2_Send(uint32_t stdID, int16_t *dat)
{
	CanTxMsg	txMsg;
	
	txMsg.StdId = stdID;		// ʹ�ñ�׼��ʶ��
	txMsg.IDE = CAN_ID_STD;	// ʹ�ñ�׼ģʽ
	txMsg.RTR = CAN_RTR_DATA;	// 0 - ����֡
	txMsg.DLC = 8;	// ���ݳ���

	// �ȷ���8λ���ݣ��ٷ���8λ����
	txMsg.Data[0] = (uint8_t)((int16_t)dat[0] >> 8);
	txMsg.Data[1] = (uint8_t)((int16_t)dat[0]);
	txMsg.Data[2] = (uint8_t)((int16_t)dat[1] >> 8);
	txMsg.Data[3] = (uint8_t)((int16_t)dat[1]);
	txMsg.Data[4] = (uint8_t)((int16_t)dat[2] >> 8);
	txMsg.Data[5] = (uint8_t)((int16_t)dat[2]);
	txMsg.Data[6] = (uint8_t)((int16_t)dat[3] >> 8);
	txMsg.Data[7] = (uint8_t)((int16_t)dat[3]);
	
	CAN_Transmit(CAN2, &txMsg);
	//xQueueSend(CAN2_Queue, &txMsg, 1);
}

/**
 *	@brief	CAN2 ������亯��
 *	@param	uint32_t stdID	- ��׼��ʶ��
 *					int16_t* dat - ���ݻ�����
 *	@note		Ĭ��8���ֽڵ�����
 *	@debug	
 */
void CAN2_QueueSend(uint32_t stdID, int16_t *dat)
{
	CanTxMsg	txMsg;
	
	txMsg.StdId = stdID;		// ʹ�ñ�׼��ʶ��
	txMsg.IDE = CAN_ID_STD;	// ʹ�ñ�׼ģʽ
	txMsg.RTR = CAN_RTR_DATA;	// 0 - ����֡
	txMsg.DLC = 8;	// ���ݳ���

	// �ȷ���8λ���ݣ��ٷ���8λ����
	txMsg.Data[0] = (uint8_t)((int16_t)dat[0] >> 8);
	txMsg.Data[1] = (uint8_t)((int16_t)dat[0]);
	txMsg.Data[2] = (uint8_t)((int16_t)dat[1] >> 8);
	txMsg.Data[3] = (uint8_t)((int16_t)dat[1]);
	txMsg.Data[4] = (uint8_t)((int16_t)dat[2] >> 8);
	txMsg.Data[5] = (uint8_t)((int16_t)dat[2]);
	txMsg.Data[6] = (uint8_t)((int16_t)dat[3] >> 8);
	txMsg.Data[7] = (uint8_t)((int16_t)dat[3]);
	
	//CAN_Transmit(CAN2, &txMsg);
	xQueueSend(CAN2_Queue, &txMsg, 1);
}

/**
 *	@brief	CAN1 �����ж�(��������̨)
 *	@note		������Ҫ�������жϴ�ӡ����(��ռ��Դ) 
 */
uint16_t js_201_last_current = 0;
float js_201_power = 0;
void CAN1_RX0_IRQHandler(void)
{
	static uint8_t bm = 0;
	static uint8_t cnt = 0;
	CanRxMsg	rxMsg;
	
	if(CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET) {
		CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);	// ����жϱ�־λ
		CAN_Receive(CAN1, CAN_FIFO0, &rxMsg);
		
		if(rxMsg.StdId == 0x201) {	// ��ǰ
			/* ʵ��ת�ص�����¼ */
			g_Chassis_Motor_Info[LEFT_FRON_201].current = CAN_GetMotorCurrent(&rxMsg);
			// ȡǰ������ĵ���ֵƽ��ֵ����ƽ�����ʣ���Ϊ��ѹΪ�ȶ���24V)
			js_201_power = 24 * (abs(g_Chassis_Motor_Info[LEFT_FRON_201].current)+abs(js_201_last_current))/(2*16384.f) * 20;
			/* ��е�Ƕȼ�¼ */
			g_Chassis_Motor_Info[LEFT_FRON_201].angle = CAN_GetMotorAngle(&rxMsg);
			/* ���̵���ۼӽǶȷ�����¼ */			
			CalcMotorAngleSum(g_Chassis_Motor_Info[LEFT_FRON_201].angle, g_Chassis_Motor_Info[LEFT_FRON_201].angle_prev, &g_Chassis_Motor_Info[LEFT_FRON_201].angle_sum);
			g_Chassis_Motor_Info[LEFT_FRON_201].angle_prev = g_Chassis_Motor_Info[LEFT_FRON_201].angle;
			Chassis_PID[LEFT_FRON_201].Angle.feedback = g_Chassis_Motor_Info[LEFT_FRON_201].angle_sum;
			/* �ٶȻ�ʵʱ�ٶȼ�¼ */
			g_Chassis_Motor_Info[LEFT_FRON_201].speed = CAN_GetMotorSpeed(&rxMsg);
			Chassis_PID[LEFT_FRON_201].Speed.feedback = CAN_GetMotorSpeed(&rxMsg);
			
			bm |= BM_CAN_REPORT_201;
		}
		
		if(rxMsg.StdId == 0x202) {	// ��ǰ
			/* ʵ��ת�ص�����¼ */
			g_Chassis_Motor_Info[RIGH_FRON_202].current = CAN_GetMotorCurrent(&rxMsg);
			/* ��е�Ƕȼ�¼ */
			g_Chassis_Motor_Info[RIGH_FRON_202].angle = CAN_GetMotorAngle(&rxMsg);
			/* ���̵���ۼӽǶȷ�����¼ */			
			CalcMotorAngleSum(g_Chassis_Motor_Info[RIGH_FRON_202].angle, g_Chassis_Motor_Info[RIGH_FRON_202].angle_prev, &g_Chassis_Motor_Info[RIGH_FRON_202].angle_sum);
			g_Chassis_Motor_Info[RIGH_FRON_202].angle_prev = g_Chassis_Motor_Info[RIGH_FRON_202].angle;
			Chassis_PID[RIGH_FRON_202].Angle.feedback = g_Chassis_Motor_Info[RIGH_FRON_202].angle_sum;
			/* �ٶȻ�ʵʱ�ٶȼ�¼ */
			g_Chassis_Motor_Info[RIGH_FRON_202].speed = CAN_GetMotorSpeed(&rxMsg);
			Chassis_PID[RIGH_FRON_202].Speed.feedback = CAN_GetMotorSpeed(&rxMsg);
			
			bm |= BM_CAN_REPORT_202;
		}
		
		if(rxMsg.StdId == 0x203) {	// ���
			/* ʵ��ת�ص�����¼ */
			g_Chassis_Motor_Info[LEFT_BACK_203].current = CAN_GetMotorCurrent(&rxMsg);			
			/* ��е�Ƕȼ�¼ */
			g_Chassis_Motor_Info[LEFT_BACK_203].angle = CAN_GetMotorAngle(&rxMsg);
			/* ���̵���ۼӽǶȷ�����¼ */			
			CalcMotorAngleSum(g_Chassis_Motor_Info[LEFT_BACK_203].angle, g_Chassis_Motor_Info[LEFT_BACK_203].angle_prev, &g_Chassis_Motor_Info[LEFT_BACK_203].angle_sum);
			g_Chassis_Motor_Info[LEFT_BACK_203].angle_prev = g_Chassis_Motor_Info[LEFT_BACK_203].angle;
			Chassis_PID[LEFT_BACK_203].Angle.feedback = g_Chassis_Motor_Info[LEFT_BACK_203].angle_sum;
			/* �ٶȻ�ʵʱ�ٶȼ�¼ */
			g_Chassis_Motor_Info[LEFT_BACK_203].speed = CAN_GetMotorSpeed(&rxMsg);
			Chassis_PID[LEFT_BACK_203].Speed.feedback = CAN_GetMotorSpeed(&rxMsg);
			
			bm |= BM_CAN_REPORT_203;
		}
		
		if(rxMsg.StdId == 0x204) {	// �Һ�
			/* ʵ��ת�ص�����¼ */
			g_Chassis_Motor_Info[RIGH_BACK_204].current = CAN_GetMotorCurrent(&rxMsg);			
			/* ��е�Ƕȼ�¼ */
			g_Chassis_Motor_Info[RIGH_BACK_204].angle = CAN_GetMotorAngle(&rxMsg);
			/* ���̵���ۼӽǶȷ�����¼ */			
			CalcMotorAngleSum(g_Chassis_Motor_Info[RIGH_BACK_204].angle, g_Chassis_Motor_Info[RIGH_BACK_204].angle_prev, &g_Chassis_Motor_Info[RIGH_BACK_204].angle_sum);
			g_Chassis_Motor_Info[RIGH_BACK_204].angle_prev = g_Chassis_Motor_Info[RIGH_BACK_204].angle;
			Chassis_PID[RIGH_BACK_204].Angle.feedback = g_Chassis_Motor_Info[RIGH_BACK_204].angle_sum;
			/* �ٶȻ�ʵʱ�ٶȼ�¼ */
			g_Chassis_Motor_Info[RIGH_BACK_204].speed = CAN_GetMotorSpeed(&rxMsg);
			Chassis_PID[RIGH_BACK_204].Speed.feedback = CAN_GetMotorSpeed(&rxMsg);

			bm |= BM_CAN_REPORT_204;
		}
		
		if(rxMsg.StdId == 0x205) {	// Yaw����̨���
			/* ����¶ȼ�¼ */
			g_Gimbal_Motor_Info[YAW_205].temperature = CAN_GetMotorTemperature(&rxMsg);
			/* �ٶȻ�ʵʱ��¼ */			
			// ..�޸ĳ���IMU�ķ���ֵ
			// Gimbal_PID[MECH][YAW_205].Speed.feedback = CAN_GetMotorSpeed(&rxMsg);
			g_Gimbal_Motor_Info[YAW_205].speed = CAN_GetMotorSpeed(&rxMsg);
			
			/* λ�û��Ƕȼ�¼ */
			g_Gimbal_Motor_Info[YAW_205].angle = CAN_GetMotorAngle(&rxMsg);			
			Gimbal_PID[MECH][YAW_205].Angle.feedback = CAN_GetMotorAngle(&rxMsg);	// ��еģʽ YAW 
			Chassis_Z_PID.Angle.feedback = CAN_GetMotorAngle(&rxMsg);
			
			bm |= BM_CAN_REPORT_205;
		}
		
		if(rxMsg.StdId == 0x206) {	// Pitch����̨���
			/* ����¶ȼ�¼ */
			g_Gimbal_Motor_Info[PITCH_206].temperature = CAN_GetMotorTemperature(&rxMsg);
			/* �ٶȻ�ʵʱ��¼ */
			// ..�޸ĳ���IMU�ķ���ֵ
			// Gimbal_PID[MECH][PITCH_206].Speed.feedback = CAN_GetMotorSpeed(&rxMsg);			
			g_Gimbal_Motor_Info[PITCH_206].speed = CAN_GetMotorSpeed(&rxMsg);
			
			/* λ�û��Ƕȼ�¼ */
			g_Gimbal_Motor_Info[PITCH_206].angle = CAN_GetMotorAngle(&rxMsg);
			Gimbal_PID[MECH][PITCH_206].Angle.feedback = CAN_GetMotorAngle(&rxMsg);			
			
			bm |= BM_CAN_REPORT_206;
		}

		/* 
		   ÿ����200���ж��ж�1���Ƿ�CANʧ��
		   2020/04/26 �趨Ϊ100�ε�ʱ��ᷢ��yaw���������ʧ��������Ӷ����µ��ж��
		*/
		cnt++;
		if(cnt > 200) {
			cnt = 0;
			BitMask.Chassis.CanReport = bm & (BM_CAN_REPORT_201 | BM_CAN_REPORT_202 | BM_CAN_REPORT_203 | BM_CAN_REPORT_204);
			BitMask.Gimbal.CanReport  = bm & (BM_CAN_REPORT_205 | BM_CAN_REPORT_206);
			bm = 0;
		}
	}
}

/**
 *	@brief	CAN2 �����ж�(�������)
 */
void CAN2_RX0_IRQHandler(void)
{
	static uint8_t bm = 0;
	static uint8_t cnt = 0;	
	CanRxMsg	rxMsg;
	
	if(CAN_GetITStatus(CAN2, CAN_IT_FMP0) != RESET) {
		CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);	// ����жϱ�־λ
		CAN_Receive(CAN2, CAN_FIFO0, &rxMsg);
		
		if(rxMsg.StdId == REVOLVER_ID)	// 0x207
		{
			/* �������ʵʱת�ٷ�����¼ */
			g_Revolver_Motor_Info.speed = CAN_GetMotorSpeed(&rxMsg);
			Revolver_PID.Speed.feedback = CAN_GetMotorSpeed(&rxMsg);

			/* ��������ۼӽǶȷ�����¼ */			
			g_Revolver_Motor_Info.angle = CAN_GetMotorAngle(&rxMsg);
			CalcMotorAngleSum(g_Revolver_Motor_Info.angle, g_Revolver_Motor_Info.angle_prev, &g_Revolver_Motor_Info.angle_sum);
			g_Revolver_Motor_Info.angle_prev = g_Revolver_Motor_Info.angle;
			Revolver_PID.Angle.feedback = g_Revolver_Motor_Info.angle_sum;
			
			bm |= REVOLVER_BM_CAN_REPORT;
		}
		
		cnt++;
		if(cnt > 100) {
			cnt = 0;
			BitMask.Revolver.CanReport = bm & REVOLVER_BM_CAN_REPORT;
			bm = 0;
		}
	}		
}

///**
// *	@brief	CAN2 �����ж�
// */
//void CAN2_TX_IRQHandler(void)
//{
//	if(CAN_GetITStatus(CAN2, CAN_IT_TME) != RESET) {
//		CAN_ClearITPendingBit(CAN2, CAN_IT_TME);
//	}
//}
