/**
 * @file        mpu6500.c
 * @author      MaWeiming
 * @Version     V1.0
 * @date        24-February-2020
 * @brief       This file includes the MPU6500 external functions 
 * 				(based on ST Peripherals Libaray Keil.STM32F4xx_DFP.2.13.0)
 */

/**
 *	# MPU6500 SPI Interface
 *	1. MSB ����
 *	2. SCLK ������ - �������� �½��� - ��������
 *	3. SCLK <= 1MHz
 *	4. SPI��д����������16��ʱ������( >=2�ֽ����� )
 *		��ʽ����ַ(1�ֽ�) + ����(>=1�ֽ�)
 *
 *		SPI Address Format
 *		MSB							LSB
 *		R/W	A6	A5	A4	A3	A2	A1	A0
 *		(1/0)
 *
 *		SPI Data Format
 *		MSB							LSB
 *		D7	D6	D5	D4	D3	D2	D1	D0
 */

/* Includes ------------------------------------------------------------------*/
#include "mpu6500.h"

#include "spi1.h"
#include "sys.h"
#include "delay.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#define MPU6500_CS				PAout(15)
#define MPU6500_CS_ENABLE()		(MPU6500_CS = 0)
#define MPU6500_CS_DISABLE()	(MPU6500_CS = 1)

#define DUMMY_BYTE				0xff

/*----��MPU6500�Ĵ�����----*/
#define REG_SELF_TEST_X_GYRO	0x00	// R/W
#define REG_SELF_TEST_Y_GYRO	0x01	// R/W
#define REG_SELF_TEST_Z_GYRO	0x02	// R/W
#define REG_SELF_TEST_X_ACCEL	0x0D	// R/W
#define REG_SELF_TEST_Y_ACCEL	0x0E	// R/W
#define REG_SELF_TEST_Z_ACCEL	0x0F	// R/W

#define REG_XG_OFFSET_H			0x13	// R/W
#define REG_XG_OFFSET_L			0x14	// R/W
#define REG_YG_OFFSET_H			0x15	// R/W
#define REG_YG_OFFSET_L			0x16	// R/W
#define REG_ZG_OFFSET_H			0x17	// R/W
#define REG_ZG_OFFSET_L			0x18	// R/W

#define REG_SMPLRT_DIV			0x19 	// R/W
#define REG_CONFIG				0x1A	// R/W
#define REG_GYRO_CONFIG			0x1B	// R/W
#define REG_ACCEL_CONFIG		0x1C	// R/W
#define REG_ACCEL_CONFIG_2		0x1D	// R/W
#define REG_LP_ACCEL_ODR		0x1E	// R/W
#define REG_WHOM_THR			0x1F	// R/W
#define REG_FIFO_EN				0x23	// R/W
#define REG_I2C_MST_CTRL		0x24	// R/W
#define REG_I2C_SLV0_ADDR		0x25	// R/W
#define REG_I2C_SLV0_REG		0x26	// R/W
#define REG_I2C_SLV0_CTRL		0x27	// R/W
#define REG_I2C_SLV1_ADDR		0x28	// R/W
#define REG_I2C_SLV1_REG		0x29	// R/W
#define REG_I2C_SLV1_CTRL		0x2A	// R/W
#define REG_I2C_SLV2_ADDR		0x2B	// R/W
#define REG_I2C_SLV2_REG		0x2C	// R/W
#define REG_I2C_SLV2_CTRL		0x2D	// R/W
#define REG_I2C_SLV3_ADDR		0x2E	// R/W
#define REG_I2C_SLV3_REG		0x2F	// R/W
#define REG_I2C_SLV3_CTRL		0x30	// R/W
#define REG_I2C_SLV4_ADDR		0x31	// R/W
#define REG_I2C_SLV4_REG		0x32	// R/W
#define REG_I2C_SLV4_DO			0x33	// R/W
#define REG_I2C_SLV4_CTRL		0x34	// R/W
#define REG_I2C_SLV4_DI			0x35	// R
#define REG_I2C_MST_STATUS		0x36	// R

#define REG_INT_PIN_CFG			0x37	// R/W
#define REG_INT_ENABLE			0x38	// R/W
#define REG_INT_STATUS			0x3A	// R

#define REG_ACCEL_XOUT_H		0x3B	// R
#define REG_ACCEL_XOUT_L		0x3C	// R
#define REG_ACCEL_YOUT_H		0x3D	// R
#define REG_ACCEL_YOUT_L		0x3E	// R
#define REG_ACCEL_ZOUT_H		0x3F	// R
#define REG_ACCEL_ZOUT_L		0x40	// R

#define REG_TEMP_OUT_H			0x41	// R
#define REG_TEMP_OUT_L			0x42	// R

#define REG_GYRO_XOUT_H			0x43	// R
#define REG_GYRO_XOUT_L			0x44	// R
#define REG_GYRO_YOUT_H			0x45	// R
#define REG_GYRO_YOUT_L			0x46	// R
#define REG_GYRO_ZOUT_H			0x47	// R
#define REG_GYRO_ZOUT_L			0x48	// R

#define REG_I2C_SLV0_DO			0x63	// R/W
#define REG_I2C_SLV1_DO			0x64	// R/W
#define REG_I2C_SLV2_DO			0x65	// R/W
#define REG_I2C_SLV3_DO			0x66	// R/W
#define REG_I2C_MST_DELAY_CTRL	0x67	// R/W
#define REG_SIGNAL_PATH_RESET	0x68	// R/W
#define REG_ACCEL_INTEL_CTRL	0x69	// R/W
#define REG_USER_CTRL			0x6A	// R/W
#define REG_PWR_MGMT_1			0x6B	// R/W
#define REG_PWR_MGMT_2			0x6C	// R/W
#define REG_FIFO_COUNT_H		0x72	// R/W
#define REG_FIFO_COUNT_L		0x73	// R/W
#define REG_FIFO_R_W			0x74	// R/W
#define REG_WHO_AM_I			0x75	// R

#define REG_XA_OFFSET_H			0x77	// R/W
#define REG_XA_OFFSET_L			0x78	// R/W
#define REG_YA_OFFSET_H			0x7A	// R/W
#define REG_YA_OFFSET_L			0x7B	// R/W
#define REG_ZA_OFFSET_H			0x7D	// R/W
#define REG_ZA_OFFSET_L			0x7E	// R/W

/*----��MPU6500�Ĵ�����----*/

/* Private variables ---------------------------------------------------------*/
/* ## Global variables ## ----------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void MPU6500_CS_Pin_Init(void);
static void MPU6500_SPI_Init(void);
static uint8_t MPU6500_SPI_Transfer(uint8_t data, int8_t *status);
static uint8_t MPU6500_WriteByte(uint8_t reg, uint8_t data);
static uint8_t MPU6500_ReadByte(uint8_t reg, uint8_t *data);

/* Private functions ---------------------------------------------------------*/
/**
 *	@note	PA15 - SPI1_NSS
 */
static void MPU6500_CS_Pin_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15,
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* ��ʼ������ */
	GPIO_SetBits(GPIOA, GPIO_Pin_15);		
}

static void MPU6500_SPI_Init(void)
{
	MPU6500_CS_Pin_Init();
	SPI1_Init();
}

static uint8_t MPU6500_SPI_Transfer(uint8_t data, int8_t *status)
{
	return SPI_Transfer(SPI1, data, status);
}

/**
 *	@brief	MPU6500 ���Ĵ���д��1���ֽ�
 *	@param	uint8_t	reg  - 7λ�Ĵ�����ַ
 *			uint8_t data - �����ֽ�
 *	@return 0 - ����
 *			1 - ����
 */
static uint8_t MPU6500_WriteByte(uint8_t reg, uint8_t data)
{
	int8_t status;
	
	MPU6500_CS = 0;
	
	MPU6500_SPI_Transfer(reg, &status);	// д�Ĵ���
	if(status) {
		MPU6500_CS = 1;
		return 1;
	}
	
	MPU6500_SPI_Transfer(data, &status);	// д�Ĵ���
	if(status) {
		MPU6500_CS = 1;
		return 1;
	}
	
	MPU6500_CS = 1;
	return 0;
}

/**
 *	@brief	MPU6500 �ӼĴ�������1���ֽ�
 *	@param	uint8_t	reg  - 7λ�Ĵ�����ַ
 *			uint8_t *data - ��ȡ������
 *	@return 0 - ����
 *			1 - ����
 */
static uint8_t MPU6500_ReadByte(uint8_t reg, uint8_t *data)
{
	int8_t status;
	
	MPU6500_CS = 0;
	
	MPU6500_SPI_Transfer((reg|0x80), &status);	// ���Ĵ���
	if(status) {
		MPU6500_CS = 1;
		return 1;
	}
	
	*data = MPU6500_SPI_Transfer(DUMMY_BYTE, &status);	// д�Ĵ���
	if(status) {
		MPU6500_CS = 1;
		return 1;
	}
	
	MPU6500_CS = 1;
	return 0;
}

/**
 *	@brief	����MPU6500�����Ǵ����������̷�Χ(Degree Per Second, ��/s)
 *	@param	uint8_t fs
 *					0x00 - ��250dps
 *					0x01 - ��500dps
 *					0x02 - ��1000dps
 *					0x03 - ��2000dps
 *	@return 0 - ����
 *			1 - ����
 */
static uint8_t MPU6500_SetGyroFullScale(uint8_t fs)
{
	return MPU6500_WriteByte(REG_GYRO_CONFIG, fs<<3);
}

/**
 *	@brief	����MPU6500���ٶȴ����������̷�Χ(g, m/(s^2))
 *	@param	uint8_t fs
 *					0x00 - ��2g
 *					0x01 - ��4g
 *					0x02 - ��8g
 *					0x03 - ��16g
 *	@return 0 - ����
 *			1 - ����
 */
static uint8_t MPU6500_SetAccelFullScale(uint8_t fs)
{
	return MPU6500_WriteByte(REG_ACCEL_CONFIG, fs<<3);
}

/**
 *	@brief	����MPU6500���ֵ�ͨ�˲���
 *	@param	uint16_t lpf	���ֵ�ͨ�˲���Ƶ��(Hz)
 *	@return 0 - ����
 *			1 - ����
 */
static uint8_t MPU6500_SetLpf(uint16_t lpf)
{
	uint8_t cfg;
	
	if(lpf <= 5)
		cfg = 6;
	else if(lpf <= 10)
		cfg = 5;
	else if(lpf <= 20)
		cfg = 4;
	else if(lpf <= 41)
		cfg = 3;
	else if(lpf <= 92)
		cfg = 2;
	else if(lpf <= 184)
		cfg = 1;
	else
		cfg = 0;
	
	return MPU6500_WriteByte(REG_CONFIG, cfg);
}

/**
 *	@brief	����MPU6500��������(�ڲ������� = 1kHz)
 *			SMAPLE_RATE = INTERNAL_SAMPLE_RATE / (1+SMPLRT_DIV)
 *	@param	uint16_t rate	����Ƶ��(Hz)
 *	@note	������֤ rate �ɱ�1000����������ʧ����
 *	@return 0 - ����
 *			1 - ����
 */
static uint8_t MPU6500_SetSampleRate(uint16_t rate)
{
	uint8_t smplrt_div;
	
	if(rate > 1000)
		rate = 1000;
	else if(rate < 4)
		rate = 4;
	
	smplrt_div = 1000/rate - 1;
	
	MPU6500_WriteByte(REG_SMPLRT_DIV, smplrt_div);

	return MPU6500_SetLpf(rate/2);	// �Զ�����Ϊ�����ʵ�һ��
}

/* API functions -------------------------------------------------------------*/
uint8_t MPU6500_Init(void)
{
	uint8_t device_id;
	
	// ��ʼ��SPI����
	MPU6500_SPI_Init();
	// ��λMPU6500
	MPU6500_WriteByte(REG_PWR_MGMT_1, 0x80);
	delay_ms(100);
	MPU6500_WriteByte(REG_USER_CTRL, 0x11);
	delay_ms(100);
	// ��������������
	MPU6500_SetGyroFullScale(0x03);
	// ���ü��ٶȼ�����
	MPU6500_SetAccelFullScale(0x00);
	// ���ò�����
	MPU6500_SetSampleRate(50);
	
	MPU6500_ReadByte(REG_WHO_AM_I, &device_id);
	if(device_id == 0x70) {
		MPU6500_WriteByte(REG_PWR_MGMT_1, 0x01);
	} else {
		return 1;
	}
	return 0;
}

/**
 *	@brief	MPU6500 д�Ĵ���
 *	@param	uint8_t	addr - ��ַ(SPIͨ�Ų���)
 *			uint8_t	reg  - 7λ�Ĵ�����ַ
 *			uint8_t len	 - д������ݳ���
 *			uint8_t *txBuf - д���ݻ�����
 *	@return 0 - ����
 *			1 - ����
 */
uint8_t MPU6500_WriteReg(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *txBuf)
{
	uint8_t i = 0;
	int8_t status = 0;
	
	// SPIͨ������CS������ѡ��ӻ���ַ����addr����(Ϊ����iic��д������ʽ)
	MPU6500_CS = 0;
	
	MPU6500_SPI_Transfer(reg, &status);	// д�Ĵ���
	if(status) {
		MPU6500_CS = 1;	// ȡ��Ƭѡ
		return 1;
	}
	
	while(len--) {
		MPU6500_SPI_Transfer(txBuf[i++], &status);
		if(status) {
			MPU6500_CS = 1;	// ȡ��Ƭѡ
			return 1;
		}
	}
	
	MPU6500_CS = 1;
	return 0;
}

/**
 *	@brief	MPU6500 д�Ĵ���
 *	@param	uint8_t	addr - ��ַ(SPIͨ�Ų���)
 *			uint8_t	reg  - 7λ�Ĵ�����ַ
 *			uint8_t len	 - ���������ݳ���
 *			uint8_t *rxBuf - �����ݻ�����
 *	@return 0 - ����
 *			1 - ����
 */
uint8_t MPU6500_ReadReg(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *rxBuf)
{
	uint8_t i = 0;
	int8_t status = 0;
	
	// SPIͨ������CS������ѡ��ӻ���ַ����addr����(Ϊ����iic��д������ʽ)
	MPU6500_CS = 0;
	
	MPU6500_SPI_Transfer((reg|0x80), &status);
	if(status) {
		MPU6500_CS = 1;	// ȡ��Ƭѡ
		return 1;
	}
	
	while(len--) {
		rxBuf[i++] = MPU6500_SPI_Transfer(DUMMY_BYTE, &status);
		if(status) {
			MPU6500_CS = 1;	// ȡ��Ƭѡ
			return 1;
		}
	}
	
	MPU6500_CS = 1;
	return 0;	
}

/**
 *	@brief	�����¶�ֵ
 */
float MPU6500_GetTemperature(void)
{
	uint8_t rxBuf[2];
	int16_t raw;
	float temp;
	
	MPU6500_ReadByte(REG_TEMP_OUT_H, &rxBuf[0]);
	MPU6500_ReadByte(REG_TEMP_OUT_L, &rxBuf[0]);
	
	raw = ((uint16_t)rxBuf[0]<<8) | rxBuf[1];
	
	temp = 36.53+((double)raw)/340;
	
	return temp;
}

/**
 *	@brief	�õ������ǽ��ٶ�ֵ(ԭʼֵ)
 */
uint8_t MPU6500_GetGyroScope(int16_t *gx, int16_t *gy, int16_t *gz)
{
	uint8_t res = 0;
	uint8_t rxBuf[6];
	
	res |= MPU6500_ReadByte(REG_GYRO_XOUT_H, &rxBuf[0]);
	res |= MPU6500_ReadByte(REG_GYRO_XOUT_L, &rxBuf[1]);
	res |= MPU6500_ReadByte(REG_GYRO_YOUT_H, &rxBuf[2]);
	res |= MPU6500_ReadByte(REG_GYRO_YOUT_L, &rxBuf[3]);
	res |= MPU6500_ReadByte(REG_GYRO_ZOUT_H, &rxBuf[4]);
	res |= MPU6500_ReadByte(REG_GYRO_ZOUT_L, &rxBuf[5]);
	
	if(res == 0) {
		*gx = ((uint16_t)rxBuf[0]<<8) | rxBuf[1];
		*gy = ((uint16_t)rxBuf[2]<<8) | rxBuf[3];
		*gz = ((uint16_t)rxBuf[4]<<8) | rxBuf[5];
	}
	return res;
}

/**
 *	@brief	�õ����ٶȼƼ��ٶ�ֵ(ԭʼֵ)
 */
uint8_t MPU6500_GetAccelerometer(int16_t *ax, int16_t *ay, int16_t *az)
{
	uint8_t res = 0;
	uint8_t rxBuf[6];
	
	res |= MPU6500_ReadByte(REG_ACCEL_XOUT_H, &rxBuf[0]);
	res |= MPU6500_ReadByte(REG_ACCEL_XOUT_L, &rxBuf[1]);
	res |= MPU6500_ReadByte(REG_ACCEL_YOUT_H, &rxBuf[2]);
	res |= MPU6500_ReadByte(REG_ACCEL_YOUT_L, &rxBuf[3]);
	res |= MPU6500_ReadByte(REG_ACCEL_ZOUT_H, &rxBuf[4]);
	res |= MPU6500_ReadByte(REG_ACCEL_ZOUT_L, &rxBuf[5]);
	
	if(res == 0) {
		*ax = ((uint16_t)rxBuf[0]<<8) | rxBuf[1];
		*ay = ((uint16_t)rxBuf[2]<<8) | rxBuf[3];
		*az = ((uint16_t)rxBuf[4]<<8) | rxBuf[5];
	}
	return res;
}
