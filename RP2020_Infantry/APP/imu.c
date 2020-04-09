/**
 * @file        ultra.c
 * @author      MaWeiming
 * @Version     V1.0
 * @date        21-January-2020
 * @brief       This file includes the Ultrasonic(������) external functions 
 * 				(based on ST Peripherals Libaray Keil.STM32F4xx_DFP.2.13.0)
 * @Version		
 */

/**
 *	# ������
 */

/* Includes ------------------------------------------------------------------*/
#include "imu.h"

#include "mpu6050.h"
#include "mpu6500.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "delay.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* ## Global variables ## ----------------------------------------------------*/
extern Mpu_Info_t Mpu_Info;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
static void IMU_CalcAvrOffset(void)
{
	uint16_t i;
	for(i = 0; i < 5; i++) {
		delay_ms(1);
		// ��ȡ�����ǽǶȺͽ��ٶ�
		//mpu_dmp_get_data( &Mpu_Info.roll, &Mpu_Info.pitch, &Mpu_Info.yaw);
		//MPU_Get_Gyroscope( &Mpu_Info.ratePitch, &Mpu_Info.rateRoll, &Mpu_Info.rateYaw);		
	}
	for(i = 0; i < 250; i++) {
		//delay_us(100);
		//MPU_Get_Gyroscope(&Mpu_Info.ratePitch, &Mpu_Info.rateRoll, &Mpu_Info.rateYaw);
		MPU6500_GetGyroScope(&Mpu_Info.ratePitch, &Mpu_Info.rateRoll, &Mpu_Info.rateYaw);
		Mpu_Info.ratePitchOffset += Mpu_Info.ratePitch;
		Mpu_Info.rateYawOffset   += Mpu_Info.rateYaw;
	}
	Mpu_Info.ratePitchOffset = (Mpu_Info.ratePitchOffset/250);
	Mpu_Info.rateYawOffset   = (Mpu_Info.rateYawOffset/250);	
}

/* API functions -------------------------------------------------------------*/
/* #������# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@note	����pass_flag��ʼ����ʱ��һ��Ҫ��1����Ȼ����������Ư��һ������
 *			�����˲��1��������̨Ư�ƵĽ���취������ж����������ˣ�
 */
bool pass_flag = 1;
void IMU_Init(void)
{
	static int16_t  sTimeCnt = 0;	
	static uint32_t ulCurrentTime = 0;
	static portTickType ulLoopTime = 0;
	
	ulCurrentTime = millis();
	
	//�洫MPU��ʼ��
	//MPU_Init();
	MPU6500_Init();
	while (mpu_dmp_init( )) 
	{
		ulCurrentTime = millis();

		if (ulCurrentTime >= ulLoopTime)  
		{
		  /* 100MS��ʱ */
			ulLoopTime = ulCurrentTime + TIME_STAMP_100MS;
		
		  /* 300ms�����Լ� */
			if (sTimeCnt >= 2) 
			{
				pass_flag = 0;
				sTimeCnt  = 0;//10;
			}
			else
			{
				sTimeCnt++;
			}
		}
	}

	IMU_CalcAvrOffset();
}

/* #��Ϣ��# ---------------------------------------------------------------------------------------------------------------------------------------*/

/* #Ӧ�ò�# ---------------------------------------------------------------------------------------------------------------------------------------*/

/* #�����# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	��ȡIMU����
 */
void IMU_Task(void)
{
//	/* ��ȡMPU6050���������� */
//	mpu_dmp_get_data( &Mpu_Info.roll, &Mpu_Info.pitch, &Mpu_Info.yaw);
//	MPU_Get_Gyroscope( &Mpu_Info.ratePitch, &Mpu_Info.rateRoll, &Mpu_Info.rateYaw);		
	mpu_dmp_get_data( &Mpu_Info.roll, &Mpu_Info.pitch, &Mpu_Info.yaw);
	MPU6500_GetGyroScope( &Mpu_Info.ratePitch, &Mpu_Info.rateRoll, &Mpu_Info.rateYaw);
}
