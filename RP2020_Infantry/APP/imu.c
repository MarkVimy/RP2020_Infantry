/**
 * @file        ultra.c
 * @author      MaWeiming
 * @Version     V1.0
 * @date        21-January-2020
 * @brief       This file includes the Ultrasonic(超声波) external functions 
 * 				(based on ST Peripherals Libaray Keil.STM32F4xx_DFP.2.13.0)
 * @Version		
 */

/**
 *	# 超声波
 */

/* Includes ------------------------------------------------------------------*/
#include "imu.h"

#include "mpu6050.h"
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
		// 读取陀螺仪角度和角速度
		//mpu_dmp_get_data( &Mpu_Info.roll, &Mpu_Info.pitch, &Mpu_Info.yaw);
		//MPU_Get_Gyroscope( &Mpu_Info.ratePitch, &Mpu_Info.rateRoll, &Mpu_Info.rateYaw);		
	}
	for(i = 0; i < 250; i++) {
		//delay_us(100);
		MPU_Get_Gyroscope( &Mpu_Info.ratePitch, &Mpu_Info.rateRoll, &Mpu_Info.rateYaw);
		Mpu_Info.ratePitchOffset += Mpu_Info.ratePitch;
		Mpu_Info.rateYawOffset   += Mpu_Info.rateYaw;
	}
	Mpu_Info.ratePitchOffset = (Mpu_Info.ratePitchOffset/250);
	Mpu_Info.rateYawOffset   = (Mpu_Info.rateYawOffset/250);	
}

/* API functions -------------------------------------------------------------*/
/* #驱动层# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@note	这里pass_flag初始化的时候一定要置1，不然陀螺仪数据漂得一批！！
 *			（想了差不多1个半月云台漂移的解决办法，真的有毒！！告诫后人）
 */
bool pass_flag = 1;
void IMU_Init(void)
{
	static int16_t  sTimeCnt = 0;	
	static uint32_t ulCurrentTime = 0;
	static portTickType ulLoopTime = 0;
	
	ulCurrentTime = millis();
	
	//祖传MPU初始化
	MPU_Init();
	while (mpu_dmp_init( )) 
	{
		ulCurrentTime = millis();

		if (ulCurrentTime >= ulLoopTime)  
		{
		  /* 100MS延时 */
			ulLoopTime = ulCurrentTime + TIME_STAMP_100MS;
		
		  /* 300ms屏蔽自检 */
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

/* #信息层# ---------------------------------------------------------------------------------------------------------------------------------------*/

/* #应用层# ---------------------------------------------------------------------------------------------------------------------------------------*/

/* #任务层# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	读取IMU数据
 */
void IMU_Task(void)
{
	/* 读取MPU6050传感器数据 */
	mpu_dmp_get_data( &Mpu_Info.roll, &Mpu_Info.pitch, &Mpu_Info.yaw);
	MPU_Get_Gyroscope( &Mpu_Info.ratePitch, &Mpu_Info.rateRoll, &Mpu_Info.rateYaw);		
}
