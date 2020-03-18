#include "rng.h"
#include "stm32f4xx_rng.h"
#include "delay.h"

/**
 *	@brief	Stm32F4 - 自带随机数发生器
 */
bool RNG_Init(void)
{
    uint16_t retry = 0;

    RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_RNG, ENABLE);

    RCC_AHB2PeriphResetCmd(RCC_AHB2Periph_RNG, ENABLE);
    RCC_AHB2PeriphResetCmd(RCC_AHB2Periph_RNG, DISABLE);

    RNG_Cmd(ENABLE);

    while (RNG_GetFlagStatus(RNG_FLAG_DRDY) == RESET && retry < 10000)
    {
        retry++;
        delay_us(100);
    }
    if (retry >= 10000)
        return 1;
    return 0;
}

/**
 *	@brief	反馈32位随机数
 */
uint32_t RNG_ulGetRandomNum(void)
{
	while(RNG_GetFlagStatus(RNG_FLAG_DRDY) == RESET);
	
	return RNG_GetRandomNumber();
}

/**
 *	@brief	反馈8位无符号随机数
 */
uint8_t RNG_ucGetRandomNum(uint8_t min, uint8_t max)
{
	return (uint8_t)(RNG_ulGetRandomNum() % (max - min + 1) + min);
}

