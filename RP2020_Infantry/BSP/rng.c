#include "rng.h"
#include "stm32f4xx_rng.h"
#include "delay.h"

/**
 *	@brief	Stm32F4 - �Դ������������
 */
bool RNG_init(void)
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
 *	@brief	����32λ�����
 */
uint32_t get_random_num(void)
{
	while(RNG_GetFlagStatus(RNG_FLAG_DRDY) == RESET);
	
	return RNG_GetRandomNumber();
}

/**
 *	@brief	����8λ�޷��������
 */
uint8_t get_u8_random_num(uint8_t min, uint8_t max)
{
	return (uint8_t)(get_random_num() % (max - min + 1) + min);
}

