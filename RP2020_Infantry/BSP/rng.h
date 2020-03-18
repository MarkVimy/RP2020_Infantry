#ifndef __RNG_H
#define __RNG_H
#include "sys.h"

bool RNG_Init(void);
uint32_t RNG_ulGetRandomNum(void);
uint8_t RNG_ucGetRandomNum(uint8_t min, uint8_t max);

#endif
