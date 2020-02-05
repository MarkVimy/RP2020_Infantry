#ifndef __RNG_H
#define __RNG_H
#include "sys.h"

bool RNG_init(void);
uint32_t get_random_num(void);
uint8_t get_u8_random_num(uint8_t min, uint8_t max);

#endif
