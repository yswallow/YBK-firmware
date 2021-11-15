#ifndef __NEOPIXEL_DATA_H
#define __NEOPIXEL_DATA_H

#include "neopixel.h"

extern uint32_t neopixel_splash[NEOPIXEL_MAX_CHAINS*2];
extern uint32_t neopixel_dimmer[NEOPIXEL_MAX_CHAINS*7];
void neopixel_data_init(void);

#endif