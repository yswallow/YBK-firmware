#pragma once

#include <stdint.h>
#include "neopixel.h"

void setNumber(uint8_t number, uint32_t hier_color, uint32_t lower_color, uint32_t bg_color);

extern uint8_t neopixel_time_data[NEOPIXEL_FRAME_BYTES];

extern uint8_t m_second;
extern uint8_t m_minute;
extern uint8_t m_hour;