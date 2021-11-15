#ifndef __NEOPIXEL_H
#define  __NEOPIXEL_H

#include <stdint.h>

#define NEOPIXEL_MAX_CHAINS 40
#define NEOPIXEL_GRB(green,red,blue) ( ((uint8_t)(blue)) | (((uint8_t)(red)) << 8) | (((uint8_t)(green)) << 16))

typedef void (*neopixel_handler_t)(void);

void neopixel_init(uint8_t pin, neopixel_handler_t handler);
void neopixel_uninit(void);
void neopixel_write(uint32_t *colors, uint32_t leds_count);

#endif
