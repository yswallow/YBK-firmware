#ifndef __NEOPIXEL_H
#define __NEOPIXEL_H

#include <stdint.h>

#define NEOPIXEL_MAX_CHAINS 160
#define NEOPIXEL_MAX_FRAMES 20
#define NEOPIXEL_BYTES_PER_PIXEL 3

#define NEOPIXEL_FRAME_BYTES ( NEOPIXEL_BYTES_PER_PIXEL * NEOPIXEL_MAX_CHAINS )
#define NEOPIXEL_PATTERN_BYTES ( NEOPIXEL_FRAME_BYTES * NEOPIXEL_MAX_FRAMES )
#define NEOPIXEL_BITS (NEOPIXEL_BYTES_PER_PIXEL*8)

#define NEOPIXEL_GRB(green,red,blue) ( ((uint8_t)(blue)) | (((uint8_t)(red)) << 8) | (((uint8_t)(green)) << 16))

enum {
    UART_NEOPIXEL_SYNC_TIMING_ID = 1,
    UART_NEOPIXEL_SYNC_PATTERN_ID,
    UART_NEOPIXEL_SET_PATTERN_CONF_ID,
    UART_NEOPIXEL_SET_PATTERN_ID,
    UART_NEOPIXEL_GET_PATTERN_CONF_ID,
    UART_NEOPIXEL_GET_PATTERN_ID,
    UART_NEOPIXEL_SAVE_ID
};

typedef void (*neopixel_handler_t)(void);

void neopixel_init(uint8_t pin, neopixel_handler_t handler);
void neopixel_uninit(void);
void neopixel_write_uint32(uint32_t *colors, uint32_t leds_count);
void neopixel_write_uint8(uint8_t *colors, uint32_t leds_count);
void neopixel_sync(void);

extern uint8_t m_neopixel_tick_count;
extern uint8_t m_neopixel_index;
extern uint8_t m_neopixel_pattern;
#endif
