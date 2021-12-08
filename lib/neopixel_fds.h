
#ifndef __NEOPIXEL_FDS_H
#define __NEOPIXEL_FDS_H

#include <stdint.h> 
#include "neopixel.h"

#define NEOPIXEL_USER_DEFINED_COUNT 3

#define NEOPIXEL_FRAME_BYTES ( NEOPIXEL_BYTES_PER_PIXEL * NEOPIXEL_MAX_CHAINS )
#define NEOPIXEL_PATTERN_BYTES ( NEOPIXEL_FRAME_BYTES * NEOPIXEL_MAX_FRAMES )

#define FDS_NEOPIXEL_FILE_ID (0x8139)
#define NEOPIXEL_FDS_FRAME_FILE_ID (0x8140)
#define NEOPIXEL_FDS_FRAME_REC_KEY_BASE (0x0001)
#define FDS_NEOPIXEL_REC_KEY (0x0001)
#define FDS_NEOPIXEL_CONF_FILE_ID FDS_NEOPIXEL_FILE_ID
#define FDS_NEOPIXEL_CONF_REC_KEY (0x0010)

typedef struct {
    uint8_t frame_count;
    uint8_t interval_ticks;
} neopixel_user_defined_config_t;

enum {
    KBD_NEOPIXEL = 0x10,
    KBD_NEOPIXEL_CONF,
};

extern neopixel_user_defined_config_t neopixel_user_defined_config[NEOPIXEL_USER_DEFINED_COUNT];
extern uint8_t neopixel_user_defined[NEOPIXEL_USER_DEFINED_COUNT][NEOPIXEL_MAX_FRAMES][NEOPIXEL_FRAME_BYTES];

void neopixel_fds_init(void);
void raw_hid_receive_neopixel(uint8_t *data, uint8_t length);


#endif // __NEOPIXEL_FDS_H