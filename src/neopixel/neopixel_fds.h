
#ifndef __NEOPIXEL_FDS_H
#define __NEOPIXEL_FDS_H

#include <stdint.h> 
#include "neopixel.h"
#include "via_fds.h"

#define NEOPIXEL_USER_DEFINED_COUNT 3

#define NEOPIXEL_ARRAY_COLS_MAX 16
#define NEOPIXEL_ARRAY_ROWS_MAX 16

// migration
#define NEOPIXEL_FDS_FRAME_FILE_ID_8140 (0x8140)
#define NEOPIXEL_MAX_CHAINS_8140 120
#define NEOPIXEL_FRAME_BYTES_8140 ( NEOPIXEL_BYTES_PER_PIXEL * NEOPIXEL_MAX_CHAINS_8140 )

#define NEOPIXEL_FDS_FRAME_FILE_ID (0x8141)
#define NEOPIXEL_FDS_FRAME_REC_KEY_BASE (0x0001)
#define FDS_NEOPIXEL_REC_KEY (0x0001)
#define FDS_NEOPIXEL_CONF_FILE_ID (0x8139)
#define FDS_NEOPIXEL_CONF_REC_KEY (0x0010)

#define FDS_NEOPIXEL_ARRAY_FILE_ID (0x8200)
#define FDS_NEOPIXEL_ARRAY_REC_KEY (0x0001)
#define FDS_NEOPIXEL_ARRAY_CONF_REC_KEY (0x0010)

#define NEOPIXEL_PLACE_INVALID 0xFF

typedef struct {
    uint8_t frame_count;
    uint8_t interval_ticks;
} neopixel_user_defined_config_t;

typedef struct {
    uint8_t rows;
    uint8_t cols;
} neopixel_array_config_t;  

enum {
    KBD_NEOPIXEL = 0x10,
    KBD_NEOPIXEL_CONF,
    KBD_NEOPIXEL_PERIPH,
    KBD_NEOPIXEL_PERIPH_CONF,
    KBD_NEOPIXEL_ARRAY,
    KBD_NEOPIXEL_ARRAY_CONF,
    KBD_NEOPIXEL_TIME,
};

extern neopixel_user_defined_config_t neopixel_user_defined_config[NEOPIXEL_USER_DEFINED_COUNT];
extern uint8_t neopixel_user_defined[NEOPIXEL_USER_DEFINED_COUNT][NEOPIXEL_MAX_FRAMES][NEOPIXEL_FRAME_BYTES];
extern neopixel_array_config_t neopixel_array_config;
extern uint8_t neopixel_array[NEOPIXEL_ARRAY_COLS_MAX*NEOPIXEL_ARRAY_ROWS_MAX];

void neopixel_fds_init(void);
raw_hid_receive_t raw_hid_receive_neopixel(uint8_t *data, uint8_t length);
void save_neopixel(void);
void save_neopixel_one_frame(void);

#endif // __NEOPIXEL_FDS_H