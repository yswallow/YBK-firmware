#ifndef __VIA_FDS_H
#define __VIA_FDS_H

#include <stdbool.h>
#include "fds.h"

#define KBD_SETTING_SIZE (0x60)
#define KBD_SETTING_COL_PINS_MAX 30
#define KBD_SETTING_ROW_PINS_MAX 30
#define KEYMAP_SIZE_BYTES 800
#define VIA_EEPROM_MAGIC_ADDR (via_fds_get_eeprom_addr()+KEYMAP_SIZE_BYTES)
#define EEPROM_SIZE 900

#define KBD_SETTING_ADDITIONAL_LEN 0x0A

enum {
    KBD_SETTING_ADDITIONAL_TAPPING_TERM_INDEX = 0,
    KBD_SETTING_ADDITIONAL_DEBOUNCING_INDEX = 2,
    KBD_SETTING_ADDITIONAL_LAYER_COUNT_INDEX,
    KBD_SETTING_ADDITIONAL_POWER_LED_EN_INDEX,
    KBD_SETTING_ADDITIONAL_POWER_LED_PIN_INDEX,
    KBD_SETTING_ADDITIONAL_DEFAULT_LAYER_INDEX,
    KBD_SETTING_ADDITIONAL_NEOPIXEL_LEN_INDEX,
    KBD_SETTING_ADDITIONAL_NEOPIXEL_PIN_INDEX
};

typedef bool raw_hid_receive_t;
typedef uint8_t matrix_row_t;

static volatile bool m_fds_initialized;
extern uint8_t kbd_setting[KBD_SETTING_SIZE];

void create_fds_new_entry(fds_record_desc_t* p_desc, fds_record_t* p_record);
void update_fds_entry(fds_record_desc_t* p_desc, fds_record_t* p_record);
void via_fds_init(void);
uint8_t* via_fds_get_eeprom_addr(void);
void save_keymap(void);
void save_kbd_setting(void);
void raw_hid_receive_kb(uint8_t *data, uint8_t length);
#endif // __VIA_FDS_H