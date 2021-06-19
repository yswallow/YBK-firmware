#include <stdbool.h>

#ifndef __VIA_FDS_H
#define __VIA_FDS_H

#define CONFIG_FILE     (0x8010)
#define CONFIG_REC_KEY  (0x7010)
#define KBD_SETTING_FILE (0x8989)
#define KBD_SETTING_REC_KEY (0x1212)

#define VIA_EEPROM_MAGIC_ADDR (via_fds_get_eeprom_addr()+700)
#define EEPROM_SIZE 900

#define KBD_SETTING_COL_PINS_MAX 30
#define KBD_SETTING_ROW_PINS_MAX 30
#define KBD_SETTING_ADDITIONAL_LEN 30
#define KBD_SETTING_ADDITIONAL_DEBOUNCING_INDEX 2
#define KBD_SETTING_ADDITIONAL_LAYER_COUNT_INDEX 3
#define KBD_SETTING_ADDITIONAL_TAPPING_TERM_INDEX 0
#define KBD_SETTING_SIZE (0x60)

static volatile bool m_fds_initialized;

void via_fds_init(void);
uint8_t* via_fds_get_eeprom_addr(void);
void save_keymap(void);
void raw_hid_receive_kb(uint8_t *data, uint8_t length);
#endif // __VIA_FDS_H