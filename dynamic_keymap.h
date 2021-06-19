#include <stdint.h>
#define DYNAMIC_KEYMAP_LAYER_COUNT 6

uint8_t dynamic_keymap_get_layer_count(void);
void dynamic_keymap_get_buffer(uint16_t offset, uint16_t size, uint8_t *data);
void dynamic_keymap_set_buffer(uint16_t offset, uint16_t size, uint8_t *data);
uint16_t dynamic_keymap_get_keycode(uint8_t layer, uint8_t row, uint8_t column);
void dynamic_keymap_set_keycode(uint8_t layer, uint8_t row, uint8_t column, uint16_t keycode);
uint8_t eeprom_read_byte(uint8_t *address);
void eeprom_update_byte(uint8_t* addr, uint8_t val);