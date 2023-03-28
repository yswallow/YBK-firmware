#include <stdint.h>


#define DYNAMIC_KEYMAP_LAYER_COUNT  ( dynamic_keymap_get_layer_count() )

uint8_t dynamic_keymap_get_layer_count(void);
void dynamic_keymap_get_buffer(const uint16_t offset, const uint16_t size, uint8_t *data);
void dynamic_keymap_set_buffer(const uint16_t offset, const uint16_t size, uint8_t *data);
uint16_t dynamic_keymap_get_keycode(const uint8_t layer, const uint8_t row, const uint8_t column);
void dynamic_keymap_set_keycode(const uint8_t layer, const uint8_t row, const uint8_t column, const uint16_t keycode);
uint8_t eeprom_read_byte(const uint8_t *address);
void eeprom_update_byte(uint8_t* addr, const uint8_t val);