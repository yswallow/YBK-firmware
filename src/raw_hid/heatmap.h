#include "keyboard_generic.h"

void response_heatmap(uint8_t *data, uint8_t length);
void heatmap_init(void);

extern uint16_t m_heatmap[KBD_SETTING_ROW_PINS_MAX][KBD_SETTING_ROW_PINS_MAX];