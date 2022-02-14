#include "heatmap.h"
#include "string.h"

uint16_t m_heatmap[KBD_SETTING_ROW_PINS_MAX][KBD_SETTING_ROW_PINS_MAX];
void response_heatmap(uint8_t *data, uint8_t length) {
    uint8_t row = *(data+2);
    uint8_t index = *(data+3);
    memcpy( data+4, &(m_heatmap[row][13*index]), length-4);
    return;
}

void heatmap_init(void) {
    memset(m_heatmap, 0, sizeof(m_heatmap));
}