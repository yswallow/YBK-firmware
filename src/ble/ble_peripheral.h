#ifndef __BLE_PERIPHERAL_H
#define __BLE_PERIPHERAL_H

extern uint16_t m_conn_handle;
extern bool ble_nus_echobacked;
void send_place_ble(uint8_t row, uint8_t col, bool press);
void send_mouse_periph(int16_t x, int16_t y);
void periph_nus_init(void);
void cache_pop_peripheral(void);
#endif // __BLE_PERIPHERAL_H