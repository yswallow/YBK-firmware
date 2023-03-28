#ifndef __BLE_PERIPHERAL_H
#define __BLE_PERIPHERAL_H

extern uint16_t m_conn_handle;
extern bool ble_nus_echobacked;
void send_place_ble(const uint8_t row, const uint8_t col, const bool press);
void periph_nus_init(void);
void cache_pop_peripheral(void);
#endif // __BLE_PERIPHERAL_H