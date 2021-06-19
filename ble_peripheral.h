#ifndef __BLE_SLAVE_H
#define __BLE_SLAVE_H
extern uint16_t m_conn_handle;
extern bool ble_nus_echobacked;
void send_place_ble(uint8_t row, uint8_t col, bool press);
void nus_init(void);
#endif // __BLE_SLAVE_H