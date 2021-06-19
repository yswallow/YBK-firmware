#ifdef KEYBOARD_PERIPH
#include "ble_nus.h"
#include "ble_peripheral.h"
#include "nrf_log.h"
#include "app_error.h"

BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);

/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_evt       Nordic UART Service event.
 */
/**@snippet [Handling the data received over BLE] */

static void nus_data_handler(ble_nus_evt_t * p_evt)
{
    return;
}
/**@snippet [Handling the data received over BLE] */

void nus_init(void) {
    ret_code_t ret;
    ble_nus_init_t     nus_init;
    
    // Initialize NUS.
    memset(&nus_init, 0, sizeof(nus_init));
    
    nus_init.data_handler = nus_data_handler;

    ret = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(ret);
}


char num2ascii(uint8_t n) {
    return n<10 ? 0x30+n : 0x41+n-10;
}

void send_place_ble(uint8_t row, uint8_t col, bool press) {
    char message[8];
    uint16_t message_sent = 6;
    message[0] = press ? 'P' : 'R';
    message[1] = num2ascii(row);
    message[2] = ',';
    message[3] = num2ascii(col);
    message[4] = '\r';
    message[5] = '\n';
    message[6] = '\0';
    ble_nus_data_send(&m_nus, message, &message_sent, m_conn_handle);
    if(press) {
        NRF_LOG_INFO("Send KeyPress");
    } else {
        NRF_LOG_INFO("Send KeyRelease");
    }
}
#endif