#ifdef KEYBOARD_PERIPH
#include <string.h>

#include "ble_nus.h"
#include "nrf_log.h"
#include "app_error.h"
#include "app_util_platform.h"

#include "ble_peripheral.h"
#include "neopixel.h"
#include "neopixel_fds.h"
#include "via.h"

#define UART_CACHE_SIZE 32
#define UART_CACHE_COUNT 7

BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);

typedef struct {
    uint8_t length;
    uint8_t data[UART_CACHE_SIZE];
} uart_cache_t;
 
static uart_cache_t uart_cache[UART_CACHE_COUNT];
static uint16_t uart_cache_usage = 0;

/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_evt       Nordic UART Service event.
 */
static void nus_data_handler(ble_nus_evt_t * p_evt)
{
    const uint8_t *data;
    uint16_t len;

    switch(p_evt->type) {
    case BLE_NUS_EVT_RX_DATA:
        data = p_evt->params.rx_data.p_data;
        len = p_evt->params.rx_data.length;
        
        if( len > UART_CACHE_SIZE-1 ) {
            len = UART_CACHE_SIZE - 1;
        }
        
        for(uint8_t i=0;i<UART_CACHE_COUNT;i++) {
            if(! (uart_cache_usage & (1<<i)) ) {
                memcpy(uart_cache[i].data, data, len);
                uart_cache[i].length = len;
                uart_cache_usage |= (1<<i);
                break;
            }
        }
        
        break;
    default:
        break;
    }
}


static void uart_receive_peripheral(uint8_t* p_data, uint16_t len) {
    NRF_LOG_HEXDUMP_DEBUG(p_data,len)

#ifndef NO_NEOPIXEL
    switch(p_data[0]) {
    case UART_NEOPIXEL_SYNC_TIMING_ID:
        neopixel_sync();
        break;
    case UART_NEOPIXEL_SAVE_ID:
        save_neopixel();
        break;
    case UART_NEOPIXEL_SYNC_PATTERN_ID:
        if( p_data[1]<NEOPIXEL_USER_DEFINED_COUNT ) {
            m_neopixel_pattern = p_data[1];
        }
        break;

    case UART_NEOPIXEL_SET_PATTERN_ID:
    case UART_NEOPIXEL_SET_PATTERN_CONF_ID:
        p_data[0] = ID_SET_KEYBOARD_VALUE;
        p_data[1] -= 2;
        raw_hid_receive_neopixel(p_data, len);
        break;

    case UART_NEOPIXEL_GET_PATTERN_CONF_ID:
    case UART_NEOPIXEL_GET_PATTERN_ID:
        p_data[0] = ID_GET_KEYBOARD_VALUE;
        p_data[1] -= 2;
        raw_hid_receive_neopixel(p_data, len);
        break;
    }
#endif // NO_NEOPIXEL
}


void cache_pop_peripheral(void) {
    uint8_t data[UART_CACHE_SIZE];
    uint16_t len;
    if(uart_cache_usage) {
        for(uint8_t i=0;i<UART_CACHE_COUNT;i++) {
            if( uart_cache_usage & (1<<i) ) {
                memcpy(data, uart_cache[i].data, uart_cache[i].length);
                len = uart_cache[i].length;
                uart_cache_usage &= ~(1<<i);
                
                uart_receive_peripheral(data,len);
            }
        }
    }
}


void periph_nus_init(void) {
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
    ble_nus_data_send(&m_nus, (uint8_t *)message, &message_sent, m_conn_handle);
    if(press) {
        NRF_LOG_DEBUG("Send KeyPress");
    } else {
        NRF_LOG_DEBUG("Send KeyRelease");
    }
}
#endif
