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

#define UART_CACHE_SIZE 48
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

static uint8_t m_uart_send_buffer[UART_CACHE_SIZE];
static uint16_t m_uart_send_len;

void uart_send_peripheral(uint8_t *p_data, uint8_t len) {
    ret_code_t ret;
    uint8_t head = 0;
    bool is_end = false;
    bool is_start = true;
    //if( m_periph_connected ) {
        for(uint8_t i=0;i<(len+UART_CACHE_SIZE-1)/UART_CACHE_SIZE;i++) {
            if( (len-head)>(UART_CACHE_SIZE-1) ) {
                m_uart_send_len = UART_CACHE_SIZE-1;
            } else {
                m_uart_send_len = len-head;
            }

            if( i==(len+UART_CACHE_SIZE-1)/UART_CACHE_SIZE ) {
                is_end = true;
            } 
            m_uart_send_buffer[0] = ( is_start ? 0x80 : 0 ) | ( is_end ? 0x40 : 0 ) | m_uart_send_len;
            memcpy(m_uart_send_buffer+1, p_data+head, m_uart_send_len);
            NRF_LOG_DEBUG("Sending..");
            NRF_LOG_HEXDUMP_DEBUG(m_uart_send_buffer, m_uart_send_len);
            m_uart_send_len++;
            ret = ble_nus_data_send(&m_nus,m_uart_send_buffer, &m_uart_send_len, m_conn_handle);
            APP_ERROR_CHECK(ret);
            is_start = false;
        }
    //}
}


void send_data_ble(uint8_t* data, uint8_t len) {
    uint8_t message[40];
    uint16_t message_sent = len+3;
    uint8_t i;

    message[0] = 0xC0 | len;
    message[1] = 'D';
    message[2] = len;
    for(i=0;i<len;i++) {
        message[i+3] = data[i];
    }

    message[i++] = '\r';
    message[i++] = '\n';
    message[i++] = '\0';
    ble_nus_data_send(&m_nus, (uint8_t *)message, &message_sent, m_conn_handle);
    
    NRF_LOG_DEBUG("Data Sent");
}

static uint8_t uart_received_data[UART_CACHE_SIZE];
static uint16_t uart_received_data_head = 0;
static void uart_receive_peripheral(uint8_t* p_data, uint16_t len);

static void uart_join_peripheral(uint8_t* p_data, uint16_t len) {
    bool is_start = (*p_data >> 7) & 0x01;
    bool is_end = (*p_data >> 6) & 0x01;
    uint8_t data_size = *p_data & 0x3F;

    if(is_start) {
        memcpy(uart_received_data, p_data+1, data_size);
        uart_received_data_head = data_size;
    } else {
        memcpy(uart_received_data+uart_received_data_head, p_data+1, data_size);
        uart_received_data_head += data_size;
    }

    if(is_end) {
        uart_receive_peripheral(uart_received_data, uart_received_data_head);
        uart_received_data_head = 0;
    }
}

static void uart_receive_peripheral(uint8_t* p_data, uint16_t len) {
    NRF_LOG_HEXDUMP_DEBUG(p_data,len)

    switch(p_data[0]) {
#ifndef NO_NEOPIXEL
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
#endif // NO_NEOPIXEL
    default:
        raw_hid_receive(p_data, len);
        break;
    }

    if( p_data[0] != ID_UNHANDLED ) {
        send_data_ble(p_data, len);
    }
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
                
                uart_join_peripheral(data,len);
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
    uint16_t message_sent = 8;
    uint8_t i = 0;
    message[i++] = 0xC0 | (message_sent-1);
    message[i++] = press ? 'P' : 'R';
    message[i++] = num2ascii(row);
    message[i++] = ',';
    message[i++] = num2ascii(col);
    message[i++] = '\r';
    message[i++] = '\n';
    message[i++] = '\0';
    ble_nus_data_send(&m_nus, (uint8_t *)message, &message_sent, m_conn_handle);
    if(press) {
        NRF_LOG_DEBUG("Send KeyPress");
    } else {
        NRF_LOG_DEBUG("Send KeyRelease");
    }
}
#endif
