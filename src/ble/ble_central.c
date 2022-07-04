/**
 * Copyright (c) 2016 - 2020, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifdef KEYBOARD_CENTRAL
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "nordic_common.h"
#include "app_error.h"
#include "ble_db_discovery.h"
#include "app_timer.h"
#include "app_util.h"
#include "ble.h"
#include "ble_gap.h"
#include "ble_hci.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "ble_nus_c.h"
#include "nrf_ble_gatt.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_ble_scan.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "ble_conn_state.h"
#include "app_timer.h"
#include "peer_manager.h"
#include "id_manager.h"

#include "keyboard_generic.h"
#include "ble_central.h"
#include "ble_setting.h" // for KEYBOARD_PERIPH_NAME

#ifndef NO_NEOPIXEL
#include "neopixel.h"
#endif

#include "keyboard_config.h"
#include "debug_message_hid.h"
#include "raw_hid.h"
#include "via.h"


#define APP_BLE_CONN_CFG_TAG    1                                       /**< Tag that refers to the BLE stack configuration set with @ref sd_ble_cfg_set. The default tag is @ref BLE_CONN_CFG_TAG_DEFAULT. */
#define APP_BLE_OBSERVER_PRIO   3                                       /**< BLE observer priority of the application. There is no need to modify this value. */

#define UART_TX_BUF_SIZE        256                                     /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE        256                                     /**< UART RX buffer size. */

#define NUS_SERVICE_UUID_TYPE   BLE_UUID_TYPE_VENDOR_BEGIN              /**< UUID type for the Nordic UART Service (vendor specific). */

#define ECHOBACK_BLE_UART_DATA  1                                       /**< Echo the UART data that is received over the Nordic UART Service (NUS) back to the sender. */

#define PAIRING_TIMEOUT_TICKS   APP_TIMER_TICKS(2000)
#define KEYBOARD_CENTRAL_SCAN_TIMEOUT_TICKS   APP_TIMER_TICKS(60000)

#define UART_CACHE_SIZE 16

BLE_NUS_C_DEF(m_ble_nus_c);                                             /**< BLE Nordic UART Service (NUS) client instance. */
NRF_BLE_GATT_DEF(m_gatt_c);                                               /**< GATT module instance. */
BLE_DB_DISCOVERY_DEF(m_db_disc);                                        /**< Database discovery module instance. */
NRF_BLE_SCAN_DEF(m_scan);                                               /**< Scanning Module instance. */
NRF_BLE_GQ_DEF(m_ble_gatt_queue,                                        /**< BLE GATT Queue instance. */
               NRF_SDH_BLE_CENTRAL_LINK_COUNT,
               NRF_BLE_GQ_QUEUE_SIZE);
APP_TIMER_DEF(m_pairing_timer);
APP_TIMER_DEF(m_central_scan_timer);

static uint16_t m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - OPCODE_LENGTH - HANDLE_LENGTH; /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
ble_data_t m_scan_adv_report_buffer[BLE_GAP_SCAN_BUFFER_MAX];

static uint8_t m_uart_send_buffer[UART_CACHE_SIZE];
static uint8_t m_uart_send_len;
static bool m_periph_connected = false;

/**@brief NUS UUID. */
static ble_uuid_t const m_nus_uuid =
{
    .uuid = BLE_UUID_NUS_SERVICE,
    .type = NUS_SERVICE_UUID_TYPE
};
/*
static ble_uuid_t const m_hid_uuid =
{
    .uuid = BLE_UUID_HUMAN_INTERFACE_DEVICE_SERVICE, 
    .type = BLE_UUID_TYPE_BLE
};
*/

ble_gap_scan_params_t gap_c_scan_params = {
    .active = 0,
    .interval      = NRF_BLE_SCAN_SCAN_INTERVAL,
    .window        = NRF_BLE_SCAN_SCAN_WINDOW,
    .scan_phys     = BLE_GAP_PHY_1MBPS,
    .filter_policy = BLE_GAP_SCAN_FP_WHITELIST,
    .extended      = false,
    .timeout = 1500
};


/**@brief Function for handling asserts in the SoftDevice.
 *
 * @details This function is called in case of an assert in the SoftDevice.
 *
 * @warning This handler is only an example and is not meant for the final product. You need to analyze
 *          how your product is supposed to react in case of assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num     Line number of the failing assert call.
 * @param[in] p_file_name  File name of the failing assert call.
 */
/*
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}
*/

/**@brief Function for handling the Nordic UART Service Client errors.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nus_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function to start scanning. */

static void scan_start(void)
{
    ret_code_t ret;
    //ret = sd_ble_gap_scan_start(&(m_scan.scan_params), &(m_scan.scan_buffer));
    ret = nrf_ble_scan_start(&m_scan);
    APP_ERROR_CHECK(ret);
    
    NRF_LOG_INFO("Start Scanning...");
}

#define PM_PEER_LIST_SIZE 16
static void scan_turnoff_whitelist(void* ptr) {
    ret_code_t ret;
    ble_gap_scan_params_t param;
    memcpy(&param, &gap_c_scan_params, sizeof(ble_gap_scan_params_t));
    param.filter_policy = BLE_GAP_SCAN_FP_ACCEPT_ALL;
    param.timeout       = 6000;
    ret = nrf_ble_scan_params_set(&m_scan, &param);
    APP_ERROR_CHECK(ret);
    scan_start();
}

/**@brief Function for handling Scanning Module events.
 */
static void scan_evt_handler(scan_evt_t const * p_scan_evt)
{
    ret_code_t err_code;
    //NRF_LOG_DEBUG("scan event 0x%x", p_scan_evt->scan_evt_id);
    switch(p_scan_evt->scan_evt_id)
    {
        // Connect is in ble_c_evt_handler
        case NRF_BLE_SCAN_EVT_WHITELIST_ADV_REPORT:
        case NRF_BLE_SCAN_EVT_FILTER_MATCH:
            if( ble_conn_state_central_conn_count()==0 ) {
                err_code = sd_ble_gap_connect(&(p_scan_evt->params.filter_match.p_adv_report->peer_addr), &gap_c_scan_params, &gap_conn_params, 1);
                if( err_code==NRF_SUCCESS ) {
                    NRF_LOG_INFO("Connection Request...");
                    err_code = app_timer_start(m_pairing_timer, PAIRING_TIMEOUT_TICKS, NULL);
                    APP_ERROR_CHECK(err_code);
                } else {
                    NRF_LOG_DEBUG("sd_ble_gap_connect() fail in scan_evt_handler");
                }
            }
            break;
        
        case NRF_BLE_SCAN_EVT_NOT_FOUND:
            //NRF_LOG_INFO("No Periph found");
            break;

        case NRF_BLE_SCAN_EVT_SCAN_TIMEOUT:
        {
            NRF_LOG_INFO("Scan timed out.");
            if(m_scan.scan_params.filter_policy == BLE_GAP_SCAN_FP_WHITELIST) {
                NRF_LOG_INFO("Turn off whitelist");
                scan_turnoff_whitelist(NULL);
            } else {
                sleep_mode_enter(NULL);
            }
        } break;

        case NRF_BLE_SCAN_EVT_CONNECTING_ERROR:
        {
            err_code = p_scan_evt->params.connecting_err.err_code;
            APP_ERROR_CHECK(err_code);
        } break;


        case NRF_BLE_SCAN_EVT_CONNECTED:
        {
            ble_gap_evt_connected_t const * p_connected = p_scan_evt->params.connected.p_connected;
            // Scan is automatically stopped by the connection.
            NRF_LOG_INFO("Connecting to target %02x%02x%02x%02x%02x%02x",
                      p_connected->peer_addr.addr[0],
                      p_connected->peer_addr.addr[1],
                      p_connected->peer_addr.addr[2],
                      p_connected->peer_addr.addr[3],
                      p_connected->peer_addr.addr[4],
                      p_connected->peer_addr.addr[5]
                      );
        } break;

        case NRF_BLE_SCAN_EVT_WHITELIST_REQUEST:
            NRF_LOG_INFO("BLE_SCAN_EVT_WHITELIST_REQUEST");
            
            pm_peer_id_t peer_list[PM_PEER_LIST_SIZE];
            uint32_t peer_list_size = PM_PEER_LIST_SIZE;
            
            err_code = pm_peer_id_list(peer_list, &peer_list_size, PM_PEER_ID_INVALID, PM_PEER_ID_LIST_SKIP_NO_IRK);
            APP_ERROR_CHECK(err_code);
            if( peer_list_size == 0 ) {
                scan_turnoff_whitelist(NULL);
                break;
            }
            err_code = im_whitelist_set(peer_list, peer_list_size);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            NRF_LOG_DEBUG("Unkown eventid %x", p_scan_evt->scan_evt_id);
            break;
    }
}

static void cancel_pairing(void* ptr) {
    sd_ble_gap_connect_cancel();
    NRF_LOG_INFO("Pairing Canceled");
    scan_start();
}



/**@brief Function for initializing the scanning and setting the filters.
 */
static void scan_init(void)
{
    ret_code_t          err_code;
    nrf_ble_scan_init_t init_scan;

    memset(&init_scan, 0, sizeof(init_scan));
    init_scan.p_scan_param = &gap_c_scan_params;
    init_scan.connect_if_match = false;
    init_scan.conn_cfg_tag     = APP_BLE_CONN_CFG_TAG;

    err_code = nrf_ble_scan_init(&m_scan, &init_scan, scan_evt_handler);
    APP_ERROR_CHECK(err_code);
/*
    err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_UUID_FILTER, &m_nus_uuid);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_ble_scan_filters_enable(&m_scan, NRF_BLE_SCAN_UUID_FILTER, false);
    APP_ERROR_CHECK(err_code);
*/
    err_code = app_timer_create(&m_pairing_timer, APP_TIMER_MODE_SINGLE_SHOT, cancel_pairing);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_create(&m_central_scan_timer, APP_TIMER_MODE_SINGLE_SHOT, sleep_mode_enter);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_start(m_central_scan_timer, KEYBOARD_CENTRAL_SCAN_TIMEOUT_TICKS, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling database discovery events.
 *
 * @details This function is a callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function forwards the events
 *          to their respective services.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
    ble_nus_c_on_db_disc_evt(&m_ble_nus_c, p_evt);
}


/**@brief Function for handling characters received by the Nordic UART Service (NUS).
 *
 * @details This function takes a list of characters of length data_len and prints the characters out on UART.
 *          If @ref ECHOBACK_BLE_UART_DATA is set, the data is sent back to sender.
 */
static uint8_t ascii2num(char c) {
    if( c<0x3A ) {
        return c-0x30;
    } else {
        return c-0x41+10;
    }
}
keypress_cache_t keypress_peripheral_cache[KEYPRESS_PERIPHERAL_CACHE_LEN];
uint8_t keypress_cache_head;
uint8_t keypress_cache_unexecuted_count;

void keypress_cache_init(void) {
    //memset(keypress_peripheral_cache, 2, sizeof(keypress_peripheral_cache));
    keypress_cache_head = 0;
    keypress_cache_unexecuted_count = 0;
}

uint8_t get_keypress_cache_last(void) {
    uint8_t last = keypress_cache_head+keypress_cache_unexecuted_count;
    return last%10;
}

void keypress_cache_append(uint8_t row, uint8_t col, uint8_t state) {
    uint8_t i = get_keypress_cache_last();
    keypress_peripheral_cache[i].col = col;
    keypress_peripheral_cache[i].row = row;
    keypress_peripheral_cache[i].state = state;
    CRITICAL_REGION_ENTER();
    keypress_cache_unexecuted_count++;
    CRITICAL_REGION_EXIT();
}

void cache_pop_central(void) {
    for( uint8_t i=0; i<keypress_cache_unexecuted_count; i++) {
        uint8_t j = (keypress_cache_head+i)%KEYPRESS_PERIPHERAL_CACHE_LEN;
        
        if( keypress_peripheral_cache[j].state == 1 ) {
            keypress(keypress_peripheral_cache[j].row, keypress_peripheral_cache[j].col+my_keyboard.split_keyboard.central_cols_count, false);
            NRF_LOG_INFO("Press");
        } else if( keypress_peripheral_cache[j].state == 0 ) {
            keyrelease( keypress_peripheral_cache[j].row, keypress_peripheral_cache[j].col+my_keyboard.split_keyboard.central_cols_count, false);
            NRF_LOG_INFO("Release");
        } else {
            NRF_LOG_INFO("N/A");
        }
    }
    CRITICAL_REGION_ENTER();
    keypress_cache_head = (keypress_cache_head+keypress_cache_unexecuted_count) % 10;
    keypress_cache_unexecuted_count = 0;
    CRITICAL_REGION_EXIT();
}

static bool send_keycode_central(char* p_char) {
    uint8_t row = ascii2num(*(p_char+1));
    uint8_t col = ascii2num(*(p_char+3));
    if( *p_char == 'R' ) {
        keypress_cache_append(row, col, 0);
        return true;
    } else if (*p_char=='P') {
        keypress_cache_append(row, col, 1);
        return true;
    } else {
        NRF_LOG_INFO("N/A");
        return false;
    }
}

#ifdef TRACKBALL_ENABLE
static void mouse_move_central(char* p_data) {
    int16_t delta_x = p_data[1]<<8 | p_data[2];
    int16_t delta_y = p_data[3]<<8 | p_data[4];
    
    int8_t wheel = 0; //delta_y >> 2;
    hid_functions.mouse_move(delta_x, delta_y, wheel);
}
#endif // TRACKBALL_ENABLE

static uint8_t periph_raw_rep_buffer[32];
static bool uart_data_receive(char* p_data) {
    if( p_data[0] != 'D' ) {
        return false;
    }
    uint8_t len = p_data[1];
    p_data[3] += 0x80;
    memcpy(periph_raw_rep_buffer, p_data+2, len>32? 32 : len);
    raw_hid_send(periph_raw_rep_buffer, len>32? 32 : len);
    return true;
}


static uint8_t uart_received_data[UART_CACHE_SIZE];
static uint8_t uart_received_data_head = 0;
static void ble_nus_chars_received_keyboard(uint8_t * p_data, uint16_t data_len);
static void uart_join_central(uint8_t* p_data, uint16_t len) {
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
        ble_nus_chars_received_keyboard(uart_received_data, uart_received_data_head);
        uart_received_data_head = 0;
    }
}


static void ble_nus_chars_received_keyboard(uint8_t * p_data, uint16_t data_len)
{
    NRF_LOG_DEBUG("Receiving data.");
    NRF_LOG_HEXDUMP_DEBUG(p_data, data_len);
    
    switch(*((char*)p_data)) {
        case 'P':
        case 'R':
            send_keycode_central((char*)p_data);
            break;
        case 'D':
            uart_data_receive((char*)p_data);
            break;
#ifdef TRACKBALL_ENABLE
        case 'M':
            mouse_move_central((char*)p_data);
            break;
#endif // TRACKBALL_ENABLE
        default:
            break;
    }
}

/**@brief Callback handling Nordic UART Service (NUS) client events.
 *
 * @details This function is called to notify the application of NUS client events.
 *
 * @param[in]   p_ble_nus_c   NUS client handle. This identifies the NUS client.
 * @param[in]   p_ble_nus_evt Pointer to the NUS client event.
 */

/**@snippet [Handling events from the ble_nus_c module] */
static void ble_nus_c_evt_handler(ble_nus_c_t * p_ble_nus_c, ble_nus_c_evt_t const * p_ble_nus_evt)
{
    ret_code_t err_code;
#ifndef NO_NEOPIXEL
    uint8_t save_data[1] = { UART_NEOPIXEL_SYNC_TIMING_ID };
#endif
    switch (p_ble_nus_evt->evt_type)
    {
        case BLE_NUS_C_EVT_DISCOVERY_COMPLETE:
            NRF_LOG_INFO("Discovery complete.");
            err_code = ble_nus_c_handles_assign(p_ble_nus_c, p_ble_nus_evt->conn_handle, &p_ble_nus_evt->handles);
            APP_ERROR_CHECK(err_code);

            err_code = ble_nus_c_tx_notif_enable(p_ble_nus_c);
            APP_ERROR_CHECK(err_code);
            NRF_LOG_INFO("Connected to device with Nordic UART Service.");
            m_periph_connected = true;
            
            err_code = app_timer_stop(m_pairing_timer);
            APP_ERROR_CHECK(err_code);
            err_code = app_timer_stop(m_central_scan_timer);
            APP_ERROR_CHECK(err_code);
#ifndef NO_NEOPIXEL
            err_code = ble_nus_c_string_send(p_ble_nus_c, save_data, 1);
            APP_ERROR_CHECK(err_code);
            neopixel_sync();
#endif

            advertising_start(true, BLE_ADV_MODE_FAST);
            break;

        case BLE_NUS_C_EVT_NUS_TX_EVT:
            NRF_LOG_HEXDUMP_DEBUG(p_ble_nus_evt->p_data, p_ble_nus_evt->data_len);
            uart_join_central(p_ble_nus_evt->p_data, p_ble_nus_evt->data_len);
            break;

        case BLE_NUS_C_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected.");
            for(uint8_t i=my_keyboard.split_keyboard.central_cols_count; i<my_keyboard.kbd_cols_count; i++) {
                for(uint8_t j=0;j<my_keyboard.kbd_rows_count;j++) {
                    keyrelease(j,i,false);
                }
            }
            KEYBOARD_DEBUG_HID_REGISTER_STRING("NUS Disconnected.", 14);
            m_periph_connected = false;
            scan_start();
            break;
    }
}

void gatt_c_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED)
    {
        NRF_LOG_INFO("ATT MTU exchange completed.");

        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("Ble NUS max data length set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }
}


/**@brief Function for initializing the GATT library. */
void gatt_c_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt_c, gatt_c_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_central_set(&m_gatt_c, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}

/**@snippet [Handling events from the ble_nus_c module] */

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
bool ble_c_connected = false;
void ble_c_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t            err_code;
    ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            err_code = pm_conn_secure(p_ble_evt->evt.gap_evt.conn_handle, false);
            APP_ERROR_CHECK(err_code);
            err_code = ble_nus_c_handles_assign(&m_ble_nus_c, p_ble_evt->evt.gap_evt.conn_handle, NULL);
            APP_ERROR_CHECK(err_code);

            // start discovery of services. The NUS Client waits for a discovery result
            err_code = ble_db_discovery_start(&m_db_disc, p_ble_evt->evt.gap_evt.conn_handle);
            APP_ERROR_CHECK(err_code);
            ble_c_connected = true;
            NRF_LOG_INFO("Connected: Central");
            //err_code = app_timer_stop(m_pairing_timer);
            //APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected. conn_handle: 0x%x, reason: 0x%x",
                         p_gap_evt->conn_handle,
                         p_gap_evt->params.disconnected.reason);
            if( p_gap_evt->params.disconnected.reason == 0x3E ) { //BLE_HCI_CONN_FAILED_TO_BE_ESTABLISHED
                delete_secure_failed_peer(p_gap_evt->conn_handle);
            }
            ble_c_connected = false;
            scan_turnoff_whitelist(NULL);
            break;

        case BLE_GAP_EVT_TIMEOUT:
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                NRF_LOG_INFO("Connection Request timed out.");
            }
            
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported.
            //err_code = sd_ble_gap_sec_params_reply(p_ble_evt->evt.gap_evt.conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            //APP_ERROR_CHECK(err_code);
            break;
        
        case BLE_GAP_EVT_ADV_REPORT:
            //NRF_LOG_DEBUG("BLE_GAP_EVT_ADV_REPORT");
            //if( ble_conn_state_central_conn_count()==0 ) {
                if( (
                        //ble_advdata_short_name_find(p_gap_evt->params.adv_report.data.p_data, p_gap_evt->params.adv_report.data.len, KBD_PERIPH_SHORT_NAME, 2)  && 
                        ble_advdata_uuid_find(p_gap_evt->params.adv_report.data.p_data, p_gap_evt->params.adv_report.data.len, &m_nus_uuid ) 
                    )  && (
                        p_gap_evt->params.adv_report.type.connectable
                    ) ) {
                    err_code = sd_ble_gap_connect(&(p_gap_evt->params.adv_report.peer_addr), &(m_scan.scan_params), &gap_conn_params, 1);
                    if( err_code==NRF_SUCCESS ) {
                        NRF_LOG_INFO("Connection Request...");
                        err_code = app_timer_start(m_pairing_timer, PAIRING_TIMEOUT_TICKS, NULL);
                        APP_ERROR_CHECK(err_code);
                    } else {
                        NRF_LOG_DEBUG("sd_ble_gap_connect() fail in ble_c_evt_handler");
                    }
                }
            //}
            break;
        
        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
            // Accepting parameters requested by peer.
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                                    &p_gap_evt->params.conn_param_update_request.conn_params);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            break;
    }
}

/**@brief Function for initializing the Nordic UART Service (NUS) client. */
static void nus_c_init(void)
{
    ret_code_t       err_code;
    ble_nus_c_init_t init;

    init.evt_handler   = ble_nus_c_evt_handler;
    init.error_handler = nus_error_handler;
    init.p_gatt_queue  = &m_ble_gatt_queue;

    err_code = ble_nus_c_init(&m_ble_nus_c, &init);
    APP_ERROR_CHECK(err_code);
}


/** @brief Function for initializing the database discovery module. */
static void db_discovery_init(void)
{
    ble_db_discovery_init_t db_init;

    memset(&db_init, 0, sizeof(ble_db_discovery_init_t));

    db_init.evt_handler  = db_disc_handler;
    db_init.p_gatt_queue = &m_ble_gatt_queue;

    ret_code_t err_code = ble_db_discovery_init(&db_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details Handles any pending log operations, then sleeps until the next event occurs.

static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}
 */

void uart_send_central(uint8_t *p_data, uint8_t len) {
    ret_code_t ret;
    uint8_t head = 0;
    bool is_end = false;
    bool is_start = true;
    if( m_periph_connected ) {
        for(uint8_t i=0;i<(len+UART_CACHE_SIZE-1)/UART_CACHE_SIZE;i++) {
            if( (len-head)>(UART_CACHE_SIZE-1) ) {
                m_uart_send_len = UART_CACHE_SIZE-1;
            } else {
                m_uart_send_len = len-head;
            }

            if( i==(len+UART_CACHE_SIZE-1)/UART_CACHE_SIZE-1 ) {
                is_end = true;
            } 
            m_uart_send_buffer[0] = ( is_start ? 0x80 : 0 ) | ( is_end ? 0x40 : 0 ) | m_uart_send_len;
            memcpy(m_uart_send_buffer+1, p_data+head, m_uart_send_len);
            NRF_LOG_DEBUG("Sending..");
            NRF_LOG_HEXDUMP_DEBUG(m_uart_send_buffer, m_uart_send_len);
            ret = ble_nus_c_string_send(&m_ble_nus_c,m_uart_send_buffer, m_uart_send_len+1);
            APP_ERROR_CHECK(ret);
            is_start = false;
        }
    }
}


void ble_central_init(void)
{
    // Initialize.
    memset(keypress_peripheral_cache, 0, sizeof(keypress_peripheral_cache));
    db_discovery_init();
    gatt_c_init();
    nus_c_init();
    scan_init();
    /*
    ble_conn_state_conn_handle_list_t connections = ble_conn_state_central_handles();
    if( connections.len > 0 ) {
       sd_ble_gap_disconnect(connections.conn_handles[0], BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    }
    */
    NRF_LOG_INFO("BLE UART central example started.");
}

void ble_central_start(void)
{
    scan_start();
}

#define SETTING (*(data+1))

raw_hid_receive_t raw_hid_receive_for_peripheral(uint8_t *data, uint8_t length) {
    if( 0x84<=SETTING && SETTING<=0x87 ) {
        SETTING = SETTING - 0x80;
        uart_send_central(data, length);
        *data = ID_UNHANDLED;
        return true;
    } else {
        return false;
    }
}
#endif //KEYBOARD_CENTRAL