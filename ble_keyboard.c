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

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_assert.h"
#include "app_error.h"
#include "ble.h"
#include "ble_err.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advertising.h"
#include "ble_advdata.h"
#include "ble_hids.h"
#include "ble_dis.h"
#include "ble_conn_params.h"
#include "app_scheduler.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "fds.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"
#include "peer_manager_handler.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "ble_keyboard.h"
#include "main.h"
#include "keyboard_generic.h"

#ifdef KEYBOARD_CENTRAL
#include "ble_central.h"
#else
#ifdef KEYBOARD_PERIPH
#include "ble_nus.h"
#include "ble_peripheral.h"
#endif
#endif

#include "usb_mouse.h"
#include "via.h"



BLE_HIDS_DEF(m_hids,                                                /**< Structure used to identify the HID service. */
             NRF_SDH_BLE_TOTAL_LINK_COUNT,
             INPUT_REPORT_RAW_MAX_LEN+1,OUTPUT_REPORT_RAW_MAX_LEN+1, FEATURE_REPORT_MAX_LEN);
NRF_BLE_GATT_DEF(m_gatt);                                           /**< GATT module instance. */
BLE_ADVERTISING_DEF(m_advertising);                                 /**< Advertising module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                             /**< Context for the Queued Write module.*/

keyboard_hid_functions_t ble_hid_functions = {
    .keycode_append = keycode_append_ble,
    .keycode_remove = keycode_remove_ble,
    .handle_keycode = handle_keycode_ble,
    .reset = keyboard_reset_ble,
    .handle_mouse = handle_keycode_mouse_ble,
    .tick_handler_mouse = tick_handler_mouse_ble
};

static bool              m_in_boot_mode = false;                    /**< Current protocol mode. */
uint16_t          m_conn_handle  = BLE_CONN_HANDLE_INVALID;  /**< Handle of the current connection. */
static pm_peer_id_t      m_peer_id;                                 /**< Device reference handle to the current bonded central. */
static bool m_ble_hid_rep_pending = false;

static ble_uuid_t m_adv_uuids[] = {
#ifdef KEYBOARD_PERIPH
    {BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}
#else
    {BLE_UUID_HUMAN_INTERFACE_DEVICE_SERVICE, BLE_UUID_TYPE_BLE}
//,    {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}
    
#endif
};

mouse_report_ble_t mouse_report_ble;

static void on_hids_evt(ble_hids_t * p_hids, ble_hids_evt_t * p_evt);

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for setting filtered whitelist.
 *
 * @param[in] skip  Filter passed to @ref pm_peer_id_list.
 */
static void whitelist_set(pm_peer_id_list_skip_t skip)
{
    pm_peer_id_t peer_ids[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
    uint32_t     peer_id_count = BLE_GAP_WHITELIST_ADDR_MAX_COUNT;

    ret_code_t err_code = pm_peer_id_list(peer_ids, &peer_id_count, PM_PEER_ID_INVALID, skip);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("\tm_whitelist_peer_cnt %d, MAX_PEERS_WLIST %d",
                   peer_id_count + 1,
                   BLE_GAP_WHITELIST_ADDR_MAX_COUNT);

    err_code = pm_whitelist_set(peer_ids, peer_id_count);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for setting filtered device identities.
 *
 * @param[in] skip  Filter passed to @ref pm_peer_id_list.
 */
static void identities_set(pm_peer_id_list_skip_t skip)
{
    pm_peer_id_t peer_ids[BLE_GAP_DEVICE_IDENTITIES_MAX_COUNT];
    uint32_t     peer_id_count = BLE_GAP_DEVICE_IDENTITIES_MAX_COUNT;

    ret_code_t err_code = pm_peer_id_list(peer_ids, &peer_id_count, PM_PEER_ID_INVALID, skip);
    APP_ERROR_CHECK(err_code);

    err_code = pm_device_identities_list_set(peer_ids, peer_id_count);
    APP_ERROR_CHECK(err_code);
}


/**@brief Clear bond information from persistent storage.
 */
static void delete_bonds(void)
{
    ret_code_t err_code;

    NRF_LOG_INFO("Erase bonds!");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);

    pm_whitelist_set(NULL,NULL);
}


/**@brief Function for starting advertising.
 */
void advertising_start(void)
{
    //whitelist_set(PM_PEER_ID_LIST_SKIP_NO_ID_ADDR);
    ret_code_t ret;
    if( ble_conn_state_peripheral_conn_count()==0 ) {
        ret = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
        if( ret != NRF_SUCCESS && ret != NRF_ERROR_INVALID_STATE ) {
            APP_ERROR_CHECK(ret);
        }
        NRF_LOG_INFO("Start Advertising...");
    }
}

void delete_secure_failed_peer(uint16_t conn_handle) {
    ret_code_t err_code;
    pm_peer_id_t peer_id;
    NRF_LOG_INFO("Delete bonding...");
    err_code = pm_peer_id_get(conn_handle, &peer_id);
    APP_ERROR_CHECK(err_code);
    if(peer_id == 0xFFFF) { return; }
    err_code = pm_peer_delete(peer_id);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    pm_handler_on_pm_evt(p_evt);
    pm_handler_flash_clean(p_evt);
    ret_code_t err_code;
    switch (p_evt->evt_id)
    {
        case PM_EVT_CONN_SEC_SUCCEEDED:
            m_peer_id = p_evt->peer_id;
            break;

        case PM_EVT_PEERS_DELETE_SUCCEEDED:
#ifdef KEYBOARD_CENTRAL
            if(ble_conn_state_central_conn_count()) {
#endif
            advertising_start();
#ifdef KEYBOARD_CENTRAL
            }
#endif
            break;
        case PM_EVT_CONN_SEC_FAILED:
            //delete_bonds();
            delete_secure_failed_peer(p_evt->conn_handle);
            break;
        case PM_EVT_CONN_SEC_CONFIG_REQ:
        {
            // Allow pairing request from an already bonded peer.
            pm_conn_sec_config_t conn_sec_config = {.allow_repairing = true};
            pm_conn_sec_config_reply(p_evt->conn_handle, &conn_sec_config);
        } break;

        case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
            if (     p_evt->params.peer_data_update_succeeded.flash_changed
                 && (p_evt->params.peer_data_update_succeeded.data_id == PM_PEER_DATA_ID_BONDING))
            {
                NRF_LOG_INFO("New Bond, add the peer to the whitelist if possible");
                // Note: You should check on what kind of white list policy your application should use.

                whitelist_set(PM_PEER_ID_LIST_SKIP_NO_ID_ADDR);
            }
            break;

        default:
            break;
    }
}


/**@brief Function for handling Service errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void service_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for handling advertising errors.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void ble_advertising_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}



/**@brief Function for handling the HID Report Characteristic Write event.
 *
 * @param[in]   p_evt   HID service event.
 */
static void on_hid_rep_char_write(ble_hids_evt_t * p_evt)
{
    if (p_evt->params.char_write.char_id.rep_type == BLE_HIDS_REP_TYPE_OUTPUT)
    {
        ret_code_t err_code;
        uint8_t  report_val[32];
        uint8_t  report_index = p_evt->params.char_write.char_id.rep_index;
        
        STATIC_ASSERT( OUTPUT_REPORT_MAX_LEN == 1 );
        switch(report_index)
        {
        case OUTPUT_REPORT_KEYS_INDEX:
            // This code assumes that the output report is one byte long. Hence the following
            // static assert is made.
            
            err_code = ble_hids_outp_rep_get(&m_hids,
                                             report_index,
                                             OUTPUT_REPORT_MAX_LEN,
                                             0,
                                             m_conn_handle,
                                             report_val);
            APP_ERROR_CHECK(err_code);
            break;

        case OUTPUT_REPORT_RAW_INDEX:
            err_code = ble_hids_outp_rep_get(&m_hids,
                                             OUTPUT_REPORT_RAW_INDEX,
                                             OUTPUT_REPORT_RAW_MAX_LEN,
                                             0,
                                             m_conn_handle,
                                             report_val);
            APP_ERROR_CHECK(err_code);
            NRF_LOG_INFO("RAW_OUTPUT:");
            NRF_LOG_HEXDUMP_INFO(report_val, OUTPUT_REPORT_RAW_MAX_LEN);
            raw_hid_receive(report_val, OUTPUT_REPORT_RAW_MAX_LEN);
            break;
        }
    }
}



/**@brief Function for handling HID events.
 *
 * @details This function will be called for all HID events which are passed to the application.
 *
 * @param[in]   p_hids  HID service structure.
 * @param[in]   p_evt   Event received from the HID service.
 */
static void on_hids_evt(ble_hids_t * p_hids, ble_hids_evt_t * p_evt)
{
    switch (p_evt->evt_type)
    {
        case BLE_HIDS_EVT_BOOT_MODE_ENTERED:
            m_in_boot_mode = true;
            break;

        case BLE_HIDS_EVT_REPORT_MODE_ENTERED:
            m_in_boot_mode = false;
            break;

        case BLE_HIDS_EVT_REP_CHAR_WRITE:
            on_hid_rep_char_write(p_evt);
            break;

        case BLE_HIDS_EVT_NOTIF_ENABLED:
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    ret_code_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_DIRECTED_HIGH_DUTY:
            NRF_LOG_INFO("High Duty Directed advertising.");
            
            break;

        case BLE_ADV_EVT_DIRECTED:
            NRF_LOG_INFO("Directed advertising.");
            
            break;

        case BLE_ADV_EVT_FAST:
            NRF_LOG_INFO("Fast advertising.");
            
            break;

        case BLE_ADV_EVT_SLOW:
            NRF_LOG_INFO("Slow advertising.");
           
            break;

        case BLE_ADV_EVT_FAST_WHITELIST:
            NRF_LOG_INFO("Fast advertising with whitelist.");
            
            break;

        case BLE_ADV_EVT_SLOW_WHITELIST:
            NRF_LOG_INFO("Slow advertising with whitelist.");
            
            break;

        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter(NULL);
            break;

        case BLE_ADV_EVT_WHITELIST_REQUEST:
        {
            NRF_LOG_INFO("BLE_ADV_EVT_WHITELIST_REQUEST");
            ble_gap_addr_t whitelist_addrs[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
            ble_gap_irk_t  whitelist_irks[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
            uint32_t       addr_cnt = BLE_GAP_WHITELIST_ADDR_MAX_COUNT;
            uint32_t       irk_cnt  = BLE_GAP_WHITELIST_ADDR_MAX_COUNT;

            err_code = pm_whitelist_get(whitelist_addrs, &addr_cnt,
                                        whitelist_irks,  &irk_cnt);
            APP_ERROR_CHECK(err_code);
            NRF_LOG_DEBUG("pm_whitelist_get returns %d addr in whitelist and %d irk whitelist",
                          addr_cnt, irk_cnt);

            // Set the correct identities list (no excluding peers with no Central Address Resolution).
            identities_set(PM_PEER_ID_LIST_SKIP_NO_IRK);

            // Apply the whitelist.
            err_code = ble_advertising_whitelist_reply(&m_advertising,
                                                       whitelist_addrs,
                                                       addr_cnt,
                                                       whitelist_irks,
                                                       irk_cnt);
             
            APP_ERROR_CHECK(err_code);
        } break; //BLE_ADV_EVT_WHITELIST_REQUEST

        case BLE_ADV_EVT_PEER_ADDR_REQUEST:
        {
            NRF_LOG_INFO("BLE_ADV_EVT_PEER_ADDR_REQUEST");
            pm_peer_data_bonding_t peer_bonding_data;

            // Only Give peer address if we have a handle to the bonded peer.
            if (m_peer_id != PM_PEER_ID_INVALID)
            {
                err_code = pm_peer_data_bonding_load(m_peer_id, &peer_bonding_data);
                if (err_code != NRF_ERROR_NOT_FOUND)
                {
                    APP_ERROR_CHECK(err_code);

                    // Manipulate identities to exclude peers with no Central Address Resolution.
                    identities_set(PM_PEER_ID_LIST_SKIP_ALL);

                    ble_gap_addr_t * p_peer_addr = &(peer_bonding_data.peer_ble_id.id_addr_info);
                    err_code = ble_advertising_peer_addr_reply(&m_advertising, p_peer_addr);
                    APP_ERROR_CHECK(err_code);
                }
            }
        } break; //BLE_ADV_EVT_PEER_ADDR_REQUEST

        default:
            break;
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_p_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected");
            
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected");
            // Dequeue all keys without transmission.
            
            m_conn_handle = BLE_CONN_HANDLE_INVALID;

            // Reset m_caps_on variable. Upon reconnect, the HID host will re-send the Output
            // report containing the Caps lock state.
            
            //APP_ERROR_CHECK(err_code);

            break; // BLE_GAP_EVT_DISCONNECTED

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

        case BLE_GATTS_EVT_HVN_TX_COMPLETE:
            // Send next key event
            m_ble_hid_rep_pending = false;
            break;

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
            // No implementation needed.
            break;
    }
}

static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    uint16_t conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
#ifdef KEYBOARD_CENTRAL
    uint16_t role = ble_conn_state_role(conn_handle);

    if( (role == BLE_GAP_ROLE_PERIPH) || (p_ble_evt->header.evt_id == BLE_GAP_EVT_ADV_SET_TERMINATED) ) {
#endif
        ble_p_evt_handler(p_ble_evt, p_context);
#ifdef KEYBOARD_CENTRAL
    } else if( (role==BLE_GAP_ROLE_CENTRAL) || (p_ble_evt->header.evt_id == BLE_GAP_EVT_ADV_REPORT) ) {
        ble_c_evt_handler(p_ble_evt, p_context);
    }
#endif
}
/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for the Event Scheduler initialization.
 */
static void scheduler_init(void)
{
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}


/**@brief Function for the Peer Manager initialization.
 */
static void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t               err_code;
    uint8_t                adv_flags;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    adv_flags                            = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance      = true;
    init.advdata.flags                   = adv_flags;
    init.advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.advdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_whitelist_enabled          = true;
    init.config.ble_adv_directed_high_duty_enabled = true;
    init.config.ble_adv_directed_enabled           = false;
    init.config.ble_adv_directed_interval          = 0;
    init.config.ble_adv_directed_timeout           = 0;
    init.config.ble_adv_fast_enabled               = true;
    init.config.ble_adv_fast_interval              = APP_ADV_FAST_INTERVAL;
    init.config.ble_adv_fast_timeout               = APP_ADV_FAST_DURATION;
    init.config.ble_adv_slow_enabled               = true;
    init.config.ble_adv_slow_interval              = APP_ADV_SLOW_INTERVAL;
    init.config.ble_adv_slow_timeout               = APP_ADV_SLOW_DURATION;

    init.evt_handler   = on_adv_evt;
    init.error_handler = ble_advertising_error_handler;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing Device Information Service.
 */
static void dis_init(void)
{
    ret_code_t       err_code;
    ble_dis_init_t   dis_init_obj;
    ble_dis_pnp_id_t pnp_id;

    pnp_id.vendor_id_source = PNP_ID_VENDOR_ID_SOURCE;
    pnp_id.vendor_id        = PNP_ID_VENDOR_ID;
    pnp_id.product_id       = PNP_ID_PRODUCT_ID;
    pnp_id.product_version  = PNP_ID_PRODUCT_VERSION;

    memset(&dis_init_obj, 0, sizeof(dis_init_obj));

    ble_srv_ascii_to_utf8(&dis_init_obj.manufact_name_str, MANUFACTURER_NAME);
    dis_init_obj.p_pnp_id = &pnp_id;

    dis_init_obj.dis_char_rd_sec = SEC_JUST_WORKS;

    err_code = ble_dis_init(&dis_init_obj);
    APP_ERROR_CHECK(err_code);
}



/**@brief Function for initializing HID Service.
 */
static void hids_init(void)
{
    ret_code_t                    err_code;
    ble_hids_init_t               hids_init_obj;
    ble_hids_inp_rep_init_t     * p_input_report;
    ble_hids_outp_rep_init_t    * p_output_report;
    ble_hids_feature_rep_init_t * p_feature_report;
    uint8_t                       hid_info_flags;

    static ble_hids_inp_rep_init_t     input_report_array[3];
    static ble_hids_outp_rep_init_t    output_report_array[2];
    static ble_hids_feature_rep_init_t feature_report_array[1];
    static uint8_t                     report_map_data[] =
    {
        0x05, 0x01,       // Usage Page (Generic Desktop)
        0x09, 0x06,       // Usage (Keyboard)
        0xA1, 0x01,       // Collection (Application)
        0x85, INPUT_REP_REF_ID,
        0x05, 0x07,       // Usage Page (Key Codes)
        0x19, 0xe0,       // Usage Minimum (224)
        0x29, 0xe7,       // Usage Maximum (231)
        0x15, 0x00,       // Logical Minimum (0)
        0x25, 0x01,       // Logical Maximum (1)
        0x75, 0x01,       // Report Size (1)
        0x95, 0x08,       // Report Count (8)
        0x81, 0x02,       // Input (Data, Variable, Absolute)

        0x95, 0x01,       // Report Count (1)
        0x75, 0x08,       // Report Size (8)
        0x81, 0x01,       // Input (Constant) reserved byte(1)

        0x95, 0x05,       // Report Count (5)
        0x75, 0x01,       // Report Size (1)
        0x05, 0x08,       // Usage Page (Page# for LEDs)
        0x19, 0x01,       // Usage Minimum (1)
        0x29, 0x05,       // Usage Maximum (5)
        0x91, 0x02,       // Output (Data, Variable, Absolute), Led report
        0x95, 0x01,       // Report Count (1)
        0x75, 0x03,       // Report Size (3)
        0x91, 0x01,       // Output (Data, Variable, Absolute), Led report padding

        0x95, 0x06,       // Report Count (6)
        0x75, 0x08,       // Report Size (8)
        0x15, 0x00,       // Logical Minimum (0)
        0x25, 0x65,       // Logical Maximum (101)
        0x05, 0x07,       // Usage Page (Key codes)
        0x19, 0x00,       // Usage Minimum (0)
        0x29, 0x65,       // Usage Maximum (101)
        0x81, 0x00,       // Input (Data, Array) Key array(6 bytes)
        
        
        0xC0

// Mouse Definision
        ,
        0x05, 0x01,       /* Usage Page (Generic Desktop),       */     
        0x09, 0x02,       /* Usage (Mouse),                      */     
        0xA1, 0x01,       /*  Collection (Application),          */     
        0x85, INPUT_REP_REF_MOUSE_ID,
        0x09, 0x01,       /*   Usage (Pointer),                  */     
        0xA1, 0x00,       /*  Collection (Physical),             */     
        0x05, 0x09,       /*     Usage Page (Buttons),           */     
        0x19, 0x01,       /*     Usage Minimum (01),             */     
        0x29, 0x03,       /*     Usage Maximum (bcnt),           */     
        0x15, 0x00,       /*     Logical Minimum (0),            */     
        0x25, 0x01,       /*     Logical Maximum (1),            */     
        0x75, 0x01,       /*     Report Size (1),                */     
        0x95, 0x03,       /*     Report Count (bcnt),            */     
        0x81, 0x02,       /*     Input (Data, Variable, Absolute)*/     
        0x75, 0x05,       /*     Report Size (8-(bcnt)),         */     
        0x95, 0x01,       /*     Report Count (1),               */     
        0x81, 0x01,       /*     Input (Constant),               */     
        0x05, 0x01,       /*     Usage Page (Generic Desktop),   */     
        0x09, 0x30,       /*     Usage (X),                      */     
        0x09, 0x31,       /*     Usage (Y),                      */     
        0x09, 0x38,       /*     Usage (Scroll),                 */     
        0x15, 0x81,       /*     Logical Minimum (-127),         */     
        0x25, 0x7F,       /*     Logical Maximum (127),          */     
        0x75, 0x08,       /*     Report Size (8),                */     
        0x95, 0x03,       /*     Report Count (3),               */     
        0x81, 0x06,       /*     Input (Data, Variable, Relative)*/     
        0xC0,         /*  End Collection,                        */     
        0xC0         /* End Collection                          */     
#if 1
        ,
        0x06, 0x60, 0xFF,
        0x09, 0x61,
        0xa1, 0x01,
        0x85, INPUT_REP_REF_RAW_ID,
        
        0x09, 0x62, 
        0x15, 0x00, 
        0x26, 0xFF, 0x00, 
        0x95, INPUT_REPORT_RAW_MAX_LEN,
        0x75, 0x08, 
        0x81, 0x06, 
      
        0x09, 0x63, 
        0x15, 0x00, 
        0x26, 0xFF, 0x00, 
        0x95, OUTPUT_REPORT_RAW_MAX_LEN, //REPORT_COUNT(32)
        0x75, 0x08, //REPORT_SIZE(8)
        0x91, 0x83, 
        0xC0             // End Collection (Application)

        // RAW HID
#endif
        
        
    };

    memset((void *)input_report_array, 0, sizeof(input_report_array));
    memset((void *)output_report_array, 0, sizeof(output_report_array));
    memset((void *)feature_report_array, 0, sizeof(ble_hids_feature_rep_init_t));

    // Initialize HID Service
    p_input_report                      = &input_report_array[INPUT_REPORT_KEYS_INDEX];
    p_input_report->max_len             = INPUT_REPORT_KEYS_MAX_LEN;
    p_input_report->rep_ref.report_id   = INPUT_REP_REF_ID;
    p_input_report->rep_ref.report_type = BLE_HIDS_REP_TYPE_INPUT;

    p_input_report->sec.cccd_wr = SEC_JUST_WORKS;
    p_input_report->sec.wr      = SEC_JUST_WORKS;
    p_input_report->sec.rd      = SEC_JUST_WORKS;

    p_output_report                      = &output_report_array[OUTPUT_REPORT_KEYS_INDEX];
    p_output_report->max_len             = OUTPUT_REPORT_MAX_LEN;
    p_output_report->rep_ref.report_id   = OUTPUT_REP_REF_ID;
    p_output_report->rep_ref.report_type = BLE_HIDS_REP_TYPE_OUTPUT;

    p_output_report->sec.wr = SEC_JUST_WORKS;
    p_output_report->sec.rd = SEC_JUST_WORKS;
    // Mouse Setting
    p_input_report                      = &input_report_array[INPUT_REPORT_MOUSE_INDEX];
    p_input_report->max_len             = INPUT_REPORT_MOUSE_MAX_LEN;
    p_input_report->rep_ref.report_id   = INPUT_REP_REF_MOUSE_ID;
    p_input_report->rep_ref.report_type = BLE_HIDS_REP_TYPE_INPUT;

    p_input_report->sec.cccd_wr = SEC_JUST_WORKS;
    p_input_report->sec.wr      = SEC_JUST_WORKS;
    p_input_report->sec.rd      = SEC_JUST_WORKS;

    // RAW Setting
    p_input_report                      = &input_report_array[INPUT_REPORT_RAW_INDEX];
    p_input_report->max_len             = INPUT_REPORT_RAW_MAX_LEN;
    p_input_report->rep_ref.report_id   = INPUT_REP_REF_RAW_ID;
    p_input_report->rep_ref.report_type = BLE_HIDS_REP_TYPE_INPUT;

    p_input_report->sec.cccd_wr = SEC_JUST_WORKS;
    p_input_report->sec.wr      = SEC_JUST_WORKS;
    p_input_report->sec.rd      = SEC_JUST_WORKS;

    p_output_report                      = &output_report_array[OUTPUT_REPORT_RAW_INDEX];
    p_output_report->max_len             = OUTPUT_REPORT_RAW_MAX_LEN;
    p_output_report->rep_ref.report_id   = OUTPUT_REP_REF_RAW_ID;
    p_output_report->rep_ref.report_type = BLE_HIDS_REP_TYPE_OUTPUT;

    p_output_report->sec.wr = SEC_JUST_WORKS;
    p_output_report->sec.rd = SEC_JUST_WORKS;
    // RAW Setting end

    p_feature_report                      = &feature_report_array[FEATURE_REPORT_INDEX];
    p_feature_report->max_len             = FEATURE_REPORT_MAX_LEN;
    p_feature_report->rep_ref.report_id   = FEATURE_REP_REF_ID;
    p_feature_report->rep_ref.report_type = BLE_HIDS_REP_TYPE_FEATURE;

    p_feature_report->sec.rd              = SEC_JUST_WORKS;
    p_feature_report->sec.wr              = SEC_JUST_WORKS;

    hid_info_flags = HID_INFO_FLAG_REMOTE_WAKE_MSK | HID_INFO_FLAG_NORMALLY_CONNECTABLE_MSK;

    memset(&hids_init_obj, 0, sizeof(hids_init_obj));

    hids_init_obj.evt_handler                    = on_hids_evt;
    hids_init_obj.error_handler                  = service_error_handler;
    hids_init_obj.is_kb                          = true;
    hids_init_obj.is_mouse                       = false;
#if 1
    hids_init_obj.inp_rep_count                  = 3;
    hids_init_obj.outp_rep_count                 = 2;
#else
    hids_init_obj.inp_rep_count                  = 1;
    hids_init_obj.outp_rep_count                 = 1;
#endif
    hids_init_obj.p_inp_rep_array                = input_report_array;
    hids_init_obj.p_outp_rep_array               = output_report_array;
    hids_init_obj.feature_rep_count              = 1;
    hids_init_obj.p_feature_rep_array            = feature_report_array;
    hids_init_obj.rep_map.data_len               = sizeof(report_map_data);
    hids_init_obj.rep_map.p_data                 = report_map_data;
    hids_init_obj.hid_information.bcd_hid        = BASE_USB_HID_SPEC_VERSION;
    hids_init_obj.hid_information.b_country_code = 0;
    hids_init_obj.hid_information.flags          = hid_info_flags;
    hids_init_obj.included_services_count        = 0;
    hids_init_obj.p_included_services_array      = NULL;

    hids_init_obj.rep_map.rd_sec         = SEC_JUST_WORKS;
    hids_init_obj.hid_information.rd_sec = SEC_JUST_WORKS;

    hids_init_obj.boot_kb_inp_rep_sec.cccd_wr = SEC_JUST_WORKS;
    hids_init_obj.boot_kb_inp_rep_sec.rd      = SEC_JUST_WORKS;

    hids_init_obj.boot_kb_outp_rep_sec.rd = SEC_JUST_WORKS;
    hids_init_obj.boot_kb_outp_rep_sec.wr = SEC_JUST_WORKS;

    hids_init_obj.protocol_mode_rd_sec = SEC_JUST_WORKS;
    hids_init_obj.protocol_mode_wr_sec = SEC_JUST_WORKS;
    hids_init_obj.ctrl_point_wr_sec    = SEC_JUST_WORKS;

    err_code = ble_hids_init(&m_hids, &hids_init_obj);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for initializing the Queued Write Module.
 */
static void qwr_init(void)
{
    ret_code_t         err_code;
    nrf_ble_qwr_init_t qwr_init_obj = {0};

    qwr_init_obj.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init_obj);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */

ble_gap_conn_params_t   gap_conn_params;

void ble_common_init(void) {
    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;
}

static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_HID_KEYBOARD);
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}



/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{ 
    qwr_init();
#ifdef KEYBOARD_PERIPH
    periph_nus_init();
#else
    
    dis_init();
    hids_init();
#endif
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = NULL;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

ret_code_t ble_mouse_init(void) {
    memset(&mouse_report_ble, 0, sizeof(mouse_report_ble));
    return NRF_SUCCESS;
}

uint8_t ble_keyboard_rep_buffer[BLE_HID_KBD_REP_LEN];

void ble_keyboard_init(void) {
    ble_conn_state_init();
    power_management_init();
    peer_manager_init();
    ble_stack_init();
    scheduler_init();
    gap_params_init();
    gatt_init();
    
    services_init();
    conn_params_init();
    advertising_init();

    memset(ble_keyboard_rep_buffer, 0, sizeof(ble_keyboard_rep_buffer));
    ble_mouse_init();
}

ret_code_t raw_hid_send_ble(uint8_t *data, uint8_t length) {
    return ble_hids_inp_rep_send(&m_hids,
                                 INPUT_REPORT_RAW_INDEX,
                                 INPUT_REPORT_RAW_MAX_LEN,
                                 data,
                                 m_conn_handle);
}

ret_code_t keyboard_report_send_ble(void) {
    while(m_ble_hid_rep_pending) {};
    return ble_hids_inp_rep_send(&m_hids,
                                 INPUT_REPORT_KEYS_INDEX,
                                 BLE_HID_KBD_REP_LEN,
                                 ble_keyboard_rep_buffer,
                                 m_conn_handle);
}

ret_code_t keycode_append_ble(uint8_t kc) {
    bool updated = false;
    if( (kc&0xF8) == 0xe0 ) {
        // modifiers
        if( ble_keyboard_rep_buffer[BLE_HID_KBD_MODS_INDEX] & ( 1<<(kc-0xe0) ) ) {
        
        } else {
            ble_keyboard_rep_buffer[BLE_HID_KBD_MODS_INDEX] |= ( 1<<(kc-0xe0) );
            updated = true;
        }
    } else
    {
        for(uint8_t i = BLE_HID_KBD_KEYS_START; i<BLE_HID_KBD_REP_LEN; i++) {
            if(ble_keyboard_rep_buffer[i]==0x00) {
                ble_keyboard_rep_buffer[i] = kc;
                updated = true;
                break;
            }
            
            if(ble_keyboard_rep_buffer[i]==kc) {
                break;
            }
        }
    }
    
    if( updated ) {
        keyboard_report_send_ble();
    }
    return NRF_SUCCESS;
}

ret_code_t keycode_remove_ble(uint8_t kc) {
    bool after_kc = false;
    bool updated = false;
    if( (kc&0xF8) == 0xe0 ) {
        // modifiers
        if( ble_keyboard_rep_buffer[BLE_HID_KBD_MODS_INDEX] & ( 1<<(kc&0x07) ) ) {
            ble_keyboard_rep_buffer[BLE_HID_KBD_MODS_INDEX] &= ~( 1<<(kc&0x07) );
            updated = true;
        }
    }
    else {
        for(uint8_t i = BLE_HID_KBD_KEYS_START; i<BLE_HID_KBD_REP_LEN; i++) {
            if(ble_keyboard_rep_buffer[i]==0x00) {
                break;
            }

            if(after_kc) {
                ble_keyboard_rep_buffer[i-1] = ble_keyboard_rep_buffer[i];
                ble_keyboard_rep_buffer[i] = 0x00;
            }

            if(ble_keyboard_rep_buffer[i]==kc) {
                after_kc = true;
                updated = true;
                ble_keyboard_rep_buffer[i] = 0x00; // for if i is the last
            }
        }
    }
    if(updated) {
        keyboard_report_send_ble();
        //restart_timeout_timer();
    }
    return NRF_SUCCESS;
}

ret_code_t keyboard_reset_ble(void) {
    memset(ble_keyboard_rep_buffer, 0, sizeof(ble_keyboard_rep_buffer));
    keyboard_report_send_ble();
    return NRF_SUCCESS;
}

ret_code_t handle_keycode_ble(uint16_t keycode, bool press) {
    return NRF_SUCCESS;
}

// Mouse Functions
ret_code_t mouse_report_send_ble(void) {
    return ble_hids_inp_rep_send(&m_hids,
                                 INPUT_REPORT_MOUSE_INDEX,
                                 INPUT_REPORT_MOUSE_MAX_LEN,
                                 (uint8_t *) &mouse_report_ble,
                                 m_conn_handle);
}

ret_code_t mouse_reset_ble(void) {
    memset(&mouse_report_ble, 0, sizeof(mouse_report_ble));
    mouse_report_send_ble();
    return NRF_SUCCESS;
}

ret_code_t handle_keycode_mouse_ble(uint16_t keycode, bool press) {
    if( (keycode&0x00F0)!=0x00F0 ) {
        return NRF_ERROR_INVALID_PARAM;
    }

    if( press ) { 
        switch(keycode) {
        
        case 0xF0:
            mouse_report_ble.y = -(MOUSE_MOVE_DISTANCE>>1);
            break;
        case 0xF1:
            mouse_report_ble.y =  MOUSE_MOVE_DISTANCE>>1;
            break;
        case 0xF2:
            mouse_report_ble.x = -(MOUSE_MOVE_DISTANCE>>1);
            break;
        case 0xF3:
            mouse_report_ble.x = MOUSE_MOVE_DISTANCE>>1;
            break;
        case 0xF4:
        case 0xF5:
        case 0xF6:
        case 0xF7:
        case 0xF8:
            mouse_report_ble.buttons |= 1 << (keycode - 0xF4);
            break;
        case 0xF9:
            mouse_report_ble.wheel = WHEEL_MOVE_DISTANCE>>1;
            break;
        case 0xFA:
            mouse_report_ble.wheel = -(WHEEL_MOVE_DISTANCE>>1);
            break;
        }
    } else {
        switch(keycode) {
        case 0xF0:
        case 0xF1:
            mouse_report_ble.y = 0;
            break;
        case 0xF2:
        case 0xF3:
            mouse_report_ble.x = 0;
            break;
        case 0xF4:
        case 0xF5:
        case 0xF6:
        case 0xF7:
        case 0xF8:
            mouse_report_ble.buttons &= 0 << (keycode - 0xF4);
            break;
        case 0xF9:
        case 0xFA:
            mouse_report_ble.wheel = 0;
            break;
        }
    }
    return mouse_report_send_ble();
}


ret_code_t tick_handler_mouse_ble(keys_t *p_key) {
    if( (p_key->kc&0x00F0)!=0x00F0 ) {
        return NRF_ERROR_INVALID_PARAM;
    }
    
    if(! (p_key->tick % MOUSE_MOVE_INTERVAL_TICKS) ) {
        mouse_report_send_ble();
    }
    return NRF_SUCCESS;
}


