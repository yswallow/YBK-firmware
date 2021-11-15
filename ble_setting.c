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
//#include "fds.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"
#include "peer_manager_handler.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrf_delay.h"
#include "app_timer.h"

#include "ble_hiddevice.h"
#include "ble_setting.h"
//#include "main.h"
#include "keyboard_generic.h"
#include "via.h"

#ifdef KEYBOARD_CENTRAL
#include "ble_central.h"
#else

#endif

#ifdef KEYBOARD_PERIPH
#include "ble_nus.h"
#include "ble_peripheral.h"
#else
#include "ble_bas.h"
#include "ble_hiddevice.h"
#endif


#ifndef KEYBOARD_PERIPH
BLE_BAS_DEF(m_bas);
#endif

NRF_BLE_GATT_DEF(m_gatt);                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                 /**< Advertising module instance. */

ble_gap_conn_params_t   gap_conn_params;
uint16_t                  m_conn_handle  = BLE_CONN_HANDLE_INVALID;  /**< Handle of the current connection. */
static pm_peer_id_t       m_peer_id;                                 /**< Device reference handle to the current bonded central. */
static bool m_disconnected_by_user = false;

static void set_advertising_conf_default(void);

#ifdef KEYBOARD_PERIPH
static ble_uuid_t m_adv_uuids[] = { {BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE} };
#else

static ble_uuid_t m_adv_uuids[] = {
    {BLE_UUID_HUMAN_INTERFACE_DEVICE_SERVICE, BLE_UUID_TYPE_BLE}
,   {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}
};

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

static void bas_init(void)
{
    ret_code_t     err_code;
    ble_bas_init_t bas_init_obj;

    memset(&bas_init_obj, 0, sizeof(bas_init_obj));

    bas_init_obj.evt_handler          = NULL;
    bas_init_obj.support_notification = true;
    bas_init_obj.p_report_ref         = NULL;
    bas_init_obj.initial_batt_level   = 100;

    bas_init_obj.bl_rd_sec        = SEC_JUST_WORKS;
    bas_init_obj.bl_cccd_wr_sec   = SEC_JUST_WORKS;
    bas_init_obj.bl_report_rd_sec = SEC_JUST_WORKS;

    err_code = ble_bas_init(&m_bas, &bas_init_obj);
    APP_ERROR_CHECK(err_code);
}
#endif // n KEYBOARD_PERIPH


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
    //APP_ERROR_CHECK(err_code);
}

static int whitelist_set_one_device(uint8_t index)
{
    pm_peer_id_t peer_ids[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
    uint32_t     peer_id_count = BLE_GAP_WHITELIST_ADDR_MAX_COUNT;

    ret_code_t err_code = pm_peer_id_list(peer_ids, &peer_id_count, PM_PEER_ID_INVALID, PM_PEER_ID_LIST_SKIP_NO_ID_ADDR);
    APP_ERROR_CHECK(err_code);
    if(index<peer_id_count) {
        err_code = pm_whitelist_set(peer_ids+index, 1);
        APP_ERROR_CHECK(err_code);
        err_code = pm_device_identities_list_set(peer_ids+index, 1);
        APP_ERROR_CHECK(err_code);
        return index;
    } else {
    /*
        err_code = pm_whitelist_set(NULL, 0);
        APP_ERROR_CHECK(err_code);
        err_code = pm_device_identities_list_set(NULL, 0);
        APP_ERROR_CHECK(err_code);
    */
        return -1;
    }
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
void advertising_start(bool set_whitelist, ble_adv_mode_t adv_mode)
{
    ret_code_t ret;

    if(set_whitelist) {
        whitelist_set(PM_PEER_ID_LIST_SKIP_NO_ID_ADDR);
    }
    
    if( ble_conn_state_peripheral_conn_count()==0 ) {
        //while(m_conn_handle!=BLE_CONN_HANDLE_INVALID) {}
        ret = ble_advertising_start(&m_advertising, adv_mode);    
        if( ret == NRF_SUCCESS ) {
            NRF_LOG_INFO("Start Advertising...");
        } else {
            NRF_LOG_INFO("Advertising start error");
        }
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
    pm_handler_disconnect_on_sec_failure(p_evt);
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
            if(! m_disconnected_by_user ) {
                
                advertising_start(true, BLE_ADV_MODE_DIRECTED_HIGH_DUTY);
            }
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



/**@brief Function for handling advertising errors.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void ble_advertising_error_handler(uint32_t nrf_error)
{
    //APP_ERROR_HANDLER(nrf_error);
}

void advertising_without_whitelist(ble_adv_mode_t mode);

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
            err_code = sd_ble_gap_adv_stop(m_advertising.adv_handle);
            APP_ERROR_CHECK(err_code);
            advertising_without_whitelist(BLE_ADV_MODE_FAST);
            //err_code = ble_advertising_restart_without_whitelist(&m_advertising);
            //APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter(NULL);
            break;

#ifndef KEYBOARD_PERIPH
        case BLE_ADV_EVT_WHITELIST_REQUEST:
        {
            NRF_LOG_INFO("BLE_ADV_EVT_WHITELIST_REQUEST");
            ble_gap_addr_t whitelist_addrs[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
            ble_gap_irk_t  whitelist_irks[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
            uint32_t       addr_cnt = BLE_GAP_WHITELIST_ADDR_MAX_COUNT;
            uint32_t       irk_cnt  = BLE_GAP_WHITELIST_ADDR_MAX_COUNT;

            err_code = pm_whitelist_get(whitelist_addrs, &addr_cnt,
                                        whitelist_irks,  &irk_cnt);
            if(err_code==5) { // NRF_ERROR_NOT_FOUND
                break;
            }
            APP_ERROR_CHECK(err_code);
            NRF_LOG_DEBUG("pm_whitelist_get returns %d addr in whitelist and %d irk whitelist",
                          addr_cnt, irk_cnt);

            // Set the correct identities list (no excluding peers with no Central Address Resolution).
            //identities_set(PM_PEER_ID_LIST_SKIP_NO_IRK);

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
                    //identities_set(PM_PEER_ID_LIST_SKIP_ALL);

                    ble_gap_addr_t * p_peer_addr = &(peer_bonding_data.peer_ble_id.id_addr_info);
                    err_code = ble_advertising_peer_addr_reply(&m_advertising, p_peer_addr);
                    APP_ERROR_CHECK(err_code);
                }
            }
        } break; //BLE_ADV_EVT_PEER_ADDR_REQUEST
#endif
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
            NRF_LOG_HEXDUMP_INFO( &(p_ble_evt->evt.gap_evt.params.connected.peer_addr.addr), 6);
            
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            m_disconnected_by_user = false;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);

            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected");
            NRF_LOG_HEXDUMP_DEBUG( &(p_ble_evt->evt.gap_evt.params.disconnected.reason), 1);
            /*
            if( p_ble_evt->evt.gap_evt.params.disconnected.reason == BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION ) {
                pm_peer_id_t peer_id;
                err_code = pm_peer_id_get(m_conn_handle, &peer_id);
                APP_ERROR_CHECK(err_code);
                err_code = pm_peer_delete(peer_id);
                APP_ERROR_CHECK(err_code);
                NRF_LOG_INFO("Delete peer");
            }
            */
            m_conn_handle = BLE_CONN_HANDLE_INVALID;

#if 0
            NRF_POWER->GPREGRET = 0x6d;
            sd_nvic_SystemReset();
#endif
            if(! m_disconnected_by_user) {
                set_advertising_conf_default();
                advertising_start(true, BLE_ADV_MODE_DIRECTED_HIGH_DUTY);
            }
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

#ifndef KEYBOARD_PERIPH
        case BLE_GATTS_EVT_HVN_TX_COMPLETE:
            // Send next key event
            m_ble_hid_rep_pending = false;
            break;
#endif
        case BLE_GATTS_EVT_SYS_ATTR_MISSING :
            NRF_LOG_DEBUG("BLE_GATTS_EVT_SYS_ATTR_MISSING");
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
#ifdef KEYBOARD_CENTRAL
    uint16_t conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
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


static void get_advertising_conf_default(ble_adv_modes_config_t *pt) {   
    memset(pt, 0, sizeof(ble_adv_modes_config_t));
#ifdef KEYBOARD_PERIPH
    pt->ble_adv_on_disconnect_disabled     = false;
    pt->ble_adv_slow_enabled               = false;
#else
    pt->ble_adv_slow_enabled               = true;
    pt->ble_adv_on_disconnect_disabled     = true;
#endif
    pt->ble_adv_whitelist_enabled          = false;
    pt->ble_adv_directed_high_duty_enabled = true;
    pt->ble_adv_directed_enabled           = true;
    pt->ble_adv_directed_interval          = 0x10;
    pt->ble_adv_directed_timeout           = 500;
    pt->ble_adv_fast_enabled               = true;
    pt->ble_adv_fast_interval              = APP_ADV_FAST_INTERVAL;
    pt->ble_adv_fast_timeout               = APP_ADV_FAST_DURATION;
    pt->ble_adv_slow_interval              = APP_ADV_SLOW_INTERVAL;
    pt->ble_adv_slow_timeout               = APP_ADV_SLOW_DURATION;
}


static void set_advertising_conf_default(void) {
    ble_adv_modes_config_t conf;
    get_advertising_conf_default(&conf);
    ble_advertising_modes_config_set(&m_advertising, &conf);
}

void advertising_without_whitelist(ble_adv_mode_t mode) {
    ble_adv_modes_config_t conf;
    sd_ble_gap_adv_stop(m_advertising.adv_handle);
    get_advertising_conf_default(&conf);
    conf.ble_adv_whitelist_enabled = false;
    ble_advertising_modes_config_set(&m_advertising, &conf);
    advertising_start(true, mode);
}


/**@brief Function for initializing the Advertising functionality.
 */
// call after service_init()
static void advertising_init(void)
{
    uint32_t               err_code;
    uint8_t                adv_flags;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    adv_flags                            = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.advdata.name_type               = BLE_ADVDATA_SHORT_NAME;
    init.advdata.short_name_len          = 4;
    init.advdata.include_appearance      = true;
    init.advdata.flags                   = adv_flags;
    init.advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.advdata.uuids_complete.p_uuids  = m_adv_uuids;
    
    get_advertising_conf_default(&(init.config));

    init.evt_handler   = on_adv_evt;
    init.error_handler = ble_advertising_error_handler;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
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
    
    // from ble_common_init() 
    memset(&gap_conn_params, 0, sizeof(gap_conn_params));
    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;
    // ble_common_init() end

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
    bas_init();
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


void ble_device_init(void) {
    ble_conn_state_init();
    ble_stack_init();
    scheduler_init();
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();
    peer_manager_init();

#ifndef KEYBOARD_PERIPH
    ble_keyboard_init();
#endif
}


void ble_connect_to_device(uint8_t index) {
    ret_code_t ret;
    ble_adv_modes_config_t conf;
    
    if( m_conn_handle != BLE_CONN_HANDLE_INVALID ) {
        ret = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(ret);
    }
    m_disconnected_by_user = true;
    ret = sd_ble_gap_adv_stop(m_advertising.adv_handle);
    //APP_ERROR_CHECK(ret);
    
    memset(&conf, 0, sizeof(conf));

    conf.ble_adv_on_disconnect_disabled = true;
    conf.ble_adv_directed_high_duty_enabled = false;
    conf.ble_adv_directed_enabled           = false;
    conf.ble_adv_directed_interval          = 0;
    conf.ble_adv_directed_timeout           = 0;
    conf.ble_adv_fast_enabled               = true;
    conf.ble_adv_fast_interval              = APP_ADV_FAST_INTERVAL;
    conf.ble_adv_fast_timeout               = APP_ADV_FAST_DURATION;
    conf.ble_adv_slow_enabled               = true;
    conf.ble_adv_slow_interval              = APP_ADV_SLOW_INTERVAL;
    conf.ble_adv_slow_timeout               = APP_ADV_SLOW_DURATION;

    if( whitelist_set_one_device(index) != -1 ){
        // valid target
        conf.ble_adv_whitelist_enabled          = true;
    } else {
        // new pair
        conf.ble_adv_whitelist_enabled          = false;
    }
    ble_advertising_modes_config_set(&m_advertising, &conf);
    
    advertising_start(false, BLE_ADV_MODE_FAST);
}


