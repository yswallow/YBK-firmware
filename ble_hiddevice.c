#ifndef KEYBOARD_PERIPH
#include <string.h>
#include "app_error.h"
#include "ble_hids.h"
#include "nrf_log.h"

#include "keyboard_generic.h"
#include "ble_hiddevice.h"
#include "ble_setting.h"
#include "via.h"


BLE_HIDS_DEF(m_hids,                                                /**< Structure used to identify the HID service. */
             NRF_SDH_BLE_TOTAL_LINK_COUNT,
             INPUT_REPORT_RAW_MAX_LEN,OUTPUT_REPORT_RAW_MAX_LEN, FEATURE_REPORT_MAX_LEN);

             
static bool               m_in_boot_mode = false;                    /**< Current protocol mode. */
bool               m_ble_hid_rep_pending = false;
mouse_report_ble_t mouse_report_ble;
uint8_t ble_keyboard_rep_buffer[BLE_HID_KBD_REP_LEN];


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


/**@brief Function for initializing HID Service.
 */
void hids_init(void)
{
    ret_code_t                    err_code;
    ble_hids_init_t               hids_init_obj;
    ble_hids_inp_rep_init_t     * p_input_report;
    ble_hids_outp_rep_init_t    * p_output_report;
    uint8_t                       hid_info_flags;

    static ble_hids_inp_rep_init_t     input_report_array[BLE_INPUT_REPORT_COUNT];
    static ble_hids_outp_rep_init_t    output_report_array[2];
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
        0x25, 0x98,       // Logical Maximum (101)
        0x05, 0x07,       // Usage Page (Key codes)
        0x19, 0x00,       // Usage Minimum (0)
        0x29, 0x98,       // Usage Maximum (101)
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

#ifdef BLE_CONSUMER_ENABLE
        ,
        0x05, 0x0C,
        0x09, 0x01,
        0xA1, 0x01,
        0x85, INPUT_REPORT_CONSUMER_ID,
        0x09, 0x40, // Menu
        0x09, 0xE9, // VOLU
        0x09, 0xEA, // VOLD
        0x09, 0xE2, // Mute
        0x09, 0x30, // Power
        0x09, 0xCD, // Play/Pause
        0x09, 0xB3, // Fast Forward
        0x09, 0xB4, // Rewind
        0x15, 0x00, // Logical Minimum (0)
        0x25, 0x01, // Logical Maximum (1)
        0x75, 0x01,
        0x95, 0x08,
        0x81, 0x02, // Input (Data, Variable, Absolute)
        0xC0
#endif

#ifdef BLE_RAW_HID
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

#endif // BLE_RAW_HID
        
        
    };

    memset((void *)input_report_array, 0, sizeof(input_report_array));
    memset((void *)output_report_array, 0, sizeof(output_report_array));
    
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

#ifdef BLE_CONSUMER_ENABLE
    // Consumer setting
    p_input_report                      = &input_report_array[INPUT_REPORT_CONSUMER_INDEX];
    p_input_report->max_len             = INPUT_REPORT_CONSUMER_MAX_LEN;
    p_input_report->rep_ref.report_id   = INPUT_REPORT_CONSUMER_ID;
    p_input_report->rep_ref.report_type = BLE_HIDS_REP_TYPE_INPUT;

    p_input_report->sec.cccd_wr = SEC_JUST_WORKS;
    p_input_report->sec.wr      = SEC_JUST_WORKS;
    p_input_report->sec.rd      = SEC_JUST_WORKS;
#endif

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

    hid_info_flags = HID_INFO_FLAG_REMOTE_WAKE_MSK | HID_INFO_FLAG_NORMALLY_CONNECTABLE_MSK;

    memset(&hids_init_obj, 0, sizeof(hids_init_obj));

    hids_init_obj.evt_handler                    = on_hids_evt;
    hids_init_obj.error_handler                  = service_error_handler;
    hids_init_obj.is_kb                          = true;
    hids_init_obj.is_mouse                       = false;

    hids_init_obj.inp_rep_count                  = BLE_INPUT_REPORT_COUNT;
    hids_init_obj.outp_rep_count                 = 2;

    hids_init_obj.p_inp_rep_array                = input_report_array;
    hids_init_obj.p_outp_rep_array               = output_report_array;
    hids_init_obj.feature_rep_count              = 0;
    hids_init_obj.p_feature_rep_array            = NULL;
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


ret_code_t ble_mouse_init(void) {
    memset(&mouse_report_ble, 0, sizeof(mouse_report_ble));
    return NRF_SUCCESS;
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

uint8_t ble_consumer_report[1];
ret_code_t send_consumer_ble(uint8_t code, bool press) {
    if(press) {
        switch(code) {
        case 0x30:
            ble_consumer_report[0] = 1<<4;
            break;
        case 0x40:
            ble_consumer_report[0] = 1<<0;
            break;
        case 0xB3:
            ble_consumer_report[0] = 1<<6;
            break;
        case 0xB4:
            ble_consumer_report[0] = 1<<7;
            break;
        case 0xCD:
            ble_consumer_report[0] = 1<<5;
            break;
        case 0xE2:
            ble_consumer_report[0] = 1<<3;
            break;
        case 0xE9:
            ble_consumer_report[0] = 1<<1;
            break;
        case 0xEA:
            ble_consumer_report[0] = 1<<2;
            break;
        }
    } else {
        ble_consumer_report[0] = 0;
    }
    return ble_hids_inp_rep_send(&m_hids,
                                 INPUT_REPORT_CONSUMER_INDEX,
                                 INPUT_REPORT_CONSUMER_MAX_LEN,
                                 ble_consumer_report,
                                 m_conn_handle);
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

void ble_keyboard_init(void) {
    memset(ble_keyboard_rep_buffer, 0, sizeof(ble_keyboard_rep_buffer));
    ble_mouse_init();
}


keyboard_hid_functions_t ble_hid_functions = {
    .keycode_append = keycode_append_ble,
    .keycode_remove = keycode_remove_ble,
    .send_consumer = send_consumer_ble,
    .reset = keyboard_reset_ble,
    .handle_mouse = handle_keycode_mouse_ble,
    .tick_handler_mouse = tick_handler_mouse_ble
};

#endif // n KEYBOARD_PERIPH
