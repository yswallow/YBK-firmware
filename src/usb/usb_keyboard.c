#include "sdk_errors.h"
#include "nrf_log.h"

#include "app_usbd_types.h"
#include "app_usbd_hid.h"
#include "app_usbd_hid_generic.h"
#include "app_usbd_hid_kbd.h"
#include "app_usbd_hid_types.h"

#include "usb_config.h"
#include "usb_hiddevice.h"


#define USB_HID_KBD_IN_QUEUE_SIZE 1
#define USB_HID_KBD_OUT_REP_MAXSIZE 1
#define USB_HID_KBD_FEATURE_REP_MAXSIZE 1

#define USB_HID_KBD_MODS_INDEX 0
#define USB_HID_KBD_KEYS_START 2
#define ENDPOINT_LIST_KEYBOARD() ( HID_KEYBOARD_EPIN, HID_KEYBOARD_EPOUT )

static void hid_keyboard_ev_handler(app_usbd_class_inst_t const * p_inst,
                                app_usbd_hid_user_event_t event);

//bool m_report_pending_kbd = false;
uint8_t usb_keyboard_rep_buffer[USB_HID_KBD_REP_LEN];

#define USBD_HID_KBD_REPORT_DSC() {                                                    \
        0x05, 0x01,                    /* USAGE_PAGE (Generic Desktop)                   */\
        0x09, 0x06,                    /* USAGE (Keyboard)                               */\
        0xa1, 0x01,                    /* COLLECTION (Application)                       */\
        0x05, 0x07,                    /*   USAGE_PAGE (Keyboard)                        */\
        0x19, 0xe0,                    /*   USAGE_MINIMUM (Keyboard LeftControl)         */\
        0x29, 0xe7,                    /*   USAGE_MAXIMUM (Keyboard Right GUI)           */\
        0x15, 0x00,                    /*   LOGICAL_MINIMUM (0)                          */\
        0x25, 0x01,                    /*   LOGICAL_MAXIMUM (1)                          */\
        0x75, 0x01,                    /*   REPORT_SIZE (1)                              */\
        0x95, 0x08,                    /*   REPORT_COUNT (8)                             */\
        0x81, 0x02,                    /*   INPUT (Data,Var,Abs)                         */\
        0x95, 0x01,                    /*   REPORT_COUNT (1)                             */\
        0x75, 0x08,                    /*   REPORT_SIZE (8)                              */\
        0x81, 0x03,                    /*   INPUT (Cnst,Var,Abs)                         */\
        0x95, 0x05,                    /*   REPORT_COUNT (5)                             */\
        0x75, 0x01,                    /*   REPORT_SIZE (1)                              */\
        0x05, 0x08,                    /*   USAGE_PAGE (LEDs)                            */\
        0x19, 0x01,                    /*   USAGE_MINIMUM (Num Lock)                     */\
        0x29, 0x05,                    /*   USAGE_MAXIMUM (Kana)                         */\
        0x91, 0x02,                    /*   OUTPUT (Data,Var,Abs)                        */\
        0x95, 0x01,                    /*   REPORT_COUNT (1)                             */\
        0x75, 0x03,                    /*   REPORT_SIZE (3)                              */\
        0x91, 0x03,                    /*   OUTPUT (Cnst,Var,Abs)                        */\
        0x95, 0x06,                    /*   REPORT_COUNT (6)                             */\
        0x75, 0x08,                    /*   REPORT_SIZE (8)                              */\
        0x15, 0x00,                    /*   LOGICAL_MINIMUM (0)                          */\
        0x25, 0x98,                    /*   LOGICAL_MAXIMUM (101)                        */\
        0x05, 0x07,                    /*   USAGE_PAGE (Keyboard)                        */\
        0x19, 0x00,                    /*   USAGE_MINIMUM (Reserved (no event indicated))*/\
        0x29, 0x98,                    /*   USAGE_MAXIMUM (Keyboard Application)         */\
        0x81, 0x00,                    /*   INPUT (Data,Ary,Abs)                         */\
        0xc0                           /* END_COLLECTION                                 */\
}

APP_USBD_HID_GENERIC_SUBCLASS_REPORT_DESC(keyboard_desc, USBD_HID_KBD_REPORT_DSC());

static const app_usbd_hid_subclass_desc_t * kbd_reps[] = {&keyboard_desc};

APP_USBD_HID_GENERIC_GLOBAL_DEF(m_app_hid_kbd,
                                HID_KEYBOARD_INTERFACE,
                                hid_keyboard_ev_handler,
                                ENDPOINT_LIST_KEYBOARD(),
                                kbd_reps,
                                USB_HID_KBD_IN_QUEUE_SIZE,
                                USB_HID_KBD_OUT_REP_MAXSIZE,
                                USB_HID_KBD_FEATURE_REP_MAXSIZE,
                                APP_USBD_HID_SUBCLASS_BOOT,
                                APP_USBD_HID_PROTO_KEYBOARD);


static ret_code_t keyboard_report_send_usb(void) {
    return app_usbd_hid_generic_in_report_set(
        &m_app_hid_kbd,
        usb_keyboard_rep_buffer,
        USB_HID_KBD_REP_LEN);
}

/*
static ret_code_t idle_handle_keyboard(app_usbd_class_inst_t const * p_inst, uint8_t report_id)
{
    return NRF_ERROR_NOT_SUPPORTED;   
}
*/

static void hid_keyboard_ev_handler(app_usbd_class_inst_t const * p_inst,
                                app_usbd_hid_user_event_t event)
{
    switch (event)
    {
        case APP_USBD_HID_USER_EVT_OUT_REPORT_READY:
        {
            break;
        }
        case APP_USBD_HID_USER_EVT_IN_REPORT_DONE:
        {
            //m_report_pending_kbd = false;
            //hid_generic_keyboard_process_state();
            break;
        }
        case APP_USBD_HID_USER_EVT_SET_BOOT_PROTO:
        {
            UNUSED_RETURN_VALUE(hid_generic_clear_buffer(p_inst));
            NRF_LOG_INFO("SET_BOOT_PROTO");
            break;
        }
        case APP_USBD_HID_USER_EVT_SET_REPORT_PROTO:
        {
            UNUSED_RETURN_VALUE(hid_generic_clear_buffer(p_inst));
            NRF_LOG_INFO("SET_REPORT_PROTO");
            break;
        }
        default:
            break;
    }
}


static ret_code_t keycode_append_usb(const uint8_t kc) {
    bool updated = false;
    if( (kc&0xF8) == 0xe0 ) {
        // modifiers
        if( usb_keyboard_rep_buffer[USB_HID_KBD_MODS_INDEX] & ( 1<<(kc-0xe0) ) ) {
        
        } else {
            usb_keyboard_rep_buffer[USB_HID_KBD_MODS_INDEX] |= ( 1<<(kc-0xe0) );
            updated = true;
        }
    } else {
        for(uint8_t i = USB_HID_KBD_KEYS_START; i<USB_HID_KBD_REP_LEN; i++) {
            if(usb_keyboard_rep_buffer[i]==0x00) {
                usb_keyboard_rep_buffer[i] = kc;
                updated = true;
                break;
            }
            
            if(usb_keyboard_rep_buffer[i]==kc) {
                break;
            }
        }
    }
    
    if( updated ) {
        keyboard_report_send_usb();
    }
    return NRF_SUCCESS;
}

static ret_code_t keycode_remove_usb(const uint8_t kc) {
    bool after_kc = false;
    bool updated = false;
    if( (kc&0xF8) == 0xe0 ) {
        // modifiers
        if( usb_keyboard_rep_buffer[USB_HID_KBD_MODS_INDEX] & ( 1<<(kc&0x07) ) ) {
            usb_keyboard_rep_buffer[USB_HID_KBD_MODS_INDEX] &= ~( 1<<(kc&0x07) );
            updated = true;
        }
    } else {
        for(uint8_t i = USB_HID_KBD_KEYS_START; i<USB_HID_KBD_REP_LEN; i++) {
            if(usb_keyboard_rep_buffer[i]==0x00) {
                break;
            }
            
            if(after_kc) {
                usb_keyboard_rep_buffer[i-1] = usb_keyboard_rep_buffer[i];
                usb_keyboard_rep_buffer[i] = 0x00;
            }

            if(usb_keyboard_rep_buffer[i]==kc) {
                after_kc = true;
                updated = true;
                usb_keyboard_rep_buffer[i] = 0x00; // for if i is the last
            }
        }
    }

    if(updated) {
        keyboard_report_send_usb();
    }
    return NRF_SUCCESS;
}

static ret_code_t keyboard_reset_usb(void) {
    memset(usb_keyboard_rep_buffer, 0, sizeof(usb_keyboard_rep_buffer));
    keyboard_report_send_usb();
    return NRF_SUCCESS;
}

void usb_keyboard_init(void) {
    //STATIC_ASSERT(HID_KEYBOARD_INTERFACE==0);
    //STATIC_ASSERT(EPIN_NUM_HID_KEYBOARD==1);
    ret_code_t ret;
    memset(usb_keyboard_rep_buffer, 0, sizeof(usb_keyboard_rep_buffer));
    app_usbd_class_inst_t const * class_inst_keyboard;
    class_inst_keyboard = app_usbd_hid_generic_class_inst_get(&m_app_hid_kbd);

    //ret = hid_generic_idle_handler_set(class_inst_keyboard, idle_handle_keyboard);
    //APP_ERROR_CHECK(ret);
    ret = app_usbd_class_append(class_inst_keyboard);
    APP_ERROR_CHECK(ret);
}

void usb_hid_init(void) {
    usb_keyboard_init();
    usb_hid_raw_init();
    usb_mouse_init();
    usb_hid_consumer_init();
}

const keyboard_hid_functions_t usb_hid_functions = {
    .keycode_append = keycode_append_usb,
    .keycode_remove = keycode_remove_usb,
    .send_consumer = send_consumer_usb,
    .reset = keyboard_reset_usb,
    .handle_mouse = handle_keycode_mouse_usb,
    .tick_handler_mouse = tick_handler_mouse_usb
};