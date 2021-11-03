#include "sdk_errors.h"
#include "nrf_log.h"

#include "app_usbd_types.h"
#include "app_usbd_hid.h"
#include "app_usbd_hid_generic.h"
#include "app_usbd_hid_mouse.h"
#include "app_usbd_hid_types.h"

#include "usb_config.h"
#include "usb_mouse.h"

#define ENDPOINT_LIST_MOUSE() ( HID_MOUSE_EPIN, HID_MOUSE_EPOUT )

static void hid_mouse_ev_handler(app_usbd_class_inst_t const * p_inst,
                                app_usbd_hid_user_event_t event);

bool m_report_pending_mouse = false;
APP_USBD_HID_MOUSE_GLOBAL_DEF(m_app_hid_mouse,
                                HID_MOUSE_INTERFACE,
                                HID_MOUSE_EPIN,
                                3,
                                hid_mouse_ev_handler,
                                APP_USBD_HID_SUBCLASS_BOOT);

static void hid_mouse_ev_handler(app_usbd_class_inst_t const * p_inst,
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
            m_report_pending_mouse = false;
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


ret_code_t mouse_reset_usb(void) {
    app_usbd_hid_mouse_button_state(&m_app_hid_mouse, 0, 0);
    app_usbd_hid_mouse_button_state(&m_app_hid_mouse, 1, 0);
    app_usbd_hid_mouse_button_state(&m_app_hid_mouse, 2, 0);
    return NRF_SUCCESS;
}

ret_code_t handle_keycode_mouse_usb(uint16_t keycode, bool press) {
    if( (keycode&0x00F0)!=0x00F0 ) {
        return NRF_ERROR_INVALID_PARAM;
    }

    if( press ) { 
        switch(keycode) {
        
        case 0xF0:
            return app_usbd_hid_mouse_y_move(&m_app_hid_mouse, -(MOUSE_MOVE_DISTANCE>>4));
        case 0xF1:
            return app_usbd_hid_mouse_y_move(&m_app_hid_mouse, MOUSE_MOVE_DISTANCE>>4);
        case 0xF2:
            return app_usbd_hid_mouse_x_move(&m_app_hid_mouse, -(MOUSE_MOVE_DISTANCE>>4));
        case 0xF3:
            return app_usbd_hid_mouse_x_move(&m_app_hid_mouse, MOUSE_MOVE_DISTANCE>>4);
        
        case 0xF4:
        case 0xF5:
        case 0xF6:
        case 0xF7:
        case 0xF8:
            return app_usbd_hid_mouse_button_state(&m_app_hid_mouse, keycode - 0xF4, 1);
        case 0xF9:
            return app_usbd_hid_mouse_scroll_move(&m_app_hid_mouse, WHEEL_MOVE_DISTANCE>>4);
        case 0xFA:
            return app_usbd_hid_mouse_scroll_move(&m_app_hid_mouse, -(WHEEL_MOVE_DISTANCE>>4));
        }
    } else {
        switch(keycode) {
        case 0xF4:
        case 0xF5:
        case 0xF6:
        case 0xF7:
        case 0xF8:
            return app_usbd_hid_mouse_button_state(&m_app_hid_mouse, keycode - 0xF4, 0);
        }
    }
    return NRF_SUCCESS;
}


ret_code_t tick_handler_mouse_usb(keys_t *p_key) {
    if( (p_key->kc&0x00F0)!=0x00F0 ) {
        return NRF_ERROR_INVALID_PARAM;
    }
    
    if(! (p_key->tick % MOUSE_MOVE_INTERVAL_TICKS) ) {
        uint8_t tock = p_key->tick / MOUSE_MOVE_INTERVAL_TICKS;
        uint8_t time_keeper = tock>4 ? 1 : (4 - tock);
        switch(p_key->kc) {
        case 0xF0:
            return app_usbd_hid_mouse_y_move(&m_app_hid_mouse, -(MOUSE_MOVE_DISTANCE/time_keeper));
        case 0xF1:
            return app_usbd_hid_mouse_y_move(&m_app_hid_mouse, (MOUSE_MOVE_DISTANCE/time_keeper));
        case 0xF2:
            return app_usbd_hid_mouse_x_move(&m_app_hid_mouse, -(MOUSE_MOVE_DISTANCE/time_keeper));
        case 0xF3:
            return app_usbd_hid_mouse_x_move(&m_app_hid_mouse, (MOUSE_MOVE_DISTANCE/time_keeper));
        case 0xF4:
        case 0xF5:
        case 0xF6:
        case 0xF7:
        case 0xF8:
            return NRF_ERROR_INVALID_PARAM;
        case 0xF9:
            return app_usbd_hid_mouse_scroll_move(&m_app_hid_mouse, WHEEL_MOVE_DISTANCE/time_keeper);
        case 0xFA:
            return app_usbd_hid_mouse_scroll_move(&m_app_hid_mouse, -(WHEEL_MOVE_DISTANCE/time_keeper));
        }
    }
    return NRF_SUCCESS;
}


ret_code_t usb_mouse_init(void) {
    ret_code_t ret;
    app_usbd_class_inst_t const * class_inst_mouse;
    class_inst_mouse = app_usbd_hid_mouse_class_inst_get(&m_app_hid_mouse);

    //ret = hid_generic_idle_handler_set(class_inst_mouse, idle_handle_mouse);
    //APP_ERROR_CHECK(ret);
    ret = app_usbd_class_append(class_inst_mouse);
    APP_ERROR_CHECK(ret);
}
