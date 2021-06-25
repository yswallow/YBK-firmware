#ifdef KEYBOARD_DEBUG_HID
#include "raw_hid.h"
#include "usb_keyboard.h"
#include "ble_keyboard.h"
#include "keyboard_generic.h"
#include "via.h"
#include "dynamic_keymap.h"
#include "app_timer.h"

APP_TIMER_DEF(m_keyboard_debug_timer);

static uint8_t hid_debug_message[32];

void keyboard_send_hid_debug(void* ptr) {
    memset( hid_debug_message, 0, 32 );
    hid_debug_message[0] = id_get_keyboard_value;
    hid_debug_message[1] = 0x81;
    hid_debug_message[2] = get_active_layer();
    memcpy( hid_debug_message+3, layer_history, 10);
    memcpy( hid_debug_message+13, usb_keyboard_rep_buffer, 8);
    memcpy( hid_debug_message+21, ble_keyboard_rep_buffer, 8);
    raw_hid_send(hid_debug_message, 32);
/*
    for(uint8_t i=0;i<2;i++) {
        hid_debug_message[1] = 82+i;
        for(uint8_t j=0;j<5;j++) {
            hid_debug_message[2+4*j] = keypress_status[j+2*i].row;
            hid_debug_message[3+4*j] = keypress_status[j+2*i].col;
            hid_debug_message[4+4*j] = keypress_status[j+2*i].kc;
            hid_debug_message[5+4*j] = keypress_status[j+2*i].application;
        }
        raw_hid_send(hid_debug_message, 22);
    } */
}

void keyboard_debug_init(void) {
    ret_code_t ret;
    ret = app_timer_create(&m_keyboard_debug_timer, APP_TIMER_MODE_REPEATED, keyboard_send_hid_debug);
    APP_ERROR_CHECK(ret);

    ret = app_timer_start(m_keyboard_debug_timer, APP_TIMER_TICKS(2000), NULL);
    APP_ERROR_CHECK(ret);
}

#endif