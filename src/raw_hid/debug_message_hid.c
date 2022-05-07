#ifdef KEYBOARD_DEBUG_HID
#include "raw_hid.h"
#include "usb_hiddevice.h"
#include "ble_hiddevice.h"
#include "keyboard_generic.h"
#include "via.h"
#include "dynamic_keymap.h"
#include "app_timer.h"

APP_TIMER_DEF(m_keyboard_debug_timer);

static uint8_t hid_debug_message[RAW_REP_SIZE];
static bool m_keyboard_hid_debug_enable = false;
static bool m_keyboard_hid_debug_pending = false;


void keyboard_debug_hid_set(uint8_t* data, uint8_t length) {
    m_keyboard_hid_debug_enable = data[2] ? true : false;
    return;
}


void keyboard_debug_hid_response(uint8_t* data, uint8_t length) {
    if( m_keyboard_hid_debug_enable ) {
        m_keyboard_hid_debug_pending = false;
        return;
    }
}


void keyboard_debug_hid_register(uint8_t *data, uint8_t length) {
    if( m_keyboard_hid_debug_enable ) {
        if( m_keyboard_hid_debug_pending ) {
            hid_debug_message[2]++;
        } else {
            memset(hid_debug_message,0,RAW_REP_SIZE);
            m_keyboard_hid_debug_pending = true;
            hid_debug_message[0] = id_get_keyboard_value;
            hid_debug_message[1] = 0x8a;
        }
        memcpy(hid_debug_message+3, data, (length<RAW_REP_SIZE ? length : RAW_REP_SIZE)-3);
        raw_hid_send(hid_debug_message, RAW_REP_SIZE);
    }
}


void keyboard_debug_hid_register_string(uint8_t *data, uint8_t length) {
    uint8_t mem[length+1];
    memcpy(mem+1, data, length);
    mem[0] = 0x08;
    keyboard_debug_hid_register(mem, length+1);
}


void keyboard_send_hid_debug_periodical(void* ptr) {
    uint8_t periodical_debug_data[29];
    memset( periodical_debug_data, 0, 29 );
    periodical_debug_data[0] = 0x01;
    periodical_debug_data[1] = get_active_layer();
    memset( periodical_debug_data+2,  0, 10);
    memcpy( periodical_debug_data+12, usb_keyboard_rep_buffer, 8);
#ifndef KEYBOARD_PERIPH
    memcpy( periodical_debug_data+20, ble_keyboard_rep_buffer, 8);
#endif
    keyboard_debug_hid_register(periodical_debug_data, 29);
}


void keyboard_debug_periodical_init(void) {
    ret_code_t ret;
    ret = app_timer_create(&m_keyboard_debug_timer, APP_TIMER_MODE_REPEATED, keyboard_send_hid_debug_periodical);
    APP_ERROR_CHECK(ret);

    ret = app_timer_start(m_keyboard_debug_timer, APP_TIMER_TICKS(2000), NULL);
    APP_ERROR_CHECK(ret);
}

#endif