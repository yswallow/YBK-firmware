#include "keyboard_generic.h"
#include "nrf_gpio.h"
#include "main.h"
#include "boards.h"
#include "usb_keyboard.h"
#include "dynamic_keymap.h"
#include "app_timer.h"
#include "nrf_delay.h"
#include "usb_mouse.h"
#include "via_fds.h"
#include "nrf_log.h"
#include "nrf_power.h"

#include "heatmap.h"
#include "debug_message_hid.h"
#ifdef KEYBOARD_PERIPH
#define KEYCODE_PERIPH 0xFFFF
#include "ble_peripheral.h"
#endif

APP_TIMER_DEF(m_tick_kbd);
#ifndef KEYBOARD_PERIPH
APP_TIMER_DEF(m_keyboard_timeout);
#endif


uint32_t keypress_bitmap[KBD_SETTING_ROW_PINS_MAX];
keys_t keypress_status[PRESS_KEYS_MAX];
uint8_t layer_history[DYNAMIC_KEYMAP_LAYER_COUNT];

keyboard_hid_functions_t hid_functions = {
    .keycode_append = keycode_append_usb,
    .keycode_remove = keycode_remove_usb,
    .handle_keycode = handle_keycode_usb,
    .reset = keyboard_reset_usb,
    .handle_mouse = handle_keycode_mouse_usb,
    .tick_handler_mouse = tick_handler_mouse_usb
};


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
void sleep_mode_enter(void *ptr)
{
    ret_code_t err_code;
    if(! ( NRF_POWER_USBREGSTATUS_VBUSDETECT_MASK & nrf_power_usbregstatus_get() ) ) {
        hid_functions.reset();
        // Prepare wakeup buttons.
        err_code = keyboard_sleep_prepare();
        APP_ERROR_CHECK(err_code);

        // Go to system-off mode (this function will not return; wakeup will cause a reset).
        err_code = sd_power_system_off();
        APP_ERROR_CHECK(err_code);
    }
}
#ifndef KEYBOARD_PERIPH
void restart_timeout_timer(void) {
    ret_code_t err_code;
    err_code = app_timer_stop(m_keyboard_timeout);
    APP_ERROR_CHECK(err_code);
    
    err_code = app_timer_start(m_keyboard_timeout, KEYBOARD_TIMEOUT_TICKS, NULL);
    APP_ERROR_CHECK(err_code);
    
}

void timeout_timer_init(void) {
    ret_code_t err_code;
    err_code = app_timer_create(&m_keyboard_timeout, APP_TIMER_MODE_SINGLE_SHOT, sleep_mode_enter);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_start(m_keyboard_timeout, KEYBOARD_TIMEOUT_TICKS, NULL);
    APP_ERROR_CHECK(err_code);
}
#endif


void layer_history_append(uint8_t layer) {
    uint8_t i=1;
    for(;i<DYNAMIC_KEYMAP_LAYER_COUNT;i++) {
        if(layer_history[i]==255) {
            break;
        }
    }
    layer_history[i] = layer;
}

void press_keycode(uint8_t kc) {
    if( kc < 0xE8 ) {
        hid_functions.keycode_append(kc);
    } else if( kc>=0xF0 ) {
        hid_functions.handle_mouse(kc,1);
    }
}

void tick_key(keys_t *p_key) {
    if( p_key->kc>=0xF0 ) {
        hid_functions.tick_handler_mouse(p_key);
    }
}

void release_keycode(uint8_t kc) {
    if( kc<0xE8 ) {
        hid_functions.keycode_remove(kc);
    } else if( kc>=0xF0 ) {
        hid_functions.handle_mouse(kc, 0);
    }
}

void press_key(keys_t *p_key) {
    uint8_t action = p_key->application & 0x0F;
    switch(p_key->application & 0xF0) {
    case 0x00:
        tick_key(p_key);
        break;
    case 0x10:
        break;
    case 0x20:
        break;
    case 0x30:
        break;
    case 0x40:
        p_key->press = true;
        layer_history_append(action);
        break;
    case 0x50:
        break;
    case 0x60:
            p_key->press = true;
            for(uint8_t j=0;j<4;j++) {
                if(action & (1<<j)) {
                    hid_functions.keycode_append(j+0xe0);
                }
            }
        break;
    case 0x70:
            p_key->press = true;
            for(uint8_t j=0;j<4;j++) {
                if(action & (1<<j)) {
                    hid_functions.keycode_append(j+0xe4);
                }
            }
        
        break;
    }
}

void kbd_tick_handler(void* p_context) {
    //called every TAPPING_TERM_TICK_MS msecs
    for(uint8_t i=0; (keypress_status[i].kc||keypress_status[i].application) && i<PRESS_KEYS_MAX; i++) {
        if(! keypress_status[i].press) {
            keypress_status[i].tick++;
            if(keypress_status[i].tick==TAPPING_TERM/TAPPING_TERM_TICK_MS) {
                press_key(&keypress_status[i]);
            } else {
                tick_key(&keypress_status[i]);
            }
        }
    }
}

void layer_history_remove(uint8_t layer) {
    uint8_t backword_count = 0;
    NRF_LOG_INFO("removing layer:");
    NRF_LOG_HEXDUMP_INFO(&layer, 1);

    for(uint8_t i=1;i<DYNAMIC_KEYMAP_LAYER_COUNT;i++) {
        if(layer_history[i]==255) {
            break;
        }

        if( layer_history[i] == layer ) {
            NRF_LOG_INFO("layer removed.");
            KEYBOARD_DEBUG_HID_REGISTER_STRING("layer removed.", 15);
            backword_count++;
        }
        
        if( backword_count ) {
            layer_history[i] = layer_history[i+backword_count];
        }
    }
}


uint8_t get_active_layer(void) {
    uint8_t i=1;
    for(;layer_history[i]!=255 && i<DYNAMIC_KEYMAP_LAYER_COUNT;i++) {}
    return layer_history[i-1];
}

void keypress(uint8_t row, uint8_t col, bool debouncing) {
    uint16_t keycode;// = dynamic_keymap_get_keycode(get_active_layer(),row,col);
    uint8_t i = 0;
    for(; (keypress_status[i].kc||keypress_status[i].application) && i<PRESS_KEYS_MAX; i++) {
        if(keypress_status[i].col == col && keypress_status[i].row == row) {
            // already pressed.
            return;
        }

#ifdef TAPPING_TERM_FORCE_HOLD
        if(! (keypress_status[i].press)) {
            press_key(&keypress_status[i]);
        }
#endif
    }
    // new press
    m_heatmap[row][col]++;
    keypress_status[i].row = row;
    keypress_status[i].col = col;
    keypress_status[i].tick = 0;
    keypress_status[i].press = false;
#ifdef KEYBOARD_PERIPH
    send_place_ble(row,col,true);
#endif
#ifndef KEYBOARD_PERIPH
    restart_timeout_timer();
#endif
    if(debouncing) {
        nrf_delay_ms(DEBOUNCING_DELAY_MS);
    }
    keycode = dynamic_keymap_get_keycode(get_active_layer(),row,col);
#ifdef KEYBOARD_PERIPH
    if(!keycode) {
        keycode = KEYCODE_PERIPH;
    }
#else
    
#endif
    keypress_status[i].kc = keycode & 0x00FF;
    keypress_status[i].application = (keycode>>8) & 0x00FF ;
    
    
    if( keypress_status[i].application ) {
        uint8_t action = keypress_status[i].application & 0x0F;
        switch(keypress_status[i].application & 0xF0) {
        case 0x00:
            for(uint8_t j=0;j<4;j++) {
                if(action & (1<<j)) {
                    hid_functions.keycode_append(j+0xe0);
                }
            }
            hid_functions.keycode_append(keypress_status[i].kc);
            break;
        case 0x10:
            for(uint8_t j=0;j<4;j++) {
                if(action & (1<<j)) {
                    hid_functions.keycode_append(j+0xe4);
                }
            }
            hid_functions.keycode_append(keypress_status[i].kc);
            break;
        case 0x50:
            if( action == 0x0C && keypress_status[i].kc == 0x00 ) {
                // RESET
                keyboard_init(my_keyboard);
                hid_functions.reset();
                break;
            }
            layer_history_append(keypress_status[i].kc);
            break;
        }
    } else {
        press_keycode(keypress_status[i].kc);
    }
}

void keyrelease(uint8_t row, uint8_t col, bool debouncing) {
    uint8_t removes_count = 0;
    for(uint8_t i=0; i<PRESS_KEYS_MAX; i++) {
        if(keypress_status[i].kc==0x00 && keypress_status[i].application==0x00) {
            break;
        }
        if(keypress_status[i].row == row && keypress_status[i].col == col ) {
            removes_count += 1;
            
            if(keypress_status[i].application) {
                uint8_t action = keypress_status[i].application & 0x0F;
                switch(keypress_status[i].application & 0xF0) {
                case 0x00:
                    hid_functions.keycode_remove(keypress_status[i].kc);
                    for(uint8_t j=0;j<4;j++) {
                        if(action & (1<<j)) {
                            hid_functions.keycode_remove(j+0xe0);
                        }
                    }
                    break;
                case 0x10:
                    hid_functions.keycode_remove(keypress_status[i].kc);
                    for(uint8_t j=0;j<4;j++) {
                        if(action & (1<<j)) {
                            hid_functions.keycode_remove(j+0xe4);
                        }
                    }
                    break;
                case 0x20:
                    break;
                case 0x30:
                    break;
                case 0x40:
                    if(keypress_status[i].press)  {
                        layer_history_remove(action);
                    } else {
                        hid_functions.keycode_append(keypress_status[i].kc);
                        hid_functions.keycode_remove(keypress_status[i].kc);
                    }
                    break;
                case 0x50:
                    layer_history_remove(keypress_status[i].kc);
                    break;
                case 0x60:
                    if(keypress_status[i].press) {
                        for(uint8_t j=0;j<4;j++) {
                            if(action & (1<<j)) {
                                hid_functions.keycode_remove(j+0xe0);
                            }
                        }
                    } else {
                        hid_functions.keycode_append(keypress_status[i].kc);
                        hid_functions.keycode_remove(keypress_status[i].kc);
                    }
                    break;
                case 0x70:
                    if(keypress_status[i].press) {
                        for(uint8_t j=0;j<4;j++) {
                            if(action & (1<<j)) {
                                hid_functions.keycode_remove(j+0xe4);
                            }
                        }
                    } else {
                        hid_functions.keycode_append(keypress_status[i].kc);
                        hid_functions.keycode_remove(keypress_status[i].kc);
                    }
                    break;
                }
            } else {
                release_keycode(keypress_status[i].kc);
            }
#ifdef KEYBOARD_PERIPH
            send_place_ble(row, col, false);
#endif
            memset(keypress_status+i, 0, sizeof(keys_t));
            if(debouncing) {
                nrf_delay_ms(DEBOUNCING_DELAY_MS);
            }
            
        }
        if(removes_count) {
            if( (removes_count + i) < PRESS_KEYS_MAX ) {
                memcpy(keypress_status+i, keypress_status+i+removes_count, sizeof(keys_t));
                //memset(keypress_status+i+removes_count, 0, sizeof(keys_t));
            } else {
                memset(keypress_status+i, 0, sizeof(keys_t));
            }
        }
    }
}

void keyboard_scan(keyboard_t keyboard) {
    (keyboard.scan_method)(keyboard.keyboard_type,keyboard.keyboard_definision);
}

void keyboard_init(keyboard_t keyboard) {
    memset(keypress_status, 0, sizeof(keypress_status));
    memset(layer_history, 255, sizeof(uint8_t)*DYNAMIC_KEYMAP_LAYER_COUNT);
    memset(keypress_bitmap, 0, sizeof(keypress_bitmap));
    heatmap_init();
    layer_history[0] = 0;
    
    app_timer_create(&m_tick_kbd, APP_TIMER_MODE_REPEATED, kbd_tick_handler);
    app_timer_start(m_tick_kbd, APP_TIMER_TICKS(TAPPING_TERM_TICK_MS),NULL);
    (keyboard.init_method)(keyboard.keyboard_type,keyboard.keyboard_definision);
#ifndef KEYBOARD_PERIPH
    timeout_timer_init();
#endif
    // initialize indicator
    if(keyboard.kbd_power_led_enable) {
        nrf_gpio_cfg_output(keyboard.kbd_power_led);
        nrf_gpio_pin_set(keyboard.kbd_power_led);
    }
}

#if 0
void keyboard_sleep_prepare(keyboard_t keyboard) {
    (keyboard.sleep_prepare)(keyboard.keyboard_type, keyboard.keyboard_definision);
}
#endif
