#include "keyboard_generic.h"
#include "nrf_gpio.h"
#include "usb_keyboard.h"
#include "dynamic_keymap.h"
#include "app_timer.h"
#include "nrf_delay.h"
#include "usb_mouse.h"
#include "via_fds.h"
#include "nrf_log.h"
#include "nrf_power.h"
#include "nrfx_rtc.h"

#include "heatmap.h"
#include "debug_message_hid.h"
#ifdef KEYBOARD_PERIPH
#define KEYCODE_PERIPH 0xFFFF
#include "ble_peripheral.h"
#endif

#include "ble_setting.h"
#include "neopixel.h"
//#include "neopixel_data.h"
#include "neopixel_fds.h"

#define DEBOUNCING_TICK_INVALID 0xFFFFFFFFUL
#define KC_INVALID 0xFF

#ifndef KEYBOARD_PERIPH
#ifdef KEYBOARD_TIMEOUT
APP_TIMER_DEF(m_keyboard_timeout);
#endif
#endif

uint32_t debouncing_bitmap[KBD_SETTING_ROW_PINS_MAX];
uint32_t keypress_bitmap[KBD_SETTING_ROW_PINS_MAX];
keys_t keypress_status[PRESS_KEYS_MAX];
uint8_t kc_release_next_tick[PRESS_KEYS_MAX];
//uint8_t layer_history[DYNAMIC_KEYMAP_LAYER_COUNT];
uint8_t current_layer;

uint8_t m_neopixel_tick_count;
uint8_t m_neopixel_pattern = 0;
uint8_t m_neopixel_index;


keyboard_hid_functions_t hid_functions = {
    .keycode_append = keycode_append_usb,
    .keycode_remove = keycode_remove_usb,
    .send_consumer = send_consumer_usb,
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
    NRF_LOG_DEBUG("sleep mode enter");
    if(! ( NRF_POWER_USBREGSTATUS_VBUSDETECT_MASK & nrf_power_usbregstatus_get() ) ) {
        NRF_LOG_INFO("Going to Sleep...");
        hid_functions.reset();
        // Prepare wakeup buttons.
        err_code = keyboard_sleep_prepare();
        APP_ERROR_CHECK(err_code);

        // skip bootloader
        //NRF_POWER->GPREGRET = 0x6d;
        // Go to system-off mode (this function will not return; wakeup will cause a reset).
        err_code = sd_power_system_off();
        APP_ERROR_CHECK(err_code);
    }
}
#ifndef KEYBOARD_PERIPH
#ifdef KEYBOARD_TIMEOUT
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
#endif

int8_t get_layer(uint8_t current) {
    int8_t layer = -1;
    for(uint8_t i=0; (keypress_status[i].kc||keypress_status[i].application); i++) {
        switch(keypress_status[i].application & 0xF0) {
            case 0x00:
                break;
            case 0x10:
                break;
            case 0x20:
                break;
            case 0x30:
                break;
            case 0x40:
                if(keypress_status[i].press) {
                    if(current != (keypress_status[i].application & 0x0F) ) {
                        layer = keypress_status[i].application & 0x0F;
                    }
                }
                break;
            case 0x50:
                {
                    if(current != (keypress_status[i].kc) ) {
                        layer = keypress_status[i].kc;
                    }
                }
                break;
            case 0x60:
                break;
            case 0x70:
                break;
        }
    }
    return layer;
}

void layer_history_append(uint8_t layer) {
    current_layer = layer;
}

void layer_history_remove(uint8_t layer) {
    uint8_t backword_count = 0;
    NRF_LOG_INFO("removing layer:");
    NRF_LOG_HEXDUMP_INFO(&layer, 1);
    if( layer == get_active_layer() ) {
        int8_t l = get_layer(get_active_layer());
        if( l==-1 ) {
            current_layer =  my_keyboard.default_layer;
        } else {
            current_layer = l;
        }
    }
}

uint8_t get_active_layer(void) {
    return current_layer;
}

/*
void press_keycode(uint8_t kc) {
    if( kc < 0xE8 ) {
        hid_functions.keycode_append(kc);
    } else if( kc>=0xF0 ) {
        hid_functions.handle_mouse(kc,1);
    }
}
*/

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

debounceing_keys_t debouncing_keys[PRESS_KEYS_MAX];

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

    for(uint8_t i=0; i<PRESS_KEYS_MAX; i++) {
        if( debouncing_keys[i].tick != DEBOUNCING_TICK_INVALID ) {
            debouncing_keys[i].tick++;

            if( debouncing_keys[i].tick == DEBOUNCING_TICKS ) {
                debouncing_bitmap[debouncing_keys[i].row] &= ~(1<<debouncing_keys[i].col);
                debouncing_keys[i].tick = DEBOUNCING_TICK_INVALID;
            }
        }
    }
    if(my_keyboard.neopixel_length &&
       (++m_neopixel_tick_count%(neopixel_user_defined_config[m_neopixel_pattern].interval_ticks)==0)) {
        neopixel_write_uint8( neopixel_user_defined[m_neopixel_pattern][m_neopixel_index], my_keyboard.neopixel_length);
        m_neopixel_index++;
        if( neopixel_user_defined_config[m_neopixel_pattern].frame_count <= m_neopixel_index ) {
            m_neopixel_index = 0;
        }
        m_neopixel_tick_count = 0;
    }
}

void register_debounce(uint8_t row, uint8_t col, bool press) {
    debouncing_bitmap[row] |= (1<<col);
    
    for(uint8_t i=0; i<PRESS_KEYS_MAX; i++) {
        if( debouncing_keys[i].tick == DEBOUNCING_TICK_INVALID ) {
            debouncing_keys[i].row = row;
            debouncing_keys[i].col = col;
            debouncing_keys[i].tick = press ? 0 : 0;
            break;
        }
    }
}

ret_code_t handle_keycode(uint16_t keycode, bool press) {
    uint8_t kc = keycode & 0x00FF;
    
    switch(kc) {
        case 0x66:
            return hid_functions.send_consumer(0x30, press);
            
        case 0x76:
            return hid_functions.send_consumer(0x40, press);
            
        case 0xA8:
            return hid_functions.send_consumer(0xE2, press);
            
        case 0xA9: //VOLU
            return hid_functions.send_consumer(0xE9, press);
            
        case 0xAA: //VOLD
            return hid_functions.send_consumer(0xEA, press);
            
        case 0xAE:
            return hid_functions.send_consumer(0xCD, press);
            
        case 0xBB:
            return hid_functions.send_consumer(0xB3, press);
            
        case 0xBC:
            return hid_functions.send_consumer(0xB4, press);
            
    }

  
    if( ((kc & 0xF0) == 0xC0) && press ) {
        // change pair
        /*
        if( (kc&0x0F) < ble_peer_addr_array.count ) {
            NRF_LOG_DEBUG("existing peer");
            ble_connect_to_device( ble_peer_addr_array.peer_addr[kc&0x0F] );
        } else {
            NRF_LOG_DEBUG("Create new connection...");
            ble_gap_addr_t addr = { .addr_id_peer = 0 };
            ble_connect_to_device(addr);
        }
        */
        if( (kc&0x0F)==0x0F ) {
            advertising_without_whitelist(BLE_ADV_MODE_FAST);
        } else {
            ble_connect_to_device( kc&0x0F );
        }
    }

    if( kc < 0xE8 ) {
        if( press ) {
            return hid_functions.keycode_append(kc);
        } else {
            return hid_functions.keycode_remove(kc);
        }
    } else if( kc>=0xF0 ) {
        return hid_functions.handle_mouse(kc,press);
    }

}

void keypress(uint8_t row, uint8_t col, bool debouncing) {
    uint16_t keycode;// = dynamic_keymap_get_keycode(get_active_layer(),row,col);
    uint8_t i = 0;
    uint8_t kc;
    
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
#ifdef KEYBOARD_TIMEOUT
    restart_timeout_timer();
#endif
#endif
    if(debouncing) {
        //nrf_delay_ms(DEBOUNCING_DELAY_MS);
        register_debounce(row, col, true);
    }
    keycode = dynamic_keymap_get_keycode(get_active_layer(),row,col);
#ifdef KEYBOARD_PERIPH
    if(!keycode) {
        keycode = KEYCODE_PERIPH;
    }
#else
    
#endif
    kc = keypress_status[i].kc = keycode & 0x00FF;
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
            handle_keycode(keypress_status[i].kc,true);
            break;
        case 0x10:
            for(uint8_t j=0;j<4;j++) {
                if(action & (1<<j)) {
                    hid_functions.keycode_append(j+0xe4);
                }
            }
            handle_keycode(keypress_status[i].kc, true);
            break;
        case 0x50:
            if( action == 0x0C && keypress_status[i].kc == 0x00 ) {
                // RESET
                keyboard_init(my_keyboard);
                hid_functions.reset();
                break;
            } else if( action == 0x02 ) {
                my_keyboard.default_layer = kc;
                kbd_setting[0x50+KBD_SETTING_ADDITIONAL_DEFAULT_LAYER_INDEX] = kc;
                save_kbd_setting();
            }
            layer_history_append(keypress_status[i].kc);
            
            break;
        }
    } else {
        handle_keycode(keypress_status[i].kc, true);
    }
}


void register_kc_release_next_tick(uint8_t kc){
    uint8_t i=0;
    for(;kc_release_next_tick[i]!=KC_INVALID;i++){}
    kc_release_next_tick[i]=kc;
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
                    handle_keycode(keypress_status[i].kc, false);
                    for(uint8_t j=0;j<4;j++) {
                        if(action & (1<<j)) {
                            hid_functions.keycode_remove(j+0xe0);
                        }
                    }
                    break;
                case 0x10:
                    handle_keycode(keypress_status[i].kc, false);
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
                        handle_keycode(keypress_status[i].kc, true);
                        register_kc_release_next_tick(keypress_status[i].kc);
                    }
                    break;
                case 0x50:
                    if( (action!=0x0C) && (action!=0x02) ) {
                        layer_history_remove(keypress_status[i].kc);
                    }
                    break;
                case 0x60:
                    if(keypress_status[i].press) {
                        for(uint8_t j=0;j<4;j++) {
                            if(action & (1<<j)) {
                                hid_functions.keycode_remove(j+0xe0);
                            }
                        }
                    } else {
                        handle_keycode(keypress_status[i].kc, true);
                        register_kc_release_next_tick(keypress_status[i].kc);
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
                        handle_keycode(keypress_status[i].kc, true);
                        register_kc_release_next_tick(keypress_status[i].kc);
                    }
                    break;
                case 0xC0:
                    // especial functions
                    if( keypress_status[i].kc==0xFE ) {
                        // Coffee to sleep
                        
                        hid_functions.reset();
                        // wait to release all physical keys
                        //nrf_delay_ms(1000);
                        sleep_mode_enter(NULL);
                        break;
                    }
                }
            } else {
                handle_keycode(keypress_status[i].kc, false);
            }
#ifdef KEYBOARD_PERIPH
            send_place_ble(row, col, false);
#endif
            memset(keypress_status+i, 0, sizeof(keys_t));
            if(debouncing) {
                register_debounce(row,col,false);
                //nrf_delay_ms(DEBOUNCING_DELAY_MS);
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


void release_prev_tick_kc(void){
    for(uint8_t i=0;kc_release_next_tick[i]!=KC_INVALID;i++){
        hid_functions.keycode_remove(kc_release_next_tick[i]);kc_release_next_tick[i]=KC_INVALID;
    }
}

void keyboard_init(keyboard_t keyboard) {
    memset(keypress_status, 0, sizeof(keypress_status));
    //memset(layer_history, 255, sizeof(uint8_t)*DYNAMIC_KEYMAP_LAYER_COUNT);
    memset(keypress_bitmap, 0, sizeof(keypress_bitmap));
    memset(debouncing_bitmap, 0, sizeof(debouncing_bitmap));
    memset(kc_release_next_tick,KC_INVALID,PRESS_KEYS_MAX);
    heatmap_init();
    if(my_keyboard.neopixel_length) {
        //neopixel_data_init();
        neopixel_init(my_keyboard.neopixel_pin, NULL);
        m_neopixel_index = 0;
    }
    //layer_history[0] = 0;
    current_layer = my_keyboard.default_layer;
    
    (keyboard.init_method)(keyboard.keyboard_type,keyboard.keyboard_definision);
#ifndef KEYBOARD_PERIPH
#ifdef KEYBOARD_TIMEOUT
    timeout_timer_init();
#endif
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
