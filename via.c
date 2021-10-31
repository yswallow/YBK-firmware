/* Copyright 2019 Jason Williams (Wilba)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "via.h"
#include "raw_hid.h"
#include "dynamic_keymap.h"
#include "keyboard_config.h"
#include "via_fds.h"


// This is generalized so the layout options EEPROM usage can be
// variable, between 1 and 4 bytes.
uint32_t via_get_layout_options(void) {
    uint32_t value = 0;
    // Start at the most significant byte
    void *source = (void *)(VIA_EEPROM_LAYOUT_OPTIONS_ADDR);
    for (uint8_t i = 0; i < VIA_EEPROM_LAYOUT_OPTIONS_SIZE; i++) {
        value = value << 8;
        value |= eeprom_read_byte(source);
        source++;
    }
    return value;
}

void via_set_layout_options(uint32_t value) {
    // Start at the least significant byte
    void *target = (void *)(VIA_EEPROM_LAYOUT_OPTIONS_ADDR + VIA_EEPROM_LAYOUT_OPTIONS_SIZE - 1);
    for (uint8_t i = 0; i < VIA_EEPROM_LAYOUT_OPTIONS_SIZE; i++) {
        eeprom_update_byte(target, value & 0xFF);
        value = value >> 8;
        target--;
    }
}


// VIA handles received HID messages first, and will route to
// raw_hid_receive_kb() for command IDs that are not handled here.
// This gives the keyboard code level the ability to handle the command
// specifically.
//
// raw_hid_send() is called at the end, with the same buffer, which was
// possibly modified with returned values.
bool keymap_updated = false;
void raw_hid_receive(uint8_t *data, uint8_t length) {
    uint8_t *command_id   = &(data[0]);
    uint8_t *command_data = &(data[1]);
    switch (*command_id) {
        case id_get_protocol_version: {
            command_data[0] = VIA_PROTOCOL_VERSION >> 8;
            command_data[1] = VIA_PROTOCOL_VERSION & 0xFF;
            break;
        }
        case id_get_keyboard_value: {
            switch (command_data[0]) {
                case id_uptime: {
                    uint32_t value  = 0xC0FFEE;//timer_read32();
                    command_data[1] = (value >> 24) & 0xFF;
                    command_data[2] = (value >> 16) & 0xFF;
                    command_data[3] = (value >> 8) & 0xFF;
                    command_data[4] = value & 0xFF;
                    break;
                }
                case id_layout_options: {
                    uint32_t value  = via_get_layout_options();
                    command_data[1] = (value >> 24) & 0xFF;
                    command_data[2] = (value >> 16) & 0xFF;
                    command_data[3] = (value >> 8) & 0xFF;
                    command_data[4] = value & 0xFF;
                    break;
                }
                case id_switch_matrix_state: {
                    
                    if ((my_keyboard.kbd_cols_count / 8 + 1) * my_keyboard.kbd_rows_count <= 28) {
                        uint8_t i = 1;
                        for (uint8_t row = 0; row < my_keyboard.kbd_rows_count; row++) {
                            matrix_row_t value = 8;//matrix_get_row(row);
                            if (my_keyboard.kbd_cols_count > 24){
                              command_data[i++] = (value >> 24) & 0xFF;
                            }
                            if (my_keyboard.kbd_cols_count > 16){
                              command_data[i++] = (value >> 16) & 0xFF;
                            }
                            if (my_keyboard.kbd_cols_count > 8){
                              command_data[i++] = (value >> 8) & 0xFF;
                            }
                            command_data[i++] = value & 0xFF;
                        }
                    }
                    break;
                }
                default: {
                    raw_hid_receive_kb(data, length);
                    break;
                }
            }
            break;
        }
        case id_set_keyboard_value: {
            switch (command_data[0]) {
                case id_layout_options: {
                    uint32_t value = ((uint32_t)command_data[1] << 24) | ((uint32_t)command_data[2] << 16) | ((uint32_t)command_data[3] << 8) | (uint32_t)command_data[4];
                    //via_set_layout_options(value);
                    break;
                }
                default: {
                    raw_hid_receive_kb(data, length);
                    break;
                }
            }
            break;
        }
        case id_dynamic_keymap_get_keycode: {
            uint16_t keycode = dynamic_keymap_get_keycode(command_data[0], command_data[1], command_data[2]);
            command_data[3]  = keycode >> 8;
            command_data[4]  = keycode & 0xFF;
            break;
        }
        case id_dynamic_keymap_set_keycode: {
            dynamic_keymap_set_keycode(command_data[0], command_data[1], command_data[2], (command_data[3] << 8) | command_data[4]);
            keymap_updated = true;
            break;
        }
        case id_dynamic_keymap_reset: {
            //dynamic_keymap_reset();
            break;
        }
        case id_dynamic_keymap_macro_get_count: {
            //command_data[0] = dynamic_keymap_macro_get_count();
            break;
        }
        case id_dynamic_keymap_macro_get_buffer_size: {
            uint16_t size   = 0x0;//dynamic_keymap_macro_get_buffer_size();
            command_data[0] = size >> 8;
            command_data[1] = size & 0xFF;
            break;
        }
        case id_dynamic_keymap_macro_get_buffer: {
            uint16_t offset = (command_data[0] << 8) | command_data[1];
            uint16_t size   = command_data[2];  // size <= 28
            //dynamic_keymap_macro_get_buffer(offset, size, &command_data[3]);
            break;
        }
        case id_dynamic_keymap_macro_set_buffer: {
            uint16_t offset = (command_data[0] << 8) | command_data[1];
            uint16_t size   = command_data[2];  // size <= 28
            //dynamic_keymap_macro_set_buffer(offset, size, &command_data[3]);
            break;
        }
        case id_dynamic_keymap_macro_reset: {
            //dynamic_keymap_macro_reset();
            break;
        }
        case id_dynamic_keymap_get_layer_count: {
            command_data[0] = dynamic_keymap_get_layer_count();
            break;
        }
        case id_dynamic_keymap_get_buffer: {
            uint16_t offset = (command_data[0] << 8) | command_data[1];
            uint16_t size   = command_data[2];  // size <= 28
            dynamic_keymap_get_buffer(offset, size, &command_data[3]);
            if( keymap_updated ) {
                save_keymap();
                keymap_updated = false;
            }
            break;
        }
        case id_dynamic_keymap_set_buffer: {
            uint16_t offset = (command_data[0] << 8) | command_data[1];
            uint16_t size   = command_data[2];  // size <= 28
            dynamic_keymap_set_buffer(offset, size, &command_data[3]);
            keymap_updated = true;
            break;
        }
        default: {
            // The command ID is not known
            // Return the unhandled state
            *command_id = id_unhandled;
            break;
        }
    }

    // Return the same buffer, optionally with values changed
    // (i.e. returning state to the host, or the unhandled state).
    if(length > RAW_REP_SIZE) {
        length = RAW_REP_SIZE;
    }
    if(data[0] != id_unhandled) {
        raw_hid_send(data, length);
    }
}

#if defined(VIA_QMK_BACKLIGHT_ENABLE)

#    if BACKLIGHT_LEVELS == 0
#        error BACKLIGHT_LEVELS == 0
#    endif

void via_qmk_backlight_get_value(uint8_t *data) {
    uint8_t *value_id   = &(data[0]);
    uint8_t *value_data = &(data[1]);
    switch (*value_id) {
        case id_qmk_backlight_brightness: {
            // level / BACKLIGHT_LEVELS * 255
            value_data[0] = ((uint16_t)get_backlight_level()) * 255 / BACKLIGHT_LEVELS;
            break;
        }
        case id_qmk_backlight_effect: {
#    ifdef BACKLIGHT_BREATHING
            value_data[0] = is_backlight_breathing() ? 1 : 0;
#    else
            value_data[0] = 0;
#    endif
            break;
        }
    }
}

void via_qmk_backlight_set_value(uint8_t *data) {
    uint8_t *value_id   = &(data[0]);
    uint8_t *value_data = &(data[1]);
    switch (*value_id) {
        case id_qmk_backlight_brightness: {
            // level / 255 * BACKLIGHT_LEVELS
            backlight_level_noeeprom(((uint16_t)value_data[0]) * BACKLIGHT_LEVELS / 255);
            break;
        }
        case id_qmk_backlight_effect: {
#    ifdef BACKLIGHT_BREATHING
            if (value_data[0] == 0) {
                backlight_disable_breathing();
            } else {
                backlight_enable_breathing();
            }
#    endif
            break;
        }
    }
}

#endif  // #if defined(VIA_QMK_BACKLIGHT_ENABLE)

#if defined(VIA_QMK_RGBLIGHT_ENABLE)

void via_qmk_rgblight_get_value(uint8_t *data) {
    uint8_t *value_id   = &(data[0]);
    uint8_t *value_data = &(data[1]);
    switch (*value_id) {
        case id_qmk_rgblight_brightness: {
            value_data[0] = rgblight_get_val();
            break;
        }
        case id_qmk_rgblight_effect: {
            value_data[0] = rgblight_get_mode();
            break;
        }
        case id_qmk_rgblight_effect_speed: {
            value_data[0] = rgblight_get_speed();
            break;
        }
        case id_qmk_rgblight_color: {
            value_data[0] = rgblight_get_hue();
            value_data[1] = rgblight_get_sat();
            break;
        }
    }
}

void via_qmk_rgblight_set_value(uint8_t *data) {
    uint8_t *value_id   = &(data[0]);
    uint8_t *value_data = &(data[1]);
    switch (*value_id) {
        case id_qmk_rgblight_brightness: {
            rgblight_sethsv_noeeprom(rgblight_get_hue(), rgblight_get_sat(), value_data[0]);
            break;
        }
        case id_qmk_rgblight_effect: {
            rgblight_mode_noeeprom(value_data[0]);
            if (value_data[0] == 0) {
                rgblight_disable_noeeprom();
            } else {
                rgblight_enable_noeeprom();
            }
            break;
        }
        case id_qmk_rgblight_effect_speed: {
            rgblight_set_speed_noeeprom(value_data[0]);
            break;
        }
        case id_qmk_rgblight_color: {
            rgblight_sethsv_noeeprom(value_data[0], value_data[1], rgblight_get_val());
            break;
        }
    }
}

#endif  // #if defined(VIA_QMK_RGBLIGHT_ENABLE)
