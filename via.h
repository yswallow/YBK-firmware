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

#pragma once
#include "raw_hid.h"
#include "via_fds.h"
//#include "tmk_core/common/eeconfig.h"  // for EECONFIG_SIZE

// Keyboard level code can change where VIA stores the magic.
// The magic is the build date YYMMDD encoded as BCD in 3 bytes,
// thus installing firmware built on a different date to the one
// already installed can be detected and the EEPROM data is reset.
// The only reason this is important is in case EEPROM usage changes
// and the EEPROM was not explicitly reset by bootmagic lite.
#ifndef VIA_EEPROM_MAGIC_ADDR
#    define VIA_EEPROM_MAGIC_ADDR (EECONFIG_SIZE)
#endif

#define VIA_EEPROM_LAYOUT_OPTIONS_ADDR (VIA_EEPROM_MAGIC_ADDR + 3)

// Changing the layout options size after release will invalidate EEPROM,
// but this is something that should be set correctly on initial implementation.
// 1 byte is enough for most uses (i.e. 8 binary states, or 6 binary + 1 ternary/quaternary )
#ifndef VIA_EEPROM_LAYOUT_OPTIONS_SIZE
#    define VIA_EEPROM_LAYOUT_OPTIONS_SIZE 1
#endif

// Allow override of the layout options default value.
// This requires advanced knowledge of how VIA stores layout options
// and is only really useful for setting a boolean layout option
// state to true by default.
#ifndef VIA_EEPROM_LAYOUT_OPTIONS_DEFAULT
#    define VIA_EEPROM_LAYOUT_OPTIONS_DEFAULT 0x00000000
#endif

// The end of the EEPROM memory used by VIA
// By default, dynamic keymaps will start at this if there is no
// custom config
#define VIA_EEPROM_CUSTOM_CONFIG_ADDR (VIA_EEPROM_LAYOUT_OPTIONS_ADDR + VIA_EEPROM_LAYOUT_OPTIONS_SIZE)

#ifndef VIA_EEPROM_CUSTOM_CONFIG_SIZE
#    define VIA_EEPROM_CUSTOM_CONFIG_SIZE 0
#endif

// This is changed only when the command IDs change,
// so VIA Configurator can detect compatible firmware.
#define VIA_PROTOCOL_VERSION 0x0009

#define  id_get_protocol_version                  0x01
#define    id_get_keyboard_value                    0x02
    #define id_set_keyboard_value                    0x03
    #define id_dynamic_keymap_get_keycode            0x04
    #define id_dynamic_keymap_set_keycode            0x05
    #define id_dynamic_keymap_reset                  0x06
    #define id_lighting_set_value                    0x07
    #define id_lighting_get_value                    0x08
    #define id_lighting_save                         0x09
    #define id_eeprom_reset                          0x0A
    #define id_bootloader_jump                       0x0B
    #define id_dynamic_keymap_macro_get_count        0x0C
    #define id_dynamic_keymap_macro_get_buffer_size  0x0D
    #define id_dynamic_keymap_macro_get_buffer       0x0E
    #define id_dynamic_keymap_macro_set_buffer       0x0F
    #define id_dynamic_keymap_macro_reset            0x10
    #define id_dynamic_keymap_get_layer_count        0x11
    #define id_dynamic_keymap_get_buffer             0x12
    #define id_dynamic_keymap_set_buffer             0x13
    #define id_unhandled                             0xFF

    #define id_uptime               0x01
    #define id_layout_options       0x02
    #define id_switch_matrix_state  0x03


/*
// Can be called in an overriding via_init_kb() to test if keyboard level code usage of
// EEPROM is invalid and use/save defaults.
bool via_eeprom_is_valid(void);

// Sets VIA/keyboard level usage of EEPROM to valid/invalid
// Keyboard level code (eg. via_init_kb()) should not call this
void via_eeprom_set_valid(bool valid);

// Flag QMK and VIA/keyboard level EEPROM as invalid.
// Used in bootmagic_lite() and VIA command handler.
// Keyboard level code should not need to call this.
void via_eeprom_reset(void);

// Called by QMK core to initialize dynamic keymaps etc.
void via_init(void);

// Used by VIA to store and retrieve the layout options.
uint32_t via_get_layout_options(void);
void     via_set_layout_options(uint32_t value);

// Called by QMK core to process VIA-specific keycodes.
bool process_record_via(uint16_t keycode, keyrecord_t *record);
*/

void raw_hid_receive(uint8_t *data, uint8_t length);