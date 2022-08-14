#include "keyboard_generic.h"
#include "keyboard_config.h"
#include "nrf_gpio.h"

void matrix_scan(keyboard_type_t type, void* ptr) {
    matrix_keyboard_t defs = *(matrix_keyboard_t*) ptr;
    if(type != KEYBOARD_TYPE_MATRIX) {
        return;
    }
    for(uint8_t i=0; i<defs.row_pins_count; i++) {
        //nrf_gpio_cfg_output(defs.row_pins[i]);
        nrf_gpio_pin_clear(defs.row_pins[i]);

        for(uint8_t j=0; j<defs.col_pins_count; j++) {
            if(! (debouncing_bitmap[i] & (1UL<<j)) ) {
                uint8_t val = nrf_gpio_pin_read(defs.col_pins[j]);
             
                if(val == 0 ) {
                    if( keypress_bitmap[i] & (1UL<<j) ) {
                    } else {
                        keypress_bitmap[i] |= (1UL<<j);
                        keypress(i,j, true);
                    }
                } else {
                    if( keypress_bitmap[i] & (1UL<<j) ) {
                        keypress_bitmap[i] &= ~(1UL<<j);
                        keyrelease(i,j, true);
                    }
                }
            }
        }

        nrf_gpio_pin_set(defs.row_pins[i]);
    }
}

void matrix_init(keyboard_type_t type, keyboard_definision_t ptr) {
    matrix_keyboard_t defs = *(matrix_keyboard_t*) ptr;
    if(type != KEYBOARD_TYPE_MATRIX) {
        return;
    }

    if(defs.col2row) {
        for(uint8_t i=0; i<defs.row_pins_count; i++) {
            nrf_gpio_cfg(defs.row_pins[i], NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0D1, NRF_GPIO_PIN_NOSENSE);
            nrf_gpio_pin_set(defs.row_pins[i]);
        }

        for(uint8_t i=0; i<defs.col_pins_count; i++) {
            nrf_gpio_cfg_input(defs.col_pins[i],NRF_GPIO_PIN_PULLUP);
        }
    }
}

ret_code_t matrix_sleep_prepare(keyboard_type_t type, keyboard_definision_t ptr) {
    matrix_keyboard_t defs = *(matrix_keyboard_t*) ptr;
    if(type != KEYBOARD_TYPE_MATRIX) {
        return NRF_ERROR_INVALID_PARAM;
    }

    if(defs.col2row) {
        for(uint8_t i=0; i<defs.row_pins_count; i++) {
            nrf_gpio_pin_clear(defs.row_pins[i]);
        }

        for(uint8_t i=0; i<defs.col_pins_count; i++) {
            nrf_gpio_cfg_sense_input(defs.col_pins[i],NRF_GPIO_PIN_PULLUP,NRF_GPIO_PIN_SENSE_LOW);
        }
    }
    return NRF_SUCCESS;
}