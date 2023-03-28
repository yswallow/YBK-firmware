#include "keyboard_generic.h"
#include "keyboard_config.h"
#include "nrf_gpio.h"

void matrix_scan(const keyboard_type_t type, const void* ptr) {
    const matrix_keyboard_t defs = *(matrix_keyboard_t*) ptr;
    if(type != KEYBOARD_TYPE_MATRIX) {
        return;
    }
    for(uint8_t i=0; i<defs.row_pins_count; i++) {
        nrf_gpio_cfg_output(defs.row_pins[i]);
        nrf_gpio_pin_clear(defs.row_pins[i]);

        for(uint8_t j=0; j<defs.col_pins_count; j++) {
            if(! (debouncing_bitmap[i] & (1UL<<j)) ) {
                uint8_t val = nrf_gpio_pin_read(defs.col_pins[j]);
             
                if(val == 0 ) {
                    if( keypress_bitmap[i] & (1UL<<j) ) {
                    } else {
                        keypress(i,j, true);
                    }
                } else {
                    if( keypress_bitmap[i] & (1UL<<j) ) {
                        keyrelease(i,j, true);
                    }
                }
            }
        }

        nrf_gpio_cfg_input(defs.row_pins[i], NRF_GPIO_PIN_PULLUP);
    }
}

void matrix_init(const keyboard_type_t type, const keyboard_definision_t ptr) {
    matrix_keyboard_t defs = *(matrix_keyboard_t*) ptr;
    if(type != KEYBOARD_TYPE_MATRIX) {
        return;
    }

    if(defs.col2row) {
        for(uint8_t i=0; i<defs.row_pins_count; i++) {
            nrf_gpio_cfg_input(defs.row_pins[i], NRF_GPIO_PIN_PULLUP);
        }
        
        for(uint8_t i=0; i<defs.col_pins_count; i++) {
            nrf_gpio_cfg_sense_input(defs.col_pins[i],NRF_GPIO_PIN_PULLUP,NRF_GPIO_PIN_NOSENSE);
        }
    }
}

ret_code_t matrix_sleep_prepare(const keyboard_type_t type, const keyboard_definision_t ptr) {
    const matrix_keyboard_t defs = *(matrix_keyboard_t*) ptr;
    if(type != KEYBOARD_TYPE_MATRIX) {
        return NRF_ERROR_INVALID_PARAM;
    }

    if(defs.col2row) {
        for(uint8_t i=0; i<defs.row_pins_count; i++) {
            nrf_gpio_cfg_output(defs.row_pins[i]);
            nrf_gpio_pin_clear(defs.row_pins[i]);
        }

        for(uint8_t i=0; i<defs.col_pins_count; i++) {
            nrf_gpio_cfg_sense_input(defs.col_pins[i],NRF_GPIO_PIN_PULLUP,NRF_GPIO_PIN_SENSE_LOW);
        }
    }
#ifdef USE_INTERRUPT
    NRF_GPIOTE->INTENSET = (1<<31);
#endif
    return NRF_SUCCESS;
}