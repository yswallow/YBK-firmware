#include <string.h>
#include "keyboard_generic.h"
#include "keyboard_config.h"
#include "nrf_gpio.h"

#define STENO_PRESSED 0x02
#define STENO_RELEASING 0x01
#define STENO_RELEASED 0x00
#define STENO_PRESS_STATUS_SIZE 20
#define STENO_PHYSICAL_ROW_MAX (KBD_SETTING_ROW_PINS_MAX/2)

#define STENO_WAIT_FOR_NEXT_INPUT_COUNT ( 10 + STENO_PRESSED ) 

typedef struct {
    uint8_t row;
    uint8_t col;
    uint8_t tick;
} steno_press_status_t;

static steno_press_status_t steno_press_status[STENO_PRESS_STATUS_SIZE];
static uint32_t steno_press_bitmap[STENO_PHYSICAL_ROW_MAX];
static uint32_t steno_releasing_bitmap[STENO_PHYSICAL_ROW_MAX];

static void steno_tick(void) {
    for(uint8_t i=0; i<STENO_PRESS_STATUS_SIZE; i++) {
        if(2<steno_press_status[i].tick) {
            if( --steno_press_status[i].tick == STENO_PRESSED) {
                uint8_t row = steno_press_status[i].row*2;
                uint8_t col = steno_press_status[i].col;
                keypress(row, col, false);
                keypress_bitmap[row] |= (1UL<<col);
            }
        }

        if(steno_press_status[i].tick==STENO_RELEASED) {
            break;
        }
    }
}

static bool physical_pressing(uint8_t physical_row, uint8_t col) {
    for(uint8_t i=0; i<STENO_PRESS_STATUS_SIZE; i++) {
        if(steno_press_status[i].tick==STENO_RELEASED) {
            return false;
        }

        if( steno_press_status[i].row==physical_row && steno_press_status[i].col==col ) {
            if( 2<steno_press_status[i].tick ) {
                return true;
            } else {
                return false;
            }
        }
    }

    return false;
}

static void set_physical_released(uint8_t physical_row, uint8_t col) {
    uint8_t backward_count = 0;
    for(uint8_t i=0; i<STENO_PRESS_STATUS_SIZE; i++) {
        if(steno_press_status[i].tick==STENO_RELEASED) {
            break;
        }
        
        if(steno_press_status[i].row==physical_row && steno_press_status[i].col==col) {
            backward_count++;
        }

        if(backward_count) {
            memcpy(steno_press_status+i, steno_press_status+i+backward_count, sizeof(steno_press_status_t));
        }
    }
    steno_releasing_bitmap[physical_row] &= ~(1UL<<col);
    steno_press_bitmap[physical_row] &= ~(1UL<<col);
}

static void set_physical_releasing(uint8_t physical_row, uint8_t col) {
    for(uint8_t i=0; i<STENO_PRESS_STATUS_SIZE; i++) {
        if(steno_press_status[i].tick==STENO_RELEASED) {
            break;
        }

        if( steno_press_status[i].row==physical_row && steno_press_status[i].col==col ) {
            steno_press_status[i].tick = STENO_RELEASING;
            break;
        }
    }
    steno_releasing_bitmap[physical_row] |= (1UL<<col);
    steno_press_bitmap[physical_row] |= (1UL<<col);
}

static void set_physical_pressing(uint8_t physical_row, uint8_t col) {
    for(uint8_t i=0; i<STENO_PRESS_STATUS_SIZE; i++) {
        if( steno_press_status[i].row==physical_row && steno_press_status[i].col==col ) {
            steno_press_status[i].tick = STENO_WAIT_FOR_NEXT_INPUT_COUNT;
            break;
        }
        
        if(steno_press_status[i].tick==STENO_RELEASED) {
            steno_press_status[i].row = physical_row;
            steno_press_status[i].col = col;
            steno_press_status[i].tick = STENO_WAIT_FOR_NEXT_INPUT_COUNT;
            break;
        }
    }
    steno_press_bitmap[physical_row] |= (1UL<<col);
}

static void set_physical_pressed(uint8_t physical_row, uint8_t col) {
    for(uint8_t i=0; i<STENO_PRESS_STATUS_SIZE; i++) {
        if( steno_press_status[i].row==physical_row && steno_press_status[i].col==col ) {
            steno_press_status[i].tick = STENO_PRESSED;
            break;
        }
        
        if(steno_press_status[i].tick==STENO_RELEASED) {
            steno_press_status[i].row = physical_row;
            steno_press_status[i].col = col;
            steno_press_status[i].tick = STENO_PRESSED;
            break;
        }
    }
    
    steno_press_bitmap[physical_row] |= (1UL<<col);
}

static int8_t check_combo_pressed(uint8_t physical_row, uint8_t col) {
    if( physical_row==0 ) {
        if( keypress_bitmap[1] & (1UL<<col) ) {
            return 1;
        } else {
            return 0;
        }
    } else if( physical_row==STENO_PHYSICAL_ROW_MAX ) {
        if( keypress_bitmap[physical_row*2-1] & (1UL<<col) ) {
            return -1;
        } else {
            return 0;
        }
    } else {
        if( keypress_bitmap[physical_row*2+1] & (1UL<<col) ) {
            return 1;
        } else if( keypress_bitmap[physical_row*2-1] & (1UL<<col) ){
            return -1;
        } else {
            return 0;
        }
    }
}

static int8_t check_combo_pressing(uint8_t physical_row, uint8_t col) {
    if( physical_row==0 ) {
        if( physical_pressing(1,col) ) {
            return 1;
        } else {
            return 0;
        }
    } else if( physical_row==STENO_PHYSICAL_ROW_MAX ) {
        if( physical_pressing( physical_row-1, col) ) {
            return -1;
        } else {
            return 0;
        }
    } else {
        if( physical_pressing( physical_row+1, col ) ) {
            return 1;
        } else if( physical_pressing( physical_row-1, col) ){
            return -1;
        } else {
            return 0;
        }
    }
}

void steno_scan(keyboard_type_t type, void* ptr) {
    matrix_keyboard_t defs = *(matrix_keyboard_t*) ptr;
    if(type != KEYBOARD_TYPE_STENO) {
        return;
    }
    for(uint8_t i=0; i<defs.row_pins_count/2; i++) {
        nrf_gpio_pin_clear(defs.row_pins[i]);

        for(uint8_t j=0; j<defs.col_pins_count; j++) {
            uint8_t col = j;
            uint8_t val = nrf_gpio_pin_read(defs.col_pins[j]);
            
            if(val) {
                // released
                if( steno_press_bitmap[i] & (1UL<<col) ) {
                    if( steno_releasing_bitmap[i] & (1UL<<col) ) {
                        set_physical_released(i,col);
                    } else {
                        int8_t combo = check_combo_pressed(i, col);
                    
                        keyrelease(2*i+combo, col, false);
                        keypress_bitmap[2*i+combo] &= ~(1UL<<col);

                        set_physical_released(i, col);
                        if(combo) {
                            set_physical_releasing(i+combo, col);
                        }
                    }
                } else {
                }
            } else {
                // press
                if( steno_press_bitmap[i] & (1UL<<col) ) {
                } else {
                    if( steno_releasing_bitmap[i] & (1UL<<col) ) {

                    } else {
                        int8_t combo = check_combo_pressing(i, col);
                    
                        if( combo ) {
                            keypress(2*i+combo, col, false);
                            keypress_bitmap[2*i+combo] |= (1UL<<col);
                            set_physical_pressed(i, col);
                            set_physical_pressed(i+combo, col);
                        } else {
                            set_physical_pressing(i, col);
                        }
                    }
                }
            }
        }

        nrf_gpio_pin_set(defs.row_pins[i]);
    }
    steno_tick();
}

void steno_init(keyboard_type_t type, keyboard_definision_t ptr) {
    matrix_keyboard_t defs = *(matrix_keyboard_t*) ptr;

    memset(steno_press_status, STENO_RELEASED, sizeof(steno_press_status));
    memset(steno_releasing_bitmap, 0, sizeof(steno_releasing_bitmap));
    memset(steno_press_bitmap, 0, sizeof(steno_press_bitmap));
    
    if(type != KEYBOARD_TYPE_STENO) {
        return;
    }

    if(defs.col2row) {
        for(uint8_t i=0; i<defs.row_pins_count/2; i++) {
            nrf_gpio_cfg_output(defs.row_pins[i]);
            nrf_gpio_pin_set(defs.row_pins[i]);
        }

        for(uint8_t i=0; i<defs.col_pins_count; i++) {
            nrf_gpio_cfg_input(defs.col_pins[i],NRF_GPIO_PIN_PULLUP);
        }
    }
}

ret_code_t steno_sleep_prepare(keyboard_type_t type, keyboard_definision_t ptr) {
    matrix_keyboard_t defs = *(matrix_keyboard_t*) ptr;
    if(type != KEYBOARD_TYPE_STENO) {
        return NRF_ERROR_INVALID_PARAM;
    }

    if(defs.col2row) {
        for(uint8_t i=0; i<defs.row_pins_count/2; i++) {
            nrf_gpio_pin_clear(defs.row_pins[i]);
        }

        for(uint8_t i=0; i<defs.col_pins_count; i++) {
            nrf_gpio_cfg_sense_input(defs.col_pins[i],NRF_GPIO_PIN_PULLUP,NRF_GPIO_PIN_SENSE_LOW);
        }
    }
    return NRF_SUCCESS;
}