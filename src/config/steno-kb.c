#include "matrix_keyboard.h"
#include "general-kb.h"
#include "steno_matrix.h"
#include "nrf_gpio.h"

#ifdef SCAN_MATRIX_STENO

matrix_keyboard_t definision = {
    .col2row = 1
};

keyboard_t my_keyboard = {
    .keyboard_type = KEYBOARD_TYPE_STENO,
    .keyboard_definision = &definision,
    .split_keyboard = (split_keyboard_t) { .central_cols_count = 0 },
    .kbd_power_led_enable = 0,
    .scan_method = steno_scan,
    .init_method = steno_init,
    .sleep_prepare = steno_sleep_prepare
};

ret_code_t keyboard_sleep_prepare(void) {
    if(my_keyboard.kbd_power_led_enable) {
        nrf_gpio_pin_clear(my_keyboard.kbd_power_led);
    }
    return steno_sleep_prepare(KEYBOARD_TYPE_STENO, (keyboard_definision_t)&definision);
}

#endif