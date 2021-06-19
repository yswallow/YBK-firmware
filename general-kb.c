#include "matrix_keyboard.h"
#include "general-kb.h"

matrix_keyboard_t definision = {
    .col2row = 1
};

keyboard_t my_keyboard = {
    .keyboard_type = KEYBOARD_TYPE_MATRIX,
    .keyboard_definision = &definision,
    .scan_method = matrix_scan,
    .init_method = matrix_init,
    .sleep_prepare = matrix_sleep_prepare
};

ret_code_t keyboard_sleep_prepare(void) {
    nrf_gpio_pin_clear(13);
    return matrix_sleep_prepare(KEYBOARD_TYPE_MATRIX, (keyboard_definision_t)&definision);
}