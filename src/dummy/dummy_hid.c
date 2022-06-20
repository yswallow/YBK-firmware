#include "keyboard_generic.h"

ret_code_t keycode_append_dummy(uint8_t kc) { return NRF_SUCCESS; }
ret_code_t keycode_remove_dummy(uint8_t kc) { return NRF_SUCCESS; }
ret_code_t keyboard_reset_dummy(void) { return NRF_SUCCESS; }
ret_code_t send_consumer_dummy(uint8_t code, bool press) { return NRF_SUCCESS; }
ret_code_t mouse_reset_dummy(void) { return NRF_SUCCESS; }
ret_code_t handle_keycode_mouse_dummy(uint16_t keycode, bool press) { return NRF_SUCCESS; }
ret_code_t tick_handler_mouse_dummy(keys_t *p_key) { return NRF_SUCCESS; }

const keyboard_hid_functions_t dummy_hid_functions = {
    .keycode_append = keycode_append_dummy,
    .keycode_remove = keycode_remove_dummy,
    .send_consumer = send_consumer_dummy,
    .reset = keyboard_reset_dummy,
    .handle_mouse = handle_keycode_mouse_dummy,
    .tick_handler_mouse = tick_handler_mouse_dummy
};