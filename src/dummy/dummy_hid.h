#include "keyboard_generic.h"

#ifndef __DUMMY_HID_H
#define __DUMMY_HID_H
ret_code_t keycode_append_dummy(uint8_t kc);
ret_code_t keycode_remove_dummy(uint8_t kc);
ret_code_t keyboard_reset_dummy(void);
ret_code_t send_consumer_dummy(uint8_t code, bool press);
ret_code_t mouse_reset_dummy(void);
ret_code_t handle_keycode_mouse_dummy(uint16_t keycode, bool press);
ret_code_t tick_handler_mouse_dummy(keys_t *p_key);

extern const keyboard_hid_functions_t dummy_hid_functions;
#endif // __DUMMY_HID_H
