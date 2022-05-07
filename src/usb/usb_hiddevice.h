#include "sdk_errors.h"
#include "keyboard_generic.h"

#ifndef __USB_HIDDEVICE_H
#define __USB_HIDDEVICE_H

#define USB_HID_KBD_REP_LEN 8

void usb_keyboard_init(void);
ret_code_t keycode_append_usb(uint8_t kc);
ret_code_t keycode_remove_usb(uint8_t kc);
ret_code_t keyboard_reset_usb(void);

void usb_hid_consumer_init(void);
ret_code_t send_consumer_usb(uint8_t code, bool press);

void usb_mouse_init(void);
ret_code_t mouse_reset_usb(void);
ret_code_t handle_keycode_mouse_usb(uint16_t keycode, bool press);
ret_code_t tick_handler_mouse_usb(keys_t *p_key);

void usb_hid_raw_init(void);
void raw_hid_send_usb(uint8_t *data, uint8_t length);

extern keyboard_hid_functions_t usb_hid_functions;

extern uint8_t usb_keyboard_rep_buffer[USB_HID_KBD_REP_LEN];

static inline void usb_hid_init(void) {
    usb_keyboard_init();
    usb_hid_raw_init();
    usb_mouse_init();
    usb_hid_consumer_init();
}
#endif // __USB_HIDDEVICE_H