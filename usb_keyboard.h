#include "sdk_errors.h"
#include "app_usbd_hid_kbd.h"
#include "keyboard_generic.h"

#define USB_HID_KBD_IN_QUEUE_SIZE 1
#define USB_HID_KBD_OUT_REP_MAXSIZE 1
#define USB_HID_KBD_FEATURE_REP_MAXSIZE 1

#define USB_HID_KBD_MODS_INDEX 0
#define USB_HID_KBD_KEYS_START 2
#define USB_HID_KBD_REP_LEN 8

ret_code_t keycode_append_usb(uint8_t kc);
ret_code_t keycode_remove_usb(uint8_t kc);
ret_code_t keyboard_reset_usb(void);
//ret_code_t handle_keycode_usb(uint16_t keycode, bool press);
ret_code_t send_consumer_usb(uint8_t code, bool press);

void usb_keyboard_init(void);

extern const app_usbd_hid_generic_t m_app_hid_kbd;

extern keyboard_hid_functions_t usb_hid_functions;
extern uint8_t usb_keyboard_rep_buffer[USB_HID_KBD_REP_LEN];