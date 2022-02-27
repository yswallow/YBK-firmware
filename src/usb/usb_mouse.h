#include "sdk_errors.h"
#include "keyboard_generic.h"
#include "app_usbd_hid_mouse.h"

#define USB_HID_MOUSE_BUTTONS_INDEX 2
#define USB_HID_MOUSE_REP_LEN 4

ret_code_t mouse_reset_usb(void);
ret_code_t handle_keycode_mouse_usb(uint16_t keycode, bool press);
ret_code_t tick_handler_mouse_usb(keys_t *p_key);
void usb_mouse_init(void);

extern const app_usbd_hid_mouse_t m_app_hid_mouse;