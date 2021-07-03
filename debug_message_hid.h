#ifdef KEYBOARD_DEBUG_HID
void keyboard_debug_periodical_init(void);
void keyboard_debug_hid_set(uint8_t *data, uint8_t length);
void keyboard_debug_hid_response(uint8_t *data, uint8_t length);
void keyboard_debug_hid_register(uint8_t *data, uint8_t length);
void keyboard_debug_hid_register_string(uint8_t *data, uint8_t length);
#define KEYBOARD_DEBUG_HID_SET(data,length) keyboard_debug_hid_set(data, length)
#define KEYBOARD_DEBUG_HID_RESPONSE(data,length) keyboard_debug_hid_response(data,length)
#define KEYBOARD_DEBUG_HID_REGISTER(data,length) keyboard_debug_hid_register(data,length)
#define KEYBOARD_DEBUG_HID_INIT() keyboard_debug_periodical_init()
#define KEYBOARD_DEBUG_HID_REGISTER_STRING(data,length) keyboard_debug_hid_register_string(data,length)
#else
#define KEYBOARD_DEBUG_HID_SET(data,length)
#define KEYBOARD_DEBUG_HID_RESPONSE(data,length)
#define KEYBOARD_DEBUG_HID_REGISTER(data,length)
#define KEYBOARD_DEBUG_HID_INIT()
#define KEYBOARD_DEBUG_HID_REGISTER_STRING(data,length);
#endif
