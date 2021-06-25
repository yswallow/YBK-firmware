#ifdef KEYBOARD_DEBUG_HID

void keyboard_debug_init(void);
#define KEYBOARD_DEBUG_HID_INIT() keyboard_debug_init()

#else
#define KEYBOARD_DEBUG_HID_INIT()
#endif
