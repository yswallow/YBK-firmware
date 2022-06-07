#include "sdk_errors.h"
#include "keyboard_config.h"
#include "via_fds.h"
#include "dynamic_keymap.h"
#include "app_timer.h"
#include "nrfx_rtc.h"

#ifndef __KEYBOARD_H
#define __KEYBOARD_H
typedef struct {
    uint8_t kc;
    uint8_t application;
    uint8_t row;
    uint8_t col;
    uint32_t tick;
    volatile bool press;
} keys_t;

typedef struct {
    uint8_t row;
    uint8_t col;
    uint32_t tick;
} debouncing_keys_t;

typedef struct {
    ret_code_t (*keycode_append)(uint8_t);
    ret_code_t (*keycode_remove)(uint8_t);
    ret_code_t (*send_consumer)(uint8_t,bool);
    ret_code_t (*reset)(void);
    ret_code_t (*handle_mouse)(uint16_t,bool);
    ret_code_t (*tick_handler_mouse)(keys_t*);
} keyboard_hid_functions_t;

typedef enum  {
    KEYBOARD_TYPE_MATRIX,
    KEYBOARD_TYPE_DIRECTPIN,
    KEYBOARD_TYPE_STENO,
} keyboard_type_t;

typedef struct {
    uint8_t col_pins_count;
    uint8_t row_pins_count;
    uint8_t *col_pins;
    uint8_t *row_pins;
    uint8_t col2row;
} matrix_keyboard_t;

typedef struct {
    uint8_t pins_count;
    uint8_t *pins;
} directpin_keyboard_t;

typedef void* keyboard_definision_t;

typedef struct {
    uint8_t central_cols_count;
} split_keyboard_t;

typedef struct {
    keyboard_type_t keyboard_type;
    keyboard_definision_t keyboard_definision;
    split_keyboard_t split_keyboard;
    uint8_t kbd_cols_count;
    uint8_t kbd_rows_count;
    uint8_t kbd_power_led;
    uint8_t kbd_power_led_enable;
    uint8_t default_layer;
    uint8_t neopixel_pin;
    uint8_t neopixel_length;
    void (*scan_method)(keyboard_type_t, keyboard_definision_t);
    void (*init_method)(keyboard_type_t, keyboard_definision_t);
    ret_code_t (*sleep_prepare)(keyboard_type_t, keyboard_definision_t);
} keyboard_t;

void keyboard_scan(keyboard_t keyboard);
void keyboard_init(keyboard_t keyboard);
ret_code_t keyboard_sleep_prepare(void);

void keypress(uint8_t row, uint8_t col, bool debouncing);
void keyrelease(uint8_t row, uint8_t col, bool debouncing);
uint8_t get_active_layer(void);
void release_prev_tick_kc(void);

void restart_timeout_timer(void);
void sleep_mode_enter(void *ptr);
void kbd_tick_handler(void* p_context);

#define PRESS_KEYS_MAX 10
#define DEBOUNCING_DELAY_MS 30
#define DEBOUNCING_TICKS 8
#define TAPPING_TERM_TICK_MS 5
#define MOUSE_MOVE_INTERVAL_TICKS ( MOUSE_MOVE_INTERVAL / TAPPING_TERM_TICK_MS )
#define KEYBOARD_TIMEOUT_TICKS APP_TIMER_TICKS(600000)

extern keyboard_hid_functions_t hid_functions;
extern uint32_t keypress_bitmap[KBD_SETTING_ROW_PINS_MAX];
extern uint32_t debouncing_bitmap[KBD_SETTING_ROW_PINS_MAX];

extern keys_t keypress_status[PRESS_KEYS_MAX];

#endif //__KEYBOARD_H