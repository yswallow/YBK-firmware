#ifndef __BLE_HIDDEVICE_H
#define __BLE_HIDDEVICE_H

#include "nrf_timer.h"
#include "app_timer.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "sdk_errors.h"

#include "keyboard_generic.h"

// BLE DEFINES START

#define BLE_CONSUMER_ENABLE

#ifdef BLE_CONSUMER_ENABLE
#define BLE_INPUT_REPORT_COUNT              4
#else
#define BLE_INPUT_REPORT_COUNT              3
#endif

#define OUTPUT_REPORT_KEYS_INDEX            0                                          /**< Index of Output Report. */
#define OUTPUT_REPORT_MAX_LEN               1                                          /**< Maximum length of Output Report. */
#define INPUT_REPORT_KEYS_INDEX             0                                          /**< Index of Input Report. */
#define INPUT_REP_REF_ID                    1                                          /**< Id of reference to Keyboard Input Report. */
#define OUTPUT_REP_REF_ID                   INPUT_REP_REF_ID                                           /**< Id of reference to Keyboard Output Report. */

#define FEATURE_REPORT_MAX_LEN              0 

#define INPUT_REPORT_MOUSE_INDEX            2
#define INPUT_REP_REF_MOUSE_ID              2
#define INPUT_REPORT_MOUSE_MAX_LEN          4

#define INPUT_REPORT_CONSUMER_INDEX         3
#define INPUT_REPORT_CONSUMER_ID            3
#define INPUT_REPORT_CONSUMER_MAX_LEN       1

#define INPUT_REPORT_RAW_INDEX              1
#define OUTPUT_REPORT_RAW_INDEX             1
#define INPUT_REP_REF_RAW_ID                8
#define OUTPUT_REP_REF_RAW_ID               8
#define INPUT_REPORT_RAW_MAX_LEN            32
#define OUTPUT_REPORT_RAW_MAX_LEN           32

#define MAX_BUFFER_ENTRIES                  5                                          /**< Number of elements that can be enqueued */

#define BASE_USB_HID_SPEC_VERSION           0x0101                                     /**< Version number of base USB HID Specification implemented by this application. */

#define BLE_HID_KBD_REP_LEN                 8                                          /**< Maximum length of the Input Report characteristic. */
#define INPUT_REPORT_KEYS_MAX_LEN           BLE_HID_KBD_REP_LEN

#define BLE_HID_KBD_MODS_INDEX              0                                          /**< Position of the modifier byte in the Input Report. */
#define BLE_HID_KBD_KEYS_START              2                                          /**< The start position of the key scan code in a HID Report. */

#define MAX_KEYS_IN_ONE_REPORT              (INPUT_REPORT_KEYS_MAX_LEN - SCAN_CODE_POS)/**< Maximum number of key presses that can be sent in one Input Report. */



// BLE DEFINES END

// variables

extern keyboard_hid_functions_t ble_hid_functions;
extern uint8_t ble_keyboard_rep_buffer[BLE_HID_KBD_REP_LEN];
extern bool m_ble_hid_rep_pending;

typedef struct {
    uint8_t buttons;
    int8_t x;
    int8_t y;
    int8_t wheel;
} mouse_report_ble_t;

// functions

void hids_init(void);
void ble_keyboard_init(void);

ret_code_t keycode_append_ble(uint8_t kc);
ret_code_t keycode_remove_ble(uint8_t kc);
ret_code_t keyboard_reset_ble(void);

ret_code_t mouse_reset_ble(void);
ret_code_t handle_keycode_mouse_ble(uint16_t keycode, bool press);
ret_code_t tick_handler_mouse_ble(keys_t *p_key);
ret_code_t ble_mouse_init(void);

ret_code_t raw_hid_send_ble(uint8_t *data, uint8_t length);
ret_code_t send_consumer_ble(uint8_t code, bool press);

#endif // __BLE_HIDDEVICE_H