#include "usb_hiddevice.h"

#ifdef BLE_RAW_HID
#include "ble_hiddevice.h"
#endif

#ifndef __RAW_HID_H
#define __RAW_HID_H

#define RAW_REP_SIZE 32

static inline void raw_hid_send(uint8_t *data, uint8_t length) {
#ifdef BLE_RAW_HID
    raw_hid_send_ble(data, length);
#endif
    raw_hid_send_usb(data, length);
}

#endif // __RAW_HID_H
