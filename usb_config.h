#include "nrf_drv_usbd.h"
#include "nordic_common.h"
#include "app_usbd_hid_generic.h"
#include "keyboard_config.h"

#ifndef __MAIN_H
#define __MAIN_H

#define ENABLE_USB_HID_KBD
#define ENABLE_USB_HID_RAW
#define ENABLE_USB_HID_MOUSE
#define ENABLE_USB_HID_CONSUMER

enum USBD_INTERFACE {
    USBD_IFACE_BASE = -1
#ifdef ENABLE_USB_HID_KBD
    ,HID_KEYBOARD_INTERFACE
#endif
#ifdef ENABLE_USB_HID_RAW
    ,HID_RAW_INTERFACE
#endif
#ifdef ENABLE_USB_HID_MOUSE
    ,HID_MOUSE_INTERFACE
#endif
#ifdef ENABLE_USB_HID_CONSUMER
    ,HID_CONSUMER_INTERFACE
#endif
};

enum USBD_EPIN_NUMS {
    EPIN_NUM_BASE = 0 // 0 is reserved for control
#ifdef ENABLE_USB_HID_KBD
    ,EPIN_NUM_HID_KEYBOARD
#endif
#ifdef ENABLE_USB_HID_RAW
    ,EPIN_NUM_HID_RAW
#endif
#ifdef ENABLE_USB_HID_MOUSE
    ,EPIN_NUM_HID_MOUSE
#endif
#ifdef ENABLE_USB_HID_CONSUMER
    ,EPIN_NUM_HID_CONSUMER
#endif
};

enum USBD_EPOUT_NUMS {
    EPOUT_NUM_BASE = 0  // 0 is reserved for control
#ifdef ENABLE_USB_HID_KBD
    ,EPOUT_NUM_HID_KEYBOARD
#endif
#ifdef ENABLE_USB_HID_RAW
    ,EPOUT_NUM_HID_RAW
#endif
};


#ifdef ENABLE_USB_HID_KBD
#define HID_KEYBOARD_EPIN NRF_DRV_USBD_EPIN(EPIN_NUM_HID_KEYBOARD)
#define HID_KEYBOARD_EPOUT NRF_DRV_USBD_EPOUT(EPOUT_NUM_HID_KEYBOARD)
#endif

#ifdef ENABLE_USB_HID_RAW
#define HID_RAW_EPIN NRF_DRV_USBD_EPIN(EPIN_NUM_HID_RAW)
#define HID_RAW_EPOUT NRF_DRV_USBD_EPOUT(EPOUT_NUM_HID_RAW)
#endif

#ifdef ENABLE_USB_HID_MOUSE
#define HID_MOUSE_EPIN NRF_DRV_USBD_EPIN(EPIN_NUM_HID_MOUSE)
#endif

#ifdef ENABLE_USB_HID_CONSUMER
#define HID_CONSUMER_EPIN NRF_DRV_USBD_EPIN(EPIN_NUM_HID_CONSUMER)
#endif

// fds configuration

/* Flag to check fds initialization. */





#endif // ifndef __MAIN_H