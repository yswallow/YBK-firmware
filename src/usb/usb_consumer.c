#include "sdk_errors.h"
#include "nrf_log.h"

#include "app_usbd_types.h"
#include "app_usbd_hid.h"
#include "app_usbd_hid_generic.h"
#include "app_usbd_hid_types.h"

#include "usb_config.h"

#define USB_CONSUMER_REPORT_IN_QUEUE_SIZE 1
#define USB_CONSUMER_REPORT_OUT_MAXSIZE   0
#define USB_CONSUMER_REPORT_FEATURE_MAXSIZE 0
#define USB_CONSUMER_REPORT_LEN 1

#define ENDPOINT_LIST_CONSUMER() ( HID_CONSUMER_EPIN )

#define CONSUMER_REPORT_DSC() { \
    0x05, 0x0C, \
    0x09, 0x01, \
    0xA1, 0x01, \
    0x09, 0x40, /* Menu */ \
    0x09, 0xE9, /* VOLU */ \
    0x09, 0xEA, /* VOLD */ \
    0x09, 0xE2, /* Mute */ \
    0x09, 0x30, /* Power */ \
    0x09, 0xCD, /* Play/Pause */ \
    0x09, 0xB3, /* Fast Forward */ \
    0x09, 0xB4, /* Rewind */ \
    0x15, 0x00, \
    0x25, 0x01, \
    0x75, 0x01, \
    0x95, 0x08, \
    0x81, 0x02, /* Input (Data, Variable, Absolute) */ \
    0xC0 \
}

APP_USBD_HID_GENERIC_SUBCLASS_REPORT_DESC(consumer_desc, CONSUMER_REPORT_DSC());

static const app_usbd_hid_subclass_desc_t * consumer_reps[] = {&consumer_desc};

static void hid_consumer_ev_handler(app_usbd_class_inst_t const * p_inst,
                                app_usbd_hid_user_event_t event);

bool m_report_pending_consumer = false;

uint8_t usb_consumer_report[USB_CONSUMER_REPORT_LEN];

APP_USBD_HID_GENERIC_GLOBAL_DEF(m_app_hid_consumer,
                                HID_CONSUMER_INTERFACE,
                                hid_consumer_ev_handler,
                                ENDPOINT_LIST_CONSUMER(),
                                consumer_reps,
                                USB_CONSUMER_REPORT_IN_QUEUE_SIZE,
                                USB_CONSUMER_REPORT_OUT_MAXSIZE,
                                USB_CONSUMER_REPORT_FEATURE_MAXSIZE,
                                APP_USBD_HID_SUBCLASS_NONE,
                                APP_USBD_HID_PROTO_GENERIC);


/*
static ret_code_t idle_handle_consumer(app_usbd_class_inst_t const * p_inst, uint8_t report_id)
{
    return NRF_ERROR_NOT_SUPPORTED;
}
*/

static void hid_consumer_ev_handler(app_usbd_class_inst_t const * p_inst,
                                app_usbd_hid_user_event_t event)
{
    switch (event)
    {
        case APP_USBD_HID_USER_EVT_OUT_REPORT_READY:
        {
            break;
        }
        case APP_USBD_HID_USER_EVT_IN_REPORT_DONE:
        {
            m_report_pending_consumer = false;
            break;
        }
        case APP_USBD_HID_USER_EVT_SET_BOOT_PROTO:
        {
            UNUSED_RETURN_VALUE(hid_generic_clear_buffer(p_inst));
            NRF_LOG_INFO("SET_BOOT_PROTO");
            break;
        }
        case APP_USBD_HID_USER_EVT_SET_REPORT_PROTO:
        {
            UNUSED_RETURN_VALUE(hid_generic_clear_buffer(p_inst));
            NRF_LOG_INFO("SET_REPORT_PROTO");
            break;
        }
        default:
            break;
    }
}


ret_code_t send_consumer_usb(uint8_t code, bool press) {
    STATIC_ASSERT(USB_CONSUMER_REPORT_LEN==1);
    if(press) {
        switch(code) {
        case 0x30:
            usb_consumer_report[0] = 1<<4;
            break;
        case 0x40:
            usb_consumer_report[0] = 1<<0;
            break;
        case 0xB3:
            usb_consumer_report[0] = 1<<6;
            break;
        case 0xB4:
            usb_consumer_report[0] = 1<<7;
            break;
        case 0xCD:
            usb_consumer_report[0] = 1<<5;
            break;
        case 0xE2:
            usb_consumer_report[0] = 1<<3;
            break;
        case 0xE9:
            usb_consumer_report[0] = 1<<1;
            break;
        case 0xEA:
            usb_consumer_report[0] = 1<<2;
            break;
        }
    } else {
        usb_consumer_report[0] = 0;
    }
    return app_usbd_hid_generic_in_report_set(
        &m_app_hid_consumer,
        usb_consumer_report,
        USB_CONSUMER_REPORT_LEN);
}

void usb_hid_consumer_init(void){
    ret_code_t ret;
    app_usbd_class_inst_t const * class_inst_consumer;
    class_inst_consumer = app_usbd_hid_generic_class_inst_get(&m_app_hid_consumer);

    ret = app_usbd_class_append(class_inst_consumer);
    APP_ERROR_CHECK(ret);
}