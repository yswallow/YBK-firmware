#include "app_usbd_hid_generic.h"
#include "nrf_log.h"
#include "via.h"
#include <stddef.h>

#include "nrf.h"
#include "app_usbd_hid.h"
#include "usb_config.h"

#define RAW_ENABLE

#ifndef __RAW_HID_H
#define __RAW_HID_H
/**
 * @brief Number of reports defined in report descriptor.
 */
#define REPORT_IN_QUEUE_SIZE    1

/**
 * @brief Size of maximum output report. HID generic class will reserve
 *        this buffer size + 1 memory space. 
 *
 * Maximum value of this define is 63 bytes. Library automatically adds
 * one byte for report ID. This means that output report size is limited
 * to 64 bytes.
 */
#define REPORT_OUT_MAXSIZE  32

/**
 * @brief Feature report maximum size. HID generic class will reserve
 *        this buffer size + 1 memory space. 
 */
#define REPORT_FEATURE_MAXSIZE  31

#define HID_RAW_EP_COUNT 2
#define ENDPOINT_LIST_RAW() \
( \
    HID_RAW_EPIN, HID_RAW_EPOUT \
)
//#define HID_RAW_INTERFACE 4

#define RAW_REP_SIZE 32
#define RAW_REPORT_DSC() { \
  0x06, 0x60, 0xFF, \
  0x09, 0x61, \
  0xa1, 0x01, \
  0x09, 0x62, \
  0x15, 0x00, \
  0x26, 0xFF, 0x00, \
  0x95, 0x20, \
  0x75, 0x08, \
  0x81, 0x06, \
\
  0x09, 0x63, \
  0x15, 0x00, \
  0x26, 0xFF, 0x00, \
  0x95, 0x20, /*REPORT_COUNT(32)*/ \
  0x75, 0x08, /*REPORT_SIZE(8)*/ \
  0x91, 0x06, \
  0xC0 \
}

void hid_raw_ev_handler(app_usbd_class_inst_t const * p_inst,
                                app_usbd_hid_user_event_t event);

APP_USBD_HID_GENERIC_SUBCLASS_REPORT_DESC(raw_desc, RAW_REPORT_DSC());

static const app_usbd_hid_subclass_desc_t * raw_reps[] = {&raw_desc};
//extern bool m_pending_report_raw;




typedef uint8_t matrix_row_t;

void raw_hid_send(uint8_t *data, uint8_t length);
static ret_code_t idle_handle_raw(app_usbd_class_inst_t const * p_inst, uint8_t report_id);
//app_usbd_hid_generic_t * get_raw_inst(void);

void update_keycode(uint8_t kc);
extern const app_usbd_hid_generic_t m_app_hid_raw;
void usb_hid_raw_init(void);

#endif // __RAW_HID_H