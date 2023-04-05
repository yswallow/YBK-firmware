/**
 * Copyright (c) 2017 - 2020, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 *
 * @defgroup usbd_ble_uart_example main.c
 * @{
 * @ingroup  usbd_ble_uart_example
 * @brief    USBD CDC ACM over BLE application main file.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service
 * and USBD CDC ACM library.
 * This application uses the @ref srvlib_conn_params module.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrf_drv_clock.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_drv_power.h"
#include "nrf_pwr_mgmt.h"

#include "app_error.h"
#include "app_util.h"

#ifdef NRF52840_XXAA
#include "app_timer.h"
#include "app_util_platform.h"
#include "nrf_drv_usbd.h"
#include "app_usbd_core.h"
#include "app_usbd.h"
#include "app_usbd_string_desc.h"
#include "app_usbd_serial_num.h"
#include "app_usbd_hid.h"
#include "app_usbd_hid_generic.h"
#include "app_usbd_hid_kbd.h"
#include "app_usbd_hid_kbd_desc.h"
#include "usb_hiddevice.h"
#else
#include "dummy_hid.h"
#endif // NRF52840_XXAA

#include "keyboard_generic.h"
#include "ble_setting.h"
#include "via_fds.h"
#include "debug_message_hid.h"

#ifndef KEYBOARD_PERIPH
#include "ble_hiddevice.h"
#endif

#ifdef KEYBOARD_CENTRAL
#include "ble_central.h"
#include "ble_nus.h"
#elif KEYBOARD_PERIPH
#include "ble_peripheral.h"
#include "ble_nus.h"
#endif

#ifdef ENABLE_USB_CDC_ACM
#include "app_usbd_cdc_acm.h"
#define LED_CDC_ACM_CONN (BSP_BOARD_LED_2)
#define LED_CDC_ACM_RX   (BSP_BOARD_LED_3)

#define LED_BLINK_INTERVAL 800
#define ENDLINE_STRING "\r\n"
#endif

#define NRF_LOG_DEBUG_FLUSH(x) do {\
    __DMB();\
    for(uint8_t i=0;i<10;i++) {\
        NRF_LOG_PROCESS();\
    }\
    NRF_LOG_DEBUG(x);\
    NRF_LOG_PROCESS();\
} while(0)

APP_TIMER_DEF(m_keyboard_job_timer);
#ifdef NRF52840_XXAA
static bool m_usb_connected = false;
#endif

volatile bool keyboard_running = false;

/** @brief Function for initializing the timer module. */
static void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}


/** @brief Function for initializing the nrf_log module. */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/** @brief Function for placing the application in low power state while waiting for events. */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}


/**
 * @brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
/*
void idle_state_handle(void)
{
    app_sched_execute();
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}
*/

// USB CODE START

#ifdef NRF52840_XXAA
#ifdef ENABLE_USB_CDC_ACM

/** @brief User event handler @ref app_usbd_cdc_acm_user_ev_handler_t */
static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                    app_usbd_cdc_acm_user_event_t event)
{
    app_usbd_cdc_acm_t const * p_cdc_acm = app_usbd_cdc_acm_class_get(p_inst);

    switch (event)
    {
        case APP_USBD_CDC_ACM_USER_EVT_PORT_OPEN:
        {
            /*Set up the first transfer*/
            ret_code_t ret = app_usbd_cdc_acm_read(&m_app_cdc_acm,
                                                   m_cdc_data_array,
                                                   1);
            UNUSED_VARIABLE(ret);
            ret = app_timer_stop(m_blink_cdc);
            APP_ERROR_CHECK(ret);
            bsp_board_led_on(LED_CDC_ACM_CONN);
            NRF_LOG_INFO("CDC ACM port opened");
            break;
        }

        case APP_USBD_CDC_ACM_USER_EVT_PORT_CLOSE:
            NRF_LOG_INFO("CDC ACM port closed");
            if (m_usb_connected)
            {
                ret_code_t ret = app_timer_start(m_blink_cdc,
                                                 APP_TIMER_TICKS(LED_BLINK_INTERVAL),
                                                 (void *) LED_CDC_ACM_CONN);
                APP_ERROR_CHECK(ret);
            }
            break;

        case APP_USBD_CDC_ACM_USER_EVT_TX_DONE:
            break;

        case APP_USBD_CDC_ACM_USER_EVT_RX_DONE:
        {
            ret_code_t ret;
            static uint8_t index = 0;
            index++;

            do
            {
                if ((m_cdc_data_array[index - 1] == '\n') ||
                    (m_cdc_data_array[index - 1] == '\r') ||
                    (index >= (m_ble_nus_max_data_len)))
                {
                    if (index > 1)
                    {
                        bsp_board_led_invert(LED_CDC_ACM_RX);
                        NRF_LOG_DEBUG("Ready to send data over BLE NUS");
                        NRF_LOG_HEXDUMP_DEBUG(m_cdc_data_array, index);

                        do
                        {
                            uint16_t length = (uint16_t)index;
                            if (length + sizeof(ENDLINE_STRING) < BLE_NUS_MAX_DATA_LEN)
                            {
                                memcpy(m_cdc_data_array + length, ENDLINE_STRING, sizeof(ENDLINE_STRING));
                                length += sizeof(ENDLINE_STRING);
                            }

                            ret = ble_nus_data_send(&m_nus,
                                                    (uint8_t *) m_cdc_data_array,
                                                    &length,
                                                    m_conn_handle);

                            if (ret == NRF_ERROR_NOT_FOUND)
                            {
                                NRF_LOG_INFO("BLE NUS unavailable, data received: %s", m_cdc_data_array);
                                break;
                            }

                            if (ret == NRF_ERROR_RESOURCES)
                            {
                                NRF_LOG_ERROR("BLE NUS Too many notifications queued.");
                                break;
                            }

                            if ((ret != NRF_ERROR_INVALID_STATE) && (ret != NRF_ERROR_BUSY))
                            {
                                APP_ERROR_CHECK(ret);
                            }
                        }
                        while (ret == NRF_ERROR_BUSY);
                    }

                    index = 0;
                }

                /*Get amount of data transferred*/
                size_t size = app_usbd_cdc_acm_rx_size(p_cdc_acm);
                NRF_LOG_DEBUG("RX: size: %lu char: %c", size, m_cdc_data_array[index - 1]);

                /* Fetch data until internal buffer is empty */
                ret = app_usbd_cdc_acm_read(&m_app_cdc_acm,
                                            &m_cdc_data_array[index],
                                            1);
                if (ret == NRF_SUCCESS)
                {
                    index++;
                }
            }
            while (ret == NRF_SUCCESS);

            break;
        }
        default:
            break;
    }
}
#endif // ENABLE_USB_CDC_ACM

static void usbd_user_ev_handler(app_usbd_event_type_t event)
{
    switch (event)
    {
        case APP_USBD_EVT_DRV_SUSPEND:
            break;

        case APP_USBD_EVT_DRV_RESUME:
            break;

        case APP_USBD_EVT_STARTED:
            break;

        case APP_USBD_EVT_STOPPED:
            app_usbd_disable();
            break;

        case APP_USBD_EVT_POWER_DETECTED:
            NRF_LOG_INFO("USB power detected");

            if (!nrf_drv_usbd_is_enabled())
            {
                app_usbd_enable();
            }
            break;

        case APP_USBD_EVT_POWER_REMOVED:
        {
            NRF_LOG_INFO("USB power removed");
#ifdef ENABLE_USB_CDC_ACM
            ret_code_t err_code = app_timer_stop(m_blink_cdc);
            APP_ERROR_CHECK(err_code);
            bsp_board_led_off(LED_CDC_ACM_CONN);
#endif //ENABLE_USB_CDC_ACM
            m_usb_connected = false;
            app_usbd_stop();
#ifndef KEYBOARD_PERIPH
            hid_functions = ble_hid_functions;
#endif
        }
            break;

        case APP_USBD_EVT_POWER_READY:
        {
            NRF_LOG_INFO("USB ready");
#ifdef ENABLE_USB_CDC_ACM
            ret_code_t err_code = app_timer_start(m_blink_cdc,
                                                  APP_TIMER_TICKS(LED_BLINK_INTERVAL),
                                                  (void *) LED_CDC_ACM_CONN);
            APP_ERROR_CHECK(err_code);
#endif
            m_usb_connected = true;
            app_usbd_start();
#ifndef FORCE_BLE
            hid_functions = usb_hid_functions;
#endif      
        }
            break;

        default:
            break;
    }
}
#endif // NRF52840_XXAA
// USB CODE END

/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}

void keyboard_tick_init(void) {
    ret_code_t err_code = app_timer_create(&m_keyboard_job_timer, APP_TIMER_MODE_REPEATED, kbd_tick_handler);
    APP_ERROR_CHECK(err_code);
}

void keyboard_tick_start(void) {
    ret_code_t err_code = app_timer_start(m_keyboard_job_timer, APP_TIMER_TICKS(5), NULL);
    APP_ERROR_CHECK(err_code);
}

void keyboard_tick_stop(void) {
    ret_code_t err_code = app_timer_stop(m_keyboard_job_timer);
    APP_ERROR_CHECK(err_code);
}

/** @brief Application main function. */
int main(void)
{
    ret_code_t ret;

    // Initialize.
    log_init();
    NRF_LOG_DEBUG_FLUSH("LOG INIT");

    via_fds_init();
    NRF_LOG_DEBUG_FLUSH("FDS INIT");

    timers_init();
    NRF_LOG_DEBUG_FLUSH("TIMER INIT");
    //buttons_leds_init();

#ifdef NRF52840_XXAA
    app_usbd_serial_num_generate();

    // if power supply from VDDH and output voltage is not configured (is reset state),
    // configure output voltage 3V3
    if( (NRF_UICR->REGOUT0 & UICR_REGOUT0_VOUT_Msk)==UICR_REGOUT0_VOUT_DEFAULT && (NRF_POWER->MAINREGSTATUS & 1) ) {
        NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen;
        while( NRF_NVMC->READY == NVMC_READY_READY_Busy ) {
        }

        NRF_UICR->REGOUT0 = UICR_REGOUT0_VOUT_3V3 << UICR_REGOUT0_VOUT_Pos;
        __DMB();

        while( NRF_NVMC->READY == NVMC_READY_READY_Busy ) {
        }
        NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren;

        // Must reset to enable change.
        NVIC_SystemReset();
    }
#endif
    ret = nrf_drv_clock_init();
    APP_ERROR_CHECK(ret);
    NRF_LOG_DEBUG_FLUSH("CLOCK INIT");

    NRF_LOG_INFO("USBD BLE UART Keyboard started.");
    NRF_LOG_DEBUG_FLUSH(DEVICE_NAME);

#ifdef NRF52840_XXAA
	static const app_usbd_config_t usbd_config = {
        .ev_state_proc = usbd_user_ev_handler
    };
    ret = app_usbd_init(&usbd_config);
    APP_ERROR_CHECK(ret);

    usb_hid_init();
#endif
/*
    app_usbd_class_inst_t const * class_cdc_acm = app_usbd_cdc_acm_class_inst_get(&m_app_cdc_acm);
    ret = app_usbd_class_append(class_cdc_acm);
    APP_ERROR_CHECK(ret);
*/
    ble_device_init();
    NRF_LOG_DEBUG_FLUSH("BLE INIT");
    power_management_init();
#ifdef KEYBOARD_CENTRAL
    ble_central_init();
#endif
    // Start execution.

#if !defined( KEYBOARD_PERIPH )
    hid_functions = ble_hid_functions;
#elif defined( NRF52840_XXAA )
    hid_functions = usb_hid_functions;
#else
	hid_functions = dummy_hid_functions;
#endif

#ifdef NRF52840_XXAA
    ret = app_usbd_power_events_enable();
    APP_ERROR_CHECK(ret);
#endif
    keyboard_init(my_keyboard);
    keyboard_tick_init();
    KEYBOARD_DEBUG_HID_INIT();
    NRF_LOG_DEBUG_FLUSH("KEYBOARD INIT");

#ifdef KEYBOARD_CENTRAL
    ble_central_start();
#else
#ifdef KEYBOARD_PERIPH
    advertising_start(false, BLE_ADV_MODE_FAST); // in central, advertising_start() is called after peripheral connected
#else
    advertising_start(true, BLE_ADV_MODE_FAST);
#endif  
#endif
    NRF_LOG_DEBUG_FLUSH("Enter main loop");

    // Enter main loop.
    for (;;)
    {
#ifdef NRF52840_XXAA
        while (app_usbd_event_queue_process())
        {
            /* Nothing to do */
        }
#endif

#ifdef USE_INTERRUPT
        if(any_keypress) {
#else
        if(1) {
#endif
            if(! keyboard_running) {
                sd_clock_hfclk_request();
                (my_keyboard.init_method)(my_keyboard.keyboard_type,my_keyboard.keyboard_definision);
                keyboard_tick_start();
                keyboard_running = true;
            }

            keyboard_scan(my_keyboard);
        } else {
            if(keyboard_running) {
                keyboard_tick_stop();
                keyboard_sleep_prepare();
                sd_nvic_EnableIRQ(GPIOTE_IRQn);
                sd_clock_hfclk_release();
                keyboard_running = false;
            }
        }
        if(tick) {
            keyboard_tick();
            if( (++keypress_ticks>MATRIX_SCAN_TIMEOUT_TICKS)
                && no_key_pressed() ) {
                any_keypress = false;
            }
            tick = false;
        }

#ifdef KEYBOARD_CENTRAL
        cache_pop_central();
#elif KEYBOARD_PERIPH
        cache_pop_peripheral();
#endif
        NRF_LOG_PROCESS();
        power_manage();
    }
}

/**
 * @}
 */
