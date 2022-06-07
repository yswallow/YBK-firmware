/**
 * Copyright (c) 2016 - 2020, Nordic Semiconductor ASA
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

#include "fds.h"
#include "nrf_error.h"
#include "via_fds.h"
#include "nrf_log.h"
#include "via.h"
#include "keyboard_config.h"
#include "heatmap.h"
#include "debug_message_hid.h"
#include "neopixel_fds.h"
#include "nrf_soc.h"


#define CONFIG_FILE     (0x8010)
#define CONFIG_REC_KEY  (0x7010)
#define KBD_SETTING_FILE (0x8989)
#define KBD_SETTING_REC_KEY (0x1212)

#define KBD_SETTING_ID_HEATMAP 0x80
#define KBD_SETTING_ID_DEBUG_SETTING 0x88
#define KBD_SETTING_ID_DEBUG_RECEIVED 0x89
#define CALL_FUNCTION_AND_BREAK_IF_TRUE(func) executed=func(data,length);if(executed){break;}

static fds_record_desc_t eeprom_desc;
static fds_record_desc_t setting_desc;

uint8_t eeprom[EEPROM_SIZE];
uint8_t kbd_setting[KBD_SETTING_SIZE];

static bool fds_waiting_gc = true;

static char const * fds_evt_str[] =
{
    "FDS_EVT_INIT",
    "FDS_EVT_WRITE",
    "FDS_EVT_UPDATE",
    "FDS_EVT_DEL_RECORD",
    "FDS_EVT_DEL_FILE",
    "FDS_EVT_GC",
};

enum {
    KBD_SETTING_ID_COL_PINS = 0x04,
    KBD_SETTING_ID_ROW_PINS,
    KBD_SETTING_ID_PINS_COUNT,
    KBD_SETTING_ID_ADDITIONAL,
};


static fds_record_t const m_eeprom_record = {
    .file_id = CONFIG_FILE,
    .key = CONFIG_REC_KEY,
    .data.p_data = eeprom,
    .data.length_words = (EEPROM_SIZE*sizeof(uint8_t) + 3)/sizeof(uint32_t)
};

static fds_record_t const m_kbd_setting_record = {
    .file_id = KBD_SETTING_FILE,
    .key = KBD_SETTING_REC_KEY,
    .data.p_data = kbd_setting,
    .data.length_words = (KBD_SETTING_SIZE*sizeof(uint8_t) + 3)/sizeof(uint32_t)
};


static void wait_for_fds_ready(void)
{
    while (!m_fds_initialized)
    {
        uint32_t err_code = sd_app_evt_wait();
        APP_ERROR_CHECK(err_code);
    }
}


static void wait_for_fds_gc_complete(void) {
    while(fds_waiting_gc) {}
    fds_waiting_gc = true;
    return;
}


__INLINE uint8_t* via_fds_get_eeprom_addr(void) {
    return eeprom;
}


const char *fds_err_str(ret_code_t ret)
{
    /* Array to map FDS return values to strings. */
    static char const * err_str[] =
    {
        "FDS_ERR_OPERATION_TIMEOUT",
        "FDS_ERR_NOT_INITIALIZED",
        "FDS_ERR_UNALIGNED_ADDR",
        "FDS_ERR_INVALID_ARG",
        "FDS_ERR_NULL_ARG",
        "FDS_ERR_NO_OPEN_RECORDS",
        "FDS_ERR_NO_SPACE_IN_FLASH",
        "FDS_ERR_NO_SPACE_IN_QUEUES",
        "FDS_ERR_RECORD_TOO_LARGE",
        "FDS_ERR_NOT_FOUND",
        "FDS_ERR_NO_PAGES",
        "FDS_ERR_USER_LIMIT_REACHED",
        "FDS_ERR_CRC_CHECK_FAILED",
        "FDS_ERR_BUSY",
        "FDS_ERR_INTERNAL",
    };

    return err_str[ret - NRF_ERROR_FDS_ERR_BASE];
}

static void fds_evt_handler(fds_evt_t const * p_evt)
{
    if (p_evt->result == NRF_SUCCESS)
    {
        NRF_LOG_INFO("Event: %s received (NRF_SUCCESS)",
                      fds_evt_str[p_evt->id]);
    }
    else
    {
        NRF_LOG_INFO("Event: %s received (%s)",
                      fds_evt_str[p_evt->id],
                      fds_err_str(p_evt->result));
    }

    switch (p_evt->id)
    {
        case FDS_EVT_INIT:
            if (p_evt->result == NRF_SUCCESS)
            {
                m_fds_initialized = true;
            }
            break;

        case FDS_EVT_WRITE:
        {
            if (p_evt->result == NRF_SUCCESS)
            {
                NRF_LOG_INFO("Record ID:\t0x%04x",  p_evt->write.record_id);
                NRF_LOG_INFO("File ID:\t0x%04x",    p_evt->write.file_id);
                NRF_LOG_INFO("Record key:\t0x%04x", p_evt->write.record_key);
            }
        } break;

        case FDS_EVT_DEL_RECORD:
        {
            if (p_evt->result == NRF_SUCCESS)
            {
                NRF_LOG_INFO("Record ID:\t0x%04x",  p_evt->del.record_id);
                NRF_LOG_INFO("File ID:\t0x%04x",    p_evt->del.file_id);
                NRF_LOG_INFO("Record key:\t0x%04x", p_evt->del.record_key);
            }
            //m_delete_all.pending = false;
        } break;
        case FDS_EVT_GC:
            if (p_evt->result == NRF_SUCCESS) {
                fds_waiting_gc = false;
            } else {
                
            }
            break;
        default:
            break;
    }
}



void update_fds_entry(fds_record_desc_t* p_desc, fds_record_t* p_record) {
    ret_code_t ret;
    if(p_desc->p_record==0) {
        fds_find_token_t tok;
        memset(&tok, 0, sizeof(fds_find_token_t));
        ret = fds_record_find(p_record->file_id, p_record->key, p_desc, &tok);
        if(ret==FDS_ERR_NOT_FOUND) {
            NRF_LOG_ERROR("Record Not Found.");
        }
        APP_ERROR_CHECK(ret);
    }
    
    ret = fds_record_update(p_desc, p_record);
    if ((ret != NRF_SUCCESS) && (ret == FDS_ERR_NO_SPACE_IN_FLASH))
    {
        ret = fds_gc();
        wait_for_fds_gc_complete();
        ret = fds_record_update(p_desc, p_record);
        if(ret!=NRF_SUCCESS) {
            NRF_LOG_INFO("No space in flash, delete some records to update the config file.");
        }
    }
    else
    {
        //fds_record_delete(p_desc);
        APP_ERROR_CHECK(ret);
    }
    //fds_record_close(&desc);
}

void save_keymap(void) {
    ret_code_t ret;
    fds_find_token_t tok;
    memset(&tok, 0, sizeof(fds_find_token_t));

    if( eeprom_desc.p_record == 0 ) {
        ret=fds_record_find(CONFIG_FILE, CONFIG_REC_KEY, &eeprom_desc, &tok);
        if( ret == FDS_ERR_NOT_FOUND ) {
            NRF_LOG_ERROR("EEPROM record Not Found.");
        }
        
        APP_ERROR_CHECK(ret);
    }
    
    ret = fds_record_update(&eeprom_desc, &m_eeprom_record);
    
    if ( ret == FDS_ERR_NO_SPACE_IN_FLASH )
    {
        ret = fds_gc();
        wait_for_fds_gc_complete();
        ret = fds_record_update(&eeprom_desc, &m_eeprom_record);
        if(ret!=NRF_SUCCESS) {
            NRF_LOG_INFO("No space in flash, delete some records to update the config file.");
        }
    } else
    {
        APP_ERROR_CHECK(ret);
    }
    //fds_record_close(&desc);
}

void apply_kbd_setting(void) {
    matrix_keyboard_t* definision = my_keyboard.keyboard_definision;
    my_keyboard.kbd_cols_count = kbd_setting[0x41];
    my_keyboard.kbd_rows_count = kbd_setting[0x40];
    my_keyboard.split_keyboard.central_cols_count = kbd_setting[0x42];
    
    my_keyboard.kbd_power_led = kbd_setting[0x50+KBD_SETTING_ADDITIONAL_POWER_LED_PIN_INDEX];
    my_keyboard.kbd_power_led_enable = kbd_setting[0x50+KBD_SETTING_ADDITIONAL_POWER_LED_EN_INDEX];
    my_keyboard.default_layer = kbd_setting[0x50+KBD_SETTING_ADDITIONAL_DEFAULT_LAYER_INDEX];

    my_keyboard.neopixel_pin = kbd_setting[0x50+KBD_SETTING_ADDITIONAL_NEOPIXEL_PIN_INDEX];
    my_keyboard.neopixel_length = kbd_setting[0x50+KBD_SETTING_ADDITIONAL_NEOPIXEL_LEN_INDEX];

    definision->col_pins = &kbd_setting[0];
    definision->row_pins = &kbd_setting[0x20];
    definision->row_pins_count = kbd_setting[0x40];
    if(kbd_setting[0x42]) {
        // split keyboard
        definision->col_pins_count = kbd_setting[0x42];
    } else {
        definision->col_pins_count = kbd_setting[0x41];
    }
}


void create_fds_new_entry(fds_record_desc_t* p_desc, fds_record_t* p_record) {
    /* System config not found; write a new one. */
    NRF_LOG_INFO("Writing config file...");
    ret_code_t ret = fds_gc();
    wait_for_fds_gc_complete();
    ret = fds_record_write(p_desc, p_record);
    if ((ret != NRF_SUCCESS) && (ret == FDS_ERR_NO_SPACE_IN_FLASH))
    {
        NRF_LOG_INFO("No space in flash, delete some records to update the config file.");
    }
    else
    {
        APP_ERROR_CHECK(ret);
    }

}


void via_fds_init(void) {
    ret_code_t ret;
    fds_find_token_t  tok  = {0};
    fds_flash_record_t config = {0};
    (void) fds_register(fds_evt_handler);
    
    ret = fds_init();
    APP_ERROR_CHECK(ret);

    wait_for_fds_ready();
    
    fds_stat_t stat = {0};
    ret = fds_stat(&stat);
    APP_ERROR_CHECK(ret);
    
    NRF_LOG_INFO("Found %d valid records.", stat.valid_records);
    NRF_LOG_INFO("Found %d dirty records (ready to be garbage collected).", stat.dirty_records);

    if(stat.dirty_records) {
        ret = fds_gc();
        wait_for_fds_gc_complete();
    }
    
    ret = fds_record_find(CONFIG_FILE, CONFIG_REC_KEY, &eeprom_desc, &tok);

    if (ret == NRF_SUCCESS)
    {
        /* A config file is in flash. Let's update it. */

        /* Open the record and read its contents. */
        ret = fds_record_open(&eeprom_desc, &config);
        APP_ERROR_CHECK(ret);
       
        /* Copy the configuration from flash into m_dummy_cfg. */
        memcpy(eeprom, config.p_data, sizeof(eeprom));
        fds_record_close(&eeprom_desc);
    }
    else
    {
        /* System config not found; write a new one. */
        NRF_LOG_INFO("Writing config file...");
        ret = fds_gc();
        wait_for_fds_gc_complete();
        ret = fds_record_write(&eeprom_desc, &m_eeprom_record);
        if ((ret != NRF_SUCCESS) && (ret == FDS_ERR_NO_SPACE_IN_FLASH))
        {
            NRF_LOG_INFO("No space in flash, delete some records to update the config file.");
        }
        else
        {
            APP_ERROR_CHECK(ret);
        }
    }
    memset(&tok, 0, sizeof(fds_find_token_t));
    ret = fds_record_find(KBD_SETTING_FILE, KBD_SETTING_REC_KEY, &setting_desc, &tok);

    if (ret == NRF_SUCCESS)
    {
        /* A config file is in flash. Let's update it. */

        /* Open the record and read its contents. */
        ret = fds_record_open(&setting_desc, &config);
        APP_ERROR_CHECK(ret);
       
        /* Copy the configuration from flash into m_dummy_cfg. */
        memcpy(kbd_setting, config.p_data, sizeof(kbd_setting));
        apply_kbd_setting();
        fds_record_close(&setting_desc);
    }
    else
    {
        /* System config not found; write a new one. */

        NRF_LOG_INFO("Writing config file...");
        ret = fds_record_write(&setting_desc, &m_kbd_setting_record);
        if ((ret != NRF_SUCCESS) && (ret == FDS_ERR_NO_SPACE_IN_FLASH))
        {
            NRF_LOG_INFO("No space in flash, delete some records to update the config file.");
        }
        else
        {
            APP_ERROR_CHECK(ret);
        }
    }
    neopixel_fds_init();
}


void save_kbd_setting(void) {
    ret_code_t ret;
    if(setting_desc.p_record==0) {
        fds_find_token_t tok;
        memset(&tok, 0, sizeof(fds_find_token_t));
        ret = fds_record_find(KBD_SETTING_FILE, KBD_SETTING_REC_KEY, &setting_desc, &tok);
        if(ret==FDS_ERR_NOT_FOUND) {
            NRF_LOG_ERROR("Setting Record Not Found.");
        }
        APP_ERROR_CHECK(ret);
    }
    
    ret = fds_record_update(&setting_desc, &m_kbd_setting_record);
    if ((ret != NRF_SUCCESS) && (ret == FDS_ERR_NO_SPACE_IN_FLASH))
    {
        ret = fds_gc();
        wait_for_fds_gc_complete();
        ret = fds_record_update(&setting_desc, &m_kbd_setting_record);
        if(ret!=NRF_SUCCESS) {
            NRF_LOG_INFO("No space in flash, delete some records to update the config file.");
        }
    }
    else
    {
        APP_ERROR_CHECK(ret);
    }
    //fds_record_close(&desc);
}


// Keyboard level code can override this to handle custom messages from VIA.
// See raw_hid_receive() implementation.
// DO NOT call raw_hid_send() in the override function.
static bool kbd_setting_updated;
void raw_hid_receive_kb(uint8_t *data, uint8_t length) {
    uint8_t *command_id = &(data[0]);
    uint8_t setting_id = data[1];
    bool executed = false;

    switch(*command_id) {
    case ID_GET_KEYBOARD_VALUE:
        if( kbd_setting_updated ) {
            save_kbd_setting();
            apply_kbd_setting();
            kbd_setting_updated = false;
        }
        switch(setting_id) {
        case KBD_SETTING_ID_COL_PINS:
            memcpy(data+2, kbd_setting, kbd_setting[0x41]<KBD_SETTING_COL_PINS_MAX ? kbd_setting[0x41] : KBD_SETTING_COL_PINS_MAX );
            break;
        case KBD_SETTING_ID_ROW_PINS:
            memcpy(data+2, kbd_setting+0x20, kbd_setting[0x40]<KBD_SETTING_ROW_PINS_MAX ? kbd_setting[0x40] : KBD_SETTING_ROW_PINS_MAX );
            break;
        case KBD_SETTING_ID_PINS_COUNT:
            *(data+2) = *(kbd_setting+0x40);
            *(data+3) = *(kbd_setting+0x41);
            *(data+4) = *(kbd_setting+0x42);
            break;
        case KBD_SETTING_ID_ADDITIONAL:
            memcpy(data+2, kbd_setting+0x50, KBD_SETTING_ADDITIONAL_LEN );
            break;
        case KBD_SETTING_ID_HEATMAP:
            response_heatmap(data, length);
            break;
        case KBD_SETTING_ID_DEBUG_SETTING:
            KEYBOARD_DEBUG_HID_SET(data, length);
            break;
        case KBD_SETTING_ID_DEBUG_RECEIVED:
            KEYBOARD_DEBUG_HID_RESPONSE(data, length);
            break;

        default:
            CALL_FUNCTION_AND_BREAK_IF_TRUE(raw_hid_receive_neopixel);
            *command_id         = ID_UNHANDLED;
            break;
        }
        break;
    case ID_SET_KEYBOARD_VALUE:
        switch(setting_id) {
        case KBD_SETTING_ID_COL_PINS:
            memcpy(kbd_setting, data+2, (length-2)<KBD_SETTING_COL_PINS_MAX ? length-2 : KBD_SETTING_COL_PINS_MAX);
            kbd_setting_updated = true;
            break;
        case KBD_SETTING_ID_ROW_PINS:
            memcpy(kbd_setting+0x20, data+2, (length-2)<KBD_SETTING_ROW_PINS_MAX ? length-2 : KBD_SETTING_ROW_PINS_MAX);
            kbd_setting_updated = true;
            break;
        case KBD_SETTING_ID_PINS_COUNT:
            *(kbd_setting+0x40) = *(data+2);
            *(kbd_setting+0x41) = *(data+3);
            *(kbd_setting+0x42) = *(data+4);
            kbd_setting_updated = true;
            break;
        case KBD_SETTING_ID_ADDITIONAL:
            memcpy(kbd_setting+0x50, data+2, (length-2)<KBD_SETTING_ADDITIONAL_LEN ? length-2 : KBD_SETTING_ADDITIONAL_LEN);
            kbd_setting_updated = true;
            break;

        default:
            CALL_FUNCTION_AND_BREAK_IF_TRUE(raw_hid_receive_neopixel);
            *command_id         = ID_UNHANDLED;
            break;
        }
        break;
    default:
        CALL_FUNCTION_AND_BREAK_IF_TRUE(raw_hid_receive_neopixel);
        *command_id         = ID_UNHANDLED;
        break;
    }
    
}
