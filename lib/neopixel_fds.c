#include "neopixel_fds.h"
#include "via.h"
#include "string.h"
#include "fds.h"
#include "nrf_error.h"

static fds_record_desc_t neopixel_desc[NEOPIXEL_USER_DEFINED_COUNT];
static fds_record_desc_t neopixel_conf_desc;

neopixel_user_defined_config_t neopixel_user_defined_config[NEOPIXEL_USER_DEFINED_COUNT];
uint8_t neopixel_user_defined[NEOPIXEL_USER_DEFINED_COUNT][NEOPIXEL_PATTERN_BYTES];

static fds_record_t fds_neopixel_records[NEOPIXEL_USER_DEFINED_COUNT];
static fds_record_t fds_neopixel_record_base = {
    .file_id = FDS_NEOPIXEL_FILE_ID,
    .key = FDS_NEOPIXEL_REC_KEY,
    .data.p_data = neopixel_user_defined,
    .data.length_words = (sizeof(uint8_t)*NEOPIXEL_PATTERN_BYTES + 3)/sizeof(uint32_t)
};

static fds_record_t fds_neopixel_conf_record = {
    .file_id = FDS_NEOPIXEL_CONF_FILE_ID,
    .key = FDS_NEOPIXEL_CONF_REC_KEY,
    .data.p_data = neopixel_user_defined_config,
    .data.length_words = (sizeof(neopixel_user_defined_config_t)*sizeof(neopixel_user_defined_config) + 3)/sizeof(uint32_t)
};


static uint32_t neopixel_user_defined_pattern_updated = 0;
static bool neopixel_user_defined_config_updated = false;

void neopixel_fds_init(void){
    ret_code_t ret;
    fds_find_token_t  tok  = {0};
    fds_flash_record_t config = {0};
    
    for(uint8_t i=0;i<NEOPIXEL_USER_DEFINED_COUNT;i++) {
        memcpy(&fds_neopixel_records[i], &fds_neopixel_record_base, sizeof(fds_record_t));
        fds_neopixel_records[i].key = FDS_NEOPIXEL_REC_KEY+i;
        fds_neopixel_records[i].data.p_data = neopixel_user_defined[i];
    }

    // not to conf.interval_ticks==0
    memset(neopixel_user_defined_config, 0x02, sizeof(neopixel_user_defined_config_t)*sizeof(neopixel_user_defined_config));

    ret = fds_record_find(FDS_NEOPIXEL_CONF_FILE_ID, FDS_NEOPIXEL_CONF_REC_KEY, &neopixel_conf_desc, &tok);
    if (ret == NRF_SUCCESS)
    {
        /* A config file is in flash. Let's update it. */
        ret = fds_record_open(&neopixel_conf_desc, &config);
        APP_ERROR_CHECK(ret);
       
        memcpy(neopixel_user_defined_config, config.p_data, sizeof(neopixel_user_defined_config_t)*sizeof(neopixel_user_defined_config));
    }
    else
    {
        create_fds_new_entry(&neopixel_conf_desc, &fds_neopixel_conf_record);
    }
    
    for(uint8_t i=0;i<NEOPIXEL_USER_DEFINED_COUNT;i++) {
        memset(&tok, 0, sizeof(fds_find_token_t));
        ret = fds_record_find(FDS_NEOPIXEL_FILE_ID, FDS_NEOPIXEL_REC_KEY+i, &neopixel_desc[i], &tok);

        if (ret == NRF_SUCCESS)
        {
            /* A config file is in flash. Let's update it. */
            ret = fds_record_open(&neopixel_desc[i], &config);
            APP_ERROR_CHECK(ret);
       
            memcpy(neopixel_user_defined, config.p_data, sizeof(uint8_t)*sizeof(neopixel_user_defined));
        }
        else
        {
           create_fds_new_entry(&neopixel_desc[i], &fds_neopixel_records[i]);
        }
    }
}


void save_neopixel(void) {
    if(neopixel_user_defined_config_updated) {
        update_fds_entry(&neopixel_conf_desc, &fds_neopixel_conf_record);
        neopixel_user_defined_config_updated = false;
    }

    for(uint8_t i=0;i<NEOPIXEL_USER_DEFINED_COUNT;i++) {
        if(neopixel_user_defined_pattern_updated & (1<<i)) {
            update_fds_entry(&neopixel_desc[i], &fds_neopixel_records[i]);
            neopixel_user_defined_pattern_updated &= ~(1<<i);
        }
    }
}


void raw_hid_receive_neopixel(uint8_t *data, uint8_t length) {
    neopixel_user_defined_config_t *conf;
    switch( *data ) {
    case id_get_keyboard_value:
        switch( *(data+1) ) {
        case KBD_NEOPIXEL:
            save_neopixel();
            *(data+2) = NEOPIXEL_MAX_CHAINS;
            break;
        }
        break;
    case id_set_keyboard_value:
        switch( *(data+1) ) {
        case KBD_NEOPIXEL:
            memcpy(
                neopixel_user_defined[*(data+2)]
                +(NEOPIXEL_FRAME_BYTES*(*(data+3)))
                +(NEOPIXEL_BYTES_PER_PIXEL*(*(data+4))),
                data+5,
                length-5
                );
            *data = id_unhandled;
            neopixel_user_defined_pattern_updated |= 1<<(*(data+2));
            break;
        case KBD_NEOPIXEL_CONF:
            conf = &neopixel_user_defined_config[*(data+2)];
            conf->frame_count = *(data+3);
            conf->interval_ticks = *(data+4);
            *data = id_unhandled;
            neopixel_user_defined_config_updated = true;
            break;
        }
        break;
    default:
        break;
    }
}