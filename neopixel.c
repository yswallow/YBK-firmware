
#include "neopixel.h"
#include "nrfx_pwm.h"

#include "app_error.h"
#include "nrfx.h"

#define PWM_NEOPIXEL_INSTANCE 0

static nrfx_pwm_t m_pwm0 = NRFX_PWM_INSTANCE(PWM_NEOPIXEL_INSTANCE);
static nrf_pwm_values_common_t pwm_sequence_values[NEOPIXEL_MAX_CHAINS * NEOPIXEL_BITS];
static neopixel_handler_t m_handler = NULL;

enum {
    CLOCK = NRF_PWM_CLK_8MHz,
    TOP = 12,
    DUTY0 = 3,
    DUTY1 = 5,
};

static void pwm_handler(nrfx_pwm_evt_type_t event_type) {
    if (event_type == NRFX_PWM_EVT_FINISHED) {
        if (m_handler != NULL) {
            //m_handler();
        }
    }
}


void neopixel_init(uint8_t pin, neopixel_handler_t handler) {
    ret_code_t ret; 
    m_handler = handler;
    nrfx_pwm_config_t const config0 =
        {
            .output_pins =
                {
                    pin,                      // channel 0
                    NRFX_PWM_PIN_NOT_USED, // channel 1
                    NRFX_PWM_PIN_NOT_USED, // channel 2
                    NRFX_PWM_PIN_NOT_USED  // channel 3
                },
            .irq_priority = APP_IRQ_PRIORITY_LOWEST,
            .base_clock = CLOCK,
            .count_mode = NRF_PWM_MODE_UP,
            .top_value = TOP,
            .load_mode = NRF_PWM_LOAD_COMMON,
            .step_mode = NRF_PWM_STEP_AUTO  };

    ret = nrfx_pwm_init(&m_pwm0, &config0, pwm_handler);
    APP_ERROR_CHECK(ret);
}


void neopixel_set_black(void) {
    nrf_pwm_values_common_t *ptr = pwm_sequence_values;
    for (uint32_t i=0; i<NEOPIXEL_MAX_CHAINS; i++) {
        for (uint8_t j=0; j<NEOPIXEL_BITS; ++j) {
            *ptr = DUTY0 | 0x8000;
            ptr++;
        }
    }

    nrf_pwm_sequence_t const seq0 =
        {
            .values.p_common = pwm_sequence_values,
            .length = NEOPIXEL_BITS * NEOPIXEL_MAX_CHAINS,
            .repeats = 0,
            .end_delay = 4000
        };

    (void)nrfx_pwm_simple_playback(&m_pwm0, &seq0, 1, NRFX_PWM_FLAG_STOP);
}


void neopixel_uninit(void) {
    nrfx_pwm_uninit(&m_pwm0);
}


void neopixel_write_uint8(uint8_t *colors, uint32_t leds_count) {
    if (leds_count > NEOPIXEL_MAX_CHAINS) {
        leds_count = NEOPIXEL_MAX_CHAINS - 1;
    }

    nrf_pwm_values_common_t *ptr = pwm_sequence_values;
    for (uint32_t i=0; i<leds_count; i++) {
        for(uint8_t k=0; k<NEOPIXEL_BYTES_PER_PIXEL; k++) {
            uint8_t color = colors[NEOPIXEL_BYTES_PER_PIXEL*i+k];
            for (uint8_t j=0; j<8; ++j) {
                nrf_pwm_values_common_t value = 0;
                
                if ((color & (1<<7)) == 0) {
                    value = DUTY0;
                } else {
                    value = DUTY1;
                }

                *ptr = value | 0x8000;
                ptr++;
                color <<= 1;
            }
        }
    }

    nrf_pwm_sequence_t const seq0 =
        {
            .values.p_common = pwm_sequence_values,
            .length = NEOPIXEL_BITS * leds_count,
            .repeats = 0,
            .end_delay = 4000
        };

    (void)nrfx_pwm_simple_playback(&m_pwm0, &seq0, 1, NRFX_PWM_FLAG_STOP);
}


void neopixel_write_uint32(uint32_t *colors, uint32_t leds_count) {
    if (leds_count > NEOPIXEL_MAX_CHAINS) {
        leds_count = NEOPIXEL_MAX_CHAINS - 1;
    }

    nrf_pwm_values_common_t *ptr = pwm_sequence_values;
    for (uint32_t i=0; i<leds_count; i++) {
        uint32_t color = colors[i];
        for (uint8_t j=0; j<NEOPIXEL_BITS; ++j) {
            nrf_pwm_values_common_t value = 0;
            if ((color & (1<<23)) == 0) {
                value = DUTY0;
            } else {
                value = DUTY1;
            }
            *ptr = value | 0x8000;
            ptr++;
            color <<= 1;
        }
    }

    nrf_pwm_sequence_t const seq0 =
        {
            .values.p_common = pwm_sequence_values,
            .length = NEOPIXEL_BITS * leds_count,
            .repeats = 0,
            .end_delay = 4000
        };

    (void)nrfx_pwm_simple_playback(&m_pwm0, &seq0, 1, NRFX_PWM_FLAG_STOP);
}
