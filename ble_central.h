void ble_central_init(void);
void ble_central_start(void);
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name);
void ble_c_evt_handler(ble_evt_t const * p_ble_evt, void * p_context);
void cache_pop_central(void);

typedef struct {
    uint8_t row;
    uint8_t col;
    uint8_t state;
} keypress_cache_t;

#define KEYPRESS_PERIPHERAL_CACHE_LEN 10
