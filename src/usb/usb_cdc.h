#ifdef NRF52840_XXAA

void usb_cdc_write(char* buffer, size_t len);
void usb_cdc_init(void);

#else

static inline void usb_cdc_write(char* buffer, size_t len) {}
static inline void usb_cdc_init(void) {}

#endif // NRF52840_XXAA
