#include <stdint.h>

#define CLK_PIN 29
#define DIO_PIN 2
#define CS_PIN 13


void mouse_move_central(char* p_data) {
    int16_t delta_x = p_data[1]<<8 | p_data[2];
    int16_t delta_y = p_data[3]<<8 | p_data[4];
    
    int8_t wheel = 0; //delta_y >> 2;
    hid_functions.mouse_move(delta_x, delta_y, wheel);
}

void poll_trackball(void) {
    uint8_t buf[10];
    char cdc_buf[64];
    static int8_t wheel_drop = 0;

    read_pmw3610_regs(0x12, 4, buf);
            
    uint16_t delta_x,delta_y;
    int8_t wheel;
    delta_x = ((buf[3]&0xF0)<<4) | buf[1];
    delta_y = ((buf[3]&0x0F)<<8) | buf[2];
    
    if(delta_x & 0x0800 ) {
        delta_x |= 0xF000;
    }

    if(delta_y & 0x0800 ) {
        delta_y |= 0xF000;
    }
    
    wheel = (delta_y + wheel_drop)>>3;
    wheel_drop = (delta_y + wheel_drop) - (wheel<<3);

    if( ( buf[0] & 0x80 ) ) {
#ifdef KEYBOARD_PERIPH
        send_mouse_periph(delta_x, delta_y);
#else
        hid_functions.mouse_move(delta_x, 0, wheel);
#endif // KEYBOARD_PERIPH
    } else if( (buf[0]&0x0F) != 0x09 ) {
        nrf_delay_us(10);
        usb_cdc_write("resetting....\r\n", 18);
        write_pmw3610_reg(0x3a, 0x5a); // soft reset
        nrf_delay_ms(4);
        write_pmw3610_reg(0x41, 0xba);
        nrf_delay_ms(1);
        write_pmw3610_reg(0x32, 0x10);
        write_pmw3610_reg(0x10, 0x01);
        nrf_delay_ms(1);

        for(uint8_t i=0;i<4;i++) {
            read_pmw3610_regs(0x0c+i, 1, buf);
            size_t size = sprintf(cdc_buf, "CRC#%d: 0x%02x\r\n", i, buf[0]);
            usb_cdc_write(cdc_buf, size);
        }

        write_pmw3610_reg(0x3a, 0x5a); // soft reset
    }

    size_t size = sprintf(cdc_buf, "MOTION: %x, DX: %d, DY: %d\r\n", buf[0], (int16_t)delta_x, (int16_t)delta_y);
    usb_cdc_write(cdc_buf, size);
}