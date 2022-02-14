#include <string.h>
#include <stdint.h>
#include "neopixel.h"
#include "neopixel_fds.h"

uint8_t neopixel_time_data[NEOPIXEL_FRAME_BYTES];
static const uint8_t segData[10] = {0x77,0x24,0x5D,0x6D,0x2E,0x6B,0x7B,0x27,0x7F,0x6F};

void setSegment(uint8_t c0, uint8_t c1, uint8_t r0, uint8_t r1, uint32_t color) {
    for(uint8_t c=c0;c<c1;c++) {
        for(uint8_t r=r0;r<r1;r++) {
            uint8_t i = neopixel_array[neopixel_array_config.cols*r+c];
            if( i != NEOPIXEL_PLACE_INVALID ) {
                neopixel_time_data[i*NEOPIXEL_BYTES_PER_PIXEL] = (color>>16) & 0xFF;  // G
                neopixel_time_data[i*NEOPIXEL_BYTES_PER_PIXEL+1] = (color>>8) & 0xFF; // R
                neopixel_time_data[i*NEOPIXEL_BYTES_PER_PIXEL+2] = (color) & 0xFF;    // B
            }
        }
    }
}

void setSegments(uint8_t number, uint8_t width, uint8_t shift, uint32_t color) {
    const uint8_t seg = segData[number];
    const uint8_t rows = neopixel_array_config.rows;
    const uint8_t cols = neopixel_array_config.cols;
    
    const uint8_t col_width = width/3;
    const uint8_t col_amari = width - col_width*3;
    const uint8_t c0_b = shift;
    const uint8_t c1_b = c0_b + col_width;
    const uint8_t c2_b = c1_b + col_width + col_amari;
    const uint8_t c2_e = shift+width;

    const uint8_t row_height = rows/5;
    const uint8_t row_amari = rows - row_height*5;
    const uint8_t r0_b = 0;
    const uint8_t r1_b = row_height;
    const uint8_t r2_b = r1_b + row_height + (row_amari>1 ? 1 : 0);
    const uint8_t r3_b = r2_b + row_height + (row_amari>2 ? 1 : 0);
    const uint8_t r4_b = r3_b + row_height + (row_amari>0 ? 1 : 0);
    const uint8_t r4_e = rows;

    for(uint8_t i=0;i<8;i++) {
        if( seg & (1<<i) ) {
            switch(i){
            case 0:
                setSegment(c0_b,c2_e,r0_b,r1_b,color);
                break;
            case 1:
                setSegment(c0_b,c1_b,r0_b,r3_b,color);
                break;
            case 2:
                setSegment(c2_b,c2_e,r0_b,r3_b,color);
                break;
            case 3:
                setSegment(c0_b,c2_e,r2_b,r3_b,color);
                break;
            case 4:
                setSegment(c0_b,c1_b,r2_b,r4_e,color);
                break;
            case 5:
                setSegment(c2_b,c2_e,r2_b,r4_e,color);
                break;
            case 6:
                setSegment(c0_b,c2_e,r4_b,r4_e,color);
                break;
            }
        }
    }
}


void setNumber(uint8_t number, uint32_t hier_color, uint32_t lower_color, uint32_t bg_color) {
    const uint8_t hier = (number / 10) % 10;
    const uint8_t lower = number % 10;
    const uint8_t cols = neopixel_array_config.cols;
    
    for(uint8_t i=0; i<NEOPIXEL_MAX_CHAINS; i++) {
        neopixel_time_data[NEOPIXEL_BYTES_PER_PIXEL*i] = (bg_color>>16) & 0xFF;
        neopixel_time_data[NEOPIXEL_BYTES_PER_PIXEL*i+1] = (bg_color>>8) & 0xFF;
        neopixel_time_data[NEOPIXEL_BYTES_PER_PIXEL*i+2] = (bg_color) & 0xFF;
    }
    setSegments(hier, cols/2, 0, hier_color);
    setSegments(lower,cols/2, cols/2, lower_color);
}
