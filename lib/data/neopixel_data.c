#include "neopixel_data.h"

uint32_t neopixel_splash[NEOPIXEL_MAX_CHAINS*2];
uint32_t neopixel_dimmer[NEOPIXEL_MAX_CHAINS*7];  
void neopixel_data_init(void) {
    for(uint8_t i=0; i<NEOPIXEL_MAX_CHAINS; i++) {
        neopixel_splash[i] = NEOPIXEL_GRB(0,0,0);
    }
    
    neopixel_splash[NEOPIXEL_MAX_CHAINS] = NEOPIXEL_GRB(0,255,100);

    for(uint8_t i=NEOPIXEL_MAX_CHAINS+1; i<NEOPIXEL_MAX_CHAINS*2; i++) {
        neopixel_splash[i] = NEOPIXEL_GRB(0,0,0);
    }
    
    uint16_t mode = 0;
    uint16_t step = 255UL/((uint16_t)NEOPIXEL_MAX_CHAINS);
    uint8_t step_width = NEOPIXEL_MAX_CHAINS;
    for(uint16_t i=0; i<NEOPIXEL_MAX_CHAINS*7; i++) {
        switch(mode) {
        case 0:
            neopixel_dimmer[i] = NEOPIXEL_GRB(step*i,0,0);
            break;
        case 1:
            neopixel_dimmer[i] = NEOPIXEL_GRB(255-(step*(i-step_width)),0,0);
            break;
        case 2:
            neopixel_dimmer[i] = NEOPIXEL_GRB(0,step*(i-step_width*2),0);
            break;
        case 3:
            neopixel_dimmer[i] = NEOPIXEL_GRB(0,255-(step*(i-step_width*3)),0);
            break;
        case 4: 
            neopixel_dimmer[i] = NEOPIXEL_GRB(0,0,step*(i-step_width*4));
            break;
        case 5:
            neopixel_dimmer[i] = NEOPIXEL_GRB(0,0,255-(step*(i-step_width*5)));
            break;
        default:
            neopixel_dimmer[i] = NEOPIXEL_GRB(128,128,128);
            break;
        }
        mode = i / step_width;
    }
}
