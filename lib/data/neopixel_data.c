//#include "neopixel_data.h"
#include "neopixel.h"

uint32_t neopixel_splash[NEOPIXEL_MAX_CHAINS*2];
uint32_t neopixel_dimmer[NEOPIXEL_MAX_CHAINS*7];
#define NEOPIXEL_MAX_BRIGHTNESS 127
 
void neopixel_data_init(void) {
    for(uint8_t i=0; i<NEOPIXEL_MAX_CHAINS; i++) {
        neopixel_splash[i] = NEOPIXEL_GRB(255,255,255);
    }
    
    neopixel_splash[NEOPIXEL_MAX_CHAINS] = NEOPIXEL_GRB(255,0,0);

    for(uint8_t i=NEOPIXEL_MAX_CHAINS+1; i<NEOPIXEL_MAX_CHAINS*2; i++) {
        neopixel_splash[i] = NEOPIXEL_GRB(255,255,255);
    }
    
    uint16_t mode = 0;
    uint16_t step = ((uint16_t) NEOPIXEL_MAX_BRIGHTNESS)/((uint16_t)NEOPIXEL_MAX_CHAINS);
    uint8_t step_width = NEOPIXEL_MAX_CHAINS;
    for(uint16_t i=0; i<NEOPIXEL_MAX_CHAINS*7; i++) {
        switch(mode) {
        case 0:
            neopixel_dimmer[i] = NEOPIXEL_GRB(step*i,0,0);
            break;
        case 1:
            neopixel_dimmer[i] = NEOPIXEL_GRB(NEOPIXEL_MAX_BRIGHTNESS-(step*(i-step_width)),0,0);
            break;
        case 2:
            neopixel_dimmer[i] = NEOPIXEL_GRB(0,step*(i-step_width*2),0);
            break;
        case 3:
            neopixel_dimmer[i] = NEOPIXEL_GRB(0,NEOPIXEL_MAX_BRIGHTNESS-(step*(i-step_width*3)),0);
            break;
        case 4: 
            neopixel_dimmer[i] = NEOPIXEL_GRB(0,0,step*(i-step_width*4));
            break;
        case 5:
            neopixel_dimmer[i] = NEOPIXEL_GRB(0,0,NEOPIXEL_MAX_BRIGHTNESS-(step*(i-step_width*5)));
            break;
        case 6:
            neopixel_dimmer[i] = NEOPIXEL_GRB((step*(i-step_width*6)),0,0);
            break;
        default:
            neopixel_dimmer[i] = NEOPIXEL_GRB(NEOPIXEL_MAX_BRIGHTNESS/2,NEOPIXEL_MAX_BRIGHTNESS/2,NEOPIXEL_MAX_BRIGHTNESS/2);
            break;
        }
        mode = i / step_width;
    }
}
