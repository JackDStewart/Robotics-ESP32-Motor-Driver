#include "driver/pulse_cnt.h"
#include "pcnt_motor_encoder.h"
#include "driver/gpio.h"
#include "esp_private/periph_ctrl.h"
#include "soc/periph_defs.h"
#include "soc/pcnt_struct.h"
#include "soc/gpio_sig_map.h"
#include "esp_rom_gpio.h"

// GPIO I decided to use (all on the right side and can change later)
#define ENC_L_A 13
#define ENC_L_B 12
#define ENC_R_A 27
#define ENC_R_B 33

/*
Notes:

I found the headers that we need to use for PCNT

*/

/*
TODO: 

1. Need to figure out which pins on the ESP32 we want to use for the encoder (done)
2. Need to implement PCNT
    a. init
    b. start

*/

// made it static so that it is only in this file and inline because it is accessing registers (units)
static inline void pcnt_start(int unit){

    // enabling the init and clock for PCNT
    periph_module_enable(PERIPH_PCNT_MODULE);
    PCNT.ctrl.clk_en = 1;

    // want to pause, reset, and then resume (using the first two registers (units) because we only have two motors )
    if (unit == 0){

        PCNT.ctrl.cnt_pause_u0 = 1;
        PCNT.ctrl.cnt_rst_u0 = 1;
        PCNT.ctrl.cnt_rst_u0 = 0;
        PCNT.ctrl.cnt_pause_u0 = 0;
    }
    else if (unit == 1){

        PCNT.ctrl.cnt_pause_u1 = 1;
        PCNT.ctrl.cnt_rst_u1 = 1;
        PCNT.ctrl.cnt_rst_u1 = 0;
        PCNT.ctrl.cnt_pause_u1 = 0;
    }
}

// static void pcnt_config