// headers go here
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_timer.h"
#include "pcnt_motor_encoder.h"


void app_main(void){
    encoder_pcnt_init();

    // If you want counts starting from 0 at boot, this is already done in init.
    // If you want periodic "windowed" counts, you could clear every loop instead.

    xTaskCreate(
        pcnt_read_task,   // task function
        "pcnt_read_task",   // name of task
        4096,      // stack size in bytes
        NULL,      // args
        5,         // priority value (5 = medium priority)
        NULL       // task handle (kill or suspend task later)
    );

}
