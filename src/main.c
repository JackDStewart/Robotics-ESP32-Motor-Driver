// headers go here
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_timer.h"
#include "pcnt_motor_encoder.h"
#include "comms.h"

// in the future we can use xTaskCreatePinnedToCore() creates a task with a particular core affinity. The task's memory is dynamically allocated.
// More info:
// 0, which pins the created task to Core 0
// 1, which pins the created task to Core 1
// tskNO_AFFINITY, which allows the task to be run on both cores
// The IDF FreeRTOS scheduler implements a Best Effort Round Robin time slicing for ready-state tasks of the same priority

// // For Tick Interrupts:
// Core 0 executes all of the tick interrupt responsibilities listed above
// Core 1 only checks for time slicing and executes the application tick hook

// Important Note (about tick interrupts): 
// Core 0 is solely responsible for keeping time in IDF FreeRTOS. Therefore, anything that prevents Core 0 from incrementing the tick count, such as 
// suspending the scheduler on Core 0, will cause the entire scheduler's timekeeping to lag behind.

#define SEND_CORE 1

// tasks to be done in main:
    // 1. initialize PCNT module (done)
    // 2. init communication (UART)
    // 3. create the queue/buffer
    // 4. create the compute task and transport task (optionally pinned)
    // 5. then returns (or idle)

void app_main(void){
   
    // initialize PCNT module (Step 1)
    encoder_pcnt_init();

    // initialize uart transmission (Step 2)
    uart_init();



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

    // we are going to use this one in the future because we want to use both cores on the ESP32
    xTaskCreatePinnedToCore(
        pcnt_read_task,     // task function
        "pcnt_read_task",   // name of task
        4096,               // stack size in bytes
        NULL,               // args
        5,                  // priority value (5 = medium priority)
        NULL,               // task handle (kill or suspend task later)
        SEND_CORE           // the core we want to use to send       
    );

}
