// headers go here
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_timer.h"
#include "esp_log.h"

#include "pcnt_motor_encoder.h"
#include "comms.h"
#include "pi_controller.h"
#include "encoder_queue_struct.h"


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

// #define SEND_CORE 1
// #define COMPUTE_CORE 0

#define INBOUND_FLOW_CORE 1
#define OUTBOUND_FLOW_CORE 0

SemaphoreHandle_t vel_mutex;
float shared_velocity_left = 0.0f;
float shared_velocity_right = 0.0f;


// tasks to be done in main:
    // 1. initialize PCNT module (done)
    // 2. init communication (UART) (done)
    // 3. create the queue/buffer
    // 4. create the compute task and transport task (optionally pinned)
    // 5. then returns (or idle)

void app_main(void){
    
    // initialize PCNT module (Step 1)
    encoder_pcnt_init();

    // initialize uart transmission (Step 2)
    uart_init();
    
    //set log level to ERROR only
    esp_log_level_set("*", ESP_LOG_VERBOSE);

    // TODO: check if pcnt_tick_queue, target_speed_queue, and vel_mutex fail. Also can later change the stack size to 8192 from 4096 if ther is overflow

    // creating a queue for PCNT ticks
    QueueHandle_t pcnt_tick_queue;
    pcnt_tick_queue = xQueueCreate(32, sizeof(encoder_data_t));     // chose a random number (I chose to keep 32 samples at once - can change later)

    // creating a queue to receive data from UART
    QueueHandle_t target_speed_queue;
    target_speed_queue = xQueueCreate(32, sizeof(target_speed_packet_t));

    // create mutex before starting tasks (need a mutex because I am accessing )
    vel_mutex = xSemaphoreCreateMutex();

    // we are going to use this one in the future because we want to use both cores on the ESP32
    // xTaskCreatePinnedToCore(
    //     pcnt_read_task,         // task function
    //     "pcnt_read_task",       // name of task
    //     4096,                   // stack size in bytes
    //     pcnt_tick_queue,        // args
    //     5,                      // priority value (5 = medium priority)
    //     NULL,                   // task handle (kill or suspend task later)
    //     OUTBOUND_FLOW_CORE      // the core we want to use to compute       
    // );

    // xTaskCreatePinnedToCore(
    //     uart_send_task,         // task function
    //     "uart_send_task",       // name of task
    //     4096,                   // stack size in bytes
    //     pcnt_tick_queue,        // args
    //     5,                      // priority value (5 = medium priority)
    //     NULL,                   // task handle (kill or suspend task later)
    //     OUTBOUND_FLOW_CORE      // the core we want to use to send       
    // );

    xTaskCreatePinnedToCore(
        uart_receive_task,      // task function
        "uart_receive_task",    // name of task
        4096,                   // stack size in bytes
        target_speed_queue,     // args
        6,                      // priority value (5 = medium priority)
        NULL,                   // task handle (kill or suspend task later)
        INBOUND_FLOW_CORE       // the core we want to use to send       
    );

    xTaskCreatePinnedToCore(
        pi_task,                // task function
        "pi_task",              // name of task
        4096,                   // stack size in bytes
        target_speed_queue,     // args
        7,                      // priority value (5 = medium priority)
        NULL,                   // task handle (kill or suspend task later)
        INBOUND_FLOW_CORE       // the core we want to use to send       
    );

    // deleting the app_main task
    vTaskDelete(NULL);
}
