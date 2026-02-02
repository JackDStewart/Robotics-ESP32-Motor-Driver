// here is the C file we can use for communication

// Here is my idea:
// We connect the ESP32 to the Jetson GPU Serially (using UART)
// In this module we will do all the sending of the data and use Core 0 (I was thinking that we use Core 1 for data crunching)


// In the documentation for UART Transmission:
// Events defined in uart_event_type_t can be reported to a user application using the FreeRTOS queue functionality.

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "comms.h"
#include "driver/uart.h"
#include "encoder_queue_struct.h"

// if we want to enable interrupts:
// const uart_port_t uart_num = UART_NUM_2;
// // Configure a UART interrupt threshold and timeout
// uart_intr_config_t uart_intr = {
//     .intr_enable_mask = UART_INTR_RXFIFO_FULL | UART_INTR_RXFIFO_TOUT,
//     .rxfifo_full_thresh = 100,
//     .rx_timeout_thresh = 10,
// };
// ESP_ERROR_CHECK(uart_intr_config(uart_num, &uart_intr));

// // Enable UART RX FIFO full threshold and timeout interrupts
// ESP_ERROR_CHECK(uart_enable_rx_intr(uart_num));

// global variables 
const uart_port_t uart_num = UART_NUM_2;


// here is where we will initialize UART sending
void uart_init(void){

    // allocating required internal resoruces for the UART driver  
    // Setup UART buffered IO with event queue (value from documentation, we can change later)
    const int uart_buffer_size = (1024 * 2);

    // Install UART driver using an event queue here
    ESP_ERROR_CHECK(uart_driver_install(uart_num, uart_buffer_size, uart_buffer_size, 0, NULL, 0));


    // configuring UART comms parameters
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,  // disabled it because we are sending over USB
        .rx_flow_ctrl_thresh = 122,             // because it is disabled, this is not looked at
    };
    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));

    // Set UART pins(TX: IO4, RX: IO5, RTS: IO18, CTS: IO19) (these were the given ones but I think we need to change this - come back to this)
    ESP_ERROR_CHECK(uart_set_pin(uart_num, 17, 16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}


void uart_send_task(void* arg){

    // want to get the data from the PCNT queue
    QueueHandle_t q = (QueueHandle_t) arg;
    encoder_data trasnmit_encoder_data;

    for (;;){

        // this current way is blocking until there is a sample available - can change later
        if (xQueueReceive(q, &trasnmit_encoder_data, portMAX_DELAY) == pdTRUE){

            // Write data to UART
            uart_write_bytes(uart_num, (const char*)&trasnmit_encoder_data, sizeof(trasnmit_encoder_data));
        }
    }
}


// got rid of this in main:
    // // Wait for packet to be sent
    // ESP_ERROR_CHECK(uart_wait_tx_done(uart_num, 100)); // wait timeout is 100 RTOS ticks (TickType_t)

    // // Read data from UART.
    // const uart_port_t uart_num = UART_NUM_2;
    // uint8_t data[128];
    // int length = 0;
    // ESP_ERROR_CHECK(uart_get_buffered_data_len(uart_num, (size_t*)&length));
    // length = uart_read_bytes(uart_num, data, length, 100);

    // can use this if we want a break signal
    // // Write data to UART, end with a break signal.
    // uart_write_bytes_with_break(uart_num, "test break\n",strlen("test break\n"), 100);
