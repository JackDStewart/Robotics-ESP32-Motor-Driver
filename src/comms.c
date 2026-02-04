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

#define TX_PIN 17
#define RX_PIN 16

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
const uart_port_t uart_num = UART_NUM_0;


// here is where we will initialize UART sending
void uart_init(void){

    // allocating required internal resoruces for the UART driver  
    // Setup UART buffered IO with event queue (value from documentation, we can change later) - changed tx to 256 and rx to 1024 becauase I wanted it to 
    // be small and non-blocking for UART
    const int uart_tx_buffer_size = 256;
    const int uart_rx_buffer_size = 1024;

    // Install UART driver using an event queue here
    ESP_ERROR_CHECK(uart_driver_install(uart_num, uart_tx_buffer_size, uart_rx_buffer_size, 0, NULL, 0));

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
}


void uart_send_task(void* arg){

    // want to get the data from the PCNT queue
    QueueHandle_t q = (QueueHandle_t) arg;
    encoder_data_t trasnmit_encoder_data;

    for (;;){

        // this current way is blocking until there is a sample available - can change later
        if (xQueueReceive(q, &trasnmit_encoder_data, portMAX_DELAY) == pdTRUE){

            // build packet
            char packet_buf[sizeof(uart_header_t) + sizeof(encoder_data_t)];
            int err = build_data_packet(packet_buf, trasnmit_encoder_data);

            // Write data to UART
            uart_write_bytes(uart_num, (const char*)&trasnmit_encoder_data, sizeof(trasnmit_encoder_data));
            // const char* test_msg = "Hello from UART\n";
            // size_t msg_length = strlen(test_msg);
            // uart_write_bytes(UART_NUM_0, test_msg, msg_length);
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

void build__data_packet(void *buffer, encoder_data_t encoder_data) {
    static seq_num = 0; // value persists throughout calls
    //build header
    uart_header_t *header = (uart_header_t *)buffer;
    header->seq = seq_num++;
    header->type_flag = 1;
    header->length = sizeof(encoder_data_t);

    //build payload
    memcpy(buffer + sizeof(uart_header_t), &encoder_data, sizeof(encoder_data_t));

    // calculate checksum
    header->checksum = calculate_checksum(buffer, (sizeof(uart_header_t) + sizeof(encoder_data_t)));
}

uint16_t calculate_checksum(const void *data, size_t len) { // stolen from schmitt if we have any issues 
    const uint8_t *bytes = (const uint8_t *)data;
    uint32_t sum = 0;
    for (size_t i = 0; i < len; i++) {
        sum += bytes[i];
    }
    // Fold 32-bit sum into 16 bits
    while (sum >> 16) {
        sum = (sum & 0xFFFF) + (sum >> 16);
    }
    return (uint16_t)~sum;
}