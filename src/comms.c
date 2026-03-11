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
#include "cobs.h"

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
        if (xQueueReceive(q, &trasnmit_encoder_data, portMAX_DELAY) == pdTRUE) {

            char packet_buf[sizeof(data_packet_t) + 1]; //+1 for cobs
            build_data_packet(packet_buf, trasnmit_encoder_data);

            // Write data to UART (pointer decays to byte pointer automatically)
            uart_write_bytes(uart_num, packet_buf, sizeof(data_packet_t));
        }
    }
}

void build_data_packet(void *buffer, encoder_data_t encoder_data) {
    
    static uint8_t seq_num = 0; // value persists throughout calls
    //build packets
    data_packet_t packet = {0};
    packet.seq = seq_num++;          // wraps naturally at 255
    memcpy(&packet.encoder_data, &encoder_data, sizeof(encoder_data_t));

    /* checksum covers the entire packet with the checksum field set to 0 */
    packet.checksum = 0;
    packet.checksum = calculate_checksum(&packet, sizeof(data_packet_t));

    cobs_encode(buffer, sizeof(buffer), &packet, sizeof(data_packet_t));
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