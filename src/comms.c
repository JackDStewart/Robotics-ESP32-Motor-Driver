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
#include "driver/uart.h"

#include "comms.h"
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

// ------------------- FUNCTIONALITY FOR SENDING DATA OVER UART --------------------------------------------

void uart_send_task(void* arg){

    // want to get the data from the PCNT queue
    QueueHandle_t q = (QueueHandle_t) arg;
    encoder_data_t trasnmit_encoder_data;

    for (;;){

        // this current way is blocking until there is a sample available - can change later
        if (xQueueReceive(q, &trasnmit_encoder_data, portMAX_DELAY) == pdTRUE) {

            // char packet_buf[sizeof(data_packet_t) + 1]; //+1 for cobs - changed this

            uint8_t packet_buf[sizeof(data_packet_t) + 2]; // +2 for COBS overhead + zero terminator
            size_t encoded_len = 0;

            build_data_packet(packet_buf,sizeof(packet_buf) - 1, trasnmit_encoder_data, &encoded_len);
            packet_buf[encoded_len] = 0x00; // manually append zero terminator


            // Write data to UART (pointer decays to byte pointer automatically)
            uart_write_bytes(uart_num, packet_buf, encoded_len + 1);
        }
    }
}

void build_data_packet(void *buffer, size_t buf_size, encoder_data_t encoder_data, size_t *out_len) {
    
    static uint8_t seq_num = 0; // value persists throughout calls
    //build packets
    data_packet_t packet = {0};
    packet.seq = seq_num++;          // wraps naturally at 255
    memcpy(&packet.encoder_data, &encoder_data, sizeof(encoder_data_t));

    /* checksum covers the entire packet with the checksum field set to 0 */
    packet.checksum = 0;
    packet.checksum = calculate_checksum(&packet, sizeof(data_packet_t));

    cobs_encode_result result = cobs_encode(buffer, buf_size, &packet, sizeof(data_packet_t));
    if (result.status != COBS_ENCODE_OK) {
        ESP_LOGE("COMMS", "COBS encode failed: %d", result.status);
    }
    ESP_LOGI("COMMS", "packet size=%d, buf_size=%d, encoded_len=%d", sizeof(data_packet_t), buf_size, result.out_len);
    *out_len = result.out_len;
}
// ----------------------------------------------------------------------------------------------------------------------


// ------------------- FUNCTIONALITY FOR RECEIVING DATA OVER UART -------------------------------------

void uart_receive_task(void* arg){

    // want to get the data over UART
    QueueHandle_t q = (QueueHandle_t) arg;

    // want these to persist across multiple iterations
    static uint8_t read_buf[(sizeof(target_speed_packet_t) + 2) * 4];       // max size in gobilda read function as well (made it four packets)
    static size_t buf_len = 0;

    for (;;){

        // if there is data over UART
        int bytes_read = uart_read_bytes(uart_num, read_buf + buf_len, sizeof(read_buf) - buf_len, pdMS_TO_TICKS(10));
        if (bytes_read > 0){    
            buf_len += bytes_read;
        }
        
        // finding the 0x00 delimeter
        int r = 0;
        uint8_t encoded_frame[TSP_ENCODED_SIZE];
        bool found_packet = false;

        while (r < buf_len){

            // same logic as the orin code -> 0x00 byte
            if (!read_buf[r]){

                if (r == 0){
                    
                    // the same thing as .erase() in c++
                    memmove(read_buf, read_buf + 1, buf_len - 1);
                    buf_len--;
                    r = 0;
                    continue;
                }

                // copying the packet into an encoded frame
                size_t copy_len = (r < TSP_ENCODED_SIZE) ? r : TSP_ENCODED_SIZE;
                memcpy(encoded_frame, read_buf + (r - TSP_ENCODED_SIZE), copy_len);

                // remove bytes between beginning and r and updating the buf_len
                memmove(read_buf, read_buf + r + 1, buf_len - (r + 1));
                buf_len = buf_len - (r + 1);

                found_packet = true;
                break;
            }
            r++;
        }

        // couldn't find a full packet
        if (!found_packet){
            continue;
        }

        target_speed_packet_t tsp;

        if (r != sizeof(encoded_frame)) continue;

        cobs_decode_result res = cobs_decode(&tsp, sizeof(target_speed_packet_t), encoded_frame, sizeof(encoded_frame)); // decode into struct
        if (res.status != COBS_DECODE_OK){
            // print some error ESP_LOGE
            continue;
        } 

        if (res.out_len != sizeof(target_speed_packet_t)){
            // print some error ESP_LOGE
            continue;
        }

        uint16_t true_checksum = tsp.checksum;
        tsp.checksum = 0;
        uint16_t calc_checksum = calculate_checksum(&tsp, sizeof(target_speed_packet_t));
        
        // drop the packet
        if (true_checksum != calc_checksum){
            // print some error ESP_LOGE
            continue;
        }

        // all the checks passed, for now just print the data to the terminal or add to queue
        xQueueSend(q, &tsp, 0);     // 0 means it won't block when the queue is full
    }
}

// -----------------------------------------------------------------------------------------------------

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