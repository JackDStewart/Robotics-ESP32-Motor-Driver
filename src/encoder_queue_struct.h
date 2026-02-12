// here is the struct for sending the data to the queue
#pragma once
#include <stdint.h>

typedef struct {
    int16_t dL;      // delta left
    int16_t dR;     // delta right
    uint32_t timestamp_ms;  // var for timestamp
} __attribute__((packed)) encoder_data_t;

typedef struct {

    uint8_t seq; // seq num for testing loss
    encoder_data_t encoder_data;
    uint16_t checksum; 

} __attribute__((packed)) data_packet_t;
