// here is the struct for sending the data to the queue
#pragma once
#include <stdint.h>

extern SemaphoreHandle_t vel_mutex;
extern float shared_velocity_left;
extern float shared_velocity_right;


// struct for sending data over uart
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


// struct for receiving data over UART (different parameters)
typedef struct {

    float target_left_rads;
    float target_right_rads;
    uint16_t seq;
    uint16_t checksum;

} __attribute__((packed)) target_speed_packet_t;

