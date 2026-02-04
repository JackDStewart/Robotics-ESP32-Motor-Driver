// here is the struct for sending the data to the queue
#include <stdint.h>


typedef struct {
    uint16_t SOF; // signal new packet
    uint8_t seq; // for testing lost packets
    uint8_t length; // length of payload
    uint8_t type_flag; // 1 == data, 0 == flag
    uint16_t checksum;

} __attribute__((packed)) uart_header_t;

typedef struct {

    int32_t left_tick;      // var for left_pcnt_tick
    int32_t right_tick;     // var for right_pcnt_tick
    uint32_t timestamp_ms;  // var for timestamp

} __attribute__((packed)) encoder_data_t;

typedef struct {

    uint8_t error_type; // make later

} __attribute__((packed)) encoder_err_t;
