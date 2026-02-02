// here is the struct for sending the data to the queue
#include <stdint.h>


typedef struct {

    int32_t left_tick;      // var for left_pcnt_tick
    int32_t right_tick;     // var for right_pcnt_tick
    uint32_t timestamp_ms;  // var for timestamp

} encoder_data;
