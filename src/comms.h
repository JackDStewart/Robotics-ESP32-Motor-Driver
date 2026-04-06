// this is going to be the header file we can use for communication
#include "encoder_queue_struct.h"


#define TSP_ENCODED_SIZE sizeof(target_speed_packet_t) + 1 // +1 for COBS overhead

void uart_init(void);
void uart_send_task(void* args);
void build_data_packet(void *buffer, size_t buf_size, encoder_data_t encoder_data, size_t *out_len);
void uart_receive_task(void* arg);
uint16_t calculate_checksum(const void *data, size_t len);