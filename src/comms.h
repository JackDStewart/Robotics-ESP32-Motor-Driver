// this is going to be the header file we can use for communication
#include "encoder_queue_struct.h"

void uart_init(void);
void uart_send_task(void* args);
void build_data_packet(void *buffer, encoder_data_t encoder_data);
uint16_t calculate_checksum(const void *data, size_t len);