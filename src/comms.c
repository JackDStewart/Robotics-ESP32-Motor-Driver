// here is the C file we can use for communication

// Here is my idea:
// We connect the ESP32 to the Jetson GPU Serially (using UART)
// In this module we will do all the sending of the data and use Core 0 (I was thinking that we use Core 1 for data crunching)


// In the documentation for UART Transmission:
// Events defined in uart_event_type_t can be reported to a user application using the FreeRTOS queue functionality.

#include "comms.h"


// here is where we will initialize UART sending
void uart_init(void){



}




