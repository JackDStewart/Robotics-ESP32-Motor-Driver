#include <stdint.h>
#pragma once    // means you only want the compiler to compile it once (it is being included in main.cpp and pcnt_motor_encoder.c)


void encoder_pcnt_init(void);

void encoder_pcnt_read(int16_t *left, int16_t *right);
