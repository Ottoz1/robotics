#include <wiringPi.h>
#include <stdio.h>
#include <wiringPiSPI.h>
#include "spi_com.h"

extern signed int l_encoder;   // Left encoder value
extern signed int r_encoder;   // Right encoder value
extern int encoders_ready;     // Flag to indicate if encoders are ready

void call_motors(signed int l_speed, signed int r_speed);
void init_motors();
void test_motors();