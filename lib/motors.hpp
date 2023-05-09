#include <stdio.h>
extern "C" {
    #include <wiringPi.h>
    #include <wiringPiSPI.h>
    #include "spi_com.h"
}

extern int l_encoder;   // Left encoder value
extern int r_encoder;   // Right encoder value
extern int encoders_ready;     // Flag to indicate if encoders are ready
extern int m1_speed;
extern int m2_speed;

void call_motors(int l_speed, int r_speed);
void init_motors();