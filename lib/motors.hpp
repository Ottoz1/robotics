#include <stdio.h>
extern "C" {
    #include <wiringPi.h>
    #include <wiringPiSPI.h>
    #include "spi_com.h"
}

// Motor 1 = left
// Motor 2 = right   

extern int encoders_ready;     // Flag to indicate if encoders are ready

void call_motors(int l_speed, int r_speed);
void init_motors();

// Get functions..
float get_delta_D();
float get_delta_theta();
int get_l_encoder();
int get_r_encoder();
int get_l_motorSpeed();
int get_r_motorSpeed();