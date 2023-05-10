#include <stdio.h>
extern "C" {
    #include <wiringPi.h>
    #include <wiringPiSPI.h>
    #include "spi_com.h"
}

// Motor 1 = left
// Motor 2 = right   

extern int encoders_ready;     // Flag to indicate if encoders are ready

const float Motor_C = 4096.0;   // Motor count per revolution
const float Gearbox1 = 6.6/1.0; // Gearbox 1 ratio
const float Motor_C2 = Motor_C * Gearbox1; // Motor count per revolution after gearbox 1
const float Gearbox2 = 24.0/1.0; // Gearbox 2 ratio
const float Wheel_C = Motor_C2 * Gearbox2; // Motor count per revolution after gearbox 2

void call_motors(int l_speed, int r_speed);
void init_motors();

// Get functions..
float get_delta_D();
float get_delta_theta();
int get_l_encoder();
int get_r_encoder();
int get_l_motorSpeed();
int get_r_motorSpeed();