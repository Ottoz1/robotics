#include "motors.hpp"

MotorDataType MotorData;
static const int SPI_Channel = 1;

// Create variables for the encoder values as singed int 32 
int l_encoder = 0;
int r_encoder = 0;
int encoders_ready = 0; // Flag to indicate if the encoders are ready

int m1_speed = 0;
int m2_speed = 0;

void init_motors(){
    wiringPiSetup(); 
	wiringPiSPISetup(SPI_Channel, 1000000);
    l_encoder = 0;
    r_encoder = 0;
    encoders_ready = 0;
}

void call_motors(int l_speed, int r_speed){
    encoders_ready = 0;

    MotorData.Set_Speed_M1 = l_speed;
    MotorData.Set_Speed_M2 = r_speed;

    Send_Read_Motor_Data(&MotorData);

    l_encoder = MotorData.Encoder_M1;
    r_encoder = MotorData.Encoder_M2;

	m1_speed = MotorData.Act_Speed_M1;
	m2_speed = MotorData.Act_Speed_M2;

    encoders_ready = 1;
}
