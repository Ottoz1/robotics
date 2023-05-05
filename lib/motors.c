#include "motors.h"

MotorDataType MotorData;
static const int SPI_Channel = 1;

// Create variables for the encoder values as singed int 32 
signed int l_encoder = 0;
signed int r_encoder = 0;
int encoders_ready = 0; // Flag to indicate if the encoders are ready

void init_motors(){
    wiringPiSetup(); 
	wiringPiSPISetup(SPI_Channel, 1000000);
    l_encoder = 0;
    r_encoder = 0;
    encoders_ready = 0;
}

void call_motors(signed int l_speed, signed int r_speed){
    encoders_ready = 0;

    MotorData.Set_Speed_M1 = l_speed;
    MotorData.Set_Speed_M2 = r_speed;

    Send_Read_Motor_Data(&MotorData);

    l_encoder = MotorData.Encoder_M1;
    r_encoder = MotorData.Encoder_M2;

    encoders_ready = 1;
}

void test_motors(){
    short Des_Speed = 0;
	int Select = 0;
	int Counter = 0;

    while(1){
		delay(50);
		Counter++;
		if(Counter > 40){
			Counter =0;
			switch(Select){
				case 0:
					Des_Speed = 3000;
					Select = 1;
				break;
				case 1:
					Des_Speed = -3000;
					Select = 2;
				break;
				case 2:
					Des_Speed = -3000;
					Select = 3;
				break;
				case 3:
					Des_Speed = 0;
					Select = 0;
				break;
			}
			printf("Speed_M1=%d Speed_M2=%d Enkoder_M1= %d Enkoder_M2 %d\n", MotorData.Act_Speed_M1,MotorData.Act_Speed_M2,MotorData.Encoder_M1,MotorData.Encoder_M2);
		}
		MotorData.Set_Speed_M1=Des_Speed;
		MotorData.Set_Speed_M2=150;
		Send_Read_Motor_Data(&MotorData);
	}
}
