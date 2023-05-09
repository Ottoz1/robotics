
#include <wiringPi.h>
#include <stdio.h>
#include <wiringPiSPI.h>
#include "spi_com.h"


MotorDataType MotorData;

static const int SPI_Channel = 1;

short Des_Speed = 0;
int Select = 0;
int Counter = 0;

int main(void){

	wiringPiSetup(); 
	wiringPiSPISetup(SPI_Channel, 1000000);
	
	while(1){
		delay(50);
		Counter++;
		if(Counter > 40){
			Counter =0;
			switch(Select){
				case 0:
					Des_Speed = -3000;
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
					Des_Speed = -3000;
					Select = 0;
				break;
			}
			printf("Speed_M1=%d Speed_M2=%d Enkoder_M1= %d Enkoder_M2 %d\n", MotorData.Act_Speed_M1,MotorData.Act_Speed_M2,MotorData.Encoder_M1,MotorData.Encoder_M2);
		}
		MotorData.Set_Speed_M1=Des_Speed;
		MotorData.Set_Speed_M2=3000;
		Send_Read_Motor_Data(&MotorData);
	}
}
