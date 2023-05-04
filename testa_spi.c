#include "lib/motors.h"

int main(void){
	init_motors();
	while (1)
	{
		call_motors(200, 200);
		printf("Left encoder: %d\n", l_encoder);
		printf("Right encoder: %d\n", r_encoder);
		delay(50);
	}
}
