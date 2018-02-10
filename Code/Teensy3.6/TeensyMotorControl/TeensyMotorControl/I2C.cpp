// 
// 
// 

#include "I2C.h"
#include<Wire.h>

int	I2CSelfTestBit = 0;

void Init_I2C() { //Initialize 
	Wire2.begin(); //Initialize I2C as Master
	Wire2.setClock(400000); //400kHz clock
	Wire2.setTimeout(200000); //200ms Timeout
	//Will need to set up interrupt
	I2CSelfTestBit = 1;
	if (I2CSelfTestBit) {
		Serial.println("I2C Failed to Initialize");
	}
	else {
		Serial.println("I2C Successfully Initialize");
	}
}
