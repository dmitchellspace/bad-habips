// 
// 
// 

#include "GPIO.h"
void Init_GPIO() {
	pinMode(13, OUTPUT);//Set up Pin to blink on board LED
	pinMode(20, INPUT);//Set up Pin 20 to Input to Control Turning on Motor from Host MSP430
					   //Green LED is to be used to show that data is being collected on the SD card.
	pinMode(24, OUTPUT);//Set up Pin to Control Green LED.
	pinMode(25, OUTPUT);//Set up Pin to Control Blue LED.  This LED is used as the heartbeat.
	Serial.println("GPIO Successfully Initialized");
}