/*
Name:		TeensyMotorControl_PsuedoCode.ino
Created:	11/14/2017 7:12:13 PM
Brandon Codi
Code for Teensy Motor Controller on HABIP
*/
#include "Clocks.h"
#include "GPIO.h"
#include "I2C.h"
#include "SPI.h"
#include "SDCard.h"
#include "MotorControl.h"
void setup() { //Only runs once upon powering up the board
	SDCard_Setup();//Do initial setup for SD Card
	Init_Clock(); //Initiliaze System Clocks
	Init_RTC(); //Initiliaze Real Time Clock
	Init_GPIO(); //Do initilization on GPIO Pints
	Init_SPI();//Initiliaze SPI interface
	Init_I2C();//Initiliaze I2C interface
	Init_MotorInterface(); //Initliaze Motor Interface
}

// the loop function runs over and over again until power down or reset
void loop() {//Main Loop
	GPIOC_PTOR ^= 0x20; //Toggle On board LED (Pin C5).
	GPIOA_PTOR ^= 0x20; //Toggle Blue LED (Pin A5).
	delay(1000);  // wait for a second
	if (!digitalRead(PIN_A3)) {//Check to see if motor should be turned on
		CollectData();
	}

	else { //Motor should be on
		ReactionWheelOn();
	}
}

void CollectData() {
	//Collect Data When Reaction Wheel is Not On
}

void ReactionWheelOn() {
	//Run Reaction Wheel Algorithim
}