/*
Name:		TeensyMotorControl_PsuedoCode.ino
Created:	11/14/2017 7:12:13 PM
Brandon Codi
Code for Teensy Motor Controller on HABIP
Version 1:
	Uploaded on 11/20/17
	Psuedo Code
	GPIO Setup
	Heartbeat
Version 2:
	Uploaded on 11/22/2017
	RTC Setup
	SD Card Initlization
	Simple Self Test run on RTC and SD Card, Results printed to Comm Port
	If SD Card Fails to Initilize, it will continue to retry in the main loop.
*/

//DEBUG
int Time;
int InitTime;
//DEBUG

#include "Clocks.h"
#include "GPIO.h"
#include "I2C.h"
#include "SPI.h"
#include "SDCard.h"
#include "MotorControl.h"

void setup() { //Only runs once upon powering up the board

	//DEBUG
	Serial.begin(9600); //Set Up Serial Interface
	while (!Serial); // DEBUG DEBUG DEBUG THE PROGRAM WILL NOT START UNTIL THE SERIAL COMM PORT RESPONDS MAKE SURE TO TAKE OUT
	Time = millis();
	//DEBUG

	Init_GPIO(); //Do initilization on GPIO Pins
	Init_RTC(); //Initiliaze Real Time Clock
	SDCard_Setup();//Do initial setup for SD Card
	Init_Clock(); //Initiliaze System Clocks
	Init_SPI();//Initiliaze SPI interface
	Init_I2C();//Initiliaze I2C interface
	Init_MotorInterface(); //Initliaze Motor Interface
	Init_ADC(); //Initliaze ADC, used for feedback from Motor

	//DEBUG
	Serial.print("Initialize Time = "); //Display Time it took to initilize
	Serial.print(millis() - Time); //Display Time it took to initilize
	Serial.print("ms");//Time is in milliseconds
	Serial.println(); //New Line
	//DEBUG
}

// the loop function runs over and over again until power down or reset
void loop() {//Main Loop
	if (!digitalRead(PIN_A3)) {//Check to see if motor should be turned on
		CollectData();
	}
	else { //Motor should be on
		ReactionWheelOn();
	}
}

void CollectData() {
	 while (!digitalRead(PIN_A3)) { //Stay in this loop until Reaction Wheel is turned on
		 
		 GPIOC_PTOR ^= 0x20; //Toggle On board LED (Pin C5).
		 GPIOA_PTOR ^= 0x20; //Toggle Blue LED (Pin A5).

		 //DEBUG
		 delay(1000);  // wait for a second
		 //DEBUG

		 if (SDCardPresent == 0) { //SD Card is not present
			SDCard_Setup(); //Try to set it up again
			}
		else {//SD Card is there, store data
			//Collect Data When Reaction Wheel is Not On
		}
	}
}

void ReactionWheelOn() {
	GPIOC_PTOR ^= 0x20; //Toggle On board LED (Pin C5).
	GPIOA_PTOR ^= 0x20; //Toggle Blue LED (Pin A5).

	//DEBUG
	delay(1000);  // wait for a second
	//DEBUG

	while (digitalRead(PIN_A3)) {//Stay in this loop until Reaction Wheel is turned off
		//Run Reaction Wheel Algorithim
	}
}