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
Version 3:
	Uploaded on 12/01/2017
	LED is blinked three times quickly to show user SD card is not working.
	I2C Initiliazation is setup.  Verified correct address and R/W commands were being TXd
	Without any of the components we cannot verify it is working, because we don't get an 
	acknowledge byte back.  A RX interrupt still needs to be setup.
	Pins for motor control are setup.
	PWM is setup and verified.  PWM signal will be running at 1kHz as opposed to 50 Hz from last semester.
Version 4:
	Uploaded on 12/06/2017
	ADC Initialization was implemented and successfully tested.
	A calibration of the unit is performed and the Successfull/Not Successful is determined based off of the cal results.
	The ISR has not yet been integrated.
	Frequency of PWM signal was changed to 75Hz.  This allows for more varience in the PW.
	SPI has been initialized.  Nothing has been tested yet, this still needs to be done.  Interrupts need to be set up.
*/

int BootTime;

#include "Clocks.h"
#include "GPIO.h"
#include "I2C.h"
#include "SPI.h"
#include "SDCard.h"
#include "MotorControl.h"

void setup() { //Only runs once upon powering up the board

	Serial.begin(9600); //Set Up Serial Interface
	//DEBUG
	while (!Serial); // DEBUG DEBUG DEBUG THE PROGRAM WILL NOT START UNTIL THE SERIAL COMM PORT 
	//RESPONDS MAKE SURE TO TAKE OUT
	//DEBUG
	BootTime = millis();
	ADC_Calibration(); //Caibration takes some time to complete, so initiate it before anything else is done
	Init_GPIO(); //Do initilization on GPIO Pins
	Init_RTC(); //Initiliaze Real Time Clock
	SDCard_Setup();//Do initial setup for SD Card
	Init_SPI();//Init\iliaze SPI interface
	Init_I2C();//Initiliaze I2C interface
	Init_MotorInterface(); //Initliaze Motor Interface

	Serial.print("Initialize Time = "); //Display Time it took to initilize
	Serial.print(millis() - BootTime); //Display Time it took to initilize
	Serial.print("ms");//Time is in milliseconds
	Serial.println(); //New Line
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
			for (int i = 1; i < 7;i++) {//Blink the LED Three Quick Times to let the user know SD Card is not working
				GPIOC_PTOR ^= 0x20;
				delay(200);
			}
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