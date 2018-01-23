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
Version 5:
	Uploaded on 01/23/2018
	ADC Interrupt Setup
	SPI transmitting to IMU verified.
	Self Test added where SPI will write to IMU "WHO_AM_I" register. If the correct response is heard back, it will mark it as a pass.
	Right now requests are being made for the IMU data, but nothing is being recorded.
	The IMU read function is set up to read from either of the two chips.  The chip select is an input to the function.
	The IMU that we currently have was wired as an IMU0 and then as an IMU1.  It was verified that we are able to talk to it in either configuration.
*/

//Start Variable Declaration

int BootTime;
byte PressTempFlag = 0; //The pressure and temperature only needs to be collected once a second, and so a timer will be set up to indicate when it is time to check

//This data is used to send the address to the IMU
const byte XAccelAddress = 0x28, YAccelAddress = 0x2A, ZAccelAddress = 0x2C, XGyroAddress = 0x18, YGyroAddress = 0x1A, ZGyroAddress = 0x1C, IMU0 = 0, IMU1 = 1;
int XAccelData, YAccelData, ZAccelData, XGyroData, YGyroData, ZGyroData;

//End Variable Declaration

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
	Serial.println("ms");//Time is in milliseconds
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

			XAccelData = IMURead(XAccelAddress, 0, IMU1); //Collect X Accel
			delay(1000); //DEBUG
			YAccelData = IMURead(YAccelAddress, 0, IMU1); //Collect Y Accel
			delay(1000); //DEBUG
			ZAccelData = IMURead(ZAccelAddress, 0, IMU1); //Collect Z Accel
			delay(1000); //DEBUG
			XGyroData = IMURead(XGyroAddress, 0, IMU1); //Collect X Gyro
			delay(1000); //DEBUG
			YGyroData = IMURead(YGyroAddress, 0, IMU1); //Collect Y Gyro
			delay(1000); //DEBUG
			ZGyroData = IMURead(ZGyroAddress, 0, IMU1); //Collect Z Gyro
			
			if (PressTempFlag == 1) { //Has one second gone by since the last Temp/Press Measurement?
				PressTempFlag = 0; //Reset Flag
			} //End Pressure Temperature if

		} //End SD card else
	} //End While statement
} //End collect Data

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