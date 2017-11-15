/*
 Name:		TeensyMotorControl_PsuedoCode.ino
 Created:	11/14/2017 7:12:13 PM
 Brandon Codi
 Psuedo Code for Teensy Motor Controller on HABIP
*/
#include "Clocks.h"
#include "GPIO.h"
#include "I2C.h"
#include "SPI.h"
#include "SDCard.h"
#include "MotorControl.h"
void setup() { //Only runs once upon powering up the board
	SDCard_Setup();//Do initial setup for SD Card
	Init_GPIO(); //Do initilization on GPIO Pints
	Init_Clock(); //Initiliaze System Clocks
	Init_RTC(); //Initiliaze Real Time Clock
	Init_MotorInterface(); //Initliaze Motor Interface
	Init_SPI();//Initiliaze SPI interface
	Init_I2C();//Initiliaze I2C interface

}

// the loop function runs over and over again until power down or reset
void loop() {//Main Loop
	pinMode(13, OUTPUT);//Set up pin to blink on board LED
	digitalWrite(13, HIGH);   // turn on borad LED on
	delay(1000);              // wait for a second
	digitalWrite(13, LOW);    // turn on board LED off
	delay(1000);              // wait for a second
  //Check to see if motor is turned on
	//if no
	CollectData();
	//else
	ReactionWheelOn();
}

void CollectData() {
	//Collect Data When Reaction Wheel is Not On
}

void ReactionWheelOn() {
	//Run Reaction Wheel Algorithim
}