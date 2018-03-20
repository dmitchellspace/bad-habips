// 
// 
// 
#include "Clocks.h"
#include <TimeLib.h>
#include <DS1307RTC.h>  // a basic DS1307 library that returns time as a time_t

const int hours = 0, minutes = 0, seconds = 0, date = 22,  mnth = 2, yr = 2018; //Used For Real Time Clock
int RTCCurrentData[6]; //Goes from small to big ie: [0] = seconds [5] = yr

void Init_RTC() {
	if (timeStatus() != timeSet) { //Did the teensy reset, but the RTC retain power?
		setTime(hours, minutes, seconds, date, mnth, yr); //Load Data into RTC
	}

	if (timeStatus() == timeSet) { //Check if RTC was successfully setup
		Serial.println("RTC Successfully Initialized"); //It was
	}
	else {
		Serial.println("RTC Failed"); //It was not
	}
}

void GetClock() {
	RTCCurrentData[0] = second();
	RTCCurrentData[1] = minute();
	RTCCurrentData[2] = hour();
	RTCCurrentData[3] = day();
	RTCCurrentData[4] = month();
	RTCCurrentData[5] = year();
}

void Init1SecTimer() {
	SIM_SCGC6 |= 1000000; //Enable FTM0
	FTM0_SC = 0x80; //Clear the register
	FTM0_C0SC = 0x80; //Clear the register
	FTM0_CNT = 0x00; //Clear the counter
	FTM0_MODE |= 0x40; //Allows you to write to the module
	FTM0_MOD = 0x7A12; //Count to 1 sec
	FTM0_SC = 0xD0; //FF Clock, IE, Divider = 1
	NVIC_ISER1 |= 0x400; //Enable interrupts
	FTM0_CONF = 0xC0; //Run mode
	FTM0_MODE = 0x00; //Enable Module
}