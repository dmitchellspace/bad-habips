// 
// 
// 
#include "Clocks.h"
#include <TimeLib.h>
#include <Wire.h>
#include <DS1307RTC.h>  // a basic DS1307 library that returns time as a time_t

int hours = 0; //Used For Real Time Clock
int minutes = 0; //Used For Real Time Clock
int seconds = 0; //Used For Real Time Clock
int date = 20; //Used For Real Time Clock
int mnth = 11; //Used For Real Time Clock
int yr = 2017; //Used For Real Time Clock

void Init_RTC() { //TODO Look at adding code to check if the RTC was already set up (ie. The Teensy lost power but the RTC did not)
	setTime(hours, minutes, seconds, date, mnth, yr); //Load Data into RTC

	if (timeStatus() == timeSet) { //Check if RTC was successfully setup
		Serial.println("RTC Successfully Initialized"); //It was
	}
	else {
		Serial.println("RTC Failed"); //It was not
	}
}