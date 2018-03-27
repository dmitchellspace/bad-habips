#include "GPIO.h"

const int OnBoardLED = 13, TeensyButton = 18, RedLED = 19, GreenLED = 24, BlueLED = 25;
//This is the Fault Matrix that is being used to display what failures have occured on the LEDs.  It is an 8 element boolean array that marks everything as a pass or a fail
bool FaultMatrix[8] = { 0 }; // 8 Element Array

/*
The array is formatted as follows:
[Current Sensor, ADC0, ADC1, DAQCS SPI, Temp/Press, IMU0, IMU1, SD Card]
There are two kinds of failures defined, "Hard Failures", and "Soft Failures".  Hard failures make it such that the unit is not operational.  A soft failure means that
something is not working, but the unit can fly anyway.  A hard failure is defined by two different scenarios:
1) IMU0 and IMU1 Failures
2) SD Card Failure
A soft failure is any other failure that occurs.
A hard failure is defined by the Red LED on the board being turned on and left on until reset.
A soft failure is defined by blinking the Red LED.  The LED is blinked the position in the array + 1 times, ie. Current sensor = 1 time.
If multiple soft failures occur there is a two second delay between each set.
*/

void Init_GPIO() {
	pinMode(OnBoardLED, OUTPUT);//Set up Pin to blink on board LED
	pinMode(RedLED, OUTPUT); //Red LED as output
	digitalWrite(RedLED, LOW); //Init As Off
	pinMode(GreenLED, OUTPUT);//Set up Pin to Control Green LED.
	digitalWrite(GreenLED, LOW); //Init As Off
	pinMode(BlueLED, OUTPUT);//Set up Pin to Control Blue LED.  This LED is used as the heartbeat.
	digitalWrite(BlueLED, LOW); //Init As Off
	pinMode(TeensyButton, INPUT);//This is the pin that's used in order to trigger the calibration sequence
	Serial.println("GPIO Successfully Initialized");
}

void SetupTimerButton() { //
	//PORTB_PCR3 |= 1090000; //Clear interrupt flag, enable interrupt on rising edge.
}

void SelfTestDisplayResults() {
	if ((FaultMatrix[7]) || (FaultMatrix[5] && FaultMatrix[6])) { //Is there a hard failure?
		digitalWrite(RedLED, HIGH);
	}
	else{ //Check for soft failures
		digitalWrite(GreenLED, HIGH); //Set the green LED if there is no Hard failure
		for (int counter = 0; counter <= 6; counter++) { //Check each of the bits, except for the SD Card

			if (FaultMatrix[counter]) { //Did it fail?
				for (int BlinkNumber = 0; BlinkNumber < (counter + 1); BlinkNumber++) {
					digitalWrite(RedLED, HIGH); //Turn it on
					delay(200); //Delay
					digitalWrite(RedLED, LOW); //Turn it off
					delay(200); //Delay
				} //End blink for loop
				delay(2000); //Wait two seconds to distinguish between faults.
			} //End inner if

		} //End outer for loop
	} //End else
}