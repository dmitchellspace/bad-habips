// 
// 
// 

#include "I2C.h"
#include<Wire.h>

const int ClockSpeed = 400000; //Max Clock Frequency is 400kHz
const int Timeout = 200000; //200ms Timeout
int	I2CSelfTestBit = 0;

// Temperature Variable
const int TemperatureAddress = 0x48;
const int TemperatureLength = 2; //Temperature Data is 2 Bytes
char TemperatureBuffer[TemperatureLength];
const byte TemperaturePointer = 0x00; //To Read Temperature 

//Pressure Variables
const int PressureAddress = 0x77;
const int PressureLength = 3; //Pressure Data is 3 Bytes Long
char PressureBuffer[PressureLength];
const byte ResetPressure = 0x1E;  
const int InitPressureSequence = 0x48;
const int ReadPressure = 0x00;

void Init_I2C() { //Initialize 
	Wire2.begin(); //Initialize I2C as Master
	Wire2.setClock(400000); //400kHz clock
	Wire2.setTimeout(200000); //200ms Timeout
	//Will need to set up interrupt
	SetTemperaturePointer();
	ResetPressureSensor();//This is done here in order to check if we can talk to the sensor	
	if (I2CSelfTestBit) {
		Serial.println("I2C Failed to Initialize");
	}
	else {
		Serial.println("I2C Successfully Initialize");
	}
}

void SetTemperaturePointer() { //Set Pointer to temperature for temperature sensor.  Only needs to be done on reset.
	Wire2.beginTransmission(TemperatureAddress);  // Write to the Teperature Sensor
	Wire2.write(TemperaturePointer);  // Write Pointer to the Temperature Sensor
	Wire2.endTransmission(); // End Transmission
	if (Wire2.getWriteError()) {
		I2CSelfTestBit = 1;
	}
}

void ResetPressureSensor() {
	Wire2.beginTransmission(PressureAddress); //Write to Pressure Sensor
	Wire2.write(ResetPressure); //Reset It
	Wire2.endTransmission(); //End
	if (Wire2.getWriteError()) {
		I2CSelfTestBit = 1;
	}
}