// 
// 
// 

#include "I2C.h"

// Temp Pressure Sensor Data

/*
Bit banging is going to be used for I2C.  The on board I2C modue was used but it was not functioning correctly.  Multiple days were taken to
investigate, and debug.  The "wire" Arduino library was also run and tested, and similar issues rose.  The main issue was that the Teensy would
not release the Data line when the Sensors would try to write.  This resulted in the sensor trying to drive the line low, while the teensy was 
trying to drive the line high.  This resulted in about 1.7V being the "low", and this was being viewed as a 1 by the Teensy buffer.  Multiple
days were spent looking into this issue with Carlos Barrios and no solution was found.  Due to a shortage in time this is being changed to bit 
banging.  This is not nearly as efficient as the build in I2C, but due to time constraints bit banging will be used.

In order to make this as efficient as possible none of the arduino libaries are going to be used.  All of the pin configurations, and pin reads are
going to be done via register writes in order to make it run as quickly as possible.
*/ 

const byte TempPressureAddress = 0x77, TempPressureStartRegister = 0xF7, TempPressureNumRegisters = 6;
const byte TempPressureSelfTestRegister = 0xD0, TempPressureSelfTestData = 0x60, TemperatureEnableRegister = 0xF4;
const byte TemperatureSettingsRegister = 0xF5, TemperatureEnableData = 0x27, TemperatureSettingsData = 0x80;
const int BitBangDelay = 4; //122kHz signal
byte I2CRxData[6];


void Init_I2C() { //Initialize 
	PORTA_PCR12 = 0x100; //Pin 3 (A12) is the SCL line
	PORTA_PCR13 = 0x100; //Pin 4 (A13) is the SDL line
	GPIOA_PDDR |= 0x3000; //Set up the pins as outputs
	GPIOA_PSOR |= 0x3000; //Set up the outputs as high
	Serial.println("I2C Successfully Initalized");
	TempPressureRead(1, TempPressureAddress, TempPressureSelfTestRegister);
	if(I2CRxData[5] == TempPressureSelfTestData){
		Serial.println("Temperature/Pressure Sensor Successfully Initialize");
		TempPressureWrite(TempPressureAddress, TemperatureEnableRegister, TemperatureEnableData); //Enable the Sensor
		TempPressureWrite(TempPressureAddress, TemperatureSettingsRegister, TemperatureSettingsData); //Setting to sample every 0.5s
	}
	else {
		Serial.println("Temperature/Pressure Sensor Failed To Initialize");
	}
}

void TempPressureWrite(byte I2CAddress, byte I2CRegister, byte TxData) {
	//A write to the temperature/pressure sensor requires three seperate writes.  The first is the I2C Address, the second is the register number,
	//and the third is the data.

	byte TempAddress = I2CAddress, TempI2CRegister = I2CRegister, TempTxData = TxData;
	bool WriteAddressTxData[8], RegisterTxData[8], TransmitData[8];

	WriteAddressTxData[0] = 0; //LSB is 0 for Write

	RegisterTxData[0] = (TempI2CRegister % 2); //This is run one less time so the first will be done out here
	TempI2CRegister = TempI2CRegister >> 1; //Shift one
	TransmitData[0] = (TempTxData % 2); //This is run one less time so the first will be done out here
	TempTxData = TempTxData >> 1; //Shift

	for (int counter = 1; counter < 8; counter++) { //The data is going to be preprocessed to make the TX sequence faster
		WriteAddressTxData[counter] = (TempAddress % 2); //Get the LSB
		RegisterTxData[counter] = (TempI2CRegister % 2);
		TransmitData[counter] = (TempTxData % 2);
		TempI2CRegister = TempI2CRegister >> 1; //Shift one
		TempAddress = TempAddress >> 1; //Shift one
		TempTxData = TempTxData >> 1; //Shift one
	}

	GPIOA_PCOR |= 0x2000; //Start Bit
	delayMicroseconds(BitBangDelay / 2); //Wait

	for (int counter = 7; counter >= 0; counter--) { //Tx the Write Address

		GPIOA_PCOR |= 0x1000; //Clear the clock
		delayMicroseconds(BitBangDelay / 2); //Wait
		if (WriteAddressTxData[counter]) {
			GPIOA_PSOR |= 0x2000; //Data = 1
		}
		else {
			GPIOA_PCOR |= 0x2000; //Data = 0
		}
		delayMicroseconds(BitBangDelay / 2); //Wait
		GPIOA_PSOR |= 0x1000; //Set the clock
		delayMicroseconds(BitBangDelay); //Wait
	}
	GPIOA_PDDR &= ~0x2000; //Set up the SDA as input for ACK
	GPIOA_PCOR |= 0x1000; //ACK Bit Ready
	delayMicroseconds(BitBangDelay);
	GPIOA_PSOR |= 0x1000; //Read ACK Bit
	delayMicroseconds(BitBangDelay); //Wait
	GPIOA_PDDR |= 0x2000; //Set up the SDA as output for ACK
	GPIOA_PCOR |= 0x1000; //CLK Low
	GPIOA_PCOR |= 0x2000; //Output Low
	delayMicroseconds(2 * BitBangDelay); //Wait

	for (int counter = 7; counter >= 0; counter--) { //Tx the register

		GPIOA_PCOR |= 0x1000; //Clear the clock
		delayMicroseconds(BitBangDelay / 2); //Wait
		if (RegisterTxData[counter]) {
			GPIOA_PSOR |= 0x2000; //Data = 1
		}
		else {
			GPIOA_PCOR |= 0x2000; //Data = 0
		}
		delayMicroseconds(BitBangDelay / 2); //Wait
		GPIOA_PSOR |= 0x1000; //Set the clock
		delayMicroseconds(BitBangDelay); //Wait
	}

	GPIOA_PDDR &= ~0x2000; //Set up the SDA as input for ACK
	GPIOA_PCOR |= 0x1000; //ACK Bit Ready
	delayMicroseconds(BitBangDelay);
	GPIOA_PSOR |= 0x1000; //Read ACK Bit
	delayMicroseconds(BitBangDelay); //Wait
	GPIOA_PDDR |= 0x2000; //Set up the SDA as output for ACK
	GPIOA_PCOR |= 0x1000; //CLK Low
	GPIOA_PCOR |= 0x2000; //Output Low
	delayMicroseconds(2 * BitBangDelay); //Wait

	for (int counter = 7; counter >= 0; counter--) { //Tx the Data

		GPIOA_PCOR |= 0x1000; //Clear the clock
		delayMicroseconds(BitBangDelay / 2); //Wait
		if (TransmitData[counter]) {
			GPIOA_PSOR |= 0x2000; //Data = 1
		}
		else {
			GPIOA_PCOR |= 0x2000; //Data = 0
		}
		delayMicroseconds(BitBangDelay / 2); //Wait
		GPIOA_PSOR |= 0x1000; //Set the clock
		delayMicroseconds(BitBangDelay); //Wait
	}

	GPIOA_PDDR &= ~0x2000; //Set up the SDA as input for ACK
	GPIOA_PCOR |= 0x1000; //ACK Bit Ready
	delayMicroseconds(BitBangDelay);
	GPIOA_PSOR |= 0x1000; //Read ACK Bit
	delayMicroseconds(BitBangDelay); //Wait
	GPIOA_PDDR |= 0x2000; //Set up the SDA as output for ACK
	GPIOA_PCOR |= 0x1000; //Clear the clock
	delayMicroseconds(BitBangDelay / 2);
	GPIOA_PCOR |= 0x2000; //Prepare for Stop Bit
	delayMicroseconds(BitBangDelay / 2);
	GPIOA_PSOR |= 0x1000; //Set the Clock
	delayMicroseconds(BitBangDelay / 2);
	GPIOA_PSOR |= 0x2000; // Stop Bit
}

void TempPressureRead(byte NumBytesRx, byte I2CAddress, byte I2CRegister) {
	//The process to perform a read is to write the address in write mode along with the register being read from.  Then the start bit is sent a second time
	// the address with a read bit, and then the data will be returned.

	byte TempAddress = I2CAddress, TempI2CRegister = I2CRegister;
	bool WriteAddressTxData[8], ReadAddressTxData[8], RegisterTxData[8];
	int ArrayCounter = 5;

	WriteAddressTxData[0] = 0; //LSB is 0 for Write
	ReadAddressTxData[0] = 1; //LSB is 1 for Read

	RegisterTxData[0] = (TempI2CRegister % 2); //This is run one less time so the first will be done out here
	TempI2CRegister = TempI2CRegister >> 1; //Shift one

	for (int counter = 1; counter < 8; counter++) { //The data is going to be preprocessed to make the TX sequence faster
		WriteAddressTxData[counter] = (TempAddress % 2); //Get the LSB
		ReadAddressTxData[counter] = WriteAddressTxData[counter];
		RegisterTxData[counter] = (TempI2CRegister % 2);
		TempI2CRegister = TempI2CRegister >> 1; //Shift one
		TempAddress = TempAddress >> 1; //Shift one
	}

	for (int counter = 0; counter < 6; counter++) {
		I2CRxData[counter] = 0;
	}

	GPIOA_PCOR |= 0x2000; //Start Bit
	delayMicroseconds(BitBangDelay/2); //Wait

	for (int counter = 7; counter >= 0; counter--) { //Tx the Write Address

		GPIOA_PCOR |= 0x1000; //Clear the clock
		delayMicroseconds(BitBangDelay / 2); //Wait
		if (WriteAddressTxData[counter]) {
			GPIOA_PSOR |= 0x2000; //Data = 1
		}
		else {
			GPIOA_PCOR |= 0x2000; //Data = 0
		}
		delayMicroseconds(BitBangDelay/2); //Wait
		GPIOA_PSOR |= 0x1000; //Set the clock
		delayMicroseconds(BitBangDelay); //Wait
	}
	GPIOA_PDDR &= ~0x2000; //Set up the SDA as input for ACK
	GPIOA_PCOR |= 0x1000; //ACK Bit Ready
	delayMicroseconds(BitBangDelay);
	GPIOA_PSOR |= 0x1000; //Read ACK Bit
	delayMicroseconds(BitBangDelay); //Wait
	GPIOA_PDDR |= 0x2000; //Set up the SDA as output for ACK
	GPIOA_PCOR |= 0x1000; //CLK Low
	GPIOA_PCOR |= 0x2000; //Output Low
	delayMicroseconds(2*BitBangDelay); //Wait

	for (int counter = 7; counter >= 0; counter--) { //Tx the register

		GPIOA_PCOR |= 0x1000; //Clear the clock
		delayMicroseconds(BitBangDelay / 2); //Wait
		if (RegisterTxData[counter]) {
			GPIOA_PSOR |= 0x2000; //Data = 1
		}
		else {
			GPIOA_PCOR |= 0x2000; //Data = 0
		}
		delayMicroseconds(BitBangDelay / 2); //Wait
		GPIOA_PSOR |= 0x1000; //Set the clock
		delayMicroseconds(BitBangDelay); //Wait
	}

	GPIOA_PDDR &= ~0x2000; //Set up the SDA as input for ACK
	GPIOA_PCOR |= 0x1000; //ACK Bit Ready
	delayMicroseconds(BitBangDelay);
	GPIOA_PSOR |= 0x1000; //Read ACK Bit
	delayMicroseconds(BitBangDelay); //Wait
	GPIOA_PDDR |= 0x2000; //Set up the SDA as output for ACK
	GPIOA_PCOR |= 0x1000; //CLK Low
	GPIOA_PSOR |= 0x2000; //Output High
	delayMicroseconds(BitBangDelay);
	GPIOA_PSOR |= 0x1000; //Clock High
	delayMicroseconds(BitBangDelay / 2);
	GPIOA_PCOR |= 0x2000; //Resend Start Bit
	delayMicroseconds(2*BitBangDelay);

	for (int counter = 7; counter >= 0; counter--) { //Tx the Read Address

		GPIOA_PCOR |= 0x1000; //Clear the clock
		delayMicroseconds(BitBangDelay / 2); //Wait
		if (ReadAddressTxData[counter]) {
			GPIOA_PSOR |= 0x2000; //Data = 1
		}
		else {
			GPIOA_PCOR |= 0x2000; //Data = 0
		}
		delayMicroseconds(BitBangDelay / 2); //Wait
		GPIOA_PSOR |= 0x1000; //Set the clock
		delayMicroseconds(BitBangDelay); //Wait
	}

	GPIOA_PDDR &= ~0x2000; //Set up the SDA as input for ACK
	GPIOA_PCOR |= 0x1000; //ACK Bit Ready
	delayMicroseconds(BitBangDelay);
	GPIOA_PSOR |= 0x1000; //Read ACK Bit
	delayMicroseconds(BitBangDelay); //Wait
	GPIOA_PCOR |= 0x1000; //CLK Low
	delayMicroseconds(BitBangDelay); //Wait

	for (byte CurrentBytesRx = 0; CurrentBytesRx < NumBytesRx; CurrentBytesRx++) { //Stay in this loop for number of bytes

		GPIOA_PDDR &= ~0x2000; //Set up the SDA as Input

		for (int counter = 7; counter >= 0; counter--) { //Read
			I2CRxData[ArrayCounter] = I2CRxData[ArrayCounter] << 1;
			GPIOA_PCOR |= 0x1000; //Clear the clock
			delayMicroseconds(BitBangDelay); //Wait
			GPIOA_PSOR |= 0x1000; //Set the clock
			if ((GPIOA_PDIR & 0x2000) == 0x2000) {
				I2CRxData[ArrayCounter] |= 0x01;
			}
			delayMicroseconds(BitBangDelay); //Wait
		} //End byte for

		GPIOA_PCOR |= 0x1000; //Clear the clock
		GPIOA_PDDR |= 0x2000; //Set up the SDA as output for ACK
		if (CurrentBytesRx == (NumBytesRx - 1)) {
			GPIOA_PSOR |= 0x2000; // NACK Bit
		}
		else {
			GPIOA_PCOR |= 0x2000; // ACK Bit
		}

		delayMicroseconds(BitBangDelay); //Wait
		GPIOA_PSOR |= 0x1000; //Set the clock
		delayMicroseconds(BitBangDelay); //Delay
		ArrayCounter--; //Dec which element of the array you're putting the data in
	} //End Receive For

	GPIOA_PCOR |= 0x1000; //Clear the clock
	delayMicroseconds(BitBangDelay / 2);
	GPIOA_PCOR |= 0x2000; //Prepare for Stop Bit
	delayMicroseconds(BitBangDelay/2);
	GPIOA_PSOR |= 0x1000; //Set the Clock
	delayMicroseconds(BitBangDelay / 2);
	GPIOA_PSOR |= 0x2000; // Stop Bit
}