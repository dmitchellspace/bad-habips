// 
// 
// 

#include "I2C.h"
#include "SDCard.h"
#include "GPIO.h"

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

const byte TempPressureAddress = 0x76, TempPressureStartRegister = 0xF7, TempPressureNumRegisters = 6;
const byte TempPressureSelfTestRegister = 0xD0, TempPressureSelfTestData = 0x60, TemperatureEnableRegister = 0xF4;
const byte TemperatureSettingsRegister = 0xF5, TemperatureEnableData = 0xB7, TemperatureSettingsData = 0x80, TempCalRegister = 0x88;
const byte PressCalReg1 = 0x8E, PressCalReg2 = 0x94, PressCalReg3 = 0x9A;
const int BitBangDelay = 4; //122kHz signal

//Current Sensor Registers and data
//const byte CurrentSensorAddress = 0x6F;
//According to the schematic the addresss should be 6F because both pins should be grounded.  It seems like one of the pins is actually not 
//connected (thanks to Dan screwing it up), and so the address is actually 0x6E.  If the pin is properly grounded the address should be changed 
//back to 0x6F.

const byte CurrentSensorAddress = 0x6E;
const byte CurrentCtrlReg = 0x00, CurrentADCConfigReg = 0x04, CurrentSensorLDO = 0x46, CurrentSensorMainBattery = 0x14, CurrentSelfTestReg = 0xE8;
const byte CurrentCtrlData = 0x08, CurrentADCConfigData = 0x80, CurrentSelfTestResult = 0x62;
const byte CurrentSensorNumBytes = 1;
//End of current stuff

byte I2CRxData[6];
unsigned short TCal1;
signed short TCal2, TCal3;
unsigned short PCal1;
signed short PCal2, PCal3, PCal4, PCal5, PCal6, PCal7, PCal8, PCal9;

void Init_I2C() { //Initialize 
	PORTA_PCR12 = 0x100; //Pin 3 (A12) is the SCL line
	PORTA_PCR13 = 0x100; //Pin 4 (A13) is the SDL line
	GPIOA_PDDR |= 0x3000; //Set up the pins as outputs
	GPIOA_PSOR |= 0x3000; //Set up the outputs as high
	Serial.println("I2C Successfully Initalized");
	
	InitTempPressure(); //Initialize the temperature and pressure
	InitCurrentSensor(); //Initialize the current
}

void InitTempPressure() {
	I2CRead(1, TempPressureAddress, TempPressureSelfTestRegister);

	if (I2CRxData[5] == TempPressureSelfTestData) {

		Serial.println("Temperature/Pressure Sensor Successfully Initialize");

		I2CWrite(TempPressureAddress, TemperatureEnableRegister, TemperatureEnableData); //Enable the Sensor
		I2CWrite(TempPressureAddress, TemperatureSettingsRegister, TemperatureSettingsData); //Setting to sample every 0.5s
		I2CRead(6, TempPressureAddress, TempCalRegister); //Get Temparature Cal Data

		TCal1 = I2CRxData[4]; //Parse the data
		TCal1 = ((TCal1 << 8) & 0xFF00) | I2CRxData[5]; //Parse the data
		TCal2 = I2CRxData[2]; //Parse the data
		TCal2 = ((TCal2 << 8) & 0xFF00) | I2CRxData[3]; //Parse the data
		TCal3 = I2CRxData[0]; //Parse the data
		TCal3 = ((TCal3 << 8) & 0xFF00) | I2CRxData[1]; //Parse the data

		I2CRead(6, TempPressureAddress, PressCalReg1); //First group of Pressure Measurements
		PCal1 = I2CRxData[4]; //Parse the data
		PCal1 = ((PCal1 << 8) & 0xFF00) | I2CRxData[5]; //Parse the data
		PCal2 = I2CRxData[2]; //Parse the data
		PCal2 = ((PCal2 << 8) & 0xFF00) | I2CRxData[3]; //Parse the data
		PCal3 = I2CRxData[0]; //Parse the data
		PCal3 = ((PCal3 << 8) & 0xFF00) | I2CRxData[1]; //Parse the data

		I2CRead(6, TempPressureAddress, PressCalReg2); //Second group of data
		PCal4 = I2CRxData[4]; //Parse the data
		PCal4 = ((PCal4 << 8) & 0xFF00) | I2CRxData[5]; //Parse the data
		PCal5 = I2CRxData[2]; //Parse the data
		PCal5 = ((PCal5 << 8) & 0xFF00) | I2CRxData[3]; //Parse the data
		PCal6 = I2CRxData[0]; //Parse the data
		PCal6 = ((PCal6 << 8) & 0xFF00) | I2CRxData[1]; //Parse the data

		I2CRead(6, TempPressureAddress, PressCalReg3); //Third group of data
		PCal7 = I2CRxData[4]; //Parse the data
		PCal7 = ((PCal7 << 8) & 0xFF00) | I2CRxData[5]; //Parse the data
		PCal8 = I2CRxData[2]; //Parse the data
		PCal8 = ((PCal8 << 8) & 0xFF00) | I2CRxData[3]; //Parse the data
		PCal9 = I2CRxData[0]; //Parse the data
		PCal9 = ((PCal9 << 8) & 0xFF00) | I2CRxData[1]; //Parse the data

		SDCard_CalibrationDataWrite(TCal1, TCal2, TCal3, 0, 0, 0, 0, 0, 0, 1); //Writes the Temp Cal Data to the SD Card
		SDCard_CalibrationDataWrite(PCal1, PCal2, PCal3, PCal4, PCal5, PCal6, PCal7, PCal8, PCal9, 0); //Writes the Pressure Cal Data to the SD Card
	}

	else {
		Serial.println("Temperature/Pressure Sensor Failed To Initialize");
		FaultMatrix[4] = 1;
	}
}

void InitCurrentSensor() {

	I2CRead(CurrentSensorNumBytes, CurrentSensorAddress, CurrentSelfTestReg); //Read from the ID register
	if (I2CRxData[5] == CurrentSelfTestResult) {
		Serial.println("Current Sensor Successfully Initialized.");
	}
	else {
		Serial.println("Current Sensor Failed To Initialize.");
		FaultMatrix[0] = 1; //Set the entry in the Fault Matrix
	}
	I2CWrite(CurrentSensorAddress, CurrentADCConfigReg, CurrentADCConfigData); //set up ADC for 8 bit resolution.  This is done for speed
	I2CWrite(CurrentSensorAddress, CurrentCtrlReg, CurrentCtrlData); //Set it up for continuous mode
}

bool CutDownPressure(int32_t RawTemp, int32_t RawPressure) {
//	/*This is here because the altitude check on the Comms board was not working.  Instead we are going to base it off of time elapsed as well as pressure.
//	This function is a boolean function that will say if it's time to cut down or not.  It is going to be compared against 187hPa, which is the pressure at
//	40,000ft above sea level.  This was Dr. Patrus request, to cut down if this is true, and 3 hours have passed.
//	*/

	//The code for calculating the pressure values was taken out of the sensor datasheet.  Part number BME280
	const int CutoffPressure = 187;
	int32_t var3, var4, t_fine;
	int64_t var1, var2, Press;
	const int32_t T1 = TCal1, T2 = TCal2, T3 = PCal3;
	const int64_t Num1 = 1, P1 = PCal1, P2 = PCal2, P3 = PCal3, P4 = PCal4, P5 = PCal5, P6 = PCal6, P7 = PCal7, P8 = PCal8, P9 = PCal9;
	uint32_t p;
	float Pressure;

	var3 = ((((RawTemp >> 3) - (T1 << 1)))*(T2)) >> 11;
	var4 = (((((RawTemp >> 4) - (T1))*((RawTemp >> 4) - (T1))) >> 12)*(T3)) >> 14;
	t_fine = var3 + var4;

	var1 = (t_fine)-128000;
	var2 = var1 * var1 * P6;
	var2 = var2 + ((var1*P5) << 17);
	var2 = var2 + ((P4) << 35);
	var1 = ((var1*var1*P3) >> 8) + ((var1*P2) << 12);
	var1 = ((((Num1) << 47) + var1))*(P1) >> 33;
	if (var1 == 0) { //Avoid divide by 0 exception
		return false;
	}
	Press = 1048576 - RawPressure;
	Press = (((Press << 31) - var2) * 3125) / var1;
	var1 = ((P9)*(Press >> 13)*(Press >> 13)) >> 25;
	var2 = ((P8)*Press) >> 19;
	Press = ((Press + var1 + var2) >> 8) + ((P7) << 4);
	p = Press;
	Pressure = (p / 256)*.01;

	if (Pressure < CutoffPressure) {
		return true; //Above 40,000 feet
	}

	else {
		return false; //Below 40,000 feet
	}

}//End pressure cutdown function

void I2CWrite(byte I2CAddress, byte I2CRegister, byte TxData) {
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

void I2CRead(byte NumBytesRx, byte I2CAddress, byte I2CRegister) {
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