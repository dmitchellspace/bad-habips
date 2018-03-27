// 
// 
// 

#include "SPI.h"
#include "GPIO.h"

const int IMU_ST_Result = 0x68; //This is what the ST data should return.
const byte IMU_ST_Address = 0xF, EnableGyroAddress = 0x1E, TurnOnGyroAddress = 0x10, EnableAccelAddress = 0x1F, TurnOnAccelAddress = 0x20, EnableGyroData = 0x38;
const byte TurnOnGyroData = 0xD8, EnableAccelData = 0x38, TurnOnAccelData = 0xD0, IMU0 = 0, IMU1 = 1, XGyroAddress = 0x19, YGyroAddress = 0x1B, ZGyroAddress = 0x1D;
int IMU_ST_Data;
byte SPI0RxData, SPI0CompleteFlag = 0, DataNotValidSPI0, IMUSelect; //This is all used for SPI0
byte SPI1RxByteCount; //This counts the number of bytes that have been received.  It should be three bytes per transaction.  This is intialized up here so that it can be set to 0.
bool ResetSPI1Bus, TxSPI1Msg; //SPI1 Variables

void Init_SPI() {//Initiliaze SPI interface
	Init_DAQCS_SPI(); //Set up SPI to act as slave with main MSP430
	Init_IMU_SPI(); //Set up SPI to act as master with IMU
	IMUSelfTest(); //Run SelfTest
	//TODO Add Case for if both IMUs fail
}

void Init_DAQCS_SPI() { //Set up SPI to act as slave with main MSP430
	pinMode(31, INPUT);
	PORTB_PCR10 |= 0x200; //Set up Pin 31 as chip select
	pinMode(32, INPUT);
	PORTB_PCR11 |= 0x200; //Set up Pin 32 as CLK
	pinMode(0, OUTPUT);
	PORTB_PCR16 |= 0x200; //Set up Pin 0 as MISO
	pinMode(1, INPUT);
	PORTB_PCR17 |= 0x200; //Set up Pin 1 as MOSI

	SIM_SCGC6 |= 0x2000; //Enable clock to SPI 1
	SPI1_MCR = 0x001; // Disable the Module
	SPI1_SR = 0x000; //Clear the SR
	SPI1_MCR |= 0x3F0C00; // Sets up SPI1 as a slave device.  Clears the buffer
	SPI1_CTAR0 = 0x38000000; //You have to write it in both places, because of course you do...
	SPI1_CTAR0_SLAVE = 0x38000000; // Frame size of 8 (writes a 7 to the register), Inactive state of SCK is low, 
								  // Data is captured on leading edge of SCK and changed on trailing edge
	SPI1_RSER |= 0x80000000; //Enable interrupts on complete of transfer
	NVIC_ISER0 |= 0x8000000; //Enable interrupts
	SPI1_SR = 0xDA0A0000; // Clears all the FIFOs and flags
	SPI1_MCR |= 0x3F0000; // CS is active low
	SPI1_MCR &= ~0x0001; //Clear the Halt bit
	ResetSPI1Bus = 0; //Don't restart the bus
	TxSPI1Msg = 0; //Don't transmit the next cycle
	SPI1RxByteCount = 0; //You've Rx'd 0 bytes
	Serial.println("DAQCS SPI Bus Successfully Initialized");
	//TODO Have a failure case and set the Fault Matrix Entry
}

void Init_IMU_SPI() { //Set up SPI to act as master with IMU

	//We are using a GPIO as the chip select for the one IMU.  It manually needs to be toggled in order to do the 
	//transaction.  In order to keep the SPI unit happy the CS5 will be set as an output, and it will be set anytime
	//transaction needs to be done.  So CS5 will be set to keep the SPI module happy, but the actual chip select
	//will manually be toggled.


	//Set up all pins
	pinMode(29, OUTPUT); //CS_M on IMU set up as output
	digitalWrite(29, HIGH); //We don't use this so we set this as a high
	pinMode(27, OUTPUT); //CS_M on IMU set up as output
	digitalWrite(27, HIGH); //We don't use this so we set this as a high
	pinMode(11, OUTPUT);
	PORTC_PCR6 &= ~0x700; //Clear Mux bits
	PORTC_PCR6 |= 0x200; //MOSI
	pinMode(12, INPUT);
	PORTC_PCR7 &= ~0x700; //Clear Mux bits
	PORTC_PCR7 |= 0x200; //MISO
	pinMode(9, OUTPUT); //IMU0 CS
	digitalWrite(9, HIGH); // Initialize it as off.
	pinMode(14, OUTPUT);
	PORTD_PCR1 &= ~0x700; //Clear Mux bits
	PORTD_PCR1 |= 0x200; //SCK
	pinMode(26, OUTPUT);
	PORTA_PCR14 &= ~0x700; //Clear Mux bits
	PORTA_PCR14 |= 0x200; //IMU1 CS0

	SIM_SCGC6 |= 0x1000; //Enable the clock
	SPI0_MCR = 0x000; // Disable the Module
	SPI0_SR = 0x00; //Clear the status register
	SPI0_CTAR1 = 0x7F170112;  //Set up Clock dividers, Set frame size as 16 bits, LSB First
	SPI0_RSER |= 0x80000000; //Enables interrupts on completion of transmission
	NVIC_ISER0 |= 0x4000000; //Enables SPI0 Interrupt
	SPI0_SR = 0xDA0A0000; //Clear all flags in the SR
	SPI0_MCR = 0x803F0C00; //Set up SPI0 in Master Mode, Non Continuous Clock
	IMU_ST_Data = IMURead(IMU_ST_Address, 1, 1);

}

void IMUSelfTest() {

	IMU_ST_Data = IMURead(IMU_ST_Address, 1, 0); //Test IMU0

	if (IMU_ST_Data == IMU_ST_Result) {
		Serial.println("IMU 0 Selftest Passed");
	}
	else {
		Serial.println("IMU 0 Selftest Failed");
		Serial.println(IMU_ST_Data); //DEBUG
		FaultMatrix[5] = 1;
	}

	IMU_ST_Data = IMURead(IMU_ST_Address, 1, 1); //Test IMU1

	if (IMU_ST_Data == IMU_ST_Result) {
		Serial.println("IMU 1 Selftest Passed");
		IMUSelect = IMU1;
	}
	else {
		Serial.println("IMU 1 Selftest Failed");
		Serial.println(IMU_ST_Data);
		FaultMatrix[6] = 1;
		IMUSelect = IMU0;
	}

	IMUWrite(EnableAccelAddress, EnableAccelData, 0); //Enable Accel on IMU0
	IMUWrite(TurnOnAccelAddress, TurnOnAccelData, 0); //Enable Accel on IMU0, +/-4g range (LSB of 0.122mg)
	IMUWrite(EnableGyroAddress, EnableGyroData, 0); //Enable Gyro on IMU0
	IMUWrite(TurnOnGyroAddress, TurnOnGyroData, 0); //Enable Gyro on IMU0, make range +/-2000dps (LSB of 0.061deg/sec)
	IMUWrite(EnableAccelAddress, EnableAccelData, 1); //Enable Accel on IMU1
	IMUWrite(TurnOnAccelAddress, TurnOnAccelData, 1); //Enable Accel on IMU1, +/-4g range (LSB of 0.122mg)
	IMUWrite(EnableGyroAddress, EnableGyroData, 1); //Enalbe Gyro on IMU1
	IMUWrite(TurnOnGyroAddress, TurnOnGyroData, 1); //Enable Gyro on IMU1, make range +/-2000dps (LSB of 0.061deg/sec)
}

int IMURead(byte TxAddress, byte SingleReg, byte IMUNumber) {
	//Some of the data requires data from two registers while some data needs only one register.  The SingleReg input is a binary value saying if a second read is 
	//neccessary.  A 1 in this value means only one read is neccessary.
	byte TxData; //This might need to leave the function and become overarching. 
	int Timeout = 0; //Initialize to 0
	short SPI0Data = 0;
	DataNotValidSPI0 = 0; // Data not Valid defaults to data valid
	for (byte counter = 1; counter < 3; counter++) { //Need to do two words to read from the IMU.  See IMU Data sheet for detail
		TxData = (TxAddress & 0xF0) >> 4 | (TxAddress & 0x0F) << 4; //This reverses the order of the address
		TxData = (TxData & 0xCC) >> 2 | (TxData & 0x33) << 2; //This reverses the order of the address
		TxData = (TxData & 0xAA) >> 1 | (TxData & 0x55) << 1; //This reverses the order of the address
		TxData = TxData | 0x01; //Set LSB to 1 to signify read

		if (IMUNumber == 0) { //IMU 0
			digitalWrite(9, LOW); // Pull the CS Line.
			SPI0_PUSHR = 0x10000000 | TxData; //CTAR1 is used.
		}
		else { //IMU 1
			SPI0_PUSHR = 0x10010000 | TxData; //CTAR1 is used.  Bit 16 for CS0
		}
		
		Timeout = millis();
		TxAddress--; //Next Address

		while (SPI0CompleteFlag == 0) {//Wait until the transaction is complete
			delay(0);
			if (millis() - Timeout > 1){//Wait until Transaction is complete
				SPI0CompleteFlag = 1; //Trick it into leaving
				DataNotValidSPI0 = 1; //Set Data not valid flag.
			}//End If

		} //End while

		SPI0CompleteFlag = 0; //Clear the flag
		digitalWrite(9, HIGH); // Let go of CS Line.

		if (SingleReg == 1) {
			counter = 3; //exit the loop
			SPI0Data = SPI0RxData;		
		}
		else {
			if (counter == 1) {//First set of Data
				SPI0Data = SPI0RxData;
				SPI0Data = ((SPI0Data & 0x00FF) << 8) | ((SPI0Data & 0xFF00) >> 8); //This is a byte swap
			}
			else { //Second set of data
				SPI0Data = ((SPI0Data & 0xFF00) | (SPI0RxData&0x00FF)); //Keep the upper most byte and put in the second byte
			}
		}
	}//End for loop
	return SPI0Data;
}

void IMUWrite(byte TxAddress, byte TxData, byte IMUNumber) {
	int SPI0TxData = 0, SPI0TempAddress = 0, Timeout = 0; //Initialize to 0
		
	DataNotValidSPI0 = 0; //Defaults to data valid

		SPI0TempAddress = (TxAddress & 0xF0) >> 4 | (TxAddress & 0x0F) << 4; //This reverses the order of the address
		SPI0TempAddress = (SPI0TempAddress & 0xCC) >> 2 | (SPI0TempAddress & 0x33) << 2; //This reverses the order of the address
		SPI0TempAddress = (SPI0TempAddress & 0xAA) >> 1 | (SPI0TempAddress & 0x55) << 1; //This reverses the order of the address
		
		SPI0TxData = (TxData & 0xF0) >> 4 | (TxData & 0x0F) << 4; //This reverses the order of the data
		SPI0TxData = (SPI0TxData & 0xCC) >> 2 | (SPI0TxData & 0x33) << 2; //This reverses the order of the data
		SPI0TxData = (SPI0TxData & 0xAA) >> 1 | (SPI0TxData & 0x55) << 1; //This reverses the order of the data

		SPI0TxData = ((SPI0TxData & 0x00FF) << 8) | ((SPI0TxData & 0xFF00) >> 8); //This puts the data in the MSB
		SPI0TxData = (SPI0TxData & 0xFF00) | SPI0TempAddress; //Puts the address in the LSB

		if (IMUNumber == 0) { //IMU 0
			digitalWrite(9, LOW); // Pull the CS Line.
			SPI0_PUSHR = 0x10000000 | SPI0TxData; //CTAR1 is used.
		}
		else { //IMU 1
			SPI0_PUSHR = 0x10010000 | SPI0TxData; //CTAR1 is used.  Bit 16 for CS0
		}

		Timeout = millis();

		while (SPI0CompleteFlag == 0) {//Wait until the transaction is complete
			delay(0);
			if (millis() - Timeout > 1) {//Wait until Transaction is complete
				SPI0CompleteFlag = 1; //Trick it into leaving
				DataNotValidSPI0 = 1; //Set Data not valid flag.
			}//End If

		} //End while

		SPI0CompleteFlag = 0; //Clear the flag
		digitalWrite(9, HIGH); // Let go of CS Line.

}

void spi0_isr() {
	int TempData;
	SPI0_SR |= 0x80000000; //Clears bit to indicate interrupt
	SPI0CompleteFlag = 1; //Set the interrupt flag
	TempData = SPI0_POPR; //Pop the data from the FIFO
	TempData = ((TempData & 0x00FF) << 8) | ((TempData & 0xFF00) >> 8); //This is a byte swap
	SPI0RxData = TempData; //We only want the least significant byte
	SPI0RxData = (SPI0RxData & 0xF0) >> 4 | (SPI0RxData & 0x0F) << 4; //This reverses the order of the data
	SPI0RxData = (SPI0RxData & 0xCC) >> 2 | (SPI0RxData & 0x33) << 2; //This reverses the order of the data
	SPI0RxData = (SPI0RxData & 0xAA) >> 1 | (SPI0RxData & 0x55) << 1; //This reverses the order of the data
}