// 
// 
// 

#include "SPI.h"

void Init_SPI() {//Initiliaze SPI interface
	Init_DAQCS_SPI(); //Set up SPI to act as slave with main MSP430
	Init_IMU_SPI(); //Set up SPI to act as master with IMU
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
	SPI1_MCR = 0x000; // Disable the Module
	SPI1_CTAR0_SLAVE = 0x3E000000; // Frame size of 8 (writes a 7 to the register), Inactive state of SCK is high, 
								  // Data is changed on leading edge of SCK and captured on trailing edge
	SPI1_SR = 0xDA0A0000; // Clears all the FIFOs and flags
	SPI1_MCR = 0xC00; // Sets up SPI1 as a slave device.  Clears the buffer
	Serial.println("DAQCS SPI Bus Successfully Initialized");
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
	PORTC_PCR6 |= 0x200; //MOSI
	pinMode(12, INPUT);
	PORTC_PCR7 |= 0x200; //MISO
	pinMode(9, OUTPUT); //IMU0 CS
	digitalWrite(9, HIGH); // Initialize it as off.
	pinMode(14, OUTPUT);
	PORTD_PCR1 |= 0x200; //SCK
	pinMode(26, OUTPUT);
	PORTA_PCR14 |= 0x200; //IMU1 CS0

	SIM_SCGC6 |= 0x1000; //Enable the clock
	SPI0_MCR = 0x000; // Disable the Module
	SPI0_SR = 0x00; //Clear the status register
	SPI0_CTAR1 = 0x3E140112; //Set up Clock dividers, Set frame size as 8 bits 
	SPI0_SR = 0xDA0A0000; //Clear all flags in the SR
	SPI0_MCR = 0xC0210C00;  //Set Up SPI0 in master mode
	Serial.println("IMU SPI Bus Successfully Initialized");
}
