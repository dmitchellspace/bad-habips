// 
// 
// 

#include "SPI.h"
void Init_SPI() {
	//Initliaze SPI for Comm with Host board. Teensy is a slave.
	/*
	MOSI Pin 0
	MISO Pin 1
	SCLK Pin 32
	*/

	//Initiliaze SPI for Comm with Motor controller.  Teensy is master.
	/*
	MOSI Pin 44
	MISO Pin 45
	SCLK Pin 46
	SS Pin 43
	*/
}