/*
Name:		TeensyMotorControl_PsuedoCode.ino
Created:	11/14/2017 7:12:13 PM
Brandon Codi
Code for Teensy Motor Controller on HABIP
Version 1:
	Uploaded on 11/20/17
	Psuedo Code
	GPIO Setup
	Heartbeat
Version 2:
	Uploaded on 11/22/2017
	RTC Setup
	SD Card Initlization
	Simple Self Test run on RTC and SD Card, Results printed to Comm Port
	If SD Card Fails to Initilize, it will continue to retry in the main loop.
Version 3:
	Uploaded on 12/01/2017
	LED is blinked three times quickly to show user SD card is not working.
	I2C Initiliazation is setup.  Verified correct address and R/W commands were being TXd
	Without any of the components we cannot verify it is working, because we don't get an 
	acknowledge byte back.  A RX interrupt still needs to be setup.
	Pins for motor control are setup.
	PWM is setup and verified.  PWM signal will be running at 1kHz as opposed to 50 Hz from last semester.
Version 4:
	Uploaded on 12/06/2017
	ADC Initialization was implemented and successfully tested.
	A calibration of the unit is performed and the Successfull/Not Successful is determined based off of the cal results.
	The ISR has not yet been integrated.
	Frequency of PWM signal was changed to 75Hz.  This allows for more varience in the PW.
	SPI has been initialized.  Nothing has been tested yet, this still needs to be done.  Interrupts need to be set up.
Version 5:
	Uploaded on 01/23/2018
	ADC Interrupt Setup
	SPI transmitting to IMU verified.
	Self Test added where SPI will write to IMU "WHO_AM_I" register. If the correct response is heard back, it will mark it as a pass.
	Right now requests are being made for the IMU data, but nothing is being recorded.
	The IMU read function is set up to read from either of the two chips.  The chip select is an input to the function.
	The IMU that we currently have was wired as an IMU0 and then as an IMU1.  It was verified that we are able to talk to it in either configuration.
Version 6:
	Uploaded on 01/26/2018
	Added timeout on SPI transactions.  1ms timeout
	Right now I'm setting a flag that says a timeout happened, but not actually doing anything with that flag.  This functionality needs to be added.
	Added Data Write function for SPI0
	Data coming back from the IMU is not being handled right now.  Matt C. is going to look into how the data needs to be handled.  This will probably be a couple versions from now.
	It is verified that everything on the IMU is being enabled and that the data coming back changes once the IMU is moved around.
Version 7:
	Uploaded on 01/29/2018
	Added function to get the current time from the RTC.  It populates an array as [sec, min, hr, day, month, year]
	Added Data Write function to the SD Card
	Added New line write function to the data card.
	Right now I have it working where it prints all of the data on one line with one timestamp for all 6 IMU data pieces.  
	Each set of 6 reads/write takes approximately 60ms, so one time stamp is fine for this.
	Add timer to trigger an interrupt every second.  This says that its time to get the pressure and temp functions.  Inside this loop the Heartbeat function is called which flashes the LED.
Version 8:
	Uploaded on 01/30/2018
	Made DataNotValid a global variable so that it can be checked anywhere.
	Made SD Card file comma seperated
	Added IMU Select.  If IMU0 passes initialization it will gather data from that.  If it fails initialization it will gather data from IMU1.
	Added check for SPI0 timeout.  It will print an error message on the SD card as well as reinitialize the SPI bus.
	Added Twos Compliment calculation for all the IMU data.  With the two's compliment done, the reads/writes take 20ms.
	Change file type on SD Card to .csv
	*/

//Start Variable Declaration

int BootTime;
byte PressTempFlag = 0; //The pressure and temperature only needs to be collected once a second, and so a timer will be set up to indicate when it is time to check

//This data is used to send the address to the IMU
const byte XAccelAddress = 0x29, YAccelAddress = 0x2B, ZAccelAddress = 0x2D, XGyroAddress = 0x19, YGyroAddress = 0x1B, ZGyroAddress = 0x1D, IMU0 = 0, IMU1 = 1;
int XAccelData, YAccelData, ZAccelData, XGyroData, YGyroData, ZGyroData;

//This is for the SD Card writes
const byte Timestamp = 1, NoTimestamp = 0, SPI0Timeout = 0;
//End Variable Declaration


#include "Clocks.h"
#include "GPIO.h"
#include "I2C.h"
#include "SPI.h"
#include "SDCard.h"
#include "MotorControl.h"

void setup() { //Only runs once upon powering up the board

	Serial.begin(9600); //Set Up Serial Interface
	//DEBUG
	while (!Serial); // DEBUG DEBUG DEBUG THE PROGRAM WILL NOT START UNTIL THE SERIAL COMM PORT 
	//RESPONDS MAKE SURE TO TAKE OUT
	//DEBUG
	BootTime = millis();
	ADC_Calibration(); //Caibration takes some time to complete, so initiate it before anything else is done
	Init_GPIO(); //Do initilization on GPIO Pins
	Init_RTC(); //Initiliaze Real Time Clock
	SDCard_Setup();//Do initial setup for SD Card
	Init_SPI();//Init\iliaze SPI interface
	Init_I2C();//Initiliaze I2C interface
	Init_MotorInterface(); //Initliaze Motor Interface
	Init1SecTimer(); //Init Timer to trigger an interrupt every 1 second

	Serial.print("Initialize Time = "); //Display Time it took to initilize
	Serial.print(millis() - BootTime); //Display Time it took to initilize
	Serial.println("ms");//Time is in milliseconds

}

// the loop function runs over and over again until power down or reset
void loop() {//Main Loop
	if (!digitalRead(PIN_A3)) {//Check to see if motor should be turned on
		CollectData();
	}
	else { //Motor should be on
		ReactionWheelOn();
	}
}

void CollectData() {
	while (!digitalRead(PIN_A3)) { //Stay in this loop until Reaction Wheel is turned on

		 if (SDCardPresent == 0) { //SD Card is not present
			SDCard_Setup(); //Try to set it up again
			for (int i = 1; i < 7;i++) {//Blink the LED Three Quick Times to let the user know SD Card is not working
				GPIOC_PTOR ^= 0x20;
				delay(200);
			}
		}
		else {//SD Card is there, store data
			
			XAccelData = IMURead(XAccelAddress, 0, IMUSelect); //Collect X Accel
			XAccelData = TwosCompliment(XAccelData); //Two's compliment
			SDCard_Write(XAccelData, Timestamp); //Write it to the SD Card

			YAccelData = IMURead(YAccelAddress, 0, IMUSelect); //Collect Y Accel
			YAccelData = TwosCompliment(YAccelData); //Two's compliment
			SDCard_Write(YAccelData, NoTimestamp); //Write it to the SD Card
			
			ZAccelData = IMURead(ZAccelAddress, 0, IMUSelect); //Collect Z Accel
			ZAccelData = TwosCompliment(ZAccelData); //Two's compliment
			SDCard_Write(ZAccelData, NoTimestamp); //Write it to the SD Card
			
			XGyroData = IMURead(XGyroAddress, 0, IMUSelect); //Collect X Gyro
			XGyroData = TwosCompliment(XGyroData); //Two's compliment
			SDCard_Write(XGyroData, NoTimestamp); //Write it to the SD Card
			
			YGyroData = IMURead(YGyroAddress, 0, IMUSelect); //Collect Y Gyro
			YGyroData = TwosCompliment(YGyroData); //Two's compliment
			SDCard_Write(YGyroData, NoTimestamp); //Write it to the SD Card
			
			ZGyroData = IMURead(ZGyroAddress, 0, IMUSelect); //Collect Z Gyro
			ZGyroData = TwosCompliment(ZGyroData); //Two's compliment
			SDCard_Write(ZGyroData, NoTimestamp); //Write it to the SD Card

			if (PressTempFlag == 1) { //Has one second gone by since the last Temp/Press Measurement?
				PressTempFlag = 0; //Reset Flag
				Heartbeat(); //1 second heartbeat
				//TODO SD Card Writes
				SDCard_NewLine(); //New Line
			} //End Pressure Temperature if
			else {
				SDCard_NewLine();
			}
			
			if (DataNotValidSPI0 == 1) { //If SPI0 Timeout Occured
				Init_IMU_SPI(); //Reset SPI0
				IMUSelfTest(); //Test Them
				SDCard_NewLine(); //Enter New Line
				SDCard_SensorFailure(SPI0Timeout); //Print Error Message
			}

		} //End SD card else
	} //End While statement
} //End collect Data

void ReactionWheelOn() {
	GPIOC_PTOR ^= 0x20; //Toggle On board LED (Pin C5).
	GPIOA_PTOR ^= 0x20; //Toggle Blue LED (Pin A5).

	//DEBUG
	delay(1000);  // wait for a second
	//DEBUG

	while (digitalRead(PIN_A3)) {//Stay in this loop until Reaction Wheel is turned off
		//Run Reaction Wheel Algorithim
	}
}

int TwosCompliment(int Argument) {//This does the two's compliment conversion of a 16 bit number
	int Solution;
	if (0x8000 & Argument) { //Is it a negative number?
		Solution = Argument ^ 0xFFFF; //Flip all the bits
		Solution++; //Add one
		Solution = Solution * -1; //Make it negative
	}
	else { //It it not
		Solution = Argument;
	}
	return Solution;
}

void Heartbeat() {
	GPIOC_PTOR ^= 0x20; //Toggle On board LED (Pin C5).
	GPIOA_PTOR ^= 0x20; //Toggle Blue LED (Pin A5).
}

 void ftm0_isr() { //1 Second Timer
	FTM0_SC &= ~0x80; //Clear the flag
	PressTempFlag = 1;
}