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
Version 9:
	Uploaded on 02/07/2018
	Added functions to turn motor on/off
	Started reaction wheel algorithim.  Only going to do LSB conversions for Z-Gyro when the reaction wheel is running.  Otherwise its using recources that we don't
	need to use.  Data is captured and then converted to rev/min.
	Took out the two's compliment function.  Now when the data comes back from the IMU it's being placed into a 16 bit short instead of a 32 bit int.  Because it's in 
	the 16 bit short the compiler recognizes it as a negative number, which makes no need for two's compliment.
	A button will be used in order to determine if a calibration of the IMU is to be done.  This button needs to be held down on startup in order for the calibration
	to take place.  This is done so that a calibration does not get accidentally triggered during the flight.
	Calibration is stored in flash memory.  If the button is not pressed it does not perform the calibration
	The calibration is only done for the gyroscope.
	The motor is working about 90% of the way.  The last issue is that the integral won't totally settle to 0 after it is spun fast in one direction and then not at all 
	in the other direction.  This should fix itself once it is connected to the actual motor.  This is because it will spin it slightly in one direction and that will 
	drive the integral to zero.  This will be tested once we can mount everything together.
Version 10:
	Uploaded on 02/10/2018
	In order to make the motor controller algorithim run as efficiently as possible, the manner in which SD card writes is done is going to be chagned (only for while
	the motor is on).  The opening and closing of the file structure is what takes the longest amount of time, and therefor this will only be done when neccessary.  The
	reason that closing it takes a while is because it clears the buffer on a close.  The buffer is 512 characters long and so it is not neccessary to do this every time.
	As a result the file will be opened once upon entering the loop, closed once upon leaving the loop, and the buffer will be cleared after all six of the IMU writes,
	instead of after every individual write.  This should make each write take about 200us instead of about 3ms.
	Added SD Card Writes while the motor is on.
	Added code so that it now stores two sets of IMU calibration data in memory.  The first 3 pieces are IMU0 and the second three are IMU1.
	Changed the functions in I2C because now we're using a different sensor that has both the temperature and pressure in one sensor.
	I2C is going to be completely rewritten not using the Arduino libraries.  Those libraries have too many restrictions, and therefor can not be utilizied for what we need.
	This will be part of the next version.
Version 11:
	Uploaded on 02/15/2018
	Added initialization for I2C
	Added Write Function for I2C
	Added Read Function for I2C
	The come up for the sensor is the following process:
	1) Initialize pins
	2) Read ID register, and mark sensor as pass/fail
	3) If pass write to config registers
	This functionality has been tested and confirmed.
Version 12:
	Uploaded on 02/16/2018
	Added temp/pressure read every second
	Added temperature calibration data read on startup
	Added pressure calibration read on startup
	The writes to the SD card were added.
	The writes for temperature and pressure sensors take a really long time due to their size.  As a result the motor on method is used in order to keep it in a
	reasonable time window. The card is opened once, both values are written, and then the SD card is closed.
	Read/Write are also added for when motor is on.
Version 13:
	Uploaded on 02/21/2018
	A loop was added so that the motor algorithim is run 8 times in between reads and writes.  This includes a 10ms delay.  This delay can be taken back out if other functionality
	needs to be added.  This keeps it so that the IMU is only being written to the SD card once every 70ms, which is the desired time.  This will make the motor more precise.
	In order to read from all nine battery cells both ADC modules will be used.  The initialization was added to intialize both of them.  The isr was set up for 
	the second ADC module, and a switch statement is used in both of the isrs.  This is used so that this can be run in the background while other data is being processed.
	Added SD Card write of this data once per second.
	Added function that checks the level of the batteries.  If they fall below 3.3V the battery is shut down.  If it's the main battery, the backup is turned on and the
	main is turned off.  If it's the motor battery then the motor is simply disabled.
	Once per second the data is written to the SD card.  When this write occurs it kicks off the next series of conversions.  These conversions happen in the background
	(they're controlled by the isr).  This sequence should only take a few milliseconds, but it happens in the background while other processing is happening in the
	foreground.
	The functionality of the ADC has been tested and verified.
Version 14:
	Uploaded on 02/23/2018
	A series of SD card files were added.  Because they need to be of type const char* they were hardcoded in an array.  It is set up so that a new file can be added every
	5 minutes for 4 hours.  After it adds the last file it will just continue to write to this file.
	The file number that we are on is written to non-volitiale memory.  This is done so that if we lose power we don't overwrite the data that is already written
	The RTC is checked on startup to see if it is already initialized.  This is done in case the teensy loses power but the RTC does not.  If this is the case the old 
	RTC values are used.
	*/

//Start Variable Declaration

int BootTime;
bool PressTempFlag = 0; //The pressure and temperature only needs to be collected once a second, and so a timer will be set up to indicate when it is time to check

//This data is used to send the address to the IMU
const byte XAccelAddress = 0x29, YAccelAddress = 0x2B, ZAccelAddress = 0x2D, XGyroAddress = 0x19, YGyroAddress = 0x1B, ZGyroAddress = 0x1D, IMU0 = 0, IMU1 = 1;
int XAccelData, YAccelData, ZAccelData, XGyroData, YGyroData, ZGyroData;
short XCalibrationData, YCalibrationData, ZCalibrationData;
int XCalibrationMemoryLocation = 0, YCalibrationMemoryLocation = 2, ZCalibrationMemoryLocation = 4; //Each takes two bytes of memory, these locations plus six will be for IMU1

//This is used for Temperature and Pressure
int PressureData, TemperatureData;

//This is for the SD Card writes
const byte Timestamp = 1, NoTimestamp = 0, SPI0Timeout = 0;
const int NewSDFileNumSeconds = 300; //5 minutes for a new file
int CurrentNumSeconds = 0; // Current time until next SD File

//Variables for motor
const byte CCW = 0, CW = 1;
const int NumMotorWrites = 6;

//End Variable Declaration

#include "Clocks.h"
#include "GPIO.h"
#include "I2C.h"
#include "SPI.h"
#include "SDCard.h"
#include "MotorControl.h"
#include <EEPROM.h>

void setup() { //Only runs once upon powering up the board

	delay(1000); //Put in a 1 secon delay so that everything has time to come up.
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
	Init_I2C();//Initiliaze I2C interface
	Init_SPI();//Init\iliaze SPI interface
	Init_MotorInterface(); //Initliaze Motor Interface
	Init1SecTimer(); //Init Timer to trigger an interrupt every 1 second

	if (!digitalRead(PIN_A4)) { //This checks the IMU calibration button.  THE BUTTON NEEDS TO BE HELD ON STARTUP IN ORDER FOR THE CALIBRATION TO TAKE PLACE
		XCalibrationData = XCalibration(); //Perform a cal
		YCalibrationData = YCalibration(); //Perform a cal
		
		ZCalibrationData = ZCalibration(); //Perform a cal
		
		if (IMUSelect == IMU0) { //Write to the first block of memory
			CalibrationDataWrite(XCalibrationData, XCalibrationMemoryLocation); //Write the data to flash
			CalibrationDataWrite(YCalibrationData, YCalibrationMemoryLocation); //Write the data to flash
			CalibrationDataWrite(ZCalibrationData, ZCalibrationMemoryLocation); //Write the data to flash
			Serial.println("IMU0 Calibration Performed");
		}
		else {
			CalibrationDataWrite(XCalibrationData, XCalibrationMemoryLocation+6); //Write the data to flash
			CalibrationDataWrite(YCalibrationData, YCalibrationMemoryLocation+6); //Write the data to flash
			CalibrationDataWrite(ZCalibrationData, ZCalibrationMemoryLocation+6); //Write the data to flash
			Serial.println("IMU1 Calibration Performed");
		}
		Serial.print("XCal = ");
		Serial.println(XCalibrationData);
		Serial.print("YCal = ");
		Serial.println(YCalibrationData);
		Serial.print("ZCal = ");
		Serial.println(ZCalibrationData);
	}

	else {
		if (IMUSelect == IMU0) {
			XCalibrationData = CalibrationDataRead(XCalibrationMemoryLocation); // Get X from flash
			YCalibrationData = CalibrationDataRead(YCalibrationMemoryLocation); //Get Y from flash
			ZCalibrationData = CalibrationDataRead(ZCalibrationMemoryLocation); //Get Z from flash
			Serial.println("No Calibration Performed.  IMU0 taken from Memory");
		}
		else {
			XCalibrationData = CalibrationDataRead(XCalibrationMemoryLocation+6); // Get X from flash
			YCalibrationData = CalibrationDataRead(YCalibrationMemoryLocation+6); //Get Y from flash
			ZCalibrationData = CalibrationDataRead(ZCalibrationMemoryLocation+6); //Get Z from flash
			Serial.println("No Calibration Performed.  IMU1 taken from Memory");
		}
	}

	Serial.print("Initialize Time = "); //Display Time it took to initilize
	Serial.print(millis() - BootTime); //Display Time it took to initilize
	Serial.println("ms");//Time is in milliseconds

}

// the loop function runs over and over again until power down or reset
void loop() {//Main Loop
	if (!digitalRead(PIN_A6)) {//Check to see if motor should be turned on
		CollectData();
	}
	else { //Motor should be on
		ReactionWheelOn();
	}
}

void CollectData() {

	/*
	None of the LSB conversions will be done on board.  This is an unnecessary use of resources when these calculations can be done on the ground.
	The only LSB conversion that will be done, is the Z-Gyro and that will only be done when the motor is turned on, because that is necessary for the motor
	algorithim, it will be done on board.
	The writes for temperature and pressure sensors take a really long time due to their size.  As a result the motor on method is used in order to keep it in a 
	reasonable time window. The card is opened once, both values are written, and then the SD card is closed.
	*/
	while (!digitalRead(PIN_A6)) { //Stay in this loop until Reaction Wheel is turned on

		 if (SDCardPresent == 0) { //SD Card is not present
			SDCard_Setup(); //Try to set it up again
			for (int i = 1; i < 7;i++) {//Blink the LED Three Quick Times to let the user know SD Card is not working
				GPIOC_PTOR ^= 0x20;
				delay(200);
			}
		}
		else {//SD Card is there, store data
			XAccelData = IMURead(XAccelAddress, 0, IMUSelect); //Collect X Accel
			SDCard_Write(XAccelData, Timestamp); //Write it to the SD Card

			YAccelData = IMURead(YAccelAddress, 0, IMUSelect); //Collect Y Accel
			SDCard_Write(YAccelData, NoTimestamp); //Write it to the SD Card
			
			ZAccelData = IMURead(ZAccelAddress, 0, IMUSelect); //Collect Z Accel
			SDCard_Write(ZAccelData, NoTimestamp); //Write it to the SD Card
			
			XGyroData = IMURead(XGyroAddress, 0, IMUSelect) - XCalibrationData; //Collect X Gyro
			SDCard_Write(XGyroData, NoTimestamp); //Write it to the SD Card
			
			YGyroData = IMURead(YGyroAddress, 0, IMUSelect) - YCalibrationData; //Collect Y Gyro
			SDCard_Write(YGyroData, NoTimestamp); //Write it to the SD Card
			
			ZGyroData = IMURead(ZGyroAddress, 0, IMUSelect) - ZCalibrationData; //Collect Z Gyro
			SDCard_Write(ZGyroData, NoTimestamp); //Write it to the SD Card

			if (PressTempFlag) { //Has one second gone by since the last Temp/Press Measurement?
				/*The temperature and pressure measurements are performed as six straight reads.  The first is the MSB of pressure, the
				second is the LSB of the pressure.  The third is four addition least significant bits.  The 4th-6th registers are used 
				the same way but for temperature.
				*/
				SDCardOpenFile();
				PressTempFlag = 0; //Reset Flag
				Heartbeat(); //1 second heartbeat

				TempPressureRead(TempPressureNumRegisters, TempPressureAddress, TempPressureStartRegister);

				TemperatureData = I2CRxData[2];
				TemperatureData = ((TemperatureData << 8) & 0xFF00) | I2CRxData[1];
				TemperatureData = ((TemperatureData << 4) & 0xFFFF0) | ((I2CRxData[0] >> 4) & 0x0F);
				SDCard_WriteMotorOn(TemperatureData, NoTimestamp);

				PressureData = I2CRxData[5];
				PressureData = ((PressureData << 8) & 0xFF00) | I2CRxData[4];
				PressureData = ((PressureData << 4) & 0xFFFF0) | ((I2CRxData[3] >> 4) & 0x0F);
				SDCard_WriteMotorOn(PressureData, NoTimestamp);

				if ((ADC0_Select > 2) && (ADC1_Select > 5)) { //Write the ADC Data if it's done
					
					for (int counter = 0; counter < 9; counter++) {
						SDCard_WriteMotorOn(ADCData[counter], NoTimestamp); //Write all of the ADC Data
					}
					CheckBatteryLevel(); //Check if the battery needs to be shut down
					ADC0_Select = 0; //Reset the cycle
					ADC1_Select = 0; //Reset the cycle
					BeginADCConversion(); //Kick off the conversions
				}

				SDCardCloseFile();
			} //End Pressure Temperature if
				SDCard_NewLine();

			if (DataNotValidSPI0 == 1) { //If SPI0 Timeout Occured
				Init_IMU_SPI(); //Reset SPI0
				IMUSelfTest(); //Test Them
				SDCard_NewLine(); //Enter New Line
				SDCard_SensorFailure(SPI0Timeout); //Print Error Message
			}

			if (CurrentNumSeconds >= NewSDFileNumSeconds) {
				NewSDFile(); //Create a new file
				CurrentNumSeconds = 0; //Restart the counter
			}

		} //End SD card else
	} //End While statement
} //End collect Data

void ReactionWheelOn() {
	/*
	In order to make the motor controller algorithim run as efficiently as possible, the manner in which SD card writes is done is going to be chagned (only for while
	the motor is on).  The opening and closing of the file structure is what takes the longest amount of time, and therefor this will only be done when neccessary.  The
	reason that closing it takes a while is because it clears the buffer on a close.  The buffer is 512 characters long and so it is not neccessary to do this every time.
	As a result the file will be opened once upon entering the loop, closed once upon leaving the loop, and the buffer will be cleared after all six of the IMU writes,
	instead of after every individual write.  This should make each write take about 200us instead of about 3ms.
	*/
	double ZGyro_RawData, ZGyro_RPM = 0, Error, Integral = 0, Control_Speed;
	double kp = 120; //For algorithim
	double ki = -0.2; //For algorithim
	const double GyroLSB = 0.061035156; //(2000/2^15): Range of data is +/-2000deg/sec.

	SDCardOpenFile(); //Open the file for the duration of the motor being turned on
	TurnMotorOn(); //Enable the motor

	while (digitalRead(PIN_A6)) {//Stay in this loop until Reaction Wheel is turned on
		XAccelData = IMURead(XAccelAddress, 0, IMUSelect); //Collect X Accel
		SDCard_WriteMotorOn(XAccelData, Timestamp); //Write it to the SD Card

		YAccelData = IMURead(YAccelAddress, 0, IMUSelect); //Collect Y Accel
		SDCard_WriteMotorOn(YAccelData, NoTimestamp); //Write it to the SD Card

		ZAccelData = IMURead(ZAccelAddress, 0, IMUSelect); //Collect Z Accel
		SDCard_WriteMotorOn(ZAccelData, NoTimestamp); //Write it to the SD Card

		XGyroData = IMURead(XGyroAddress, 0, IMUSelect) - XCalibrationData; //Collect X Gyro
		SDCard_WriteMotorOn(XGyroData, NoTimestamp); //Write it to the SD Card

		YGyroData = IMURead(YGyroAddress, 0, IMUSelect) - YCalibrationData; //Collect Y Gyro
		SDCard_WriteMotorOn(YGyroData, NoTimestamp); //Write it to the SD Card

		ZGyroData = IMURead(ZGyroAddress, 0, IMUSelect) - ZCalibrationData; //Collect Z Gyro
		SDCard_WriteMotorOn(ZGyroData, NoTimestamp); //Write it to the SD Card

		if (PressTempFlag) { //Has one second gone by since the last Temp/Press Measurement?
			PressTempFlag = 0; //Reset Flag
			Heartbeat(); //1 second heartbeat

			TempPressureRead(TempPressureNumRegisters, TempPressureAddress, TempPressureStartRegister); //Read Temp/Press

			TemperatureData = I2CRxData[2];
			TemperatureData = ((TemperatureData << 8) & 0xFF00) | I2CRxData[1];
			TemperatureData = ((TemperatureData << 4) & 0xFFFF0) | ((I2CRxData[0] >> 4) & 0x0F);
			SDCard_WriteMotorOn(TemperatureData, NoTimestamp); //Write to card

			PressureData = I2CRxData[5];
			PressureData = ((PressureData << 8) & 0xFF00) | I2CRxData[4];
			PressureData = ((PressureData << 4) & 0xFFFF0) | ((I2CRxData[3] >> 4) & 0x0F);
			SDCard_WriteMotorOn(PressureData, NoTimestamp); //Write to card

			if ((ADC0_Select > 2) && (ADC1_Select > 5)) { //Write the ADC Data if it's done

				for (int counter = 0; counter < 9; counter++) {
					SDCard_WriteMotorOn(ADCData[counter], NoTimestamp); //Write all of the ADC Data
				}

				CheckBatteryLevel(); //Check if the battery needs to be shut down
				ADC0_Select = 0; //Reset the cycle
				ADC1_Select = 0; //Reset the cycle
				BeginADCConversion(); //Kick off the conversions
			}

		} //End Pressure Temperature if

		if (CurrentNumSeconds >= NewSDFileNumSeconds) {
			SDCardCloseFile(); //Close the previous file
			NewSDFile(); //Create a new file
			CurrentNumSeconds = 0; //Restart the counter
			SDCardOpenFile(); //Open the new file
		}

		SDCard_NewLineMotorOn(); //Enter a new line
		SDCard_FlushBuffer(); //Flush the buffer
		
		//for (int counter = 0; counter <= NumMotorWrites; counter++) { //run the motor algorithim NumMotorWrites times and then read more data
			ZGyro_RawData = (ZGyroData * GyroLSB) / 6; //The divide by 6 does the conversion from deg/sec to rev/minute (deg/sec * 60/360 = rev/min)
			ZGyro_RPM = EMA(ZGyro_RawData, ZGyro_RPM); //Calculate the running average of the data.  This will be used to run the reaction wheel

			if ((ZGyro_RPM < 0.05) && (ZGyro_RPM > -0.05)) {
				ZGyro_RPM = 0; //This is a filter on the data so it doesn't spin when there is nothing going on.  It filters it for +/-0.05rev/min
			}

			Error = -ZGyro_RPM; //The '-' is for orientation
			Integral = Integral + Error; //Keep adding to the integral.  This is the very literal of the definition of an integral, being a running sum
			Control_Speed = (kp * Error) + (ki * Integral); //Take a weighted sum of the two.

			if (Control_Speed > 0.2) {
				Motor_Direction(CCW); //Turn it on CCW
				MotorSpeed(Control_Speed); //Turn it on at the desired speed
			}

			else if (Control_Speed < -0.2) {
				Motor_Direction(CW); //Turn it on CW
				MotorSpeed((-Control_Speed)); //Turn it on to the desired speed
			}

			else {
				NoMotorSpeed(); //Speed is = 0
			}
			delay(10);
		//}//End for loop

	}//End While

	TurnMotorOff(); //Disable the motor
	SDCardCloseFile(); //Close the SD Card File

} //End ReactionWheelOn

void Heartbeat() {
	GPIOC_PTOR ^= 0x20; //Toggle On board LED (Pin C5).
	GPIOA_PTOR ^= 0x20; //Toggle Blue LED (Pin A5).
}

short XCalibration() {
	int CalData;
	CalData = IMURead(XGyroAddress, 0, IMUSelect);
	CalData = 0; //The first piece of data on startup is ofter corrupted
	for (int counter = 0; counter < 5; counter++) {
		CalData += IMURead(XGyroAddress, 0, IMUSelect); //Running Sum
	}
	return CalData / 5; //Take the average
}

short YCalibration() {
	int CalData;
	CalData = IMURead(YGyroAddress, 0, IMUSelect);
	CalData = 0; //The first piece of data on startup is ofter corrupted
	for (int counter = 0; counter < 5; counter++) {
		CalData += IMURead(YGyroAddress, 0, IMUSelect); //Running Sum
	}
	return CalData / 5; //Take the average
}

short ZCalibration() {
	int CalData;
	CalData = IMURead(ZGyroAddress, 0, IMUSelect);
	CalData = 0; //The first piece of data on startup is ofter corrupted
	for (int counter = 0; counter < 5; counter++) {
		CalData += IMURead(ZGyroAddress, 0, IMUSelect); //Running Sum
	}
	return CalData = CalData / 5; //Take the average
}

void CalibrationDataWrite(short Data, int Address) { //This is used to write the calibration data to the flash memory
	byte TempData; //The Data needs to be written in bytes
	TempData = Data; //Get the first byte
	EEPROM.write(Address, TempData); //Write the first byte
	Data = ((Data & 0x00FF) << 8) | ((Data & 0xFF00) >> 8); //This is a byte swap
	TempData = Data; //Get the second byte
	Address++; //Increment the address
	EEPROM.write(Address, TempData); //Write the second byte
}

short CalibrationDataRead(int Address) {
	short Data, TempData;
		Address++; //MSB First
		Data = EEPROM.read(Address); //Read it 
		Data = Data << 8; //Bit shift
		Address--; //LSB Address
		TempData = EEPROM.read(Address); //Read it
		Data = (Data & 0xFF00) | (TempData & 0x00FF); //Combine them
		return Data;
}

 void ftm0_isr() { //1 Second Timer
	FTM0_SC &= ~0x80; //Clear the flag
	PressTempFlag = 1;
	if (FileNumber < 48) { //If its the last file you dont want to increment this anymore
		CurrentNumSeconds++;
	}
}