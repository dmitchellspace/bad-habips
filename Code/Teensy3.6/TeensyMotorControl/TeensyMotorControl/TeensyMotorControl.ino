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
Version 15:
	Uploaded on 02/27/2018
	Added a check where the user can press the button in order to restart the file system.  If the button is pressed before the LED starts to flash the calibration on the IMU
	will be performed.  If the button is pressed in the ~4second window while the LED is flashing rapidly the file system will restart at number 0.  Anytime after this window the
	button will not do anything.
	The file names were shortened from HABIP#.csv to H#.  This was done in order to save on RAM.
	Added a function to display the results of the Self test on the LEDs. A boolean array was added to store the results. The array is formatted as follows:
	[Current Sensor, ADC0, ADC1, DAQCS SPI, Temp/Press, IMU0, IMU1, SD Card]
	There are two kinds of failures defined, "Hard Failures", and "Soft Failures".  Hard failures make it such that the unit is not operational.  A soft failure means that
	something is not working, but the unit can fly anyway.  A hard failure is defined by two different scenarios:
	1) IMU0 and IMU1 Failures
	2) SD Card Failure
	A soft failure is any other failure that occurs.
	A hard failure is defined by the Red LED on the board being turned on and left on until reset.
	A soft failure is defined by blinking the Red LED.  The LED is blinked the position in the array + 1 times, ie. Current sensor = 1 time.
	If multiple soft failures occur there is a two second delay between each set.
	The green LED is turned on if there is no Hard Failure.
	I2C interface with the Currrent sensor was written.  It uses the same style of read and writes as the pressure/temperature sensor, and so that code was reused.
	Once a second a read is done from the two current sensors and stored on the SD card.  It is just one byte of data.
	A ST is added for the current sensor where it reads from the ID register on the board.  This is how it's marked as a pass or a fail.  The sensor is initialized either way
	and then the data is read.
	The Current sensor has not been tested.  It just came in the mail and will be tested in the upcoming weeks.
Version 16:
	Uploaded on 03/13/2018
	Added SPI Interface with the MSP430.
	Messages are parsed and data is formatted to be sent out.  Everything is done inside the ISR, because a request from the MSP430 can come at any point in time.
	This was tested using a test bench constructed in a seperate Visual Studio file.  The MSP430 code has not yet been written, and so this was the only method to test.
	The functionality of the algorithim was verified, but the functonality of the SPI slave configuration cannot be verified until the MSP430 code has been written.
Version 17:
	Uploaded on 03/21/2018
	Changed the way SD files are named.  const_cast<char*> is now used to convert from a char* to a const char*, that way the file name can be created on the fly.
	The new format of file names are hhmmss.csv.  This works as a timestamp
	According to the schematic the addresss should be 6F because both pins should be grounded.  It seems like one of the pins is actually not connected, and so the 
	address is actually 0x6E.  If the pin is properly grounded the address should be changed back to 0x6F.
	Updated so that IMU1 is the default IMU (the one that's centered around the motor) and if it fails then IMU0 is used.
	A retry was added on the calibration because sometimes on come up the data comes back corrupted.  Five retries were added with a 50ms delay in between each retry.
	It seems that when the IMU goes from cold to running, the data comes back corrupted on the first attempt.  These short delays seem enough where only a single retry 
	is neccessary to get a good calibration measurement.  The five retries are in there in order for some wiggle room.
	The way that the batteries are handled is being slightly changed. During testing it was found that there was a lot of noise on the individual cell lines.  After talking
	to Carlos it was detemined that this cannot be avoided (he seemed surprised we were only having 150mV on each line, and not more).  His recomendation was to only read the
	rail voltage and to make the determination based off of that.  So all the cells are still being read and written to the SD card, but to determine if the battery should be
	turned on or off just the rail voltage is used.  It is also changed so that 10 consecutive good or bad readings need to happen in order for the battery to be turned on or off.
Version 18:
	Uploaded on 03/27/2018
	A derivative was added to the controller, making it a PID instead of a PI.  The values are now kp = 120, ki = 0.2, kd = 0.01.  Testing with the current setup (Two PCBs the arduino 
	board, the motor controller, motor and three batteries mounted to the chassis) has shown to be good.  The numbers might need to be adjusted again once everything is connected.
	A delay was added after the write to the motor of 50ms.  It was found that writing too fast caused problems.  We were writing at a rate significantly faster than the mechanical time
	constant, and this was giving issues.  Now with this delay everything seems happy.
	The SPI is added but IT IS NOT WORKING.  The algorithim and logic was tested and is working but something with the configuration is not working.  Due to time constraints this will be
	left as is.  If time permits this will be looked at, but for not it is not functioning.
	A function was added to calculate the pressure on the board.  There is a very good chance that the communications board will not be working in time for the launch.  So Dr. Patru has requested
	that everything is done locally. Part of this is to cut down at a certain alitude (40,000 ft + 3hrs).  This is approximated using pressure measurements.  So this function has been added.  
	It has not been called yet, I'm meeting with Carlos tomorrow to discuss how we want to go about this.
Version 19
	Uploaded on 04/04/2018
	Some of the launch day patches were added.  This includes using the altitude to check for 40,000 feet and 10,000 feet.  It triggers a cutdown at three hours and 40,000 feet.  The motor turns on at either
	10,000 feet or ten minutes.  The timer for these two start once we are past two hundred feet from the ground.  the two hundred foot measurement is the only relative measurment.  The other two are taken from 
	sea level.  The original time is stored in flash memory.  It is reset when the button is held on startup.
	There is a problem with the RTC losing power.  We thought using a super cap would be enough to keep it going, but that doesn't seem to be enough.  As a result I am using the SD card to restore the RTC.  I put the
	SD file number in flash memory, and use this to estimate where the RTC was.  A new SD file is created every five minutes, and so this makes it possible to estimate.  TO NEXT YEARS TEAM: Even though last version is the 
	good version, you may want to bring over this RTC functionality.
	The RTC is reset if the button is held down on startup.  Right now if the button is held on startup it will calibrate the gyroscope, get a ground pressure measurment, and reset the RTC.
	*/

//Start Variable Declaration

int BootTime;
bool PressTempFlag = 0; //The pressure and temperature only needs to be collected once a second, and so a timer will be set up to indicate when it is time to check

//Data to be used for SPI
short SPI1Data[16]; 
const byte SPIXGyro = 0, SPIXAccel = 1, SPIYGyro = 2, SPIYAccel = 3, SPIZGyro = 4, SPIZAccel = 5, SPIMainVoltage = 6;
const byte SPIMainCurrent = 7, SPIPressure = 8, SPIOnBoardTemp = 9, SPIOffBoardTemp = 10, SPIMotorEnable = 11, SPIMotorDirection = 12;
const byte SPIMotorVoltage = 13, SPIMotorCurrent = 14, SPIMotorSpeed = 15; //These are where in the array everything is stored

//These variables are used for SPI1.  They are up here so they don't get reinitailized.
unsigned short SPI1RxData, SPI1TxData; //The data will be placed into a short.  The CRC will be placed into it's own byte
int TempCRC, TempCRCData, CRCExponential;
short SPI1_Tx_Data[16];
short SPI1_Tx_SegmentSelect[16], SPI1_Tx_SegmentData[16];
short MsgNum, SegmentSelect, TxMsgNum;
byte RxCRC, TxCRC, TotalDataPiecesTx, NextTxByte, CurrentDataTx, TempBitStorage;
bool MsgError, NoMessage4[16] = { 0 }; //This is set to 1 if the message is bad
const byte SegmentSelectSPI1[16] = { 1,1,1,1,1,1,3,3,0,2,2,4,4,4,4,4 }; // This is the segment select that is tx.  It corresponds with the order of SPI1_Tx_Data
const byte SegmentDataSPI1[16] = { 0,1,2,3,4,5,0,1,0,0,1,0,1,2,3,4 }; //This is the Segment Data to Tx.  It also corresponds with the order of SPI1_Tx_Data
const byte CRC = 0xB1;

//This data is used to send the address to the IMU
const byte XAccelAddress = 0x29, YAccelAddress = 0x2B, ZAccelAddress = 0x2D, XGyroAddress = 0x19, YGyroAddress = 0x1B, ZGyroAddress = 0x1D, IMU0 = 0, IMU1 = 1;
int XAccelData, YAccelData, ZAccelData, XGyroData, YGyroData, ZGyroData;
short XCalibrationData, YCalibrationData, ZCalibrationData;
int XCalibrationMemoryLocation = 0, YCalibrationMemoryLocation = 2, ZCalibrationMemoryLocation = 4; //Each takes two bytes of memory, these locations plus six will be for IMU1

short PressureCalibration;
int PressureCalibrationMemoryLocation = 12;
//This is used for Temperature and Pressure
int PressureData, TemperatureData;
bool AltitudeCutoff;

//This is for the current sensor
int LDO_Current, MainCurrent;

//This is for the SD Card writes
const byte Timestamp = 1, NoTimestamp = 0, SPI0Timeout = 0;
const int NewSDFileNumSeconds = 300; //5 minutes for a new file
int CurrentNumSeconds = 0; // Current time until next SD File

//Variables for motor
const byte CCW = 0, CW = 1;
const int NumMotorWrites = 6;

//Timer Variables
int DeltaTime = 0;
const int FiveSeconds = 5000, TenMinutes = 600, ThreeHours = 10800; //FiveSeconds is in ms, the other two are in seconds.
//Three hours is 3:10 to account for time to prep for takeoff. 10 minutes is 20 for the same reason.
int OriginalTime = 0, CurrentTime = 0;
bool MotorWasOn = 0, CutDownAltitude = 0, CutDownAlreadyCheck = 0;
float GroundPressure;
const float TwoHundredFootDelta = 6, FortyThousandFootAltitude = 187, TenThousandFootAltitude = 697;
const int OriginTimeMemoryLocation = 16;
short OriginTimeFromMemory;
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
	//while (!Serial); // DEBUG DEBUG DEBUG THE PROGRAM WILL NOT START UNTIL THE SERIAL COMM PORT 
	//RESPONDS MAKE SURE TO TAKE OUT
	//DEBUG
	BootTime = millis();
	SDFileNumber = EEPROM.read(SDFileMemoryLocation);//Get the SD File Number
	ADC_Calibration(); //Caibration takes some time to complete, so initiate it before anything else is done
	Init_GPIO(); //Do initilization on GPIO Pins
	Init_RTC(); //Initiliaze Real Time Clock
	SDCard_Setup();//Do initial setup for SD Card
	Init_I2C();//Initiliaze I2C interface
	Init_SPI();//Init\iliaze SPI interface
	Init_MotorInterface(); //Initliaze Motor Interface
	Init1SecTimer(); //Init Timer to trigger an interrupt every 1 second

	if (!digitalRead(PIN_A4)) { //This checks the IMU calibration button.  THE BUTTON NEEDS TO BE HELD ON STARTUP IN ORDER FOR THE CALIBRATION TO TAKE PLACE
		int CalCounter = 0;
		
		EEPROM.write(SDFileMemoryLocation, 0); //This is to restart the RTC
		SDFileNumber = 0; //Restart the file system
		OriginalTime = 0; //Restart the time
		CalibrationDataWrite(0, OriginTimeMemoryLocation);
		Init_RTC(); //Reset the clock
		NewSDFile(); //New File

		SDCard_CalibrationDataWrite(TCal1, TCal2, TCal3, 0, 0, 0, 0, 0, 0, 1); //Writes the Temp Cal Data to the SD Card
		SDCard_CalibrationDataWrite(PCal1, PCal2, PCal3, PCal4, PCal5, PCal6, PCal7, PCal8, PCal9, 0); //Writes the Pressure Cal Data to the SD Card

		I2CRead(TempPressureNumRegisters, TempPressureAddress, TempPressureStartRegister);

		TemperatureData = I2CRxData[2];
		TemperatureData = ((TemperatureData << 8) & 0xFF00) | I2CRxData[1];
		TemperatureData = ((TemperatureData << 4) & 0xFFFF0) | ((I2CRxData[0] >> 4) & 0x0F);
		SDCard_WriteMotorOn(TemperatureData, NoTimestamp);
		SPI1Data[SPIOnBoardTemp] = (TemperatureData / (2 ^ 4)); //Cut off last four bits

		PressureData = I2CRxData[5];
		PressureData = ((PressureData << 8) & 0xFF00) | I2CRxData[4];
		PressureData = ((PressureData << 4) & 0xFFFF0) | ((I2CRxData[3] >> 4) & 0x0F);
		SDCard_WriteMotorOn(PressureData, NoTimestamp);
		SPI1Data[SPIPressure] = (PressureData / (2 ^ 4)); //Cut off last four bits

		GroundPressure = PressureConversion(TemperatureData, PressureData);
		Serial.print("Ground Pressure = ");
		Serial.println(GroundPressure);
		PressureCalibration = GroundPressure; //Make it a short
		CalibrationDataWrite(PressureCalibration, PressureCalibrationMemoryLocation); //Put it in memory

		XCalibrationData = XCalibration(); //Perform a cal
		CalCounter++;

		//This retry was added because sometimes on come up the data comes back corrupted.  The delay seems to fix it.
		while (((XCalibrationData > 20) || (XCalibrationData < -20)) && (CalCounter < 5)) {
			delay(50);
			XCalibrationData = XCalibration(); //Perform a cal
			CalCounter++;
		}

		YCalibrationData = YCalibration(); //Perform a cal
		CalCounter = 1;
		while (((YCalibrationData > 20) || (YCalibrationData < -20)) && (CalCounter < 5)) {
			delay(50);
			YCalibrationData = YCalibration(); //Perform a cal
			CalCounter++;
		}

		ZCalibrationData = ZCalibration(); //Perform a cal
		CalCounter = 1;
		while (((ZCalibrationData > 20) || (ZCalibrationData < -20)) && (CalCounter < 5)) {
			delay(50);
			ZCalibrationData = ZCalibration(); //Perform a cal
			CalCounter++;
		}

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
		PressureCalibration = CalibrationDataRead(PressureCalibrationMemoryLocation);
		GroundPressure = PressureCalibration;
		OriginTimeFromMemory = CalibrationDataRead(OriginTimeMemoryLocation);
		OriginalTime = OriginTimeFromMemory;
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
	} //End no calibration else

	SelfTestDisplayResults(); //Display the results on the red LED
	
	Serial.print("Initialize Time = "); //Display Time it took to initilize
	Serial.print(millis() - BootTime); //Display Time it took to initilize
	Serial.println("ms");//Time is in milliseconds
} //End of setup

void loop() {//Main Loop
	if (!MotorOn) {//Check to see if motor should be turned on
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
	SPI1Data[SPIMotorEnable] = 0;

	while (!MotorOn) { //Stay in this loop until Reaction Wheel is turned on

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
			SPI1Data[SPIXAccel] = XAccelData; //This makes it a 16 bit number instead of 32

			YAccelData = IMURead(YAccelAddress, 0, IMUSelect); //Collect Y Accel
			SDCard_Write(YAccelData, NoTimestamp); //Write it to the SD Card
			SPI1Data[SPIYAccel] = YAccelData;
			
			ZAccelData = IMURead(ZAccelAddress, 0, IMUSelect); //Collect Z Accel
			SDCard_Write(ZAccelData, NoTimestamp); //Write it to the SD Card
			SPI1Data[SPIZAccel] = ZAccelData;

			XGyroData = IMURead(XGyroAddress, 0, IMUSelect) - XCalibrationData; //Collect X Gyro
			SDCard_Write(XGyroData, NoTimestamp); //Write it to the SD Card
			SPI1Data[SPIXGyro] = XGyroData;

			YGyroData = IMURead(YGyroAddress, 0, IMUSelect) - YCalibrationData; //Collect Y Gyro
			SDCard_Write(YGyroData, NoTimestamp); //Write it to the SD Card
			SPI1Data[SPIYGyro] = YGyroData;

			ZGyroData = IMURead(ZGyroAddress, 0, IMUSelect) - ZCalibrationData; //Collect Z Gyro
			SDCard_Write(ZGyroData, NoTimestamp); //Write it to the SD Card
			SPI1Data[SPIZGyro] = ZGyroData;

			if (PressTempFlag) { //Has one second gone by since the last Temp/Press Measurement?
				/*The temperature and pressure measurements are performed as six straight reads.  The first is the MSB of pressure, the
				second is the LSB of the pressure.  The third is four addition least significant bits.  The 4th-6th registers are used 
				the same way but for temperature.
				*/
				SDCardOpenFile();
				PressTempFlag = 0; //Reset Flag
				Heartbeat(); //1 second heartbeat

				I2CRead(TempPressureNumRegisters, TempPressureAddress, TempPressureStartRegister);

				TemperatureData = I2CRxData[2];
				TemperatureData = ((TemperatureData << 8) & 0xFF00) | I2CRxData[1];
				TemperatureData = ((TemperatureData << 4) & 0xFFFF0) | ((I2CRxData[0] >> 4) & 0x0F);
				SDCard_WriteMotorOn(TemperatureData, NoTimestamp);
				SPI1Data[SPIOnBoardTemp] = (TemperatureData / (2^4)); //Cut off last four bits

				PressureData = I2CRxData[5];
				PressureData = ((PressureData << 8) & 0xFF00) | I2CRxData[4];
				PressureData = ((PressureData << 4) & 0xFFFF0) | ((I2CRxData[3] >> 4) & 0x0F);
				SDCard_WriteMotorOn(PressureData, NoTimestamp);
				SPI1Data[SPIPressure] = (PressureData / (2^4)); //Cut off last four bits

				if ((OriginalTime == 0) && ((GroundPressure - TwoHundredFootDelta) >= PressureConversion(TemperatureData, PressureData))) { //Get original Clock, you're over 200ft
					Serial.println("Clock Set");
					GetClock(); //Get clock
					OriginalTime = ((RTCCurrentData[2] * 3600) + (RTCCurrentData[1] * 60) + (RTCCurrentData[0])); //Gets the number of seconds.
					OriginTimeFromMemory = OriginalTime;
					CalibrationDataWrite(OriginTimeFromMemory, OriginTimeMemoryLocation);
				}

					for (int counter = 0; counter < 9; counter++) {
						SDCard_WriteMotorOn(ADCData[counter], NoTimestamp); //Write all of the ADC Data
					}

					SPI1Data[SPIMainVoltage] = ADCData[6]; //For these two just the first cell will be used
					SPI1Data[SPIMotorVoltage] = ADCData[0];

					CheckBatteryLevel(); //Check if the battery needs to be shut down
					ADC0_Select = 0; //Reset the cycle
					ADC1_Select = 0; //Reset the cycle
					BeginADCConversion(); //Kick off the conversions

				I2CRead(CurrentSensorNumBytes, CurrentSensorAddress, CurrentSensorMainBattery);
				MainCurrent = I2CRxData[5]; //Get the Main Current from the buffer
				I2CRead(CurrentSensorNumBytes, CurrentSensorAddress, CurrentSensorLDO);
				LDO_Current = I2CRxData[5]; //Get the LDO Current from the buffer

				SDCard_WriteMotorOn(MainCurrent, NoTimestamp); //Write to the SD Card
				SDCard_WriteMotorOn(LDO_Current, NoTimestamp); //Write to the SD Card

				SPI1Data[SPIMainCurrent] = MainCurrent;
				SPI1Data[SPIMotorCurrent] = LDO_Current;

				SDCardCloseFile();
			} //End Pressure Temperature if
				SDCard_NewLine();

			if (DataNotValidSPI0 == 1) { //If SPI0 Timeout Occured
				Init_IMU_SPI(); //Reset SPI0
				IMUSelfTest(); //Test Them
				SDCard_NewLine(); //Enter New Line
				SDCard_SensorFailure(SPI0Timeout); //Print Error Message
			}

			if ((CurrentNumSeconds % 60) == 0) {
				GetClock();
				CurrentTime = ((RTCCurrentData[2] * 3600) + (RTCCurrentData[1] * 60) + (RTCCurrentData[0])); //Get number of seconds

				if ((((CurrentTime - OriginalTime) >= TenMinutes) || PressureComparison(TemperatureData, PressureData, TenThousandFootAltitude)) && (!MotorWasOn)) { //Is it time to turn the motor on?
					MotorOn = 1;
					MotorWasOn = 1;
				} //End motor on if

				else if (((CurrentTime - OriginalTime) >= ThreeHours) && (!CutDownAlreadyCheck)) { //Check if it's time to cut down
					CutDownAlreadyCheck = 1; //Mark that you checked this
					if (PressureComparison(TemperatureData, PressureData, FortyThousandFootAltitude)) {
						Cutdown();
					} //End 40,000 feet check
				} //End cut down time
			}

			if (CurrentNumSeconds >= NewSDFileNumSeconds) {
				NewSDFile(); //Create a new file
				CurrentNumSeconds = 0; //Restart the counter
			} //End five minute if

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
	double ZGyro_RawData, ZGyro_RPM = 0, Error, OldError = 0, Integral = 0, Control_Speed, Derivative = 0;
	const double kp = 140; //For algorithim
	const double ki = 0.2; //For algorithim
	const double kd = 0.01; //For algorithim
	const double GyroLSB = 0.061035156; //(2000/2^15): Range of data is +/-2000deg/sec.
	double CurrentPressure = 0, PreviousPressure = 10000; //Init Previous Pressure as a big number
	byte ConsecutiveDescending = 0; //Init to 0

	Serial.println("Motor Turned On");
	SPI1Data[SPIMotorEnable] = 1;

	SDCardOpenFile(); //Open the file for the duration of the motor being turned on
	TurnMotorOn(); //Enable the motor
	digitalWrite(BlueLED, HIGH);

	while (MotorOn) {//Stay in this loop until Reaction Wheel is turned on

		XAccelData = IMURead(XAccelAddress, 0, IMUSelect); //Collect X Accel
		SDCard_WriteMotorOn(XAccelData, Timestamp); //Write it to the SD Card
		SPI1Data[SPIXAccel] = XAccelData;

		YAccelData = IMURead(YAccelAddress, 0, IMUSelect); //Collect Y Accel
		SDCard_WriteMotorOn(YAccelData, NoTimestamp); //Write it to the SD Card
		SPI1Data[SPIYAccel] = YAccelData;

		ZAccelData = IMURead(ZAccelAddress, 0, IMUSelect); //Collect Z Accel
		SDCard_WriteMotorOn(ZAccelData, NoTimestamp); //Write it to the SD Card
		SPI1Data[SPIZAccel] = ZAccelData;

		XGyroData = IMURead(XGyroAddress, 0, IMUSelect) - XCalibrationData; //Collect X Gyro
		SDCard_WriteMotorOn(XGyroData, NoTimestamp); //Write it to the SD Card
		SPI1Data[SPIXGyro] = XGyroData;

		YGyroData = IMURead(YGyroAddress, 0, IMUSelect) - YCalibrationData; //Collect Y Gyro
		SDCard_WriteMotorOn(YGyroData, NoTimestamp); //Write it to the SD Card
		SPI1Data[SPIYGyro] = YGyroData;

		ZGyroData = IMURead(ZGyroAddress, 0, IMUSelect) - ZCalibrationData; //Collect Z Gyro
		SDCard_WriteMotorOn(ZGyroData, NoTimestamp); //Write it to the SD Card
		SPI1Data[SPIZGyro] = ZGyroData;

		if (PressTempFlag) { //Has one second gone by since the last Temp/Press Measurement?
			PressTempFlag = 0; //Reset Flag
			Heartbeat(); //1 second heartbeat

			I2CRead(TempPressureNumRegisters, TempPressureAddress, TempPressureStartRegister); //Read Temp/Press

			TemperatureData = I2CRxData[2];
			TemperatureData = ((TemperatureData << 8) & 0xFF00) | I2CRxData[1];
			TemperatureData = ((TemperatureData << 4) & 0xFFFF0) | ((I2CRxData[0] >> 4) & 0x0F);
			SDCard_WriteMotorOn(TemperatureData, NoTimestamp); //Write to card
			SPI1Data[SPIOnBoardTemp] = (TemperatureData / (2^4)); //Cut off the last four bits

			PressureData = I2CRxData[5];
			PressureData = ((PressureData << 8) & 0xFF00) | I2CRxData[4];
			PressureData = ((PressureData << 4) & 0xFFFF0) | ((I2CRxData[3] >> 4) & 0x0F);

			SDCard_WriteMotorOn(PressureData, NoTimestamp); //Write to card
			SPI1Data[SPIPressure] = (PressureData / (2^4)); //Cut off the last four bits

				for (int counter = 0; counter < 9; counter++) {
					SDCard_WriteMotorOn(ADCData[counter], NoTimestamp); //Write all of the ADC Data
				}

				SPI1Data[SPIMainVoltage] = ADCData[5]; //For these two just the last cell will be used
				SPI1Data[SPIMotorVoltage] = ADCData[8];

				CheckBatteryLevel(); //Check if the battery needs to be shut down
				ADC0_Select = 0; //Reset the cycle
				ADC1_Select = 0; //Reset the cycle
				BeginADCConversion(); //Kick off the conversions

			I2CRead(CurrentSensorNumBytes, CurrentSensorAddress, CurrentSensorMainBattery);
			MainCurrent = I2CRxData[5]; //Get the Main Current from the buffer
			I2CRead(CurrentSensorNumBytes, CurrentSensorAddress, CurrentSensorLDO);
			LDO_Current = I2CRxData[5]; //Get the LDO Current from the buffer

			SDCard_WriteMotorOn(MainCurrent, NoTimestamp); //Write to the SD Card
			SDCard_WriteMotorOn(LDO_Current, NoTimestamp); //Write to the SD Card

			SPI1Data[SPIMainCurrent] = MainCurrent;
			SPI1Data[SPIMotorCurrent] = LDO_Current;
		} //End Pressure Temperature if
		
		if ((CurrentNumSeconds % 60) == 0) { //Check once a minute
			GetClock();
			CurrentTime = ((RTCCurrentData[2] * 3600) + (RTCCurrentData[1] * 60) + (RTCCurrentData[0])); //Get number of seconds

			if (((CurrentTime - OriginalTime) >= ThreeHours) && (!CutDownAlreadyCheck)) { //Check if it's time to cut down
				CutDownAlreadyCheck = 1; //Mark that you checked this
				MotorOn = 0; //Turn the motor off
				if (PressureComparison(TemperatureData, PressureData, FortyThousandFootAltitude)) { //Are you above 40,000?
					Cutdown(); //Cutdown
				} //End 40,000 feet check
			} //End cut down time

			else { //You only want to check this if the motor is still going to be on
				CurrentPressure = PressureConversion(TemperatureData, PressureData);
				if (CurrentPressure > (PreviousPressure + 2)) { //Are you descending?
					ConsecutiveDescending++; //Increment
					if (ConsecutiveDescending >= 3) { //Three in a row descending
						MotorOn = 0; //Turn the motor off
					} //End >=3 if
				} //End descending if
				else { //You're not descending
					ConsecutiveDescending = 0; //Reset the counter
				} //End else
				PreviousPressure = CurrentPressure; //Move the current pressure into previous
			} //End overarching else
		} //End one minute loop

		if (CurrentNumSeconds >= NewSDFileNumSeconds) {
			SDCardCloseFile(); //Close the previous file
			NewSDFile(); //Create a new file
			CurrentNumSeconds = 0; //Restart the counter
			SDCardOpenFile(); //Open the new file
		} //End five minute if

		SDCard_NewLineMotorOn(); //Enter a new line
		SDCard_FlushBuffer(); //Flush the buffer
		
			ZGyro_RawData = (ZGyroData * GyroLSB) / 6; //The divide by 6 does the conversion from deg/sec to rev/minute (deg/sec * 60/360 = rev/min)
			ZGyro_RPM = EMA(ZGyro_RawData, ZGyro_RPM); //Calculate the running average of the data.  This will be used to run the reaction wheel

			if ((ZGyro_RPM < 0.05) && (ZGyro_RPM > -0.05)) {
				ZGyro_RPM = 0; //This is a filter on the data so it doesn't spin when there is nothing going on.  It filters it for +/-0.05rev/min
			}

			Error = -ZGyro_RPM; //The '-' is for orientation

			Integral = Integral + Error; //Keep adding to the integral.  This is the very literal of the definition of an integral, being a running sum

			Derivative = (Error - OldError); //Get the derivative term (delta between the two points)
			OldError = Error; //Store the old error
			
			Control_Speed = (kp * Error) + (ki * Integral) + (kd * Derivative); //Take a weighted sum of the two.

			if (Control_Speed > 0.2) {
				SPI1Data[SPIMotorDirection] = 0;
				Motor_Direction(CCW); //TODO DEBUG DEBUG DEBUG THIS IS FOR THE UPSIDE DOWN IMU
				//Motor_Direction(CW); //Turn it on CW
				MotorSpeed(Control_Speed); //Turn it on at the desired speed
			}

			else if (Control_Speed < -0.2) {
				SPI1Data[SPIMotorDirection] = 1;
				//Motor_Direction(CCW); //Turn it on CCW
				Motor_Direction(CW); //TODO DEBUG DEBUG DEBUG THIS IS FOR THE UPSIDE DOWN IMU
				MotorSpeed((-Control_Speed)); //Turn it on to the desired speed
			}

			else {
				NoMotorSpeed(); //Speed is = 0
			}
			delay(50);
	}//End While

	digitalWrite(BlueLED, LOW);
	TurnMotorOff(); //Disable the motor
	SDCardCloseFile(); //Close the SD Card File
	Serial.println("Motor Turned Off");
} //End ReactionWheelOn

void Heartbeat() {
	GPIOC_PTOR ^= 0x20; //Toggle On board LED (Pin C5).
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
	CurrentNumSeconds++;
}

void spi1_isr() {
	 //See ICD for details on how the messages are constructed. Document name MSP430_Teensy_ICD.docx in Document folder in 
	 // bad-hapibs Github
	 Serial.println("Interrupt");
	 SPI1_SR |= 0x80000000; //Clears bit to indicate interrupt

	 if (SPI1RxByteCount == 0) { //First byte received
		 SPI1RxByteCount++;
		 SPI1RxData = 0;
		 MsgError = 0;
		 SPI1RxData = (SPI1_POPR & 0xFF);
		 MsgNum = SPI1RxData >> 4; //The Msg number is the most significant nibble

		 switch (MsgNum) {
		 case 0:
			 break; //Bits 19-16 are not used
		 case 2:
			 SegmentSelect = (SPI1RxData & 0x0F); //Get the segment select

			 if (SegmentSelect <= 4) { //these only need one piece of data
				 TotalDataPiecesTx = 1;
			 }
			 else { //These need multiple pieces of data
				 switch (SegmentSelect) { //Find out how much data
				 case 5:
					 TotalDataPiecesTx = 3;
					 break;
				 case 6:
					 TotalDataPiecesTx = 3;
					 break;
				 case 7:
					 TotalDataPiecesTx = 6;
					 break;
				 case 8:
					 TotalDataPiecesTx = 2;
					 break;
				 case 9:
					 TotalDataPiecesTx = 2;
					 break;
				 case 10:
					 TotalDataPiecesTx = 4;
					 break;
				 case 11:
					 TotalDataPiecesTx = 16;
					 break;
				 default:
					 MsgError = 1;
					 TxMsgNum = 1;
					 TxSPI1Msg = 1;
					 break;
				 } //End switch
			 } //End else

			 break;
		 case 5:
			 SPI1_PUSHR_SLAVE = (SPI1TxData & 0xFF); //Tx Second Byte
			 break; //Bits 19-16 are not used
		 default:
			 MsgError = 1;
			 TxMsgNum = 1;
			 TxSPI1Msg = 1;
			 break;
		 } //End switch

		 SPI1RxData = SPI1RxData << 8; //Byte shift
	 } //End if

	 else if (SPI1RxByteCount == 1) { //Second byte received
		 SPI1RxByteCount++;
		 SPI1RxData = (SPI1RxData & 0xFF00) | (SPI1_POPR & 0xFF); //Put it in the SPI1RxData 
		 switch (MsgNum)
		 {
		 case 0:
			 if ((SPI1RxData & 0x20) != 0) {
				 ResetSPI1Bus = 1; //The bus needs to be reset
			 }
			 else if ((SPI1RxData & 0x10) != 0) {
				 TxSPI1Msg = 1; //Previous Message Needs to be sent
			 }
			 else {
				 TxSPI1Msg = 1; //Resend the message
				 TxMsgNum = (SPI1RxData & 0x0F); //Resend the message number stored in these bits
			 }
			 break;
		 case 2://Message 2

			 switch (SegmentSelect) { //Get the data to be transmitted
			 case 0: //Pressure Data
				 SPI1_Tx_Data[0] = SPI1Data[SPIPressure];
				 SPI1_Tx_SegmentSelect[0] = SegmentSelectSPI1[SPIPressure];
				 SPI1_Tx_SegmentData[0] = SegmentDataSPI1[SPIPressure];
				 break;
			 case 1: //Get the proper IMU data
				 SPI1_Tx_Data[0] = SPI1Data[(SPI1RxData & 0x07) + SPIXGyro];
				 SPI1_Tx_SegmentSelect[0] = SegmentSelectSPI1[(SPI1RxData & 0x07) + SPIXGyro];
				 SPI1_Tx_SegmentData[0] = SegmentDataSPI1[(SPI1RxData & 0x07) + SPIXGyro];
				 break;
			 case 2://Get the temperature data
				 SPI1_Tx_Data[0] = SPI1Data[((SPI1RxData >> 3) & 0x01) + SPIOnBoardTemp];
				 SPI1_Tx_SegmentSelect[0] = SegmentSelectSPI1[((SPI1RxData >> 3) & 0x01) + SPIOnBoardTemp];
				 SPI1_Tx_SegmentData[0] = SegmentDataSPI1[((SPI1RxData >> 3) & 0x01) + SPIOnBoardTemp];
				 break;
			 case 3: //Get the battery Data
				 SPI1_Tx_Data[0] = SPI1Data[((SPI1RxData >> 4) & 0x01) + SPIMainVoltage];
				 SPI1_Tx_SegmentSelect[0] = SegmentSelectSPI1[((SPI1RxData >> 4) & 0x01) + SPIMainVoltage];
				 SPI1_Tx_SegmentData[0] = SegmentDataSPI1[((SPI1RxData >> 4) & 0x01) + SPIMainVoltage];
				 break;
			 case 4:  //Motor Data
				 if (((SPI1RxData >> 5) & 0x07) < 2) { //Is it a binary response?
					 NoMessage4[0] = 1;
					 SPI1_Tx_Data[0] = SPI1Data[((SPI1RxData >> 5) & 0x07) + SPIMotorEnable];
					 SPI1_Tx_SegmentSelect[0] = SegmentSelectSPI1[((SPI1RxData >> 5) & 0x07) + SPIMotorEnable];
					 SPI1_Tx_SegmentData[0] = SegmentDataSPI1[((SPI1RxData >> 5) & 0x07) + SPIMotorEnable];
				 }
				 else {
					 SPI1_Tx_Data[0] = SPI1Data[((SPI1RxData >> 5) & 0x07) + SPIMotorEnable];
					 SPI1_Tx_SegmentSelect[0] = SegmentSelectSPI1[((SPI1RxData >> 5) & 0x07) + SPIMotorEnable];
					 SPI1_Tx_SegmentData[0] = SegmentDataSPI1[((SPI1RxData >> 5) & 0x07) + SPIMotorEnable];
				 }
				 break;
			 case 5://All gyro data
				 for (int counter = 0; counter < 3; counter++) {
					 SPI1_Tx_Data[counter] = SPI1Data[(counter * 2) + SPIXGyro];
					 SPI1_Tx_SegmentSelect[counter] = SegmentSelectSPI1[(counter * 2) + SPIXGyro];
					 SPI1_Tx_SegmentData[counter] = SegmentDataSPI1[(counter * 2) + SPIXGyro];
				 } //End for
				 break;
			 case 6: //All Accel Data
				 for (int counter = 0; counter < 3; counter++) {
					 SPI1_Tx_Data[counter] = SPI1Data[(counter * 2) + 1 + SPIXGyro];
					 SPI1_Tx_SegmentSelect[counter] = SegmentSelectSPI1[(counter * 2) + 1 + SPIXGyro];
					 SPI1_Tx_SegmentData[counter] = SegmentDataSPI1[(counter * 2) + 1 + SPIXGyro];
				 } //End for
				 break;
			 case 7: //All IMU Data
				 for (int counter = 0; counter < 6; counter++) {
					 SPI1_Tx_Data[counter] = SPI1Data[counter + SPIXGyro];
					 SPI1_Tx_SegmentSelect[counter] = SegmentSelectSPI1[counter + SPIXGyro];
					 SPI1_Tx_SegmentData[counter] = SegmentDataSPI1[counter + SPIXGyro];
				 } //End for
				 break;
			 case 8: //Both Temperature Sensors
				 for (int counter = 0; counter < 2; counter++) {
					 SPI1_Tx_Data[counter] = SPI1Data[(counter + SPIOnBoardTemp)];
					 SPI1_Tx_SegmentSelect[counter] = SegmentSelectSPI1[(counter + SPIOnBoardTemp)];
					 SPI1_Tx_SegmentData[counter] = SegmentDataSPI1[(counter + SPIOnBoardTemp)];
				 }//End for
				 break;
			 case 9: //Main Battery Data
				 for (int counter = 0; counter < 2; counter++) {
					 SPI1_Tx_Data[counter] = SPI1Data[(counter + SPIMainVoltage)];
					 SPI1_Tx_SegmentSelect[counter] = SegmentSelectSPI1[(counter + SPIMainVoltage)];
					 SPI1_Tx_SegmentData[counter] = SegmentDataSPI1[(counter + SPIMainVoltage)];
				 }//End for
				 break;
			 case 10: //All Motor Data
				 for (int counter = 0; counter < 5; counter++) {
					 if (counter < 2) {
						 SPI1_Tx_Data[counter] = SPI1Data[SPIMotorEnable + counter];
						 NoMessage4[counter] = 1;
						 SPI1_Tx_SegmentSelect[counter] = SegmentSelectSPI1[SPIMotorEnable + counter];
						 SPI1_Tx_SegmentData[counter] = SegmentDataSPI1[SPIMotorEnable + counter];
					 }//End if
					 else {
						 SPI1_Tx_Data[counter] = SPI1Data[SPIMotorEnable + counter];
						 NoMessage4[counter] = 0; //This shouldn't be necessary
						 SPI1_Tx_SegmentSelect[counter] = SegmentSelectSPI1[SPIMotorEnable + counter];
						 SPI1_Tx_SegmentData[counter] = SegmentDataSPI1[SPIMotorEnable + counter];
					 }// end else
				 } //End for
				 break;
			 case 11: //All Data
				 for (int counter = 0; counter < 15; counter++) {
					 if ((counter == SPIMotorEnable) || (counter == SPIMotorDirection)) {
						 SPI1_Tx_Data[counter] = SPI1Data[counter];
						 NoMessage4[counter] = 1;
						 SPI1_Tx_SegmentSelect[counter] = SegmentSelectSPI1[counter];
						 SPI1_Tx_SegmentData[counter] = SegmentDataSPI1[counter];
					 }//End if
					 else {
						 SPI1_Tx_Data[counter] = SPI1Data[counter];
						 NoMessage4[counter] = 0; //This isn't necessary
						 SPI1_Tx_SegmentSelect[counter] = SegmentSelectSPI1[counter];
						 SPI1_Tx_SegmentData[counter] = SegmentDataSPI1[counter];
					 }// end else
				 } //End for
				 break;
			 default:
				 MsgError = 1;
				 TxMsgNum = 1;
				 TxSPI1Msg = 1;
			 } //End get data switch

			 TxSPI1Msg = 1; //A message is going to be sent
			 TxMsgNum = 0x03; //It's message 3
			 CurrentDataTx = 0; //You haven't Tx'd anything yet
			 break;
		 case 5:
			 SPI1_PUSHR_SLAVE = TxCRC; // Tx the CRC
			 break; //Bits 15-8 are not used
		 }
	 } //End else if

	 else { //CRC Received
		 SPI1TxData = 0; //Reset this
		 SPI1RxByteCount = 0; //Reset for the next cycle

		 RxCRC = (SPI1_POPR & 0xFF); //Get it from the buffer

		 TempCRCData = (SPI1RxData << 8) | (RxCRC); //Put it in a data buffer
		 TempCRC = CRC << 16; //Shift it over

		 for (int counter = 23; counter >= 7; counter--) { //Calculate the CRC
			 CRCExponential = pow(2, counter);
			 if ((TempCRCData & CRCExponential) != 0) {
				 TempCRCData = TempCRC ^ TempCRCData;
			 }
			 TempCRC = TempCRC >> 1;
		 }
		 
		 if (TempCRCData != 0) { //CRC Failure
			 TxSPI1Msg = 1;
			 TxMsgNum = 1;
			 MsgError = 1;
		 }

		 if (ResetSPI1Bus) { //Reset the bus
			 Init_DAQCS_SPI();//Reset the bus
		 }

		 else { //Don't reset the bus
			 if (TxSPI1Msg) {//Do you have to transmit on the next cycle?
				 switch (TxMsgNum) {
				 case 1:
					 SPI1TxData = ((TxMsgNum << 12) & 0xF000);

					 if (MsgError) {
						 SPI1TxData |= 0x20;
						 ResetSPI1Bus = 1;
					 }

					 TxSPI1Msg = 0;
					 break;
				 
				 case 3:
					 SPI1TxData = ((0xF000 & (TxMsgNum << 12)) | ((SPI1_Tx_SegmentSelect[CurrentDataTx] << 9) & 0x0E00) | ((SPI1_Tx_SegmentData[CurrentDataTx] << 5) & 0x00E0));

					 if (NoMessage4[CurrentDataTx]) { //No message four?
						 SPI1TxData |= ((0x100)|((SPI1_Tx_Data[CurrentDataTx] << 4) & 0x10));
					 }
					 
					 else { //Yes message four
						 SPI1TxData |= (0x000F & (SPI1_Tx_Data[CurrentDataTx] >> 12));
					 }

					 if ((NoMessage4[CurrentDataTx]) && ((CurrentDataTx + 1) == TotalDataPiecesTx)) { //Do you not need a message 4, and is this your last message
						 TxSPI1Msg = 0;
					 }
					 else if (!NoMessage4[CurrentDataTx]) { //Next Message is 4
						 TxMsgNum = 4; //Set it up to Tx 4 on the next loop
					 }
					 NoMessage4[CurrentDataTx] = 0; //Reset this bit
					 
					 break;
				 case 4:
					 SPI1TxData = (((TxMsgNum << 12) & 0xF000) | (SPI1_Tx_Data[CurrentDataTx] & 0x0FFF));

					 CurrentDataTx++;
					 if (CurrentDataTx < TotalDataPiecesTx) {
						 TxMsgNum = 3;
					 }
					 else {
						 TxSPI1Msg = 0;
					 }
					 break;
				 default:
					 SPI1TxData = (((TxMsgNum << 12) & 0xF000) | 0x20); //Something went wrong, reset the bus
					 ResetSPI1Bus = 1;
					 break;
				 } //End Tx Msg Switch


				 TempCRCData = SPI1TxData << 8; //Put it in a data buffer
				 TempCRC = CRC << 16; //Shift it over

				 for (int counter = 23; counter >= 7; counter--) { //Calculate the CRC
					 CRCExponential = pow(2, counter);
					 if ((TempCRCData & CRCExponential) != 0) {
						 TempCRCData = TempCRC ^ TempCRCData;
					 }
					 TempCRC = TempCRC >> 1;
				 }

				 TxCRC = TempCRCData;
				 
				 SPI1_PUSHR_SLAVE = ((SPI1TxData >> 8) & 0xFF); //Tx First Byte
			 } //End Tx if
		 } //end not reseting the bus else.
	 } //End third byte else
 }