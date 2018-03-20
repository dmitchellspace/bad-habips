// 
// 
// 

#include "SDCard.h"
#include <SD.h>
#include "Clocks.h"
#include <EEPROM.h>
#include "GPIO.h"

//Define File Names
//const char* SDFileName[48] = { "HABIP0.csv", "HABIP1.csv", "HABIP2.csv", "HABIP3.csv", "HABIP4.csv", "HABIP5.csv", "HABIP6.csv", "HABIP7.csv", "HABIP8.csv", "HABIP9.csv", "HABIP10.csv", "HABIP11.csv", "HABIP12.csv", "HABIP13.csv", "HABIP14.csv", "HABIP15.csv", "HABIP16.csv", "HABIP17.csv", "HABIP18.csv", "HABIP19.csv", "HABIP20.csv", "HABIP21.csv", "HABIP22.csv", "HABIP23.csv", "HABIP24.csv", "HABIP25.csv", "HABIP26.csv", "HABIP27.csv", "HABIP28.csv", "HABIP29.csv", "HABIP30.csv", "HABIP31.csv", "HABIP32.csv", "HABIP33.csv", "HABIP34.csv", "HABIP35.csv", "HABIP36.csv", "HABIP37.csv", "HABIP38.csv", "HABIP39.csv", "HABIP40.csv", "HABIP41.csv", "HABIP42.csv", "HABIP43.csv", "HABIP44.csv", "HABIP45.csv", "HABIP46.csv", "HABIP47.csv" };
const char* SDFileName[48] = { "H0","H1","H2","H3","H4","H5","H6","H7","H8","H9","H10","H11","H12","H13","H14","H15","H16","H17","H18","H19","H20","H21","H22","H23","H24","H25","H26","H27","H28","H29","H30","H31","H32","H33","H34","H35","H36","H37","H38","H39","H40","H41","H42","H43","H44","H45","H46","H47"};

const int chipSelect = BUILTIN_SDCARD;
File SDFile;
int SDCardPresent; //Is the SD Card Present?
bool MotorOn = 0;
byte FileNumber;
int FileNumberMemoryLocation = 12; //This is used to store the file number in EEPROM

void SDCard_Setup() { //Initiliaze SD Card
	if (!SD.begin(chipSelect)) { //Check if SD Card Is present
		Serial.println("SD Card failed, or Not Present"); //It is not
		FaultMatrix[7] = 1;
		SDCardPresent = 0; //SD Card Failed
	}
	else { //Set up New File
		//FileNumber = EEPROM.read(FileNumberMemoryLocation); DEBUG DEBUG DEBUG  UNCOMMENT THIS FOR LAUNCH
		FileNumber = -1; //TODO Change this to EEPROM Read for Launch
		NewSDFile();
	}
}

void SDCard_Write(int SDCardData, byte Timestamp) {
	SDFile = SD.open(SDFileName[FileNumber], FILE_WRITE); //Open the file to write
	if (Timestamp == 1) {
		GetClock(); //Get Current RTC
		SDFile.print(RTCCurrentData[2]); //Hours
		SDFile.print(':');
		SDFile.print(RTCCurrentData[1]); //Minutes
		SDFile.print(":");
		SDFile.print(RTCCurrentData[0]); //Seconds
		SDFile.print(",");
	}
	SDFile.print(SDCardData); //Record the data
	SDFile.print(","); //Add a comma
	SDFile.close(); //Close SD File
}

void SDCard_NewLine() { //Add a new line to the SD Card
	SDFile = SD.open(SDFileName[FileNumber], FILE_WRITE); //Open the file to write
	SDFile.println();
	SDFile.close(); //Close SD File
}

void SDCard_SensorFailure(byte ErrorCode) {
	SDFile = SD.open(SDFileName[FileNumber], FILE_WRITE); //Open the file to write
	switch (ErrorCode)
		case 0: Serial.println("SPI0 Timeout. Last Line of Data is not Valid");
	SDFile.close();
}

void NewSDFile() {
		FileNumber++; //Increment to next file
		EEPROM.write(FileNumberMemoryLocation, FileNumber); //Write the File number to memory

		if (SD.exists(SDFileName[FileNumber])) { //DEBUG MAKE SURE TO REMOVE FOR FINAL VERSION
			//DEBUG MAKE SURE TO REMOVE FOR FINAL VERSION
			SD.remove(SDFileName[FileNumber]); //This deletes what is currently on the card
											   //DEBUG MAKE SURE TO REMOVE FOR FINAL VERSION
		}

	SDFile = SD.open(SDFileName[FileNumber], FILE_WRITE); //Creates a new file
	if (SDFile) {//Check to see if file is successfully opened
		SDFile.print("HAPIB Flight "); //Put Header on SD Card File
		GetClock();
		SDFile.print(RTCCurrentData[4]); //Add month
		SDFile.print("/");
		SDFile.print(RTCCurrentData[3]); //Add day
		SDFile.print("/");
		SDFile.println(RTCCurrentData[5]); //Add year
		Serial.print("SD Card initialized, HAPIB.csv created.  File Number: "); //Success
		Serial.println(FileNumber + 1);
		SDCardPresent = 1; //SD Card Passed
		SDFile.println("Time,XAccel,YAccel,ZAccel,XGyro,YGyro,ZGyro,Temperature,Pressure,S1,S2,S3,S4,S5,S5,M1,M2,M3,M_Current,LDO_Current");
	}
	else {
		Serial.println("SD Card initialized, HAPIB.txt failed to open"); //Failed
		SDCardPresent = 0; //SD Card Failed
	}

SDFile.close(); //Close SD File
}

/*
In order to make the motor controller algorithim run as efficiently as possible, the manner in which SD card writes is done is going to be chagned (only for while
the motor is on).  The opening and closing of the file structure is what takes the longest amount of time, and therefor this will only be done when neccessary.  The
reason that closing it takes a while is because it clears the buffer on a close.  The buffer is 512 characters long and so it is not neccessary to do this every time.
As a result the file will be opened once upon entering the loop, closed once upon leaving the loop, and the buffer will be cleared after all six of the IMU writes,
instead of after every individual write.  This should make each write take about 200us instead of about 3ms.
All of the following functions are used for that purpose:
*/

void SDCardOpenFile() {
	SDFile = SD.open(SDFileName[FileNumber], FILE_WRITE); //Open the file to write
	if (digitalRead(PIN_A6)) {
		SDFile.println("Motor Turned On");
		MotorOn = 1;
	}
}

void SDCardCloseFile() {
	if (MotorOn == 1) {
		SDFile.println("Motor Turned Off");
		MotorOn = 0;
	}
	SDFile.close(); //Close SD File
}


void SDCard_WriteMotorOn(int SDCardData, byte Timestamp) {
	if (Timestamp == 1) {
		GetClock(); //Get Current RTC
		SDFile.print(RTCCurrentData[2]); //Hours
		SDFile.print(':');
		SDFile.print(RTCCurrentData[1]); //Minutes
		SDFile.print(":");
		SDFile.print(RTCCurrentData[0]); //Seconds
		SDFile.print(",");
	}
	SDFile.print(SDCardData); //Record the data
	SDFile.print(","); //Add a comma
}

void SDCard_NewLineMotorOn() {
	SDFile.println();
}

void SDCard_FlushBuffer(){
	SDFile.flush();
}

void SDCard_CalibrationDataWrite(unsigned short Cal1, signed short Cal2, signed short Cal3, signed short Cal4, signed short Cal5, signed short Cal6, signed short Cal7, signed short Cal8, signed short Cal9, bool Temp) {
	// If the calibration data being written is temperature, the "Temp" bit shall be set to 1, and only the first 3 Cal Data will be 
	// used.  If the Temp bit is not set, all 9 will be used.

	SDFile = SD.open(SDFileName[FileNumber], FILE_WRITE); //Creates a new file

	if (Temp) { //Temperature Data
		SDFile.print(",");
		SDFile.print("T1 = ,");
		SDFile.print(Cal1); 
		SDFile.print(",");
		SDFile.print("T2 = ,");
		SDFile.print(Cal2);
		SDFile.print(",");
		SDFile.print("T3 = ,");
		SDFile.print(Cal3);
		SDFile.print(",");
	}
	else { //Pressure Data
		SDFile.print(",");
		SDFile.print("P1 = ,");
		SDFile.print(Cal1);
		SDFile.print(",");
		SDFile.print("P2 = ,");
		SDFile.print(Cal2);
		SDFile.print(",");
		SDFile.print("P3 = ,");
		SDFile.print(Cal3);
		SDFile.print(",");
		SDFile.print("P4 = ,");
		SDFile.print(Cal4);
		SDFile.print(",");
		SDFile.print("P5 = ,");
		SDFile.print(Cal5);
		SDFile.print(",");
		SDFile.print("P6 = ,");
		SDFile.print(Cal6);
		SDFile.print(",");
		SDFile.print("P7 = ,");
		SDFile.print(Cal7);
		SDFile.print(",");
		SDFile.print("P8 = ,");
		SDFile.print(Cal8);
		SDFile.print(",");
		SDFile.print("P9 = ,");
		SDFile.print(Cal9);
		SDFile.print(",");
	}
	SDFile.println();
	SDFile.close(); //Close SD File
}