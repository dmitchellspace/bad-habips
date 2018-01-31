// 
// 
// 

#include "SDCard.h"
#include <SD.h>
#include "Clocks.h"
#include <string>

const int chipSelect = BUILTIN_SDCARD;
File SDFile;
int SDCardPresent; //Is the SD Card Present?
const char* SDFileName = "HAPIB.csv";

void SDCard_Setup() { //Initiliaze SD Card
	if (!SD.begin(chipSelect)) { //Check if SD Card Is present
		Serial.println("SD Card failed, or Not Present"); //It is not
		SDCardPresent = 0; //SD Card Failed
	}
	else {
		//Set up New File
		if (SD.exists(SDFileName)) {
			//DEBUG MAKE SURE TO REMOVE FOR FINAL VERSION
			SD.remove(SDFileName); //This deletes what is currently on the card
			//DEBUG MAKE SURE TO REMOVE FOR FINAL VERSION
		}
		NewSDFile();
	}
}

void SDCard_Write(int SDCardData, byte Timestamp) {
	SDFile = SD.open(SDFileName, FILE_WRITE); //Open the file to write
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
	SDFile.print(","); //Add a tab
	SDFile.close(); //Close SD File
}

void SDCard_NewLine() { //Add a new line to the SD Card
	SDFile = SD.open(SDFileName, FILE_WRITE); //Open the file to write
	SDFile.println();
	SDFile.close(); //Close SD File
}

void SDCard_SensorFailure(byte ErrorCode) {
	SDFile = SD.open(SDFileName, FILE_WRITE); //Open the file to write
	switch (ErrorCode)
		case 0: Serial.println("SPI0 Timeout. Last Line of Data is not Valid");
	SDFile.close();
}

void NewSDFile() {
	SDFile = SD.open(SDFileName, FILE_WRITE); //Creates a new file
	if (SDFile) {//Check to see if file is successfully opened
		SDFile.print("HAPIB Flight "); //Put Header on SD Card File
		GetClock();
		SDFile.print(RTCCurrentData[4]); //Add month
		SDFile.print("/");
		SDFile.print(RTCCurrentData[3]); //Add day
		SDFile.print("/");
		SDFile.println(RTCCurrentData[5]); //Add year
		Serial.println("SD Card initialized, HAPIB.txt created"); //Success
		SDCardPresent = 1; //SD Card Passed
		SDFile.println("Time,XAccel,YAccel,ZAccel,XGyro,YGyro,ZGyro	...	ADD REST OF SENSORS"); //Make sure to add rest of sensors
	}
	else {
		Serial.println("SD Card initialized, HAPIB.txt failed to open"); //Failed
		SDCardPresent = 0; //SD Card Failed
	}
SDFile.close(); //Close SD File
}