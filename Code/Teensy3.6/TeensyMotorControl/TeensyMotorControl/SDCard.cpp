// 
// 
// 
#include "SDCard.h"
#include <SD.h>
#include <SPI.h>

const int chipSelect = BUILTIN_SDCARD;
File SDFile;
int SDCardPresent; //Is the SD Card Present?

void SDCard_Setup() { //Initiliaze SD Card
	if (!SD.begin(chipSelect)) { //Check if SD Card Is present
		Serial.println("SD Card failed, or Not Present"); //It is not
		SDCardPresent = 0; //SD Card Failed
	}
	else {
		//Set up New File
		if (SD.exists("HAPIB.txt")) {
			//DEBUG MAKE SURE TO REMOVE FOR FINAL VERSION
			SD.remove("HAPIB.txt"); //This deletes what is currently on the card
			//DEBUG MAKE SURE TO REMOVE FOR FINAL VERSION
		}
		SDFile = SD.open("HAPIB.txt", FILE_WRITE); //Creates a new file
		if (SDFile) {//Check to see if file is successfully opened
			//DEBUG Enter Date before flight
			SDFile.println("HAPIB Flight XX/XX/XXXX"); //Put Header on SD Card File
			Serial.println("SD Card initialized, HAPIB.txt created"); //Success
			SDCardPresent = 1; //SD Card Passed
		}
		else {
			Serial.println("SD Card initialized, HAPIB.txt failed to open"); //Failed
			SDCardPresent = 0; //SD Card Failed
		}
	}
	SDFile.close(); //Close SD File
}