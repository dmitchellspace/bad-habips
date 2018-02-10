// SDCard.h

#ifndef _SDCARD_h
#define _SDCARD_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "wprogram.h"
#else
#include "WProgram.h"
#endif


#endif

void SDCard_Setup();
void SDCard_Write(int SDCardData, byte Timestamp);
void SDCard_NewLine();
void SDCard_SensorFailure(byte ErrorCode);
void NewSDFile();
extern int SDCardPresent; //Is the SD Card Present?
void SDCard_WriteDouble(double SDCardData);
void SDCardOpenFile();
void SDCardCloseFile();
void SDCard_WriteMotorOn(int SDCardData, byte Timestamp);
void SDCard_NewLineMotorOn();
void SDCard_FlushBuffer();