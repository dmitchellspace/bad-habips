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
//void SDCard_WriteDouble(double SDCardData);
void SDCardOpenFile();
void SDCardCloseFile();
void SDCard_WriteMotorOn(int SDCardData, byte Timestamp);
void SDCard_NewLineMotorOn();
void SDCard_FlushBuffer();
void SDCard_CalibrationDataWrite(unsigned short Cal1, signed short Cal2, signed short Cal3, signed short Cal4, signed short Cal5, signed short Cal6, signed short Cal7, signed short Cal8, signed short Cal9, bool Temp);
void SDCard_WriteDoubleMotorOn(double SDCardData, byte Timestamp);

extern byte FileNumber;