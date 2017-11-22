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
extern int SDCardPresent; //Is the SD Card Present?