// Clocks.h

#ifndef _CLOCKS_h
#define _CLOCKS_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "wprogram.h"
#else
#include "WProgram.h"
#endif


#endif

void Init_RTC();
void GetClock();
void Init1SecTimer();
extern int RTCCurrentData[6];