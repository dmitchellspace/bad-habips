// Clocks.h

#ifndef _CLOCKS_h
#define _CLOCKS_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "wprogram.h"
#else
	#include "WProgram.h"
#endif


#endif

void Init_Clock();
void Init_RTC();