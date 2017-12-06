// SPI.h

#ifndef _SPI_h
#define _SPI_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "wprogram.h"
#else
#include "WProgram.h"
#endif


#endif

void Init_SPI();
void Init_IMU_SPI();
void Init_DAQCS_SPI();