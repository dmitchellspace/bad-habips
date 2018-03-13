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
int IMURead(byte TxAddress, byte SingleReg, byte IMUNumber);
void IMUSelfTest();
void IMUWrite(byte TxAddress, byte TxData, byte IMUNumber);

extern byte DataNotValidSPI0, IMUSelect, SPI1RxByteCount;
extern bool ResetSPI1Bus, TxSPI1Msg;