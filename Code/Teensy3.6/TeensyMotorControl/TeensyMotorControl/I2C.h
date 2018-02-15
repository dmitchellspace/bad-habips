// I2C.h

#ifndef _I2C_h
#define _I2C_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "wprogram.h"
#else
#include "WProgram.h"
#endif


#endif

void Init_I2C();
void TempPressureRead(byte NumBytesRx, byte I2CAddress, byte I2CRegister);
void TempPressureWrite(byte I2CAddress, byte I2CRegister, byte TxData);

extern const byte TempPressureAddress, TempPressureStartRegister, TempPressureNumRegisters;
extern byte I2CRxData[6];