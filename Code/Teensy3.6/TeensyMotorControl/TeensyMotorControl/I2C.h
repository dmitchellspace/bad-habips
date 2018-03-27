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
void I2CRead(byte NumBytesRx, byte I2CAddress, byte I2CRegister);
void I2CWrite(byte I2CAddress, byte I2CRegister, byte TxData);
void InitTempPressure();
void InitCurrentSensor();
bool CutDownPressure(int32_t RawTemp, int32_t RawPressure);

extern const byte TempPressureAddress, TempPressureStartRegister, TempPressureNumRegisters;
extern byte I2CRxData[6];

extern const byte CurrentSensorAddress, CurrentSensorLDO, CurrentSensorMainBattery, CurrentSensorNumBytes;