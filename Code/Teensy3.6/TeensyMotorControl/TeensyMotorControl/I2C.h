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
float PressureConversion(int32_t RawTemp, int32_t RawPressure);
bool PressureComparison(int32_t RawTemp, int32_t RawPressure, float Comparison);

extern const byte TempPressureAddress, TempPressureStartRegister, TempPressureNumRegisters;
extern byte I2CRxData[6];

extern const byte CurrentSensorAddress, CurrentSensorLDO, CurrentSensorMainBattery, CurrentSensorNumBytes;

extern unsigned short TCal1;
extern signed short TCal2, TCal3;
extern unsigned short PCal1;
extern signed short PCal2, PCal3, PCal4, PCal5, PCal6, PCal7, PCal8, PCal9;