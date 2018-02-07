// MotorControl.h

#ifndef _MOTORCONTROL_h
#define _MOTORCONTROL_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "wprogram.h"
#else
#include "WProgram.h"
#endif


#endif

void Init_MotorInterface();
void Init_ADC();
void Init_PWM();
void BeginADCConversion();
void ADC_Calibration();
void TurnMotorOn();
void TurnMotorOff();
double EMA(double GyroData, double OldEMA);
void Motor_Direction(byte Direction);
void MotorSpeed(double MotorRPM);
void NoMotorSpeed();