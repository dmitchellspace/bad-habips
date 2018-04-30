// GPIO.h

#ifndef _GPIO_h
#define _GPIO_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "wprogram.h"
#else
#include "WProgram.h"
#endif


#endif

void Init_GPIO();
void SelfTestDisplayResults();
void SetupTimerButton();
void Cutdown();
extern bool FaultMatrix[8];
extern const int BlueLED, BackupMSP430LED, ResetPI0Clock;