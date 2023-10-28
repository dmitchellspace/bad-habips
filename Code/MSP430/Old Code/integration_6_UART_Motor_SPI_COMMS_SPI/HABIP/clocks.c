/*
 * CLOCKS.c
 *
 *  Created on: Apr 11, 2017
 *      Author: Lincster
 */
// #include <clocks.h>
#include <msp430.h>
#include "driverlib.h"
#include "common.h"

void config_XT1_GPIO(void){
  PJSEL0 |= BIT4 | BIT5;                  // For XT1
  activate_GPIO_config();
}
void config_XT1_ACLK_32768Hz(void){
  /*Dependencies:
   * config_XT1_GPIO();
   */
// XT1 Setup
    CSCTL0_H = CSKEY_H;                     // Unlock CS registers
    CSCTL2 = SELA__LFXTCLK | SELS__DCOCLK | SELM__DCOCLK;
    CSCTL3 = DIVA__1 | DIVS__1 | DIVM__1;   // Set all dividers
    CSCTL4 &= ~LFXTOFF;
    do
    {
        CSCTL5 &= ~LFXTOFFG;                // Clear XT1 fault flag
        SFRIFG1 &= ~OFIFG;
    } while (SFRIFG1 & OFIFG);              // Test oscillator fault flag
    CSCTL0_H = 0;                           // Lock CS registers
}
void config_XT1_ACLK_32768Hz_DCO_1MHz(void){
  /*Dependencies:
   * config_XT1_GPIO();
   */
  // XT1 Setup
  CSCTL0_H = CSKEY_H;                     // Unlock CS registers
  CSCTL1 = DCOFSEL_0;                     // Set DCO to 1MHz
  CSCTL1 &= ~DCORSEL;
  CSCTL2 = SELA__LFXTCLK | SELS__DCOCLK | SELM__DCOCLK;
  CSCTL3 = DIVA__1 | DIVS__1 | DIVM__1;   // Set all dividers
  CSCTL4 &= ~LFXTOFF;
  do
  {
    CSCTL5 &= ~LFXTOFFG;                // Clear XT1 fault flag
    SFRIFG1 &= ~OFIFG;
  } while (SFRIFG1 & OFIFG);              // Test oscillator fault flag
  CSCTL0_H = 0;                           // Lock CS registers
}
void config_DCO_8MHz(void){
// Startup clock system with max DCO setting ~8MHz
  CSCTL0_H = CSKEY_H;                     // Unlock CS registers
  CSCTL1 = DCOFSEL_3 | DCORSEL;           // Set DCO to 8MHz
  //          ACLK      SMCLK     MCLK
  CSCTL2 = SELA__VLOCLK | SELS__DCOCLK | SELM__DCOCLK; // select clock sources
  CSCTL3 = DIVA__1 | DIVS__1 | DIVM__1;   // Set all dividers
  CSCTL0_H = 0;                           // Lock CS registers
}
void config_ACLK_32k_DCO_8MHz(void){
// Startup clock system with max DCO setting ~8MHz
  CSCTL0_H = CSKEY_H;                     // Unlock CS registers
  CSCTL1 = DCOFSEL_3 | DCORSEL;           // Set DCO to 8MHz
  //          ACLK      SMCLK     MCLK
  CSCTL2 = SELA__LFXTCLK | SELS__DCOCLK | SELM__DCOCLK;
  CSCTL3 = DIVA__1 | DIVS__1 | DIVM__1;   // Set all dividers
  CSCTL4 &= ~LFXTOFF;
   do
   {
     CSCTL5 &= ~LFXTOFFG;                // Clear XT1 fault flag
     SFRIFG1 &= ~OFIFG;
   } while (SFRIFG1 & OFIFG);              // Test oscillator fault flag
  CSCTL0_H = 0;                           // Lock CS registers
}
void config_ACLK_XT1_32KHz_DCO_8MHz_SMCLK_250KHz(void){
	// Dependent: config_XT1_GPIO();
// Startup clock system with max DCO setting ~8MHz
  CSCTL0_H = CSKEY_H;                     // Unlock CS registers
  CSCTL1 = DCOFSEL_3 | DCORSEL;           // Set DCO to 8MHz
  //          ACLK      SMCLK     MCLK
  CSCTL2 = SELA__LFXTCLK | SELS__DCOCLK | SELM__DCOCLK;
  CSCTL3 = DIVA__1 | DIVS__32 | DIVM__1;   // Set all dividers
  CSCTL4 &= ~LFXTOFF;
   do
   {
     CSCTL5 &= ~LFXTOFFG;                // Clear XT1 fault flag
     SFRIFG1 &= ~OFIFG;
   } while (SFRIFG1 & OFIFG);              // Test oscillator fault flag
  CSCTL0_H = 0;                           // Lock CS registers
}

void config_ACLK_32k_DCO_8MHz_SMCLK(void){
// Startup clock system with max DCO setting ~8MHz
  CSCTL0_H = CSKEY_H;                     // Unlock CS registers
  CSCTL1 = DCOFSEL_3 | DCORSEL;           // Set DCO to 8MHz
  //          ACLK      SMCLK     MCLK
  CSCTL2 = SELA__LFXTCLK | SELS__DCOCLK | SELM__DCOCLK;
  CSCTL3 = DIVA__1 | DIVS__1 | DIVM__1;   // Set all dividers
  CSCTL4 &= ~LFXTOFF;
   do
   {
     CSCTL5 &= ~LFXTOFFG;                // Clear XT1 fault flag
     SFRIFG1 &= ~OFIFG;
   } while (SFRIFG1 & OFIFG);              // Test oscillator fault flag
  CSCTL0_H = 0;                           // Lock CS registers
}

void config_DCO_1MHz(void){
// Startup clock system with max DCO setting ~8MHz
  CSCTL0_H = CSKEY_H;                     // Unlock CS registers
//  CSCTL1 = 0x0000;
//  CSCTL1 = DCOFSEL_3 | DCORSEL;           // Set DCO to 8MHz
//          ACLK      SMCLK     MCLK
  CSCTL2 = SELA__VLOCLK | SELS__DCOCLK | SELM__DCOCLK;
  CSCTL3 = DIVA__1 | DIVS__1 | DIVM__1;   // Set all dividers
  CSCTL0_H = 0;                           // Lock CS registers
}
