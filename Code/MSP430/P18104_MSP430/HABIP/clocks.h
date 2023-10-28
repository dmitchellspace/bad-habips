/*
 * CLOCKS.h
 *
 *  Created on: Apr 11, 2017
 *      Author: Lincster
 */

#ifndef HABIP_CLOCKS_H_
#define HABIP_CLOCKS_H_
// #include <msp430.h>
// #include <driverlib.h>

  // Clocks
void config_XT1_GPIO(void);
void config_XT1_ACLK_32768Hz(void);
void config_XT1_ACLK_32768Hz_DCO_1MHz(void);
void config_DCO_8MHz(void);
void config_ACLK_32k_DCO_8MHz(void);
void config_DCO_1MHz(void);
void config_ACLK_XT1_32KHz_DCO_8MHz_SMCLK_250KHz(void);

#endif /* HABIP_CLOCKS_H_ */
