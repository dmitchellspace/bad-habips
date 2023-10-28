/*
 * common.c
 *
 *  Created on: Apr 11, 2017
 *      Author: Lincster
 */
// #include <common.h>
#include <msp430.h>
#include "driverlib.h" //added for blink LED
#include "common.h"

//*********************************************************************************************************//

void activate_GPIO_config(void){
  /*Scope: Run after configured GPIO to activate the config*/
  // Disable the GPIO power-on default high-impedance mode to activate
  // previously configured port settings
    PMM_unlockLPM5();
  //PM5CTL0 &= ~LOCKLPM5;
}
void config_DS4_LED(void){
    GPIO_setAsOutputPin(          // config P1.0 (DS4 LED) GPIO as output
            GPIO_PORT_P1,
            GPIO_PIN0
            );
    GPIO_setOutputLowOnPin(
    		GPIO_PORT_P1,
			GPIO_PIN0
			);
    activate_GPIO_config();
}
void Toggle_ON_OFF_DS4_LED(void){
  /*Scope: Toggles LED ON/OFF once when message received*/
  /*Dependencies:
  #include <driverlib.h>
  config_DS4_LED();
  */
  GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
  // Delay
  delay_LED();
    // Toggle P1.0 output
  GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);
  // Delay
  delay_LED();
  // Toggle P1.0 output
  GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);
}

void delay_LED(void){
  int i;
  int j;
  for(i=500;i>0;i--){
    __no_operation();
    for(j=500;j>0;j--) {
      __no_operation();
    }
  }
}
