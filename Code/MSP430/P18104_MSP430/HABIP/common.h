/*
 * COMMON.h
 *
 *  Created on: Apr 11, 2017
 *      Author: Lincster
 */

#ifndef HABIP_COMMON_H_
#define HABIP_COMMON_H_

  // General
void activate_GPIO_config(void);
void config_HEARTBEAT_LED(void);
void Toggle_ON_OFF_HEARTBEAT_LED(void);
void delay_LED(void);

// Constants
#define START_CHAR 0x7B
#define END_CHAR 0x7D
#define MSG_LEN 18 // Longest = 17 characters Plus 1 for '\0'
#define BUFF_LEN 93 // Longest data in Bytes (24-bit request + (x30) 24-bit message 5's = 93 Bytes)

#endif /* HABIP_COMMON_H_ */
