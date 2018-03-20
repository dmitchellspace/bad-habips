/*
 * habip.h
 *
 *  Created on: Apr 11, 2017
 *      Author: Lincster
 */

#ifndef HABIP_HABIP_H_
#define HABIP_HABIP_H_

#include "battery.h"
#include "clocks.h"
#include "common.h"
#include "command_interface.h"
#include "spi.h"
#include "uart.h"

#endif /* HABIP_HABIP_H_ */

//http://www.embedded.com/electronics-blogs/beginner-s-corner/4023801/Introduction-to-the-Volatile-Keyword

// A variable should be declared volatile whenever its value could change unexpectedly. In practice, only three types of variables could change:

//Memory-mapped peripheral registers
//Global variables modified by an interrupt service routine
//Global variables within a multi-threaded application
