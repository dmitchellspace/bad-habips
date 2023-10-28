/*
 * UART.c
 *
 *  Created on: Apr 11, 2017
 *      Author: Lincster
 */
#include <msp430.h>
#include <driverlib.h>
#include "common.h"
#include "uart.h"
#include "command_interface.h"
#include "string.h"

// UART UD Variables
char uart_read_buffer[4][BUFF_LEN]={{}};
char uart_read_message[4][MSG_LEN]={{}};
char uart_read_message_buffer[4][PI_HAT_SENSOR_CNT][MSG_LEN]={{{}}};
char uart_read_message_buffer_status[4][PI_HAT_SENSOR_CNT]={{AVAILABLE}};
volatile unsigned int msg_buffer_index[4]={0};
volatile unsigned int uart_index[4] = {0};
volatile unsigned int uart_read_index[4] = {0};
volatile unsigned int uart_status_index[4] = {0};
volatile int uart_status[4] = {0};
volatile int uart_fsm_state[4] = {{LISTENING_FOR_RESPONSE}};

void UART_parse(int brd_num){
	int i;
	if(uart_status[brd_num]>0){
		for (i=0;i<PI_HAT_SENSOR_CNT;i++){
			if(uart_read_message_buffer_status[brd_num][i]==VALID){
				parse_response_pi_hat(brd_num,uart_read_message_buffer[brd_num][i]);
				uart_read_message_buffer_status[brd_num][i] = AVAILABLE;
				uart_read_message_buffer[brd_num][i][0] = '\0';
				uart_status[brd_num]--;
				__no_operation();
				if(uart_status[brd_num]==0){
					break;
				}
			}
		}
	}
}

void config_UART_GPIO(int brd_num){
	switch(brd_num)
	{
	case 0:
		// USCI_A0 UART operation
		P2SEL1 |= (BIT0 | BIT1);
		P2SEL0 &= ~(BIT0 | BIT1);
		break;
	case 1:
		// USCI_A1 UART operation
		P2SEL1 |= (BIT5 | BIT6);
		P2SEL0 &= ~(BIT5 | BIT6);
		break;
	case 2:
		// USCI_A2 UART operation
		P5SEL1 &= ~(BIT4 | BIT5);
		P5SEL0 |= (BIT4 | BIT5);
		break;
	case 3:
		// USCI_A3 UART operation
		P6SEL1 &= ~(BIT0 | BIT1);
		P6SEL0 |= (BIT0 | BIT1);
		break;
	default: break;
	}
	activate_GPIO_config();
}
void config_UART_9600_ACLK_32768Hz(int brd_num){
	/* Dependencies
	 * config_XT1_ACLK_32768Hz();
	 */
	switch(brd_num)
	{
	case 0:
		// Configure USCI_A0 for UART mode
		UCA0CTLW0 = UCSWRST;                    // Put eUSCI in reset
		UCA0CTLW0 = 0x0000;
		UCA0CTLW0 |= UCSSEL__ACLK;              // CLK = ACLK
		UCA0BRW = 3;                            // 9600 baud
		UCA0MCTLW |= 0x5300;                    // 32768/9600 - INT(32768/9600)=0.41
												// UCBRSx value = 0x53 (See UG)
		UCA0CTLW0 &= ~UCSWRST;                  // Initialize eUSCI
		break;
	case 1:
		// Configure USCI_A1 for UART mode
		UCA1CTLW0 = UCSWRST;                    // Put eUSCI in reset
		UCA1CTLW0 = 0x0000;
		UCA1CTLW0 |= UCSSEL__ACLK;              // CLK = ACLK
		UCA1BRW = 3;                            // 9600 baud
		UCA1MCTLW |= 0x5300;                    // 32768/9600 - INT(32768/9600)=0.41
												// UCBRSx value = 0x53 (See UG)
		UCA1CTLW0 &= ~UCSWRST;                  // Initialize eUSCI
		break;
	case 2:
		// Configure USCI_A2 for UART mode
		UCA2CTLW0 = UCSWRST;                    // Put eUSCI in reset
		UCA2CTLW0 = 0x0000;
		UCA2CTLW0 |= UCSSEL__ACLK;              // CLK = ACLK
		UCA2BRW = 3;                            // 9600 baud
		UCA2MCTLW |= 0x5300;                    // 32768/9600 - INT(32768/9600)=0.41
												// UCBRSx value = 0x53 (See UG)
		UCA2CTLW0 &= ~UCSWRST;                  // Initialize eUSCI
		break;
	case 3:
		// Configure USCI_A3 for UART mode
		UCA3CTLW0 = UCSWRST;                    // Put eUSCI in reset
		UCA3CTLW0 = 0x0000;
		UCA3CTLW0 |= UCSSEL__ACLK;              // CLK = ACLK
		UCA3BRW = 3;                            // 9600 baud
		UCA3MCTLW |= 0x5300;                    // 32768/9600 - INT(32768/9600)=0.41
												// UCBRSx value = 0x53 (See UG)
		UCA3CTLW0 &= ~UCSWRST;                  // Initialize eUSCI
		break;
	default: break;
	}
}
void config_UART_9600_SMCLK_8MHz(int brd_num){
	/* Dependencies:
	 * config_DCO_8MHz();
	 */
	// Baud Rate calculation
	// 8000000/(16*9600) = 52.083
	// Fractional portion = 0.083
	// User's Guide Table 21-4: UCBRSx = 0x04
	// UCBRFx = int ( (52.083-52)*16) = 1
	//	UCAXBRW = 52;                           // 8000000/16/9600
	switch(brd_num)
	{
	case 0:
		// Configure USCI_A0 for UART mode
		UCA0CTLW0 = UCSWRST;                    // Put eUSCI in reset
		UCA0CTLW0 |= UCSSEL__SMCLK;             // CLK = SMCLK
		UCA0BRW = 52;
		UCA0MCTLW |= UCOS16 | UCBRF_1 | 0x4900;
		UCA0CTLW0 &= ~UCSWRST;                  // Initialize eUSCI
		UCA0IE |= UCRXIE;                       // Enable USCI_A0 RX interrupt
		break;
	case 1:
		// Configure USCI_A1 for UART mode
		UCA1CTLW0 = UCSWRST;                    // Put eUSCI in reset
		UCA1CTLW0 |= UCSSEL__SMCLK;             // CLK = SMCLK
		UCA1BRW = 52;
		UCA1MCTLW |= UCOS16 | UCBRF_1 | 0x4900;
		UCA1CTLW0 &= ~UCSWRST;                  // Initialize eUSCI
		UCA1IE |= UCRXIE;                       // Enable USCI_A1 RX interrupt
		break;
	case 2:
		// Configure USCI_A2 for UART mode
		UCA2CTLW0 = UCSWRST;                    // Put eUSCI in reset
		UCA2CTLW0 |= UCSSEL__SMCLK;             // CLK = SMCLK
		UCA2BRW = 52;
		UCA2MCTLW |= UCOS16 | UCBRF_1 | 0x4900;
		UCA2CTLW0 &= ~UCSWRST;                  // Initialize eUSCI
		UCA2IE |= UCRXIE;                       // Enable USCI_A2 RX interrupt
		break;
	case 3:
		// Configure USCI_A3 for UART mode
		UCA3CTLW0 = UCSWRST;                    // Put eUSCI in reset
		UCA3CTLW0 |= UCSSEL__SMCLK;             // CLK = SMCLK
		UCA3BRW = 52;
		UCA3MCTLW |= UCOS16 | UCBRF_1 | 0x4900;
		UCA3CTLW0 &= ~UCSWRST;                  // Initialize eUSCI
		UCA3IE |= UCRXIE;                       // Enable USCI_A3 RX interrupt
		break;
	default: break;
	}
}
void config_UART_9600_SMCLK_250KHz(int brd_num){
	/* Dependencies:
	 * config_ACLK_XT1_32KHz_DCO_8MHz_SMCLK_250KHz();
	 */
	// Baud Rate calculation
	// 250000/(16*9600) = 1.6276
	// Fractional portion = 0.6276
	// User's Guide Table 21-4: 250000/9600 frac = 0.0417 UCBRSx = 0x01
	// UCBRFx = int ( (1.6276-1)*16) = 10
	//	UCAXBRW = 1;                           // 250000/16/9600
	switch(brd_num)
	{
	case 0:
		// Configure USCI_A0 for UART mode
		UCA0CTLW0 = UCSWRST;                    // Put eUSCI in reset
		UCA0CTLW0 |= UCSSEL__SMCLK;             // CLK = SMCLK
		UCA0BRW = 1;
		UCA0MCTLW |= UCOS16 | UCBRF_10 | 0x0100;
		UCA0CTLW0 &= ~UCSWRST;                  // Initialize eUSCI
		UCA0IE |= UCRXIE;                       // Enable USCI_A0 RX interrupt
		break;
	case 1:
		// Configure USCI_A1 for UART mode
		UCA1CTLW0 = UCSWRST;                    // Put eUSCI in reset
		UCA1CTLW0 |= UCSSEL__SMCLK;             // CLK = SMCLK
		UCA1BRW = 1;
		UCA1MCTLW |= UCOS16 | UCBRF_10 | 0x0100;
		UCA1CTLW0 &= ~UCSWRST;                  // Initialize eUSCI
		UCA1IE |= UCRXIE;                       // Enable USCI_A1 RX interrupt
		break;
	case 2:
		// Configure USCI_A2 for UART mode
		UCA2CTLW0 = UCSWRST;                    // Put eUSCI in reset
		UCA2CTLW0 |= UCSSEL__SMCLK;             // CLK = SMCLK
		UCA2BRW = 1;
		UCA2MCTLW |= UCOS16 | UCBRF_10 | 0x0100;
		UCA2CTLW0 &= ~UCSWRST;                  // Initialize eUSCI
		UCA2IE |= UCRXIE;                       // Enable USCI_A2 RX interrupt
		break;
	case 3:
		// Configure USCI_A3 for UART mode
		UCA3CTLW0 = UCSWRST;                    // Put eUSCI in reset
		UCA3CTLW0 |= UCSSEL__SMCLK;             // CLK = SMCLK
		UCA3BRW = 1;
		UCA3MCTLW |= UCOS16 | UCBRF_10 | 0x0100;
		UCA3CTLW0 &= ~UCSWRST;                  // Initialize eUSCI
		UCA3IE |= UCRXIE;                       // Enable USCI_A3 RX interrupt
		break;
	default: break;
	}
}

void UART_write_msg(int brd_num, char* message){
	int i;
	i = 0;
	switch(brd_num)
	{
	case 0:
		while(message[i] != '\0'){
			while(!(UCA0IFG&UCTXIFG));
			UCA0TXBUF = message[i];
			i++;
		}
//		while(!(UCA0IFG&UCTXIFG));
//		UCA0TXBUF = END_CHAR; // TODO: future dev decide on passing in {XX} or just XX
		break;
	case 1:
		while(message[i] != '\0'){
			while(!(UCA1IFG&UCTXIFG));
			UCA1TXBUF = message[i];
			i++;
		}
//		while(!(UCA1IFG&UCTXIFG));
//		UCA1TXBUF = END_CHAR; // TODO: future dev decide on passing in {XX} or just XX
		break;
	case 2:
		while(message[i] != '\0'){
			while(!(UCA2IFG&UCTXIFG));
			UCA2TXBUF = message[i];
			i++;
		}
//		while(!(UCA2IFG&UCTXIFG));
//		UCA2TXBUF = END_CHAR; // TODO: future dev decide on passing in {XX} or just XX
		break;
	case 3:
		while(message[i] != '\0'){
			while(!(UCA3IFG&UCTXIFG));
			UCA3TXBUF = message[i];
			i++;
		}
//		while(!(UCA3IFG&UCTXIFG));
//		UCA3TXBUF = END_CHAR; // TODO: future dev decide on passing in {XX} or just XX
		break;
	default: break;
	}
}
