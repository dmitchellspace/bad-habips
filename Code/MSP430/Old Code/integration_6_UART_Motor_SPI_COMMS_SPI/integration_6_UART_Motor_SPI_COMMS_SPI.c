/* --COPYRIGHT--,BSD_EX
 * Copyright (c) 2015, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************
 *
 *                       MSP430 CODE EXAMPLE DISCLAIMER
 *
 * MSP430 code examples are self-contained low-level programs that typically
 * demonstrate a single peripheral function or device feature in a highly
 * concise manner. For this the code may rely on the device's power-on default
 * register values and settings such as the clock configuration and care must
 * be taken when combining code from several examples to avoid potential side
 * effects. Also see www.ti.com/grace for a GUI- and www.ti.com/msp430ware
 * for an API functional library-approach to peripheral configuration.
 *
 * --/COPYRIGHT--*/
//******************************************************************************

//******************************************************************************
#include <msp430.h>
#include "driverlib.h"
#include "habip.h"
#include "stdlib.h"
#include <string.h>

// UART UD Variables
extern char uart_read_buffer[4][MSG_LEN];
extern char uart_read_message[4][MSG_LEN];
extern volatile int uart_fsm_state[4];
extern char uart_read_message_buffer[4][PI_HAT_SENSOR_CNT][MSG_LEN];
extern char uart_read_message_buffer_status[4][PI_HAT_SENSOR_CNT];
extern volatile int msg_buffer_index[4];
extern volatile int uart_index[4];
extern volatile int uart_read_index[4];
extern volatile int uart_status_index[4];
extern volatile int uart_status[4];

// SPI UD Variables
// Master (mst)
extern volatile int spi_mst_fsm_state;
extern char spi_mst_read_buffer[BUFF_LEN];
extern char spi_mst_read_message[MSG_LEN];
extern char spi_mst_send_message[MSG_LEN];
extern char spi_mst_send_buffer[BUFF_LEN];
extern volatile int spi_mst_index;
extern volatile int spi_mst_write_index;
extern volatile int spi_mst_read_index;
extern volatile char spi_mst_tx_data;
extern volatile int spi_mst_readDoneFG;
extern volatile int spi_mst_sendDoneFG;
extern volatile int spi_mst_send_only;
volatile char spi_mst_rx_data;
extern volatile int spi_mst_time_out_happened;

// Slave (slv)
extern volatile int spi_slv_fsm_state;
extern char spi_slv_read_buffer[BUFF_LEN];
extern char spi_slv_read_message[MSG_LEN];
extern char spi_slv_send_message[MSG_LEN];
extern char spi_slv_send_buffer[BUFF_LEN];
extern volatile int spi_slv_index;
extern volatile int spi_slv_req_data;
extern volatile int spi_slv_data_available;
extern volatile int spi_slv_write_index;
extern volatile int spi_slv_read_index;
extern volatile char spi_slv_tx_data;

extern char respond_all_data_msg[1024];

volatile int timer_counter = 0;
int motor_en_cnt = 0;
#define MOTOR_CNT 6
//*********************************************************************************************************//
int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;               // Stop Watchdog

// Configure GPIO
    config_DS4_LED(); // heartbeat
    config_UART_GPIO(0);
    config_UART_GPIO(1);
    config_UART_GPIO(2);
    config_UART_GPIO(3);
    config_SPI_B0_Master_GPIO();
    config_SPI_B1_Slave_GPIO();
    config_RST_PI_GPIO();
    config_CUTDOWN_GPIO();
    config_XT1_GPIO();

// temp
    P1DIR |= (BIT2);
    P1OUT &= ~(BIT2);
    P7DIR |= (BIT0);
    P7OUT &= ~(BIT0);

// Temp talk to Motor MSP for reaction wheel.
    P8DIR |= (BIT0);
    P8OUT &= ~(BIT0);

// Configure Clock
    config_ACLK_XT1_32KHz_DCO_8MHz_SMCLK_250KHz();

// Configure Comms
    config_UART_9600_SMCLK_250KHz(0);
    config_UART_9600_SMCLK_250KHz(1);
    config_UART_9600_SMCLK_250KHz(2);
    config_UART_9600_SMCLK_250KHz(3);
    config_SPI_B0_Master_SMCLK();
    config_SPI_B1_Slave();

// Timer
    TA1CCTL0 = CCIE;                        // TACCR0 interrupt enabled
    TA1CCR0 = 65000;						// ~2 seconds
    TA1CTL = TASSEL__ACLK | MC__UP;         // ACLK, up mode

    __no_operation();
    __bis_SR_register(GIE);

//    P8OUT |= BIT0;

// Begin Main Code
while(1){
	P1OUT ^= BIT2;
	toggle_heartbeat();
	__bis_SR_register(LPM0_bits); // Enter LPM0
	__no_operation();
	if(spi_slv_fsm_state == PARSING_COMMAND){
		parse_cmd_from_comms(spi_slv_read_message);
	}
	if(timer_counter == 1){
		grab_all_pi_hat(0);
		timer_counter++;
	}
	else if(timer_counter == 3){
		grab_all_pi_hat(1);
		timer_counter++;
	}
	else if(timer_counter == 5){
		grab_all_pi_hat(2);
		timer_counter++;
	}
	else if(timer_counter == 7){
		grab_all_pi_hat(3);
//		if(motor_en_cnt == MOTOR_CNT){
//			P8OUT ^= (BIT0);
//			motor_en_cnt = 0;
//		}
//		else {
//			motor_en_cnt++;
//		}
		timer_counter++;
	}
	else if(timer_counter == 9){
//		grab_all_motor_msp();
		timer_counter++;
	}
	UART_parse(0);
	UART_parse(1);
	UART_parse(2);
	UART_parse(3);
//            parse_cmd_from_comms("{01}");
	__no_operation();
}
// End Main Code

//	while(1) ; // catchall for debug
}
// Timer A1 interrupt service routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = TIMER1_A0_VECTOR
__interrupt void Timer1_A0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER1_A0_VECTOR))) Timer1_A0_ISR (void)
#else
#error Compiler not supported!
#endif
{
    if(timer_counter == 10){
        timer_counter = 0;
    }
    else{
        timer_counter++;
    }
    __bic_SR_register_on_exit(LPM0_bits);
}
//*********************************************************************************************************//
//*********************************		TALK TO PI HATS UART	  *****************************************//
//*********************************************************************************************************//
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=EUSCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(EUSCI_A0_VECTOR))) USCI_A0_ISR (void)
#else
#error Compiler not supported!
#endif
{
    switch(__even_in_range(UCA0IV, USCI_UART_UCTXCPTIFG))
    {
        case USCI_NONE: break;
        case USCI_UART_UCRXIFG:
        	UART_ISR(0);
			break;
        case USCI_UART_UCTXIFG: break;
        case USCI_UART_UCSTTIFG: break;
        case USCI_UART_UCTXCPTIFG: break;
        default: break;
    }
}
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=EUSCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(EUSCI_A1_VECTOR))) USCI_A1_ISR (void)
#else
#error Compiler not supported!
#endif
{
    switch(__even_in_range(UCA1IV, USCI_UART_UCTXCPTIFG))
    {
        case USCI_NONE: break;
        case USCI_UART_UCRXIFG:
        	UART_ISR(1);
			break;
        case USCI_UART_UCTXIFG: break;
        case USCI_UART_UCSTTIFG: break;
        case USCI_UART_UCTXCPTIFG: break;
        default: break;
    }
}
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=EUSCI_A2_VECTOR
__interrupt void USCI_A2_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(EUSCI_A2_VECTOR))) USCI_A2_ISR (void)
#else
#error Compiler not supported!
#endif
{
    switch(__even_in_range(UCA2IV, USCI_UART_UCTXCPTIFG))
    {
        case USCI_NONE: break;
        case USCI_UART_UCRXIFG:
        	UART_ISR(2);
			break;
        case USCI_UART_UCTXIFG: break;
        case USCI_UART_UCSTTIFG: break;
        case USCI_UART_UCTXCPTIFG: break;
        default: break;
    }
}
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=EUSCI_A3_VECTOR
__interrupt void USCI_A3_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(EUSCI_A3_VECTOR))) USCI_A3_ISR (void)
#else
#error Compiler not supported!
#endif
{
    switch(__even_in_range(UCA3IV, USCI_UART_UCTXCPTIFG))
    {
        case USCI_NONE: break;
        case USCI_UART_UCRXIFG:
        	UART_ISR(3);
			break;
        case USCI_UART_UCTXIFG: break;
        case USCI_UART_UCSTTIFG: break;
        case USCI_UART_UCTXCPTIFG: break;
        default: break;
    }
}
//*********************************************************************************************************//
//*********************************     TALK TO COMMS PI SPI     *****************************************//
//*********************************************************************************************************//
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=EUSCI_B1_VECTOR
__interrupt void USCI_B1_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(EUSCI_B1_VECTOR))) USCI_B1_ISR (void)
#else
#error Compiler not supported!
#endif
{
    switch(__even_in_range(UCB1IV, USCI_SPI_UCTXIFG))
    {
        case USCI_NONE: break;
        case USCI_SPI_UCRXIFG:
            spi_slv_read_buffer[spi_slv_index] = UCB1RXBUF;
             switch(spi_slv_fsm_state)
             {
                case LISTENING_FOR_COMMAND:
                    if(spi_slv_read_buffer[spi_slv_index] == '{'){
                      spi_slv_fsm_state = CAPTURING_COMMAND;
                      spi_slv_read_message[0] = '{';
                      spi_slv_read_index = 1;
                      TX_B1_SPI('C');
                    }
                    else{
                        TX_B1_SPI('L');
                    }
                    break;
                case CAPTURING_COMMAND:
                    spi_slv_read_message[spi_slv_read_index++] = spi_slv_read_buffer[spi_slv_index];
                    if(spi_slv_read_buffer[spi_slv_index] == '}'){
                      spi_slv_fsm_state = PARSING_COMMAND;
                      spi_slv_read_message[spi_slv_read_index] = '\0';
                      spi_slv_write_index = 0;
                      TX_B1_SPI('D');
                      __bic_SR_register_on_exit(LPM0_bits);
                    }
                    else {
                      TX_B1_SPI('C');
                    }
                    break;
                case PARSING_COMMAND:
                    TX_B1_SPI('P');
                    break;
                case OBTAINING_DATA:
                    TX_B1_SPI('O');
                    break;
                case RESPONDING_WITH_DATA:
                    spi_slv_tx_data = spi_slv_send_message[spi_slv_write_index++];
                    if(spi_slv_tx_data == '}'){
                      spi_slv_fsm_state = LISTENING_FOR_COMMAND;
                    }
                    TX_B1_SPI(spi_slv_tx_data);
                    break;
                case RESPONDING_WITH_ALL_DATA:
                    spi_slv_tx_data = respond_all_data_msg[spi_slv_write_index++];
                    if(spi_slv_tx_data == '$'){
                      spi_slv_fsm_state = LISTENING_FOR_COMMAND;
                    }
                    else {
                        TX_B1_SPI(spi_slv_tx_data);
                    }
                    break;
                default:

                    break;
             }
            if(spi_slv_index == BUFF_LEN-1){
                spi_slv_index = 0;
            }
              else {
                spi_slv_index++;
              }
            break;
        case USCI_SPI_UCTXIFG:
            break;
        default: break;
    }
}
//*********************************************************************************************************//
//*********************************************************************************************************//
//*********************************		TALK TO MOTOR MSP SPI	  *****************************************//
//*********************************************************************************************************//
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=EUSCI_B0_VECTOR
__interrupt void USCI_B0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(EUSCI_B0_VECTOR))) USCI_B0_ISR (void)
#else
#error Compiler not supported!
#endif
{
    switch(__even_in_range(UCB0IV, USCI_SPI_UCTXIFG))
    {
        case USCI_NONE: break;
        case USCI_SPI_UCRXIFG:
//        	spi_mst_read_buffer[spi_mst_index] = UCB0RXBUF;
        	spi_mst_rx_data = UCB0RXBUF;
			spi_mst_read_buffer[spi_mst_index] = spi_mst_rx_data;
             switch(spi_mst_fsm_state)
             {
                case MST_IDLE:
//                	spi_mst_fsm_state = 6;
//                	TX_B0('M');
                    break;
//                case 6:
//                	__no_operation();
//                	break;
                case CHECKING_IF_SLAVE_READY:
                    if(spi_mst_read_buffer[spi_mst_index] == 'L'){
                      spi_mst_fsm_state = SENDING_COMMAND;
                      spi_mst_write_index = 0;
                    }
                    else {
                    	spi_mst_fsm_state = spi_mst_fsm_state;
                    }
                    TX_B0('R');
                    break;
                case SENDING_COMMAND:
                    spi_mst_tx_data = spi_mst_send_message[spi_mst_write_index++];
                    if(spi_mst_tx_data == '}'){
                      spi_mst_sendDoneFG = 1;
                      if(spi_mst_send_only == 1){
                        spi_mst_fsm_state = MST_IDLE;
                      }
                      else{
                        spi_mst_fsm_state = SPI_LISTENING_FOR_RESPONSE;
                      }
                    }
                    else {
                    	spi_mst_fsm_state = spi_mst_fsm_state;
                    }
                    TX_B0(spi_mst_tx_data);
                    break;
                case SPI_LISTENING_FOR_RESPONSE:
                    if(spi_mst_read_buffer[spi_mst_index] == '{'){
                      spi_mst_fsm_state = SPI_CAPTURING_RESPONSE;
                      spi_mst_read_message[0] = '{';
                      spi_mst_read_index = 1;
                      TX_B0('C');
                    }
                    else{
                    	spi_mst_fsm_state = spi_mst_fsm_state;
                        TX_B0('L');
                    }
                    break;
                case SPI_CAPTURING_RESPONSE:
                    spi_mst_read_message[spi_mst_read_index] = spi_mst_read_buffer[spi_mst_index];
                    spi_mst_read_index++;
                    if(spi_mst_rx_data == '}'){
                      spi_mst_fsm_state = MST_IDLE;
                      spi_mst_readDoneFG = 1;
                      spi_mst_read_message[spi_mst_read_index] = '\0';
                      TX_B0('D');
                      __bic_SR_register_on_exit(LPM0_bits);
                    }
                    else {
                    	spi_mst_fsm_state = spi_mst_fsm_state;
                      TX_B0('C');
                    }
                    break;
                default:
//                	spi_mst_fsm_state = MST_IDLE; // Note: TEMP BANDAID ONLY TODO:
                    break;
             }
        	if(spi_mst_index == BUFF_LEN-1){
        		spi_mst_index = 0;
        	}
             else {
                   spi_mst_index++;
             }
            // Wake up to setup next TX
            __bic_SR_register_on_exit(LPM0_bits);
            break;
        case USCI_SPI_UCTXIFG:
            break;
        default: break;
    }
}
//*********************************************************************************************************//
