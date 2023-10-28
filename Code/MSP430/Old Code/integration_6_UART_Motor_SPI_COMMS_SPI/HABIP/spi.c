/*
 * SPI.c
 *
 *  Created on: Apr 11, 2017
 *      Author: Lincster
 */
#include <msp430.h>
#include "common.h"
#include "spi.h"
#include "string.h"

// SPI UD Variables
// Master (mst)
volatile int spi_mst_fsm_state = MST_IDLE;
char spi_mst_read_buffer[BUFF_LEN]={};
char spi_mst_read_message[MSG_LEN]={};
char spi_mst_send_message[MSG_LEN]={};
char spi_mst_send_buffer[BUFF_LEN]={};
volatile int spi_mst_index = 0;
volatile int spi_mst_write_index = 0;
volatile int spi_mst_read_index = 0;
volatile char spi_mst_tx_data = '\0';
volatile int spi_mst_readDoneFG = 0;
volatile int spi_mst_sendDoneFG = 0;
volatile int spi_mst_send_only = 0;
volatile int spi_mst_time_out_happened = 0;
#define TIMEOUT 10000
volatile int spi_mst_time_out_cnt = 0;

// Slave (slv)
volatile int spi_slv_fsm_state = LISTENING_FOR_COMMAND;
char spi_slv_read_buffer[BUFF_LEN]={};
char spi_slv_read_message[MSG_LEN]={};
char spi_slv_send_message[MSG_LEN]={};
char spi_slv_send_buffer[BUFF_LEN]={};
volatile int spi_slv_index = 0;
volatile int spi_slv_req_data = 0;
volatile int spi_slv_data_available = 0;
volatile int spi_slv_write_index = 0;
volatile int spi_slv_read_index = 0;
volatile char spi_slv_tx_data = '\0';
volatile int spi_slv_message_to_parse = 0;

void config_SPI_B0_Master_GPIO(void){
    // Configure SPI GPIO for Host MSP (MSP-MSP)
    // STE/SS & SIMO & SOMI
    P1SEL0 &= ~(BIT3 | BIT6 | BIT7);
    P1SEL1 |= (BIT3 | BIT6 | BIT7);
    // SCLK
    P2SEL0 &= ~(BIT2);
    P2SEL1 |= (BIT2);
    activate_GPIO_config();
}

void config_SPI_B1_Slave_GPIO(void){
    // Configure SPI GPIO for Host MSP (COMMS-MSP)
    P5SEL1 &= ~(BIT0 | BIT1 | BIT2 | BIT3);        // USCI_B1 SCLK, MOSI,
    P5SEL0 |= (BIT0 | BIT1 | BIT2 | BIT3);         // STE, and MISO pin
    activate_GPIO_config();
}

void config_SPI_A0_Slave_GPIO(void){
    // Configure SPI GPIO for Slave MSP (MSP-MSP)
    // SIMO & SOMI
    P2SEL0 &= ~(BIT0 | BIT1);
    P2SEL1 |= (BIT0 | BIT1);
    // STE/SS & SCLK
    P1SEL0 &= ~(BIT4 | BIT5);
    P1SEL1 |= (BIT4 | BIT5);
    activate_GPIO_config();
}

void config_SPI_B0_Master_ACLK(void){
    /*
     * Dependencies:
     * config_SPI_B0_Master_GPIO();
     * config_XT1_ACLK_32768Hz_DCO_1MHz();
     */
// Configure USCI_B0 for SPI operation
    UCB0CTLW0 = UCSWRST;                    // **Put state machine in reset**
    UCB0CTLW0 |= UCMST | UCSYNC | UCCKPL | UCMSB | UCMODE_1 | UCSTEM; // 4-pin, 8-bit SPI master
                                            // Clock polarity high, MSB
    UCB0CTLW0 |= UCSSEL__ACLK;              // ACLK
    UCB0BRW = 0x02;                         // /2
    //UCB0MCTLW = 0;                          // No modulation
    UCB0CTLW0 &= ~UCSWRST;                  // **Initialize USCI state machine**
}
void config_SPI_B0_Master_SMCLK(void){
    /*
     * Dependencies:
     * config_SPI_B0_Master_GPIO();
     * config_ACLK_XT1_32KHz_DCO_8MHz_SMCLK_250KHz();
     */
// Configure USCI_B0 for SPI operation
    UCB0CTLW0 = UCSWRST;                    // **Put state machine in reset**
    UCB0CTLW0 |= UCMST | UCSYNC | UCCKPH | UCMSB | UCMODE_1 | UCSTEM; // 4-pin, 8-bit SPI master
                                            // Clock polarity high, MSB
    UCB0CTLW0 |= UCSSEL__SMCLK;             // SMCLK
    UCB0BRW = 0x08;                         // 	/8
    //UCB0MCTLW = 0;                          // No modulation
    UCB0CTLW0 &= ~UCSWRST;                  // **Initialize USCI state machine**
}
void config_SPI_B1_Slave(void){
    /*
     * Dependencies:
     * config_SPI_B1_Slave_GPIO();
     * config_XT1_ACLK_32768Hz_DCO_1MHz();
     */
// Configure USCI_B1 for SPI operation
   UCB1CTLW0 = UCSWRST;                    // **Put state machine in reset**
   UCB1CTLW0 |= UCSYNC | UCCKPH | UCMSB | UCMODE_2;   // 4-pin, 8-bit SPI slave
                                           // Clock polarity high, MSB, SS active low
   UCB1BRW = 0x02;                         // /2
   UCB1CTLW0 &= ~UCSWRST;                  // **Initialize USCI state machine**
   UCB1IE |= UCRXIE;                       // Enable USCI_B1 RX interrupt
}
void config_SPI_A0_Slave(void){
	/*
	 * Dependencies:
	 * config_SPI_A0_Slave_GPIO();
	 * config_XT1_ACLK_32768Hz_DCO_1MHz();
	 */
// Configure USCI_A0 for SPI operation
   UCA0CTLW0 = UCSWRST;                    // **Put state machine in reset**
   //UCA0CTLW0 |= UCSYNC | UCCKPL | UCMSB | UCMODE_1 | UCSTEM; // 4-pin, 8-bit SPI slave
   UCA0CTLW0 |= UCSYNC | UCCKPH | UCMSB | UCMODE_1; // 4-pin, 8-bit SPI slave

                                           // Clock polarity high, MSB
   UCA0BRW = 0x02;                         // /2
   UCA0MCTLW = 0;                          // No modulation
   UCA0CTLW0 &= ~UCSWRST;                  // **Initialize USCI state machine**
   UCA0IE |= UCRXIE;                       // Enable USCI_A0 RX interrupt
}
void SPI_command_host_to_slave(char* message,volatile int* read_done){
    // Kick of interactions
    UCB0IE |= UCRXIE; // char by char interaction
    strcpy(spi_mst_send_message,message);
    spi_mst_fsm_state = CHECKING_IF_SLAVE_READY;
    TX_B0('R');
    // Chill until have successfully received a reply from slave
    // or until flag that shows need to reset?
    while(*read_done == 0 && spi_mst_time_out_happened == 0){
//        __bis_SR_register(LPM0_bits); // Enter LPM0
    	spi_mst_time_out_cnt++;
        __no_operation();
        if(spi_mst_time_out_cnt >= TIMEOUT){
        	spi_mst_time_out_happened = 1;
        }
    }
    spi_mst_send_only = 0;
    spi_mst_time_out_happened = 0;
    spi_mst_time_out_cnt = 0;
    spi_mst_fsm_state = MST_IDLE;
    *read_done = 0;
    UCB0IE &= ~UCRXIE; // Clear RX interrupt so no noise read in?
}

void SPI_command_host_to_slave_no_response(char* message,volatile int* send_done){
    // Kick of interactions
    UCB0IE |= UCRXIE; // char by char interaction
    strcpy(spi_mst_send_message,message);
    spi_mst_send_only = 1;
    spi_mst_fsm_state = CHECKING_IF_SLAVE_READY;
    TX_B0('R');
    // Chill until have successfully received a reply from slave
    // or until flag that shows need to reset?
    while(*send_done == 0 && spi_mst_time_out_happened == 0){
//        __bis_SR_register(LPM0_bits); // Enter LPM0
    	spi_mst_time_out_cnt++;
        __no_operation();
        if(spi_mst_time_out_cnt >= TIMEOUT){
        	spi_mst_time_out_happened = 1;
        }
    }
    spi_mst_send_only = 0;
    spi_mst_time_out_happened = 0;
    spi_mst_time_out_cnt = 0;
    spi_mst_fsm_state = MST_IDLE;
    *send_done = 0;
    UCB0IE &= ~UCRXIE; // Clear RX interrupt so no noise read in?
}
