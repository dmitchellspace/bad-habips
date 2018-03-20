/* --COPYRIGHT--,BSD_EX
 * Copyright (c) 2015, Texas Instruments Incorporated
 * All rights reserved.
 * *****************************************************************************
 * Originally written by: HABIP P17105
 * Heavily Edited & Commented by: Daniel Mitchell of HABIP P18104
 *******************************************************************************/
#include <msp430.h>
#include "driverlib.h"
#include "habip.h"
#include "stdlib.h"
#include <string.h>

#define SELECTED_LIMIT 4 //The number of times a desired video feed stays active before returning to cycle
#define GREEN_LED_BLINK_PERIOD 40000 //The number of main loops that happen for green LED to stay active when blinking
#define HEARTBEAT_LOOP_PERIOD 65000 //The number of loops that represent the heartbeat blinking period

// UART UD Variables
extern char uart_read_buffer[4][MSG_LEN];
extern volatile int uart_fsm_state[4];
extern char uart_read_message_buffer[4][PI_HAT_SENSOR_CNT][MSG_LEN];
extern char uart_read_message_buffer_status[4][PI_HAT_SENSOR_CNT];
extern volatile unsigned int msg_buffer_index[4];
extern volatile unsigned int uart_index[4];
extern volatile unsigned int uart_read_index[4];
extern volatile unsigned int uart_status_index[4];
extern volatile int uart_status[4];

// SPI UD Variables
// Master (mst)
extern volatile int spi_mst_fsm_state;
extern char spi_mst_read_buffer[BUFF_LEN];
extern char spi_mst_read_message[MSG_LEN];
extern char spi_mst_send_message[MSG_LEN];
extern char spi_mst_send_buffer[BUFF_LEN];
extern volatile unsigned int spi_mst_index;
extern volatile unsigned int spi_mst_write_index;
extern volatile unsigned int spi_mst_read_index;
extern volatile char spi_mst_tx_data;
extern volatile int spi_mst_readDoneFG;
extern volatile int spi_mst_sendDoneFG;
extern volatile int spi_mst_send_only;
volatile char spi_mst_rx_data;

// Slave (slv)
extern volatile int spi_slv_fsm_state;
extern char spi_slv_read_buffer[BUFF_LEN];
extern char spi_slv_read_message[MSG_LEN];
extern char spi_slv_send_message[MSG_LEN];
extern char spi_slv_send_buffer[BUFF_LEN];
extern volatile unsigned int spi_slv_index;
extern volatile unsigned int spi_slv_write_index;
extern volatile unsigned int spi_slv_read_index;
extern volatile char spi_slv_tx_data;

extern char respond_all_data_msg[1024];

volatile int timer_counter = 0;
int motor_en_cnt = 0;
#define MOTOR_CNT 6

//Variables that need to be organized
char b0_ISR_Flag = 0;   //Software flag for the B0 USCI
char b1_ISR_Flag = 0;   //Software flag for the B1 USCI
char uART1_ISR_Flag = 0; //Software flag for UART1 Receive
char uART2_ISR_Flag = 0; //Software flag for UART2 Receive
char uART3_ISR_Flag = 0; //Software flag for UART3 Receive
char uART4_ISR_Flag = 0; //Software flag for UART4 Receive
char timerA0Flag = 0;   //Software flag for the A0 timer
char selected = 0;      //Boolean to represent if a video feed is selected by ground station

char selectedCounter = 0;   //Count for the amount of time a selected feed has been active
char videoFeed = 3;     //Represents the pi that is supplying the video feed. (0 is default - see main loop)

unsigned int heartbeatLoopCount = 0;
unsigned int countToDisableGreenLED = 0;

//Function Prototypes
void SPI (void);
void ISR_FLAG_CHECK (void);
//********************************************************************************************************************
//Main Function
//********************************************************************************************************************
int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;   // Stop Watchdog

// Configure GPIO
    config_HEARTBEAT_LED(); // Configure blue LED (P1.1) as heartbeat
    config_UART_GPIO(0);    // Configure UART1 (P2.0 & P2.1)
    config_UART_GPIO(1);    // Configure UART2 (P2.5 & P2.6)
    config_UART_GPIO(2);    // Configure UART3 (P5.4 & P5.5)
    config_UART_GPIO(3);    // Configure UART4 (P6.0 & P6.1)
    config_SPI_B0_Master_GPIO();  // Configure DACQS SPI: MSP430 (Master) <--> Teensy 3.6 (Slave)
    config_SPI_B1_Slave_GPIO();   // Configure COMMS SPI: MSP430 (Slave) <--> Comms Raspi (Master)
    config_RST_PI_GPIO();      // Configure the (x4) GPIO for Raspi0 Reset (P4.4, P4.5, P4.6, P4.7)
    config_XT1_GPIO();         // Configure GPIO for 32.768 kHz clock (PJ.4 & PJ.5)
    P1DIR |= (BIT0 | BIT2);    // Configure Green and Red LEDs (P1.0 and P1.2 respectively)
    P1OUT &= ~(BIT0 | BIT2);   // Turn Green and Red LEDs off
    P4DIR |= BIT0;             // Configure the cut down GPIO (P4.0)
    P4OUT &= ~BIT0;            // Make sure the cut down GPIO is low/off

// Configure Clock
    config_ACLK_XT1_32KHz_DCO_8MHz_SMCLK_250KHz(); // Configure ACLK, SMCLK, MCLK

// Temp talk to Motor MSP for reaction wheel.
    P8DIR |= (BIT0 | BIT1 | BIT2 | BIT3);    // Configure MSP_GPIO that goes to the Teensy (P8.0, P8.1, P8.2, P8.3)
    P8OUT &= ~(BIT0 | BIT1 | BIT2 | BIT3);   // Set all MSP <--> Teensy 3.6 GPIO to low

// Configure Comms
    config_UART_9600_SMCLK_250KHz(0);   // Configure UART1 at 9600 baud
    config_UART_9600_SMCLK_250KHz(1);   // Configure UART2 at 9600 baud
    config_UART_9600_SMCLK_250KHz(2);   // Configure UART3 at 9600 baud
    config_UART_9600_SMCLK_250KHz(3);   // Configure UART4 at 9600 baud
    config_SPI_B0_Master_SMCLK();       // Configure USCI B0 for Master SPI (link w/ Teensy)
    config_SPI_B1_Slave();              // Configure USCI B1 for Slave SPI (link w/ Comms)

// Timer Configuration
    TA1CCTL0 |= CCIE;                   //Enables Interrupts
    TA1CTL = 0;                         // Clears Timer Configuration
    TA1CTL |= (TACLR + TASSEL_1 + ID_3);// Clears count, ACLK(32.768kHz), ACLK/8 = 4,096 Hz
    TA1CCR0 = 61440;                    // 15 seconds / 4096 oscillations per sec = 61440 clock cycles
    TA1CTL |= MC_1;                     // Starts Timer in Up Mode (counts to TA0CCR0)

    __no_operation();                   // No-Operation. Not sure why this is here. Leaving it for now
    __bis_SR_register(GIE);             // Sets the selected bits of the status register - General Interrupt Enable
//********************************************************************************************************************
// Main Loop
//********************************************************************************************************************
    while(1){
        //Toggle heartbeat only once in a while - otherwise it will look solid
        if(heartbeatLoopCount >= HEARTBEAT_LOOP_PERIOD){
            toggle_heartbeat();     // Toggle the blue LED as a heartbeat
            heartbeatLoopCount = 0; //Reset the heartbeat loop counter
        }

        //Turn off Green LED after a certain amount of time
        if(heartbeatLoopCount == countToDisableGreenLED){
            P1OUT &= ~BIT0; //Turn the Green LED off
        }

        //Check to see if it's been 15 seconds for switching video feeds
        if(timerA0Flag == 1){

            TX_B0(0x    );

            timerA0Flag = 0;    //Reset the software flag

            grab_all_pi_hat(0); //Remove before flight

            //Check to see if a video feed has been selected but not up for at least SELECTED_LIMIT timer intervals
            if(selected == 1 && selectedCounter < SELECTED_LIMIT){
                selectedCounter++;  //Increment the count for how long a selected feed has been active
            }

            //If a video feed is not currently selected by ground station or
            //      if a video feed is selected by ground station and its been 60 seconds,
            //      cycle the feed to the next camera.
            if(selected == 0 || (selected == 1 && selectedCounter == SELECTED_LIMIT)){
                selected = 0;       //Move back to automatic video feed cycling
                selectedCounter = 0;//Reset the selected timer interval count

                if(videoFeed == 3){
                    videoFeed = 0;  //Cycle back to the first feed (0)
                }
                else{
                    videoFeed++; //Increment to the next video feed
                }
                select_pi_video(videoFeed); //Begin the video feed
                P1OUT |= BIT0;              //Turn on the green LED to signal a video feed change

                //Calculation of the next hearbeat loop count to disable the green LED
                if(heartbeatLoopCount > HEARTBEAT_LOOP_PERIOD - GREEN_LED_BLINK_PERIOD){
                    countToDisableGreenLED = GREEN_LED_BLINK_PERIOD - (HEARTBEAT_LOOP_PERIOD - heartbeatLoopCount);
                }
                else{
                    countToDisableGreenLED = heartbeatLoopCount + GREEN_LED_BLINK_PERIOD;
                }
            }
        }

        //Check for any ISR flags that may have occurred since last loop
        ISR_FLAG_CHECK();

        // If an SPI message has been read in and needs to be parsed
        if(spi_slv_fsm_state == PARSING_COMMAND){
            parse_cmd_from_comms(spi_slv_read_message); // Parse the read message
        }

//        UART_parse(0);
//        UART_parse(1);
//        UART_parse(2);
//        UART_parse(3);
        //parse_cmd_from_comms("{01}");
        heartbeatLoopCount++;
    }// End Main Loop
}//End Main Code

void ISR_FLAG_CHECK (void){
    // Check to see if a UART message has been received
    if(uART1_ISR_Flag == 1){
        uART1_ISR_Flag = 0;     //Reset the SW flag
        UART_ISR(0);            //
    }
    if(uART2_ISR_Flag == 1){
        uART2_ISR_Flag = 0;
        UART_ISR(1);
    }
    if(uART3_ISR_Flag == 1){
        uART3_ISR_Flag = 0;
        UART_ISR(2);
    }
    if(uART4_ISR_Flag == 1){
        uART4_ISR_Flag = 0;
        UART_ISR(3);
    }

    //Check to see if an SPI message has been received
    if(b1_ISR_Flag == 1){
        b1_ISR_Flag = 0;    //Reset the ISR software flag
        SPI();              //C
    }

}

void SPI (void){
    switch(spi_slv_fsm_state){
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
}

//********************************************************************************************************************
// Interrupt Service Routines
//********************************************************************************************************************
// Timer A1 interrupt service routine
//*********************************************************************************************************//
#pragma vector = TIMER1_A0_VECTOR
__interrupt void Timer1_A0_ISR(void){
    timerA0Flag = 1;
}
//*********************************************************************************************************//
//Pi Hat UART Receive ISRs
//*********************************************************************************************************//
#pragma vector=EUSCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void){
    switch(__even_in_range(UCA0IV, USCI_UART_UCTXCPTIFG)){
        case USCI_UART_UCRXIFG:
            uart_read_buffer[0][uart_index[0]] = UCA0RXBUF;
            uART1_ISR_Flag = 1;
			break;
        default: break;
    }
}
#pragma vector=EUSCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void){
    switch(__even_in_range(UCA1IV, USCI_UART_UCTXCPTIFG)){
        case USCI_UART_UCRXIFG:
            uart_read_buffer[1][uart_index[1]] = UCA1RXBUF;
            uART2_ISR_Flag = 1;
			break;
        default: break;
    }
}
#pragma vector=EUSCI_A2_VECTOR
__interrupt void USCI_A2_ISR(void){
    switch(__even_in_range(UCA2IV, USCI_UART_UCTXCPTIFG)){
        case USCI_UART_UCRXIFG:
            uart_read_buffer[2][uart_index[2]] = UCA2RXBUF;
            uART3_ISR_Flag = 1;
			break;
        default: break;
    }
}
#pragma vector=EUSCI_A3_VECTOR
__interrupt void USCI_A3_ISR(void){
    switch(__even_in_range(UCA3IV, USCI_UART_UCTXCPTIFG)){
        case USCI_UART_UCRXIFG:
            uart_read_buffer[3][uart_index[3]] = UCA3RXBUF;
            uART4_ISR_Flag = 1;
			break;
        default: break;
    }
}
//*********************************************************************************************************//
//Comms Board SPI Receive ISR
//*********************************************************************************************************//
#pragma vector=EUSCI_B1_VECTOR
__interrupt void USCI_B1_ISR(void){
    // Looks to see if the B1 Interrupt Vector is even up to 4 (inclusive)
    // This means that an interrupt can be generated by Receive buffer full (02h)
    //      or that the transmit buffer is empty (04h)
    switch(__even_in_range(UCB1IV, USCI_SPI_UCTXIFG)){
        case USCI_SPI_UCRXIFG:
            spi_slv_read_buffer[spi_slv_index] = UCB1RXBUF; //Read in the message from the Buffer
            b1_ISR_Flag = 1;    //Toggle the software flag
            break;
        default:break;
    }
}
//*********************************************************************************************************//
//Teensy 3.6 SPI Receive ISR
//*********************************************************************************************************//
#pragma vector=EUSCI_B0_VECTOR
__interrupt void USCI_B0_ISR(void){
    switch(__even_in_range(UCB0IV, USCI_SPI_UCTXIFG)){
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
                    	//spi_mst_fsm_state = spi_mst_fsm_state;
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
                    	//spi_mst_fsm_state = spi_mst_fsm_state;
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
                    	//spi_mst_fsm_state = spi_mst_fsm_state;
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
                    	//spi_mst_fsm_state = spi_mst_fsm_state;
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
