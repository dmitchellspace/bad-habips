/* --COPYRIGHT--,BSD_EX
 * Copyright (c) 2015, Texas Instruments Incorporated
 * All rights reserved.
 * *****************************************************************************
 * Written By: Daniel Mitchell of HABIP P18104 - ddm9599@rit.edu
 * Notes:
 *      -Green LED is when a message is sent out
 *      -Blue LED is when a message is received
 *      -Red LED is when something has gone wrong
 *      -P8IN BITx (MSP_GPIO_x on schematic) is used:
 *          >BIT0 is unused but may not be working w/ the MSP430...
 *          >BIT1 for an indication that backup battery is being used
 *          >BIT2 for cutdown enable
 *          >BIT3 for calibration startup sequence (from Teensy button)
 *
 * Resources:
 *      -MSP430FR599x Datasheet:
 *          http://www.ti.com/lit/ds/symlink/msp430fr5994.pdf
 *
 *      -MSP430FR599x User Guide:
 *          http://www.ti.com/lit/ug/slau367o/slau367o.pdf
 *******************************************************************************/
#include <msp430.h>
#include "driverlib.h"
#include "habip.h"
#include "stdlib.h"
#include <string.h>

#define SELECTED_LIMIT 4 //The number of times a desired video feed stays active before returning to cycle
#define GREEN_LED_BLINK_PERIOD 40000 //The number of main loops that happen for green LED to stay active before turning off
#define BLUE_LED_BLINK_PERIOD 40000 //The number of main loops that happen for blue LED to stay active before turning off
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
char B0_RX_FLAG;
extern long CRC_POLYNOMIAL;
extern char expectedB0ReturnBytes;
extern char tx_message_byte_len;
extern char spi_mst_read_buffer[BUFF_LEN];
extern char spi_mst_send_buffer[BUFF_LEN];
extern volatile unsigned int spi_mst_rx_index;
extern volatile unsigned int spi_mst_write_index;
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
char cutdownEnable = 1; //Enable cutdown if digital input from Teensy is high
char cuttingDown = 0;   //Boolean to represent the state when cutdown is happening
char hasResetPiZeros = 0;   //Boolean to represent if the pi zeros have been reset
char b0_ISR_Flag = 0;   //Software flag for the B0 USCI
char b1_ISR_Flag = 0;   //Software flag for the B1 USCI
char uART1_ISR_Flag = 0; //Software flag for UART1 Receive
char uART2_ISR_Flag = 0; //Software flag for UART2 Receive
char uART3_ISR_Flag = 0; //Software flag for UART3 Receive
char uART4_ISR_Flag = 0; //Software flag for UART4 Receive
char timer1A3Flag = 0;   //Software flag for the timer0 A3 timer
char timer0A5Flag = 0;  //Software flag for the timer1 A5 timer
char selected = 0;      //Boolean to represent if a video feed is selected by ground station

char selectedCounter = 0;   //Count for the amount of time a selected feed has been active
char videoFeed = 3;     //Represents the pi that is supplying the video feed. (0 is default - see main loop)

unsigned int heartbeatLoopCount = 0;
unsigned int fifteenSecondCount = 0;
unsigned int countToDisableGreenLED = 0;
unsigned int countToDisableBlueLED = 40000;

long crcExponent;

//Function Prototypes
typedef struct spiMessage SPIMSG;
void B0_Parse(void);
void B1_Parse(void);
void ISR_FLAG_CHECK(void);
void LED_Management(void);
void CRC_Generate(SPIMSG *spiM);
void Fifteen_Second_Logic(void);
void Video_Feed_Management(void);
void CRC_Exponential(unsigned long val);

//Structure Instantiation for each spi message - See SPI.h
extern struct spiMessage previousCommand;
struct spiMessage all_data = {ALL_DATA_MSG, ALL_DATA_EXP, 0};
struct spiMessage pressure = {PRESSURE_MSG, PRESSURE_EXP, 0};
struct spiMessage temp_board = {TEMP_BOARD_MSG, TEMP_BOARD_EXP, 0};
struct spiMessage imu_x_gyro = {IMU_X_GYRO_MSG, IMU_X_GYRO_EXP, 0};
struct spiMessage imu_y_gyro = {IMU_Y_GYRO_MSG, IMU_Y_GYRO_EXP, 0};
struct spiMessage imu_z_gyro = {IMU_Z_GYRO_MSG, IMU_Z_GYRO_EXP, 0};
struct spiMessage motor_speed = {MOTOR_SPEED_MSG, MOTOR_SPEED_EXP, 0};
struct spiMessage all_imu_data = {ALL_IMU_DATA_MSG, ALL_IMU_DATA_EXP, 0};
struct spiMessage motor_enable = {MOTOR_ENABLE_MSG, MOTOR_ENABLE_EXP, 0};
struct spiMessage all_temp_data = {ALL_TEMP_DATA_MSG, ALL_TEMP_DATA_EXP, 0};
struct spiMessage all_gyro_data = {ALL_GYRO_DATA_MSG, ALL_GYRO_DATA_EXP, 0};
struct spiMessage temp_external = {TEMP_EXTERNAL_MSG, TEMP_EXTERNAL_EXP, 0};
struct spiMessage all_motor_data = {ALL_MOTOR_DATA_MSG, ALL_MOTOR_DATA_EXP, 0};
struct spiMessage supply_voltage = {SUPPLY_VOLTAGE_MSG, SUPPLY_VOLTAGE_EXP, 0};
struct spiMessage supply_current = {SUPPLY_CURRENT_MSG, SUPPLY_CURRENT_EXP, 0};
struct spiMessage motor_direction = {MOTOR_DIRECTION_MSG, MOTOR_DIRECTION_EXP, 0};
struct spiMessage all_supply_data = {ALL_SUPPLY_DATA_MSG, ALL_SUPPLY_DATA_EXP, 0};
struct spiMessage teensy_response = {TEENSY_RESPONSE_MSG, TEENSY_RESPONSE_EXP, 0};
struct spiMessage imu_x_acceleration = {IMU_X_ACCELERATION_MSG, IMU_X_ACCELERATION_EXP, 0};
struct spiMessage imu_y_acceleration = {IMU_Y_ACCELERATION_MSG, IMU_Y_ACCELERATION_EXP, 0};
struct spiMessage imu_z_acceleration = {IMU_Z_ACCELERATION_MSG, IMU_Z_ACCELERATION_EXP, 0};
struct spiMessage motor_battery_voltage = {MOTOR_BATTERY_VOLTAGE_MSG, MOTOR_BATTERY_VOLTAGE_EXP, 0};
struct spiMessage motor_battery_current = {MOTOR_BATTERY_CURRENT_MSG, MOTOR_BATTERY_CURRENT_EXP, 0};
struct spiMessage all_acceleration_data = {ALL_ACCELERATION_DATA_MSG, ALL_ACCELERATION_DATA_EXP, 0};
//*********************************************************************************************************//
//Main Function
//*********************************************************************************************************//
int main(void)
{
      WDTCTL = WDTPW | WDTHOLD;   // Stop Watchdog

    //Generate CRCs for each SPI Message
    CRC_Generate(&all_data);
    CRC_Generate(&pressure);
    CRC_Generate(&temp_board);
    CRC_Generate(&imu_x_gyro);
    CRC_Generate(&imu_y_gyro);
    CRC_Generate(&imu_z_gyro);
    CRC_Generate(&motor_speed);
    CRC_Generate(&all_imu_data);
    CRC_Generate(&motor_enable);
    CRC_Generate(&all_temp_data);
    CRC_Generate(&all_gyro_data);
    CRC_Generate(&temp_external);
    CRC_Generate(&all_motor_data);
    CRC_Generate(&supply_voltage);
    CRC_Generate(&supply_current);
    CRC_Generate(&motor_direction);
    CRC_Generate(&all_supply_data);
    CRC_Generate(&teensy_response);
    CRC_Generate(&imu_x_acceleration);
    CRC_Generate(&imu_y_acceleration);
    CRC_Generate(&imu_z_acceleration);
    CRC_Generate(&motor_battery_voltage);
    CRC_Generate(&motor_battery_current);
    CRC_Generate(&all_acceleration_data);

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
    P8SEL0 &= ~(BIT0 | BIT1 | BIT2 | BIT3);
    P8SEL1 &= ~(BIT0 | BIT1 | BIT2 | BIT3);
    P8OUT &= ~(BIT0 | BIT1 | BIT2 | BIT3);   // Set all MSP <--> Teensy 3.6 GPIO to low

// Configure Comms
    config_UART_9600_SMCLK_250KHz(0);   // Configure UART1 at 9600 baud
    config_UART_9600_SMCLK_250KHz(1);   // Configure UART2 at 9600 baud
    config_UART_9600_SMCLK_250KHz(2);   // Configure UART3 at 9600 baud
    config_UART_9600_SMCLK_250KHz(3);   // Configure UART4 at 9600 baud
    config_SPI_B0_Master_SMCLK();       // Configure USCI B0 for Master SPI (link w/ Teensy)
    config_SPI_B1_Slave();              // Configure USCI B1 for Slave SPI (link w/ Comms)

// Timer1 A3 Configuration
    TA1CCTL0 |= CCIE;                   //Enables Interrupts
    TA1CTL = 0;                         // Clears Timer Configuration
    TA1CTL |= (TACLR + TASSEL_1 + ID_3);// Clears count, ACLK(32.768kHz), ACLK/8 = 4,096 Hz
    TA1CCR0 = 61440;                    // 15 seconds @ 4096 oscillations per sec = 61440 clock cycles
    TA1CTL |= MC_1;                     // Starts Timer in Up Mode (counts to TA1CCR0)

// Timer0 A5 Configuration
    TA0CCTL0 |= CCIE;                   //Enables Interrupts
    TA0CTL = 0;                         // Clears Timer Configuration
    TA0CTL |= (TACLR + TASSEL_1 + ID_3);// Clears count, ACLK(32.768kHz), ACLK/8 = 4,096 Hz
    TA0CCR0 = 40960;                     // 2 seconds @ 4096 oscillations per sec = 8192 clock cycles

    __no_operation();                   // No-Operation. Not sure why this is here. Leaving it for now
    __bis_SR_register(GIE);             // Sets the selected bits of the status register - General Interrupt Enable
//********************************************************************************************************************
// Main Loop
//********************************************************************************************************************
    while(1){
        LED_Management();
        ISR_FLAG_CHECK();

        //Check for a notification from Teensy that startup calibration has been selected (Teensy button press on startup)
        if(((P8IN & BIT3) >> 3) == 1 && hasResetPiZeros != 1){
            hasResetPiZeros = 1;    //Change the value of the boolean variable to reflect that the pi zeros have been reset

            // Reset all Pi 0's
            reset_pi(0);
            reset_pi(1);
            reset_pi(2);
            reset_pi(3);

            P1OUT |= BIT0;  //Turn on the green LED to signal that the pi zero's have been reset

            //Calculation of the next hearbeat loop count to disable the green LED
            if(heartbeatLoopCount > HEARTBEAT_LOOP_PERIOD - GREEN_LED_BLINK_PERIOD){
                countToDisableGreenLED = GREEN_LED_BLINK_PERIOD - (HEARTBEAT_LOOP_PERIOD - heartbeatLoopCount);
            }
            else{
                countToDisableGreenLED = heartbeatLoopCount + GREEN_LED_BLINK_PERIOD;
            }
        }
    }
}

void LED_Management(void){
    //Manage the heartbeat
    if(heartbeatLoopCount == 65000){
        P1OUT ^= BIT1;  //Turn on the Blue LED
        heartbeatLoopCount = 0; //Reset the heartbeatLoopCount

        //Only toggle the red LED for bad battery if we are currently NOT cutting down and the GPIO from the Teensy is high
        //We don't want to toggle the LED during cutdown because I have it held high for the duration of that action
        if(((P8IN & BIT1)>>1) == 1 && cuttingDown != 1){
            P1OUT ^= BIT2; //Toggle Red LED
        }
        if(((P8IN & BIT1)>>1) == 0 && cuttingDown != 1 && ((P1OUT & BIT2) >> 2) == 1){
            P1OUT &= ~ BIT2;
        }
    }

    //Turn off Green LED after a certain amount of time
    if(heartbeatLoopCount == countToDisableGreenLED){
        P1OUT &= ~BIT0; //Turn the Green LED off
    }

    heartbeatLoopCount++;   //Increment the loop counter
}

void Fifteen_Second_Logic(void){
    Video_Feed_Management();    //Manage the video feeds (through UART)

    //Check for cutdown enable input from Teensy
    if((((P8IN & BIT2) >> 2) == 1) && cutdownEnable == 1){
        TA0CTL |= MC_1; // Starts Timer in Up Mode (counts to TA0CCR0 [10 seconds])
        cutdownEnable = 0;    // Set cutdown to 0 that we don't cutdown again
        cuttingDown = 1;    // Boolean for showing that we are in a state of cutting down
        P4OUT |= BIT0;  // Toggle Cutdown GPIO
        P1OUT |= BIT2;  // Turn on the Red LED
    }

    if(fifteenSecondCount == 3 && hasResetPiZeros == 1){
        //Sync the relative time with all the pi 0's 15 seconds after startup
        UART_write_msg(0, "{06:1524283200}");
        UART_write_msg(1, "{06:1524283200}");
        UART_write_msg(2, "{06:1524283200}");
        UART_write_msg(3, "{06:1524283200}");
    }

    P1OUT |= BIT0;  //Turn on the green LED to signal a 15-second loop
    //Calculation of the next hearbeat loop count to disable the green LED
    if(heartbeatLoopCount > HEARTBEAT_LOOP_PERIOD - GREEN_LED_BLINK_PERIOD){
        countToDisableGreenLED = GREEN_LED_BLINK_PERIOD - (HEARTBEAT_LOOP_PERIOD - heartbeatLoopCount);
    }
    else{
        countToDisableGreenLED = heartbeatLoopCount + GREEN_LED_BLINK_PERIOD;
    }

    fifteenSecondCount++;   //Increment the counter
}

void Video_Feed_Management(void){
//    //Check to see if a video feed has been selected but not up for at least SELECTED_LIMIT timer intervals
//    if(selected == 1 && selectedCounter < SELECTED_LIMIT){
//        selectedCounter++;  //Increment the count for how long a selected feed has been active
//    }
//
//    //If a video feed is not currently selected by ground station or
//    //      if a video feed is selected by ground station and its been 60 seconds,
//    //      cycle the feed to the next camera.
//    if(selected == 0 || (selected == 1 && selectedCounter == SELECTED_LIMIT)){
//        selected = 0;       //Move back to automatic video feed cycling
//        selectedCounter = 0;//Reset the selected timer interval count
//
//        if(videoFeed == 3){
//            videoFeed = 0;  //Cycle back to the first feed (0)
//        }
//        else{
//            videoFeed++; //Increment to the next video feed
//        }
//        select_pi_video(videoFeed); //Begin the video feed
//        P1OUT |= BIT0;              //Turn on the green LED to signal a video feed change
//
//        //Calculation of the next hearbeat loop count to disable the green LED
//        if(heartbeatLoopCount > HEARTBEAT_LOOP_PERIOD - GREEN_LED_BLINK_PERIOD){
//            countToDisableGreenLED = GREEN_LED_BLINK_PERIOD - (HEARTBEAT_LOOP_PERIOD - heartbeatLoopCount);
//        }
//        else{
//            countToDisableGreenLED = heartbeatLoopCount + GREEN_LED_BLINK_PERIOD;
//        }
//    }
}

void CRC_Generate (SPIMSG *spiM){
    //Generates the CRC value for each transmit (request) message since they are always going to be the same
    int i;
    unsigned long tempData = 0;
    unsigned long tempCRC = 0;

    tempData = spiM->msgCode;
    tempData <<= 8;
    tempCRC = CRC_POLYNOMIAL << 16;

    for(i=23; i>=7; i--){
        CRC_Exponential(i);

        if((tempData & crcExponent) != 0){
            tempData ^= tempCRC;
        }
        tempCRC >>= 1;
    }

    spiM->crc = tempData;
}

void CRC_Exponential(unsigned long val){
    //I had to write my own exponential function because the built-in ones did not work with the variable types I need to use...
    signed char i;
    unsigned long temp = 2;

    for(i=1; i<val; i++){
        temp = temp * 2;
    }
    crcExponent = temp;
}

void ISR_FLAG_CHECK (void){

    //Check for the 15 second timer flag
    if(timer1A3Flag == 1){
        timer1A3Flag = 0;    //Reset the ISR software flag
        Fifteen_Second_Logic();
    }

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

    //Check to see if a Comms SPI message has been received
    if(b1_ISR_Flag == 1){
        b1_ISR_Flag = 0;    //Reset the ISR software flag
        B1_Parse();         //Call the function that contains logic which was originally in the ISR... - old code probably not working
    }

    //Check to see if a Teensy SPI message has been received
    if(B0_RX_FLAG == 1){
        B0_RX_FLAG = 0; //Reset the Rx Software Flag
        P1OUT |= BIT1; //Light the blue LED

        //Calculation of the next hearbeat loop count to disable the green LED
        if(heartbeatLoopCount > HEARTBEAT_LOOP_PERIOD - BLUE_LED_BLINK_PERIOD){
            countToDisableBlueLED = BLUE_LED_BLINK_PERIOD - (HEARTBEAT_LOOP_PERIOD - heartbeatLoopCount);
        }
        else{
            countToDisableBlueLED = heartbeatLoopCount + BLUE_LED_BLINK_PERIOD;
        }

        B0_Parse();
    }
}

void B0_Parse(){
    //Parses data from the SPI bus....sadly never got to be implemented :'(
    //Note: This parsing is barely touched. You may just wanna start from scratch with this function
//    char msgHeader = (spi_mst_read_buffer[0] & 0xF0) >> 4;
//
//    //Request for the MSP430 to resend the data or reset the bus
//    if(msgHeader == 1){
//        if(spi_mst_read_buffer[1] & 0x20) == 0x20){
//            config_SPI_B0_Master_SMCLK();       // Configure USCI B0 for Master SPI (link w/ Teensy)
//            TODO: Reset the Bus here (maybe call a function that does it?)
//        }
//        else{
//            if(previousCommand.msgCode == (spi_mst_read_buffer[])){
//
//                //TODO: Check to see if this is the second failure in a row. If so, reset both buses.
//            }
//            else{
//                Request_Teensy(previousCommand);
//            }
//        }
//    }
}

void B1_Parse (void){
    //Parses data from the UART bus
    //Note: I (Dan) did not write any of this UART code. Our launch did not utilize receiving data from the pi 0's because we didn't have
    //      a way to communicate it to the ground station so there was no point.
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

//*********************************************************************************************************//
// Interrupt Service Routines
//*********************************************************************************************************//
// Timer1 A3 interrupt service routine
//*********************************************************************************************************//
#pragma vector = TIMER1_A0_VECTOR
__interrupt void Timer1_A0_ISR(void){
    timer1A3Flag = 1;
}
//*********************************************************************************************************//
// Timer0 A5 interrupt service routine
//*********************************************************************************************************//
#pragma vector = TIMER0_A0_VECTOR
__interrupt void Timer0_A1_ISR(void){
    P4OUT &= ~BIT0;     //Turns off the Red LED (Placeholder for cutdown test) - Remove before flight
    cuttingDown = 0;    //Boolean to show that cutdown is no longer occurring
    P1OUT &= ~BIT2;     //Disable the current through the nichrome wire
    TA0CCTL0 &= ~CCIE;  //Disables interrupts for this vector
    timer0A5Flag = 1;   //Enable the flag
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
//Teensy 3.6 SPI Transmit and Receive ISR
//*********************************************************************************************************//
#pragma vector=EUSCI_B0_VECTOR
__interrupt void USCI_B0_ISR(void){
    switch(__even_in_range(UCB0IV, USCI_SPI_UCTXIFG)){
        case USCI_SPI_UCRXIFG:
			spi_mst_read_buffer[spi_mst_rx_index] = UCB0RXBUF;  //Read the byte in from the buffer

			//TODO: Add in error logic. What happens if more bytes are received if you arent expecting them? etc.
        	if(spi_mst_rx_index == (expectedB0ReturnBytes-1)){
        	    //Receive is finished - Reset the index and toggle the SW flag
        		spi_mst_rx_index = 0;
        		B0_RX_FLAG = 1;
        	}
             else {
                   //Increment the receive buffer index for the next incoming byte
                   spi_mst_rx_index++;
             }
            break;
        case USCI_SPI_UCTXIFG:
            if(spi_mst_write_index == tx_message_byte_len){
                //Transmission is complete - no need to load anything else
                break;
            }
            else{
                //Load the next byte into the HW Tx buffer
                UCB0TXBUF = spi_mst_send_buffer[spi_mst_write_index];   //Load the next byte into the tx buffer
                spi_mst_send_buffer[spi_mst_write_index] = 0;   //Clear the data after sending it
                spi_mst_write_index++;  //Increment to the next index in the tx buffer
                break;
            }
    }
}
//*********************************************************************************************************//
