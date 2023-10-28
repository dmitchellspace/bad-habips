/*
 * SPI.h
 *
 *  Created on: Apr 11, 2017
 *      Author: Lincster
 */

#ifndef SPI_H_
#define SPI_H_

// SPI Message Structure
struct spiMessage {
  int msgCode;            // Message Code - See constants below
  char expectedReturn;    // Expected # Return Messages - See constants below
  unsigned char crc;      // CRC - Generated on startup in case of any future changes in message codes
};

// 16-bit Message Codes to Teensy (spiMessage msgCode)
#define PRESSURE_MSG                0x2000
#define IMU_X_GYRO_MSG              0x2100
#define IMU_Y_GYRO_MSG              0x2102
#define IMU_Z_GYRO_MSG              0x2104
#define IMU_X_ACCELERATION_MSG      0x2101
#define IMU_Y_ACCELERATION_MSG      0x2103
#define IMU_Z_ACCELERATION_MSG      0x2105
#define TEMP_BOARD_MSG              0x2200
#define TEMP_EXTERNAL_MSG           0x2208
#define SUPPLY_VOLTAGE_MSG          0x2300
#define SUPPLY_CURRENT_MSG          0x2310
#define MOTOR_ENABLE_MSG            0x2400
#define MOTOR_DIRECTION_MSG         0x2410
#define MOTOR_BATTERY_VOLTAGE_MSG   0x2420
#define MOTOR_BATTERY_CURRENT_MSG   0x2430
#define MOTOR_SPEED_MSG             0x2440
#define ALL_GYRO_DATA_MSG           0x2500
#define ALL_ACCELERATION_DATA_MSG   0x2600
#define ALL_IMU_DATA_MSG            0x2700
#define ALL_TEMP_DATA_MSG           0x2800
#define ALL_SUPPLY_DATA_MSG         0x2900
#define ALL_MOTOR_DATA_MSG          0x2A00
#define ALL_DATA_MSG                0x2B00
#define TEENSY_RESPONSE_MSG         0x5000

// Expected # of return messages from Teensy for a given request (spiMessage expectedReturn)
#define PRESSURE_EXP                2
#define IMU_X_GYRO_EXP              2
#define IMU_Y_GYRO_EXP              2
#define IMU_Z_GYRO_EXP              2
#define IMU_X_ACCELERATION_EXP      2
#define IMU_Y_ACCELERATION_EXP      2
#define IMU_Z_ACCELERATION_EXP      2
#define TEMP_BOARD_EXP              2
#define TEMP_EXTERNAL_EXP           2
#define SUPPLY_VOLTAGE_EXP          2
#define SUPPLY_CURRENT_EXP          2
#define MOTOR_ENABLE_EXP            1
#define MOTOR_DIRECTION_EXP         1
#define MOTOR_BATTERY_VOLTAGE_EXP   2
#define MOTOR_BATTERY_CURRENT_EXP   2
#define MOTOR_SPEED_EXP             2
#define ALL_GYRO_DATA_EXP           6
#define ALL_ACCELERATION_DATA_EXP   6
#define ALL_IMU_DATA_EXP            12
#define ALL_TEMP_DATA_EXP           4
#define ALL_SUPPLY_DATA_EXP         4
#define ALL_MOTOR_DATA_EXP          8
#define ALL_DATA_EXP                30
#define TEENSY_RESPONSE_EXP         0

// SPI Master FSM States
#define MST_IDLE 0x00
#define CHECKING_IF_SLAVE_READY 0x01
#define SENDING_COMMAND 0x02
#define SPI_LISTENING_FOR_RESPONSE 0x03
#define SPI_CAPTURING_RESPONSE 0x04
#define SPI_MST_FSM_STATE_CNT 5

// SPI Slave FSM States
#define LISTENING_FOR_COMMAND 0x00
#define CAPTURING_COMMAND 0x01
#define PARSING_COMMAND 0x02
#define OBTAINING_DATA 0x03
#define RESPONDING_WITH_DATA 0x04
#define RESPONDING_WITH_ALL_DATA 0x05
#define SPI_SLV_FSM_STATE_CNT 6

#define TX_B0(val) ({\
            while(!(UCB0IFG & UCTXIFG)) ;\
            UCB0TXBUF = val;\
            spi_mst_send_buffer[spi_mst_rx_index] = val;\
            spi_mst_rx_index++;\
           })
#define TX_A0_SPI(val) ({\
            while(!(UCA0IFG&UCTXIFG)) ;\
            UCA0TXBUF = val;\
            spi_slv_send_buffer[spi_slv_index] = val;\
           })

#define TX_B1_SPI(val) ({\
            while(!(UCB1IFG&UCTXIFG)) ;\
            UCB1TXBUF = val;\
            spi_slv_send_buffer[spi_slv_index] = val;\
           })

#define spi_slave_parse() ({if(spi_slv_fsm_state == PARSING_COMMAND){parse_cmd_from_host(spi_slv_read_message);}})

void config_SPI_B0_Master_GPIO(void);
void config_SPI_B0_Master_ACLK(void);
void config_SPI_B0_Master_SMCLK(void);
void config_SPI_B1_Slave_GPIO(void);
void config_SPI_B1_Slave(void);
void config_SPI_A0_Slave_GPIO(void);
void config_SPI_A0_Slave(void);
void SPI_command_host_to_slave(char* message,volatile int* read_done);
void SPI_command_host_to_slave_no_response(char* message,volatile int* send_done);

void Request_Teensy(struct spiMessage spiM);

#endif /* SPI_H_ */
