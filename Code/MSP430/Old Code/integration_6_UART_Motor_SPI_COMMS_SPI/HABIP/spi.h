/*
 * SPI.h
 *
 *  Created on: Apr 11, 2017
 *      Author: Lincster
 */

#ifndef SPI_H_
#define SPI_H_

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
            while(!(UCB0IFG&UCTXIFG)) ;\
            UCB0TXBUF = val;\
            spi_mst_send_buffer[spi_mst_index] = val;\
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

#endif /* SPI_H_ */
