/*
 * command_interface.h
 *
 *  Created on: Apr 15, 2017
 *      Author: Lincster
 */

#ifndef HABIP_COMMAND_INTERFACE_H_
#define HABIP_COMMAND_INTERFACE_H_

// Pi Hat Board Sensor Info Indicies
#define PI_HAT_SENSOR_CNT 10
#define PI_TD0 0
#define PI_TB0 1
#define PI_TB1 2
#define PI_TE0 3
#define PI_TE1 4
#define PI_P0 5
#define PI_P1 6
#define PI_H 7
#define PI_V 8
#define PI_C 9

// DAQCS Board Sensor Info Indicies
#define DAQCS_SENSOR_CNT 16
#define DQ_TB0 0
#define DQ_P0 1
#define DQ_PB 2
#define DQ_V 3
#define DQ_C 4
#define DQ_XGY 5
#define DQ_XAC 6
#define DQ_YGY 7
#define DQ_YAC 8
#define DQ_ZGY 9
#define DQ_ZAC 10
#define DQ_MS 11
#define DQ_MC 12
#define DQ_MV 13
#define DQ_MD 14
#define DQ_ME 15

//Quotes
#define Q_0 "0"
#define Q_1 "1"
#define Q_2 "2"
#define Q_3 "3"
#define Q_TD0 "TD0"
#define Q_TB0 "TB0"
#define Q_TB1 "TB1"
#define Q_TE0 "TE0"
#define Q_TE1 "TE1"
#define Q_P0 "P0"
#define Q_P1 "P1"
#define Q_PB "PB"
#define Q_H "H"
#define Q_V "V"
#define Q_C "C"
#define Q_XGY "XGY"
#define Q_XAC "XAC"
#define Q_YGY "YGY"
#define Q_YAC "YAC"
#define Q_ZGY "ZGY"
#define Q_ZAC "ZAC"
#define Q_MS "MS"
#define Q_MC "MC"
#define Q_MV "MV"
#define Q_MD "MD"
#define Q_ME "ME"

// response_buffer data status
#define OLD 0x00
#define NEW 0x01
#define ERROR 0xEE

#define GRAB_PI_HAT(brd_num,sns) {\
    strcat(respond_all_data_msg,";B"Q_##brd_num":"Q_##sns":");\
    strcat(respond_all_data_msg,response_buffer[brd_num][PI_##sns]);\
    response_status[brd_num][PI_##sns] = OLD;\
  }
#define GRAB_DAQCS(sns) {\
    strcat(respond_all_data_msg,";B4:"Q_##sns":");\
    strcat(respond_all_data_msg,response_buffer_b4[DQ_##sns]);\
    response_status_b4[DQ_##sns] = OLD;\
  }
#define grab_all_pi_hat(brd_num) UART_write_msg(brd_num,"{01}");

#define grab_all_daqcs(sns) SPI_command_host_to_slave("{00:B4:"Q_##sns"}",&spi_mst_readDoneFG);\
  parse_response(spi_mst_read_message);\

#define toggle_heartbeat() GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN1);

int get_colon_count(const char* s);
void select_pi_video(int brd_num);
void rmv_start_end_chars(char* s);
void store_response_val(int brd_num, char* sns, char* val);
void read_response_val(int brd_num, char* sns, char** val);
void read_response_val_b4(char* sns, char* val);
void parse_cmd_from_comms(char* msg);
void parse_cmd_from_host(char* msg);
void create_comms_response(char* brd, char* sns, char* val);
void create_host_response(char* sns, char* val);
void parse_response_pi_hat(int brd_num, char* msg);
void parse_response(char* msg);
void create_respond_all_data_msg(void);
void one_colon_extract(char* msg, char** first, char** second);
void two_colon_extract(char* msg, char** first, char** second, char** third);
void config_RST_PI_GPIO(void);
void reset_pi(int brd_num);
void config_CUTDOWN_GPIO(void);
void cutdown(void);
void grab_all_motor_msp(void);
#endif /* HABIP_COMMAND_INTERFACE_H_ */
