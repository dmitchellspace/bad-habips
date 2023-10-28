/*
 * command_interface.c
 *
 *  Created on: Apr 15, 2017
 *      Author: Lincster
 */
#include "command_interface.h"
#include "msp430.h"
#include <string.h>
#include "common.h"
#include "uart.h"
#include "spi.h"

char response_buffer[4][PI_HAT_SENSOR_CNT][MSG_LEN]={};
char response_buffer_b4[DAQCS_SENSOR_CNT][MSG_LEN]={};

char response_status[4][PI_HAT_SENSOR_CNT] = {{{OLD}}};
char response_status_b4[DAQCS_SENSOR_CNT] = {{OLD}};

char respond_all_data_msg[1024] = {};

char val_b4[10];

extern volatile int spi_slv_fsm_state;
extern volatile int spi_mst_readDoneFG;
extern volatile int spi_mst_sendDoneFG;
extern char spi_slv_send_message[MSG_LEN];
extern char spi_mst_read_message[MSG_LEN];

void create_respond_all_data_msg(void){
	respond_all_data_msg[0] = '\0';
    strcat(respond_all_data_msg,"{B0:TD0:");
    strcat(respond_all_data_msg,response_buffer[0][PI_TD0]);
    response_status[0][PI_TD0] = OLD;
	GRAB_PI_HAT(0,TB0);
	GRAB_PI_HAT(0,TB1);
	GRAB_PI_HAT(0,TE0);
	GRAB_PI_HAT(0,TE1);
	GRAB_PI_HAT(0,P0);
	GRAB_PI_HAT(0,P1);
	GRAB_PI_HAT(0,H);
	GRAB_PI_HAT(0,V);
	GRAB_PI_HAT(0,C);
	GRAB_PI_HAT(1,TD0);
	GRAB_PI_HAT(1,TB0);
	GRAB_PI_HAT(1,TB1);
	GRAB_PI_HAT(1,TE0);
	GRAB_PI_HAT(1,TE1);
	GRAB_PI_HAT(1,P0);
	GRAB_PI_HAT(1,P1);
	GRAB_PI_HAT(1,H);
	GRAB_PI_HAT(1,V);
	GRAB_PI_HAT(1,C);
	GRAB_PI_HAT(2,TD0);
	GRAB_PI_HAT(2,TB0);
	GRAB_PI_HAT(2,TB1);
	GRAB_PI_HAT(2,TE0);
	GRAB_PI_HAT(2,TE1);
	GRAB_PI_HAT(2,P0);
	GRAB_PI_HAT(2,P1);
	GRAB_PI_HAT(2,H);
	GRAB_PI_HAT(2,V);
	GRAB_PI_HAT(2,C);
	GRAB_PI_HAT(3,TD0);
	GRAB_PI_HAT(3,TB0);
	GRAB_PI_HAT(3,TB1);
	GRAB_PI_HAT(3,TE0);
	GRAB_PI_HAT(3,TE1);
	GRAB_PI_HAT(3,P0);
	GRAB_PI_HAT(3,P1);
	GRAB_PI_HAT(3,H);
	GRAB_PI_HAT(3,V);
	GRAB_PI_HAT(3,C);
	GRAB_DAQCS(TB0);
	GRAB_DAQCS(P0);
	GRAB_DAQCS(PB);
	GRAB_DAQCS(V);
	GRAB_DAQCS(C);
	GRAB_DAQCS(XGY);
	GRAB_DAQCS(XAC);
	GRAB_DAQCS(YGY);
	GRAB_DAQCS(YAC);
	GRAB_DAQCS(ZGY);
	GRAB_DAQCS(ZAC);
	GRAB_DAQCS(MS);
	GRAB_DAQCS(MC);
	GRAB_DAQCS(MV);
	GRAB_DAQCS(MD);
	GRAB_DAQCS(ME);
	strcat(respond_all_data_msg,"}$");
}

void rmv_start_end_chars(char* s){
	if(strstr(s,"{")!=NULL){
		strncpy(s,s+1,strlen(s)-1);
		s[strlen(s)-1]='\0';
	}
	if(strstr(s,"}")!=NULL){
		s[strlen(s)-1]='\0';
	}
}

int get_colon_count(const char* s){
	// Note: if passed a string with 3 or more colons, will return 2.
	char* pcolon = strstr(s,":");
	if(pcolon == NULL){
		return 0;
	}
	else{
		char* pcolon2 = strstr(pcolon+1,":");
		if(pcolon2 == NULL){
			return 1;
		}
		else {
			return 2;
		}
	}
}
void read_response_val(int brd_num, char* sns, char** val){
	if(!((brd_num>=0)&&(brd_num<=4))){
		// error msg?
	}
	if((brd_num >= 0) && (brd_num <= 3)){
		if(strcmp(sns,"TD0")==0){
			response_status[brd_num][PI_TD0] = OLD;
			strcpy(*val,response_buffer[brd_num][PI_TD0]);
		}
		else if(strcmp(sns,"TB0")==0){
			response_status[brd_num][PI_TB0] = OLD;
			strcpy(*val,response_buffer[brd_num][PI_TB0]);
		}
		else if(strcmp(sns,"TB1")==0){
			response_status[brd_num][PI_TB1] = OLD;
			strcpy(*val,response_buffer[brd_num][PI_TB1]);
		}
		else if(strcmp(sns,"TE0")==0){
			response_status[brd_num][PI_TE0] = OLD;
			strcpy(*val,response_buffer[brd_num][PI_TE0]);
		}
		else if(strcmp(sns,"TE1")==0){
			response_status[brd_num][PI_TE1] = OLD;
			strcpy(*val,response_buffer[brd_num][PI_TE1]);
		}
		else if(strcmp(sns,"P0")==0){
			response_status[brd_num][PI_P0] = OLD;
			strcpy(*val,response_buffer[brd_num][PI_P0]);
		}
		else if(strcmp(sns,"P1")==0){
			response_status[brd_num][PI_P1] = OLD;
			strcpy(*val,response_buffer[brd_num][PI_P1]);
		}
		else if(strcmp(sns,"H")==0){
			response_status[brd_num][PI_H] = OLD;
			strcpy(*val,response_buffer[brd_num][PI_H]);
		}
		else if(strcmp(sns,"V")==0){
			response_status[brd_num][PI_V] = OLD;
			strcpy(*val,response_buffer[brd_num][PI_V]);
		}
		else if(strcmp(sns,"C")==0){
			response_status[brd_num][PI_C] = OLD;
			strcpy(*val,response_buffer[brd_num][PI_C]);
		}
		else {
			// error msg?
		}
	}
	else if(brd_num == 4){
		if(strcmp(sns,"TB0")==0){
			response_status_b4[DQ_TB0] = OLD;
			strcpy(*val,response_buffer_b4[DQ_TB0]);
		}
		else if(strcmp(sns,"P0")==0){
			response_status_b4[DQ_P0] = OLD;
			strcpy(*val,response_buffer_b4[DQ_P0]);
		}
		else if(strcmp(sns,"PB")==0){
			response_status_b4[DQ_PB] = OLD;
			strcpy(*val,response_buffer_b4[DQ_PB]);
		}
		else if(strcmp(sns,"V")==0){
			response_status_b4[DQ_V] = OLD;
			strcpy(*val,response_buffer_b4[DQ_V]);
		}
		else if(strcmp(sns,"C")==0){
			response_status_b4[DQ_C] = OLD;
			strcpy(*val,response_buffer_b4[DQ_C]);
		}
		else if(strcmp(sns,"XGY")==0){
			response_status_b4[DQ_XGY] = OLD;
			strcpy(*val,response_buffer_b4[DQ_XGY]);
		}
		else if(strcmp(sns,"XAC")==0){
			response_status_b4[DQ_XAC] = OLD;
			strcpy(*val,response_buffer_b4[DQ_XAC]);
		}
		else if(strcmp(sns,"YGY")==0){
			response_status_b4[DQ_YGY] = OLD;
			strcpy(*val,response_buffer_b4[DQ_YGY]);
		}
		else if(strcmp(sns,"YAC")==0){
			response_status_b4[DQ_YAC] = OLD;
			strcpy(*val,response_buffer_b4[DQ_YAC]);
		}
		else if(strcmp(sns,"ZGY")==0){
			response_status_b4[DQ_ZGY] = OLD;
			strcpy(*val,response_buffer_b4[DQ_ZGY]);
		}
		else if(strcmp(sns,"ZAC")==0){
			response_status_b4[DQ_ZAC] = OLD;
			strcpy(*val,response_buffer_b4[DQ_ZAC]);
		}
		else if(strcmp(sns,"MS")==0){
			response_status_b4[DQ_MS] = OLD;
			strcpy(*val,response_buffer_b4[DQ_MS]);
		}
		else if(strcmp(sns,"MC")==0){
			response_status_b4[DQ_MC] = OLD;
			strcpy(*val,response_buffer_b4[DQ_MC]);
		}
		else if(strcmp(sns,"MV")==0){
			response_status_b4[DQ_MV] = OLD;
			strcpy(*val,response_buffer_b4[DQ_MV]);
		}
		else if(strcmp(sns,"MD")==0){
			response_status_b4[DQ_MD] = OLD;
			strcpy(*val,response_buffer_b4[DQ_MD]);
		}
		else if(strcmp(sns,"ME")==0){
			response_status_b4[DQ_ME] = OLD;
			strcpy(*val,response_buffer_b4[DQ_ME]);
		}
		else {
			// error msg?
		}
	}
	else {
		// error msg?
	}
}
void read_response_val_b4(char* sns, char* val){
	if(strcmp(sns,"TB0")==0){
		response_status_b4[DQ_TB0] = OLD;
		strcpy(val,response_buffer_b4[DQ_TB0]);
	}
	else if(strcmp(sns,"P0")==0){
		response_status_b4[DQ_P0] = OLD;
		strcpy(val,response_buffer_b4[DQ_P0]);
	}
	else if(strcmp(sns,"PB")==0){
		response_status_b4[DQ_PB] = OLD;
		strcpy(val,response_buffer_b4[DQ_PB]);
	}
	else if(strcmp(sns,"V")==0){
		response_status_b4[DQ_V] = OLD;
		strcpy(val,response_buffer_b4[DQ_V]);
	}
	else if(strcmp(sns,"C")==0){
		response_status_b4[DQ_C] = OLD;
		strcpy(val,response_buffer_b4[DQ_C]);
	}
	else if(strcmp(sns,"XGY")==0){
		response_status_b4[DQ_XGY] = OLD;
		strcpy(val,response_buffer_b4[DQ_XGY]);
	}
	else if(strcmp(sns,"XAC")==0){
		response_status_b4[DQ_XAC] = OLD;
		strcpy(val,response_buffer_b4[DQ_XAC]);
	}
	else if(strcmp(sns,"YGY")==0){
		response_status_b4[DQ_YGY] = OLD;
		strcpy(val,response_buffer_b4[DQ_YGY]);
	}
	else if(strcmp(sns,"YAC")==0){
		response_status_b4[DQ_YAC] = OLD;
		strcpy(val,response_buffer_b4[DQ_YAC]);
	}
	else if(strcmp(sns,"ZGY")==0){
		response_status_b4[DQ_ZGY] = OLD;
		strcpy(val,response_buffer_b4[DQ_ZGY]);
	}
	else if(strcmp(sns,"ZAC")==0){
		response_status_b4[DQ_ZAC] = OLD;
		strcpy(val,response_buffer_b4[DQ_ZAC]);
	}
	else if(strcmp(sns,"MS")==0){
		response_status_b4[DQ_MS] = OLD;
		strcpy(val,response_buffer_b4[DQ_MS]);
	}
	else if(strcmp(sns,"MC")==0){
		response_status_b4[DQ_MC] = OLD;
		strcpy(val,response_buffer_b4[DQ_MC]);
	}
	else if(strcmp(sns,"MV")==0){
		response_status_b4[DQ_MV] = OLD;
		strcpy(val,response_buffer_b4[DQ_MV]);
	}
	else if(strcmp(sns,"MD")==0){
		response_status_b4[DQ_MD] = OLD;
		strcpy(val,response_buffer_b4[DQ_MD]);
	}
	else if(strcmp(sns,"ME")==0){
		response_status_b4[DQ_ME] = OLD;
		strcpy(val,response_buffer_b4[DQ_ME]);
	}
	else {
		// error msg?
	}
}
// TODO: initializing low for GPIO for cutdown and board resets.
void store_response_val(int brd_num,char* sns, char*val){
	if(!((brd_num>=0)&&(brd_num<=4))){
		// error msg?
	}
	if((brd_num >= 0) && (brd_num <= 3)){
		if(strcmp(sns,"TD0")==0){
			strcpy(response_buffer[brd_num][PI_TD0],val);
			response_status[brd_num][PI_TD0] = NEW;
		}
		else if(strcmp(sns,"TB0")==0){
			strcpy(response_buffer[brd_num][PI_TB0],val); // TODO: Null char needed?
			response_status[brd_num][PI_TB0] = NEW;
		}
		else if(strcmp(sns,"TB1")==0){
			strcpy(response_buffer[brd_num][PI_TB1],val);
			response_status[brd_num][PI_TB1] = NEW;
		}
		else if(strcmp(sns,"TE0")==0){
			strcpy(response_buffer[brd_num][PI_TE0],val);
			response_status[brd_num][PI_TE0] = NEW;
		}
		else if(strcmp(sns,"TE1")==0){
			strcpy(response_buffer[brd_num][PI_TE1],val);
			response_status[brd_num][PI_TE1] = NEW;
		}
		else if(strcmp(sns,"P0")==0){
			strcpy(response_buffer[brd_num][PI_P0],val);
			response_status[brd_num][PI_P0] = NEW;
		}
		else if(strcmp(sns,"P1")==0){
			strcpy(response_buffer[brd_num][PI_P1],val);
			response_status[brd_num][PI_P1] = NEW;
		}
		else if(strcmp(sns,"H")==0){
			strcpy(response_buffer[brd_num][PI_H],val);
			response_status[brd_num][PI_H] = NEW;
		}
		else if(strcmp(sns,"V")==0){
			strcpy(response_buffer[brd_num][PI_V],val);
			response_status[brd_num][PI_V] = NEW;
		}
		else if(strcmp(sns,"C")==0){
			strcpy(response_buffer[brd_num][PI_C],val);
			response_status[brd_num][PI_C] = NEW;
		}
		else {
			// error msg?
		}
	}
	else if(brd_num == 4){
		if(strcmp(sns,"TB0")==0){
			strcpy(response_buffer_b4[DQ_TB0],val);
			response_status_b4[DQ_TB0] = NEW;
		}
		else if(strcmp(sns,"P0")==0){
			strcpy(response_buffer_b4[DQ_P0],val);
			response_status_b4[DQ_P0] = NEW;
		}
		else if(strcmp(sns,"PB")==0){
			strcpy(response_buffer_b4[DQ_PB],val);
			response_status_b4[DQ_PB] = NEW;
		}
		else if(strcmp(sns,"V")==0){
			strcpy(response_buffer_b4[DQ_V],val);
			response_status_b4[DQ_V] = NEW;
		}
		else if(strcmp(sns,"C")==0){
			strcpy(response_buffer_b4[DQ_C],val);
			response_status_b4[DQ_C] = NEW;
		}
		else if(strcmp(sns,"XGY")==0){
			strcpy(response_buffer_b4[DQ_XGY],val);
			response_status_b4[DQ_XGY] = NEW;
		}
		else if(strcmp(sns,"XAC")==0){
			strcpy(response_buffer_b4[DQ_XAC],val);
			response_status_b4[DQ_XAC] = NEW;
		}
		else if(strcmp(sns,"YGY")==0){
			strcpy(response_buffer_b4[DQ_YGY],val);
			response_status_b4[DQ_YGY] = NEW;
		}
		else if(strcmp(sns,"YAC")==0){
			strcpy(response_buffer_b4[DQ_YAC],val);
			response_status_b4[DQ_YAC] = NEW;
		}
		else if(strcmp(sns,"ZGY")==0){
			strcpy(response_buffer_b4[DQ_ZGY],val);
			response_status_b4[DQ_ZGY] = NEW;
		}
		else if(strcmp(sns,"ZAC")==0){
			strcpy(response_buffer_b4[DQ_ZAC],val);
			response_status_b4[DQ_ZAC] = NEW;
		}
		else if(strcmp(sns,"MS")==0){
			strcpy(response_buffer_b4[DQ_MS],val);
			response_status_b4[DQ_MS] = NEW;
		}
		else if(strcmp(sns,"MC")==0){
			strcpy(response_buffer_b4[DQ_MC],val);
			response_status_b4[DQ_MC] = NEW;
		}
		else if(strcmp(sns,"MV")==0){
			strcpy(response_buffer_b4[DQ_MV],val);
			response_status_b4[DQ_MV] = NEW;
		}
		else if(strcmp(sns,"MD")==0){
			strcpy(response_buffer_b4[DQ_MD],val);
			response_status_b4[DQ_MD] = NEW;
		}
		else if(strcmp(sns,"ME")==0){
			strcpy(response_buffer_b4[DQ_ME],val);
			response_status_b4[DQ_ME] = NEW;
		}
		else {
			// error msg?
		}
	}
	else {
		// error msg?
	}
}
void parse_response(char* msg){
	char msg_copy[MSG_LEN];
	char* resp_brd = "";
	char* resp_sns = "";
	char* resp_val = "";
	strcpy(msg_copy,msg);
	rmv_start_end_chars(msg_copy);
	int count = get_colon_count(msg_copy);
	if (count == 2){
		// Insert specific code for handling 2 colon commands or call fnc
		two_colon_extract(msg_copy,&resp_brd,&resp_sns,&resp_val);
		// TODO: LP future can do error checking for making sure valid msg
		if(strcmp(resp_brd,"B4")==0){
			if(resp_val){
				store_response_val(4, resp_sns, resp_val);
			}
		}
		else if(strcmp(resp_brd,"B1")==0){
			store_response_val(1, resp_sns, resp_val);
		}
		else if(strcmp(resp_brd,"B2")==0){
			store_response_val(2, resp_sns, resp_val);
		}
		else if(strcmp(resp_brd,"B3")==0){
			store_response_val(3, resp_sns, resp_val);
		}
		else if(strcmp(resp_brd,"B0")==0){
			store_response_val(0, resp_sns, resp_val);
		}
		else {
			// error msg?
		}
	}
	else {
		// error msg
	}
}
void parse_response_pi_hat(int brd_num, char* msg){
	char msg_copy[MSG_LEN];
	char* resp_sns = "";
	char* resp_val = "";
	strcpy(msg_copy,msg);
	rmv_start_end_chars(msg_copy);
	int count = get_colon_count(msg_copy);
	if (count == 1){
		one_colon_extract(msg_copy,&resp_sns,&resp_val);
		if(resp_val){
			store_response_val(brd_num, resp_sns, resp_val);
		}
	}
	else {
		// error msg
	}
}
void parse_cmd_from_comms(char* msg){
	char msg_copy[MSG_LEN] = {};
	char* comms2_cmd = "";
	char* comms2_val = "";
	char* comms3_cmd = "";
	char* comms3_brd = "";
	char* comms3_sns = "";
	char* resp_val = "";
	strcpy(msg_copy,msg);
	rmv_start_end_chars(msg_copy);
	int count = get_colon_count(msg_copy);
	switch(count)
		{
			case 0:
				// Insert specific code for handling 0 colon commands or call fnc
				if(strcmp(msg_copy,"01")==0){
					spi_slv_fsm_state = OBTAINING_DATA;
					create_respond_all_data_msg();
					spi_slv_fsm_state = RESPONDING_WITH_ALL_DATA;
				}
				else if(strcmp(msg_copy,"FF")==0){
					// Trigger Cutdown
					cutdown();
					spi_slv_fsm_state = LISTENING_FOR_COMMAND;
				}
				else {
					// error msg?
				}
				break;
			case 1:
				// Insert specific code for handling 1 colon commands or call fnc
				one_colon_extract(msg_copy,&comms2_cmd,&comms2_val);
				if((strcmp(comms2_cmd,"03")==0)||(strcmp(comms2_cmd,"04")==0)){
					// forward reaction wheel command to motor msp
					if(strcmp(comms2_cmd,"03")==0){
						if(*comms2_val == '0'){
							P8OUT &= ~(BIT0);
						}
						else if(*comms2_val == '1'){
							P8OUT |= (BIT0);
						}
					}
//					SPI_command_host_to_slave_no_response(msg, &spi_mst_sendDoneFG);
					spi_slv_fsm_state = LISTENING_FOR_COMMAND;
				}
				else if(strcmp(comms2_cmd,"05")==0){
					if(strcmp(comms2_val,"B0")==0){
						// Trigger Board Reset to B0
						reset_pi(0);
						spi_slv_fsm_state = LISTENING_FOR_COMMAND;
					}
					else if(strcmp(comms2_val,"B1")==0){
						// Trigger Board Reset to B1
						reset_pi(1);
						spi_slv_fsm_state = LISTENING_FOR_COMMAND;
					}
					else if(strcmp(comms2_val,"B2")==0){
						// Trigger Board Reset to B2
						reset_pi(2);
						spi_slv_fsm_state = LISTENING_FOR_COMMAND;
					}
					else if(strcmp(comms2_val,"B3")==0){
						// Trigger Board Reset to B3
						reset_pi(3);
						spi_slv_fsm_state = LISTENING_FOR_COMMAND;
					}
					else {
						// error msg?
					}
				}
				else if(strcmp(comms2_cmd,"06")==0){
					// forward same msg to all 4 pis
					UART_write_msg(0,msg);
					UART_write_msg(1,msg);
					UART_write_msg(2,msg);
					UART_write_msg(3,msg);
					// forward same msg to motor MSP
					SPI_command_host_to_slave_no_response(msg, &spi_mst_sendDoneFG);
					spi_slv_fsm_state = LISTENING_FOR_COMMAND;
					// update own timestamp. TODO:
				}
				else {
					// error msg?
				}

				break;
			case 2:
				// Insert specific code for handling 2 colon commands or call fnc
				two_colon_extract(msg_copy,&comms3_cmd,&comms3_brd,&comms3_sns);
				// TODO: LP future can do error checking for making sure valid msg from comms for other areas
				if(strcmp(comms3_brd,"B0")==0){
					spi_slv_fsm_state = OBTAINING_DATA;
					read_response_val(0, comms3_sns, &resp_val);
					create_comms_response(comms3_brd,comms3_sns,resp_val);
					spi_slv_fsm_state = RESPONDING_WITH_DATA;
				}
				else if(strcmp(comms3_brd,"B1")==0){
					spi_slv_fsm_state = OBTAINING_DATA;
					read_response_val(1, comms3_sns, &resp_val);
					create_comms_response(comms3_brd,comms3_sns,resp_val);
					spi_slv_fsm_state = RESPONDING_WITH_DATA;
				}
				else if(strcmp(comms3_brd,"B2")==0){
					spi_slv_fsm_state = OBTAINING_DATA;
					read_response_val(2, comms3_sns, &resp_val);
					create_comms_response(comms3_brd,comms3_sns,resp_val);
					spi_slv_fsm_state = RESPONDING_WITH_DATA;
				}
				else if(strcmp(comms3_brd,"B3")==0){
					spi_slv_fsm_state = OBTAINING_DATA;
					read_response_val(3, comms3_sns, &resp_val);
					create_comms_response(comms3_brd,comms3_sns,resp_val);
					spi_slv_fsm_state = RESPONDING_WITH_DATA;
				}
				else if(strcmp(comms3_brd,"B4")==0){
					spi_slv_fsm_state = OBTAINING_DATA;
					read_response_val(4, comms3_sns, &resp_val);
					create_comms_response(comms3_brd,comms3_sns,resp_val);
					spi_slv_fsm_state = RESPONDING_WITH_DATA;
				}
				else {
					// error msg?
				}
				break;

			default: break;
		}
}
void parse_cmd_from_host(char* msg){
	char msg_copy[MSG_LEN] = {};
	char* host2_cmd = "";
	char* host2_val = "";
	char* host3_cmd = "";
	char* host3_brd = "";
	char* host3_sns = "";
	strcpy(msg_copy,msg);
	rmv_start_end_chars(msg_copy);
	int count = get_colon_count(msg_copy);
	switch(count)
		{
			case 0:
				if(strcmp(msg_copy,"01")==0){
					// Support this? TODO: LP

				}
				else {
					// error msg?
				}
				break;
			case 1:
				// Insert specific code for handling 1 colon commands or call fnc
				one_colon_extract(msg_copy,&host2_cmd,&host2_val);
				if(strcmp(host2_cmd,"03")==0){
					// Insert Reaction wheel cmd TODO:

					spi_slv_fsm_state = LISTENING_FOR_COMMAND;
				}
				else if(strcmp(host2_cmd,"04")==0){
					// Insert Reaction wheel cmd TODO:

					spi_slv_fsm_state = LISTENING_FOR_COMMAND;
				}
				else if(strcmp(host2_cmd,"06")==0){
					// Insert Time Sync CMD TODO:

					spi_slv_fsm_state = LISTENING_FOR_COMMAND;
				}
				else {
					// error msg?
				}
				break;
			case 2:
				// Insert specific code for handling 2 colon commands or call fnc
				two_colon_extract(msg_copy,&host3_cmd,&host3_brd,&host3_sns);
				if(strcmp(host3_brd,"B4")==0){
					spi_slv_fsm_state = OBTAINING_DATA;
//					read_response_val(4, host3_sns, &resp_val);val_b4
//					create_host_response(host3_sns,resp_val);
					read_response_val_b4(host3_sns,val_b4);
					create_host_response(host3_sns,val_b4);
					spi_slv_fsm_state = RESPONDING_WITH_DATA;
				}
				else {
					// error msg?
				}
				break;

			default: break;
		}
}
void create_comms_response(char* brd, char* sns, char* val){
	char spi_send_msg_temp[MSG_LEN] = {};
	strcpy(spi_send_msg_temp,"{");
	strcat(spi_send_msg_temp,brd);
	strcat(spi_send_msg_temp,":");
	strcat(spi_send_msg_temp,sns);
	strcat(spi_send_msg_temp,":");
	strcat(spi_send_msg_temp,val);
	strcat(spi_send_msg_temp,"}");
	strcpy(spi_slv_send_message,spi_send_msg_temp);
}

void create_host_response(char* sns, char* val){
	char spi_send_msg_temp[MSG_LEN] = {};
	strcpy(spi_send_msg_temp,"{B4:");
	strcat(spi_send_msg_temp,sns);
	strcat(spi_send_msg_temp,":");
	strcat(spi_send_msg_temp,val);
	strcat(spi_send_msg_temp,"}");
	strcpy(spi_slv_send_message,spi_send_msg_temp);
}

void one_colon_extract(char* msg, char** first, char** second){
	*first = strtok(msg,":");
	*second = strtok(NULL,":");
}
void two_colon_extract(char* msg, char** first, char** second, char** third){
	*first = strtok(msg,":");
	*second = strtok(NULL,":");
	*third = strtok(NULL,":");
}

void config_RST_PI_GPIO(void){
	// erase previous configs for bp
	P4SEL1 &= ~(BIT4 | BIT5 | BIT6 | BIT7);
	P4SEL0 &= ~(BIT4 | BIT5 | BIT6 | BIT7);
	// set as outputs
	P4DIR |= (BIT4 | BIT5 | BIT6 | BIT7);
	// set low (assuming active high)
	P4OUT &= ~(BIT4 | BIT5 | BIT6 | BIT7);
}
void reset_pi(int brd_num){
	// dependent: config_rst_pi_GPIO();
	// Note: currently designed assuming active high board reset
	// Note: currently designed assuming 100ms @ 8MHz hold
	switch(brd_num)
	{
	case 0:
		P4OUT |= (BIT4);
		__delay_cycles(800000);
		P4OUT &= ~(BIT4);
		break;
	case 1:
		P4OUT |= (BIT5);
		__delay_cycles(800000);
		P4OUT &= ~(BIT5);
		break;
	case 2:
		P4OUT |= (BIT6);
		__delay_cycles(800000);
		P4OUT &= ~(BIT6);
		break;
	case 3:
		P4OUT |= (BIT7);
		__delay_cycles(800000);
		P4OUT &= ~(BIT7);
		break;
	default: break;
	}
}

void config_CUTDOWN_GPIO(void){
	// erase previous configs for bp
	P4SEL1 &= ~(BIT0);
	P4SEL0 &= ~(BIT0);
	// set as outputs
	P4DIR |= (BIT0);
	// set low (assuming active high)
	P4OUT &= ~(BIT0);
}

void cutdown(void){
	// configured for 10 seconds
	// TODO: RTC?
	P4OUT |= (BIT0);
	__delay_cycles(80000000);
	P4OUT &= ~(BIT0);
}

void grab_all_motor_msp(void){
//	grab_all_daqcs(TB0);
//	grab_all_daqcs(P0);
//	grab_all_daqcs(PB);
//	grab_all_daqcs(V);
//	grab_all_daqcs(C);
//	grab_all_daqcs(XGY);
//	grab_all_daqcs(XAC);
//	grab_all_daqcs(YGY);
//	grab_all_daqcs(YAC);
	grab_all_daqcs(ZGY);
//	grab_all_daqcs(ZAC);
	grab_all_daqcs(MS);
	grab_all_daqcs(MC);
//	grab_all_daqcs(MV);
	grab_all_daqcs(MD);
	grab_all_daqcs(ME);
}
