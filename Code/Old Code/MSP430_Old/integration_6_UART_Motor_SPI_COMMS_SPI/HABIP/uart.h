/*
 * UART.h
 *
 *  Created on: Apr 11, 2017
 *      Author: Lincster
 */

#ifndef HABIP_UART_H_
#define HABIP_UART_H_

#define LISTENING_FOR_RESPONSE 0x00
#define CAPTURING_RESPONSE 0x01
#define MSG_BUFFER_FULL 0x02
#define UART_FSM_STATE_CNT 3

// buffer status
#define AVAILABLE 0x00
#define VALID 0x01

  // UART
void config_UART_GPIO(int brd_num);
void config_UART_9600_ACLK_32768Hz(int brd_num);
void config_UART_9600_SMCLK_8MHz(int brd_num);
void config_UART_9600_SMCLK_250KHz(int brd_num);
void UART_write_msg(int brd_num, char* message);
void UART_parse(int brd_num);

#define UART_ISR(brd_num) ({\
        if(uart_status[brd_num] >= PI_HAT_SENSOR_CNT){\
          uart_fsm_state[brd_num] = MSG_BUFFER_FULL;\
        }\
          /* MSG_BUFFER_FULL*/\
        if(uart_fsm_state[brd_num] == MSG_BUFFER_FULL){\
          if(uart_status[brd_num] < PI_HAT_SENSOR_CNT){\
            uart_fsm_state[brd_num] = LISTENING_FOR_RESPONSE;\
          }\
          else {\
            while(!(UCA##brd_num##IFG&UCTXIFG));\
              UCA##brd_num##TXBUF = 'F'; /* TODO: Chris OK with?*/\
              uart_index[brd_num]++;\
          }\
        }\
      /* LISTENING_FOR_RESPONSE*/\
        if(uart_fsm_state[brd_num] == LISTENING_FOR_RESPONSE){\
          if(uart_read_buffer[brd_num][uart_index[brd_num]] == 0x7B){\
            /* find open buffer slot and if none found, go to buffer full state*/\
            for(uart_status_index[brd_num] = 0; uart_status_index[brd_num] <PI_HAT_SENSOR_CNT; uart_status_index[brd_num]++){\
              if(uart_read_message_buffer_status[brd_num][uart_status_index[brd_num]]==AVAILABLE){\
                uart_fsm_state[brd_num] = CAPTURING_RESPONSE;\
                uart_read_index[brd_num] = 0; /* May cause overwriting in future*/\
                msg_buffer_index[brd_num] = uart_status_index[brd_num];\
                break;\
              }\
              else if(uart_status_index[brd_num] == PI_HAT_SENSOR_CNT-1){\
                uart_fsm_state[brd_num] = MSG_BUFFER_FULL;\
              }\
            }\
          }\
          else{\
            while(!(UCA##brd_num##IFG&UCTXIFG));\
            UCA##brd_num##TXBUF = 'L';\
            uart_index[brd_num]++;\
          }\
        }\
      /* CAPTURING_RESPONSE*/\
        if(uart_fsm_state[brd_num] == CAPTURING_RESPONSE){\
          uart_read_message_buffer[brd_num][msg_buffer_index[brd_num]][uart_read_index[brd_num]] = uart_read_buffer[brd_num][uart_index[brd_num]];\
          if(uart_read_message_buffer[brd_num][msg_buffer_index[brd_num]][uart_read_index[brd_num]] == 0x7D){ /* TODO: If end of msg len stop reading in...*/\
            uart_fsm_state[brd_num] = LISTENING_FOR_RESPONSE;\
            uart_read_message_buffer[brd_num][msg_buffer_index[brd_num]][uart_read_index[brd_num]+1] = '\0';\
            uart_read_message_buffer_status[brd_num][uart_status_index[brd_num]]=VALID;\
            msg_buffer_index[brd_num]++;\
            uart_status[brd_num]++;\
            while(!(UCA##brd_num##IFG&UCTXIFG));\
            UCA##brd_num##TXBUF = 'D';\
          }\
          else{\
            uart_read_index[brd_num]++;\
            uart_index[brd_num]++;\
            while(!(UCA##brd_num##IFG&UCTXIFG));\
            UCA##brd_num##TXBUF = 'C';\
          }\
        }\
        if(uart_index[brd_num] == MSG_LEN){\
          uart_index[brd_num] = 0;\
        }\
})


#endif /* HABIP_UART_H_ */
