/*
 * rc_uart_uscia1.h
 * Created 6/20/15 by Nishant Pol
 * Robotics Capstone 16-474/Mechatronics 18-578
 * MSP430F552x UART driver
 * Roboclaw UART Driver on USCIA1
 *
 * Code Composer v6
 * Version History
 * 6/20/15: Initial version for MSP430G2 family
 * 2/29/16: Modified for MSP430F5 family
 * 3/18/16: Ported from dbg_uart_uscia0.c, modified for uscia1
 */

#ifndef RC_UART_USCIA1_H_
#define RC_UART_USCIA1_H_

#include <msp430.h>
#include "utils.h"

#define RC_UART_TX_BUF_SIZE 250
#define RC_UART_RX_BUF_SIZE 250

struct RC_UART_data_struct{
	uint8_t tx_bytes[RC_UART_TX_BUF_SIZE];	//Bytes to send
	uint8_t rx_bytes[RC_UART_RX_BUF_SIZE];	//Bytes recieved
	uint8_t tx_head;					//Index of byte to transmit next
	uint8_t tx_tail;					//Index of next empty byte slot in tx buffer
	uint8_t rx_head;					//Index of oldest byte recieved
	uint8_t rx_tail;					//Index of next empty byte slot in rx buffer
};
extern volatile struct RC_UART_data_struct RC_UART_data;

void setup_rc_uart(void);
void disable_rc_uart_txint(void);
void enable_rc_uart_txint(void);
void rc_uart_send_byte(uint8_t data);
void rc_uart_send_string(uint8_t *data, uint8_t size);
uint8_t rc_uart_get_byte(void);
uint8_t rc_uart_get_string(uint8_t *buffer, uint8_t size);
uint8_t is_rc_uart_rx_data_ready(void);
uint8_t is_rc_uart_rx_ndata_ready(void);



#endif /* RC_UART_USCIA1_H_ */


