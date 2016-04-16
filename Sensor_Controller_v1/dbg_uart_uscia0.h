/*
 * dbg_uart_uscia0.h
 * Created 6/20/15 by Nishant Pol
 * Robotics Capstone 16-474/Mechatronics 18-578
 * MSP430F552x UART driver
 * Debug UART on USCIA0
 *
 * Code Composer v6
 * Version History
 * 6/20/15: Initial version for MSP430G2 family
 * 2/29/16: Modified for MSP430F5 family
 */

#ifndef DBG_UART_USCIA0_H_
#define DBG_UART_USCIA0_H_

#include <msp430.h>
#include "utils.h"

#define DBG_UART_TX_BUF_SIZE 250
#define DBG_UART_RX_BUF_SIZE 250

struct DBG_UART_data_struct{
	uint8_t tx_bytes[DBG_UART_TX_BUF_SIZE];	//Bytes to send
	uint8_t rx_bytes[DBG_UART_RX_BUF_SIZE];	//Bytes recieved
	uint8_t tx_head;					//Index of byte to transmit next
	uint8_t tx_tail;					//Index of next empty byte slot in tx buffer
	uint8_t rx_head;					//Index of oldest byte recieved
	uint8_t rx_tail;					//Index of next empty byte slot in rx buffer
};
extern volatile struct DBG_UART_data_struct DBG_UART_data;

void setup_dbg_uart(void);
void disable_dbg_uart_txint(void);
void enable_dbg_uart_txint(void);
void dbg_uart_send_byte(uint8_t data);
void dbg_uart_send_string(uint8_t *data, uint8_t size);
uint8_t dbg_uart_get_byte(void);
uint8_t dbg_uart_get_string(uint8_t *buffer, uint8_t size);
uint8_t is_dbg_uart_rx_data_ready(void);



#endif /* DBG_UART_USCIA0_H_ */

