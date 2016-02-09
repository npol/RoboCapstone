/*
 * uart.h
 * Created 6/20/15 by Nishant Pol
 * Consulting for eInsightsLive
 *
 * UART for Xbee communication
 *
 * Code Composer v6
 */

#ifndef UART_H_
#define UART_H_

#include <msp430.h>
#include "utils.h"

#define UART_TX_BUF_SIZE 64
#define UART_RX_BUF_SIZE 64

struct UART_data_struct{
	uint8_t tx_bytes[UART_TX_BUF_SIZE];	//Bytes to send
	uint8_t rx_bytes[UART_RX_BUF_SIZE];	//Bytes recieved
	uint8_t tx_head;					//Index of byte to transmit next
	uint8_t tx_tail;					//Index of next empty byte slot in tx buffer
	uint8_t rx_head;					//Index of oldest byte recieved
	uint8_t rx_tail;					//Index of next empty byte slot in rx buffer
};
extern volatile struct UART_data_struct UART_data;

void setup_uart(void);
void disable_uart_txint(void);
void enable_uart_txint(void);
void uart_send_byte(uint8_t data);
void uart_send_string(uint8_t *data, uint8_t size);
uint8_t uart_get_byte(void);
uint8_t uart_get_string(uint8_t *buffer, uint8_t size);
uint8_t is_uart_rx_data_ready(void);



#endif /* UART_H_ */
