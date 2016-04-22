/*
 * pc_uart_uscia1.h
 * Created 6/20/15 by Nishant Pol
 * Robotics Capstone 16-474
 * MSP430F552x UART driver
 * PC UART on USCIA1
 *
 * Code Composer v6
 */

#ifndef PC_UART_USCIA1_H_
#define PC_UART_USCIA1_H_

#include <msp430.h>
#include "utils.h"

#define PC_UART_TX_BUF_SIZE 250
#define PC_UART_RX_BUF_SIZE 250

struct PC_UART_data_struct{
	uint8_t tx_bytes[PC_UART_TX_BUF_SIZE];	//Bytes to send
	uint8_t rx_bytes[PC_UART_RX_BUF_SIZE];	//Bytes recieved
	uint8_t tx_head;					//Index of byte to transmit next
	uint8_t tx_tail;					//Index of next empty byte slot in tx buffer
	uint8_t rx_head;					//Index of oldest byte recieved
	uint8_t rx_tail;					//Index of next empty byte slot in rx buffer
};
extern volatile struct PC_UART_data_struct PC_UART_data;

void setup_pc_uart(void);
void disable_pc_uart_txint(void);
void enable_pc_uart_txint(void);
void pc_uart_send_byte(uint8_t data);
void pc_uart_send_string(uint8_t *data, uint8_t size);
uint8_t pc_uart_get_byte(void);
uint8_t is_pc_uart_rx_data_ready(void);



#endif /* PC_UART_USCIA1_H_ */

