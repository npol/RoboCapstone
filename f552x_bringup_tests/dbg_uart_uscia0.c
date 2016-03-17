/*
 * dbg_uart_uscia0.c
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
#include "dbg_uart_uscia0.h"

//#define CLK_4MHZ
#define CLK_25MHZ

volatile struct DBG_UART_data_struct DBG_UART_data = {
	.tx_bytes = {0},
	.rx_bytes = {0},
	.tx_head = 0,
	.tx_tail = 0,
	.rx_head = 0,
	.rx_tail = 0,
};

/* USCIA0_UART setup
 * 115200baud, no parity, 1 stop bit, 8-bit data, LSB first
 */
void setup_dbg_uart(void){
	UCA0CTL1 |= UCSWRST;			//Hold USCI in reset
	UCA0CTL1 = UCSSEL_2 + UCSWRST;	//Source from SMCLK
#ifdef CLK_4MHZ
	UCA0BR1 = 0;					//UCBRx = 2 for 115200baud
	UCA0BR0 = 34;
	UCA0MCTL = UCBRS_6;
#endif
#ifdef CLK_25MHZ
	UCA0BR1 = 0;
	UCA0BR0 = 0xd9;
	UCA0MCTL = 0;
#endif
	P3SEL |= BIT3 + BIT4;			//P3.3: TXD, P3.4: RXD
	UCA0CTL1 &= ~UCSWRST;			//Release USCI from reset
	UCA0IE |= UCRXIE;				//Enable Rx interrupt
	return;
}

void disable_dbg_uart_txint(void){
	UCA0IE &= ~UCTXIE;
	return;
}

void enable_dbg_uart_txint(void){
	UCA0IE |= UCTXIE;
	return;
}

void dbg_uart_send_byte(uint8_t data){
	DBG_UART_data.tx_bytes[DBG_UART_data.tx_head] = data;
	DBG_UART_data.tx_head++;
	if(DBG_UART_data.tx_head >= DBG_UART_TX_BUF_SIZE){	//Wraparound condition
		DBG_UART_data.tx_head = 0;
	}
	if(DBG_UART_data.tx_head == DBG_UART_data.tx_tail){	//Buffer overflow
		//TODO Error: buffer full
	}
	enable_dbg_uart_txint();		//Data sent via UART interrupt
	return;
}

void dbg_uart_send_string(uint8_t *data, uint8_t size){
	uint8_t i = 0;
	for(i = 0; i < size; i++){
		DBG_UART_data.tx_bytes[DBG_UART_data.tx_head] = data[i];
		DBG_UART_data.tx_head++;
		if(DBG_UART_data.tx_head >= DBG_UART_TX_BUF_SIZE){
			DBG_UART_data.tx_head = 0;
		}
		if(DBG_UART_data.tx_tail == DBG_UART_data.tx_head){	//Buffer overflow
			//TODO Error: buffer full
		}
	}
	enable_dbg_uart_txint();
	return;
}

/* Application calls function to check if there is data
 * to be processed
 */
uint8_t is_dbg_uart_rx_data_ready(void){
	return DBG_UART_data.rx_tail != DBG_UART_data.rx_head;
}

uint8_t dbg_uart_get_byte(void){
	uint8_t rx_byte = DBG_UART_data.rx_bytes[DBG_UART_data.rx_tail];
	DBG_UART_data.rx_tail++;
	if(DBG_UART_data.rx_tail >= DBG_UART_RX_BUF_SIZE){	//Wraparound condition
		DBG_UART_data.rx_tail = 0;
	}
	if(DBG_UART_data.rx_head == DBG_UART_data.rx_tail){
		//TODO flag error, buffer overflow
	}
	return rx_byte;
}

uint8_t dbg_uart_get_string(uint8_t *buffer, uint8_t size){
	return 0;
}
