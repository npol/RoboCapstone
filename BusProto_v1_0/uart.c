/*
 * uart.c
 * Created 6/20/15 by Nishant Pol
 * Consulting for eInsightsLive
 *
 * UART for Xbee Communication
 *
 * Code Composer v6
 */
#include "uart.h"

volatile struct UART_data_struct UART_data = {
	.tx_bytes = {0},
	.rx_bytes = {0},
	.tx_head = 0,
	.tx_tail = 0,
	.rx_head = 0,
	.rx_tail = 0,
};

/* USCIA0_UART setup
 * 9600baud, no parity, 1 stop bit, 8-bit data, LSB first
 */
void setup_uart(void){
	UCA0CTL1 |= UCSWRST;			//Hold USCI in reset
	UCA0CTL1 = UCSSEL_2 + UCSWRST;	//Source from SMCLK !MHz
	UCA0BR1 = 0;					//UCBRx = 833 for 9600baud
	UCA0BR0 = 104;
	UCA0MCTL = UCBRS_1;
	P1SEL |= BIT1 + BIT2;			//P1.1: TXD, P1.2: RXD
	P1SEL2 |= BIT1 + BIT2;
	UCA0CTL1 &= ~UCSWRST;			//Release USCI from reset
	IE2 |= UCA0RXIE;		//Enable Rx interrupt
	return;
}

void disable_uart_txint(void){
	IE2 &= ~UCA0TXIE;
	return;
}

void enable_uart_txint(void){
	IE2 |= UCA0TXIE;
	return;
}

void uart_send_byte(uint8_t data){
	UART_data.tx_bytes[UART_data.tx_tail] = data;
	UART_data.tx_tail++;
	if(UART_data.tx_tail >= UART_TX_BUF_SIZE){	//Wraparound condition
		UART_data.tx_tail = 0;
	}
	if(UART_data.tx_tail == UART_data.tx_head){	//Buffer overflow
		//Error: buffer full
	}
	enable_uart_txint();		//Data sent via UART interrupt
	return;
}

void uart_send_string(uint8_t *data, uint8_t size){
	uint8_t i = 0;
	for(i = 0; i < size; i++){
		UART_data.tx_bytes[UART_data.tx_tail] = data[i];
		UART_data.tx_tail++;
		if(UART_data.tx_tail >= UART_TX_BUF_SIZE){
			UART_data.tx_tail = 0;
		}
		if(UART_data.tx_tail == UART_data.tx_head){	//Buffer overflow
			//Error: buffer full
		}
	}
	enable_uart_txint();
	return;
}

uint8_t uart_get_byte(void){
	return 0;

}

uint8_t uart_get_string(uint8_t *buffer, uint8_t size){
	return 0;
}


