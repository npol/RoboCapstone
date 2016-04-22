/*
 * pc_uart_uscia1.c
 * Created 6/20/15 by Nishant Pol
 * Robotics Capstone 16-474/Mechatronics 18-578
 * MSP430F552x UART driver
 * PC UART on USCIA1
 *
 * Code Composer v6
 * Version History
 * 6/20/15: Initial version for MSP430G2 family
 * 2/29/16: Modified for MSP430F5 family
 * 4/21/16: Ported from Debug UART driver (USCIA0)
 */
#include "pc_uart_uscia1.h"

//#define CLK_4MHZ
#define CLK_25MHZ

volatile struct PC_UART_data_struct PC_UART_data = {
	.tx_bytes = {0},
	.rx_bytes = {0},
	.tx_head = 0,
	.tx_tail = 0,
	.rx_head = 0,
	.rx_tail = 0,
};

/* USCIA1_UART setup
 * 115200baud, no parity, 1 stop bit, 8-bit data, LSB first
 */
void setup_pc_uart(void){
	UCA1CTL1 |= UCSWRST;			//Hold USCI in reset
	UCA1CTL1 = UCSSEL_2 + UCSWRST;	//Source from SMCLK
#ifdef CLK_4MHZ
	UCA1BR1 = 0;					//UCBRx = 2 for 115200baud
	UCA1BR0 = 34;
	UCA1MCTL = UCBRS_6;
#endif
#ifdef CLK_25MHZ
	UCA1BR1 = 0;
	UCA1BR0 = 0xd9;
	UCA1MCTL = 0;
#endif
	P4SEL |= BIT4 + BIT5;			//P4.4: TXD, P4.5: RXD
	UCA1CTL1 &= ~UCSWRST;			//Release USCI from reset
	UCA1IE |= UCRXIE;				//Enable Rx interrupt
	return;
}

void disable_pc_uart_txint(void){
	UCA1IE &= ~UCTXIE;
	return;
}

void enable_pc_uart_txint(void){
	UCA1IE |= UCTXIE;
	return;
}

void pc_uart_send_byte(uint8_t data){
	PC_UART_data.tx_bytes[PC_UART_data.tx_head] = data;
	PC_UART_data.tx_head++;
	if(PC_UART_data.tx_head >= PC_UART_TX_BUF_SIZE){	//Wraparound condition
		PC_UART_data.tx_head = 0;
	}
	if(PC_UART_data.tx_head == PC_UART_data.tx_tail){	//Buffer overflow
		issue_warning(WARN_PC_TX_BUF_FULL1);
	}
	enable_pc_uart_txint();		//Data sent via UART interrupt
	return;
}

void pc_uart_send_string(uint8_t *data, uint8_t size){
	uint8_t i = 0;
	for(i = 0; i < size; i++){
		PC_UART_data.tx_bytes[PC_UART_data.tx_head] = data[i];
		PC_UART_data.tx_head++;
		if(PC_UART_data.tx_head >= PC_UART_TX_BUF_SIZE){
			PC_UART_data.tx_head = 0;
		}
		if(PC_UART_data.tx_tail == PC_UART_data.tx_head){	//Buffer overflow
			issue_warning(WARN_PC_TX_BUF_FULL2);
		}
	}
	enable_pc_uart_txint();
	return;
}

/* Application calls function to check if there is data
 * to be processed
 */
uint8_t is_pc_uart_rx_data_ready(void){
	return PC_UART_data.rx_tail != PC_UART_data.rx_head;
}

uint8_t pc_uart_get_byte(void){
	uint8_t rx_byte = PC_UART_data.rx_bytes[PC_UART_data.rx_tail];
	PC_UART_data.rx_tail++;
	if(PC_UART_data.rx_tail >= PC_UART_RX_BUF_SIZE){	//Wraparound condition
		PC_UART_data.rx_tail = 0;
	}
	return rx_byte;
}
