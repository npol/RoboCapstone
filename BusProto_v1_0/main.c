/* main.c
 * Created 11/25/15 by Nishant Pol
 * 16-450/16-474 Robotics Capstone
 * CAN Bus Prototype
 *
 * Code Composer v6
 * MSP430G2553IPW28
 *
 * Revision History
 * 11/25/15: Nishant Pol
 * - Initial release
 * 1/24/15: Nishant Pol
 * - Added LED blink loop for HW testing, then removed loop
 * - Added SPI commands to check communication to MCP2515
 * 2/9/16: Nishant Pol
 * - Added Debug UART Interface
 */
#include <msp430.h> 
#include "utils.h"
#include "clock.h"
#include "spi.h"
#include "MCP2515.h"
#include "uart.h"


/*
 * main.c
 */
int main(void) {
    WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer
    setup_clock();
    P1DIR = BIT5+BIT6+BIT7;
    P1OUT = 0x00;
    P2DIR = BIT5;
    P2OUT = 0x00;
    P3DIR = 0xff;
    P3OUT = 0x00;
    USCIB0_SPI_setup(0,1);		//Setup SPI, idle low, out on falling edge
    setup_uart();
    __enable_interrupt();
    setup_mcp2515();
    uint8_t btn_state_prev = 0;
	P3OUT |= 0xf;
	uint16_t i;
	for(i=0; i < 40000; i++);
	P3OUT &= ~0xf;

	while(1){
		uart_send_string("Hello World!\n",13);
		uint16_t i;
		for(i=0; i < 40000; i++);
		for(i=0; i < 40000; i++);
		for(i=0; i < 40000; i++);
		for(i=0; i < 40000; i++);
		for(i=0; i < 40000; i++);
		for(i=0; i < 40000; i++);
		for(i=0; i < 40000; i++);
		for(i=0; i < 40000; i++);
		for(i=0; i < 40000; i++);
		for(i=0; i < 40000; i++);

	}

    while(1){
    	//read buttons
    	uint8_t btn_state = ~P2IN & 0xf;
    	if(btn_state != btn_state_prev){
    		btn_state_prev = btn_state;
    		//Send CAN message to update LEDs
        	mcp2515_write_register(0x36, btn_state);	//Data, 1 byte
        	mcp2515_write_register(0x35, 1);			//1 byte Data frame
        	mcp2515_write_register(0x31, 0x4f);			//SID[10:3] = 0x4f
        	mcp2515_write_register(0x32, 0x40);			//SID[2:0] = 0x2
        	mcp2515_write_register(0x30, 0x0B);			//Transmit message
    	}
    	//output to LEDs if CAN message recieved
    	int i;

    	if(P1IN&BIT3){
    		P3OUT = (P3OUT & 0xf0) | mcp2515_read_register(0x66);
        	mcp2515_write_register(0x2C, 0);	//Clear interrupts
        	while(!(P1IN&BIT3)){	//Check if interrupt cleared
        		P3OUT ^= 0xf;
        		uint16_t i;
        		for(i=0; i < 40000; i++);
        		mcp2515_write_register(0x2C, 0);	//Clear interrupts
        	}

    	}
    }

    while(1){
    	uint16_t i;
    	volatile uint8_t rx_bytes[4];	//SID[10:3], SID[2:0], data
    	volatile uint8_t val = 0;
    	P3OUT |= 0xf;
    	for(i=0; i<10000;i++);
    	P3OUT &= ~0xf;
    	for(i=0; i<10000;i++);
    	//Send dummy message
    	val = mcp2515_read_register(0x0E);
    	mcp2515_write_register(0x36, 0x55);	//Data = 0x55, 1 byte
    	mcp2515_write_register(0x35, 1);	//1 byte Data frame
    	mcp2515_write_register(0x31, 0x4f);	//SID[10:3] = 0x4f
    	mcp2515_write_register(0x32, 0x40);	//SID[2:0] = 0x2
    	mcp2515_write_register(0x30, 0x0B);	//Transmit message
    	while(P1IN & BIT3);	//Wait for RXBUF0 interrupt
    	for(i=0;i<10000;i++);
    	rx_bytes[0] = mcp2515_read_register(0x61);	//Read data recieved
    	rx_bytes[1] = mcp2515_read_register(0x62);
    	rx_bytes[2] = mcp2515_read_register(0x65);
    	rx_bytes[3] = mcp2515_read_register(0x66);
    	mcp2515_write_register(0x2C, 0);	//Clear interrupts
    	if(!(P1IN & BIT3)){
    		while(1);	//interrupt didn't clear
    	}
    	//i++;
    	//for(i=0;i<10000;i++);
    }


	
	return 0;
}

/* SPI/UART Rx Interrupt Handler
 * SPI Rx: puts most recent character in SPI datastructure
 * UART Rx:
 */
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void){
	if((IFG2 & UCA0RXIFG) && (IE2 & UCA0RXIE)){	//UART Rxbuf full interrupt
		//Get byte and clear interrupt
		UART_data.rx_bytes[UART_data.rx_tail] = UCA0RXBUF;
		UART_data.rx_tail++;
		//Wraparound condition
		if(UART_data.rx_tail >= UART_RX_BUF_SIZE){
			UART_data.rx_tail = 0;
		}
		//if(UART_data.rx_tail == UART_data.rx_head)
			//TODO: Log error: buffer full
	} else
	if((IFG2 & UCB0RXIFG) && (IE2 & UCB0RXIE)){	//SPI Rxbuf full interrupt
		SPI_data.rx_bytes[SPI_data.rx_ptr] = UCB0RXBUF;	//Get latest byte from HW
		SPI_data.rx_ptr++;								//Flag reset with buffer read
		if(SPI_data.rx_ptr >= SPI_data.num_bytes){		//Done reading data
			SPI_CS_DEASSERT;							//Disable CS and disable interrupt
			IE2 &= ~UCB0RXIE;
			SPI_data.data_ready = 1;
		}
	} else {
		//TODO: Log Error, should never go here
	}
}

/* SPI/UART Tx Interrupt Handler
 * SPI Tx: Transmits next character in SPI datastructure
 * UART Tx:
 */
#pragma vector=USCIAB0TX_VECTOR
__interrupt void USCI0TX_ISR(void){
	if((IFG2 & UCA0TXIFG) && (IE2 & UCA0TXIE)){			//UART Txbuf ready interrupt
		//Load data and clear interrupt
		UCA0TXBUF = UART_data.tx_bytes[UART_data.tx_head];
		UART_data.tx_head++;
		//Wraparound condition
		if(UART_data.tx_head >= UART_TX_BUF_SIZE){
			UART_data.tx_head = 0;
		}
		//Disable Tx interrupt if last byte in buffer has been transmitted
		if(UART_data.tx_head == UART_data.tx_tail){
			disable_uart_txint();
		}
	} else
	if((IFG2 & UCB0TXIFG) && (IE2 & UCB0TXIE)){	//SPI Txbuf ready interrupt
		UCB0TXBUF = SPI_data.tx_bytes[SPI_data.tx_ptr];	//Load next byte into HW buffer
		SPI_data.tx_ptr++;								//Flag reset with buffer write
		if(SPI_data.tx_ptr >= SPI_data.num_bytes){		//Done transmitting data
			IE2 &= ~UCB0TXIE;							//Disable Tx interrupt
		}
	} else {
		//TODO: Log Error, should never go here
	}
}
