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
#include <string.h>

/** Debug task macros and globals **/
void debug_task(void);
inline void led_on(uint8_t led);
inline void led_off(uint8_t led);
inline uint8_t button_get(uint8_t *buf);
inline uint8_t debug_mcp2515_read_reg(uint8_t *debug_cmd_buf,uint8_t *response_buf);
inline uint8_t debug_mcp2515_write_reg(uint8_t *debug_cmd_buf,uint8_t *response_buf);
inline uint8_t ascii2hex_byte(uint8_t high_char, uint8_t low_char);
inline void hex2ascii_byte(uint8_t data, uint8_t *high_char, uint8_t *low_char);

#define DEBUG_CMD_BUF_SIZE 32
uint8_t debug_cmd_buf[DEBUG_CMD_BUF_SIZE];
uint8_t debug_cmd_buf_ptr = 0;
/** END Debug task macros and globals **/


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
		debug_task();

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

/** Debug Task functions **/

void debug_task(void){
	uint8_t debug_cmd_ready = 0;
	uint8_t response_buf[DEBUG_CMD_BUF_SIZE];
	uint8_t response_size = 0;
	//Check if serial data is ready to be formed into a command
	if(is_uart_rx_data_ready()){
		uint8_t rx_byte = uart_get_byte();

		//Enter (CR) indicates command is complete
		if(rx_byte == 13){			//Enter pressed (CR)
			uart_send_byte(13);		//CR
			uart_send_byte(10);		//Line feed
			debug_cmd_ready = 1;	//Reset command buffer
		} else {//Fill command buffer
			uart_send_byte(rx_byte);	//Echo back character
			debug_cmd_buf[debug_cmd_buf_ptr] = rx_byte;
			if(debug_cmd_buf_ptr < DEBUG_CMD_BUF_SIZE){
				debug_cmd_buf_ptr++;
			} else {
				//Buffer is full
			}

		}

	}
	//Process command
	if(debug_cmd_ready){
		if(debug_cmd_buf_ptr == 0){
			//No command, do nothing
		} else if((strncmp(debug_cmd_buf,"led on",6)==0) && (debug_cmd_buf_ptr == 8)){
			//>led on <led number 0:3>
			//>led on 0
			led_on(debug_cmd_buf[7]-'0');
		} else if((strncmp(debug_cmd_buf,"led off",7)==0) && (debug_cmd_buf_ptr == 9)){
			//>led off <led number 0:3>
			//>led off 0
			led_off(debug_cmd_buf[8]-'0');
		} else if((strncmp(debug_cmd_buf,"button get",10)==0) && (debug_cmd_buf_ptr == 10)){
			//>button get
			response_size = button_get(response_buf);
			uart_send_string(response_buf,response_size);
		} else if((strncmp(debug_cmd_buf,"can regread",11)==0) && (debug_cmd_buf_ptr == 16)){
			//>can regread <register in hex>
			//>can regread 0x00
			response_size = debug_mcp2515_read_reg(debug_cmd_buf,response_buf);
			uart_send_string(response_buf,response_size);
		} else if((strncmp(debug_cmd_buf,"can regwrite",12)==0) && (debug_cmd_buf_ptr == 22)){
			//>can regwrite <register in hex> <data in hex>
			//>can regwrite 0x00 0x00
			response_size = debug_mcp2515_write_reg(debug_cmd_buf,response_buf);
			uart_send_string(response_buf,response_size);
		} else {
			uart_send_string("Invalid Command",15);
		}
		uart_send_byte(13);		//CR
		uart_send_byte(10);		//Line feed
		uart_send_byte('>');	//Terminal prompt
		debug_cmd_ready = 0;
		debug_cmd_buf_ptr = 0;
	}
}

/* Turn on LED
 * led: LED number/port 3 number
 */
inline void led_on(uint8_t led){
	P3OUT |= 0x0f&(1<<led);
	return;
}

/* Turn off LED
 * led: LED number/port 3 number
 */
inline void led_off(uint8_t led){
	P3OUT &= ~(0x0f&(1<<led));
	return;
}

/* Get button status
 * buf: response buffer
 * returns size of message
 */
inline uint8_t button_get(uint8_t *buf){
	uint8_t button_state = P2IN & 0xf;
	buf[0] = '0'+(button_state & 1);
	buf[1] = '0'+((button_state>>1) & 1);
	buf[2] = '0'+((button_state>>2) & 1);
	buf[3] = '0'+((button_state>>3) & 1);
	return 4;
}

/* Manually read MCP2515 register
 * debug_cmd_buf: Character buffer with user command
 * response_buf: Empty buffer to send response
 */
inline uint8_t debug_mcp2515_read_reg(uint8_t *debug_cmd_buf,uint8_t *response_buf){
	uint8_t reg_addr = ascii2hex_byte(debug_cmd_buf[14],debug_cmd_buf[15]);
	uint8_t reg_value = mcp2515_read_register(reg_addr);
	uint8_t ascii_value_high = '0';
	uint8_t ascii_value_low = '0';
	hex2ascii_byte(reg_value, &ascii_value_high, &ascii_value_low);
	response_buf[0] = 'r';
	response_buf[1] = 'e';
	response_buf[2] = 'a';
	response_buf[3] = 'd';
	response_buf[4] = ' ';
	response_buf[5] = '0';
	response_buf[6] = 'x';
	response_buf[7] = debug_cmd_buf[14];
	response_buf[8] = debug_cmd_buf[15];
	response_buf[9] = ':';
	response_buf[10] = ' ';
	response_buf[11] = '0';
	response_buf[12] = 'x';
	response_buf[13] = ascii_value_high;
	response_buf[14] = ascii_value_low;
	return 15;
}

/* Manually write MCP2515 register
 * debug_cmd_buf: Character buffer with user command
 * response_buf: Empty buffer to send response
 */
inline uint8_t debug_mcp2515_write_reg(uint8_t *debug_cmd_buf,uint8_t *response_buf){
	uint8_t reg_addr = ascii2hex_byte(debug_cmd_buf[15],debug_cmd_buf[16]);
	uint8_t data = ascii2hex_byte(debug_cmd_buf[20],debug_cmd_buf[21]);
	mcp2515_write_register(reg_addr, data);
	response_buf[0] = 'w';
	response_buf[1] = 'r';
	response_buf[2] = 'i';
	response_buf[3] = 't';
	response_buf[4] = 'e';
	response_buf[5] = ' ';
	response_buf[6] = '0';
	response_buf[7] = 'x';
	response_buf[8] = debug_cmd_buf[15];
	response_buf[9] = debug_cmd_buf[16];
	response_buf[10] = ':';
	response_buf[11] = ' ';
	response_buf[12] = '0';
	response_buf[13] = 'x';
	response_buf[14] = debug_cmd_buf[20];
	response_buf[15] = debug_cmd_buf[21];
	return 16;
}

/* Create numerical byte from hex ascii characters
 * high_char: ascii code for high nibble
 * low_char: ascii code for low nibble
 * returns numerical byte value
 */
inline uint8_t ascii2hex_byte(uint8_t high_char, uint8_t low_char){
	uint8_t num = 0;
	if(('0'<= high_char) && (high_char <= '9')){
		num = (high_char-'0')<<4;
	} else if(('A' <= high_char) && (high_char <= 'F')){
		num = (high_char-'A'+10)<<4;
	} else if(('a' <= high_char) && (high_char <= 'f')){
		num = (high_char-'a'+10)<<4;
	}
	if(('0'<= low_char) && (low_char <= '9')){
		num |= (low_char-'0');
	} else if(('A' <= low_char) && (low_char <= 'F')){
		num |= (low_char-'A'+10);
	} else if(('a' <= low_char) && (low_char <= 'f')){
		num |= (low_char-'a'+10);
	}
	return num;
}

/* Create ascii character representation of numerical byte
 * data: number to be converted
 * high_char: pointer to high nibble character
 * low_char: pointer to low nibble character
 */
inline void hex2ascii_byte(uint8_t data, uint8_t *high_char, uint8_t *low_char){
	uint8_t upper = (data>>4)&0xf;
	uint8_t lower = (data)&0xf;
	if(upper <= 9){
		*high_char = upper+'0';
	} else if((10 <= upper) && (upper <= 15)){
		*high_char = upper-10+'A';
	}
	if(lower <= 9){
		*low_char = lower+'0';
	} else if((10 <= lower) && (lower <= 15)){
		*low_char = lower-10+'A';
	}
	return;
}


/** END Debug Task functions **/

/* SPI/UART Rx Interrupt Handler
 * SPI Rx: puts most recent character in SPI datastructure
 * UART Rx:
 */
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void){
	if((IFG2 & UCA0RXIFG) && (IE2 & UCA0RXIE)){	//UART Rxbuf full interrupt
		//Get byte and clear interrupt
		UART_data.rx_bytes[UART_data.rx_head] = UCA0RXBUF;
		UART_data.rx_head++;
		//Wraparound condition
		if(UART_data.rx_head >= UART_RX_BUF_SIZE){
			UART_data.rx_head = 0;
		}
		//if(UART_data.rx_head == UART_data.rx_tail)
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
		UCA0TXBUF = UART_data.tx_bytes[UART_data.tx_tail];
		UART_data.tx_tail++;
		//Wraparound condition
		if(UART_data.tx_tail >= UART_TX_BUF_SIZE){
			UART_data.tx_tail = 0;
		}
		//Disable Tx interrupt if last byte in buffer has been transmitted
		if(UART_data.tx_tail == UART_data.tx_head){
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
