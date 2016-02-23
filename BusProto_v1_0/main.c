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

#define PC_CAN

void button_task(void);
void led_task(void);
inline uint8_t P1_get(uint8_t *buf);
inline uint8_t P2_get(uint8_t *buf);
inline uint8_t P3_get(uint8_t *buf);
uint8_t btn_state_prev = 0;

/** CAN macros and globals **/
void can_tx_message_buf0(uint16_t sid, uint8_t len, uint8_t *data);

#define CAN_RX_BUF_SIZE 8
uint8_t can_rx_buf[CAN_RX_BUF_SIZE];
uint8_t can_rx_buf_head = 0;
uint8_t can_rx_buf_tail = 0;

/** END CAN macros and globals **/
/** Debug task macros and globals **/
void debug_task(void);
void debug_rx_task(void);

inline void led_on(uint8_t led);
inline void led_off(uint8_t led);
inline uint8_t button_get(uint8_t *buf);
inline uint8_t debug_mcp2515_read_reg(uint8_t *debug_cmd_buf,uint8_t *response_buf);
inline uint8_t debug_mcp2515_write_reg(uint8_t *debug_cmd_buf,uint8_t *response_buf);
inline uint8_t ascii2hex_byte(uint8_t high_char, uint8_t low_char);
inline void hex2ascii_byte(uint8_t data, uint8_t *high_char, uint8_t *low_char);
inline uint8_t debug_can_rx(uint8_t *response_buf);


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

	P3OUT |= 0xf;
	uint16_t i;
	for(i=0; i < 40000; i++);
	setup_mcp2515();
	P3OUT &= ~0xf;

	while(1){
		debug_task();
		button_task();
		//led_task();
		debug_rx_task();
	}
}

/** Main application task functions **/
void button_task(void){
	//read buttons
	uint8_t btn_state = ~P2IN & 0xf;
	if(btn_state != btn_state_prev){
		btn_state_prev = btn_state;
		//Send CAN message to update LEDs
		can_tx_message_buf0(0x279,1,&btn_state);
#ifndef PC_CAN
		uart_send_string("CAN TX",6);
#endif
	}
}

void led_task(void){
	if(!(P1IN&BIT3)){
		P3OUT = (P3OUT & 0xf0) | (0xf&mcp2515_read_register(0x66));
		mcp2515_write_register(0x2C, 0);	//Clear interrupts
		if(mcp2515_read_register(0x2D)&BIT6){
#ifndef PC_CAN
			uart_send_string("Overflow",8);
#endif
			mcp2515_write_register(0x2D,0x00);
		}
		while(!(P1IN&BIT3)){	//Check if interrupt cleared
			P3OUT ^= 0xf;
			uint16_t i;
			for(i=0; i < 40000; i++);
			mcp2515_write_register(0x2C, 0);	//Clear interrupts
		}
#ifndef PC_CAN
		uart_send_string("CAN RX",6);
#endif
	}
}

void debug_rx_task(void){
	uint8_t rx_byte = 0;
	if(!(P1IN&BIT3)){
		led_on(0);
		rx_byte = mcp2515_read_register(0x66);
		P3OUT = (P3OUT & 0xf0) | (0xf&rx_byte);
		mcp2515_write_register(0x2C, 0);	//Clear interrupts
		if(mcp2515_read_register(0x2D)&BIT6){
#ifndef PC_CAN
			uart_send_string("Overflow",8);
#endif
			mcp2515_write_register(0x2D,0x00);
		}
		while(!(P1IN&BIT3)){	//Check if interrupt cleared
			P3OUT ^= 0x7;
			uint16_t i;
			for(i=0; i < 40000; i++);
			mcp2515_write_register(0x2C, 0);	//Clear interrupts
		}
#ifndef PC_CAN
		uart_send_string("CAN RX",6);
#endif
		can_rx_buf[can_rx_buf_head] = rx_byte;
		can_rx_buf_head++;
		if(can_rx_buf_head >= CAN_RX_BUF_SIZE){
			can_rx_buf_head = 0;
		}
		led_off(0);
	}
	return;
}

/** END main application task functions **/

/** CAN functions **/

/* Initiate message transmission
 * sid: Standard identifier, lowest 10 bits
 * len: number of data bytes
 * data: array of data bytes
 */
void can_tx_message_buf0(uint16_t sid, uint8_t len, uint8_t *data){
	//Standard Identifier
	mcp2515_write_register(MCP2515_TXB0SIDH,(uint8_t)(sid>>3));
	mcp2515_write_register(MCP2515_TXB0SIDL,(uint8_t)(sid<<5));
	//Data length
	mcp2515_write_register(MCP2515_TXB0DLC,len);
	//Data bytes
	uint8_t i;
	for(i=0; i<len;i++){
		mcp2515_write_register(MCP2515_TXB0D0+i,data[i]);
	}
	//Transmit request
	mcp2515_write_register(MCP2515_TXB0CTRL, MCP2515_TXREQ|MCP2515_TXP_3);
}

/** END CAN functions **/

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
				led_on(0);
				while(1);
				//Buffer is full
			}

		}

	}
	//Process command
	if(debug_cmd_ready){
		led_on(1);
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
		} else if((strncmp(debug_cmd_buf,"P1 get",6)==0) && (debug_cmd_buf_ptr == 6)){
			response_size = P1_get(response_buf);
			uart_send_string(response_buf,response_size);
		} else if((strncmp(debug_cmd_buf,"P2 get",6)==0) && (debug_cmd_buf_ptr == 6)){
			response_size = P2_get(response_buf);
			uart_send_string(response_buf,response_size);
		} else if((strncmp(debug_cmd_buf,"P3 get",6)==0) && (debug_cmd_buf_ptr == 6)){
			response_size = P3_get(response_buf);
			uart_send_string(response_buf,response_size);
		} else if((strncmp(debug_cmd_buf,"can tx",6)==0) && (debug_cmd_buf_ptr == 11)){
			//can tx 0x00
			uint8_t temp = ascii2hex_byte(debug_cmd_buf[9],debug_cmd_buf[10]);
			can_tx_message_buf0(0x279, 1, &temp);
#ifndef PC_CAN
			uart_send_string("tx",2);
#endif
		} else if((strncmp(debug_cmd_buf,"can rx",6)==0) && (debug_cmd_buf_ptr == 6)){
			response_size = debug_can_rx(response_buf);
			uart_send_string(response_buf, response_size);
		} else {
			uart_send_string("Invalid Command",15);
		}
		uart_send_byte(13);		//CR
		uart_send_byte(10);		//Line feed
		uart_send_byte('>');	//Terminal prompt
		debug_cmd_ready = 0;
		debug_cmd_buf_ptr = 0;
		led_off(1);
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

inline uint8_t P1_get(uint8_t *buf){
	buf[0] = '0';
	buf[1] = 'x';
	hex2ascii_byte(P1IN,&buf[2],&buf[3]);
	return 4;
}

inline uint8_t P2_get(uint8_t *buf){
	buf[0] = '0';
	buf[1] = 'x';
	hex2ascii_byte(P2IN,&buf[2],&buf[3]);
	return 4;
}

inline uint8_t P3_get(uint8_t *buf){
	buf[0] = '0';
	buf[1] = 'x';
	hex2ascii_byte(P3IN,&buf[2],&buf[3]);
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

inline uint8_t debug_can_rx(uint8_t *response_buf){
	uint8_t response_size = 0;
	//Check if there is data
	if(can_rx_buf_head != can_rx_buf_tail){
#ifndef PC_CAN
		hex2ascii_byte(can_rx_buf[can_rx_buf_tail],&response_buf[2],&response_buf[3]);
#else
		hex2ascii_byte(can_rx_buf[can_rx_buf_tail],&response_buf[0],&response_buf[1]);
#endif
		can_rx_buf_tail++;
		if(can_rx_buf_tail >= CAN_RX_BUF_SIZE){
			can_rx_buf_tail = 0;
		}
#ifndef PC_CAN
		response_buf[0] = '0';
		response_buf[1] = 'x';
		response_size = 4;
#else
		response_size = 2;
#endif
	} else {//Buffer empty
		response_buf[0] = 'n';
		response_buf[1] = 'a';
		response_size = 2;
	}
	return response_size;
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
		while(1);
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
		while(1);
	}
}
