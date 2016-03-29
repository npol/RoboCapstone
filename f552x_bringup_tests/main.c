/*
 * main.c
 * Created 2/29/16 by Nishant Pol
 * Robotics Capstone 16-474/Mechatronics 18-578
 * MSP430F5521/9
 * Bringup test code
 *
 * Code Composer v6
 * Version History
 * 2/29/16: Initial release
 */
#include <msp430.h> 
#include "utils.h"
#include <string.h>
#include "clock_f5.h"
#include "dbg_uart_uscia0.h"

/** Debug task macros and globals **/
void debug_task(void);

#define DEBUG_CMD_BUF_SIZE 32
uint8_t debug_cmd_buf[DEBUG_CMD_BUF_SIZE];
uint8_t debug_cmd_buf_ptr = 0;
/** END Debug task macros and globals **/

//#define BLINK
//#define BLINK_MECH
//#define CLK_TEST
//#define DBG_UART_TX_TEST
#define DBG_UART_TEST

#ifdef BLINK
int main(void) {
	WDTCTL = WDTPW | WDTHOLD;		// Stop watchdog timer
	P1DIR |= 0x01;					// Set P1.0 to output direction
	for(;;) {
		volatile unsigned int i;	// volatile to prevent optimization
		P1OUT ^= 0x01;				// Toggle P1.0 using exclusive-OR
		i = 10000;					// SW Delay
		do i--;
		while(i != 0);
	}
	return 0;
}
#endif

#ifdef BLINK_MECH
int main(void) {
	WDTCTL = WDTPW | WDTHOLD;		// Stop watchdog timer
	P1DIR |= 0x01;					// Set P1.0 to output direction
	P2DIR |= BIT2;
	P7DIR |= BIT7;
	P5DIR |= BIT6;
	for(;;) {
		volatile unsigned int i;	// volatile to prevent optimization
		P1OUT ^= 0x01;				// Toggle P1.0 using exclusive-OR
		P2OUT ^= BIT2;
		P7OUT ^= BIT7;
		P5OUT ^= BIT6;
		i = 10000;					// SW Delay
		do i--;
		while(i != 0);
	}
	return 0;
}
#endif

#ifdef CLK_TEST
int main(void) {
	WDTCTL = WDTPW + WDTHOLD;   // Stop watchdog timer
	setup_clock();
    // Enable Interrupts
    __bis_SR_register(GIE);
    enable_ACLK_out();
    enable_SMCLK_out();
    enable_MCLK_out();
    while(1)
    {
        // ...
    }
}
#endif

#ifdef DBG_UART_TX_TEST
int main(void) {
	WDTCTL = WDTPW + WDTHOLD;   // Stop watchdog timer
	setup_clock();
	setup_dbg_uart();
    // Enable Interrupts
    __bis_SR_register(GIE);
    while(1)
    {
        dbg_uart_send_byte('A');
    }
}
#endif

#ifdef DBG_UART_TEST
int main(void) {
	WDTCTL = WDTPW + WDTHOLD;   // Stop watchdog timer
	P1DIR |= BIT0;	//Debug LED1
	P2DIR |= BIT2;	//Debug LED2
	P7DIR |= BIT7;	//Debug LED3
	P4DIR |= BIT7;	//Debug ERR LED
	P5DIR |= BIT6;	//Debug WARN LED
	P5DIR |= BIT7;	//Debug OK LED
	setup_clock();
	setup_dbg_uart();
    // Enable Interrupts
    __bis_SR_register(GIE);
    while(1)
    {
        debug_task();
    }
}
#endif

/** Debug Task functions **/

/* Process debug command functions */
void debug_task(void){
	uint8_t debug_cmd_ready = 0;
	uint8_t response_buf[DEBUG_CMD_BUF_SIZE];
	uint8_t response_size = 0;
	//Check if serial data is ready to be formed into a command
	if(is_dbg_uart_rx_data_ready()){
		uint8_t rx_byte = dbg_uart_get_byte();

		//Enter (CR) indicates command is complete
		if(rx_byte == 13){			//Enter pressed (CR)
			dbg_uart_send_byte(13);		//CR
			dbg_uart_send_byte(10);		//Line feed
			debug_cmd_ready = 1;	//Reset command buffer
		} else {//Fill command buffer
			dbg_uart_send_byte(rx_byte);	//Echo back character
			debug_cmd_buf[debug_cmd_buf_ptr] = rx_byte;
			if(debug_cmd_buf_ptr < DEBUG_CMD_BUF_SIZE){
				debug_cmd_buf_ptr++;
			} else {
				while(1);
				//TODO Buffer is full
			}

		}

	}
	//Process command
	if(debug_cmd_ready){
		if(debug_cmd_buf_ptr == 0){
			//No command, do nothing
		} else if((strncmp(debug_cmd_buf,"led1 on",6)==0) && (debug_cmd_buf_ptr == 7)){
			//>led1 on
			led_P1_0_on();
		} else if((strncmp(debug_cmd_buf,"led1 off",7)==0) && (debug_cmd_buf_ptr == 8)){
			//>led1 off
			led_P1_0_off();
		} else if((strncmp(debug_cmd_buf,"led2 on",6)==0) && (debug_cmd_buf_ptr == 7)){
			//>led2 on
			led_P2_2_on();
		} else if((strncmp(debug_cmd_buf,"led2 off",7)==0) && (debug_cmd_buf_ptr == 8)){
			//>led2 off
			led_P2_2_off();
		} else if((strncmp(debug_cmd_buf,"led3 on",6)==0) && (debug_cmd_buf_ptr == 7)){
			//>led3 on
			led_P7_7_on();
		} else if((strncmp(debug_cmd_buf,"led3 off",7)==0) && (debug_cmd_buf_ptr == 8)){
			//>led3 off
			led_P7_7_off();
		} else if((strncmp(debug_cmd_buf,"led4 on",6)==0) && (debug_cmd_buf_ptr == 7)){
			//>led4 on
			led_P5_7_on();
		} else if((strncmp(debug_cmd_buf,"led4 off",7)==0) && (debug_cmd_buf_ptr == 8)){
			//>led4 off
			led_P5_7_off();
		} else if((strncmp(debug_cmd_buf,"led5 on",6)==0) && (debug_cmd_buf_ptr == 7)){
			//>led5 on
			led_P5_6_on();
		} else if((strncmp(debug_cmd_buf,"led5 off",7)==0) && (debug_cmd_buf_ptr == 8)){
			//>led5 off
			led_P5_6_off();
		} else if((strncmp(debug_cmd_buf,"led6 on",6)==0) && (debug_cmd_buf_ptr == 7)){
			//>led6 on
			led_P4_7_on();
		} else if((strncmp(debug_cmd_buf,"led6 off",7)==0) && (debug_cmd_buf_ptr == 8)){
			//>led6 off
			led_P4_7_off();
/*		} else if((strncmp(debug_cmd_buf,"button get",10)==0) && (debug_cmd_buf_ptr == 10)){
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
			uart_send_string(response_buf,response_size);*/
		} else if((strncmp(debug_cmd_buf,"P1 get",6)==0) && (debug_cmd_buf_ptr == 6)){
			response_size = P1_get(response_buf);
			dbg_uart_send_string(response_buf,response_size);
		} else if((strncmp(debug_cmd_buf,"P2 get",6)==0) && (debug_cmd_buf_ptr == 6)){
			response_size = P2_get(response_buf);
			dbg_uart_send_string(response_buf,response_size);
		} else if((strncmp(debug_cmd_buf,"P3 get",6)==0) && (debug_cmd_buf_ptr == 6)){
			response_size = P3_get(response_buf);
			dbg_uart_send_string(response_buf,response_size);
		} else if((strncmp(debug_cmd_buf,"P4 get",6)==0) && (debug_cmd_buf_ptr == 6)){
			response_size = P4_get(response_buf);
			dbg_uart_send_string(response_buf,response_size);
		} else if((strncmp(debug_cmd_buf,"P5 get",6)==0) && (debug_cmd_buf_ptr == 6)){
			response_size = P5_get(response_buf);
			dbg_uart_send_string(response_buf,response_size);
		} else if((strncmp(debug_cmd_buf,"P6 get",6)==0) && (debug_cmd_buf_ptr == 6)){
			response_size = P6_get(response_buf);
			dbg_uart_send_string(response_buf,response_size);
		} else if((strncmp(debug_cmd_buf,"P7 get",6)==0) && (debug_cmd_buf_ptr == 6)){
			response_size = P7_get(response_buf);
			dbg_uart_send_string(response_buf,response_size);
		} else if((strncmp(debug_cmd_buf,"P8 get",6)==0) && (debug_cmd_buf_ptr == 6)){
			response_size = P8_get(response_buf);
			dbg_uart_send_string(response_buf,response_size);
/*		} else if((strncmp(debug_cmd_buf,"can tx",6)==0) && (debug_cmd_buf_ptr == 11)){
			//can tx 0x00
			uint8_t temp = ascii2hex_byte(debug_cmd_buf[9],debug_cmd_buf[10]);
			can_tx_message_buf0(0x279, 1, &temp);
#ifndef PC_CAN
			uart_send_string("tx",2);
#endif
		} else if((strncmp(debug_cmd_buf,"can rx",6)==0) && (debug_cmd_buf_ptr == 6)){
			response_size = debug_can_rx(response_buf);
			uart_send_string(response_buf, response_size);*/
		} else {
			dbg_uart_send_string("Invalid Command",15);
		}
		dbg_uart_send_byte(13);		//CR
		dbg_uart_send_byte(10);		//Line feed
		dbg_uart_send_byte('>');	//Terminal prompt
		debug_cmd_ready = 0;
		debug_cmd_buf_ptr = 0;
	}
}

/** END Debug Task functions **/

/** Interrupts **/
/* Debug UART USCIA0 InterruptHandler
 * UART Rx: Recieves incoming bytes from debug channel, puts into UART datastructure
 * UART Tx: Sends bytes from UART datastructure to debug channel
 */
#pragma vector=USCI_A0_VECTOR
__interrupt void USCIA0_ISR(void){
	if((UCA0IE & UCRXIE) && (UCA0IFG & UCRXIFG)){	//UART Rxbuf full interrupt
		//Get byte and clear interrupt
		DBG_UART_data.rx_bytes[DBG_UART_data.rx_head] = UCA0RXBUF;
		DBG_UART_data.rx_head++;
		//Wraparound condition
		if(DBG_UART_data.rx_head >= DBG_UART_RX_BUF_SIZE){
			DBG_UART_data.rx_head = 0;
		}
		//if(DBG_UART_data.rx_head == DBG_UART_data.rx_tail)
			//TODO: Log error: buffer full
	} else if((UCA0IE & UCTXIE) && (UCA0IFG & UCTXIFG)){	//UART Txbuf ready interrupt
		//Load data and clear interrupt
		UCA0TXBUF = DBG_UART_data.tx_bytes[DBG_UART_data.tx_tail];
		DBG_UART_data.tx_tail++;
		//Wraparound condition
		if(DBG_UART_data.tx_tail >= DBG_UART_TX_BUF_SIZE){
			DBG_UART_data.tx_tail = 0;
		}
		//Disable Tx interrupt if last byte in buffer has been transmitted
		if(DBG_UART_data.tx_tail == DBG_UART_data.tx_head){
			disable_dbg_uart_txint();
		}
	} else {
		//TODO: Log Error, should never go here
		while(1);
	}
}


/* System Unmaskable Interrupt Handler
 * NMIIFG:
 * OFIFG: Oscillator Fault
 */
#pragma vector=UNMI_VECTOR
__interrupt void unmi_isr(void){
	switch(__even_in_range(SYSUNIV, 0x08)){
		case 0x00: break;
		case 0x02: break; // NMIIFG
		case 0x04: 			// OFIFG
			if(UCSCTL7 & XT2OFFG){		//XT2 Oscillator fault
				//TODO: Add error condition
			}
			if(UCSCTL7 & XT1LFOFFG){	//XT1 Oscillator failt, low frequency mode
				//TODO: Add error condition
			}
			if(UCSCTL7 & DCOFFG){		//DCO fault
				//TODO: Add error condition
			}
			break;
		case 0x06: break; // ACCVIFG
		case 0x08: // BUSIFG
			// If needed, obtain the flash error location here.
			//ErrorLocation = MidGetErrAdr();
			switch(__even_in_range(SYSBERRIV, 0x08)){
				case 0x00: break; // no bus error
				case 0x02: break; // USB bus error
				case 0x04: break; // reserved
				case 0x06: // MID error
					//<place your MID error handler code here>
					break;
				case 0x08: break;
				default: break;
			}
			break;
		default: break;
	}
}
/** END Interrupts **/
