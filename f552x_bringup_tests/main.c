#include <msp430.h> 
#include "clock_f5.h"
#include "dbg_uart_uscia0.h"

//#define BLINK
//#define CLK_TEST
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

#ifdef DBG_UART_TEST
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
