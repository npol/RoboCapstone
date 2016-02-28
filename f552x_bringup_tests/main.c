#include <msp430.h> 
#include "clock_f5.h"

/*
 * main.c
 */
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
