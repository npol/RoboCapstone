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
