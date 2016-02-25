#include <msp430.h> 
#include "clock_f5.h"

/*
 * main.c
 */
int main(void) {

	WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer
	setup_clock();
	enable_ACLK_out();
	enable_SMCLK_out();
	return 0;
}
