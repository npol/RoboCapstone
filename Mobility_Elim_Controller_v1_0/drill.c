/*
 * drill.c
 * Created 3/20/16 by Nishant Pol
 * Robotics Capstone 16-474
 * MSP430F5521
 * Mobility/Elimination Controller Code: Drill functions
 *
 * Code Composer v6
 * Version History
 * 3/20/16: Creation
 */

#include "drill.h"

/** Drill Motor Task functions **/
/* Setup Drill logic and driver
 * Set DRILL_S1, DRILL_S2, and DRILL_EN as outputs
 */
void drill_setup(void){
	P7DIR |= BIT4+BIT5+BIT6;
	P7OUT &= ~(BIT4+BIT5+BIT6);
	return;
}

/* Start drill with configured direction */
void drill_enable(void){
	P7OUT |= BIT6;
	return;
}

/* Stop drill, and disable driver (coast) */
void drill_disable(void){
	P7OUT &= ~BIT6;
	return;
}

/* Configure direction to be CW */
void drill_cw(void){
	P7OUT |= BIT4;
	P7OUT &= ~BIT5;
	return;
}

/* Configure direction to be CCW */
void drill_ccw(void){
	P7OUT &= ~BIT4;
	P7OUT |= BIT5;
	return;
}

/* Short drill through GND to brake */
void drill_brake(void){
	P7OUT &= ~(BIT4+BIT5);
	return;
}
