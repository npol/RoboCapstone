/*
 * clock_f5.c
 *
 *  Created on: Feb 24, 2016
 *      Author: nishant
 */

#include "clock_f5.h"

/* Setup clock system:
 * MCLK: sourced from XT2
 * SMCLK: sourced from XT2
 * ACLK: sourced from XT2
 * XT2 is 4MHz on launchpad, 25MHz on controllers
 */
void setup_clock(void){
	SetVcoreUp(0x01);
	//Enable XT2 on P5.2 and P5.3
	P5SEL |= BIT2|BIT3;
	//Enable XT2
	UCSCTL6 &= ~XT2OFF;
	//Set FLL divider to 1
	//UCSCTL2 &= ~(FLLD_7|0x3ff);
	UCSCTL2 =1;
	//Set FLL source to XT2
	UCSCTL3 |= SELREF_5;
	//Source ACLK, MCLK, and SMCLK from XT2
	UCSCTL4 |= SELA_5|SELS_5|SELS_5;
	return;
}

/* Enable ACLK to be output on P1.0 */
void enable_ACLK_out(void){
	P1DIR |= BIT0;
	P1SEL |= BIT0;
}

/* Return P1.0 to Digital Output */
void disable_ACLK_out(void){
	P1SEL &= ~BIT0;
	P1OUT &= ~BIT0;
}

/* Enable SMCLK to be output on P2.2 */
void enable_SMCLK_out(void){
	P2SEL |= BIT2;
	P2DIR |= BIT2;
}

/* Return P2.2 to Digital Output */
void disable_SMCLK_out(void){
	P2SEL &= ~BIT2;
	P2OUT &= ~BIT2;
}

/* Enable MCLK to be output on P7.7*/
void enable_MCLK_out(void){
	P7SEL |= BIT7;
	P7DIR |= BIT7;
}

/* Return P7.7 to Digital Output */
void disable_MCLK_out(void){
	P7SEL &= ~BIT7;
	P7OUT &= ~BIT7;
}
