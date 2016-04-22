/*
 * clock_f5.c
 *
 *  Created on: Feb 24, 2016
 *      Author: nishant
 */

#include "clock_f5.h"

//#define XT1_EN
//#define XT2_EN_4MHZ
#define XT2_EN_25MHZ

static void SetVCoreUp(unsigned int level);

/* Setup clock system:
 * MCLK: sourced from XT2
 * SMCLK: sourced from XT2
 * ACLK: sourced from XT2
 * XT2 is 4MHz on launchpad, 25MHz on controllers
 */
void setup_clock(void){
    // Increase Vcore setting to required level to support fsystem.
    // 0 = 8MHz, 1 = 12MHz, 2 = 20MHz, 3 = 25MHz.
    // NOTE: Change core voltage one level at a time.
#ifdef XT2_EN_25MHZ
    SetVCoreUp (0x01);
    SetVCoreUp (0x02);
    SetVCoreUp (0x03);
#endif
#ifdef XT1_EN
    // *** Setup XT1 *********************************
    P5SEL |= BIT4 | BIT5;                   // Port select XT1
    UCSCTL6 &= ~XT1OFF;                     // XT1 On
    // If required enable internal XT1 load cap's
    UCSCTL6 |= XCAP_3;
    //
    // Loop until XT1 fault flag is cleared
    do {UCSCTL7 &= ~XT1LFOFFG;}             // Clear XT1 fault flags
    while ((UCSCTL7 & XT1LFOFFG) != 0); // Test XT1 fault flag
    //
    // Decrease XT1 Drive according to expected frequency
    UCSCTL6 &= ~XT1DRIVE0;
#endif
    // *** Setup XT2 *********************************
    P5SEL |= BIT2 | BIT3;                   // Port select XT2
    UCSCTL6 &= ~XT2OFF;                     // XT2 On
    //
    // Loop until XT2 fault flag is cleared
    do {UCSCTL7 &= ~XT2OFFG;}               // Clear XT2 fault flags
    while ((UCSCTL7 & XT2OFFG) != 0);       // Test XT2 fault flag
    //
    // Decrease XT2 Drive according to expected frequency
    UCSCTL6 &= ~XT2DRIVE_3;

    // Set Clock sources
    UCSCTL4 = SELA__XT2CLK | SELS__XT2CLK | SELM__XT2CLK;
#ifndef XT1_EN
    //If XT1 not used, clear flag so interrupt doesn't fire
    UCSCTL7 &= ~XT1LFOFFG;
#endif
    // Stop DCO
    __bis_SR_register(SCG0 | SCG1);
    // Clear DCO fault flag
    UCSCTL7 &= ~DCOFFG;

    // Now that osc is running enable fault interrupt.
    // Add an UNMI_ISR to handle the fault.
    SFRIFG1 &= ~OFIFG;      // Clear Oscillator fault interrupt flag
    SFRIE1 |= OFIE;         // Enable fault interrupt
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

/* Datasheet provided function to increase core voltage
 * for high frequency operation
 */
static void SetVCoreUp(unsigned int level){
	// Open PMM registers for write access
	PMMCTL0_H = 0xA5;
	// Make sure no flags are set for iterative sequences
	//while (!((PMMIFG & SVSMHDLYIFG) == 0));
	//while (!((PMMIFG & SVSMLDLYIFG) == 0));
	// Set SVS/SVM high side new level
	SVSMHCTL = SVSHE + SVSHRVL0 * level + SVMHE + SVSMHRRL0 * level;
	// Set SVM low side to new level
	SVSMLCTL = SVSLE + SVMLE + SVSMLRRL0 * level;
	// Wait till SVM is settled
	while ((PMMIFG & SVSMLDLYIFG) == 0);
	// Clear already set flags
	PMMIFG &= ~(SVMLVLRIFG + SVMLIFG);
	// Set VCore to new level
	PMMCTL0_L = PMMCOREV0 * level;
	// Wait till new level reached
	if ((PMMIFG & SVMLIFG))
		while ((PMMIFG & SVMLVLRIFG) == 0);
	// Set SVS/SVM low side to new level
	SVSMLCTL = SVSLE + SVSLRVL0 * level + SVMLE + SVSMLRRL0 * level;
	// Lock PMM registers for write access
	PMMCTL0_H = 0x00;
	return;
}
