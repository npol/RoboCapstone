/*
 * stepper.c
 * Created 3/20/16 by Nishant Pol
 * Robotics Capstone 16-474
 * MSP430F5521
 * Mobility/Elimination Controller Code: Stepper Functions
 *
 * Code Composer v6
 * Version History
 * 3/20/16: Creation
 */
#include "stepper.h"

/* Setup stepper motor driver */
void stepper_setup(void){
	P1DIR |= STEP_PIN;
	P7DIR |= BIT1+BIT2;
	P7OUT |= (BIT1+BIT2);
	return;
}

/* For debugging only: Use to manually step motor */
void stepper_step_single(void){
	STEP_PORT |= STEP_PIN;
	STEP_PORT &= ~STEP_PIN;
	return;
}

/* Enable stepper for specific direction
 * direction: 0: CW, 1: CCW
 */
void stepper_enable(uint8_t direction){
	if(direction){
		P7OUT |= BIT1;
	} else {
		P7OUT &= ~BIT1;
	}
	//Enable
	P7OUT &= ~BIT2;
	return;
}

void stepper_disable(void){
	P7OUT |= BIT2;
	return;
}

