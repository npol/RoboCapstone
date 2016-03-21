/*
 * stepper.h
 * Created 3/20/16 by Nishant Pol
 * Robotics Capstone 16-474
 * MSP430F5521
 * Mobility/Elimination Controller Code: Stepper functions
 *
 * Code Composer v6
 * Version History
 * 3/20/16: Creation
 */

#ifndef STEPPER_H_
#define STEPPER_H_
#include <msp430.h>
#include "utils.h"

#define STEP_PORT P1OUT
#define STEP_PIN BIT2

void stepper_setup(void);
void stepper_step_single(void);
void stepper_enable(uint8_t direction);
void stepper_disable(void);




#endif /* STEPPER_H_ */
