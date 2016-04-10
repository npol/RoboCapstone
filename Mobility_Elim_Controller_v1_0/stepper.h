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
#define STEP_TIME 4096	//95Hz

/* Stepper globals for TA0.0 interrupt */
extern int16_t step_setpoint;
extern int16_t step_position;
extern uint8_t step_direction;
extern uint8_t step_run_done;
extern volatile uint8_t sys_ok;

void stepper_setup(void);
void stepper_task(void);
void stepper_step_single(void);
void stepper_enable(uint8_t direction);
void stepper_disable(void);




#endif /* STEPPER_H_ */
