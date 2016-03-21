/*
 * drill.h
 * Created 3/20/16 by Nishant Pol
 * Robotics Capstone 16-474
 * MSP430F5521
 * Mobility/Elimination Controller Code: Drill functions
 *
 * Code Composer v6
 * Version History
 * 3/20/16: Creation
 */

#ifndef DRILL_H_
#define DRILL_H_

#include <msp430.h>
#include "utils.h"

void drill_setup(void);
void drill_enable(void);
void drill_disable(void);
void drill_cw(void);
void drill_ccw(void);
void drill_brake(void);

#endif /* DRILL_H_ */
