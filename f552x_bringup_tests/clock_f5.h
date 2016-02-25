/*
 * clock_f5.h
 *
 *  Created on: Feb 24, 2016
 *      Author: nishant
 */

#ifndef CLOCK_F5_H_
#define CLOCK_F5_H_

#include <msp430.h>

void setup_clock(void);
void enable_ACLK_out(void);
void disable_ACLK_out(void);
void enable_SMCLK_out(void);
void disable_SMCLK_out(void);
void enable_MCLK_out(void);
void disable_MCLK_out(void);



#endif /* CLOCK_F5_H_ */
