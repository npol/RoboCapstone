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

typedef enum  {DRILL_INIT,
				DRILL_WAIT,
				DRILL_SET_DIR,
				DRILL_RUN,
				DRILL_STOP,
				DRILL_ERR} drill_state_t;

typedef enum  {DRILL_REQ_NONE,
				DRILL_REQ_OFF,
				DRILL_REQ_CW,
				DRILL_REQ_CCW,
				DRILL_REQ_BRAKE} drill_req_t;
#define DRILL_TIMEOUT 1000000
extern volatile uint8_t sys_ok;
extern volatile drill_req_t drill_request;



void drill_setup(void);
void drill_enable(void);
void drill_disable(void);
void drill_cw(void);
void drill_ccw(void);
void drill_brake(void);
void drill_task(void);

#endif /* DRILL_H_ */
