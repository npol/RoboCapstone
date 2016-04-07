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

volatile drill_state_t drillCurrState = DRILL_INIT;

volatile drill_req_t drill_request = DRILL_REQ_NONE;

/* Drill motor task
 *
 */
void drill_task(void){
	static uint16_t drill_timer = 0;
	switch(drillCurrState){
	case DRILL_INIT:
		//State acton
		drill_disable();
		//State transition
		drillCurrState = DRILL_WAIT;
		break;
	case DRILL_WAIT:
		//State action: none
		//State transition
		if(!sys_ok){
			drillCurrState = DRILL_ERR;
		} else if(drill_request == DRILL_REQ_NONE){
			drillCurrState = DRILL_WAIT;
		} else {
			drillCurrState = DRILL_SET_DIR;
		}
		break;
	case DRILL_SET_DIR:
		//State action
		if(drill_request == DRILL_REQ_OFF){
			drill_disable();
			drill_request == DRILL_REQ_NONE;
		} else if(drill_request == DRILL_REQ_CW){
			drill_disable();
			drill_cw();
			drill_enable();
		} else if(drill_request == DRILL_REQ_CCW){
			drill_disable();
			drill_cw();
			drill_enable();
		} else if(drill_request == DRILL_REQ_BRAKE){
			drill_disable();
			drill_brake();
			drill_enable();
		} else {
			issue_warning(WARN_ILLEGAL_DRILL_REQUEST);
			drill_disable();
			drill_request == DRILL_REQ_NONE;
		}
		drill_timer = 0;
		//State transition
		if(!sys_ok){
			drillCurrState = DRILL_ERR;
		} if(drill_request == DRILL_REQ_NONE){
			drillCurrState = DRILL_WAIT;
		} else if((drill_request == DRILL_REQ_CW) || (drill_request == DRILL_REQ_CCW)){
			drillCurrState = DRILL_RUN;
			drill_request = DRILL_REQ_NONE;
		} else {
			issue_warning(WARN_ILLEGAL_DRILL_REQUEST2);
		}
		break;
	case DRILL_RUN:
		//State action
		drill_timer++;
		//State transition
		if(!sys_ok){
			drillCurrState = DRILL_ERR;
		} if(drill_timer > DRILL_TIMEOUT){
			drillCurrState = DRILL_STOP;
		} else if(drill_request != DRILL_REQ_NONE){
			drillCurrState = DRILL_SET_DIR;
		} else {
			drillCurrState = DRILL_RUN;
		}
		break;
	case DRILL_STOP:
		//State action
		drill_disable();
		//State transition
		if(!sys_ok){
			drillCurrState = DRILL_ERR;
		} else {
			drillCurrState = DRILL_WAIT;
		}
		break;
	case DRILL_ERR:
		//State action
		drill_disable();
		//State transition
		if(sys_ok){
			drillCurrState = DRILL_WAIT;
		} else {
			drillCurrState = DRILL_ERR;
		}
		break;
	default:
		issue_warning(WARN_ILLEGAL_DRILL_SM_STATE);
		drillCurrState = DRILL_WAIT;
		break;
	}
}

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
