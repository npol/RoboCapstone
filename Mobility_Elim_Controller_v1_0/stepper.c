/*
 * stepper.c
 * Created 3/20/16 by Nishant Pol
 * Robotics Capstone 16-474
 * MSP430F5521
 * Mobility/Elimination Controller Code: Stepper Functions
 *
 * Pinout:
 * P1.2: Stepper STEP
 * P2.0: Lower limit switch (active low)
 * P2.1: Upper limit switch (active low)
 * P7.1: Stepper DIR
 * P7.2: Stepper Enable (active low)
 *
 * Code Composer v6
 * Version History
 * 3/20/16: Creation
 */
#include "stepper.h"

inline uint8_t is_upper_sw_pressed(void);
inline uint8_t is_lower_sw_pressed(void);
inline void start_pwm(void);
inline void stop_pwm(void);

/* Stepper globals for TA0.0 interrupt */
int16_t step_setpoint = 0;
#define STEP_MAX_POS 0x7FFF
#define STEP_HYST_POS -200

#define STEP_HOME_POS -200
int16_t step_position = 0;
uint8_t step_direction = 0;	//0: CW, 1: CCW
#define STEP_DIR_CW 0
#define STEP_DIR_CCW 1
uint8_t step_run_done = 1;	//Set if reached setpoint
uint8_t step_request = 0;		//Flag set if new position is to be requested
int16_t step_request_pos = 0;	//New requested position, referenced from top limit switch
uint8_t step_home_request = 0;	//Flag set if home routine is requested

typedef enum  {STEP_WAIT_HOME,
				STEP_START_FIND_UP_LIMIT,
				STEP_FIND_UP_LIMIT,
				STEP_START_HYST_MOVE,
				STEP_FIND_UP_LIMIT_UP,
				STEP_START_HOME,
				STEP_FIND_HOME,
				STEP_STOP_STEPPER,
				STEP_WAIT,
				STEP_START_MOVE,
				STEP_RUN_MOVE,
				STEP_ERROR} step_state_t;
volatile step_state_t stepCurrState = STEP_WAIT_HOME;

/* Stepper State Machine
 *
 */
void stepper_task(void){
	switch(stepCurrState){
	case STEP_WAIT_HOME:									//STATE_STP1
		//State action
		stop_pwm();
		//State transition
		if(is_lower_sw_pressed()){
			issue_warning(WARN_STEP_FAULT1);
			stepCurrState = STEP_ERROR;						//T_STP0
		} else if(!sys_ok){
			stepCurrState = STEP_WAIT_HOME;
		} else if(step_home_request && !is_upper_sw_pressed()){
			step_home_request = 0;
			stepCurrState = STEP_START_FIND_UP_LIMIT;		//T_STP1
		} else if(step_home_request && is_upper_sw_pressed()){
			step_home_request = 0;
			stepCurrState = STEP_START_HYST_MOVE;
		} else {
			stepCurrState = STEP_WAIT_HOME;					//T_STP2
		}
		break;
	case STEP_START_FIND_UP_LIMIT:							//STATE_STP2
		//State action
		step_setpoint = STEP_MAX_POS;
		step_position = 0;
		step_run_done = 0;
		stepper_enable(STEP_DIR_CW);
		//TODO: check direction is up
		start_pwm();
		//State transition
		if(is_lower_sw_pressed() || !sys_ok){
			issue_warning(WARN_STEP_FAULT2);
			stepCurrState = STEP_ERROR;						//T_STP4
		} else {
			stepCurrState = STEP_FIND_UP_LIMIT;				//T_STP3
		}
		break;
	case STEP_FIND_UP_LIMIT:								//STATE_STP3
		//State action
		//No action
		//State transition
		if(is_lower_sw_pressed() || !sys_ok){
			issue_warning(WARN_STEP_FAULT3);
			stepCurrState = STEP_ERROR;						//T_STP7
		} else if(is_upper_sw_pressed()){
			stepCurrState = STEP_START_HYST_MOVE;			//T_STP5
		} else if(step_run_done){	//Did not hit switch
			issue_warning(WARN_STEP_UP_LIM_SW_FAIL);
			stepCurrState = STEP_WAIT_HOME;					//T_STP32
		} else {
			stepCurrState = STEP_FIND_UP_LIMIT;				//T_STP6
		}
		break;
	case STEP_START_HYST_MOVE:								//STATE_STP4
		//State action
		stop_pwm();
		step_setpoint = STEP_HYST_POS;
		step_position = 0;
		step_run_done = 0;
		stepper_enable(STEP_DIR_CCW);
		//TODO: check direction is down
		start_pwm();
		//State transition
		if(is_lower_sw_pressed() || !sys_ok){
			issue_warning(WARN_STEP_FAULT4);
			stepCurrState = STEP_ERROR;						//T_STP8
		} else {
			stepCurrState = STEP_FIND_UP_LIMIT_UP;			//T_STP9
		}
		break;
	case STEP_FIND_UP_LIMIT_UP:								//STATE_STP5
		//State action
		//No action
		//State transition
		if(is_lower_sw_pressed() || !sys_ok){
			issue_warning(WARN_STEP_FAULT5);
			stepCurrState = STEP_ERROR;						//T_STP10
		} else if(!is_upper_sw_pressed()){
			stepCurrState = STEP_START_HOME;				//T_STP12
		} else if(step_run_done){	//Switch did not release
			issue_warning(WARN_STEP_UP_LIM_SW_FAIL2);
			stepCurrState = STEP_WAIT_HOME;					//T_STP33
		} else {
			stepCurrState = STEP_FIND_UP_LIMIT_UP;			//T_STP11
		}
		break;
	case STEP_START_HOME:									//STATE_STP6
		//State action
		stop_pwm();
		step_setpoint = STEP_HOME_POS;
		step_position = 0;
		step_run_done = 0;
		stepper_enable(STEP_DIR_CCW);
		//TODO: check direction is down
		start_pwm();
		//State transition
		if(is_lower_sw_pressed() || !sys_ok){
			issue_warning(WARN_STEP_FAULT6);
			stepCurrState = STEP_ERROR;						//T_STP13
		} else {
			stepCurrState = STEP_FIND_HOME;					//T_STP14
		}
		break;
	case STEP_FIND_HOME:									//STATE_STP7
		//State action
		//No action
		//State transition
		if(is_lower_sw_pressed() || !sys_ok){
			issue_warning(WARN_STEP_FAULT7);
			stepCurrState = STEP_ERROR;						//T_STP15
		} else if(step_run_done){
			stepCurrState = STEP_STOP_STEPPER;				//T_STP17
		} else {
			stepCurrState = STEP_FIND_HOME;					//T_STP16
		}
		break;
	case STEP_STOP_STEPPER:									//STATE_STP9
		//State action
		stop_pwm();
		stepper_disable();
		//State transition
		if(is_lower_sw_pressed() || !sys_ok){
			issue_warning(WARN_STEP_FAULT8);
			stepCurrState = STEP_ERROR;						//T_STP18
		} else {
			stepCurrState = STEP_WAIT;						//T_STP19
		}
		break;
	case STEP_WAIT:											//STATE_STP10
		//State action
		//No action
		//State transition
		if(is_lower_sw_pressed()){
			issue_warning(WARN_STEP_FAULT9);
			stepCurrState = STEP_ERROR;						//T_STP20
		} else if(!sys_ok){
			stepCurrState = STEP_WAIT;
		} else if(step_home_request){
			step_home_request = 0;
			stepCurrState = STEP_START_FIND_UP_LIMIT;		//T_STP25
		} else if(step_request){
			step_request = 0;
			stepCurrState = STEP_START_MOVE;				//T_STP22
		} else {
			stepCurrState = STEP_WAIT;						//T_STP21
		}
		break;
	case STEP_START_MOVE:									//STATE_STP11
		//State action
		__disable_interrupt();
		step_setpoint = step_request_pos;
		step_run_done = 0;
		if(step_setpoint > step_position){	//Raise
			stepper_enable(STEP_DIR_CW);
			start_pwm();
		} else if(step_setpoint < step_position){	//Lower
			stepper_enable(STEP_DIR_CCW);
			start_pwm();
		} else {	//Already at position
			stepper_disable();
			stop_pwm();
		}
		//TODO: check directions
		//State transition
		if(is_lower_sw_pressed() || !sys_ok){
			issue_warning(WARN_STEP_FAULT10);
			stepCurrState = STEP_ERROR;						//T_STP23
		} else if(step_setpoint == step_position){
			stepCurrState = STEP_WAIT;						//T_STP31
		} else {
			stepCurrState = STEP_RUN_MOVE;					//T_STP24
		}
		__enable_interrupt();
		break;
	case STEP_RUN_MOVE:										//STATE_STP12
		//State action
		//No action
		//State transition
		if(is_lower_sw_pressed() || !sys_ok){
			issue_warning(WARN_STEP_FAULT11);
			stepCurrState = STEP_ERROR;						//T_STP27
		} else if(!sys_ok){
			stepCurrState = STEP_ERROR;						//T_STP27
		} else if(step_request && !step_run_done){	//New step request
			step_request = 0;
			stepCurrState = STEP_START_MOVE;				//T_STP26
		} else if(step_run_done){
			stepCurrState = STEP_STOP_STEPPER;				//T_STP29
		} else {
			stepCurrState = STEP_RUN_MOVE;					//T_STP28
		}
		break;
	case STEP_ERROR:										//STATE_STP8
		//State action
		stepper_disable();
		stop_pwm();
		//State transition
		if(!is_lower_sw_pressed() && sys_ok){
			//stepCurrState = STEP_WAIT_HOME;						//T_STP30
			stepCurrState = STEP_STOP_STEPPER;
		} else {
			stepCurrState = STEP_ERROR;						//T_STP34
		}
		break;
	default:
		stepper_disable();
		stop_pwm();
		issue_warning(WARN_ILLEGAL_STEP_STATE);
		break;
	}
	return;
}

/* Setup stepper motor driver */
void stepper_setup(void){
	//Assign stepper pins
	P1DIR |= STEP_PIN;		//Step output
	P1SEL |= STEP_PIN;
	P7DIR |= BIT1+BIT2;		//Enable and direction outputs
	P7OUT |= (BIT1+BIT2);	//Disable motor
	//Setup TA0.1 to pulse step pin
	//Up mode, Set/Reset 50% duty
	TA0CCR0 = STEP_TIME;
	TA0CCR1 = STEP_TIME>>2;						//50% duty
	TA0CTL = TASSEL_2 + MC_0 + TACLR + ID_3;	//SMCLK, off, clear TAR, div8
	TA0EX0 = TAIDEX_7;
	TA0CCTL0 = CCIE;						//CCR0 interrupt enabled
	TA0CCTL1 = OUTMOD_3;					//Set/reset
	return;
}

inline void start_pwm(void){
	TA0CTL |= MC_1+TACLR;	//Up mode
}

inline void stop_pwm(void){
	TA0CTL &= ~MC_3; //Off
	P1OUT &= ~BIT2;
}

inline uint8_t is_upper_sw_pressed(void){
	return (P2IN & BIT1);
}

inline uint8_t is_lower_sw_pressed(void){
	return (P2IN & BIT0);
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
		step_direction = STEP_DIR_CCW;
	} else {
		P7OUT &= ~BIT1;
		step_direction = STEP_DIR_CW;
	}
	//Enable
	P7OUT &= ~BIT2;
	return;
}

void stepper_disable(void){
	P7OUT |= BIT2;
	return;
}

