/* EncoderTest: main.c
 * Nishant Pol 12/20/2015
 * CLEANUP Robotics Capstone
 * Quadrature encoder timing test, runs with emulated encoder stimuli
 * MSP430G2553 20pin
 *
 *
 * Pinout
 * P1.0 ENC1A
 * P1.1 ENC1B
 * P1.2 ENC2A
 * P1.3 ENC2B
 * P1.4 ENC3A
 * P1.5 ENC3B
 * P1.6 ENC4A
 * P1.7 ENC4B
 * P2.0 Timing indicator
 */
#include <msp430.h> 
#include "utils.h"

uint8_t encoder1state = 0;
#define ENC1P1 0
#define ENC1P2 1
#define ENC1P4 2
#define ENC1P3 3
#define ERROR 4
uint8_t encoder2state = 0;
uint8_t encoder3state = 0;
uint8_t encoder4state = 0;

int32_t encoder1count = 0;
int32_t encoder2count = 0;
int32_t encoder3count = 0;
int32_t encoder4count = 0;

inline void encoder1task(void);
/*
 * main.c
 */
int main(void) {
    WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer

    //Initialize Encoder SM
	encoder1state = P1IN & (BIT1|BIT0);
	encoder2state = P1IN & (BIT3|BIT2);
	encoder3state = P1IN & (BIT5|BIT4);
	encoder4state = P1IN & (BIT7|BIT6);

	//Initialize timing pin P2.0
	P2DIR = BIT0;
	
	while(1){
		encoder1task();
	}

}

inline void encoder1task(void){
	__disable_interrupt();
	P2OUT ^= BIT0;
	uint8_t next_state = encoder1state;
	uint8_t new_encoder = P1IN & (BIT1|BIT0);
	switch(encoder1state){
	case ENC1P1:	//P1 AB=00
		if(new_encoder == ENC1P2){
			encoder1count++;
			next_state = ENC1P2;
		} else if(new_encoder == ENC1P4){
			encoder1count--;
			next_state = ENC1P4;
		}
		break;
	case ENC1P2:	//P2 AB=01
		if(new_encoder == ENC1P3){
			encoder1count++;
			next_state = ENC1P3;
		} else if(new_encoder == ENC1P1){
			encoder1count--;
			next_state = ENC1P1;
		}
		break;
	case ENC1P4:	//P4 AB=10
		if(new_encoder == ENC1P1){
			encoder1count++;
			next_state = ENC1P1;
		} else if(new_encoder == ENC1P3){
			encoder1count--;
			next_state = ENC1P3;
		}
		break;
	case ENC1P3:	//P3 AB=11
		if(new_encoder == ENC1P4){
			encoder1count++;
			next_state = ENC1P4;
		} else if(new_encoder == ENC1P2){
			encoder1count--;
			next_state = ENC1P2;
		}
		break;
	}
	encoder1state = next_state;
	__enable_interrupt();
	return;
}
