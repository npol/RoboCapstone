/* EncoderTest2: main.c
 * Nishant Pol 12/20/2015
 * CLEANUP Robotics Capstone
 * Quadrature encoder timing test, runs with emulated encoder stimuli
 * MSP430G2553 28pin
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
#include "clock.h"

uint8_t encoder1state = 0;
#define ENC1P1 0
#define ENC1P2 1
#define ENC1P4 2
#define ENC1P3 3
#define ERROR 4
uint8_t encoder2state = 0;
#define ENC2P1 (0<<2)
#define ENC2P2 (1<<2)
#define ENC2P4 (2<<2)
#define ENC2P3 (3<<2)
uint8_t encoder3state = 0;
uint8_t encoder4state = 0;

int32_t encoder1count = 0;
int32_t encoder2count = 0;
int32_t encoder3count = 0;
int32_t encoder4count = 0;

inline void encoder1task(void);
inline void encoder2task(void);
/*
 * main.c
 */
int main(void) {
    WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer
    setup_clock();//16MHz

    //Initialize Encoder SM
	encoder1state = P1IN & (BIT1|BIT0);
	encoder2state = P1IN & (BIT3|BIT2);
	encoder3state = P1IN & (BIT5|BIT4);
	encoder4state = P1IN & (BIT7|BIT6);

	//Initialize timing pin P2.0
	P2DIR = BIT0;
	P2OUT = 0;
	//Initialize output port
	P3DIR = 0xff;
	P3OUT = 0;
	
	while(1){
		P2OUT |= BIT0;
		encoder1task();
		P2OUT &= ~BIT0;
		encoder2task();
		encoder2task();
		encoder2task();
		P3OUT = encoder1count;
	}

}

inline void encoder1task(void){
	__disable_interrupt();
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
		} else if(new_encoder != ENC1P1){
			encoder1state = 0;	//Break here: error
		}
		break;
	case ENC1P2:	//P2 AB=01
		if(new_encoder == ENC1P3){
			encoder1count++;
			next_state = ENC1P3;
		} else if(new_encoder == ENC1P1){
			encoder1count--;
			next_state = ENC1P1;
		} else if(new_encoder != ENC1P2){
			encoder1state = 0;	//Break here: error
		}
		break;
	case ENC1P4:	//P4 AB=10
		if(new_encoder == ENC1P1){
			encoder1count++;
			next_state = ENC1P1;
		} else if(new_encoder == ENC1P3){
			encoder1count--;
			next_state = ENC1P3;
		} else if(new_encoder != ENC1P4){
			encoder1state = 0;	//Break here: error
		}
		break;
	case ENC1P3:	//P3 AB=11
		if(new_encoder == ENC1P4){
			encoder1count++;
			next_state = ENC1P4;
		} else if(new_encoder == ENC1P2){
			encoder1count--;
			next_state = ENC1P2;
		} else if(new_encoder != ENC1P3){
			encoder1state = 0;	//Break here: error
		}
		break;
	}
	encoder1state = next_state;
	__enable_interrupt();
	return;
}

inline void encoder2task(void){
	__disable_interrupt();
	uint8_t next_state = encoder1state;
	uint8_t new_encoder = P1IN & (BIT3|BIT2);
	switch(encoder2state){
	case ENC2P1:	//P1 AB=00
		if(new_encoder == ENC2P2){
			encoder2count++;
			next_state = ENC2P2;
		} else if(new_encoder == ENC2P4){
			encoder2count--;
			next_state = ENC2P4;
		} else if(new_encoder != ENC2P1){
			encoder2state = 0;	//Break here: error
		}
		break;
	case ENC2P2:	//P2 AB=01
		if(new_encoder == ENC2P3){
			encoder2count++;
			next_state = ENC2P3;
		} else if(new_encoder == ENC2P1){
			encoder2count--;
			next_state = ENC2P1;
		} else if(new_encoder != ENC2P2){
			encoder2state = 0;	//Break here: error
		}
		break;
	case ENC2P4:	//P4 AB=10
		if(new_encoder == ENC2P1){
			encoder2count++;
			next_state = ENC2P1;
		} else if(new_encoder == ENC2P3){
			encoder2count--;
			next_state = ENC2P3;
		} else if(new_encoder != ENC2P4){
			encoder2state = 0;	//Break here: error
		}
		break;
	case ENC2P3:	//P3 AB=11
		if(new_encoder == ENC2P4){
			encoder2count++;
			next_state = ENC2P4;
		} else if(new_encoder == ENC2P2){
			encoder2count--;
			next_state = ENC2P2;
		} else if(new_encoder != ENC2P3){
			encoder2state = 0;	//Break here: error
		}
		break;
	}
	encoder2state = next_state;
	__enable_interrupt();
	return;
}
