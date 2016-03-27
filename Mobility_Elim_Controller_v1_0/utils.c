/*
 * utils.c
 *
 *  Created on: Feb 29, 2016
 *      Author: nishant
 */

#include "utils.h"


/* Create numerical byte from hex ascii characters
 * high_char: ascii code for high nibble
 * low_char: ascii code for low nibble
 * returns numerical byte value
 */
uint8_t ascii2hex_byte(uint8_t high_char, uint8_t low_char){
	uint8_t num = 0;
	if(('0'<= high_char) && (high_char <= '9')){
		num = (high_char-'0')<<4;
	} else if(('A' <= high_char) && (high_char <= 'F')){
		num = (high_char-'A'+10)<<4;
	} else if(('a' <= high_char) && (high_char <= 'f')){
		num = (high_char-'a'+10)<<4;
	}
	if(('0'<= low_char) && (low_char <= '9')){
		num |= (low_char-'0');
	} else if(('A' <= low_char) && (low_char <= 'F')){
		num |= (low_char-'A'+10);
	} else if(('a' <= low_char) && (low_char <= 'f')){
		num |= (low_char-'a'+10);
	}
	return num;
}

/* Create ascii character representation of numerical byte
 * data: number to be converted
 * high_char: pointer to high nibble character
 * low_char: pointer to low nibble character
 */
void hex2ascii_byte(uint8_t data, uint8_t *high_char, uint8_t *low_char){
	uint8_t upper = (data>>4)&0xf;
	uint8_t lower = (data)&0xf;
	if(upper <= 9){
		*high_char = upper+'0';
	} else if((10 <= upper) && (upper <= 15)){
		*high_char = upper-10+'A';
	}
	if(lower <= 9){
		*low_char = lower+'0';
	} else if((10 <= lower) && (lower <= 15)){
		*low_char = lower-10+'A';
	}
	return;
}

/* Create ascii character representation of numerical 16-bit int
 * data: number to be converted
 * char4: pointer to highest nibble character
 * char3: pointer to mid-high nibble character
 * char2: pointer to mid-low nibble character
 * char1: pointer to low nibble character
 */
void hex2ascii_int(uint16_t data, uint8_t *char4, uint8_t *char3, uint8_t *char2, uint8_t *char1){
	uint8_t upper = (data>>12)&0xf;
	uint8_t up_mid = (data>>8)&0xf;
	uint8_t low_mid = (data>>4)&0xf;
	uint8_t lower = (data)&0xf;
	if(upper <= 9){
		*char4 = upper+'0';
	} else if((10 <= upper) && (upper <= 15)){
		*char4 = upper-10+'A';
	}
	if(up_mid <= 9){
		*char3 = up_mid+'0';
	} else if((10 <= up_mid) && (up_mid <= 15)){
		*char3 = up_mid-10+'A';
	}
	if(low_mid <= 9){
		*char2 = low_mid+'0';
	} else if((10 <= low_mid) && (low_mid <= 15)){
		*char2 = low_mid-10+'A';
	}
	if(lower <= 9){
		*char1 = lower+'0';
	} else if((10 <= lower) && (lower <= 15)){
		*char1 = lower-10+'A';
	}
	return;
}

/* Get pin states of port 1
 * buf: character buffer of at least 4 bytes
 * returns 4, size of buffer
 */
uint8_t P1_get(uint8_t *buf){
	buf[0] = '0';
	buf[1] = 'x';
	hex2ascii_byte(P1IN,&buf[2],&buf[3]);
	return 4;
}

/* Get pin states of port 2
 * buf: character buffer of at least 4 bytes
 * returns 4, size of buffer
 */
uint8_t P2_get(uint8_t *buf){
	buf[0] = '0';
	buf[1] = 'x';
	hex2ascii_byte(P2IN,&buf[2],&buf[3]);
	return 4;
}

/* Get pin states of port 3
 * buf: character buffer of at least 4 bytes
 * returns 4, size of buffer
 */
uint8_t P3_get(uint8_t *buf){
	buf[0] = '0';
	buf[1] = 'x';
	hex2ascii_byte(P3IN,&buf[2],&buf[3]);
	return 4;
}

/* Get pin states of port 4
 * buf: character buffer of at least 4 bytes
 * returns 4, size of buffer
 */
uint8_t P4_get(uint8_t *buf){
	buf[0] = '0';
	buf[1] = 'x';
	hex2ascii_byte(P4IN,&buf[2],&buf[3]);
	return 4;
}

/* Get pin states of port 5
 * buf: character buffer of at least 4 bytes
 * returns 4, size of buffer
 */
uint8_t P5_get(uint8_t *buf){
	buf[0] = '0';
	buf[1] = 'x';
	hex2ascii_byte(P5IN,&buf[2],&buf[3]);
	return 4;
}

/* Get pin states of port 6
 * buf: character buffer of at least 4 bytes
 * returns 4, size of buffer
 */
uint8_t P6_get(uint8_t *buf){
	buf[0] = '0';
	buf[1] = 'x';
	hex2ascii_byte(P6IN,&buf[2],&buf[3]);
	return 4;
}

/* Get pin states of port 7
 * buf: character buffer of at least 4 bytes
 * returns 4, size of buffer
 */
uint8_t P7_get(uint8_t *buf){
	buf[0] = '0';
	buf[1] = 'x';
	hex2ascii_byte(P7IN,&buf[2],&buf[3]);
	return 4;
}

/* Get pin states of port 8
 * buf: character buffer of at least 4 bytes
 * returns 4, size of buffer
 * Note: only bits 2:0 are valid
 */
uint8_t P8_get(uint8_t *buf){
	buf[0] = '0';
	buf[1] = 'x';
	hex2ascii_byte(P8IN,&buf[2],&buf[3]);
	return 4;
}

/* Turn on debug LED on P1.0 */
void led_P1_0_on(void){
	P1OUT |= BIT0;
	return;
}

/* Turn off debug LED on P1.0 */
void led_P1_0_off(void){
	P1OUT &= ~BIT0;
	return;
}

/* Turn on debug LED on P2.2 */
void led_P2_2_on(void){
	P2OUT |= BIT2;
	return;
}

/* Turn off debug LED on P2.2 */
void led_P2_2_off(void){
	P2OUT &= ~BIT2;
	return;
}

/* Turn on debug LED on P7.7 */
void led_P7_7_on(void){
	P7OUT |= BIT7;
	return;
}

/* Turn off debug LED on P7.7 */
void led_P7_7_off(void){
	P7OUT &= ~BIT7;
	return;
}

/* Turn on ok LED on P5.7 */
void led_P5_7_on(void){
	P5OUT |= BIT7;
	return;
}

/* Turn off ok LED on P5.7 */
void led_P5_7_off(void){
	P5OUT &= ~BIT7;
	return;
}

/* Turn on warn LED on P5.6 */
void led_P5_6_on(void){
	P5OUT |= BIT6;
	return;
}

/* Turn off warn LED on P5.6 */
void led_P5_6_off(void){
	P5OUT &= ~BIT6;
	return;
}

/* Turn on error LED on P4.7 */
void led_P4_7_on(void){
	P4OUT |= BIT7;
	return;
}

/* Turn off error LED on P4.7 */
void led_P4_7_off(void){
	P4OUT &= ~BIT7;
	return;
}

/* Log warning code */
void issue_warning(uint16_t warn_code){
	led_P5_6_on();
	led_P5_7_off();
	warn_flag = 1;
	warn_log[warn_log_ptr] = warn_code;
	warn_log_ptr++;
	if(warn_log_ptr >= WARN_LOG_SIZE){
		issue_error(ERR_TOO_MANY_WARNS);
		warn_log_ptr = 0;
	}
}

/* Log error code */
void issue_error(uint16_t err_code){
	 led_P4_7_on();
	 led_P5_7_off();
	 err_flag = 1;
	 err_log[err_log_ptr] = err_code;
	 if(err_log_ptr >= ERR_LOG_SIZE){
		 err_log_ptr = 0;
	 }
}

/* Function for external checking of error status */
uint8_t is_error(void){
	return err_flag;
}

/* Function for external checking of warning status */
uint8_t is_warning(void){
	return warn_flag;
}

/* Get dump of error codes
 * buf: character buffer of at least 60 bytes
 * returns 60, size of buffer
 */
uint8_t error_dump(uint8_t *buf){
	uint8_t i;
	buf[0] = '#';
	buf[1] = '0';
	buf[2] = 'x';
	hex2ascii_byte(err_log_ptr,&buf[3],&buf[4]);
	buf[5] = 13;	//CR+LF
	buf[6] = 10;
	for(i = 0; i < ERR_LOG_SIZE; i++){
		hex2ascii_int(err_log[i],&buf[(6*i+7)],&buf[(6*i+7)+1],&buf[(6*i+7)+2],&buf[(6*i+7)+3]);
		buf[(6*i+7)+4] = 13;	//CR+LF
		buf[(6*i+7)+5] = 10;
	}
	return (6*i+7);
}

/* Get dump of warning codes
 * buf: character buffer of at least 60 bytes
 * returns 60, size of buffer
 */
uint8_t warning_dump(uint8_t *buf){
	uint8_t i;
	buf[0] = '#';
	buf[1] = '0';
	buf[2] = 'x';
	hex2ascii_byte(warn_log_ptr,&buf[3],&buf[4]);
	buf[5] = 13;	//CR+LF
	buf[6] = 10;
	for(i = 0; i < WARN_LOG_SIZE; i++){
		hex2ascii_int(warn_log[i],&buf[(6*i+7)],&buf[(6*i+7)+1],&buf[(6*i+7)+2],&buf[(6*i+7)+3]);
		buf[(6*i+7)+4] = 13;	//CR+LF
		buf[(6*i+7)+5] = 10;
	}
	return (6*i+7);
}
