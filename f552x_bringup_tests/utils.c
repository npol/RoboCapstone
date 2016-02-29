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



