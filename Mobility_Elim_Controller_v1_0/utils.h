/*
 * utils.h
 * Created 6/10/15 by Nishant Pol
 * Robotics Capstone 16-474/Mechatronics 18-578
 *
 * Code Composer v6
 */
#ifndef UTILS_H_
#define UTILS_H_

#include <msp430.h>

typedef unsigned char uint8_t;
typedef char int8_t;
typedef unsigned int uint16_t;
typedef int int16_t;
typedef unsigned long uint32_t;
typedef long int32_t;
typedef unsigned long long uint64_t;
typedef long long int64_t;


uint8_t ascii2hex_byte(uint8_t high_char, uint8_t low_char);
void hex2ascii_byte(uint8_t data, uint8_t *high_char, uint8_t *low_char);
uint8_t P1_get(uint8_t *buf);
uint8_t P2_get(uint8_t *buf);
uint8_t P3_get(uint8_t *buf);
uint8_t P4_get(uint8_t *buf);
uint8_t P5_get(uint8_t *buf);
uint8_t P6_get(uint8_t *buf);
uint8_t P7_get(uint8_t *buf);
uint8_t P8_get(uint8_t *buf);
void led_P1_0_on(void);
void led_P1_0_off(void);


#endif /* UTILS_H_ */

