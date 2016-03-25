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
#include "err_warn_codes.h"

typedef unsigned char uint8_t;
typedef signed char int8_t;
typedef unsigned int uint16_t;
typedef signed int int16_t;
typedef unsigned long uint32_t;
typedef signed long int32_t;
typedef unsigned long long uint64_t;
typedef signed long long int64_t;

/* Warning/Error code buffers and flags */
#define WARN_LOG_SIZE 64
extern volatile uint16_t warn_log[WARN_LOG_SIZE];
extern volatile uint8_t warn_log_ptr;
extern volatile uint8_t warn_flag;
#define ERR_LOG_SIZE 64
extern volatile uint16_t err_log[ERR_LOG_SIZE];
extern volatile uint8_t err_log_ptr;
extern volatile uint8_t err_flag;


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
void led_P2_2_on(void);
void led_P2_2_off(void);
void led_P7_7_on(void);
void led_P7_7_off(void);
void led_P5_7_on(void);
void led_P5_7_off(void);
void led_P5_6_on(void);
void led_P5_6_off(void);
void led_P4_7_on(void);
void led_P4_7_off(void);
void issue_warning(uint16_t warn_code);
void issue_error(uint16_t err_code);
uint8_t is_error(void);
uint8_t is_warning(void);

#endif /* UTILS_H_ */

