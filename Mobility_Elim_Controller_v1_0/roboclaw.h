/*
 * roboclaw.h
 * Created 3/17/16 by Nishant Pol
 * Robotics Capstone 16-474/Mechatronics 18-578
 * MSP430F5521
 * Mobility/Elimination Controller Code
 * Roboclaw control functions (packet generation)
 *
 * Code Composer v6
 * Version History
 * 3/17/16: Ported from f552x_bringup_tests
 */

#ifndef ROBOCLAW_H_
#define ROBOCLAW_H_

#include <msp430.h>
#include "utils.h"
#include "string.h"

#define RC_ADDR 0x80

#define RC_CMD_DRIVE_FW_M1 0
#define RC_CMD_DRIVE_BW_M1 1
#define RC_CMD_DRIVE_FW_M2 4
#define RC_CMD_DRIVE_BW_M2 5
#define RC_READ_FW_VERSION 21
#define RC_READ_MAIN_BATT_VOLT 24
#define RC_READ_LOGIC_BATT_VOLT 25
#define RC_READ_MOTOR_PWM_VALUES 48
#define RC_READ_MOTOR_CURRENTS 49
#define RC_SET_PIN_MODES 74
#define RC_GET_TEMP 82
#define RC_GET_STATUS 90
#define RC_READ_M1_ENC_COUNT 16
#define RC_READ_M2_ENC_COUNT 17
#define RC_READ_TEMP 82
#define RC_GET_STATUS 90
#define RC_GET_ENC_MODE 91

#define RC_STAT_NORM			0x0000
#define RC_STAT_M1_OVERCURRENT	0x0001
#define RC_STAT_M2_OVERCURRENT	0x0002
#define RC_STAT_ESTOP			0x0004
#define RC_STAT_TEMP_ERR		0x0008
#define RC_STAT_TEMP2_ERR		0x0010
#define RC_STAT_MBATT_H_ERR		0x0020
#define RC_STAT_LBATT_H_ERR		0x0040
#define RC_STAT_LBATT_L_ERR		0x0080
#define RC_STAT_M1_FAULT		0x0100
#define RC_STAT_M2_FAULT		0x0200
#define RC_STAT_MBATT_H_WARN	0x0400
#define RC_STAT_MBATT_L_WARN	0x0800
#define RC_STAT_TEMP_WARN		0x1000
#define RC_STAT_TEMP2_WARN		0x2000
#define RC_STAT_M1_HOME			0x4000
#define RC_STAT_M2_HOME			0x8000

void roboclaw_setup(void);
uint8_t driveM1Power(int8_t m1Pwr, uint8_t *buf);
uint8_t driveM2Power(int8_t m2Pwr, uint8_t *buf);
uint8_t getRCFwVer(uint8_t *buf);
uint8_t checkRFFwVer(uint8_t *buf);
uint8_t checkRCMainBatt(uint8_t *buf);
uint8_t checkRCLogicBatt(uint8_t *buf);
uint8_t checkRCPWM(uint8_t *buf);
uint8_t checkRCcurrents(uint8_t *buf);
uint8_t checkRCtemp(uint8_t *buf);
uint8_t checkRCstatus(uint8_t *buf);
uint8_t RCenableESTOP(uint8_t *buf);
uint8_t RCdisableESTOP(uint8_t *buf);
uint8_t RCgetEnc1Count(uint8_t *buf);
uint8_t RCgetEnc2Count(uint8_t *buf);

uint16_t crc16(uint8_t *packet, uint8_t nBytes);



#endif /* ROBOCLAW_H_ */
