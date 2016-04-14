/*
 * roboclaw.c
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

#include "roboclaw.h"

uint8_t rc_fw_ver[27] = "USB Roboclaw 2x60a v4.1.13";

void roboclaw_setup(void){
	//Setup TA2 to fire at intervals for triggering encoders and check routines
	TA2CCTL0 = CCIE;                          // CCR0 interrupt enabled
	TA2CCR0 = 10000;
	TA2CTL = TASSEL_2 + MC_1 + TACLR + ID_3;         // SMCLK, upmode, clear TAR, div8
	TA2EX0 = TAIDEX_7;							//div8
	return;
}

/* Generate packet for driving both motors: send motor power
 * m1Pwr: Signed 8-bit motor power value, 0 to 127 is forward, -128 to -1 is reverse
 * buf: character buffer of at least five bytes for packet
 */
uint8_t driveM1Power(int8_t m1Pwr, uint8_t *buf){
	buf[0] = RC_ADDR;
	if(m1Pwr >= 0){
		buf[1] = RC_CMD_DRIVE_FW_M1;
		buf[2] = (uint8_t)m1Pwr;
	} else {
		buf[1] = RC_CMD_DRIVE_BW_M1;
		if(m1Pwr == -128){	//Edge condition since -(-128) == -128
			m1Pwr = -127;
		}
		buf[2] = (uint8_t)(-m1Pwr);
	}
	uint16_t packet_crc = crc16(buf,3);
	buf[3] = (uint8_t)(packet_crc>>8);
	buf[4] = (uint8_t)packet_crc;
	return 5;	//Packet size
}

/* Generate packet for driving both motors: send motor power
 * m1Pwr: Signed 8-bit motor power value, 0 to 127 is forward, -128 to -1 is reverse
 * buf: character buffer of at least five bytes for packet
 */
uint8_t driveM2Power(int8_t m2Pwr, uint8_t *buf){
	buf[0] = RC_ADDR;
	if(m2Pwr >= 0){
		buf[1] = RC_CMD_DRIVE_FW_M2;
		buf[2] = (uint8_t)m2Pwr;
	} else {
		buf[1] = RC_CMD_DRIVE_BW_M2;
		if(m2Pwr == -128){	//Edge condition since -(-128) == -128
			m2Pwr = -127;
		}
		buf[2] = (uint8_t)(-m2Pwr);
	}
	uint16_t packet_crc = crc16(buf,3);
	buf[3] = (uint8_t)(packet_crc>>8);
	buf[4] = (uint8_t)packet_crc;
	return 5;	//Packet size
}

/* Get fw version string
 * buf: character of at least 48 bytes
 */
uint8_t getRCFwVer(uint8_t *buf){
	buf[0] = RC_ADDR;
	buf[1] = RC_READ_FW_VERSION;
	return 2;
}

/* Check recieved fw version string matches
 * buf: character buffer with recieved string
 */
uint8_t checkRCFwVer(uint8_t *buf){
	if(strncmp(buf,rc_fw_ver,26)==0)
		return 1;
	return 0;
}

/* Check main voltage level
 * buf: character buffer, at least 2 bytes
 */
uint8_t checkRCMainBatt(uint8_t *buf){
	buf[0] = RC_ADDR;
	buf[1] = RC_READ_MAIN_BATT_VOLT;
	return 2;
}

/* Check logic voltage level
 * buf: character buffer, at least 2 bytes
 */
uint8_t checkRCLogicBatt(uint8_t *buf){
	buf[0] = RC_ADDR;
	buf[1] = RC_READ_LOGIC_BATT_VOLT;
	return 2;
}

/* Read motor PWM values
 * buf: character buffer, at least 2 bytes
 */
uint8_t checkRCPWM(uint8_t *buf){
	buf[0] = RC_ADDR;
	buf[1] = RC_READ_MOTOR_PWM_VALUES;
	return 2;
}

/* Read motor currents
 * buf: character buffer, at least 2 bytes
 */
uint8_t checkRCcurrents(uint8_t *buf){
	buf[0] = RC_ADDR;
	buf[1] = RC_READ_MOTOR_CURRENTS;
	return 2;
}

/* Read board temperature
 * buf: character buffer, at least 2 bytes
 */
uint8_t checkRCtemp(uint8_t *buf){
	buf[0] = RC_ADDR;
	buf[1] = RC_READ_TEMP;
	return 2;
}

/* Read roboclaw status
 * buf: character buffer, at least 2 bytes
 *
 */
uint8_t checkRCstatus(uint8_t *buf){
	buf[0] = RC_ADDR;
	buf[1] = RC_GET_STATUS;
	return 2;
}

/* Enable ESTOP on S3
 * buf: character buffer, at least 7 bytes
 */
uint8_t RCenableESTOP(uint8_t *buf){
	buf[0] = RC_ADDR;
	buf[1] = RC_SET_PIN_MODES;
	buf[2] = 2;	//ESTOP (non-latching)
	buf[3] = 0;	//Disabled
	buf[4] = 0;	//Disabled
	uint16_t packet_crc = crc16(buf,5);
	buf[5] = (uint8_t)(packet_crc>>8);
	buf[6] = (uint8_t)packet_crc;
	return 7;
}

/* Enable ESTOP on S3
 * buf: character buffer, at least 7 bytes
 */
uint8_t RCdisableESTOP(uint8_t *buf){
	buf[0] = RC_ADDR;
	buf[1] = RC_SET_PIN_MODES;
	buf[2] = 0;	//Disabled
	buf[3] = 0;	//Disabled
	buf[4] = 0;	//Disabled
	uint16_t packet_crc = crc16(buf,5);
	buf[5] = (uint8_t)(packet_crc>>8);
	buf[6] = (uint8_t)packet_crc;
	return 7;
}

/* Get Encoder 1 count
 * buf: character buffer, at least 2 bytes
 */
uint8_t RCgetEnc1Count(uint8_t *buf){
	buf[0] = RC_ADDR;
	buf[1] = RC_READ_M1_ENC_COUNT;
	return 2;
}

/* Get Encoder 2 count
 * buf: character buffer, at least 2 bytes
 */
uint8_t RCgetEnc2Count(uint8_t *buf){
	buf[0] = RC_ADDR;
	buf[1] = RC_READ_M2_ENC_COUNT;
	return 2;
}

/* Set Motor 1 PID Constants
 * pid_constants: array of 4-byte numbers:
 * 0: D-value
 * 1: P-value
 * 2: I-value
 * 3: QPPS
 */
uint8_t setM1PID(uint32_t *pid_constants,uint8_t *buf){
	buf[0] = RC_ADDR;
	buf[1] = RC_PID1;
	buf[2] = (uint8_t)((pid_constants[0]>>24)&0xFF);	//D msbyte
	buf[3] = (uint8_t)((pid_constants[0]>>16)&0xFF);
	buf[4] = (uint8_t)((pid_constants[0]>>8)&0xFF);
	buf[5] = (uint8_t)((pid_constants[0])&0xFF);		//D lsbyte
	buf[6] = (uint8_t)((pid_constants[1]>>24)&0xFF);	//P msbyte
	buf[7] = (uint8_t)((pid_constants[1]>>16)&0xFF);
	buf[8] = (uint8_t)((pid_constants[1]>>8)&0xFF);
	buf[9] = (uint8_t)((pid_constants[1])&0xFF);		//P lsbyte
	buf[10] = (uint8_t)((pid_constants[2]>>24)&0xFF);	//I msbyte
	buf[11] = (uint8_t)((pid_constants[2]>>16)&0xFF);
	buf[12] = (uint8_t)((pid_constants[2]>>8)&0xFF);
	buf[13] = (uint8_t)((pid_constants[2])&0xFF);		//I lsbyte
	buf[14] = (uint8_t)((pid_constants[3]>>24)&0xFF);	//QPPS msbyte
	buf[15] = (uint8_t)((pid_constants[3]>>16)&0xFF);
	buf[16] = (uint8_t)((pid_constants[3]>>8)&0xFF);
	buf[17] = (uint8_t)((pid_constants[3])&0xFF);		//QPPS lsbyte
	uint16_t packet_crc = crc16(buf,18);
	buf[18] = (uint8_t)(packet_crc>>8);
	buf[19] = (uint8_t)packet_crc;
	return 20;
}

/* Set Motor 2 PID Constants
 * pid_constants: array of 4-byte numbers:
 * 0: D-value
 * 1: P-value
 * 2: I-value
 * 3: QPPS
 */
uint8_t setM2PID(uint32_t *pid_constants,uint8_t *buf){
	buf[0] = RC_ADDR;
	buf[1] = RC_PID2;
	buf[2] = (uint8_t)((pid_constants[0]>>24)&0xFF);	//D msbyte
	buf[3] = (uint8_t)((pid_constants[0]>>16)&0xFF);
	buf[4] = (uint8_t)((pid_constants[0]>>8)&0xFF);
	buf[5] = (uint8_t)((pid_constants[0])&0xFF);		//D lsbyte
	buf[6] = (uint8_t)((pid_constants[1]>>24)&0xFF);	//P msbyte
	buf[7] = (uint8_t)((pid_constants[1]>>16)&0xFF);
	buf[8] = (uint8_t)((pid_constants[1]>>8)&0xFF);
	buf[9] = (uint8_t)((pid_constants[1])&0xFF);		//P lsbyte
	buf[10] = (uint8_t)((pid_constants[2]>>24)&0xFF);	//I msbyte
	buf[11] = (uint8_t)((pid_constants[2]>>16)&0xFF);
	buf[12] = (uint8_t)((pid_constants[2]>>8)&0xFF);
	buf[13] = (uint8_t)((pid_constants[2])&0xFF);		//I lsbyte
	buf[14] = (uint8_t)((pid_constants[3]>>24)&0xFF);	//QPPS msbyte
	buf[15] = (uint8_t)((pid_constants[3]>>16)&0xFF);
	buf[16] = (uint8_t)((pid_constants[3]>>8)&0xFF);
	buf[17] = (uint8_t)((pid_constants[3])&0xFF);		//QPPS lsbyte
	uint16_t packet_crc = crc16(buf,18);
	buf[18] = (uint8_t)(packet_crc>>8);
	buf[19] = (uint8_t)packet_crc;
	return 20;
}

/* Read Motor 1 encoder speed in quadrature pulses per second
 * buf: character buffer, at least 2 bytes
 */
uint8_t RCgetEnc1Speed(uint8_t *buf){
	buf[0] = RC_ADDR;
	buf[1] = RC_READ_M1_ENC_SPEED;
	return 2;
}

/* Read Motor 2 encoder speed in quadrature pulses per second
 * buf: character buffer, at least 2 bytes
 */
uint8_t RCgetEnc2Speed(uint8_t *buf){
	buf[0] = RC_ADDR;
	buf[1] = RC_READ_M2_ENC_SPEED;
	return 2;
}

/* Set Motor 1 Signed speed
 * m1Spd: 4-byte speed value in quadrature pulses per second)
 * buf: character buffer with at least 8 bytes
 */
uint8_t driveM1Speed(uint32_t m1Spd, uint8_t *buf){
	buf[0] = RC_ADDR;
	buf[1] = RC_SET_M1_SPEED;
	buf[2] = (uint8_t)((m1Spd>>24)&0xFF);
	buf[3] = (uint8_t)((m1Spd>>16)&0xFF);
	buf[4] = (uint8_t)((m1Spd>>8)&0xFF);
	buf[5] = (uint8_t)(m1Spd&0xFF);
	uint16_t packet_crc = crc16(buf,6);
	buf[6] = (uint8_t)(packet_crc>>8);
	buf[7] = (uint8_t)packet_crc;
	return 8;	//Packet size
}

/* Set Motor 2 Signed speed
 * m2Spd: 4-byte speed value in quadrature pulses per second)
 * buf: character buffer with at least 8 bytes
 */
uint8_t driveM2Speed(uint32_t m2Spd, uint8_t *buf){
	buf[0] = RC_ADDR;
	buf[1] = RC_SET_M2_SPEED;
	buf[2] = (uint8_t)((m2Spd>>24)&0xFF);
	buf[3] = (uint8_t)((m2Spd>>16)&0xFF);
	buf[4] = (uint8_t)((m2Spd>>8)&0xFF);
	buf[5] = (uint8_t)(m2Spd&0xFF);
	uint16_t packet_crc = crc16(buf,6);
	buf[6] = (uint8_t)(packet_crc>>8);
	buf[7] = (uint8_t)packet_crc;
	return 8;	//Packet size
}

/* Set Signed speed for both motors
 * m1Spd: 4-byte speed value for motor 1 in quadrature pulses per second)
 * m2Spd: 4-byte speed value for motor 2 in quadrature pulses per second
 */
uint8_t driveM12Speed(uint32_t m1Spd, uint32_t m2Spd, uint8_t *buf){
	buf[0] = RC_ADDR;
	buf[1] = RC_SET_M12_SPEED;
	buf[2] = (uint8_t)((m1Spd>>24)&0xFF);
	buf[3] = (uint8_t)((m1Spd>>16)&0xFF);
	buf[4] = (uint8_t)((m1Spd>>8)&0xFF);
	buf[5] = (uint8_t)(m1Spd&0xFF);
	buf[6] = (uint8_t)((m2Spd>>24)&0xFF);
	buf[7] = (uint8_t)((m2Spd>>16)&0xFF);
	buf[8] = (uint8_t)((m2Spd>>8)&0xFF);
	buf[9] = (uint8_t)(m2Spd&0xFF);
	uint16_t packet_crc = crc16(buf,10);
	buf[10] = (uint8_t)(packet_crc>>8);
	buf[11] = (uint8_t)packet_crc;
	return 12;	//Packet size
}

/* Datasheet provided function to calculate CRC
 * For transmitted packets: CRC of address and command except CRC
 * For recieved packets: CRC of all bytes recieved (except CRC), and include sent address and command byte
 *
 */
uint16_t crc16(uint8_t *packet, uint8_t nBytes){
	uint8_t byte;
	uint8_t bit;
	uint16_t crc = 0;
	for (byte = 0; byte < nBytes; byte++) {
		crc = crc ^ ((unsigned int)packet[byte] << 8);
		for (bit = 0; bit < 8; bit++) {
			if (crc & 0x8000) {
				crc = (crc << 1) ^ 0x1021;
			} else {
				crc = crc << 1;
			}
		}
	}
	return crc;
}

