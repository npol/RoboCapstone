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

