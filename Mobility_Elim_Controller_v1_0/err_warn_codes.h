/*
 * err_warn_codes.h
 *
 *  Created on: Mar 25, 2016
 *      Author: nishant
 */

#ifndef ERR_WARN_CODES_H_
#define ERR_WARN_CODES_H_

/* General Errors */
#define ERR_NONE				0x0000
#define ERR_TOO_MANY_WARNS		0x0001
#define ERR_XT2_FAULT			0x0002
#define ERR_XT1_FAULT			0x0003
#define ERR_DCO_FAULT			0x0004
#define ERR_FLASH_VIOL			0x0005
#define ERROR_TEST				0x0006

/* Application specific Errors */

/* General Warnings */
#define WARN_NONE				0x0000
#define WARN_DBG_BUFF_OVERRUN	0x0001
#define WARN_DBG_TX_BUF_FULL1	0x0002
#define WARN_DBG_TX_BUF_FULL2	0x0003
#define WARN_DBG_RX_BUF_FULL	0x0004
#define WARN_USCIA0_INT_ILLEGAL_FLAG	0x0005
#define WARN_USCIA1_INT_ILLEGAL_FLAG	0x0006
#define WARN_NMI				0x0007
#define WARN_RST_BOR			0x0010
#define WARN_RST_RSTNMI			0x0011
#define WARN_RST_DOBOR			0x0012
#define WARN_RST_LPM5WU			0x0013
#define WARN_RST_SECYV			0x0014
#define WARN_RST_SVSL			0x0015
#define WARN_RST_SVSH			0x0016
#define WARN_RST_SVMLOVP		0x0017
#define WARN_RST_SVMHOVP		0x0018
#define WARN_RST_DOPOR			0x0019
#define WARN_RST_WDTTO			0x001A
#define WARN_RST_WDTKEY			0x001B
#define WARN_RST_KEYV			0x001C
#define WARN_RSTFLLUL			0x001D
#define ERR_RST_PERF			0x001E
#define WARN_RST_PMM_KEY		0x001F
#define WARN_TEST				0x0020
#define WARN_ERROR_LOG_CLEARED 0x0021

/* Application specific Warnings */
#define WARN_RC_TX_BUF_FULL1	0x8000
#define WARN_RC_TX_BUF_FULL2	0x8001
#define WARN_RC_RX_BUF_FULL		0x8002
#define WARN_RC_SM_ASYNC_PCKT_FAIL	0x8100
#define WARN_RC_SM_ENC1_PCKT_FAIL 0x8101
#define WARN_RC_SM_ENC2_PCKT_FAIL 0x8102
#define WARN_RC_SM_ENC1_DATA_FAIL 0x8103
#define WARN_RC_SM_ENC2_DATA_FAIL 0x8104
#define WARN_RC_SM_MBATT_PCKT_FAIL 0x8105
#define WARN_RC_SM_LBATT_PCKT_FAIL 0x8106
#define WARN_RC_SM_MCUR_PCKT_FAIL	0x8107
#define WARN_RC_SM_TEMP_PCKT_FAIL 0x8108
#define WARN_RC_SM_STAT_PCKT_FAIL 0x8109
#define WARN_ILLEGAL_RC_SM_STATE	0x810A
#define WARN_RC_SM_MBATT_DATA_FAIL	0x810B
#define WARN_RC_SM_LBATT_DATA_FAIL 0x810C
#define WARN_RC_SM_MCUR_DATA_FAIL	0x810D
#define WARN_RC_SM_TEMP_DATA_FAIL	0x810E
#define WARN_RC_SM_STAT_DATA_FAIL	0x810F
#define WARN_ILLEGAL_MON_SM_STATE	0x8110
#define WARN_RC_FW_VER_DATA_FAIL	0x8111
#define WARN_RC_BAD_FW				0x8112
#define WARN_RC_RXBUF_NOT_EMPTY		0x8113
#define WARN_RC_RXBUF_NOT_EMPTY2	0x8114
#define WARN_LOW_3V3				0x8200
#define WARN_HIGH_3V3				0x8201
#define WARN_LOW_5V0				0x8202
#define WARN_HIGH_5V0				0x8203
#define WARN_LOW_12V				0x8204
#define WARN_HIGH_12V				0x8205
#define WARN_LOW_RC_MBATT			0x8206
#define WARN_HIGH_RC_MBATT			0x8207
#define WARN_LOW_RC_LBATT			0x8208
#define WARN_HIGH_RC_LBATT			0x8209
#define WARN_LOW_M1_CURRENT			0x820A
#define WARN_HIGH_M1_CURRENT		0x820B
#define WARN_LOW_M2_CURRENT			0x820C
#define WARN_HIGH_M2_CURRENT		0x820D
#define WARN_LOW_MCU_TEMP			0x820E
#define WARN_HIGH_MCU_TEMP			0x820F
#define WARN_LOW_RC_TEMP			0x8210
#define WARN_HIGH_RC_TEMP			0x8211
#define WARN_LOW_DRILL_CURRENT		0x8212
#define WARN_HIGH_DRILL_CURRENT		0x8213
#define ESTOP_ACTIVATED				0x8214
#define WARN_RC_M1_OVERCURRENT		0x8215
#define WARN_RC_M2_OVERCURRENT		0x8216
#define WARN_RC_ESTOP				0x8217
#define WARN_RC_TEMP_ERR			0x8218
#define WARN_RC_TEMP2_ERR			0x821A
#define WARN_RC_MBATT_H_ERR			0x821B
#define WARN_RC_LBATT_H_ERR			0x821C
#define WARN_RC_LBATT_L_ERR			0x821D
#define WARN_RC_M1_FAULT			0x821E
#define WARN_RC_M2_FAULT			0x821F
#define WARN_RC_MBATT_H_WARN		0x8220
#define WARN_RC_MBATT_L_WARN		0x8221
#define WARN_RC_TEMP_WARN			0x8222
#define WARN_RC_TEMP2_WARN			0x8223
#define WARN_ILLEGAL_DRILL_SM_STATE		0x8300
#define WARN_ILLEGAL_DRILL_REQUEST	0x8301
#define WARN_ILLEGAL_DRILL_REQUEST2 	0x8302
#define WARN_ILLEGAL_STEP_STATE		0x8400
#define WARN_STEP_UP_LIM_SW_FAIL		0x8401
#define WARN_STEP_UP_LIM_SW_FAIL2		0x8402
#define WARN_STEP_FAULT1			0x8403
#define WARN_STEP_FAULT2			0x8404
#define WARN_STEP_FAULT3			0x8405
#define WARN_STEP_FAULT4			0x8406
#define WARN_STEP_FAULT5			0x8407
#define WARN_STEP_FAULT6			0x8408
#define WARN_STEP_FAULT7			0x8409
#define WARN_STEP_FAULT8			0x840A
#define WARN_STEP_FAULT9			0x840B
#define WARN_STEP_FAULT10			0x840C
#define WARN_STEP_FAULT11			0x840D
#define WARN_CAN_MODE_SET_FAIL		0x850C
#define WARN_ILLEGAL_CAN_STATE		0x8500
#define WARN_CAN_RX1OVR				0x8501
#define WARN_CAN_RX0OVR				0x8502
#define WARN_CAN_TXBO				0x8503
#define WARN_CAN_TXEP				0x8504
#define WARN_CAN_RXEP				0x8505
#define WARN_CAN_TXWAR				0x8506
#define WARN_CAN_RXWAR				0x8507
#define WARN_CAN_EWARN				0x8508
#define WARN_CAN_TX_ABTF			0x8509
#define WARN_CAN_TX_TXERR			0x850A
#define WARN_CAN_TXREQ				0x850B
#define WARN_ILLEGAL_PC_STATE		0x8600
#define WARN_BAD_MOTOR_PACKET		0x8601

#endif /* ERR_WARN_CODES_H_ */
