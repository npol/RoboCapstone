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
#define WARN_ILLEGAL_MON_SM_STATE	0x8110
#define WARN_LOW_3V3				0x8200
#define WARN_HIGH_3V3				0x8201
#define WARN_LOW_5V0				0x8202
#define WARN_HIGH_5V0				0x8203
#define WARN_LOW_12V				0x8204
#define WARN_HIGH_12V				0x8205
#define WARN_LOW_MCU_TEMP			0x820E
#define WARN_HIGH_MCU_TEMP			0x820F
#define WARN_USCIB0_INT_ILLEGAL_FLAG 0x8300
#define WARN_CAN_MODE_SET_FAIL		0x8400
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
#define WARN_CAN_SPI_OVERRUN		0x850C


#endif /* ERR_WARN_CODES_H_ */
