/*
 * MCP2515.h
 *
 *  Created on: Nov 25, 2015
 *      Author: nishant
 */
#include "utils.h"
#include "can_spi_uscib0.h"

#ifndef MCP2515_H_
#define MCP2515_H_

uint8_t mcp2515_read_register(uint8_t reg_addr);
void mcp2515_write_register(uint8_t reg_addr, uint8_t data);
void setup_mcp2515(void);
void mcp2562_shutdown_enable(void);
void mcp2562_shutdown_disable(void);
void mcp2515_reset(void);
void mcp2515_read_mult_registers_nonblock_init(uint8_t start_reg_addr, uint8_t num_regs);
uint8_t mcp2515_read_mult_registers_nonblock_getdata(uint8_t *buf);
void mcp2515_write_mult_registers_nonblock_init(uint8_t start_reg_addr, uint8_t num_regs, uint8_t *data);
void mcp2515_write_mult_registers_nonblock_end(void);
void mcp2515_bitmod_register_nonblock_init(uint8_t reg_addr, uint8_t mask, uint8_t data);
void mcp2515_bitmod_register_nonblock_end(void);
void mcp2515_rts_nonblock_init(uint8_t buffer);
void mcp2515_rts_nonblock_end(void);
void mcp2515_read_rxbuf0_nonblock_init(uint8_t start_reg_addr, uint8_t num_regs);
uint8_t mcp2515_read_rxbuf0_nonblock_getdata(uint8_t *buf);

#define CAN_RX0BUF_INT (!(P2IN&BIT5))
#define CAN_RX1BUF_INT (!(P2IN&BIT6))
#define CAN_GEN_INT		(!(P2IN&BIT4))

/* Tx Registers */
//Transmit Buffer Control Registers
#define MCP2515_TXB0CTRL	0x30	//Buffer 0
#define MCP2515_TXB1CTRL	0x40	//Buffer 1
#define MCP2515_TXB2CTRL	0x50	//Buffer 2
#define MCP2515_ABTF		BIT6	//Message Aborted: 1=Message aborted, 0=Transmission successful
#define MCP2515_MLOA		BIT5	//Message Lost Arbitration: 1=Message lost arbitration, 0=Message did not lose arbitration
#define MCP2515_TXERR		BIT4	//Transmission Error Detected: 1=Bus error occurred, 0=No bus error occurred
#define MCP2515_TXREQ		BIT3	//Message Transmit Request: 1=MCU requests transmit, 0=no transmission in progress
#define MCP2515_TXP_0		0x00	//Transmit Priority: Lowest
#define MCP2515_TXP_1		0x01	//Transmit Priority: Medium low
#define MCP2515_TXP_2		0x02	//Transmit Priority: Medium High
#define MCP2515_TXP_3		0x03	//Transmit Priority: High

//TXnRTS pin Control and Status Register
#define MCP2515_TXRTSCTRL	0x0D
#define MCP2515_B2RTS		BIT5	//TX2RTS pin state: reads digital input pin
#define MCP2515_B1RTS		BIT4	//TX1RTS pin state: reads digital input pin
#define MCP2515_B0RTS		BIT3	//TX0RTS pin state: reads digital input pin
#define MCP2515_B2RTSM		BIT2	//TX2RTS pin mode: 1=request transmission of TXB2, 0=digital input
#define MCP2515_B1RTSM		BIT1	//TX1RTS pin mode: 1=request transmission of TXB1, 0=digital input
#define MCP2515_B0RTSM		BIT0	//TX0RTS pin mode: 1=request transmission of TXB0, 0=digital input

//Transmit Buffer Standard Identifier High
#define MCP2515_TXB0SIDH	0x31	//Buffer 0, SID[10:3] in bit[7:0]
#define MCP2515_TXB1SIDH	0x41	//Buffer 1, SID[10:3] in bit[7:0]
#define MCP2515_TXB2SIDH	0x51	//Buffer 2, SID[10:3] in bit[7:0]

//Transmit Buffer Standard Identifier Low
#define MCP2515_TXB0SIDL	0x32	//Buffer 0, SID[2:0] in bit[7:5], EID[17:16] in bit[1:0]
#define MCP2515_TXB1SIDL	0x42	//Buffer 1, SID[2:0] in bit[7:5], EID[17:16] in bit[1:0]
#define MCP2515_TXB2SIDL	0x52	//Buffer 2, SID[2:0] in bit[7:5], EID[17:16] in bit[1:0]
#define MCP2515_TXEXIDE		BIT3	//Extended Identifier Enable: 1=use extended, 0=use standard

//Transmit Buffer Extended Identifier High
#define MCP2515_TXB0EID8	0x33	//Buffer 0, EID[15:8] in bit[7:0]
#define MCP2515_TXB1EID8	0x43	//Buffer 1, EID[15:8] in bit[7:0]
#define MCP2515_TXB2EID8	0x53	//Buffer 2, EID[15:8] in bit[7:0]

//Transmit Buffer Extended Identifier Low
#define MCP2515_TXB0EID0	0x34	//Buffer 0, EID[7:0] in bit[7:0]
#define MCP2515_TXB1EID0	0x44	//Buffer 1, EID[7:0] in bit[7:0]
#define MCP2515_TXB2EID0	0x54	//Buffer 2, EID[7:0] in bit[7:0]

//Transmit Buffer Data Length Code
#define MCP2515_TXB0DLC		0x35	//Buffer 0, DLC[3:0] indicate packet length
#define MCP2515_TXB1DLC		0x45	//Buffer 1, DLC[3:0] indicate packet length
#define MCP2515_TXB2DLC		0x55	//Buffer 2, DLC[3:0] indicate packet length
#define MCP2515_RTR			BIT6	//Remote Transmission Request: 1=Message is Remote frame, 0=Message is data frame

//Transmit Buffer Data Bytes
#define MCP2515_TXB0D0		0x36	//Buffer 0 byte 0
#define MCP2515_TXB0D1		0x37	//Buffer 0 byte 1
#define MCP2515_TXB0D2		0x38	//Buffer 0 byte 2
#define MCP2515_TXB0D3		0x39	//Buffer 0 byte 3
#define MCP2515_TXB0D4		0x3A	//Buffer 0 byte 4
#define MCP2515_TXB0D5		0x3B	//Buffer 0 byte 5
#define MCP2515_TXB0D6		0x3C	//Buffer 0 byte 6
#define MCP2515_TXB0D7		0x3D	//Buffer 0 byte 7

#define MCP2515_TXB1D0		0x46	//Buffer 1 byte 0
#define MCP2515_TXB1D1		0x47	//Buffer 1 byte 1
#define MCP2515_TXB1D2		0x48	//Buffer 1 byte 2
#define MCP2515_TXB1D3		0x49	//Buffer 1 byte 3
#define MCP2515_TXB1D4		0x4A	//Buffer 1 byte 4
#define MCP2515_TXB1D5		0x4B	//Buffer 1 byte 5
#define MCP2515_TXB1D6		0x4C	//Buffer 1 byte 6
#define MCP2515_TXB1D7		0x4D	//Buffer 1 byte 7

#define MCP2515_TXB2D0		0x56	//Buffer 2 byte 0
#define MCP2515_TXB2D1		0x57	//Buffer 2 byte 1
#define MCP2515_TXB2D2		0x58	//Buffer 2 byte 2
#define MCP2515_TXB2D3		0x59	//Buffer 2 byte 3
#define MCP2515_TXB2D4		0x5A	//Buffer 2 byte 4
#define MCP2515_TXB2D5		0x5B	//Buffer 2 byte 5
#define MCP2515_TXB2D6		0x5C	//Buffer 2 byte 6
#define MCP2515_TXB2D7		0x5D	//Buffer 2 byte 7

/* Receive Registers */
#define MCP2515_RXF0SIDH	0x00	//Filter 0 Standard Identifier High, SID[10:3] in bit[7:0]
#define MCP2515_RXF1SIDH	0x04	//Filter 1 Standard Identifier High, SID[10:3] in bit[7:0]
#define MCP2515_RXF2SIDH 	0x08	//Filter 2 Standard Identifier High, SID[10:3] in bit[7:0]
#define MCP2515_RXF3SIDH	0x10	//Filter 3 Standard Identifier High, SID[10:3] in bit[7:0]
#define MCP2515_RXF4SIDH	0x14	//Filter 4 Standard Identifier High, SID[10:3] in bit[7:0]
#define MCP2515_RXF5SIDH	0x18	//Filter 5 Standard Identifier High, SID[10:3] in bit[7:0]

#define MCP2515_RXF0SIDL	0x01	//Filter 0 Standard Identifier Low, SID[2:0] in bit[7:5], EID[17:16] in bit[1:0]
#define MCP2515_RXF1SIDL	0x05	//Filter 1 Standard Identifier Low, SID[2:0] in bit[7:5], EID[17:16] in bit[1:0]
#define MCP2515_RXF2SIDL	0x09	//Filter 2 Standard Identifier Low, SID[2:0] in bit[7:5], EID[17:16] in bit[1:0]
#define MCP2515_RXF3SIDL	0x11	//Filter 3 Standard Identifier Low, SID[2:0] in bit[7:5], EID[17:16] in bit[1:0]
#define MCP2515_RXF4SIDL	0x15	//Filter 4 Standard Identifier Low, SID[2:0] in bit[7:5], EID[17:16] in bit[1:0]
#define MCP2515_RXF5SIDL	0x19	//Filter 5 Standard Identifier Low, SID[2:0] in bit[7:5], EID[17:16] in bit[1:0]
#define MCP2515_RxEXIDE		BIT3	//Extended Identifier Enable, 1=Filter applied to extended, 0=filter applied to standard

#define MCP2515_RXF0EID8	0x02	//Filter 0 Extended Identifier High, EID[15:8] in bit[7:0]
#define MCP2515_RXF1EID8	0x06	//Filter 1 Extended Identifier High, EID[15:8] in bit[7:0]
#define MCP2515_RXF2EID8	0x0A	//Filter 2 Extended Identifier High, EID[15:8] in bit[7:0]
#define MCP2515_RXF3EID8	0x12	//Filter 3 Extended Identifier High, EID[15:8] in bit[7:0]
#define MCP2515_RXF4EID8	0x16	//Filter 4 Extended Identifier High, EID[15:8] in bit[7:0]
#define MCP2515_RXF5EID8	0x1A	//Filter 5 Extended Identifier High, EID[15:8] in bit[7:0]

#define MCP2515_RXF0EID0	0x03	//Filter 0 Extended Identifier Low, EID[7:0] in bit[7:0]
#define MCP2515_RXF1EID0	0x07	//Filter 1 Extended Identifier Low, EID[7:0] in bit[7:0]
#define MCP2515_RXF2EID0	0x0B	//Filter 2 Extended Identifier Low, EID[7:0] in bit[7:0]
#define MCP2515_RXF3EID0	0x13	//Filter 3 Extended Identifier Low, EID[7:0] in bit[7:0]
#define MCP2515_RXF4EID0	0x17	//Filter 4 Extended Identifier Low, EID[7:0] in bit[7:0]
#define MCP2515_RXF5EID0	0x1B	//Filter 5 Extended Identifier Low, EID[7:0] in bit[7:0]

#define MCP2515_RXM0SIDH	0x20	//Mask 0 Standard Identifier High, SID[10:3] in bit[7:0]
#define MCP2515_RXM1SIDH	0x24	//Mask 1 Standard Identifier High, SID[10:3] in bit[7:0]

#define MCP2515_RXM0SIDL	0x21	//Mask 0 Standard Identifier Low, SID[2:0] in bit[7:5], EID[17:16] in bit[1:0]
#define MCP2515_RXM1SIDL	0x25	//Mask 1 Standard Identifier Low, SID[2:0] in bit[7:5], EID[17:16] in bit[1:0]

#define MCP2515_RXM0EID8	0x22	//Mask 0 Extended Identifier High, EID[15:8] in bit[7:0]
#define MCP2515_RXM1EID8	0x26	//Mask 1 Extended Identifier High, EID[15:8] in bit[7:0]

#define MCP2515_RXM0EID0	0x23	//Mask 0 Extended Identifier Low, EID[7:0] in bit[7:0]
#define MCP2515_RXM1EID0	0x27	//Mask 1 Extended Identifier Low, EID[7:0] in bit[7:0]

#define MCP2515_RXB0CTRL	0x60	//Recieve buffer 0 control
#define MCP2515_RXB1CTRL	0x70	//Recieve buffer 1 control

/* Configuration Registers: Timing */
#define MCP2515_CNF1		0x2A	//Configuration 1
#define MCP2515_SJW_0		0x00	//Synchronization Jump Width: 1xTQ
#define MCP2515_SJW_1		0x40	//Synchronization Jump Width: 2xTQ
#define MCP2515_SJW_2		0x80	//Synchronization Jump Width: 3xTQ
#define MCP2515_SJW_3		0xC0	//Synchronization Jump Width: 4xTQ
//BRP[5:0] in bit[5:0]: Baud rate prescaler

#define MCP2515_CNF2		0x29	//Configuration 2
#define MCP2515_BTLMODE		BIT7	//PS2 Bit Time Length
#define MCP2515_SAM			BIT6	//Sample Point Configuration
//PHSEG1[2:0] in bit[5:3]: PS1 Length bits
//PRSEG[2:0] in bit[2:0]: Propagation Segment Length bits

#define MCP2515_CNF3		0x28	//Configuration 3
#define MCP2515_SOF			BIT7	//Start of Frame Signal
#define MCP2515_WAKFIL		BIT6	//Wakeup filter enable
//PHSEG2[2:0] in bit[2:0]: PS2 Length bits

/* Error Indication Registers */
#define MCP2515_CANINTE		0x2B	//Interrupt Enable
#define MCP2515_CANINTF		0x2C	//Interrupt flag
#define MCP2515_MERRF		BIT7	//Message error interrupt flag
#define MCP2515_WAKIF		BIT6	//Wake up interrupt flag
#define MCP2515_ERRIF		BIT5	//Error interrupt flag
#define MCP2515_TX2IF		BIT4	//Transmit buffer 2 empty interrupt flag
#define MCP2515_TX1IF		BIT3	//Transmit buffer 1 empty interrupt flag
#define MCP2515_TX0IF		BIT2	//Transmit buffer 0 empty interrupt flag
#define MCP2515_RX1IF		BIT1	//Recieve buffer 1 full interrupt flag
#define MCP2515_RX0IF		BIT0	//Recieve buffer 0 full interrupt flag

#define MCP2515_EFLG		0x2D	//Error flag
#define MCP2515_RX1OVR		BIT7	//Recieve buffer 1 overflow
#define MCP2515_RX0OVR		BIT6	//Recieve buffer 0 overflow
#define MCP2515_TXBO		BIT5	//Buss off error flag
#define MCP2515_TXEP		BIT4	//Tranmit error passive flag
#define MCP2515_RXEP		BIT3	//Recieve error passive flag
#define MCP2515_TXWAR		BIT2	//Transmit error warning flag
#define MCP2515_RXWAR		BIT1	//Recieve error warning flag
#define MCP2515_EWARN		BIT0	//Error warning flag
#define MCP2515_TEC			0x1C	//Transmit error count
#define MCP2515_REC			0x1D	//Recieve error count




#endif /* MCP2515_H_ */
