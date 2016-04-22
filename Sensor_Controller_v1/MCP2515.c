/* MCP2515.c
 * Created 1/26/16 by Nishant Pol
 * 16-450/16-474 Robotics Capstone
 * Sensor Controller Code
 *
 * Code Composer v6
 * MSP430F5521
 *
 * Revision History
 * 1/26/16: Nishant Pol
 * - Initial release
 * 4/16/16: Nishant Pol
 * - Adapted for MSP430F5521
 * - Added non-blocking functions for state machine
 *
 * I/O Pinout to CAN Transciever (MCP2515) and CAN driver (MCP2562)
 * Uses USCIB0
 * P3.2: SCK
 * P3.1: MISO
 * P3.0: MOSI
 * P2.3: CS (active low)
 * P2.4: RX buffer 0 full interrupt (active low)
 * P2.5: RX buffer 1 full interrupt (active low)
 * P2.7: CAN standby
 */

#include "MCP2515.h"

#define LOOPBACK_EN

void setup_mcp2515(void){
	/* Configure MCP2562 to enable standby */
	P2DIR |= BIT7;
	mcp2562_shutdown_enable();

	/* Configure MCP2515 */
	mcp2515_reset();
	//TODO: configure buf0, buf1, setup filters
	mcp2515_write_register(MCP2515_RXB0CTRL, 0x60);	//Recieve any message
	mcp2515_write_register(0x0C, BIT2+BIT0);	//Enable RX0BF interrupt
	mcp2515_write_register(MCP2515_CANINTE, BIT7);			//Enable general interrupt for errors
	mcp2515_write_register(0x2A, 3);			//Decrease clock frequency
	/* Set normal mode */
	mcp2515_write_register(0x0F, 0x0f);	//Request normal mode, CLKOUT enabled, Clk prescaler /8
	int response = mcp2515_read_register(0x0E);	//Read CANSTAT register to check mode set
	if((response >> 5) & 7 != 0){
		issue_warning(WARN_CAN_MODE_SET_FAIL);
	}
#ifdef LOOPBACK_EN
	//Loopback mode for debugging
	mcp2515_write_register(0x0F, 0x47);	//Enter loopback mode
#endif
	mcp2562_shutdown_disable();
	return;
}

void mcp2562_shutdown_enable(void){
	P2OUT |= BIT7;
}

void mcp2562_shutdown_disable(void){
	P2OUT &= ~BIT7;
}

/* Read single register from MCP2515
 * SPI transaction:
 * [ RD (0x03) , ADDR , DATA (RX) ]
 * Return third byte recieved
 */
uint8_t mcp2515_read_register(uint8_t reg_addr){
	uint8_t buf[3] = {0x03,0x00,0x00};
	buf[1] = reg_addr;
	init_CAN_SPI_transac(buf, 3);
	while(!is_CAN_spi_rx_ready());
	get_CAN_SPI_rx_data(buf);
	return buf[2];
}

/* Start read of multiple registers
 * SPI transaction:
 * [ RD (0x03), ADDR, Data1, Data2,...]
 * Max 14 registers
 */
void mcp2515_read_mult_registers_nonblock_init(uint8_t start_reg_addr, uint8_t num_regs){
	uint8_t buf[16] = {0x03,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	buf[1] = start_reg_addr;
	init_CAN_SPI_transac(buf,num_regs+2);
	return;
}

/* End read of multiple registers
 * register reads are stored in buf
 * returns number of registers read
 */
uint8_t mcp2515_read_mult_registers_nonblock_getdata(uint8_t *buf){
	uint8_t data_buf[16];
	uint8_t num_regs = get_CAN_SPI_rx_data(data_buf);
	uint8_t i;
	for(i=2; i<num_regs; i++){
		buf[i-2] = data_buf[i];
	}
	return num_regs-2;
}

/* Write single register from MCP2515
 * SPI transaction:
 * [ WR (0x02) , ADDR, DATA (TX) ]
 */
void mcp2515_write_register(uint8_t reg_addr, uint8_t data){
	uint8_t buf[3] = {0x02,0x00,0x00};
	buf[1] = reg_addr;
	buf[2] = data;
	init_CAN_SPI_transac(buf, 3);
	while(!is_CAN_spi_rx_ready());
	end_CAN_SPI_transac();
	return;
}

/* Start write of multiple registers
 * SPI transaction:
 * [ WR (0x02), ADDR, Data1, Data2,...]
 * Max 14 registers
 */
void mcp2515_write_mult_registers_nonblock_init(uint8_t start_reg_addr, uint8_t num_regs, uint8_t *data){
	uint8_t buf[16] = {0x02,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	buf[1] = start_reg_addr;
	uint8_t i;
	for(i=0; i<num_regs; i++){
		buf[i+2] = data[i];
	}
	init_CAN_SPI_transac(buf,num_regs+2);
	return;
}

/* End write of multiple registers
 */
void mcp2515_write_mult_registers_nonblock_end(void){
	end_CAN_SPI_transac();
	return;
}

/* SPI reset of MCP2515 */
void mcp2515_reset(void){
	uint8_t buf[1] = {0xC0};
	init_CAN_SPI_transac(buf,1);
	while(!is_CAN_spi_rx_ready());
	end_CAN_SPI_transac();
	return;
}

/* Bit modify register
 *
 */
void mcp2515_bitmod_register_nonblock_init(uint8_t reg_addr, uint8_t mask, uint8_t data){
	uint8_t buf[4] = {0x05,0,0,0};
	buf[1] = reg_addr;
	buf[2] = mask;
	buf[3] = data;
	init_CAN_SPI_transac(buf,4);
}

void mcp2515_bitmod_register_nonblock_end(void){
	end_CAN_SPI_transac();
}

/* Request to send message
 *
 */
void mcp2515_rts_nonblock_init(uint8_t buffer){
	uint8_t data = 0x80 | (BIT1 << buffer);
	init_CAN_SPI_transac(&data,1);
}

void mcp2515_rts_nonblock_end(void){
	end_CAN_SPI_transac();
}
