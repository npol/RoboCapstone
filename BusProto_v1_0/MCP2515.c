/* MCP2515.c
 * Created 1/26/16 by Nishant Pol
 * 16-450/16-474 Robotics Capstone
 * CAN Bus Prototype
 *
 * Code Composer v6
 * MSP430G2553IPW28
 *
 * Revision History
 * 1/26/16: Nishant Pol
 * - Initial release
 *
 * I/O Pinout to CAN Transciever (MCP2515) and CAN driver (MCP2562)
 * P2.5: STBY (active high output to MCP2562)
 * P2.4: CAN_INT (active low input)
 * P1.3: CAN_RX0BF (active low input)
 * P1.4: CAN_RX1BF (active low input)
 * P3.4: CAN_TX0RTS (active low output)
 * P3.5: CAN_TX1RTS (active low output)
 * P3.6: CAN_TX2RTS (active low output)
 * For SPI pinout and functions, see spi.c
 */

#include "MCP2515.h"

#define MCP2562_SHTDN BIT5	//P2.5

void setup_mcp2515(void){
	/* Configure MCP2562 to disable standby */
	P2DIR |= BIT5;
	mcp2562_shutdown_disable();

	/* Configure MCP2515 */
	mcp2515_reset();
	/* Temporary: Allow any message to be recieved in buffer 0 */
	mcp2515_write_register(0x60, 0x60);	//Recieve any message
	mcp2515_write_register(0x0C, 0x05);	//Enable RX0BF interrupt
	mcp2515_write_register(0x2B, 1);
	mcp2515_write_register(0x2A, 3);	//Decrease clock frequency
	/* Set normal mode */
	mcp2515_write_register(0x0F, 0x0f);	//Request normal mode, CLKOUT enabled, Clk prescaler /8
	int response = mcp2515_read_register(0x0E);	//Read CANSTAT register to check mode set
	if((response >> 5) & 7 != 0){
		while(1);//TODO: Flag error that normal mode was not entered.
	}
	//Temporary: Loopback mode for debugging
	//mcp2515_write_register(0x0F, 0x47);	//Enter loopback mode
	return;
}

void mcp2562_shutdown_enable(void){
	P2OUT |= BIT5;
}

void mcp2562_shutdown_disable(void){
	P2OUT &= ~BIT5;
}

/* Read single register from MCP2515
 * SPI transaction:
 * [ RD (0x03) , ADDR , DATA (RX) ]
 * Return third byte recieved
 */
uint8_t mcp2515_read_register(uint8_t reg_addr){
	uint8_t buf[3] = {0x03,0x00,0x00};
	buf[1] = reg_addr;
	init_SPI_transac(buf, 3);
	while(!is_spi_rx_ready());
	get_SPI_rx_data(buf);
	return buf[2];
}

/* Write single register from MCP2515
 * SPI transaction:
 * [ WR (0x02) , ADDR, DATA (TX) ]
 */
void mcp2515_write_register(uint8_t reg_addr, uint8_t data){
	uint8_t buf[3] = {0x02,0x00,0x00};
	buf[1] = reg_addr;
	buf[2] = data;
	init_SPI_transac(buf, 3);
	while(!is_spi_rx_ready());
	end_SPI_transac();
	return;
}

/* SPI reset of MCP2515 */
void mcp2515_reset(void){
	uint8_t buf[1] = {0xC0};
	init_SPI_transac(buf,1);
	while(!is_spi_rx_ready());
	end_SPI_transac();
	return;
}

