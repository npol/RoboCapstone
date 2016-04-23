/* can_spi_uscib0.c
 * Created 11/25/15 by Nishant Pol
 * Mechatronics 18-578
 * MSP430F5521
 * Sensor Controller Code
 *
 * Revision History
 * 6/10/15: Nishant Pol
 * - Initial release for eInsights Live Pressure Sensor
 * 11/25/15: Nishant Pol
 * - Repurposed code for RoboCapstone Bus Prototype
 * 1/24/16: Nishant Pol
 * - Updated CS to reflect new pinout
 * 3/30/16: Nishant Pol and Jaime Chu
 * - Repurposed code for Mechatronics
 * - Modified code for MSP430F5521 (previously MSP430G2553)
 * 4/16/16: Nishant Pol
 * - Repurposed code for RoboCapstone Sensor Controller
 *
 * SPI Pinout to CAN Controller MCP2515
 * Uses USCIB0
 * P3.2: SCK
 * P3.1: MISO
 * P3.0: MOSI
 * P2.3: CS (active low)
 * P2.4: RX buffer 0 full interrupt (active low)
 * P2.5: RX buffer 1 full interrupt (active low)
 * P2.7: CAN standby
 */
#include "can_spi_uscib0.h"

/* SPI Datastructure
 * Stores data and config relevant to current transaction
 */
volatile struct CAN_SPI_data_struct CAN_SPI_data = {
	.tx_bytes = {0},
	.rx_bytes = {0},
	.tx_ptr = 0,
	.rx_ptr = 0,
	.num_bytes = 0,
	.in_use_flag = 0,
	.data_ready = 0,
};

/* Setup SPI as Master on USCI_B0
 * Clock Source: SMCLK (25MHz)/
 * idle: When idle, clock is low (0) or high (1)
 * edge: write bus on idle-active (0) or active-idle (1) edge
 */
void CAN_SPI_setup(uint8_t idle, uint8_t edge){
	//Hold USCI in reset for setup
	UCB0CTL1 |= UCSWRST;

	UCB0CTL0 = (edge<<7) |	//Clock phase
			   (idle<<6) | 	//Clock polarity
			   UCMSB     |	//MSB first
			   UCMST     |	//Master
			   UCMODE_0;	//3-pin SPI

	UCB0CTL1 = UCSSEL_2  |	//Source from SMCLK
			   UCSWRST;		//Keep USCI in reset

	UCB0BR0 = 25;			//run at 1MHz

	//Enable use of SPI pins MOSI, MISO, SCK
	P3SEL |= BIT0 + BIT1 + BIT2;

	//CS on P2.3, set output high (disabled)
	P2DIR |= BIT3;
	CAN_SPI_CS_DEASSERT;

	UCB0CTL1 &= ~UCSWRST;	//Release USCI from Reset
	return;
}

/* SM loads SPI datastructure, start transaction
 * tx_bytes: array of bytes to send
 * num_bytes: number of bytes to send
 * brd: Board to access (0,1,2)
 */
void init_CAN_SPI_transac(uint8_t *tx_bytes, uint8_t num_bytes){
	CAN_SPI_data.in_use_flag = 1;
	uint8_t i;
	for(i = 0; i < num_bytes; i++){			//Copy Tx data to SPI datastructure
		CAN_SPI_data.tx_bytes[i] = tx_bytes[i];
	}
	CAN_SPI_data.num_bytes = num_bytes;			//Copy Transaction size to SPI datastructure
	CAN_SPI_data.tx_ptr = 1;					//Reset pointers
	CAN_SPI_data.rx_ptr = 0;
	CAN_SPI_CS_ASSERT;
	CAN_SPI_RXINT_ENABLE;	//Enable SPI Interrupt
	UCB0TXBUF = CAN_SPI_data.tx_bytes[0];		//Load first byte
	return;
}

/* SM gets recieved data, releases SPI datastructure
 * rx_data: buffer to store recieved data
 */
uint8_t get_CAN_SPI_rx_data(uint8_t *rx_data){
	uint8_t i;
	for(i = 0; i < CAN_SPI_data.num_bytes; i++){	//Copy data
		rx_data[i] = CAN_SPI_data.rx_bytes[i];
	}
	CAN_SPI_INT_DISABLE;
	CAN_SPI_data.data_ready = 0;
	CAN_SPI_data.in_use_flag = 0;						//Release SPI datastructure
	return CAN_SPI_data.num_bytes;
}

/* SM releases SPI datastructure without copying received data
 *
 */
void end_CAN_SPI_transac(void){
	CAN_SPI_INT_DISABLE;
	CAN_SPI_data.data_ready = 0;
	CAN_SPI_data.in_use_flag = 0;						//Release SPI datastructure
	return;
}

/* SM can poll SPI with this function to see if SPI is in use
 *
 */
uint8_t is_CAN_spi_busy(void){
	uint8_t ret_val;
	__disable_interrupt();
	ret_val = CAN_SPI_data.in_use_flag;
	__enable_interrupt();
	return ret_val;
}

uint8_t is_CAN_spi_rx_ready(void){
	uint8_t ret_val;
	__disable_interrupt();
	ret_val = CAN_SPI_data.data_ready;
	__enable_interrupt();
	return ret_val;
}
