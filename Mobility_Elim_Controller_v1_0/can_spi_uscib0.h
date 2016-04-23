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
 */

#ifndef CAN_SPI_USCIB0_H_
#define CAN_SPI_USCIB0_H_

#include <msp430.h>
#include "utils.h"

/* SPI Macros for CS */
#define CAN_SPI_BUF_SIZE 32
#define CAN_SPI_CS_ASSERT			(P2OUT &= ~BIT3)
#define CAN_SPI_CS_DEASSERT 		(P2OUT |= BIT3)
#define CAN_SPI_TXINT_ENABLE        (UCB0IE |= UCTXIE)
#define CAN_SPI_TXINT_DISABLE       (UCB0IE &= ~UCTXIE)
#define CAN_SPI_RXINT_ENABLE        (UCB0IE |= UCRXIE)
#define CAN_SPI_RXINT_DISABLE       (UCB0IE &= ~UCRXIE)
#define CAN_SPI_INT_ENABLE				(UCB0IE |= (UCTXIE+UCRXIE))
#define CAN_SPI_INT_DISABLE				(UCB0IE &= ~(UCTXIE+UCRXIE))

struct CAN_SPI_data_struct{
	uint8_t tx_bytes[CAN_SPI_BUF_SIZE];	//Bytes to send to slave
	uint8_t rx_bytes[CAN_SPI_BUF_SIZE];	//Bytes recieved from slave
	uint8_t tx_ptr;					//Index of byte to transmit next
	uint8_t rx_ptr;					//Index of next available empty space for recieved byte
	uint8_t num_bytes;				//Number of bytes to recieve/transmit
	uint8_t in_use_flag;			//Indicates if data has not been processed by main State machine
	uint8_t data_ready;				//Indicates that rx_bytes is valid
};
extern volatile struct CAN_SPI_data_struct CAN_SPI_data;

void CAN_SPI_setup(uint8_t idle, uint8_t edge);
void init_CAN_SPI_transac(uint8_t *tx_bytes, uint8_t num_bytes);
uint8_t get_CAN_SPI_rx_data(uint8_t *rx_data);
void end_CAN_SPI_transac(void);
uint8_t is_CAN_spi_busy(void);
uint8_t is_CAN_spi_rx_ready(void);


#endif /* SPI_H_ */
