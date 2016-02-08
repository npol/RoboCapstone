/*
 * spi.h
 * Created 6/10/15 by Nishant Pol
 * Consulting for eInsightsLive
 *
 * Device Specific code for LPS25H Pressure Sensor
 * Code Composer v6
 *
 * SPI Pinout to Pressure Sensor
 * Uses USCIB0 on MSP430G2553
 * P1.5: SCK
 * P2.4: CS (active low)
 * P1.6: MISO
 * P1.7: MOSI
 */

#ifndef SPI_H_
#define SPI_H_

#include <msp430.h>
#include "utils.h"

/* SPI Macros for CS */
#define SPI_BUF_SIZE 8
#define SPI_CS_ASSERT	(P3OUT &= ~BIT7)
#define SPI_CS_DEASSERT (P3OUT |= BIT7)
#define SPI_TXINT_ENABLE        (IE2 |= UCB0TXIE)
#define SPI_TXINT_DISABLE       (IE2 &= ~UCB0TXIE)
#define SPI_RXINT_ENABLE        (IE2 |= UCB0RXIE)
#define SPI_RXINT_DISABLE       (IE2 &= ~UCB0RXIE)

struct SPI_data_struct{
	uint8_t tx_bytes[SPI_BUF_SIZE];	//Bytes to send to slave
	uint8_t rx_bytes[SPI_BUF_SIZE];	//Bytes recieved from slave
	uint8_t tx_ptr;					//Index of byte to transmit next
	uint8_t rx_ptr;					//Index of next available empty space for recieved byte
	uint8_t num_bytes;				//Number of bytes to recieve/transmit
	uint8_t in_use_flag;			//Indicates if data has not been processed by main State machine
	uint8_t data_ready;				//Indicates that rx_bytes is valid
};
extern volatile struct SPI_data_struct SPI_data;

void USCIB0_SPI_setup(uint8_t idle, uint8_t edge);
void init_SPI_transac(uint8_t *tx_bytes, uint8_t num_bytes);
uint8_t get_SPI_rx_data(uint8_t *rx_data);
void end_SPI_transac(void);
uint8_t is_spi_busy(void);
uint8_t is_spi_rx_ready(void);


#endif /* SPI_H_ */
