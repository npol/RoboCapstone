/* clock.c
 * Created 11/25/15 by Nishant Pol
 * 16-450/16-474 Robotics Capstone
 * CAN Bus Prototype
 *
 * Code Composer v6
 * MSP430G2553IPW28
 *
 * Revision History
 * 11/25/15: Nishant Pol
 * - Initial release
 */

#include "clock.h"

/* Setup Clock Tree:
 * DCO set to 16MHz
 * ACLK sourced from DCO
 * MCLK sourced from DCO
 */
void setup_clock(void){
	//ACLK sourced from LFXT1/1 by default on POR
	//MCLK sourced from DCOCLK/1 by default on POR
	 /*//1Mhz
	  if (CALBC1_1MHZ != 0xFF)					// If calibration constant erased
	  {
		  DCOCTL = 0;                               // Select lowest DCOx and MODx settings
		  BCSCTL1 = CALBC1_1MHZ;                    // Set range
		  DCOCTL = CALDCO_1MHZ;                     // Set DCO step + modulation */
//	  } else {
		  //TODO: Log and report erase of cal data
//	  }

	/* //8Mhz
	  if (CALBC1_8MHZ==0xFF)					// If calibration constant erased
	  {
	    while(1);                               // do not load, trap CPU!!
	  }
	  DCOCTL = 0;                               // Select lowest DCOx and MODx settings
	  BCSCTL1 = CALBC1_8MHZ;                    // Set range
	  DCOCTL = CALDCO_8MHZ;                     // Set DCO step + modulation */

	/* //12Mhz
	  if (CALBC1_12MHZ==0xFF)					// If calibration constant erased
	  {
	    while(1);                               // do not load, trap CPU!!
	  }
	  DCOCTL = 0;                               // Select lowest DCOx and MODx settings
	  BCSCTL1 = CALBC1_12MHZ;                   // Set range
	  DCOCTL = CALDCO_12MHZ;                    // Set DCO step + modulation*/

	 //16Mhz
	  if (CALBC1_16MHZ==0xFF)					// If calibration constant erased
	  {
	    while(1);                               // do not load, trap CPU!!
	  }
	  DCOCTL = 0;                               // Select lowest DCOx and MODx settings
	  BCSCTL1 = CALBC1_16MHZ;                   // Set range
	  DCOCTL = CALDCO_16MHZ;                    // Set DCO step + modulation
	//SMCLK sourced from DCOCLK/1 by default on POR
	//BCSCTL3 |= XCAP_1;	//6pF internal caps

	//TODO: Enable after implementing OFIFG handler
	//IE1 |= OFIE;		//Enable OFIFG oscillator fault interrupt
	return;
}


