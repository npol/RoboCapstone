/*
 * main.c
 * Created 3/17/16 by Nishant Pol
 * Robotics Capstone 16-474/Mechatronics 18-578
 * MSP430F5521
 * Mobility/Elimination Controller Code
 *
 * Code Composer v6
 * Version History
 * 3/17/16: Ported from f552x_bringup_tests
 */
#include <msp430.h>
#include "utils.h"
#include <string.h>
#include "clock_f5.h"
#include "dbg_uart_uscia0.h"
#include "roboclaw.h"
#include "rc_uart_uscia1.h"
#include "drill.h"
#include "stepper.h"

/** Debug task macros and globals **/
void debug_task(void);

#define DEBUG_CMD_BUF_SIZE 32
uint8_t debug_cmd_buf[DEBUG_CMD_BUF_SIZE];
uint8_t debug_cmd_buf_ptr = 0;
/** END Debug task macros and globals **/

/** Warning/Error code buffers and flags **/
volatile uint16_t warn_log[WARN_LOG_SIZE] = {0};
volatile uint8_t warn_log_ptr = 0;
volatile uint8_t warn_flag = 0;
volatile uint16_t err_log[ERR_LOG_SIZE] = {0};
volatile uint8_t err_log_ptr = 0;
volatile uint8_t err_flag = 0;
/** END Warning/Error code buffers and flags **/

/** Roboclaw task globals **/

/* Roboclaw State machine globals */
typedef enum  {RC_WAIT,
				TIME_INTERVALS,
				SEND_PCKT_REQ,
				WAIT_PCKT_REQ,
				GET_PCKT_DATA,
				PCKT_ERR,
				SEND_ENC1_REQ,
				WAIT_ENC1_REQ,
				ERR_ENC1_REQ,
				SEND_ENC2_REQ,
				WAIT_ENC2_REQ,
				ERR_ENC2_REQ,
				SEND_ENC_CAN,
				SEND_MBATT_REQ,
				WAIT_MBATT_REQ,
				ERR_MBATT_REQ,
				SEND_LBATT_REQ,
				WAIT_LBATT_REQ,
				ERR_LBATT_REQ,
				SEND_MCUR_REQ,
				WAIT_MCUR_REQ,
				ERR_MCUR_REQ,
				SEND_TEMP_REQ,
				WAIT_TEMP_REQ,
				ERR_TEMP_REQ,
				SEND_STAT_REQ,
				WAIT_STAT_REQ,
				ERR_STAT_REQ,
				CHECK_STAT_REQ} rc_state_t;
volatile rc_state_t rcCurrState = RC_WAIT;
#define WAIT_THRESH 100
#define RC_RUN_CHECK_INTERVAL 1000

/* Struct to request asynchronous packet transaction */
#define RC_REQ_SIZE 64
struct RC_async_request_struct{
	uint8_t rc_request_flag;	//Set to request packet transmission
	uint8_t tx_bytes[RC_REQ_SIZE];	//Bytes to send
	uint8_t rx_bytes[RC_REQ_SIZE];	//Bytes recieved
	uint8_t tx_nbytes;				//Number of bytes to send
	uint8_t rx_nbytes;				//Number of bytes to recieve
};
volatile struct RC_async_request_struct RC_async_request = {
	.rc_request_flag = 0,
	.tx_bytes = {0},
	.rx_bytes = {0},
	.tx_nbytes = 0,
	.rx_nbytes = 0
};
/* Set on TA2 interrupt */
volatile uint8_t timer_TA2_tick = 0;
/* Encoder 1 count */
uint32_t enc1_count = 0;
/* Encoder 2 count */
uint32_t enc2_count = 0;

void roboclaw_task(void);

/** END Roboclaw task globals **/

/** Main Loop **/

int main(void) {
	WDTCTL = WDTPW + WDTHOLD;   // Stop watchdog timer
	P1DIR |= BIT0;	//Debug LED1
	P2DIR |= BIT2;	//Debug LED2
	P7DIR |= BIT7;	//Debug LED3
	P4DIR |= BIT7;	//Debug ERR LED
	P5DIR |= BIT6;	//Debug WARN LED
	P5DIR |= BIT7;	//Debug OK LED
	setup_clock();
	setup_dbg_uart();
	setup_rc_uart();
	drill_setup();
	stepper_setup();
    // Enable Interrupts
    __bis_SR_register(GIE);
    while(1)
    {
        debug_task();
        roboclaw_task();
        /*
        if(is_rc_uart_rx_data_ready()){
        	uint8_t rx_byte = rc_uart_get_byte();
        	dbg_uart_send_byte(rx_byte);
        	rc_uart_send_byte(rx_byte);
        }
        */
    }
}

/** END Main Loop **/

/** Roboclaw Task Functions **/

void roboclaw_task(void){
	static uint16_t timer_tics = 0;		//Number of TA2 tics to time encoders/check
	static uint8_t run_enc_flag = 0;	//Set when encoders should be checked
	static uint8_t run_check_flag = 0;	//Set when Check routine must run
	static uint8_t wait_cntr = 0;		//counter to prevent hang while waiting for failed packet
	uint8_t pckt_size = 0;				//Size of transmitted packet
	uint8_t buf[16];					//Buffer for transmitted/recieved packet
	switch(rcCurrState){
	case RC_WAIT:
		//State action: nothing
		//State transition
		if(timer_TA2_tick){						//T_DRV2
			rcCurrState = TIME_INTERVALS;
		} else if(run_enc_flag){				//T_DRV1
			rcCurrState = SEND_ENC1_REQ;
		} else if(RC_async_request.rc_request_flag){//T_DRV4
			rcCurrState = SEND_PCKT_REQ;
		} else if(run_check_flag){				//T_DRV22
			rcCurrState = SEND_MBATT_REQ;
		} else {
			rcCurrState = RC_WAIT;				//T_DRV49
		}
		break;
	case TIME_INTERVALS:
		//State action
		timer_tics++;
		if(timer_tics >= RC_RUN_CHECK_INTERVAL){
			timer_tics = 0;
			run_check_flag = 1;
		}
		run_enc_flag = 1;
		//State transition
		rcCurrState = RC_WAIT;						//T_DRV3
		break;
	case SEND_PCKT_REQ:
		//State action
		wait_cntr = 0;
		rc_uart_send_string(buf, pckt_size);	//Send packet to UART
		//State transition
		rcCurrState = WAIT_PCKT_REQ;			//T_DRV5
		break;
	case WAIT_PCKT_REQ:
		//State action
		wait_cntr++;
		//State transition
		if(is_rc_uart_rx_ndata_ready() == RC_async_request.rx_nbytes){
			rcCurrState = GET_PCKT_DATA;		//T_DRV7
		} else if(wait_cntr > WAIT_THRESH){
			rcCurrState = PCKT_ERR;				//T_DRV8
		} else {
			rcCurrState = WAIT_PCKT_REQ;		//T_DRV6
		}
		break;
	case GET_PCKT_DATA:
		//State action
		pckt_size = rc_uart_get_string(RC_async_request.rx_bytes,RC_async_request.rx_nbytes);
		RC_async_request.rc_request_flag = 0;
		//State transition
		rcCurrState = RC_WAIT;						//T_DRV9
		break;
	case PCKT_ERR:
		//State action
		RC_async_request.rc_request_flag = 0;
		issue_warning(WARN_RC_SM_ASYNC_PCKT_FAIL);
		//State transition
		rcCurrState = RC_WAIT;						//T_DRV10
		break;
	case SEND_ENC1_REQ:
		//State acton
		run_enc_flag = 0;
		wait_cntr = 0;
		pckt_size = RCgetEnc1Count(buf);		//Get packet
		rc_uart_send_string(buf, pckt_size);	//Send packet to UART
		//State transition
		rcCurrState = WAIT_ENC1_REQ;			//T_DRV11
		break;
	case WAIT_ENC1_REQ:
		//State acton
		wait_cntr++;
		//State transition
		if(is_rc_uart_rx_ndata_ready() == 7){
			rcCurrState = SEND_ENC2_REQ;		//T_DRV15
		} else if(wait_cntr > WAIT_THRESH){
			rcCurrState = ERR_ENC1_REQ;			//T_DRV13
		} else {
			rcCurrState = WAIT_ENC1_REQ;		//T_DRV12
		}
		break;
	case ERR_ENC1_REQ:
		//State action
		issue_warning(WARN_RC_SM_ENC1_PCKT_FAIL);
		//State transition
		rcCurrState = SEND_ENC1_REQ;			//T_DRV14
		break;
	case SEND_ENC2_REQ:
		//State action
		//Get data from encoder 1 count
		pckt_size = rc_uart_get_string(buf,7);
		if(pckt_size == 0){
			issue_warning(WARN_RC_SM_ENC1_DATA_FAIL);
		}
		enc1_count = ((uint32_t)buf[0]<<24)|((uint32_t)buf[1]<<16)|((uint32_t)buf[2]<<8)|((uint32_t)buf[3]);
		//Send encoder 2 count request
		wait_cntr = 0;
		pckt_size = RCgetEnc2Count(buf);		//Get packet
		rc_uart_send_string(buf, pckt_size);	//Send packet to UART
		//State transition
		rcCurrState = WAIT_ENC2_REQ;			//T_DRV16
		break;
	case WAIT_ENC2_REQ:
		//State action
		wait_cntr++;
		//State transition
		if(is_rc_uart_rx_ndata_ready() == 7){
			rcCurrState = SEND_ENC_CAN;			//T_DRV20
		} else if(wait_cntr > WAIT_THRESH){
			rcCurrState = ERR_ENC2_REQ;			//T_DRV18
		} else {
			rcCurrState = WAIT_ENC2_REQ;		//T_DRV17
		}
		break;
	case ERR_ENC2_REQ:
		//State action
		issue_warning(WARN_RC_SM_ENC2_PCKT_FAIL);
		//State transition
		rcCurrState = SEND_ENC2_REQ;			//T_DRV19
		break;
	case SEND_ENC_CAN:
		//State action
		//TODO: Send CAN message with encoder values
		//Get encoder 2 data
		pckt_size = rc_uart_get_string(buf,7);
		if(!pckt_size){
			issue_warning(WARN_RC_SM_ENC2_DATA_FAIL);
		}
		enc2_count = ((uint32_t)buf[0]<<24)|((uint32_t)buf[1]<<16)|((uint32_t)buf[2]<<8)|((uint32_t)buf[3]);
		//State transition
		rcCurrState = RC_WAIT;						//T_DRV21
		break;
	case SEND_MBATT_REQ:
		//State acton
		run_check_flag = 0;
		wait_cntr = 0;
		pckt_size = checkRCMainBatt(buf);			//Get packet
		rc_uart_send_string(buf, pckt_size);		//Send packet to UART
		//State transition
		rcCurrState = WAIT_MBATT_REQ;				//T_DRV23
		break;
	case WAIT_MBATT_REQ:
		//State action
		wait_cntr++;
		//State transition
		if(is_rc_uart_rx_ndata_ready() == 4){
			rcCurrState = SEND_LBATT_REQ;			//T_DRV27
		} else if(wait_cntr > WAIT_THRESH){
			rcCurrState = ERR_MBATT_REQ;			//T_DRV25
		} else {
			rcCurrState = WAIT_MBATT_REQ;			//T_DRV24
		}
		break;
	case ERR_MBATT_REQ:
		//State action
		issue_warning(WARN_RC_SM_MBATT_PCKT_FAIL);
		//State transition
		rcCurrState = SEND_MBATT_REQ;				//T_DRV26
		break;
	case SEND_LBATT_REQ:
		//State acton
		//TODO: get and check data
		wait_cntr = 0;
		pckt_size = checkRCLogicBatt(buf);			//Get packet
		rc_uart_send_string(buf, pckt_size);		//Send packet to UART
		//State transition
		rcCurrState = WAIT_LBATT_REQ;				//T_DRV23
		break;
	case WAIT_LBATT_REQ:
		//State action
		wait_cntr++;
		//State transition
		if(is_rc_uart_rx_ndata_ready() == 4){
			rcCurrState = SEND_MCUR_REQ;			//T_DRV32
		} else if(wait_cntr > WAIT_THRESH){
			rcCurrState = ERR_LBATT_REQ;			//T_DRV30
		} else {
			rcCurrState = WAIT_LBATT_REQ;			//T_DRV29
		}
		break;
	case ERR_LBATT_REQ:
		//State action
		issue_warning(WARN_RC_SM_LBATT_PCKT_FAIL);
		//State transition
		rcCurrState = SEND_LBATT_REQ;				//T_DRV31
		break;
	case SEND_MCUR_REQ:
		//State acton
		//TODO: Get and check data
		wait_cntr = 0;
		pckt_size = checkRCcurrents(buf);			//Get packet
		rc_uart_send_string(buf, pckt_size);		//Send packet to UART
		//State transition
		rcCurrState = WAIT_MCUR_REQ;				//T_DRV33
		break;
	case WAIT_MCUR_REQ:
		//State action
		wait_cntr++;
		//State transition
		if(is_rc_uart_rx_ndata_ready() == 6){
			rcCurrState = SEND_TEMP_REQ;			//T_DRV37
		} else if(wait_cntr > WAIT_THRESH){
			rcCurrState = ERR_MCUR_REQ;				//T_DRV35
		} else {
			rcCurrState = WAIT_MCUR_REQ;			//T_DRV24
		}
		break;
	case ERR_MCUR_REQ:
		//State action
		issue_warning(WARN_RC_SM_MCUR_PCKT_FAIL);
		//State transition
		rcCurrState = SEND_MCUR_REQ;				//T_DRV36
		break;
	case SEND_TEMP_REQ:
		//State acton
		//TODO: Get and check data
		wait_cntr = 0;
		pckt_size = checkRCtemp(buf);			//Get packet
		rc_uart_send_string(buf, pckt_size);		//Send packet to UART
		//State transition
		rcCurrState = WAIT_TEMP_REQ;				//T_DRV38
		break;
	case WAIT_TEMP_REQ:
		//State action
		wait_cntr++;
		//State transition
		if(is_rc_uart_rx_ndata_ready() == 4){
			rcCurrState = SEND_STAT_REQ;			//T_DRV42
		} else if(wait_cntr > WAIT_THRESH){
			rcCurrState = ERR_TEMP_REQ;				//T_DRV40
		} else {
			rcCurrState = WAIT_TEMP_REQ;			//T_DRV39
		}
		break;
	case ERR_TEMP_REQ:
		//State action
		issue_warning(WARN_RC_SM_TEMP_PCKT_FAIL);
		//State transition
		rcCurrState = SEND_TEMP_REQ;				//T_DRV41
		break;
	case SEND_STAT_REQ:
		//State acton
		//TODO: Get and check data
		wait_cntr = 0;
		pckt_size = checkRCstatus(buf);			//Get packet
		rc_uart_send_string(buf, pckt_size);		//Send packet to UART
		//State transition
		rcCurrState = WAIT_STAT_REQ;				//T_DRV43
		break;
	case WAIT_STAT_REQ:
		//State action
		wait_cntr++;
		//State transition
		if(is_rc_uart_rx_ndata_ready() == 3){
			rcCurrState = CHECK_STAT_REQ;			//T_DRV47
		} else if(wait_cntr > WAIT_THRESH){
			rcCurrState = ERR_STAT_REQ;				//T_DRV45
		} else {
			rcCurrState = WAIT_STAT_REQ;			//T_DRV44
		}
		break;
	case ERR_STAT_REQ:
		//State action
		issue_warning(WARN_RC_SM_STAT_PCKT_FAIL);
		//State transition
		rcCurrState = SEND_STAT_REQ;				//T_DRV46
		break;
	case CHECK_STAT_REQ:
		//State action
		//TODO: Get and check status packet data
		//State transition
		rcCurrState = RC_WAIT;						//T_DRV48
		break;
	default:
		issue_warning(WARN_ILLEGAL_RC_SM_STATE);
		rcCurrState = RC_WAIT;
		break;
	}
}

/** END Roboclaw Task Functions **/

/** Debug Task functions **/

/* Process debug command functions */
void debug_task(void){
	uint8_t debug_cmd_ready = 0;
	uint8_t response_buf[DEBUG_CMD_BUF_SIZE];
	uint8_t response_size = 0;
	//Check if serial data is ready to be formed into a command
	if(is_dbg_uart_rx_data_ready()){
		uint8_t rx_byte = dbg_uart_get_byte();

		//Enter (CR) indicates command is complete
		if(rx_byte == 13){			//Enter pressed (CR)
			dbg_uart_send_byte(13);		//CR
			dbg_uart_send_byte(10);		//Line feed
			debug_cmd_ready = 1;	//Reset command buffer
		} else {//Fill command buffer
			dbg_uart_send_byte(rx_byte);	//Echo back character
			debug_cmd_buf[debug_cmd_buf_ptr] = rx_byte;
			if(debug_cmd_buf_ptr < DEBUG_CMD_BUF_SIZE){
				debug_cmd_buf_ptr++;
			} else {
				issue_warning(WARN_DBG_BUFF_OVERRUN);
			}

		}

	}
	//Process command
	if(debug_cmd_ready){
		if(debug_cmd_buf_ptr == 0){
			//No command, do nothing
		} else if((strncmp(debug_cmd_buf,"led1 on",6)==0) && (debug_cmd_buf_ptr == 7)){
			//>led1 on
			led_P1_0_on();
		} else if((strncmp(debug_cmd_buf,"led1 off",7)==0) && (debug_cmd_buf_ptr == 8)){
			//>led1 off
			led_P1_0_off();
		} else if((strncmp(debug_cmd_buf,"led2 on",6)==0) && (debug_cmd_buf_ptr == 7)){
			//>led2 on
			led_P2_2_on();
		} else if((strncmp(debug_cmd_buf,"led2 off",7)==0) && (debug_cmd_buf_ptr == 8)){
			//>led2 off
			led_P2_2_off();
		} else if((strncmp(debug_cmd_buf,"led3 on",6)==0) && (debug_cmd_buf_ptr == 7)){
			//>led3 on
			led_P7_7_on();
		} else if((strncmp(debug_cmd_buf,"led3 off",7)==0) && (debug_cmd_buf_ptr == 8)){
			//>led3 off
			led_P7_7_off();
		} else if((strncmp(debug_cmd_buf,"led4 on",6)==0) && (debug_cmd_buf_ptr == 7)){
			//>led4 on
			led_P5_7_on();
		} else if((strncmp(debug_cmd_buf,"led4 off",7)==0) && (debug_cmd_buf_ptr == 8)){
			//>led4 off
			led_P5_7_off();
		} else if((strncmp(debug_cmd_buf,"led5 on",6)==0) && (debug_cmd_buf_ptr == 7)){
			//>led5 on
			led_P5_6_on();
		} else if((strncmp(debug_cmd_buf,"led5 off",7)==0) && (debug_cmd_buf_ptr == 8)){
			//>led5 off
			led_P5_6_off();
		} else if((strncmp(debug_cmd_buf,"led6 on",6)==0) && (debug_cmd_buf_ptr == 7)){
			//>led6 on
			led_P4_7_on();
		} else if((strncmp(debug_cmd_buf,"led6 off",7)==0) && (debug_cmd_buf_ptr == 8)){
			//>led6 off
			led_P4_7_off();
/*		} else if((strncmp(debug_cmd_buf,"button get",10)==0) && (debug_cmd_buf_ptr == 10)){
			//>button get
			response_size = button_get(response_buf);
			uart_send_string(response_buf,response_size);
		} else if((strncmp(debug_cmd_buf,"can regread",11)==0) && (debug_cmd_buf_ptr == 16)){
			//>can regread <register in hex>
			//>can regread 0x00
			response_size = debug_mcp2515_read_reg(debug_cmd_buf,response_buf);
			uart_send_string(response_buf,response_size);
		} else if((strncmp(debug_cmd_buf,"can regwrite",12)==0) && (debug_cmd_buf_ptr == 22)){
			//>can regwrite <register in hex> <data in hex>
			//>can regwrite 0x00 0x00
			response_size = debug_mcp2515_write_reg(debug_cmd_buf,response_buf);
			uart_send_string(response_buf,response_size);*/
		} else if((strncmp(debug_cmd_buf,"P1 get",6)==0) && (debug_cmd_buf_ptr == 6)){
			//>P1 get
			response_size = P1_get(response_buf);
			dbg_uart_send_string(response_buf,response_size);
		} else if((strncmp(debug_cmd_buf,"P2 get",6)==0) && (debug_cmd_buf_ptr == 6)){
			//>P2 get
			response_size = P2_get(response_buf);
			dbg_uart_send_string(response_buf,response_size);
		} else if((strncmp(debug_cmd_buf,"P3 get",6)==0) && (debug_cmd_buf_ptr == 6)){
			//>P3 get
			response_size = P3_get(response_buf);
			dbg_uart_send_string(response_buf,response_size);
		} else if((strncmp(debug_cmd_buf,"P4 get",6)==0) && (debug_cmd_buf_ptr == 6)){
			//>P4 get
			response_size = P4_get(response_buf);
			dbg_uart_send_string(response_buf,response_size);
		} else if((strncmp(debug_cmd_buf,"P5 get",6)==0) && (debug_cmd_buf_ptr == 6)){
			//>P5 get
			response_size = P5_get(response_buf);
			dbg_uart_send_string(response_buf,response_size);
		} else if((strncmp(debug_cmd_buf,"P6 get",6)==0) && (debug_cmd_buf_ptr == 6)){
			//>P6 get
			response_size = P6_get(response_buf);
			dbg_uart_send_string(response_buf,response_size);
		} else if((strncmp(debug_cmd_buf,"P7 get",6)==0) && (debug_cmd_buf_ptr == 6)){
			//>P7 get
			response_size = P7_get(response_buf);
			dbg_uart_send_string(response_buf,response_size);
		} else if((strncmp(debug_cmd_buf,"P8 get",6)==0) && (debug_cmd_buf_ptr == 6)){
			//>P8 get
			response_size = P8_get(response_buf);
			dbg_uart_send_string(response_buf,response_size);
		} else if((strncmp(debug_cmd_buf,"drill en",8)==0) && (debug_cmd_buf_ptr == 8)){
			//>drill en
			drill_enable();
		} else if((strncmp(debug_cmd_buf,"drill dis",9)==0) && (debug_cmd_buf_ptr == 9)){
			//>drill dis
			drill_disable();
		} else if((strncmp(debug_cmd_buf,"drill dir ",9)==0) && (debug_cmd_buf_ptr == 11)){
			//>drill dir <direction>
			//>drill dir 0
			if(debug_cmd_buf[10] == '0')
				drill_brake();
			if(debug_cmd_buf[10] == '1')
				drill_cw();
			if(debug_cmd_buf[10] == '2')
				drill_ccw();
		} else if((strncmp(debug_cmd_buf,"step",4)==0) && (debug_cmd_buf_ptr == 4)){
			//>step
			stepper_step_single();
		} else if((strncmp(debug_cmd_buf,"step cw",7)==0) && (debug_cmd_buf_ptr == 7)){
			//>step cw
			stepper_enable(0);
		} else if((strncmp(debug_cmd_buf,"step ccw",8)==0) && (debug_cmd_buf_ptr == 8)){
			//>step ccw
			stepper_enable(1);
		} else if((strncmp(debug_cmd_buf,"step dis",8)==0) && (debug_cmd_buf_ptr == 8)){
			//>step dis
			stepper_disable();
/*		} else if((strncmp(debug_cmd_buf,"can tx",6)==0) && (debug_cmd_buf_ptr == 11)){
			//can tx 0x00
			uint8_t temp = ascii2hex_byte(debug_cmd_buf[9],debug_cmd_buf[10]);
			can_tx_message_buf0(0x279, 1, &temp);
#ifndef PC_CAN
			uart_send_string("tx",2);
#endif
		} else if((strncmp(debug_cmd_buf,"can rx",6)==0) && (debug_cmd_buf_ptr == 6)){
			response_size = debug_can_rx(response_buf);
			uart_send_string(response_buf, response_size);*/
		} else {
			dbg_uart_send_string("Invalid Command",15);
		}
		dbg_uart_send_byte(13);		//CR
		dbg_uart_send_byte(10);		//Line feed
		dbg_uart_send_byte('>');	//Terminal prompt
		debug_cmd_ready = 0;
		debug_cmd_buf_ptr = 0;
	}
}

/** END Debug Task functions **/

/** Interrupts **/
/* Debug UART USCIA0 Interrupt Handler
 * UART Rx: Recieves incoming bytes from debug channel, puts into Debug UART datastructure
 * UART Tx: Sends bytes from Debug UART datastructure to debug channel
 */
#pragma vector=USCI_A0_VECTOR
__interrupt void USCIA0_ISR(void){
	if((UCA0IE & UCRXIE) && (UCA0IFG & UCRXIFG)){	//UART Rxbuf full interrupt
		//Get byte and clear interrupt
		DBG_UART_data.rx_bytes[DBG_UART_data.rx_head] = UCA0RXBUF;
		DBG_UART_data.rx_head++;
		//Wraparound condition
		if(DBG_UART_data.rx_head >= DBG_UART_RX_BUF_SIZE){
			DBG_UART_data.rx_head = 0;
		}
		//if(DBG_UART_data.rx_head == DBG_UART_data.rx_tail)
			//TODO: Log error: buffer full
	} else if((UCA0IE & UCTXIE) && (UCA0IFG & UCTXIFG)){	//UART Txbuf ready interrupt
		//Load data and clear interrupt
		UCA0TXBUF = DBG_UART_data.tx_bytes[DBG_UART_data.tx_tail];
		DBG_UART_data.tx_tail++;
		//Wraparound condition
		if(DBG_UART_data.tx_tail >= DBG_UART_TX_BUF_SIZE){
			DBG_UART_data.tx_tail = 0;
		}
		//Disable Tx interrupt if last byte in buffer has been transmitted
		if(DBG_UART_data.tx_tail == DBG_UART_data.tx_head){
			disable_dbg_uart_txint();
		}
	} else {
		issue_warning(WARN_USCIA0_INT_ILLEGAL_FLAG);
		while(1);
	}
}

/* RoboClaw UART USCIA1 Interrupt Handler
 * UART Rx: Recieves incoming bytes from Roboclaw, puts into RC UART datastructure
 * UART Tx: Sends bytes from RC UART datastructure to roboclaw
 */
#pragma vector=USCI_A1_VECTOR
__interrupt void USCIA1_ISR(void){
	if((UCA1IE & UCRXIE) && (UCA1IFG & UCRXIFG)){	//UART Rxbuf full interrupt
		//Get byte and clear interrupt
		RC_UART_data.rx_bytes[RC_UART_data.rx_head] = UCA1RXBUF;
		RC_UART_data.rx_head++;
		//Wraparound condition
		if(RC_UART_data.rx_head >= RC_UART_RX_BUF_SIZE){
			RC_UART_data.rx_head = 0;
		}
		//if(RC_UART_data.rx_head == RC_UART_data.rx_tail)
			//TODO: Log error: buffer full
	} else if((UCA1IE & UCTXIE) && (UCA1IFG & UCTXIFG)){	//UART Txbuf ready interrupt
		//Load data and clear interrupt
		UCA1TXBUF = RC_UART_data.tx_bytes[RC_UART_data.tx_tail];
		RC_UART_data.tx_tail++;
		//Wraparound condition
		if(RC_UART_data.tx_tail >= RC_UART_TX_BUF_SIZE){
			RC_UART_data.tx_tail = 0;
		}
		//Disable Tx interrupt if last byte in buffer has been transmitted
		if(RC_UART_data.tx_tail == RC_UART_data.tx_head){
			disable_rc_uart_txint();
		}
	} else {
		issue_warning(WARN_USCIA1_INT_ILLEGAL_FLAG);
		while(1);
	}
}


/* System Unmaskable Interrupt Handler
 * NMIIFG:
 * OFIFG: Oscillator Fault
 */
#pragma vector=UNMI_VECTOR
__interrupt void unmi_isr(void){
	switch(__even_in_range(SYSUNIV, 0x08)){
		case 0x00: break;	//No interrupt pending
		case 0x02: // NMIIFG
			issue_warning(WARN_NMI);
			break;
		case 0x04: 			// OFIFG
			if(UCSCTL7 & XT2OFFG){		//XT2 Oscillator fault
				issue_error(ERR_XT2_FAULT);
			}
			if(UCSCTL7 & XT1LFOFFG){	//XT1 Oscillator failt, low frequency mode
				issue_error(ERR_XT1_FAULT);
			}
			if(UCSCTL7 & DCOFFG){		//DCO fault
				issue_error(ERR_DCO_FAULT);
			}
			break;
		case 0x06: // ACCVIFG
			issue_error(ERR_FLASH_VIOL);
			break;
		case 0x08: // BUSIFG
			// If needed, obtain the flash error location here.
			//ErrorLocation = MidGetErrAdr();
			switch(__even_in_range(SYSBERRIV, 0x08)){
				case 0x00: break; // no bus error
				case 0x02: break; // USB bus error
				case 0x04: break; // reserved
				case 0x06: // MID error
					//<place your MID error handler code here>
					break;
				case 0x08: break;
				default: break;
			}
			break;
		default: break;
	}
}

/* Reset Interrupt Handler
 */
#pragma vector=RESET_VECTOR
__interrupt void reset_isr(void){
	switch(SYSRSTIV){
		case SYSRSTIV_NONE:
			break;
		case SYSRSTIV_BOR:
			issue_warning(WARN_RST_BOR);
			break;
		case SYSRSTIV_RSTNMI:
			issue_warning(WARN_RST_RSTNMI);
			break;
		case SYSRSTIV_DOBOR:
			issue_warning(WARN_RST_DOBOR);
			break;
		case SYSRSTIV_LPM5WU:
			issue_warning(WARN_RST_LPM5WU);
			break;
		case SYSRSTIV_SECYV:
			issue_warning(WARN_RST_SECYV);
			break;
		case SYSRSTIV_SVSL:
			issue_warning(WARN_RST_SVSL);
			break;
		case SYSRSTIV_SVSH:
			issue_warning(WARN_RST_SVSH);
			break;
		case SYSRSTIV_SVML_OVP:
			issue_warning(WARN_RST_SVMLOVP);
			break;
		case SYSRSTIV_SVMH_OVP:
			issue_warning(WARN_RST_SVMHOVP);
			break;
		case SYSRSTIV_DOPOR:
			issue_warning(WARN_RST_DOPOR);
			break;
		case SYSRSTIV_WDTTO:
			issue_warning(WARN_RST_WDTTO);
			break;
		case SYSRSTIV_WDTKEY:
			issue_warning(WARN_RST_WDTKEY);
			break;
		case SYSRSTIV_KEYV:
			issue_warning(WARN_RST_KEYV);
			break;
		case SYSRSTIV_FLLUL:
			issue_warning(WARN_RSTFLLUL);
			break;
		case SYSRSTIV_PERF:
			issue_error(ERR_RST_PERF);
			break;
		case SYSRSTIV_PMMKEY:
			issue_warning(WARN_RST_PMM_KEY);
			break;
		default:
			break;
	}
}
/** END Interrupts **/

#define SYSRSTIV_NONE          (0x0000)       /* No Interrupt pending */
#define SYSRSTIV_BOR           (0x0002)       /* SYSRSTIV : BOR */
#define SYSRSTIV_RSTNMI        (0x0004)       /* SYSRSTIV : RST/NMI */
#define SYSRSTIV_DOBOR         (0x0006)       /* SYSRSTIV : Do BOR */
#define SYSRSTIV_LPM5WU        (0x0008)       /* SYSRSTIV : Port LPM5 Wake Up */
#define SYSRSTIV_SECYV         (0x000A)       /* SYSRSTIV : Security violation */
#define SYSRSTIV_SVSL          (0x000C)       /* SYSRSTIV : SVSL */
#define SYSRSTIV_SVSH          (0x000E)       /* SYSRSTIV : SVSH */
#define SYSRSTIV_SVML_OVP      (0x0010)       /* SYSRSTIV : SVML_OVP */
#define SYSRSTIV_SVMH_OVP      (0x0012)       /* SYSRSTIV : SVMH_OVP */
#define SYSRSTIV_DOPOR         (0x0014)       /* SYSRSTIV : Do POR */
#define SYSRSTIV_WDTTO         (0x0016)       /* SYSRSTIV : WDT Time out */
#define SYSRSTIV_WDTKEY        (0x0018)       /* SYSRSTIV : WDTKEY violation */
#define SYSRSTIV_KEYV          (0x001A)       /* SYSRSTIV : Flash Key violation */
#define SYSRSTIV_FLLUL         (0x001C)       /* SYSRSTIV : FLL unlock */
#define SYSRSTIV_PERF          (0x001E)       /* SYSRSTIV : peripheral/config area fetch */
#define SYSRSTIV_PMMKEY        (0x0020)       /* SYSRSTIV : PMMKEY violation */
