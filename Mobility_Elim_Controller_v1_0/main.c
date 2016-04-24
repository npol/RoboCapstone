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
#include "clock_f5.h"
#include "dbg_uart_uscia0.h"
#include "roboclaw.h"
#include "rc_uart_uscia1.h"
#include "drill.h"
#include "stepper.h"

/** Debug task macros and globals **/
void debug_task(void);
void reset_isr(void);

#define DEBUG_CMD_BUF_SIZE 64
#define DEBUG_RESPONSE_BUF_SIZE 250
uint8_t debug_cmd_buf[DEBUG_CMD_BUF_SIZE];
uint8_t debug_cmd_buf_ptr = 0;

//Persistent command codes
#define PCMD_NONE	0x00
#define PCMD_RC_M1 0x01
#define PCMD_RC_M2 0x02
#define PCMD_RC_PID1 0x03
#define PCMD_RC_PID2 0x04
#define PCMD_RC_GS1 0x05
#define PCMD_RC_GS2 0x06
#define PCMD_RC_V1 0x07
#define PCMD_RC_V2 0x08
#define PCMD_RC_V12 0x09
#define PCMD_RC_GET_PID1 0x0A
#define PCMD_RC_GET_PID2 0x0B
/** END Debug task macros and globals **/

/** Warning/Error code buffers and flags **/
volatile uint16_t warn_log[WARN_LOG_SIZE] = {0};
volatile uint8_t warn_log_ptr = 0;
volatile uint8_t warn_flag = 0;
volatile uint16_t err_log[ERR_LOG_SIZE] = {0};
volatile uint8_t err_log_ptr = 0;
volatile uint8_t err_flag = 0;
/** END Warning/Error code buffers and flags **/

/** PC interface task globals **/
void process_cmd(uint8_t *buf, uint8_t buf_size);
void pc_task(void);
void pc_setup(void);
uint8_t check_timeout(void);
void reset_timer(void);

typedef enum  {PC_IDLE,
				PC_INIT_BUF,
				PC_WAIT_SIZE,
				PC_GET_SIZE,
				PC_LOAD_BUF,
				PC_EXEC_CMD,
				PC_WAIT_RC_AVAIL,
				PC_WAIT_CMD,
				PC_END_CMD,
				PC_DEBUG
				} pc_state_t;
volatile pc_state_t pc_current_state = PC_IDLE;

#define PC_CMD_BUF_SIZE 16
#define PC_RESPONSE_BUF_SIZE 64
uint8_t pc_cmd_buf[PC_CMD_BUF_SIZE];
uint8_t pc_cmd_buf_ptr = 0;
uint8_t pc_cmd_size = 0;
uint8_t exit_debug = 0;

#define PC_START_DELIMITER 0x7E
#define PC_DEBUG_DELIMITER '+'
#define ENCODER_DELIMITER 0x55
#define MONITOR_DELIMITER 0xAA
#define MSG_END_DELIMITER 0x33

/** END PC interface task globals **/

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
				CHECK_STAT_REQ,
				SEND_FW_VER,
				WAIT_FW_VER,
				READ_FW_VER,
				SEND_ESTOP,
				WAIT_ESTOP,
				ERR_ESTOP,
				READ_ESTOP} rc_state_t;
volatile rc_state_t rcCurrState = SEND_FW_VER;//RC_WAIT;
#define WAIT_THRESH 50000
#define RC_RUN_CHECK_INTERVAL 500
volatile uint8_t rc_check_ran_once = 0;

#define PCKT_TIMEOUT_THRESH		10
#define PCKT_CRC_ERR_THRESH		10

/* Struct to request asynchronous packet transaction */
#define RC_REQ_SIZE 64
struct RC_async_request_struct{
	uint8_t rc_request_flag;	//Set to request packet transmission, SW must clear
	uint8_t rc_data_ready_flag;	//Set when data is ready to be read
	uint8_t tx_bytes[RC_REQ_SIZE];	//Bytes to send
	uint8_t rx_bytes[RC_REQ_SIZE];	//Bytes recieved
	uint8_t tx_nbytes;				//Number of bytes to send
	uint8_t rx_nbytes;				//Number of bytes to recieve
	uint8_t error;					//flag to indicate transaction error
};
struct RC_async_request_struct RC_async_request = {
	.rc_request_flag = 0,
	.rc_data_ready_flag = 0,
	.tx_bytes = {0},
	.rx_bytes = {0},
	.tx_nbytes = 0,
	.rx_nbytes = 0,
	.error = 0
};
/* Set on TA2 interrupt */
volatile uint8_t timer_TA2_tick = 0;
/* Encoder 1 count */
uint32_t enc1_count = 0;
/* Encoder 2 count */
uint32_t enc2_count = 0;

#define WARN_DENSITY

void roboclaw_task(void);
void monitor_task(void);
void monitor_setup(void);
uint8_t check_rx_crc(uint8_t cmd, uint8_t *rx_data, uint8_t rx_data_size);

/** END Roboclaw task globals **/

/** Monitoring task globals **/
/* Struct to store monitoring data
 * Analog readings: Array contains current value | min value | max value | ready flag
 */
struct monitor_data_struct{
	uint16_t vsense_3V3[4];		//12-bit conversion of 3.3V rail voltage
	uint16_t vsense_5V0[4];		//12-bit conversion of 5V rail voltage
	uint16_t vsense_12V[4];		//12-bit conversion of 12V rail voltage
	uint16_t rc_mbatt[4];		//16-bit conversion of roboclaw main battery voltage
	uint16_t rc_lbatt[4];		//16-bit conversion of roboclaw logic battery voltage
	uint16_t mcu_temp[4];		//12-bit conversion of MSP430 temperature
	uint16_t rc_temp[4];		//16-bit conversion of roboclaw temperature
	uint16_t m1_current[4];		//16-bit conversion of motor 1 current
	uint16_t m2_current[4];		//16-bit conversion of motor 2 current
	uint16_t drill_current[4];	//12-bit conversion of drill current
	uint16_t rc_status;			//16-bit Roboclaw status
	uint8_t estop_status;		//Set when ESTOP condition is active
};
struct monitor_data_struct monitor_data = {
		.vsense_3V3 = {0},
		.vsense_5V0 = {0},
		.vsense_12V = {0},
		.rc_mbatt = {0},
		.rc_lbatt = {0},
		.mcu_temp = {0},
		.rc_temp = {0},
		.m1_current = {0},
		.m2_current = {0},
		.drill_current = {0},
		.rc_status = 0,
		.estop_status = 0
};
//TODO: Update thresholds
#define VSENSE_3V3_MIN 0x0000
#define VSENSE_3V3_MAX 0xFFFF
#define VSENSE_5V0_MIN 0x0000
#define VSENSE_5V0_MAX	0xFFFF
#define VSENSE_12V_MIN	0x0000
#define VSENSE_12V_MAX	0xFFFF
#define RC_MBATT_MIN	0x0000
#define RC_MBATT_MAX	0xFFFF
#define RC_LBATT_MIN	0x0000
#define RC_LBATT_MAX	0xFFFF
#define MCU_TEMP_MIN	0x0000
#define MCU_TEMP_MAX	0xFFFF
#define RC_TEMP_MIN		0x0000
#define RC_TEMP_MAX		0xFFFF
#define M_CURRENT_MIN	0x0000
#define M_CURRENT_MAX	0xFFFF
#define DR_CURRENT_MIN	0x0000
#define DR_CURRENT_MAX	0xFFFF

typedef enum  {MON_WAIT,
				START_ADC1,	//Drill-
				WAIT_ADC1,
				START_ADC2,	//Drill+
				WAIT_ADC2,
				START_ADC8,	//5Vsense
				WAIT_ADC8,
				START_ADC9,	//12Vsense
				WAIT_ADC9,
				START_ADC10,	//Temp
				WAIT_ADC10,
				START_ADC11,//3.3V divider
				WAIT_ADC11,
				SEND_MON_PCKT} mon_state_t;
volatile mon_state_t monCurrState = MON_WAIT;
#define MON_CNT_THRESH 100

//Flag to indicate if safe to operate drill and stepper
volatile uint8_t sys_ok = 0;
#define ESTOP_OVERRIDE

/** END Monioring task globals **/

/** Main Loop **/

int main(void) {
	WDTCTL = WDTPW + WDTHOLD;   // Stop watchdog timer
#ifdef ESTOP_OVERRIDE
	sys_ok = 1;
#else
	sys_ok = 0;
#endif
	P1DIR |= BIT0;	//Debug LED1
	P2DIR |= BIT2;	//Debug LED2
	P7DIR |= BIT7;	//Debug LED3
	P4DIR |= BIT7;	//Debug ERR LED
	P5DIR |= BIT6;	//Debug WARN LED
	P5DIR |= BIT7;	//Debug OK LED
	led_P1_0_off();
	led_P2_2_off();
	led_P7_7_off();
	led_P4_7_off();
	led_P5_6_off();
	led_P5_7_off();
	led_P5_7_on();
	reset_isr();
	setup_clock();
	setup_dbg_uart();
	setup_rc_uart();
	drill_setup();
	stepper_setup();
	roboclaw_setup();
	monitor_setup();
    // Enable Interrupts
    __bis_SR_register(GIE);
    while(1)
    {
        //debug_task();
    	pc_task();
        monitor_task();
        roboclaw_task();
        drill_task();
        stepper_task();
    }
}

/** END Main Loop **/

/** Roboclaw Task Functions **/

void roboclaw_task(void){
	volatile static uint16_t timer_tics = 0;		//Number of TA2 tics to time encoders/check
	volatile static uint8_t run_enc_flag = 0;	//Set when encoders should be checked
	volatile static uint8_t run_check_flag = 0;	//Set when Check routine must run
	volatile static uint16_t wait_cntr = 0;		//counter to prevent hang while waiting for failed packet
	volatile static uint8_t pckt_timeout_cntr = 0;		//Count density of packet timeouts
	volatile static uint8_t pckt_crc_err_cntr = 0;		//Count density of crc errors
	uint8_t pckt_size = 0;				//Size of transmitted packet
	uint8_t buf[32];					//Buffer for transmitted/recieved packet
	uint8_t resp_buf[16];
	uint16_t mbatt, lbatt, m1_curr, m2_curr, rc_temp;
	switch(rcCurrState){
	case RC_WAIT:
		//State action: nothing
		led_P1_0_off();
#ifdef WARN_DENSITY
		if(pckt_timeout_cntr > PCKT_TIMEOUT_THRESH){
			issue_warning(WARN_PCKT_TIMEOUT);
		}
		if(pckt_crc_err_cntr > PCKT_CRC_ERR_THRESH){
			issue_warning(WARN_PCKT_CRC);
		}
#endif
		//State transition
		if(timer_TA2_tick){						//T_DRV2
			timer_TA2_tick = 0;
			rcCurrState = TIME_INTERVALS;
		} else if(run_enc_flag){				//T_DRV1
			rcCurrState = SEND_ENC1_REQ;
		} else if(RC_async_request.rc_request_flag){//T_DRV4
			rcCurrState = SEND_PCKT_REQ;
		} else if(run_check_flag){				//T_DRV22
			rcCurrState = SEND_MBATT_REQ;
			led_P1_0_on();
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
		rc_uart_send_string(RC_async_request.tx_bytes, RC_async_request.tx_nbytes);	//Send packet to UART
		RC_async_request.rc_data_ready_flag = 0;
		RC_async_request.error = 0;
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
		RC_async_request.rc_data_ready_flag = 1;
		//State transition
		rcCurrState = RC_WAIT;						//T_DRV9
		break;
	case PCKT_ERR:
		//State action
		RC_async_request.rc_data_ready_flag = 0;
		RC_async_request.error = 1;
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
#ifdef WARN_DENSITY
			pckt_timeout_cntr++;
#else
			issue_warning(WARN_RC_SM_ENC1_DATA_FAIL);
#endif
		} else if(!check_rx_crc(RC_READ_M1_ENC_COUNT,buf,pckt_size)){
#ifdef WARN_DENSITY
			pckt_crc_err_cntr++;
#else
			issue_warning(WARN_RC_SM_ENC1_CRC_FAIL);
#endif
		} else {
#ifdef WARN_DENSITY
			if(pckt_timeout_cntr != 0){
				pckt_timeout_cntr--;
			}
			if(pckt_crc_err_cntr != 0){
				pckt_crc_err_cntr--;
			}
#endif
			enc1_count = ((uint32_t)buf[0]<<24)|((uint32_t)buf[1]<<16)|((uint32_t)buf[2]<<8)|((uint32_t)buf[3]);
		}
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
		//Get encoder 2 data
		pckt_size = rc_uart_get_string(buf,7);
		if(!pckt_size){
#ifdef WARN_DENSITY
			pckt_timeout_cntr++;
#else
			issue_warning(WARN_RC_SM_ENC2_DATA_FAIL);
#endif
		} else if(!check_rx_crc(RC_READ_M2_ENC_COUNT,buf,pckt_size)){
#ifdef WARN_DENSITY
			pckt_crc_err_cntr++;
#else
			issue_warning(WARN_RC_SM_ENC2_CRC_FAIL);
#endif
		} else {
#ifdef WARN_DENSITY
			if(pckt_timeout_cntr != 0){
				pckt_timeout_cntr--;
			}
			if(pckt_crc_err_cntr != 0){
				pckt_crc_err_cntr--;
			}
#endif
			enc2_count = ((uint32_t)buf[0]<<24)|((uint32_t)buf[1]<<16)|((uint32_t)buf[2]<<8)|((uint32_t)buf[3]);
		}
		//Send encoder data to PC
		if(pc_current_state != PC_DEBUG){
			resp_buf[0] = ENCODER_DELIMITER;
			resp_buf[1] = 8;	//Packet size
			resp_buf[2] = (uint8_t)(enc1_count >> 24);
			resp_buf[3] = (uint8_t)(enc1_count >> 16);
			resp_buf[4] = (uint8_t)(enc1_count >> 8);
			resp_buf[5] = (uint8_t)(enc1_count);
			resp_buf[6] = buf[0];
			resp_buf[7] = buf[1];
			resp_buf[8] = buf[2];
			resp_buf[9] = buf[3];
			resp_buf[10] = MSG_END_DELIMITER;
			dbg_uart_send_string(resp_buf,11);
		}
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
		//Get data from Main Battery request
		pckt_size = rc_uart_get_string(buf,4);
		if(pckt_size == 0){
			issue_warning(WARN_RC_SM_MBATT_DATA_FAIL);
		}
		//Publish main battery voltage
		mbatt = ((uint16_t)buf[0]<<8)|(uint16_t)buf[1];
		monitor_data.rc_mbatt[0] = mbatt;
		if((mbatt < monitor_data.rc_mbatt[1]) || !monitor_data.rc_mbatt[3]){	//New minimum
			monitor_data.rc_mbatt[1] = mbatt;
		}
		if((mbatt > monitor_data.rc_mbatt[2]) || !monitor_data.rc_mbatt[3]){	//New maximum
			monitor_data.rc_mbatt[2] = mbatt;
		}
		monitor_data.rc_mbatt[3] = 1;
		//Send next packet
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
		//Get data from logic battery voltage request
		pckt_size = rc_uart_get_string(buf,4);
		if(pckt_size == 0){
			issue_warning(WARN_RC_SM_LBATT_DATA_FAIL);
		}
		//Publish logic battery voltage
		lbatt = ((uint16_t)buf[0]<<8)|(uint16_t)buf[1];
		monitor_data.rc_lbatt[0] = lbatt;
		if((lbatt < monitor_data.rc_lbatt[1]) || !monitor_data.rc_lbatt[3]){	//New minimum
			monitor_data.rc_lbatt[1] = lbatt;
		}
		if((lbatt > monitor_data.rc_lbatt[2]) || !monitor_data.rc_lbatt[3]){	//New maximum
			monitor_data.rc_lbatt[2] = lbatt;
		}
		monitor_data.rc_lbatt[3] = 1;
		//Send next packet
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
		//Get data from motor current request
		pckt_size = rc_uart_get_string(buf,6);
		if(pckt_size == 0){
			issue_warning(WARN_RC_SM_MCUR_DATA_FAIL);
		}
		//Publish motor 1 current
		m1_curr = ((uint16_t)buf[0]<<8)|(uint16_t)buf[1];
		monitor_data.m1_current[0] = m1_curr;
		if((m1_curr < monitor_data.m1_current[1]) || !monitor_data.m1_current[3]){	//New minimum
			monitor_data.m1_current[1] = m1_curr;
		}
		if((m1_curr > monitor_data.m1_current[2]) || !monitor_data.m1_current[3]){	//New maximum
			monitor_data.m1_current[2] = m1_curr;
		}
		monitor_data.m1_current[3] = 1;
		//Publish motor 2 current
		m2_curr = ((uint16_t)buf[2]<<8)|(uint16_t)buf[3];
		monitor_data.m2_current[0] = m2_curr;
		if((m2_curr < monitor_data.m2_current[1]) || !monitor_data.m2_current[3]){	//New minimum
			monitor_data.m2_current[1] = m2_curr;
		}
		if((m2_curr > monitor_data.m2_current[2]) || !monitor_data.m2_current[3]){	//New maximum
			monitor_data.m2_current[2] = m2_curr;
		}
		monitor_data.m2_current[3] = 1;
		//Send next packet
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
		//Get data from temperature request
		pckt_size = rc_uart_get_string(buf,4);
		if(pckt_size == 0){
			issue_warning(WARN_RC_SM_TEMP_DATA_FAIL);
		}
		//Publish roboclaw temperature
		rc_temp = ((uint16_t)buf[0]<<8)|(uint16_t)buf[1];
		monitor_data.rc_temp[0] = rc_temp;
		if((rc_temp < monitor_data.rc_temp[1]) || !monitor_data.rc_temp[3]){	//New minimum
			monitor_data.rc_temp[1] = rc_temp;
		}
		if((rc_temp > monitor_data.rc_temp[2]) || !monitor_data.rc_temp[3]){	//New maximum
			monitor_data.rc_temp[2] = rc_temp;
		}
		monitor_data.rc_temp[3] = 1;
		//Send next packet
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
		if(is_rc_uart_rx_ndata_ready() == 4){
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
		//Get data from status request
		pckt_size = rc_uart_get_string(buf,4);
		if(pckt_size == 0){
			issue_warning(WARN_RC_SM_STAT_DATA_FAIL);
		}
		//Publish roboclaw status
		monitor_data.rc_status = ((uint16_t)buf[0]<<8)|(uint16_t)buf[1];
		//State transition
		rcCurrState = RC_WAIT;						//T_DRV48
		break;
	case SEND_FW_VER:
		//State action
		wait_cntr = 0;
		pckt_size = getRCFwVer(buf);			//Get packet
		rc_uart_send_string(buf, pckt_size);		//Send packet to UART
		//State transition
		rcCurrState = WAIT_FW_VER;					//T_DRV49
		break;
	case WAIT_FW_VER:
		//State action
		wait_cntr++;
		//State transition
		if(is_rc_uart_rx_ndata_ready() == 30){
			rcCurrState = READ_FW_VER;			//T_DRV51
		} else if(wait_cntr > WAIT_THRESH){
			rcCurrState = SEND_FW_VER;				//T_DRV50
		} else {
			rcCurrState = WAIT_FW_VER;			//T_DRV52
		}
		break;
	case READ_FW_VER:
		//State action
		pckt_size = rc_uart_get_string(buf,30);
		if(pckt_size == 0){
			issue_warning(WARN_RC_FW_VER_DATA_FAIL);
		}
		if(!checkRCFwVer(buf)){
			issue_warning(WARN_RC_BAD_FW);
		}
		//State transition
#ifdef ESTOP_OVERRIDE
		rcCurrState = RC_WAIT;						//T_DRV53
#else
		rcCurrState = SEND_ESTOP;					//T_DRV54
#endif
		break;
	case SEND_ESTOP:
		//State action
		wait_cntr = 0;
		pckt_size = RCenableESTOP(buf);			//Get packet
		rc_uart_send_string(buf, pckt_size);		//Send packet to UART
		//State transition
		rcCurrState = WAIT_ESTOP;
		break;
	case WAIT_ESTOP:
		//State action
		wait_cntr++;
		//State transition
		if(is_rc_uart_rx_ndata_ready() == 1){
			rcCurrState = READ_ESTOP;			//T_DRV59
		} else if(wait_cntr > WAIT_THRESH){
			rcCurrState = ERR_ESTOP;				//T_DRV57
		} else {
			rcCurrState = WAIT_ESTOP;			//T_DRV56
		}
		break;
	case ERR_ESTOP:
		//State action
		issue_warning(WARN_RC_SM_ESTOP_PCKT_FAIL);
		//State transition
		rcCurrState = SEND_ESTOP;
		break;
	case READ_ESTOP:
		//State action/transition
		pckt_size = rc_uart_get_string(buf,1);
		if(pckt_size == 0){
			issue_warning(WARN_RC_SM_ESTOP_DATA_FAIL1);
			rcCurrState = SEND_ESTOP;
		} else if(buf[0] != 0xFF){
			issue_warning(WARN_RC_SM_ESTOP_DATA_FAIL2);
			rcCurrState = SEND_ESTOP;
		} else {
			rcCurrState = RC_WAIT;
		}
		break;
	default:
		issue_warning(WARN_ILLEGAL_RC_SM_STATE);
		rcCurrState = RC_WAIT;
		break;
	}
}

uint8_t check_rx_crc(uint8_t cmd, uint8_t *rx_data, uint8_t rx_data_size){
	uint8_t buf[32];
	uint8_t i;
	uint16_t calc_crc, rx_crc;
	buf[0] = RC_ADDR;
	buf[1] = cmd;
	for(i=0; i < rx_data_size - 2; i++){
		buf[i+2] = rx_data[i];
	}
	calc_crc = crc16(buf,rx_data_size);
	rx_crc = rx_data[rx_data_size-2];
	rx_crc |= rx_data[rx_data_size-1] << 8;
	return calc_crc == rx_crc;
}

/** END Roboclaw Task Functions **/

/** PC interface task functions **/
void pc_setup(void){
	//Initialize timeout timer
	TA1CTL = TASSEL_2 + ID_3 + MC_1; //SMCLK div 8 Up mode
	TA1CCR0 = 31250;	//10ms
}

uint8_t check_timeout(void){
	if(TA1CCTL0 & CCIFG){	//Check interrupt flag
		TA1CCTL0 &= ~CCIFG;	//Clear flag
		return 1;
	}
	return 0;
}

void reset_timer(void){
	TA1CTL |= TACLR;
	TA1CCTL0 &= ~CCIFG;
}

void pc_task(void){
	uint8_t rx_byte = 0;
	volatile static uint8_t debug_delimiter_count = 0;
	switch(pc_current_state){
	case PC_IDLE:
		//State action
		if(is_dbg_uart_rx_data_ready()){
			rx_byte = dbg_uart_get_byte();
			if(rx_byte == PC_DEBUG_DELIMITER){
				debug_delimiter_count++;
			} else {
				debug_delimiter_count = 0;
			}
		}
		//State transition
		if(rx_byte == PC_START_DELIMITER){
			pc_current_state = PC_INIT_BUF;
		} else if(debug_delimiter_count == 3){
			debug_delimiter_count = 0;
			pc_current_state = PC_DEBUG;
		} else {
			pc_current_state = PC_IDLE;
		}
		break;
	case PC_INIT_BUF:
		//State action
		reset_timer();
		pc_cmd_buf_ptr = 0;
		pc_cmd_size = 0;
		//State transition
		pc_current_state = PC_WAIT_SIZE;
		break;
	case PC_WAIT_SIZE:
		//State action: no action
		//State transition
		if(check_timeout()){
			pc_current_state = PC_IDLE;
		} else if(is_dbg_uart_rx_data_ready()){
			reset_timer();
			pc_current_state = PC_GET_SIZE;
		} else {
			pc_current_state = PC_WAIT_SIZE;
		}
		break;
	case PC_GET_SIZE:
		//State action
		pc_cmd_size = dbg_uart_get_byte();	//Get size
		//State transition
		pc_current_state = PC_LOAD_BUF;
	case PC_LOAD_BUF:
		//State action
		if(is_dbg_uart_rx_data_ready()){
			reset_timer();
			pc_cmd_buf[pc_cmd_buf_ptr] = dbg_uart_get_byte();
			pc_cmd_buf_ptr++;
		}
		//State transition
		if(check_timeout()){
			pc_current_state = PC_IDLE;
		} else if((pc_cmd_size == pc_cmd_buf_ptr)){
			pc_current_state = PC_WAIT_RC_AVAIL;
		} else {
			pc_current_state = PC_LOAD_BUF;
		}
		break;
	case PC_WAIT_RC_AVAIL:
		//State action: no action
		//State transition
		if(RC_async_request.rc_request_flag){
			pc_current_state = PC_WAIT_RC_AVAIL;
		} else {
			pc_current_state = PC_EXEC_CMD;
		}
	case PC_EXEC_CMD:
		//State action
		//Send motor velocities
		process_cmd(pc_cmd_buf,pc_cmd_size);
		//State transition
		pc_current_state = PC_WAIT_CMD;
		break;
	case PC_WAIT_CMD:
		//State action: none
		//State transition
		if(RC_async_request.rc_data_ready_flag){
			pc_current_state = PC_END_CMD;
		} else {
			pc_current_state = PC_WAIT_CMD;
		}
		break;
	case PC_END_CMD:
		//State action
		if(RC_async_request.rx_bytes[0] != 0xFF){
			issue_warning(WARN_BAD_MOTOR_PACKET);
		}
		RC_async_request.rc_data_ready_flag = 0;
		RC_async_request.rc_request_flag = 0;
		//State transition
		pc_current_state = PC_IDLE;
		break;
	case PC_DEBUG:
		//State action
		debug_task();
		//State transition
		if(exit_debug){
			exit_debug = 0;
			pc_current_state = PC_IDLE;
		} else {
			pc_current_state = PC_DEBUG;
		}
		break;
	default:
		pc_current_state = PC_IDLE;
		issue_warning(WARN_ILLEGAL_PC_STATE);
		break;
	}
	return;
}

/* Issue commands on message
 *
 */
void process_cmd(uint8_t *buf, uint8_t buf_size){
	uint32_t m1spd;
	uint32_t m2spd;
	switch(buf[0]){
	case 0x30:	//Motor velocities
		m1spd = (uint32_t)buf[1] << 24;
		m1spd |= (uint32_t)buf[2] <<16;
		m1spd |= (uint32_t)buf[3] << 8;
		m1spd |= (uint32_t)buf[4];
		m2spd = (uint32_t)buf[5] << 24;
		m2spd |= (uint32_t)buf[6] <<16;
		m2spd |= (uint32_t)buf[7] << 8;
		m2spd |= (uint32_t)buf[8];
		RC_async_request.tx_nbytes = driveM12Speed(m1spd,m2spd,RC_async_request.tx_bytes);
		//driveM12SpeedAccel(m1spd, m2spd, ACCEL_LIMIT, RC_async_request.tx_bytes);
		RC_async_request.rx_nbytes = 1;
		RC_async_request.rc_request_flag = 1;
		break;
	}
}

/** END PC interface task functions **/

/** Monitor Task Functions **/

/* Setup monitoring task */
void monitor_setup(void){
	ADC12CTL0 = ADC12SHT1_12 +	//1024 ADCLK cycles for sampling
				ADC12SHT0_12 +
				ADC12REF2_5V +	//2.5V reference
				ADC12REFON +	//Reference on
				ADC12ON;		//ADC on
	ADC12CTL1 = ADC12DIV_7 +	//clock divider: 8
				ADC12SSEL_2 +	//clock source: MCLK
				ADC12SHP;		//Use sampling timer
	return;
}

void monitor_task(void){
	static uint8_t run_mon_cnt = 0;	//Set when Check routine must run
	static uint16_t drill_vzcr = 0;	//Negative reference for drill current
	uint8_t buf[8];
	uint16_t conversion;
	//Check ESTOP
	if(P1IN & BIT4){
		monitor_data.estop_status = 1;
#ifndef ESTOP_OVERRIDE
		sys_ok = 1;
#endif
	} else {
		monitor_data.estop_status = 0;
#ifndef ESTOP_OVERRIDE
		sys_ok = 0;
#endif
	}
	switch(monCurrState){
	case MON_WAIT:
		//State action
		run_mon_cnt++;
		//State transition
		if(run_mon_cnt > MON_CNT_THRESH){
			run_mon_cnt = 0;
			monCurrState = START_ADC1;	//T_MON0
		} else {
			monCurrState = MON_WAIT;	//T_MON1
		}
		break;
	case START_ADC1:
		//State action
		ADC12CTL0 &= ~ADC12ENC;			//Disable conversions to change channel
		ADC12MCTL0 = ADC12INCH_1;		//A1
		ADC12CTL0 |= ADC12SC+ADC12ENC;	//Start conversion
		//State transition
		monCurrState = WAIT_ADC1;		//T_MON2
		break;
	case WAIT_ADC1:
		//State action: nothing
		//State transition
		if(ADC12IFG & ADC12IFG0){
			monCurrState = START_ADC2;	//T_MON4
		} else {
			monCurrState = WAIT_ADC1;	//T_MON3
		}
		break;
	case START_ADC2:
		//State action
		drill_vzcr = ADC12MEM0;			//Get drill current reference
		ADC12CTL0 &= ~ADC12ENC;			//Disable conversions to change channel
		ADC12MCTL0 = ADC12INCH_2;		//A2
		ADC12CTL0 |= ADC12SC+ADC12ENC;	//Start conversion
		//State transition
		monCurrState = WAIT_ADC2;		//T_MON5
		break;
	case WAIT_ADC2:
		//State action: nothing
		//State transition
		if(ADC12IFG & ADC12IFG0){
			monCurrState = START_ADC8;	//T_MON7
		} else {
			monCurrState = WAIT_ADC2;	//T_MON6
		}
		break;
	case START_ADC8:
		//State action
		conversion = ADC12MEM0 - drill_vzcr;	//Get drill current
		monitor_data.drill_current[0] = conversion;
		if((conversion < monitor_data.drill_current[1]) || !monitor_data.drill_current[3]){
			monitor_data.drill_current[1] = conversion;
		}
		if((conversion > monitor_data.drill_current[2]) || !monitor_data.drill_current[3]){
			monitor_data.drill_current[2] = conversion;
		}
		monitor_data.drill_current[3] = 1;
		ADC12CTL0 &= ~ADC12ENC;			//Disable conversions to change channel
		ADC12MCTL0 = ADC12INCH_8;		//A8
		ADC12CTL0 |= ADC12SC+ADC12ENC;	//Start conversion
		//State transition
		monCurrState = WAIT_ADC8;		//T_MON8
		break;
	case WAIT_ADC8:
		//State action: nothing
		//State transition
		if(ADC12IFG & ADC12IFG0){
			monCurrState = START_ADC9;	//T_MON10
		} else {
			monCurrState = WAIT_ADC8;	//T_MON9
		}
		break;
	case START_ADC9:
		//State action
		conversion = ADC12MEM0;	//Get 5V sense voltage
		monitor_data.vsense_5V0[0] = conversion;
		if((conversion < monitor_data.vsense_5V0[1]) || !monitor_data.vsense_5V0[3]){
			monitor_data.vsense_5V0[1] = conversion;
		}
		if((conversion > monitor_data.vsense_5V0[2]) || !monitor_data.vsense_5V0[3]){
			monitor_data.vsense_5V0[2] = conversion;
		}
		monitor_data.vsense_5V0[3] = 1;
		ADC12CTL0 &= ~ADC12ENC;			//Disable conversions to change channel
		ADC12MCTL0 = ADC12INCH_9;		//A9
		ADC12CTL0 |= ADC12SC+ADC12ENC;	//Start conversion
		//State transition
		monCurrState = WAIT_ADC9;		//T_MON11
		break;
	case WAIT_ADC9:
		//State action: nothing
		//State transition
		if(ADC12IFG & ADC12IFG0){
			monCurrState = START_ADC10;	//T_MON13
		} else {
			monCurrState = WAIT_ADC9;	//T_MON12
		}
		break;
	case START_ADC10:
		//State action
		conversion = ADC12MEM0;	//Get 12V sense voltage
		monitor_data.vsense_12V[0] = conversion;
		if((conversion < monitor_data.vsense_12V[1]) || !monitor_data.vsense_12V[3]){
			monitor_data.vsense_12V[1] = conversion;
		}
		if((conversion > monitor_data.vsense_12V[2]) || !monitor_data.vsense_12V[3]){
			monitor_data.vsense_12V[2] = conversion;
		}
		monitor_data.vsense_12V[3] = 1;
		ADC12CTL0 &= ~ADC12ENC;			//Disable conversions to change channel
		while(REFCTL0 & REFGENBUSY);
		REFCTL0 = REFMSTR + REFVSEL_0 + REFON;
		ADC12CTL0 &= ~ADC12REF2_5V;		//1.5V reference
		ADC12MCTL0 = ADC12INCH_10 + ADC12SREF_1;		//A10, REF+
		ADC12CTL0 |= ADC12SC+ADC12ENC;	//Start conversion
		//State transition
		monCurrState = WAIT_ADC10;		//T_MON14
		break;
	case WAIT_ADC10:
		//State action: nothing
		//State transition
		if(ADC12IFG & ADC12IFG0){
			monCurrState = START_ADC11;	//T_MON16
		} else {
			monCurrState = WAIT_ADC10;	//T_MON15
		}
		break;
	case START_ADC11:
		//State action
		conversion = ADC12MEM0;	//Get Temp voltage
		monitor_data.mcu_temp[0] = conversion;
		if((conversion < monitor_data.mcu_temp[1]) || !monitor_data.mcu_temp[3]){
			monitor_data.mcu_temp[1] = conversion;
		}
		if((conversion > monitor_data.mcu_temp[2]) || !monitor_data.mcu_temp[3]){
			monitor_data.mcu_temp[2] = conversion;
		}
		monitor_data.mcu_temp[3] = 1;
		ADC12CTL0 &= ~ADC12ENC;			//Disable conversions to change channel
		while(REFCTL0 & REFGENBUSY);
		REFCTL0 = REFMSTR + REFVSEL_2 + REFON;
		ADC12CTL0 |= ADC12REF2_5V;		//2.5V reference
		ADC12MCTL0 = ADC12INCH_11 + ADC12SREF_1;		//A11, VREF+
		ADC12CTL0 |= ADC12SC+ADC12ENC;	//Start conversion
		//State transition
		monCurrState = WAIT_ADC11;		//T_MON17
		break;
	case WAIT_ADC11:
		//State action: nothing
		//State transition
		if(ADC12IFG & ADC12IFG0){
			monCurrState = SEND_MON_PCKT;//T_MON19
		} else {
			monCurrState = WAIT_ADC11;	//T_MON18
		}
		break;
	case SEND_MON_PCKT:
		//State action
		conversion = ADC12MEM0;	//Get 3.3V sense voltage
		monitor_data.vsense_3V3[0] = conversion;
		if((conversion < monitor_data.vsense_3V3[1]) || !monitor_data.vsense_3V3[3]){
			monitor_data.vsense_3V3[1] = conversion;
		}
		if((conversion > monitor_data.vsense_3V3[2]) || !monitor_data.vsense_3V3[3]){
			monitor_data.vsense_3V3[2] = conversion;
		}
		monitor_data.vsense_3V3[3] = 1;
		//Check parameters.  All values valid since to reach this state, all conversions must be completed.
		if(monitor_data.vsense_3V3[0] < VSENSE_3V3_MIN) issue_warning(WARN_LOW_3V3);
		if(monitor_data.vsense_3V3[0] > VSENSE_3V3_MAX) issue_warning(WARN_HIGH_3V3);
		if(monitor_data.vsense_5V0[0] < VSENSE_5V0_MIN) issue_warning(WARN_LOW_5V0);
		if(monitor_data.vsense_5V0[0] > VSENSE_5V0_MAX) issue_warning(WARN_HIGH_5V0);
		if(monitor_data.vsense_12V[0] < VSENSE_12V_MIN) issue_warning(WARN_LOW_12V);
		if(monitor_data.vsense_12V[0] > VSENSE_12V_MAX) issue_warning(WARN_HIGH_12V);
		if(monitor_data.mcu_temp[0] < MCU_TEMP_MIN) issue_warning(WARN_LOW_MCU_TEMP);
		if(monitor_data.mcu_temp[0] > MCU_TEMP_MAX) issue_warning(WARN_HIGH_MCU_TEMP);
		if(monitor_data.drill_current[0] < DR_CURRENT_MIN) issue_warning(WARN_LOW_DRILL_CURRENT);
		if(monitor_data.drill_current[0] > DR_CURRENT_MAX) issue_warning(WARN_HIGH_DRILL_CURRENT);
		if(monitor_data.estop_status) issue_warning(ESTOP_ACTIVATED);
		//These parameters are gathered by other tasks, so need to know if ran at least once.
		if(rc_check_ran_once){
			if(monitor_data.rc_mbatt[0] < RC_MBATT_MIN) issue_warning(WARN_LOW_RC_MBATT);
			if(monitor_data.rc_mbatt[0] > RC_MBATT_MAX) issue_warning(WARN_HIGH_RC_MBATT);
			if(monitor_data.rc_lbatt[0] < RC_LBATT_MIN) issue_warning(WARN_LOW_RC_LBATT);
			if(monitor_data.rc_lbatt[0] > RC_LBATT_MAX) issue_warning(WARN_HIGH_RC_LBATT);
			if(monitor_data.m1_current[0] < M_CURRENT_MIN) issue_warning(WARN_LOW_M1_CURRENT);
			if(monitor_data.m1_current[0] > M_CURRENT_MAX) issue_warning(WARN_HIGH_M1_CURRENT);
			if(monitor_data.m2_current[0] < M_CURRENT_MIN) issue_warning(WARN_LOW_M2_CURRENT);
			if(monitor_data.m2_current[0] > M_CURRENT_MAX) issue_warning(WARN_HIGH_M2_CURRENT);
			if(monitor_data.rc_temp[0] < RC_TEMP_MIN) issue_warning(WARN_LOW_RC_TEMP);
			if(monitor_data.rc_temp[0] > RC_TEMP_MAX) issue_warning(WARN_HIGH_RC_TEMP);
			if(monitor_data.rc_status & RC_STAT_M1_OVERCURRENT) issue_warning(WARN_RC_M1_OVERCURRENT);
			if(monitor_data.rc_status & RC_STAT_M2_OVERCURRENT) issue_warning(WARN_RC_M2_OVERCURRENT);
			if(monitor_data.rc_status & RC_STAT_ESTOP) issue_warning(WARN_RC_ESTOP);
			if(monitor_data.rc_status & RC_STAT_TEMP_ERR) issue_warning(WARN_RC_TEMP_ERR);
			if(monitor_data.rc_status & RC_STAT_TEMP2_ERR) issue_warning(WARN_RC_TEMP2_ERR);
			if(monitor_data.rc_status & RC_STAT_MBATT_H_ERR) issue_warning(WARN_RC_MBATT_H_ERR);
			if(monitor_data.rc_status & RC_STAT_LBATT_H_ERR) issue_warning(WARN_RC_LBATT_H_ERR);
			if(monitor_data.rc_status & RC_STAT_LBATT_L_ERR) issue_warning(WARN_RC_LBATT_L_ERR);
			if(monitor_data.rc_status & RC_STAT_M1_FAULT) issue_warning(WARN_RC_M1_FAULT);
			if(monitor_data.rc_status & RC_STAT_M2_FAULT) issue_warning(WARN_RC_M2_FAULT);
			if(monitor_data.rc_status & RC_STAT_MBATT_H_WARN) issue_warning(WARN_RC_MBATT_H_WARN);
			if(monitor_data.rc_status & RC_STAT_MBATT_L_WARN) issue_warning(WARN_RC_MBATT_L_WARN);
			if(monitor_data.rc_status & RC_STAT_TEMP_WARN) issue_warning(WARN_RC_TEMP_WARN);
			if(monitor_data.rc_status & RC_STAT_TEMP2_WARN) issue_warning(WARN_RC_TEMP2_WARN);
		}
		//Send packet to PC
		buf[0] = MONITOR_DELIMITER;
		buf[1] = 2;
		buf[2] = ((monitor_data.vsense_12V[0]>>8)&0x7F) | (monitor_data.estop_status << 8);
		buf[3] = (monitor_data.vsense_12V[0]&0xFF);
		buf[4] = MSG_END_DELIMITER;
		dbg_uart_send_string(buf,5);
		//State transition
		monCurrState = MON_WAIT;		//T_MON20
		break;
	default:
		issue_warning(WARN_ILLEGAL_MON_SM_STATE);
		monCurrState = MON_WAIT;
		break;
	}
}

/** END Monitor Task Functions **/

/** Debug Task functions **/

/* Process debug command functions */
void debug_task(void){
	static uint8_t persistent_cmd = 0;	//Code of command to continue running while other tasks run
	static uint8_t pcmd_data0 = 0;		//Storage for persistent commands
	static uint8_t pcmd_data1 = 0;
	static uint32_t pcmd_data_long[8] = {0};
	uint8_t debug_cmd_ready = 0;
	uint8_t response_buf[DEBUG_RESPONSE_BUF_SIZE];
	uint8_t response_size = 0;
	//Check if serial data is ready to be formed into a command
	if(is_dbg_uart_rx_data_ready() && !persistent_cmd){
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
	//If command needs to continue running as other tasks run, do actions here
	if(persistent_cmd){
		switch(persistent_cmd){
		case PCMD_RC_M1:
			if(!pcmd_data1){	//Roboclaw Command has not been sent yet
				if(RC_async_request.rc_request_flag){	//Wait for open request buffer
					break;
				} else {	//Buffer is open
					RC_async_request.tx_nbytes = driveM1Power(pcmd_data0,RC_async_request.tx_bytes);
					RC_async_request.rx_nbytes = 1;
					RC_async_request.rc_request_flag = 1;
					pcmd_data1 = 1;	//Flag to indicate command has been sent
					break;
				}
			} else if(RC_async_request.rc_data_ready_flag){
				//Check recieved byte
				if(RC_async_request.error){	//Packet timeout
					response_buf[0] = 'e';
					response_buf[1] = 'r';
					response_buf[2] = 'r';
					response_size = 3;
					dbg_uart_send_string(response_buf,response_size);
				} else	if(RC_async_request.rx_bytes[0] == 0xFF){	//Successful packet transmission
					response_buf[0] = 'a';
					response_buf[1] = 'c';
					response_buf[2] = 'k';
					response_size = 3;
					dbg_uart_send_string(response_buf,response_size);
				} else {	//Bad data
					response_buf[0] = 'b';
					response_buf[1] = 'a';
					response_buf[2] = 'd';
					response_size = 3;
					dbg_uart_send_string(response_buf,response_size);
				}
				//Release request datastructure
				RC_async_request.rc_data_ready_flag = 0;
				RC_async_request.rc_request_flag = 0;
				persistent_cmd = PCMD_NONE;
				pcmd_data1 = 0;	//clear data stores
				pcmd_data0 = 0;
			} //Else wait for data to be ready
			break;
		case PCMD_RC_M2:
			if(!pcmd_data1){	//Roboclaw Command has not been sent yet
				if(RC_async_request.rc_request_flag){	//Wait for open request buffer
					break;
				} else {	//Buffer is open
					RC_async_request.tx_nbytes = driveM2Power(pcmd_data0,RC_async_request.tx_bytes);
					RC_async_request.rx_nbytes = 1;
					RC_async_request.rc_request_flag = 1;
					pcmd_data1 = 1;	//Flag to indicate command has been sent
					break;
				}
			} else if(RC_async_request.rc_data_ready_flag){
				//Check recieved byte
				if(RC_async_request.error){	//Packet timeout
					response_buf[0] = 'e';
					response_buf[1] = 'r';
					response_buf[2] = 'r';
					response_size = 3;
					dbg_uart_send_string(response_buf,response_size);
				} else	if(RC_async_request.rx_bytes[0] == 0xFF){	//Successful packet transmission
					response_buf[0] = 'a';
					response_buf[1] = 'c';
					response_buf[2] = 'k';
					response_size = 3;
					dbg_uart_send_string(response_buf,response_size);
				} else {	//Bad data
					response_buf[0] = 'b';
					response_buf[1] = 'a';
					response_buf[2] = 'd';
					response_size = 3;
					dbg_uart_send_string(response_buf,response_size);
				}
				//Release request datastructure
				RC_async_request.rc_data_ready_flag = 0;
				RC_async_request.rc_request_flag = 0;
				persistent_cmd = PCMD_NONE;
				pcmd_data1 = 0;	//clear data stores
				pcmd_data0 = 0;
			} //Else wait for data to be ready
			break;
		case PCMD_RC_PID1:
			if(!pcmd_data1){	//Roboclaw Command has not been sent yet
				if(RC_async_request.rc_request_flag){	//Wait for open request buffer
					break;
				} else {	//Buffer is open
					RC_async_request.tx_nbytes = setM1PID(pcmd_data_long,RC_async_request.tx_bytes);
					RC_async_request.rx_nbytes = 1;
					RC_async_request.rc_request_flag = 1;
					pcmd_data1 = 1;	//Flag to indicate command has been sent
					break;
				}
			} else if(RC_async_request.rc_data_ready_flag){
				//Check recieved byte
				if(RC_async_request.error){	//Packet timeout
					response_buf[0] = 'e';
					response_buf[1] = 'r';
					response_buf[2] = 'r';
					response_size = 3;
					dbg_uart_send_string(response_buf,response_size);
				} else	if(RC_async_request.rx_bytes[0] == 0xFF){	//Successful packet transmission
					response_buf[0] = 'a';
					response_buf[1] = 'c';
					response_buf[2] = 'k';
					response_size = 3;
					dbg_uart_send_string(response_buf,response_size);
				} else {	//Bad data
					response_buf[0] = 'b';
					response_buf[1] = 'a';
					response_buf[2] = 'd';
					response_size = 3;
					dbg_uart_send_string(response_buf,response_size);
				}
				//Release request datastructure
				RC_async_request.rc_data_ready_flag = 0;
				RC_async_request.rc_request_flag = 0;
				persistent_cmd = PCMD_NONE;
				pcmd_data1 = 0;	//clear data stores
				pcmd_data0 = 0;
			} //Else wait for data to be ready
			break;
		case PCMD_RC_PID2:
			if(!pcmd_data1){	//Roboclaw Command has not been sent yet
				if(RC_async_request.rc_request_flag){	//Wait for open request buffer
					break;
				} else {	//Buffer is open
					RC_async_request.tx_nbytes = setM2PID(pcmd_data_long,RC_async_request.tx_bytes);
					RC_async_request.rx_nbytes = 1;
					RC_async_request.rc_request_flag = 1;
					pcmd_data1 = 1;	//Flag to indicate command has been sent
					break;
				}
			} else if(RC_async_request.rc_data_ready_flag){
				//Check recieved byte
				if(RC_async_request.error){	//Packet timeout
					response_buf[0] = 'e';
					response_buf[1] = 'r';
					response_buf[2] = 'r';
					response_size = 3;
					dbg_uart_send_string(response_buf,response_size);
				} else	if(RC_async_request.rx_bytes[0] == 0xFF){	//Successful packet transmission
					response_buf[0] = 'a';
					response_buf[1] = 'c';
					response_buf[2] = 'k';
					response_size = 3;
					dbg_uart_send_string(response_buf,response_size);
				} else {	//Bad data
					response_buf[0] = 'b';
					response_buf[1] = 'a';
					response_buf[2] = 'd';
					response_size = 3;
					dbg_uart_send_string(response_buf,response_size);
				}
				//Release request datastructure
				RC_async_request.rc_data_ready_flag = 0;
				RC_async_request.rc_request_flag = 0;
				persistent_cmd = PCMD_NONE;
				pcmd_data1 = 0;	//clear data stores
				pcmd_data0 = 0;
			} //Else wait for data to be ready
			break;
		case PCMD_RC_GS1:
			if(!pcmd_data1){	//Roboclaw Command has not been sent yet
				if(RC_async_request.rc_request_flag){	//Wait for open request buffer
					break;
				} else {	//Buffer is open
					RC_async_request.tx_nbytes = RCgetEnc1Speed(RC_async_request.tx_bytes);
					RC_async_request.rx_nbytes = 7;
					RC_async_request.rc_request_flag = 1;
					pcmd_data1 = 1;	//Flag to indicate command has been sent
					break;
				}
			} else if(RC_async_request.rc_data_ready_flag){
				//Check recieved byte
				if(RC_async_request.error){	//Packet timeout
					response_buf[0] = 'e';
					response_buf[1] = 'r';
					response_buf[2] = 'r';
					response_size = 3;
					dbg_uart_send_string(response_buf,response_size);
				} else	{	//Successful packet transmission
					if(RC_async_request.rx_bytes[4]){
						response_buf[0] = '-';
					} else {
						response_buf[0] = '+';
					}
					uint32_t val = ((uint32_t)RC_async_request.rx_bytes[0])<<24;
					val |= ((uint32_t)RC_async_request.rx_bytes[1])<<16;
					val |= ((uint32_t)RC_async_request.rx_bytes[2])<<8;
					val |= ((uint32_t)RC_async_request.rx_bytes[3]);
					hex2ascii_long(val,&response_buf[1],&response_buf[2],&response_buf[3],&response_buf[4],&response_buf[5],&response_buf[6],&response_buf[7],&response_buf[8]);
					response_size = 9;
					dbg_uart_send_string(response_buf,response_size);
				}
				//Release request datastructure
				RC_async_request.rc_data_ready_flag = 0;
				RC_async_request.rc_request_flag = 0;
				persistent_cmd = PCMD_NONE;
				pcmd_data1 = 0;	//clear data stores
				pcmd_data0 = 0;
			} //Else wait for data to be ready
			break;
		case PCMD_RC_GS2:
			if(!pcmd_data1){	//Roboclaw Command has not been sent yet
				if(RC_async_request.rc_request_flag){	//Wait for open request buffer
					break;
				} else {	//Buffer is open
					RC_async_request.tx_nbytes = RCgetEnc2Speed(RC_async_request.tx_bytes);
					RC_async_request.rx_nbytes = 7;
					RC_async_request.rc_request_flag = 1;
					pcmd_data1 = 1;	//Flag to indicate command has been sent
					break;
				}
			} else if(RC_async_request.rc_data_ready_flag){
				//Check recieved byte
				if(RC_async_request.error){	//Packet timeout
					response_buf[0] = 'e';
					response_buf[1] = 'r';
					response_buf[2] = 'r';
					response_size = 3;
					dbg_uart_send_string(response_buf,response_size);
				} else	{	//Successful packet transmission
					if(RC_async_request.rx_bytes[4]){
						response_buf[0] = '-';
					} else {
						response_buf[0] = '+';
					}
					uint32_t val = ((uint32_t)RC_async_request.rx_bytes[0])<<24;
					val |= ((uint32_t)RC_async_request.rx_bytes[1])<<16;
					val |= ((uint32_t)RC_async_request.rx_bytes[2])<<8;
					val |= ((uint32_t)RC_async_request.rx_bytes[3]);
					hex2ascii_long(val,&response_buf[1],&response_buf[2],&response_buf[3],&response_buf[4],&response_buf[5],&response_buf[6],&response_buf[7],&response_buf[8]);
					response_size = 9;
					dbg_uart_send_string(response_buf,response_size);
				}
				//Release request datastructure
				RC_async_request.rc_data_ready_flag = 0;
				RC_async_request.rc_request_flag = 0;
				persistent_cmd = PCMD_NONE;
				pcmd_data1 = 0;	//clear data stores
				pcmd_data0 = 0;
			} //Else wait for data to be ready
			break;
		case PCMD_RC_V1:
			if(!pcmd_data1){	//Roboclaw Command has not been sent yet
				if(RC_async_request.rc_request_flag){	//Wait for open request buffer
					break;
				} else {	//Buffer is open
#ifdef ACCEL
					RC_async_request.tx_nbytes = driveM1SpeedAccel(pcmd_data_long[0],ACCEL_LIMIT, RC_async_request.tx_bytes);
#else
					RC_async_request.tx_nbytes = driveM1Speed(pcmd_data_long[0],RC_async_request.tx_bytes);
#endif
					RC_async_request.rx_nbytes = 1;
					RC_async_request.rc_request_flag = 1;
					pcmd_data1 = 1;	//Flag to indicate command has been sent
					break;
				}
			} else if(RC_async_request.rc_data_ready_flag){
				//Check recieved byte
				if(RC_async_request.error){	//Packet timeout
					response_buf[0] = 'e';
					response_buf[1] = 'r';
					response_buf[2] = 'r';
					response_size = 3;
					dbg_uart_send_string(response_buf,response_size);
				} else	if(RC_async_request.rx_bytes[0] == 0xFF){	//Successful packet transmission
					response_buf[0] = 'a';
					response_buf[1] = 'c';
					response_buf[2] = 'k';
					response_size = 3;
					dbg_uart_send_string(response_buf,response_size);
				} else {	//Bad data
					response_buf[0] = 'b';
					response_buf[1] = 'a';
					response_buf[2] = 'd';
					response_size = 3;
					dbg_uart_send_string(response_buf,response_size);
				}
				//Release request datastructure
				RC_async_request.rc_data_ready_flag = 0;
				RC_async_request.rc_request_flag = 0;
				persistent_cmd = PCMD_NONE;
				pcmd_data1 = 0;	//clear data stores
				pcmd_data0 = 0;
			} //Else wait for data to be ready
			break;
		case PCMD_RC_V2:
			if(!pcmd_data1){	//Roboclaw Command has not been sent yet
				if(RC_async_request.rc_request_flag){	//Wait for open request buffer
					break;
				} else {	//Buffer is open
#ifdef ACCEL
					RC_async_request.tx_nbytes = driveM2SpeedAccel(pcmd_data_long[0],ACCEL_LIMIT, RC_async_request.tx_bytes);
#else
					RC_async_request.tx_nbytes = driveM2Speed(pcmd_data_long[0],RC_async_request.tx_bytes);
#endif
					RC_async_request.rx_nbytes = 1;
					RC_async_request.rc_request_flag = 1;
					pcmd_data1 = 1;	//Flag to indicate command has been sent
					break;
				}
			} else if(RC_async_request.rc_data_ready_flag){
				//Check recieved byte
				if(RC_async_request.error){	//Packet timeout
					response_buf[0] = 'e';
					response_buf[1] = 'r';
					response_buf[2] = 'r';
					response_size = 3;
					dbg_uart_send_string(response_buf,response_size);
				} else	if(RC_async_request.rx_bytes[0] == 0xFF){	//Successful packet transmission
					response_buf[0] = 'a';
					response_buf[1] = 'c';
					response_buf[2] = 'k';
					response_size = 3;
					dbg_uart_send_string(response_buf,response_size);
				} else {	//Bad data
					response_buf[0] = 'b';
					response_buf[1] = 'a';
					response_buf[2] = 'd';
					response_size = 3;
					dbg_uart_send_string(response_buf,response_size);
				}
				//Release request datastructure
				RC_async_request.rc_data_ready_flag = 0;
				RC_async_request.rc_request_flag = 0;
				persistent_cmd = PCMD_NONE;
				pcmd_data1 = 0;	//clear data stores
				pcmd_data0 = 0;
			} //Else wait for data to be ready
			break;
		case PCMD_RC_V12:
			if(!pcmd_data1){	//Roboclaw Command has not been sent yet
				if(RC_async_request.rc_request_flag){	//Wait for open request buffer
					break;
				} else {	//Buffer is open
					RC_async_request.tx_nbytes = driveM12Speed(pcmd_data_long[0],pcmd_data_long[1],RC_async_request.tx_bytes);
					RC_async_request.rx_nbytes = 1;
					RC_async_request.rc_request_flag = 1;
					pcmd_data1 = 1;	//Flag to indicate command has been sent
					break;
				}
			} else if(RC_async_request.rc_data_ready_flag){
				//Check recieved byte
				if(RC_async_request.error){	//Packet timeout
					response_buf[0] = 'e';
					response_buf[1] = 'r';
					response_buf[2] = 'r';
					response_size = 3;
					dbg_uart_send_string(response_buf,response_size);
				} else	if(RC_async_request.rx_bytes[0] == 0xFF){	//Successful packet transmission
					response_buf[0] = 'a';
					response_buf[1] = 'c';
					response_buf[2] = 'k';
					response_size = 3;
					dbg_uart_send_string(response_buf,response_size);
				} else {	//Bad data
					response_buf[0] = 'b';
					response_buf[1] = 'a';
					response_buf[2] = 'd';
					response_size = 3;
					dbg_uart_send_string(response_buf,response_size);
				}
				//Release request datastructure
				RC_async_request.rc_data_ready_flag = 0;
				RC_async_request.rc_request_flag = 0;
				persistent_cmd = PCMD_NONE;
				pcmd_data1 = 0;	//clear data stores
				pcmd_data0 = 0;
			} //Else wait for data to be ready
			break;
		case PCMD_RC_GET_PID1:
			if(!pcmd_data1){	//Roboclaw Command has not been sent yet
				if(RC_async_request.rc_request_flag){	//Wait for open request buffer
					break;
				} else {	//Buffer is open
					RC_async_request.tx_nbytes = getM1PID(RC_async_request.tx_bytes);
					RC_async_request.rx_nbytes = 18;
					RC_async_request.rc_request_flag = 1;
					pcmd_data1 = 1;	//Flag to indicate command has been sent
					break;
				}
			} else if(RC_async_request.rc_data_ready_flag){
				//Check recieved byte
				if(RC_async_request.error){	//Packet timeout
					response_buf[0] = 'e';
					response_buf[1] = 'r';
					response_buf[2] = 'r';
					response_size = 3;
					dbg_uart_send_string(response_buf,response_size);
				} else	{	//Successful packet transmission
					//P-value
					uint32_t val = ((uint32_t)RC_async_request.rx_bytes[0])<<24;
					val |= ((uint32_t)RC_async_request.rx_bytes[1])<<16;
					val |= ((uint32_t)RC_async_request.rx_bytes[2])<<8;
					val |= ((uint32_t)RC_async_request.rx_bytes[3]);
					hex2ascii_long(val,&response_buf[0],&response_buf[1],&response_buf[2],&response_buf[3],&response_buf[4],&response_buf[5],&response_buf[6],&response_buf[7]);
					response_buf[8] = 9;	//TAB
					response_size = 9;
					dbg_uart_send_string(response_buf,response_size);
					//I-value
					val = ((uint32_t)RC_async_request.rx_bytes[4])<<24;
					val |= ((uint32_t)RC_async_request.rx_bytes[5])<<16;
					val |= ((uint32_t)RC_async_request.rx_bytes[6])<<8;
					val |= ((uint32_t)RC_async_request.rx_bytes[7]);
					hex2ascii_long(val,&response_buf[0],&response_buf[1],&response_buf[2],&response_buf[3],&response_buf[4],&response_buf[5],&response_buf[6],&response_buf[7]);
					response_buf[8] = 9;	//TAB
					response_size = 9;
					dbg_uart_send_string(response_buf,response_size);
					//D-value
					val = ((uint32_t)RC_async_request.rx_bytes[8])<<24;
					val |= ((uint32_t)RC_async_request.rx_bytes[9])<<16;
					val |= ((uint32_t)RC_async_request.rx_bytes[10])<<8;
					val |= ((uint32_t)RC_async_request.rx_bytes[11]);
					hex2ascii_long(val,&response_buf[0],&response_buf[1],&response_buf[2],&response_buf[3],&response_buf[4],&response_buf[5],&response_buf[6],&response_buf[7]);
					response_buf[8] = 9;	//TAB
					response_size = 9;
					dbg_uart_send_string(response_buf,response_size);
					//QPPS-value
					val = ((uint32_t)RC_async_request.rx_bytes[12])<<24;
					val |= ((uint32_t)RC_async_request.rx_bytes[13])<<16;
					val |= ((uint32_t)RC_async_request.rx_bytes[14])<<8;
					val |= ((uint32_t)RC_async_request.rx_bytes[15]);
					hex2ascii_long(val,&response_buf[0],&response_buf[1],&response_buf[2],&response_buf[3],&response_buf[4],&response_buf[5],&response_buf[6],&response_buf[7]);
					response_size = 8;
					dbg_uart_send_string(response_buf,response_size);
				}
				//Release request datastructure
				RC_async_request.rc_data_ready_flag = 0;
				RC_async_request.rc_request_flag = 0;
				persistent_cmd = PCMD_NONE;
				pcmd_data1 = 0;	//clear data stores
				pcmd_data0 = 0;
			} //Else wait for data to be ready
			break;
		case PCMD_RC_GET_PID2:
			if(!pcmd_data1){	//Roboclaw Command has not been sent yet
				if(RC_async_request.rc_request_flag){	//Wait for open request buffer
					break;
				} else {	//Buffer is open
					RC_async_request.tx_nbytes = getM2PID(RC_async_request.tx_bytes);
					RC_async_request.rx_nbytes = 18;
					RC_async_request.rc_request_flag = 1;
					pcmd_data1 = 1;	//Flag to indicate command has been sent
					break;
				}
			} else if(RC_async_request.rc_data_ready_flag){
				//Check recieved byte
				if(RC_async_request.error){	//Packet timeout
					response_buf[0] = 'e';
					response_buf[1] = 'r';
					response_buf[2] = 'r';
					response_size = 3;
					dbg_uart_send_string(response_buf,response_size);
				} else	{	//Successful packet transmission
					//P-value
					uint32_t val = ((uint32_t)RC_async_request.rx_bytes[0])<<24;
					val |= ((uint32_t)RC_async_request.rx_bytes[1])<<16;
					val |= ((uint32_t)RC_async_request.rx_bytes[2])<<8;
					val |= ((uint32_t)RC_async_request.rx_bytes[3]);
					hex2ascii_long(val,&response_buf[0],&response_buf[1],&response_buf[2],&response_buf[3],&response_buf[4],&response_buf[5],&response_buf[6],&response_buf[7]);
					response_buf[8] = 9;	//TAB
					response_size = 9;
					dbg_uart_send_string(response_buf,response_size);
					//I-value
					val = ((uint32_t)RC_async_request.rx_bytes[4])<<24;
					val |= ((uint32_t)RC_async_request.rx_bytes[5])<<16;
					val |= ((uint32_t)RC_async_request.rx_bytes[6])<<8;
					val |= ((uint32_t)RC_async_request.rx_bytes[7]);
					hex2ascii_long(val,&response_buf[0],&response_buf[1],&response_buf[2],&response_buf[3],&response_buf[4],&response_buf[5],&response_buf[6],&response_buf[7]);
					response_buf[8] = 9;	//TAB
					response_size = 9;
					dbg_uart_send_string(response_buf,response_size);
					//D-value
					val = ((uint32_t)RC_async_request.rx_bytes[8])<<24;
					val |= ((uint32_t)RC_async_request.rx_bytes[9])<<16;
					val |= ((uint32_t)RC_async_request.rx_bytes[10])<<8;
					val |= ((uint32_t)RC_async_request.rx_bytes[11]);
					hex2ascii_long(val,&response_buf[0],&response_buf[1],&response_buf[2],&response_buf[3],&response_buf[4],&response_buf[5],&response_buf[6],&response_buf[7]);
					response_buf[8] = 9;	//TAB
					response_size = 9;
					dbg_uart_send_string(response_buf,response_size);
					//QPPS-value
					val = ((uint32_t)RC_async_request.rx_bytes[12])<<24;
					val |= ((uint32_t)RC_async_request.rx_bytes[13])<<16;
					val |= ((uint32_t)RC_async_request.rx_bytes[14])<<8;
					val |= ((uint32_t)RC_async_request.rx_bytes[15]);
					hex2ascii_long(val,&response_buf[0],&response_buf[1],&response_buf[2],&response_buf[3],&response_buf[4],&response_buf[5],&response_buf[6],&response_buf[7]);
					response_size = 8;
					dbg_uart_send_string(response_buf,response_size);
				}
				//Release request datastructure
				RC_async_request.rc_data_ready_flag = 0;
				RC_async_request.rc_request_flag = 0;
				persistent_cmd = PCMD_NONE;
				pcmd_data1 = 0;	//clear data stores
				pcmd_data0 = 0;
			} //Else wait for data to be ready
			break;
		default:
			persistent_cmd = 0;
			break;
		}
		//When done with command, return terminal to user
		if(!persistent_cmd){
			dbg_uart_send_byte(13);		//CR
			dbg_uart_send_byte(10);		//Line feed
			dbg_uart_send_byte('>');	//Terminal prompt
			debug_cmd_ready = 0;
			debug_cmd_buf_ptr = 0;
		}
	}
	//Process command
	if(!persistent_cmd && debug_cmd_ready){
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
		} else if((strncmp(debug_cmd_buf,"testwarn",8)==0) && (debug_cmd_buf_ptr == 8)){
			//>testwarn
			issue_warning(WARN_TEST);
		} else if((strncmp(debug_cmd_buf,"testerr",7)== 0) && (debug_cmd_buf_ptr == 7)){
			//>testerr
			issue_error(ERROR_TEST);
		} else if((strncmp(debug_cmd_buf,"warndump",8)==0) && (debug_cmd_buf_ptr == 8)){
			//>warndump
			response_size = warning_dump(response_buf);
			dbg_uart_send_string(response_buf,response_size);
		} else if((strncmp(debug_cmd_buf,"errdump",7)==0) && (debug_cmd_buf_ptr == 7)){
			//>errdump
			response_size = error_dump(response_buf);
			dbg_uart_send_string(response_buf,response_size);
		} else if((strncmp(debug_cmd_buf,"warnclear",9)==0) && (debug_cmd_buf_ptr == 9)){
			//>warnclear
			clear_warnings();
		} else if((strncmp(debug_cmd_buf,"errclear",8)==0) && (debug_cmd_buf_ptr == 8)){
			//>errclear
			clear_errors();
		} else if((strncmp(debug_cmd_buf,"mon drill",9)==0) && (debug_cmd_buf_ptr == 9)){
			//>mon drill
			response_size = print_mon_analog_value(monitor_data.drill_current, response_buf);
			dbg_uart_send_string(response_buf,response_size);
		} else if((strncmp(debug_cmd_buf,"mon estop",9)==0) && (debug_cmd_buf_ptr == 9)){
			//>mon estop
			if(monitor_data.estop_status){
				response_buf[0] = 'o';
				response_buf[1] = 'n';
				response_size = 2;
			} else {
				response_buf[0] = 'o';
				response_buf[1] = 'f';
				response_buf[2] = 'f';
				response_size = 3;
			}
			dbg_uart_send_string(response_buf,response_size);
		} else if((strncmp(debug_cmd_buf,"mon m1I",7)==0) && (debug_cmd_buf_ptr == 7)){
			//>mon m1I
			response_size = print_mon_analog_value(monitor_data.m1_current, response_buf);
			dbg_uart_send_string(response_buf,response_size);
		} else if((strncmp(debug_cmd_buf,"mon m2I",7)==0) && (debug_cmd_buf_ptr == 7)){
			//>mon m2I
			response_size = print_mon_analog_value(monitor_data.m2_current, response_buf);
			dbg_uart_send_string(response_buf,response_size);
		} else if((strncmp(debug_cmd_buf,"mon mcu temp",12)==0) && (debug_cmd_buf_ptr == 12)){
			//>mon mcu temp
			response_size = print_mon_analog_value(monitor_data.mcu_temp, response_buf);
			dbg_uart_send_string(response_buf,response_size);
		} else if((strncmp(debug_cmd_buf,"mon rc lbatt",12)==0) && (debug_cmd_buf_ptr == 12)){
			//>mon rc lbatt
			response_size = print_mon_analog_value(monitor_data.rc_lbatt, response_buf);
			dbg_uart_send_string(response_buf,response_size);
		} else if((strncmp(debug_cmd_buf,"mon rc mbatt",12) == 0) && (debug_cmd_buf_ptr == 12)){
			//>mon rc mbatt
			response_size = print_mon_analog_value(monitor_data.rc_mbatt, response_buf);
			dbg_uart_send_string(response_buf,response_size);
		} else if((strncmp(debug_cmd_buf,"mon rc status",13)==0) && (debug_cmd_buf_ptr == 13)){
			//>mon rc status
			response_buf[0] = '0';
			response_buf[1] = 'x';
			hex2ascii_int(monitor_data.rc_status, &response_buf[2], &response_buf[3], &response_buf[4], &response_buf[5]);
			response_size = 6;
			dbg_uart_send_string(response_buf,response_size);
		} else if((strncmp(debug_cmd_buf,"mon rc temp",11)==0) && (debug_cmd_buf_ptr == 11)){
			//>mon rc temp
			response_size = print_mon_analog_value(monitor_data.rc_temp, response_buf);
			dbg_uart_send_string(response_buf,response_size);
		} else if((strncmp(debug_cmd_buf,"mon 12V",7)==0) && (debug_cmd_buf_ptr == 7)){
			//>mon 12V
			response_size = print_mon_analog_value(monitor_data.vsense_12V, response_buf);
			dbg_uart_send_string(response_buf,response_size);
		} else if((strncmp(debug_cmd_buf,"mon 3V3",7)==0) && (debug_cmd_buf_ptr == 7)){
			//>mon 3V3
			response_size = print_mon_analog_value(monitor_data.vsense_3V3, response_buf);
			dbg_uart_send_string(response_buf,response_size);
		} else if((strncmp(debug_cmd_buf,"mon 5V0",7)==0) && (debug_cmd_buf_ptr == 7)){
			//>mon 5V0
			response_size = print_mon_analog_value(monitor_data.vsense_5V0, response_buf);
			dbg_uart_send_string(response_buf,response_size);
		} else if((strncmp(debug_cmd_buf,"encoders",8)==0) && (debug_cmd_buf_ptr == 8)){
			//>encoders
			response_buf[0] = '0';
			response_buf[1] = 'x';
			hex2ascii_int((uint16_t)(enc1_count>>16), &response_buf[2], &response_buf[3], &response_buf[4], &response_buf[5]);
			hex2ascii_int((uint16_t)(enc1_count), &response_buf[6], &response_buf[7], &response_buf[8], &response_buf[9]);
			response_buf[10] = '\t';
			response_buf[11] = '0';
			response_buf[12] = 'x';
			hex2ascii_int((uint16_t)(enc2_count>>16), &response_buf[13], &response_buf[14], &response_buf[15], &response_buf[16]);
			hex2ascii_int((uint16_t)(enc2_count), &response_buf[17], &response_buf[18], &response_buf[19], &response_buf[20]);
			response_size = 21;
			dbg_uart_send_string(response_buf,response_size);
		} else if((strncmp(debug_cmd_buf,"drill man en",12)==0) && (debug_cmd_buf_ptr == 12)){
			//>drill man en
			drill_enable();
		} else if((strncmp(debug_cmd_buf,"drill man dis",13)==0) && (debug_cmd_buf_ptr == 13)){
			//>drill man dis
			drill_disable();
		} else if((strncmp(debug_cmd_buf,"drill man dir ",13)==0) && (debug_cmd_buf_ptr == 15)){
			//>drill man dir <direction>
			//>drill man dir 0
			if(debug_cmd_buf[14] == '0')
				drill_brake();
			if(debug_cmd_buf[14] == '1')
				drill_cw();
			if(debug_cmd_buf[14] == '2')
				drill_ccw();
		} else if((strncmp(debug_cmd_buf,"drill off",9)==0) && (debug_cmd_buf_ptr == 9)){
			//>drill dis
			drill_request = DRILL_REQ_OFF;
		} else if((strncmp(debug_cmd_buf,"drill dir ",9)==0) && (debug_cmd_buf_ptr == 11)){
			//>drill dir <direction>
			//>drill dir 0
			if(debug_cmd_buf[10] == '0')
				drill_request = DRILL_REQ_BRAKE;
			if(debug_cmd_buf[10] == '1')
				drill_request = DRILL_REQ_CW;
			if(debug_cmd_buf[10] == '2')
				drill_request = DRILL_REQ_CCW;
		} else if((strncmp(debug_cmd_buf,"s",1)==0) && (debug_cmd_buf_ptr == 1)){
			//>st+p
			uint8_t i = 0;
			uint16_t j = 0;
			for(i=0; i < 100; i++){
				stepper_step_single();
				for(j=0; j < 10000; j++);
			}
		} else if((strncmp(debug_cmd_buf,"step cw",7)==0) && (debug_cmd_buf_ptr == 7)){
			//>step cw
			stepper_enable(0);
		} else if((strncmp(debug_cmd_buf,"step ccw",8)==0) && (debug_cmd_buf_ptr == 8)){
			//>step ccw
			stepper_enable(1);
		} else if((strncmp(debug_cmd_buf,"step dis",8)==0) && (debug_cmd_buf_ptr == 8)){
			//>step dis
			stepper_disable();
		} else if((strncmp(debug_cmd_buf,"forceTA2",8)==0) && (debug_cmd_buf_ptr == 8)){
			timer_TA2_tick = 1;
		} else if((strncmp(debug_cmd_buf,"rc m1 ",6)==0) && (debug_cmd_buf_ptr == 8)){
			//>rc m1 <hex value>
			//>rc m1 2A
			uint8_t speed = ascii2hex_byte(debug_cmd_buf[6],debug_cmd_buf[7]);
			pcmd_data0 = speed;	//Motor velocity (signed 8-bit)
			pcmd_data1 = 0;		//Flag to indicate roboclaw command has been sent
			persistent_cmd = PCMD_RC_M1;
		} else if((strncmp(debug_cmd_buf,"rc m2 ",6)==0) && (debug_cmd_buf_ptr == 8)){
			//>rc m2 <hex value>
			//>rc m2 2A
			uint8_t speed = ascii2hex_byte(debug_cmd_buf[6],debug_cmd_buf[7]);
			pcmd_data0 = speed;	//Motor velocity (signed 8-bit)
			pcmd_data1 = 0;		//Flag to indicate roboclaw command has been sent
			persistent_cmd = PCMD_RC_M2;
		} else if((strncmp(debug_cmd_buf,"rc pid1 ",8)==0) && (debug_cmd_buf_ptr == 43)){
			//>rc pid1 <D> <P> <I> <QPPS>
			//>rc pid1 12341234 56785678 9ABC9ABC DEFGDEFG
			//D-value (4 bytes)
			pcmd_data_long[0] = ascii2hex_long(debug_cmd_buf[8],debug_cmd_buf[9],debug_cmd_buf[10],debug_cmd_buf[11],debug_cmd_buf[12],debug_cmd_buf[13],debug_cmd_buf[14],debug_cmd_buf[15]);
			//P-value (4-bytes)
			pcmd_data_long[1] = ascii2hex_long(debug_cmd_buf[17],debug_cmd_buf[18],debug_cmd_buf[19],debug_cmd_buf[20],debug_cmd_buf[21],debug_cmd_buf[22],debug_cmd_buf[23],debug_cmd_buf[24]);
			//I-value (4-bytes)
			pcmd_data_long[2] = ascii2hex_long(debug_cmd_buf[26],debug_cmd_buf[27],debug_cmd_buf[28],debug_cmd_buf[29],debug_cmd_buf[30],debug_cmd_buf[31],debug_cmd_buf[32],debug_cmd_buf[33]);
			//QPPS (4-bytes)
			pcmd_data_long[3] = ascii2hex_long(debug_cmd_buf[35],debug_cmd_buf[36],debug_cmd_buf[37],debug_cmd_buf[38],debug_cmd_buf[39],debug_cmd_buf[40],debug_cmd_buf[41],debug_cmd_buf[42]);
			pcmd_data1 = 0;		//Flag to indicate roboclaw command has been sent
			persistent_cmd = PCMD_RC_PID1;
		} else if((strncmp(debug_cmd_buf,"rc pid2 ",8)==0) && (debug_cmd_buf_ptr == 43)){
			//>rc pid1 <D> <P> <I> <QPPS>
			//>rc pid1 12341234 56785678 9ABC9ABC DEFGDEFG
			//D-value (4 bytes)
			pcmd_data_long[0] = ascii2hex_long(debug_cmd_buf[8],debug_cmd_buf[9],debug_cmd_buf[10],debug_cmd_buf[11],debug_cmd_buf[12],debug_cmd_buf[13],debug_cmd_buf[14],debug_cmd_buf[15]);
			//P-value (4-bytes)
			pcmd_data_long[1] = ascii2hex_long(debug_cmd_buf[17],debug_cmd_buf[18],debug_cmd_buf[19],debug_cmd_buf[20],debug_cmd_buf[21],debug_cmd_buf[22],debug_cmd_buf[23],debug_cmd_buf[24]);
			//I-value (4-bytes)
			pcmd_data_long[2] = ascii2hex_long(debug_cmd_buf[26],debug_cmd_buf[27],debug_cmd_buf[28],debug_cmd_buf[29],debug_cmd_buf[30],debug_cmd_buf[31],debug_cmd_buf[32],debug_cmd_buf[33]);
			//QPPS (4-bytes)
			pcmd_data_long[3] = ascii2hex_long(debug_cmd_buf[35],debug_cmd_buf[36],debug_cmd_buf[37],debug_cmd_buf[38],debug_cmd_buf[39],debug_cmd_buf[40],debug_cmd_buf[41],debug_cmd_buf[42]);
			pcmd_data1 = 0;		//Flag to indicate roboclaw command has been sent
			persistent_cmd = PCMD_RC_PID2;
		} else if((strncmp(debug_cmd_buf,"rc gs1 ",6)==0) && (debug_cmd_buf_ptr == 6)){
			//>rc gs1
			pcmd_data1 = 0;		//Flag to indicate roboclaw command has been sent
			persistent_cmd = PCMD_RC_GS1;
		} else if((strncmp(debug_cmd_buf,"rc gs2 ",6)==0) && (debug_cmd_buf_ptr == 6)){
			//>rc gs1
			pcmd_data1 = 0;		//Flag to indicate roboclaw command has been sent
			persistent_cmd = PCMD_RC_GS2;
		} else if((strncmp(debug_cmd_buf,"rc v1 ",6)==0) && (debug_cmd_buf_ptr == 14)){
			//>rc v1 <velocity>
			//>rc v1 12345678
			pcmd_data_long[0] = ascii2hex_long(debug_cmd_buf[6],debug_cmd_buf[7],debug_cmd_buf[8],debug_cmd_buf[9],debug_cmd_buf[10],debug_cmd_buf[11],debug_cmd_buf[12],debug_cmd_buf[13]);
			pcmd_data1 = 0;		//Flag to indicate roboclaw command has been sent
			persistent_cmd = PCMD_RC_V1;
		} else if((strncmp(debug_cmd_buf,"rc v2 ",6)==0) && (debug_cmd_buf_ptr == 14)){
			//>rc v2 <velocity>
			//>rc v2 12345678
			pcmd_data_long[0] = ascii2hex_long(debug_cmd_buf[6],debug_cmd_buf[7],debug_cmd_buf[8],debug_cmd_buf[9],debug_cmd_buf[10],debug_cmd_buf[11],debug_cmd_buf[12],debug_cmd_buf[13]);
			pcmd_data1 = 0;		//Flag to indicate roboclaw command has been sent
			persistent_cmd = PCMD_RC_V2;
		} else if((strncmp(debug_cmd_buf,"rc v12 ",7)==0) && (debug_cmd_buf_ptr == 24)){
			//>rc v12 <velocity motor 1> <velocity motor 2>
			//>rc v12 01234567 89ABCDEF
			//Motor 1 velocity
			pcmd_data_long[0] = ascii2hex_long(debug_cmd_buf[7],debug_cmd_buf[8],debug_cmd_buf[9],debug_cmd_buf[10],debug_cmd_buf[11],debug_cmd_buf[12],debug_cmd_buf[13],debug_cmd_buf[14]);
			//Motor 2 velocity
			pcmd_data_long[1] = ascii2hex_long(debug_cmd_buf[16],debug_cmd_buf[17],debug_cmd_buf[18],debug_cmd_buf[19],debug_cmd_buf[20],debug_cmd_buf[21],debug_cmd_buf[22],debug_cmd_buf[23]);
			pcmd_data1 = 0;		//Flag to indicate roboclaw command has been sent
			persistent_cmd = PCMD_RC_V12;
		} else if((strncmp(debug_cmd_buf,"rc get pid1",11)==0) && (debug_cmd_buf_ptr == 11)){
			//>rc get pid1
			pcmd_data1 = 0;		//Flag to indicate roboclaw command has been sent
			persistent_cmd = PCMD_RC_GET_PID1;
		} else if((strncmp(debug_cmd_buf,"rc get pid2",11)==0) && (debug_cmd_buf_ptr == 11)){
			//>rc get pid2
			pcmd_data1 = 0;		//Flag to indicate roboclaw command has been sent
			persistent_cmd = PCMD_RC_GET_PID2;
		} else if((strncmp(debug_cmd_buf,"exit",4)==0) && (debug_cmd_buf_ptr == 4)){
			exit_debug = 1;
		} else {
			dbg_uart_send_string("Invalid Command",15);
		}
		debug_cmd_ready = 0;
		if(!persistent_cmd){
			dbg_uart_send_byte(13);		//CR
			dbg_uart_send_byte(10);		//Line feed
			dbg_uart_send_byte('>');	//Terminal prompt
			debug_cmd_buf_ptr = 0;
		}
	}
}

/** END Debug Task functions **/

/** Interrupts **/

/* Timer2 A0 interrupt service routine
 * Trigger synchronous check/encoder routines on roboclaw
 */
#pragma vector=TIMER2_A0_VECTOR
__interrupt void TIMER2_A0_ISR(void){
	timer_TA2_tick = 1;
}

/* Timer0 A0 interrupt service routine
 * Keep track of step counts and stop when reached setpoint
 */
#pragma vector=TIMER0_A0_VECTOR
__interrupt void TIMER0_A0_ISR(void){
	if(step_direction){	//CCW
		step_position++;
	} else {
		step_position--;
	}
	//Check if reached setpoint.  If so, notify SM
	if(step_position == step_setpoint){
		TA0CTL &= ~MC_3;	//Timer off
		step_run_done = 1;
	}
}

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
		if(DBG_UART_data.rx_head == DBG_UART_data.rx_tail)
			issue_warning(WARN_DBG_RX_BUF_FULL);
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
		if(RC_UART_data.rx_head == RC_UART_data.rx_tail)
			issue_warning(WARN_RC_RX_BUF_FULL);
	} else if((UCA1IE & UCTXIE) && (UCA1IFG & UCTXIFG)){	//UART Txbuf ready interrupt
		//Load data and clear interrupt
		volatile uint8_t temp = RC_UART_data.tx_bytes[RC_UART_data.tx_tail];
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
void reset_isr(void){
	switch(SYSRSTIV){
		case SYSRSTIV_NONE:
			break;
		case SYSRSTIV_BOR:
			//issue_warning(WARN_RST_BOR);	//Occurs during power on if debug cable is plugged in
			break;
		case SYSRSTIV_RSTNMI:
			//issue_warning(WARN_RST_RSTNMI);
			break;
		case SYSRSTIV_DOBOR:
			issue_warning(WARN_RST_DOBOR);
			break;
		case SYSRSTIV_LPM5WU:
			issue_warning(WARN_RST_LPM5WU);
			break;
		case SYSRSTIV_SECYV:
			//issue_warning(WARN_RST_SECYV); //Software reset
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
