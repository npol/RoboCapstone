/*
 * main.c
 * Created 4/21/16 by Nishant Pol
 * Robotics Capstone 16-474
 * MSP430F5521
 * PC2CAN Controller Code
 *
 * Code Composer v6
 * Version History
 * 4/21/16 Ported from Sensor Controller code
 */
#include <msp430.h>
#include "utils.h"
#include "clock_f5.h"
#include "dbg_uart_uscia0.h"
#include "pc_uart_uscia1.h"
#include "MCP2515.h"
#include "can_spi_uscib0.h"

/** Debug task macros and globals **/
void debug_task(void);
void reset_isr(void);

#define DEBUG_CMD_BUF_SIZE 64
#define DEBUG_RESPONSE_BUF_SIZE 250
uint8_t debug_cmd_buf[DEBUG_CMD_BUF_SIZE];
uint8_t debug_cmd_buf_ptr = 0;

//Persistent command codes
#define PCMD_NONE	0x00
/** END Debug task macros and globals **/

/** PC Interface Macros and globals **/
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
				PC_WAIT_CAN
				} pc_state_t;
volatile pc_state_t pc_current_state = PC_IDLE;

#define PC_CMD_BUF_SIZE 16
#define PC_RESPONSE_BUF_SIZE 64
uint8_t pc_cmd_buf[PC_CMD_BUF_SIZE];
uint8_t pc_cmd_buf_ptr = 0;
uint8_t pc_cmd_size = 0;

#define PC_START_DELIMITER 0x7E

/** END PC Interface Macros and globals **/

/** Warning/Error code buffers and flags **/
volatile uint16_t warn_log[WARN_LOG_SIZE] = {0};
volatile uint8_t warn_log_ptr = 0;
volatile uint8_t warn_flag = 0;
volatile uint16_t err_log[ERR_LOG_SIZE] = {0};
volatile uint8_t err_log_ptr = 0;
volatile uint8_t err_flag = 0;
/** END Warning/Error code buffers and flags **/

/** Monitoring task globals **/
void monitor_setup(void);
void monitor_task(void);
/* Struct to store monitoring data
 * Analog readings: Array contains current value | min value | max value | ready flag
 */
//TODO: update monitor struct
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
#define MCU_TEMP_MIN	0x0000
#define MCU_TEMP_MAX	0xFFFF

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

/** END Monioring task globals **/

/** ADC task globals **/
void adc_setup(void);
typedef enum  {INIT_SEQ1,
				WAIT_SEQ1,
				INIT_SEQ2,
				RUN_SEQ2,
				WAIT_SEQ2,
				UPDATE_OUT_BUF
				} adc_state_t;
volatile adc_state_t adc_current_state = INIT_SEQ1;
/* Output buffer and internal buffer data order
 * 0: A8 5Vsense
 * 1: A9 12Vsense
 * 2: A10 MCU temp (2.5V reference)
 * 3: A11 3V3 sense (2.5V reference)
 */
#define ADC_BUF_LEN 4
uint16_t adc_output_buffer[ADC_BUF_LEN] = {0};	//Data buffer for other tasks to read
uint8_t adc_data_ready = 0x00;	//Set bit indicates new conversion (set by adc task, cleared by other tasks)

/** END ADC task globals **/

/** CAN task globals **/
void can_task(void);

typedef enum  {CAN_IDLE,
				INIT_CHECK_ERR_REGS,
				WAIT_CHECK_ERR_REGS,
				READ_ERR_REGS,
				INIT_SETUP_TX,
				WAIT_SETUP_TX,
				INIT_TX_MSG,
				WAIT_TX_MSG,
				INIT_CHK_MSG,
				WAIT_CHK_MSG,
				READ_CHK_MSG,
				INIT_READ_MSG0,
				WAIT_READ_MSG0,
				READ_DATA_MSG0,
				WAIT_MSG0_CLEAR,
				END_MSG0,
				INIT_READ_MSG1,
				WAIT_READ_MSG1,
				READ_DATA_MSG1,
				WAIT_MSG1_CLEAR,
				END_MSG1
				} can_state_t;
volatile can_state_t can_current_state = CAN_IDLE;
#define CAN_RX_BUF_SIZE 8
uint8_t can_rx_buf[CAN_RX_BUF_SIZE];
uint8_t can_rx_buf_head = 0;
uint8_t can_rx_buf_tail = 0;
uint8_t can_dbg = 0;

struct CAN_tx_req_struct{
	uint8_t tx_request;			//Set to indicate request, cleared by CAN SM when transmitted
	uint16_t msg_id;			//11-bit standard identifier
	uint8_t length;				//Number of bytes to transmit
	uint8_t data[8];			//Data bytes
};
struct CAN_tx_req_struct can_tx_req = {
		.tx_request = 0,
		.msg_id = 0,
		.length = 0,
		.data = {0}
};

/** END CAN task globals **/

/** Main Loop **/

int main(void) {
	WDTCTL = WDTPW + WDTHOLD;   // Stop watchdog timer
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
	led_P5_7_on();		//System ok LED
	reset_isr();
	setup_clock();
	setup_dbg_uart();
	setup_pc_uart();
	monitor_setup();
	adc_setup();
	CAN_SPI_setup(0, 1);	//Idle Low, out on falling edge
	pc_setup();
    // Enable Interrupts
    __bis_SR_register(GIE);
    setup_mcp2515();
    while(1)
    {
        debug_task();
        //monitor_task();
        can_task();
        pc_task();
    }
}

/** END Main Loop **/

/** CAN task functions **/
void can_task(void){
	uint8_t action_complete;
	uint8_t buf[16];
	uint8_t resp_size = 0;
	uint8_t i;
	switch(can_current_state){
	case CAN_IDLE:						//STATE_CAN0
		//State action: no action
		//State transition
		if(!can_dbg && CAN_GEN_INT){
			can_current_state = INIT_CHECK_ERR_REGS;		//T_CAN0
		} else if(!can_dbg && CAN_RX0BUF_INT){
			can_current_state = INIT_READ_MSG0;				//T_CAN18
		} else if(!can_dbg && CAN_RX1BUF_INT){
			can_current_state = INIT_READ_MSG1;				//T_CAN26
		} else if(!can_dbg && can_tx_req.tx_request){	//TODO: complete can request struct/buffer
			can_current_state = INIT_SETUP_TX;				//T_CAN6
		} else {
			can_current_state = CAN_IDLE;					//T_CAN5
		}
		break;
	case INIT_CHECK_ERR_REGS:			//STATE_CAN1
		//State action
		mcp2515_read_mult_registers_nonblock_init(MCP2515_CANINTF,2);	//Read 0x2C, 0x2D
		//State transition
		can_current_state = WAIT_CHECK_ERR_REGS;			//T_CAN1
		break;
	case WAIT_CHECK_ERR_REGS:			//STATE_CAN2
		//State action: no action
		//State transition
		if(is_CAN_spi_rx_ready()){
			can_current_state = READ_ERR_REGS;				//T_CAN3
		} else {
			can_current_state = WAIT_CHECK_ERR_REGS;		//T_CAN2
		}
		break;
	case READ_ERR_REGS:					//STATE_CAN3
		//State action
		resp_size = mcp2515_read_mult_registers_nonblock_getdata(buf);
		//Check EFLG error source
		if(buf[1] & MCP2515_RX1OVR)
			issue_warning(WARN_CAN_RX1OVR);
		if(buf[1] & MCP2515_RX0OVR) issue_warning(WARN_CAN_RX0OVR);
		if(buf[1] & MCP2515_TXBO) issue_warning(WARN_CAN_TXBO);
		if(buf[1] & MCP2515_TXEP) issue_warning(WARN_CAN_TXEP);
		if(buf[1] & MCP2515_RXEP) issue_warning(WARN_CAN_RXEP);
		if(buf[1] & MCP2515_TXWAR) issue_warning(WARN_CAN_TXWAR);
		if(buf[1] & MCP2515_RXWAR) issue_warning(WARN_CAN_RXWAR);
		if(buf[1] & MCP2515_EWARN) issue_warning(WARN_CAN_EWARN);
		//State transition
		can_current_state = CAN_IDLE;						//T_CAN4
		break;
	case INIT_SETUP_TX:					//STATE_CAN4
		//State action
		buf[0] = MCP2515_TXP_3;
		buf[1] = (uint8_t)(can_tx_req.msg_id >> 3);
		buf[2] = (uint8_t)(can_tx_req.msg_id << 5);
		buf[3] = 0;
		buf[4] = 0;
		buf[5] = can_tx_req.length;
		for(i=0; i<can_tx_req.length; i++){
			buf[6+i] = can_tx_req.data[i];
		}
		mcp2515_write_mult_registers_nonblock_init(MCP2515_TXB0CTRL, 14, buf);
		//State transition
		can_current_state = WAIT_SETUP_TX;					//T_CAN7
		break;
	case WAIT_SETUP_TX:					//STATE_CAN5
		//State action: none
		//State transition
		if(is_CAN_spi_rx_ready()){
			can_current_state = INIT_TX_MSG;				//T_CAN9
		} else {
			can_current_state = WAIT_SETUP_TX;				//T_CAN8
		}
		break;
	case INIT_TX_MSG:					//STATE_CAN6
		//State action
		mcp2515_write_mult_registers_nonblock_end();
		//Send command to send message
		mcp2515_rts_nonblock_init(0);
		//State transition
		can_current_state = WAIT_TX_MSG;					//T_CAN10
		break;
	case WAIT_TX_MSG:					//STATE_CAN7
		//State action: no action
		//State transition
		if(is_CAN_spi_rx_ready()){
			can_current_state = INIT_CHK_MSG;				//T_CAN12
		} else {
			can_current_state = WAIT_TX_MSG;				//T_CAN11
		}
		break;
	case INIT_CHK_MSG:					//STATE_CAN8
		//State action
		mcp2515_rts_nonblock_end();
		mcp2515_read_mult_registers_nonblock_init(MCP2515_TXB0CTRL, 1);
		//State transition
		can_current_state = WAIT_CHK_MSG;					//T_CAN13
		break;
	case WAIT_CHK_MSG:					//STATE_CAN9
		//State action: no action
		//State transition
		if(is_CAN_spi_rx_ready()){
			can_current_state = READ_CHK_MSG;				//T_CAN15
		} else {
			can_current_state = WAIT_CHK_MSG;				//T_CAN14
		}
		break;
	case READ_CHK_MSG:					//STATE_CAN20
		//State action
		resp_size = mcp2515_read_mult_registers_nonblock_getdata(buf);
		if(buf[0] & MCP2515_ABTF) issue_warning(WARN_CAN_TX_ABTF);
		if(buf[0] & MCP2515_TXERR) issue_warning(WARN_CAN_TX_TXERR);
		if(buf[0] & MCP2515_TXREQ) issue_warning(WARN_CAN_TXREQ);
		//State transition
		if(buf[0] & MCP2515_MLOA){
			can_current_state = INIT_TX_MSG;					//T_CAN16
		} else {
			can_tx_req.tx_request = 0;
			can_current_state = CAN_IDLE;						//T_CAN17
		}
		break;
	case INIT_READ_MSG0:				//STATE_CAN10
		//State action
		mcp2515_read_mult_registers_nonblock_init(MCP2515_RXB0CTRL,14);	//Read 0x60 to 0x6D
		//State transition
		can_current_state = WAIT_READ_MSG0;					//T_CAN19
		break;
	case INIT_READ_MSG1:				//STATE_CAN15
		//State action
		mcp2515_read_mult_registers_nonblock_init(MCP2515_RXB1CTRL,14);	//Read 0x70 to 0x7D
		//State transition
		can_current_state = WAIT_READ_MSG1;					//T_CAN27
		break;
	case WAIT_READ_MSG0:				//STATE_CAN11
		//State action: no action
		//State transition
		if(is_CAN_spi_rx_ready()){
			can_current_state = READ_DATA_MSG0;				//T_CAN21
		} else {
			can_current_state = WAIT_READ_MSG0;				//T_CAN20
		}
		break;
	case WAIT_READ_MSG1:				//STATE_CAN16
		//State action: no action
		//State transition
		if(is_CAN_spi_rx_ready()){
			can_current_state = READ_DATA_MSG1;				//T_CAN29
		} else {
			can_current_state = WAIT_READ_MSG1;				//T_CAN28
		}
		break;
	case READ_DATA_MSG0:				//STATE_CAN12
		//State action
		//Get data
		resp_size = mcp2515_read_mult_registers_nonblock_getdata(buf);
		//Issue relevant commands
		dbg_uart_send_string("rx msg0",7);
		//can_process_msg(buf, resp_size);
		//Clear interrupt to release buffer
		mcp2515_bitmod_register_nonblock_init(MCP2515_CANINTF,MCP2515_RX0IF,0x00);
		//State transition
		can_current_state = WAIT_MSG0_CLEAR;				//T_CAN22
		break;
	case READ_DATA_MSG1:				//STATE_CAN17
		//State action
		//Get data
		resp_size = mcp2515_read_mult_registers_nonblock_getdata(buf);
		//Issue relevant commands
		dbg_uart_send_string("rx msg1",7);
		//can_process_msg(buf, resp_size);
		//Clear interrupt to release buffer
		mcp2515_bitmod_register_nonblock_init(MCP2515_CANINTF,MCP2515_RX1IF,0x00);
		//State transition
		can_current_state = WAIT_MSG1_CLEAR;				//T_CAN30
		break;
	case WAIT_MSG0_CLEAR:				//STATE_CAN13
		//State action
		action_complete = 1;
		//State transition
		if(is_CAN_spi_rx_ready() && action_complete){
			can_current_state = END_MSG0;					//T_CAN24
		} else {
			can_current_state = WAIT_MSG0_CLEAR;			//T_CAN23
		}
		break;
	case WAIT_MSG1_CLEAR:				//STATE_CAN18
		//State action
		action_complete = 1;
		//State transition
		if(is_CAN_spi_rx_ready() && action_complete){
			can_current_state = END_MSG1;					//T_CAN32
		} else {
			can_current_state = WAIT_MSG1_CLEAR;			//T_CAN31
		}
		break;
	case END_MSG0:						//STATE_CAN14
		//State action
		mcp2515_bitmod_register_nonblock_end();
		//State transition
		can_current_state = CAN_IDLE;						//T_CAN25
		break;
	case END_MSG1:						//STATE_CAN19
		//State action
		mcp2515_bitmod_register_nonblock_end();
		//State transition
		can_current_state = CAN_IDLE;						//T_CAN33
		break;
	default:
		can_current_state = CAN_IDLE;
		issue_warning(WARN_ILLEGAL_CAN_STATE);
		break;
	}
}

/* Initiate message transmission
 * sid: Standard identifier, lowest 10 bits
 * len: number of data bytes
 * data: array of data bytes
 */
void can_tx_message_buf0(uint16_t sid, uint8_t len, uint8_t *data){
	//Standard Identifier
	mcp2515_write_register(MCP2515_TXB0SIDH,(uint8_t)(sid>>3));
	mcp2515_write_register(MCP2515_TXB0SIDL,(uint8_t)(sid<<5));
	//Data length
	mcp2515_write_register(MCP2515_TXB0DLC,len);
	//Data bytes
	uint8_t i;
	for(i=0; i<len;i++){
		mcp2515_write_register(MCP2515_TXB0D0+i,data[i]);
	}
	//Transmit request
	mcp2515_write_register(MCP2515_TXB0CTRL, MCP2515_TXREQ|MCP2515_TXP_3);
}

/* Issue commands on message
 *
 */
void can_process_msg(uint8_t *buf, uint8_t buf_size){
	uint16_t dest_addr = buf[2]>>5;
	dest_addr |= buf[1] << 3;
	switch(buf[6]){
	case 0x100:

		break;
	}
}

/* Manually read MCP2515 register
 * debug_cmd_buf: Character buffer with user command
 * response_buf: Empty buffer to send response
 */
inline uint8_t debug_mcp2515_read_reg(uint8_t *debug_cmd_buf,uint8_t *response_buf){
	uint8_t reg_addr = ascii2hex_byte(debug_cmd_buf[14],debug_cmd_buf[15]);
	uint8_t reg_value = mcp2515_read_register(reg_addr);
	uint8_t ascii_value_high = '0';
	uint8_t ascii_value_low = '0';
	hex2ascii_byte(reg_value, &ascii_value_high, &ascii_value_low);
	response_buf[0] = 'r';
	response_buf[1] = 'e';
	response_buf[2] = 'a';
	response_buf[3] = 'd';
	response_buf[4] = ' ';
	response_buf[5] = '0';
	response_buf[6] = 'x';
	response_buf[7] = debug_cmd_buf[14];
	response_buf[8] = debug_cmd_buf[15];
	response_buf[9] = ':';
	response_buf[10] = ' ';
	response_buf[11] = '0';
	response_buf[12] = 'x';
	response_buf[13] = ascii_value_high;
	response_buf[14] = ascii_value_low;
	return 15;
}

/* Manually write MCP2515 register
 * debug_cmd_buf: Character buffer with user command
 * response_buf: Empty buffer to send response
 */
inline uint8_t debug_mcp2515_write_reg(uint8_t *debug_cmd_buf,uint8_t *response_buf){
	uint8_t reg_addr = ascii2hex_byte(debug_cmd_buf[15],debug_cmd_buf[16]);
	uint8_t data = ascii2hex_byte(debug_cmd_buf[20],debug_cmd_buf[21]);
	mcp2515_write_register(reg_addr, data);
	response_buf[0] = 'w';
	response_buf[1] = 'r';
	response_buf[2] = 'i';
	response_buf[3] = 't';
	response_buf[4] = 'e';
	response_buf[5] = ' ';
	response_buf[6] = '0';
	response_buf[7] = 'x';
	response_buf[8] = debug_cmd_buf[15];
	response_buf[9] = debug_cmd_buf[16];
	response_buf[10] = ':';
	response_buf[11] = ' ';
	response_buf[12] = '0';
	response_buf[13] = 'x';
	response_buf[14] = debug_cmd_buf[20];
	response_buf[15] = debug_cmd_buf[21];
	return 16;
}

inline uint8_t debug_can_rx(uint8_t *response_buf){
	uint8_t response_size = 0;
	//Check if there is data
	if(can_rx_buf_head != can_rx_buf_tail){
		hex2ascii_byte(can_rx_buf[can_rx_buf_tail],&response_buf[2],&response_buf[3]);
		can_rx_buf_tail++;
		if(can_rx_buf_tail >= CAN_RX_BUF_SIZE){
			can_rx_buf_tail = 0;
		}
		response_buf[0] = '0';
		response_buf[1] = 'x';
		response_size = 4;
	} else {//Buffer empty
		response_buf[0] = 'n';
		response_buf[1] = 'a';
		response_size = 2;
	}
	return response_size;
}

/** END CAN task functions **/

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
/*
void monitor_task(void){
	static uint8_t run_mon_cnt = 0;	//Set when Check routine must run
	uint8_t pckt_size = 0;				//Size of transmitted packet
	uint8_t buf[16];					//Buffer for transmitted/recieved packet
	uint16_t conversion;
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
		}
		//TODO: send CAN status packet
		//State transition
		monCurrState = MON_WAIT;		//T_MON20
		break;
	default:
		issue_warning(WARN_ILLEGAL_MON_SM_STATE);
		monCurrState = MON_WAIT;
		break;
	}
}
*/

/** END Monitor Task Functions **/

/** PC interface functions **/
void pc_setup(void){
	//Initialize timeout timer
	TA0CTL = TASSEL_2 + ID_3 + MC_1; //SMCLK div 8 Up mode
	TA0CCR0 = 31250;	//10ms
}

uint8_t check_timeout(void){
	if(TA0CCTL0 & CCIFG){	//Check interrupt flag
		TA0CCTL0 &= ~CCIFG;	//Clear flag
		return 1;
	}
	return 0;
}

void reset_timer(void){
	TA0CTL |= TACLR;
	TA0CCTL0 &= ~CCIFG;
}

void pc_task(void){
	uint8_t rx_byte = 0;
	uint8_t i;
	switch(pc_current_state){
	case PC_IDLE:
		//State action
		if(is_pc_uart_rx_data_ready()){
			rx_byte = pc_uart_get_byte();
		}
		//State transition
		if(rx_byte == PC_START_DELIMITER){
			pc_current_state = PC_INIT_BUF;
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
		} else if(is_pc_uart_rx_data_ready()){
			reset_timer();
			pc_current_state = PC_GET_SIZE;
		} else {
			pc_current_state = PC_WAIT_SIZE;
		}
		break;
	case PC_GET_SIZE:
		//State action
		pc_cmd_size = pc_uart_get_byte();	//Get size
		//State transition
		pc_current_state = PC_LOAD_BUF;
	case PC_LOAD_BUF:
		//State action
		if(is_pc_uart_rx_data_ready()){
			reset_timer();
			pc_cmd_buf[pc_cmd_buf_ptr] = pc_uart_get_byte();
			pc_cmd_buf_ptr++;
		}
		//State transition
		if(check_timeout()){
			pc_current_state = PC_IDLE;
		} else if((pc_cmd_size == pc_cmd_buf_ptr)){
			pc_current_state = PC_EXEC_CMD;
		} else {
			pc_current_state = PC_LOAD_BUF;
		}
		break;
	case PC_WAIT_CAN:
		//State action: no action
		//State transition
		if(!can_tx_req.tx_request){
			pc_current_state = PC_WAIT_CAN;
		} else {
			pc_current_state = PC_EXEC_CMD;
		}
	case PC_EXEC_CMD:
		//State action
		//Send motor velocities
		switch(pc_cmd_buf[0]){
		case 0x30:	//Motor Velocities
			can_tx_req.msg_id = 0x0100;
			can_tx_req.length = 8;
			for(i=0; i<8; i++){
				can_tx_req.data[i] = pc_cmd_buf[i+1];
			}
			can_tx_req.tx_request = 1;
			break;
		case 0x00:	//Test message
			can_tx_req.msg_id = 0x0000;
			can_tx_req.length = 0;
			can_tx_req.tx_request = 1;
		}
		//State transition
		pc_current_state = PC_IDLE;
		break;
	default:
		pc_current_state = PC_IDLE;
		issue_warning(WARN_ILLEGAL_PC_STATE);
		break;
	}
	return;
}
/** END PC interface functions **/

/** Debug Task functions **/

/* Process debug command functions */
void debug_task(void){
	static uint8_t persistent_cmd = 0;	//Code of command to continue running while other tasks run
	//static uint8_t pcmd_data0 = 0;		//Storage for persistent commands
	//static uint8_t pcmd_data1 = 0;
	//static uint32_t pcmd_data_long[8] = {0};
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
		} else if((strncmp(debug_cmd_buf,"can regread",11)==0) && (debug_cmd_buf_ptr == 16)){
			//>can regread <register in hex>
			//>can regread 0x00
			response_size = debug_mcp2515_read_reg(debug_cmd_buf,response_buf);
			dbg_uart_send_string(response_buf,response_size);
		} else if((strncmp(debug_cmd_buf,"can regwrite",12)==0) && (debug_cmd_buf_ptr == 22)){
			//>can regwrite <register in hex> <data in hex>
			//>can regwrite 0x00 0x00
			response_size = debug_mcp2515_write_reg(debug_cmd_buf,response_buf);
			dbg_uart_send_string(response_buf,response_size);
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
		} else if((strncmp(debug_cmd_buf,"mon mcu temp",12)==0) && (debug_cmd_buf_ptr == 12)){
			//>mon mcu temp
			response_size = print_mon_analog_value(monitor_data.mcu_temp, response_buf);
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
		} else if((strncmp(debug_cmd_buf,"can tx",6)==0) && (debug_cmd_buf_ptr == 11)){
			//can tx 0x00
			uint8_t temp = ascii2hex_byte(debug_cmd_buf[9],debug_cmd_buf[10]);
			can_tx_message_buf0(0x279, 1, &temp);
			dbg_uart_send_string("tx",2);
		} else if((strncmp(debug_cmd_buf,"can rx",6)==0) && (debug_cmd_buf_ptr == 6)){
			response_size = debug_can_rx(response_buf);
			dbg_uart_send_string(response_buf, response_size);
		} else if((strncmp(debug_cmd_buf,"can dbg en",10)==0) && (debug_cmd_buf_ptr == 10)){
			can_dbg = 1;
		} else if((strncmp(debug_cmd_buf,"can dbg dis",11)==0) && (debug_cmd_buf_ptr == 11)){
			can_dbg = 0;
		} else if((strncmp(debug_cmd_buf,"can tx2",7)==0) && (debug_cmd_buf_ptr == 7)){
			//only do when can_dbg = 1, and clear can_dbg after this command
			if(!can_tx_req.tx_request){
				can_tx_req.msg_id = 0x0000;
				can_tx_req.length = 0;
				can_tx_req.tx_request = 1;
				dbg_uart_send_string("ok",2);
			} else {
				dbg_uart_send_string("fail",4);
			}
		} else if((strncmp(debug_cmd_buf,"adc0",4)==0) && (debug_cmd_buf_ptr == 4)){
			response_size = print_int(adc_output_buffer[0],response_buf);
			dbg_uart_send_string(response_buf,response_size);
		} else if((strncmp(debug_cmd_buf,"adc1",4)==0) && (debug_cmd_buf_ptr == 4)){
			response_size = print_int(adc_output_buffer[1],response_buf);
			dbg_uart_send_string(response_buf,response_size);
		} else if((strncmp(debug_cmd_buf,"adc2",4)==0) && (debug_cmd_buf_ptr == 4)){
			response_size = print_int(adc_output_buffer[2],response_buf);
			dbg_uart_send_string(response_buf,response_size);
		} else if((strncmp(debug_cmd_buf,"adc3",4)==0) && (debug_cmd_buf_ptr == 4)){
			response_size = print_int(adc_output_buffer[3],response_buf);
			dbg_uart_send_string(response_buf,response_size);
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

/** ADC Task Functions **/
/* Output buffer and internal buffer data order
 * 0: A8 5Vsense
 * 1: A9 12Vsense
 * 2: A10 MCU temp (2.5V reference)
 * 3: A11 3V3 sense (2.5V reference)
 */
void adc_setup(void){
	//Setup ADC pins to use ADC
	P6SEL = 0xFF;
	P5SEL |= BIT0+BIT1;
	//Setup ADC12
	ADC12CTL0 &= ~ADC12ENC;						//Disable conversions to change channel
	ADC12CTL0 = ADC12SHT1_6 +	//128? maybe(check this) ADCLK cycles for sampling (775Hz sequence rate)
				ADC12SHT0_6 +
				ADC12REF2_5V +	//2.5V reference
				ADC12REFON +	//Reference on
				ADC12MSC + 		//Trigger sequential conversions automatically
				ADC12ON;		//ADC on
	ADC12CTL1 = ADC12DIV_7 +	//clock divider: 8
				ADC12SSEL_2 +	//clock source: MCLK
				ADC12SHP +		//Use sampling timer
				ADC12CONSEQ_1;	//Channel sequence, no repeat
	REFCTL0 = REFMSTR + REFVSEL_2 + REFON;	//2.5V reference
	ADC12MCTL0 = ADC12SREF_0 + ADC12INCH_8;	//5V sense
	ADC12MCTL1 = ADC12SREF_0 + ADC12INCH_9;	//12V sense
	ADC12MCTL2 = ADC12SREF_1 + ADC12INCH_10;	//MCU temp
	ADC12MCTL3 = ADC12EOS + ADC12SREF_1 + ADC12INCH_11;	//3V3 sense voltage
	ADC12IE = ADC12IE3;						//Enable ADC12 interrupt
	ADC12CTL0 |= ADC12SC+ADC12ENC;				//Start conversion
	return;
}


/** END ADC Task Functions **/

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

/* CAN SPI USCIB0 Interrupt Handler
 * SPI Rx:
 * SPI Tx:
 */
#pragma vector=USCI_B0_VECTOR
__interrupt void USCIB0_ISR(void){
	if((UCB0IE & UCRXIE) && (UCB0IFG & UCRXIFG)){				//SPI Rxbuf full interrupt
		CAN_SPI_data.rx_bytes[CAN_SPI_data.rx_ptr] = UCB0RXBUF;	//Get latest byte from HW
		CAN_SPI_data.rx_ptr++;									//Flag reset with buffer read
		if(CAN_SPI_data.rx_ptr >= CAN_SPI_data.num_bytes){		//Done reading data
			CAN_SPI_CS_DEASSERT;									//Disable CS and disable interrupt
			CAN_SPI_RXINT_DISABLE;
			CAN_SPI_data.data_ready = 1;
		} else {
			UCB0TXBUF = CAN_SPI_data.tx_bytes[CAN_SPI_data.tx_ptr];	//Load next byte into HW buffer
			CAN_SPI_data.tx_ptr++;
		}
	}
}

/* PC UART USCIA1 Interrupt Handler
 * UART Rx: Recieves incoming bytes from PC, puts into PC UART datastructure
 * UART Tx: Sends bytes from PC UART datastructure to PC
 */
#pragma vector=USCI_A1_VECTOR
__interrupt void USCIA1_ISR(void){
	if((UCA1IE & UCRXIE) && (UCA1IFG & UCRXIFG)){	//UART Rxbuf full interrupt
		//Get byte and clear interrupt
		PC_UART_data.rx_bytes[PC_UART_data.rx_head] = UCA1RXBUF;
		PC_UART_data.rx_head++;
		//Wraparound condition
		if(PC_UART_data.rx_head >= PC_UART_RX_BUF_SIZE){
			PC_UART_data.rx_head = 0;
		}
		if(PC_UART_data.rx_head == PC_UART_data.rx_tail)
			issue_warning(WARN_PC_RX_BUF_FULL);
	} else if((UCA1IE & UCTXIE) && (UCA1IFG & UCTXIFG)){	//UART Txbuf ready interrupt
		//Load data and clear interrupt
		UCA1TXBUF = PC_UART_data.tx_bytes[PC_UART_data.tx_tail];
		PC_UART_data.tx_tail++;
		//Wraparound condition
		if(PC_UART_data.tx_tail >= PC_UART_TX_BUF_SIZE){
			PC_UART_data.tx_tail = 0;
		}
		//Disable Tx interrupt if last byte in buffer has been transmitted
		if(PC_UART_data.tx_tail == PC_UART_data.tx_head){
			disable_pc_uart_txint();
		}
	} else {
		issue_warning(WARN_USCIA1_INT_ILLEGAL_FLAG);
		while(1);
	}
}

/* ADC12 Interrupt Vector
 * Sample analog data channels and voltage rail sense channels
 */
#pragma vector=ADC12_VECTOR
__interrupt void ADC12_ISR(void){
	/* Output buffer and internal buffer data order
	 * 0: A8 5Vsense
	 * 1: A9 12Vsense
	 * 2: A10 MCU temp (2.5V reference)
	 * 3: A11 3V3 sense (2.5V reference)
	 */
	//Gather conversions
	uint8_t i = 0;
	for(i=0; i<ADC_BUF_LEN; i++){
		adc_output_buffer[i] = ADC12MEM[i];
	}
	adc_data_ready = 0x0F;
	ADC12CTL0 &= ~ADC12SC;	//Clear SC bit
	ADC12CTL0 |= ADC12SC;	//Start conversion for next sequence
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
			//issue_warning(WARN_RST_SVSL);	//Occurs on powerup
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
