/****************************************************************************
 * @file     bldc_common.h
 * @brief    Public interface to BLDC Common Application
 * @date     22 September 2015
 *
 * @note
 * Copyright (C) 2015, Active-Semi International
 *
 * THIS SOFTWARE IS SUBJECT TO A SOURCE CODE LICENSE AGREEMENT WHICH PROVIDES,
 * AMONG OTHER THINGS:  (i) THAT IT CAN BE USED ONLY TO ADAPT THE LICENSEE'S
 * APPLICATION TO PAC PROCESSORS SUPPLIED BY ACTIVE-SEMI INTERNATIONAL;
 * (ii) THAT  IT IS PROVIDED "AS IS" WITHOUT WARRANTY;  (iii) THAT
 * ACTIVE-SEMICONDUCTOR IS NOT LIABLE FOR ANY INDIRECT DAMAGES OR FOR DIRECT
 * DAMAGES EXCEEDING US$1,500;  AND (iv) THAT IT CAN BE DISCLOSED TO AND USED
 * ONLY BY CERTAIN AUTHORIZED PERSONS.
 ******************************************************************************/

#ifndef BLDC_COMMON_H
#define BLDC_COMMON_H

#ifndef INCLUDE_EXTERNS
#define	EXTERN	volatile
#else
#define	EXTERN	extern	volatile
#endif

#include "pac5xxx_driver_adc.h"
#include "pac5xxx_driver_timer.h"
#include "pac5xxx_driver_socbridge.h"
#include "pac5xxx_driver_tile.h"
#include "pac5xxx_driver_system.h"
#include "pac5xxx_driver_memory.h"
#include "pac5xxx_driver_uart.h"
#include "pac5xxx_driver_spi.h"
#include "app_select.h"
#include "bldc_hw_select.h"
#include "uart.h"
#include "fix16.h"
#include "version.h"
#include "pid.h"
#include "motordef.h"
#include "state_machine.h"
#include "rc_def.h"

#define FIX16(n)	((long)(65536.0 * n ))

#define	IREG				0							// Current Regulation PI Loop
#define	STALL_DETECT		0							//modified by Lance
#define	NODC				0							// 0 for BUCK or SEPIC, 1 for No Switching Regulation
#define	TUNING				0							// For DEBUG
#define	UPDATE_DEBUG		1							// 1 for running the Update Debug functionality
#define	POT_CONTROL			0							// 1 for using potentiometer control
#define	COMM_SUPPORT		1							// 1 to include UART Support, 0 to remove from code space
#define	SPI_SUPPORT			0

#define APP_RAMFUNC_LINK
#ifdef APP_RAMFUNC_LINK
#define APP_RAMFUNC PAC5XXX_RAMFUNC
#else
#define APP_RAMFUNC
#endif

#define ADC_CHANNEL_MASK	((1L << ADCCTL_ADMUX_AD0) | (1L << ADCCTL_ADMUX_AD2) | (1L << ADCCTL_ADMUX_AD3) | (1L << ADCCTL_ADMUX_AD4))	// Mask of ADC channels to perform conversions for
#define	ADC_SEQ_EDATA		SIGMGR_MSPI_SH_COMP										// Latch for the comparator output (SH1 on EMUX) Selecting AD0 on ADCMux, although irrelevant
#define	VIN_VOLTS_VAL		PAC5XXX_ADC->AS0R0.VAL

#if SINGLE_SHUNT
#define ADC_SEQ_VIN				0													// AS0 sequence number for VIN
#define	ADC_SEQ_HBI0			1													// AS0 sequence number for H Bridge Current (HBI)
#define	ADC_SEQ_HBI1			2													// AS0 sequence number for H Bridge Current (HBI)
#define	ADC_SEQ_HBI2			3													// AS0 sequence number for H Bridge Current (HBI)
#define	ADC_SEQ_HBI3			4													// AS0 sequence number for H Bridge Current (HBI)
#define	ADC_SEQ_VPOT			5													// AS0 sequence number for Potentiometer (VPOT)
#define	ADC_SEQ_HBI_EDATA		SIGMGR_MSPI(SIGMGR_AIO10)
#define	HBI0_ADC_VAL			PAC5XXX_ADC->AS0R1.VAL
#define	HBI1_ADC_VAL			PAC5XXX_ADC->AS0R2.VAL
#define	HBI2_ADC_VAL			PAC5XXX_ADC->AS0R3.VAL
#define	HBI3_ADC_VAL			PAC5XXX_ADC->AS0R4.VAL
#define	POT_VOLTS_VAL			PAC5XXX_ADC->AS0R5.VAL
#else
#define ADC_SEQ_VIN				0													// AS0 sequence number for VIN
#define	ADC_SEQ_U1				1													// AS0 sequence number for H Bridge Current (HBI)
#define	ADC_SEQ_V1				2													// AS0 sequence number for H Bridge Current (HBI)
#define	ADC_SEQ_W1				3													// AS0 sequence number for H Bridge Current (HBI)
#define	ADC_SEQ_VPOT			4													// AS0 sequence number for H Bridge Current (HBI)
#define	ADC_SEQ_U2				5													// AS0 sequence number for Potentiometer (VPOT)
#define ADC_SEQ_V2				6													// AS0 sequence number for VIN
#define	ADC_SEQ_W2				7													// AS0 sequence number for H Bridge Current (HBI)
#define	ADC_SEQ_HBU_EDATA		SIGMGR_MSPI(SIGMGR_AIO10)
#define	ADC_SEQ_HBV_EDATA		SIGMGR_MSPI(SIGMGR_AIO32)
#define	ADC_SEQ_HBW_EDATA		SIGMGR_MSPI(SIGMGR_AIO54)
#define	U1_ADC_VAL				PAC5XXX_ADC->AS0R1.VAL
#define	V1_ADC_VAL				PAC5XXX_ADC->AS0R2.VAL
#define	W1_ADC_VAL				PAC5XXX_ADC->AS0R3.VAL
#define	POT_VOLTS_VAL			PAC5XXX_ADC->AS0R4.VAL
#define	U2_ADC_VAL				PAC5XXX_ADC->AS0R5.VAL
#define	V2_ADC_VAL				PAC5XXX_ADC->AS0R6.VAL
#define	W2_ADC_VAL				PAC5XXX_ADC->AS0R7.VAL
#endif



typedef enum
{
	TimerC_idle = 0,
	TimerC_Align_And_Go,
	TimerC_getslcomp_first_sample,
	TimerC_getslcomp_samples,
	TimerC_getslcomp_commutation_wait,
	TimerC_getslcomp_blanking_cycles
}TimerC_States;

typedef enum
{
	status_motor_enabled = 1,
	status_over_current = 2,
	status_motor_stalled = 4,
	status_closed_loop = 8,
	status_under_voltage = 16,
	status_crc_test_check = 32
} StatusStateBits;

// PLL and Clock System Configuration
#define	PLL_FREQ_100			1						// 1 for PLL @ 100 MHz, 0 for PLL @ 50 MHz
#if PLL_FREQ_100
#define	TIMER_A_FREQ_CNV		100000
#else
#define	TIMER_A_FREQ_CNV		50000
#endif

// System's Definitions
#if PAC5250
#define MOTOR_PWM_PIN_PORT_A	0xCF
#define MOTOR_PWM_PIN_PORT_D	0x80
#else
#define MOTOR_PWM_PIN_PORT_A	0x3F
#endif

#define DT_LED_TICKS			50
#define DT_TED_TICKS			50
#define	SAMPLE_DELAY_DEF		150
#define NIRQ1_PIN_MASK			0x01
#define NIRQ2_PIN_MASK			0x80
#define SLCOMP7					0xD0					//AIO7 - PHASE U to Comparator POS
#define SLCOMP8					0xE0					//AIO8 - PHASE V to Comparator POS
#define SLCOMP9					0xF0					//AIO9 - PHASE W to Comparator POS
#define	TIMER_D_FREQ_F16		0xBEBC2000				//TMRD Freq = 50MHz (This number is divided by 1024 so it can be represented by a Fix16 type)
#define	STALL_DETECT_DEF		10000
#define	ONE_SIXTH_F16			FIX16(1/6)
#define	ONEDIV60				FIX16(1/60)
#define	DEBUG_MAX				60

//***********************************************************************
// Voltage Scaling
// Scale Factor = 2.5 * (RTOP + RBOT) / (RBOT * 1023)
#define RTOP					100
#define RBOT					6.2
#define VOLT_DIV_FIX16			FIX16((RTOP + RBOT) * 2.5 / (RBOT * 1023))

//***********************************************************************
// VIN Checking - if VIN readings fall below VIN_VOLTS_LEGAL motor is disabled
// and marked as stalled
#define	VIN_VOLTS_LEGAL			FIX16(12.2)
#define	VIN_CHECK_DEB_MAX		100

//***********************************************************************
// Potentiometer Driving - if using a potentiometer to drive motor speed
#define	TURN_ON_THRESHOLD_MIN	60
#define	TURN_ON_THRESHOLD_MAX	100
#define	MAX_HZ					512

//***********************************************************************
// BEEP ON Defines
#define BEEP_FREQ_HZ_DEF		1000
#define BEEP_PWM_DC_DEF			10

//***********************************************************************
// Debug Quick Functions
#define	DEBUG_E0_R		PAC5XXX_GPIOE->OUT.b |= 0x01;
#define	DEBUG_E0_F		PAC5XXX_GPIOE->OUT.b &= ~0x01;

#define	DEBUG_E3_R		PAC5XXX_GPIOE->OUT.b |= 0x08;
#define	DEBUG_E3_F		PAC5XXX_GPIOE->OUT.b &= ~0x08;

#define	DEBUG_E4_R		PAC5XXX_GPIOE->OUT.b |= 0x10;
#define	DEBUG_E4_F		PAC5XXX_GPIOE->OUT.b &= ~0x10;

#define	DEBUG_E5_R		PAC5XXX_GPIOE->OUT.b |= 0x20;
#define	DEBUG_E5_F		PAC5XXX_GPIOE->OUT.b &= ~0x20;

#define	OPTIMIZE_O0		__attribute__((optimize("O0")))
#define	OPTIMIZE_O1		__attribute__((optimize("O1")))
#define	OPTIMIZE_O2		__attribute__((optimize("O2")))
#define	OPTIMIZE_O3		__attribute__((optimize("O3")))
#define	OPTIMIZE_Os		__attribute__((optimize("Os")))

// Function Prototypes
void speed_control_loop(void);
void check_vbat(void);
void check_adc(void);
void motor_pwm_disable(void);
void app_init(void);
void peripheral_init(void);
void cafe_init(void);
void adc_init(void);
void UART_init(void);
void SPI_Init(void);
void Set_Dead_Time(void);
void UpdateDebug(void);
void oc_reset(void);
void pac5xxx_tile_register_write_in(uint8_t address, uint8_t data);
void pi_init(void);
fix16_t HertzToTicks(fix16_t Hertz, uint32_t Freq);
fix16_t HertzToTicksSine(fix16_t Hertz, uint32_t Freq);

EXTERN PID_Data_Type iq_pid;
EXTERN PID_Data_Type speed_pid;

EXTERN uint8_t good_samples;
EXTERN uint8_t timer_d_div;
EXTERN uint8_t avg_speed_index;
EXTERN uint8_t tmp_pi_debug_index, pi_debug_index;
EXTERN uint8_t open_loop;
EXTERN uint8_t SMS_State;
EXTERN uint8_t vin_check_debounce;
EXTERN uint8_t debug_index;
EXTERN uint8_t diag_message_offset, diag_note_offset;

#ifdef HALL_SENSOR_APP
EXTERN uint8_t hall_sensor_value;
EXTERN uint8_t old_hall_sensor_value;
EXTERN uint8_t next_commutation_state;
#endif

EXTERN uint16_t dt_leading_ticks, dt_trailing_ticks;
EXTERN uint16_t pwm_period_ticks;								/*!< Number of timer A ticks for PWM period */
EXTERN uint16_t sample_delay;
EXTERN uint16_t SMS_Counter;
EXTERN uint16_t speed_ref_hz, speed_ref_command_hz;
EXTERN uint16_t ol_accel_period, ol_accel_increase;
EXTERN uint16_t cl_accel_period, tmp_cl_accel, cl_accel_increase;
EXTERN uint16_t stall_speed_ticks, stall_detect_time_ms;
EXTERN uint16_t start_iq_ref;
EXTERN uint16_t align_time_ms;
EXTERN uint16_t ol_start_hz;
EXTERN uint16_t ol_switchover_hz;
EXTERN uint16_t sine_index;
EXTERN uint16_t beep_freq_hz;
EXTERN uint16_t beep_pwm_dc;

EXTERN uint32_t pwm_duty;  //modified by Lance
EXTERN uint32_t sine_mode;
EXTERN uint32_t ADC_Counter;
EXTERN uint32_t single_shunt_current;
EXTERN uint32_t sample;
EXTERN uint32_t app_status;
EXTERN uint32_t app_pwm_period;
EXTERN uint32_t app_over_current_limit;
EXTERN uint32_t stall_counter;
EXTERN uint32_t debug_1;
EXTERN uint32_t debug_2;
EXTERN uint32_t debug_3;
EXTERN uint32_t debug_4;
EXTERN uint32_t timer_d_latch_in;
EXTERN uint32_t timer_d_offset;
EXTERN uint32_t last_sample_stored;
EXTERN uint32_t samplecounter;
EXTERN uint32_t getslcomp_state;
EXTERN uint32_t millisecond;
EXTERN uint32_t blanking_cycles, tmp_blanking_cycles;
EXTERN uint32_t commutation_time;
EXTERN uint32_t sl_current_state;
EXTERN uint32_t motorspeed;
EXTERN uint32_t avg_speed_array[6];

EXTERN int32_t CRC_Test_Check;

EXTERN fix16_t debug_array[DEBUG_MAX];
EXTERN fix16_t vin_volts;
EXTERN fix16_t pot_volts;
EXTERN fix16_t avg_speed;
EXTERN fix16_t pi_debug_value[2][5];
EXTERN fix16_t iq_ref, iq_ref_command;
EXTERN fix16_t speed_ref_ticks, speed_ref_command;
EXTERN fix16_t closed_loop_speed_hz;
EXTERN fix16_t commutation_advanced_rise;
EXTERN fix16_t commutation_advanced_fall;
EXTERN fix16_t stall_speed_hz;
EXTERN fix16_t sine_scale;
EXTERN fix16_t sine_scale_increase;

extern const uint16_t beep_notes[128];
extern const uint8_t diag_tunes[4][4];
#if defined SENSORLESS_APP || defined RC_PWM_THROTTLE_APP
	extern const fix16_t sine_wave_3phase[360][3];
#endif

// BLDC DIRECTION 1 LOGIC
//STATE TABLE		PWM-HI			PWM-LO			LO-SIDE		CurrentFlow		TriState	Comparator	Floating Phase	Polarity
//0					PWM4-A3(UH)		PWM0-A0(UL)		A1(VL)		U->V			W			SLCOMP9		W-Rising		0
//1					PWM6-A5(WH)		PWM2-A2(WL)		A1(VL)		W->V			U			SLCOMP7		U-Falling		1
//2					PWM6-A5(WH)		PWM2-A2(WL)		A0(UL)		W->U			V			SLCOMP8		V-Rising		0
//3					PWM5-A4(VH)		PWM1-A1((VL)	A0(UL)		V->U			W			SLCOMP9		W-Falling		1
//4					PWM5-A4(VH)		PWM1-A1((VL)	A2(WL)		V->W			U			SLCOMP7		U-Rising		0
//5					PWM4-A3(UH)		PWM0-A0(UL)		A2(WL)		U->W			V			SLCOMP8		V-Falling		1

// BLDC DIRECTION 0 LOGIC
//STATE TABLE		PWM-HI			PWM-LO			LO-SIDE		CurrentFlow		TriState	Comparator	Floating Phase	Polarity
//0					PWM4-A3(UH)		PWM0-A0(UL)		A2(WL)		U->W			V			SLCOMP8		V-Falling		1
//1					PWM5-A4(VH)		PWM1-A1((VL)	A2(WL)		V->W			U			SLCOMP7		U-Rising		0
//2					PWM5-A4(VH)		PWM1-A1((VL)	A0(UL)		V->U			W			SLCOMP9		W-Falling		1
//3					PWM6-A5(WH)		PWM2-A2(WL)		A0(UL)		W->U			V			SLCOMP8		V-Rising		0
//4					PWM6-A5(WH)		PWM2-A2(WL)		A1(VL)		W->V			U			SLCOMP7		U-Falling		1
//5					PWM4-A3(UH)		PWM0-A0(UL)		A1(VL)		U->V			W			SLCOMP9		W-Rising		0

#ifndef INCLUDE_EXTERNS
#if BLDC_DIRECTION
	#if PAC5250
		EXTERN const uint16_t psel_mask[6] = {0x1001, 0x0010, 0x0010, 0x4004, 0x4004, 0x1001};
		EXTERN const uint16_t psel_mask_port_d[6] = {0x0005, 0x4005, 0x4005, 0x0005, 0x0005, 0x0005};
	#else
		EXTERN const uint16_t psel_mask[6] = {0x0081, 0x0410, 0x0410, 0x0104, 0x0104, 0x0081};
	#endif
	EXTERN const uint8_t c_pwm_io_state[6] = {0x02, 0x02, 0x01, 0x01, 0x04, 0x04 };
	EXTERN const uint8_t slcomp_mux[6] = {SLCOMP9, SLCOMP7, SLCOMP8, SLCOMP9, SLCOMP7, SLCOMP8};
	EXTERN const uint32_t slcomp_cross_polarity[6] = {0x80, 0, 0x80, 0, 0x80, 0};
	//EXTERN const uint8_t hs_to_commutation[8] = {0,3,5,4,1,2,0,0};
	EXTERN const uint8_t hs_to_commutation[8] = {0,0,2,1,4,5,3,0};
#else
	#if PAC5250
		EXTERN const uint16_t psel_mask[6] = {0x1001, 0x4004, 0x4004, 0x0010, 0x0010, 0x1001};
		EXTERN const uint16_t psel_mask_port_d[6] = {0x0005, 0x0005, 0x0005, 0x4005, 0x4005, 0x0005};
	#else
		EXTERN const uint16_t psel_mask[6] = {0x0081, 0x0104, 0x0104, 0x0410, 0x0410, 0x0081};
	#endif

	EXTERN const uint8_t c_pwm_io_state[6] = {0x04, 0x04, 0x01, 0x01, 0x02, 0x02};
	EXTERN const uint8_t slcomp_mux[6] = {SLCOMP8, SLCOMP7, SLCOMP9, SLCOMP8, SLCOMP7, SLCOMP9};
	EXTERN const uint32_t slcomp_cross_polarity[6] = {0x80, 0, 0x80, 0, 0x80, 0};
	EXTERN const uint8_t hs_to_commutation[8] = {0,3,5,4,1,2,0,0};
#endif

#else
	EXTERN const uint8_t slcomp_mux[6];
	EXTERN const uint8_t c_pwm_io_state[6];
	EXTERN const uint16_t psel_mask[6];
	EXTERN const uint32_t slcomp_cross_polarity[6];
	EXTERN const uint8_t hs_to_commutation[8];
	#if PAC5250
		EXTERN const uint16_t psel_mask_port_d[6];
	#endif
#endif

#endif
