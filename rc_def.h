/****************************************************************************
 * @file     rc_def.h
 * @brief    Public interface to sensorless BLDC motor with RC Control support
 * @date     3 September 2015
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
#ifndef RC_DEF_H
#define RC_DEF_H

#define	MOTORPWM_RCPWM		1

typedef enum {
	RCGone = 0,
	RCPresent
} RCState;

//***********************************************************************
//RC Code Block Definition
#define	RCPWM_WIDTH_MIN		1563				//Min Number of Timer B ticks. 1563 * 640 ns = 1.0 ms
#define	RCPWM_WIDTH_MAX		3125				//Max Number of Timer B ticks. 3125 * 640 ns = 2.0 ms

#define	RC_PWM_50HZ			0					//0 for 50 HZ (20 ms period) or 1 for 400 Hz (2.5 ms Period)
#if RC_PWM_50HZ 								//RC PWM Frequency is 50 Hz (every 20 to 21 ms)
	#define	RCPWM_PERIOD_HI		33000				//21.12 ms (33000 * 640 ns = 21.12 ms)
	#define	RCPWM_PERIOD_LO		31000				//19.84 ms (31000 * 640 ns = 19.84 ms)
#else											//RC PWM Frequency is 400 Hz (every 2.5 ms)
	#define	RCPWM_PERIOD_HI		4000				//2.56 ms (4000 * 640 ns = 2.56 ms)
	#define	RCPWM_PERIOD_LO		3800				//2.43 ms (3800 * 640 ns = 2.432 ms)
#endif

#define MAX_PWM_PERIOD_COUNTS		5
#define	RC_MAX_SPEED_HZ				1500
#define	RCPWM_NORMALIZE_CONSTANT	FIX16(RC_MAX_SPEED_HZ/1563)	//2500 / 1563 = 1.599 in FIX16
#define	MTRPWM_MAX_DC				1000
#define	RCPWM_NORMALIZE_DC_CONSTANT	FIX16(MTRPWM_MAX_DC/1563)	//2450 / 1563 = 1.5674 in FIX16
#define FREQ_INV					FIX16(1/1562)		//1/1562 = 0.00064 in FIX16
#define	RC_SPEED_TH_HZ_ON			200
#define	RC_SPEED_TH_HZ_OFF			150

EXTERN uint8_t good_pulse;
EXTERN uint8_t rc_status;

EXTERN uint16_t prev_rcpwm_time_rise;
EXTERN uint16_t rcpwm_time_rise, rcpwm_time_fall, rcpwm_width;
EXTERN uint16_t rcpwm_period;
EXTERN uint16_t mtrpwm_width;
EXTERN int16_t rcpwm_period_counter;

EXTERN fix16_t rc_command;
EXTERN fix16_t rc_width_ms;
#endif
