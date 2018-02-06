/****************************************************************************
 * @file     bldc_hw_select.h
 * @brief    BLDC Common Application Hardware Selection and Configurator
 * @date     30 October 2015
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

#ifndef HW_SELECT_H
#define	HW_SELECT_H

//Hardware Selection - Uncomment only one
//	#define	SENSORLESS_BLDCM1_2X	1				//Sensorless BEMF or RC PWM with BLDC Head (Single Shunt) and PAC5220/23	- TESTED
	#define	SENSORLESS_BLDCM1_5X	1				//Sensorless BEMF or RC PWM with BLDC Head (Single Shunt) and PAC5250/53	- TESTED
//	#define	SENSORLESS_FOCM1_2X		1				//Sensorless BEMF or RC PWM with FOC Head (3 Shunt) and PAC5220/23			- TESTED
//	#define	SENSORLESS_FOCM1_5X		1				//Sensorless BEMF or RC PWM with FOC Head (3 Shunt) and PAC5250/53			- TESTED
//	#define	SENSORLESS_TINY_BLDC	1				//Sensorless BEMF or RC PWM with Tiny BLDC (Single Shunt) and PAC5223		- TESTED
//	#define	SENSORLESS_TINY_FOC		1				//Sensorless BEMF or RC PWM with Tiny FOC (3 Shunt) and PAC5223				- UNTESTED
//	#define	SENSORED_BLDCM1_23		1				//Hall Sensor Trapezoidal with BLDC Head (Single Shunt) and PAC5223			- TESTED
//	#define	SENSORED_BLDCM1_20		1				//Hall Sensor Trapezoidal with BLDC Head (Single Shunt) and PAC5220			- TESTED
//	#define	SENSORED_BLDCM1_50		1				//Hall Sensor Trapezoidal with BLDC Head (Single Shunt) and PAC5250			- TESTED
//	#define	SENSORED_FOCM1_23		1				//Hall Sensor Trapezoidal with FOC Head (3 Shunt) and PAC5223
//	#define	SENSORED_FOCM1_20		1				//Hall Sensor Trapezoidal with FOC Head (3 Shunt) and PAC5220
//	#define	SENSORED_FOCM1_50		1				//Hall Sensor Trapezoidal with FOC Head (3 Shunt) and PAC5250

/****************************************************************************
 * HW     			EH-BLDCM1-1
 * SENSE Structure	Single Shunt 3 mOhms
 * VIN_SCALE ADC	PC3
 * PAC5xxx			PAC5220, PAC5223
 * Algorithms		Sensorless BEMF with UART or RC support
****************************************************************************/
#ifdef	SENSORLESS_BLDCM1_2X
#define	SINGLE_SHUNT		1
#define	ADC_VIN_PC3			1
#define	PAC5250				0
#define	PAC5223				0				//Don't Care
#define	CHECK_VBAT			1
#endif

/****************************************************************************
 * HW     			EH-BLDCM1-1
 * SENSE Structure	Single Shunt 3 mOhms
 * VIN_SCALE ADC	PC3
 * PAC5xxx			PAC5250, PAC5253
 * Algorithms		Sensorless BEMF with UART or RC support
****************************************************************************/
#ifdef	SENSORLESS_BLDCM1_5X
#define	SINGLE_SHUNT		1
#define	ADC_VIN_PC3			1
#define	PAC5250				1
#define	PAC5223				0				//Don't Care
#define	CHECK_VBAT			0
#endif

/****************************************************************************
 * HW     			EH-FOCM1-1
 * SENSE Structure	3 Shunt 0.01 Ohms
 * VIN_SCALE ADC	PC4
 * PAC5xxx			PAC5220, PAC5223
 * Algorithms		Sensorless BEMF with UART or RC support
****************************************************************************/
#ifdef	SENSORLESS_FOCM1_2X
#define	SINGLE_SHUNT		0
#define	ADC_VIN_PC3			0
#define	PAC5250				0
#define	PAC5223				0				//Don't Care
#define	CHECK_VBAT			1
#endif

/****************************************************************************
 * HW     			EH-FOCM1-1
 * SENSE Structure	3 Shunt 0.01 Ohms
 * VIN_SCALE ADC	PC4
 * PAC5xxx			PAC5250, PAC5253
 * Algorithms		Sensorless BEMF with UART or RC support
****************************************************************************/
#ifdef	SENSORLESS_FOCM1_5X
#define	SINGLE_SHUNT		0
#define	ADC_VIN_PC3			0
#define	PAC5250				1
#define	PAC5223				0				//Don't Care
#define	CHECK_VBAT			0				//PAC5250 does not have PC4 input
#endif

/****************************************************************************
 * HW     			TINI-BLDC
 * SENSE Structure	Single Shunt 3 mOhms
 * VIN_SCALE ADC	PC4
 * PAC5xxx			PAC5223
 * Algorithms		Sensorless BEMF with UART or RC support
****************************************************************************/
#ifdef	SENSORLESS_TINY_BLDC
#define	SINGLE_SHUNT		1
#define	ADC_VIN_PC3			0
#define	PAC5250				0
#define	PAC5223				0				//Don't Care
#define	CHECK_VBAT			1
#endif

/****************************************************************************
 * HW     			TINI-FOC
 * SENSE Structure	3 Shunt 0.01 Ohms
 * VIN_SCALE ADC	PC4
 * PAC5xxx			PAC5223
 * Algorithms		Sensorless BEMF with UART or RC support
****************************************************************************/
#ifdef	SENSORLESS_TINY_FOC
#define	SINGLE_SHUNT		0
#define	ADC_VIN_PC3			0
#define	PAC5250				0
#define	PAC5223				0				//Don't Care
#define	CHECK_VBAT			1
#endif

/****************************************************************************
 * HW     			EH-BLDCM1-1
 * SENSE Structure	Single Shunt 3 mOhms
 * VIN_SCALE ADC	PC3
 * PAC5xxx			PAC5223 (HYDRA-X23 / HYDRA-X23S)
 * Algorithms		Hall Sensor
****************************************************************************/
#ifdef	SENSORED_BLDCM1_23
#define	SINGLE_SHUNT		1
#define	ADC_VIN_PC3			1
#define	PAC5250				0
#define	PAC5223				1
#define	CHECK_VBAT			1
#endif

/****************************************************************************
 * HW     			EH-BLDCM1-1
 * SENSE Structure	Single Shunt 3 mOhms
 * VIN_SCALE ADC	PC3
 * PAC5xxx			PAC5220 (HYDRA-X20)
 * Algorithms		Hall Sensor
****************************************************************************/
#ifdef	SENSORED_BLDCM1_20
#define	SINGLE_SHUNT		1
#define	ADC_VIN_PC3			1
#define	PAC5250				0
#define	PAC5223				0
#define	CHECK_VBAT			1
#endif

/****************************************************************************
 * HW     			EH-BLDCM1-1
 * SENSE Structure	Single Shunt 3 mOhms
 * VIN_SCALE ADC	PC3
 * PAC5xxx			PAC5250
 * Algorithms		Hall Sensor
****************************************************************************/
#ifdef	SENSORED_BLDCM1_50
#define	SINGLE_SHUNT		1
#define	ADC_VIN_PC3			1
#define	PAC5250				1
#define	PAC5223				0
#define	CHECK_VBAT			1
#endif

/****************************************************************************
 * HW     			EH-FOCM1-1
 * SENSE Structure	3 Shunt 0.01 Ohms
 * VIN_SCALE ADC	PC4
 * PAC5xxx			PAC5223 (HYDRA-X23 / HYDRA-X23S)
 * Algorithms		Hall Sensor
****************************************************************************/
#ifdef	SENSORED_FOCM1_23
#define	SINGLE_SHUNT		0
#define	ADC_VIN_PC3			1
#define	PAC5250				0
#define	PAC5223				1
#define	CHECK_VBAT			1
#endif

/****************************************************************************
 * HW     			EH-FOCM1-1
 * SENSE Structure	3 Shunt 0.01 Ohms
 * VIN_SCALE ADC	PC4
 * PAC5xxx			PAC5220 (HYDRA-X20)
 * Algorithms		Hall Sensor
****************************************************************************/
#ifdef	SENSORED_FOCM1_20
#define	SINGLE_SHUNT		0
#define	ADC_VIN_PC3			1
#define	PAC5250				0
#define	PAC5223				0
#define	CHECK_VBAT			1
#endif

/****************************************************************************
 * HW     			EH-FOCM1-1
 * SENSE Structure	3 Shunt 0.01 Ohms
 * VIN_SCALE ADC	PC4
 * PAC5xxx			PAC5250
 * Algorithms		Hall Sensor
****************************************************************************/
#ifdef	SENSORED_FOCM1_50
#define	SINGLE_SHUNT		0
#define	ADC_VIN_PC3			0
#define	PAC5250				1
#define	PAC5223				0
#define	CHECK_VBAT			0				//PAC5250 does not have PC4 input
#endif


#endif
