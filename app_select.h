/****************************************************************************
 * @file     app_select.h
 * @brief    Application Selector
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
#ifndef APP_SELECT_H
#define	APP_SELECT_H

// Application Selection (Choose ONLY one)
	#define		SENSORLESS
//	#define		HALL_SENSOR
//	#define		RC_PWM_THROTTLE

/****************************************************************************
 * Algorithm: 			Sensorless Trapezoidal with BEMF Commutation
 * Interface: 			UART based Serial communications
 * Supported Devices:	PAC5220, PAC5223, PAC5250, PAC5253
 * Supported HYDRA-X:	HYDRA-X20, HYDRA-X23, HYDRA-X23
****************************************************************************/
#ifdef 		SENSORLESS
#define		SENSORLESS_APP
#endif

/****************************************************************************
 * Algorithm: 			Sensored Trapezoidal with Hall Sensor Based Commutation
 * Interface: 			UART based Serial communications
 * Supported Devices:	PAC5220, PAC5223, PAC5250, PAC5253
 * Supported HYDRA-X:	HYDRA-X20, HYDRA-X23, HYDRA-X23
****************************************************************************/
#ifdef 		HALL_SENSOR
#define		HALL_SENSOR_APP
#endif

/****************************************************************************
 * Algorithm: 			Sensorless Trapezoidal with BEMF Commutation
 * Interface: 			RC PWM 50 Hz / 400 Hz
 * Supported Devices:	PAC5220, PAC5223, PAC5250, PAC5253
 * Supported HYDRA-X:	HYDRA-X20, HYDRA-X23, HYDRA-X23
****************************************************************************/
#ifdef		RC_PWM_THROTTLE
#define		RC_PWM_THROTTLE_APP
#endif


//app_select.h
#endif
