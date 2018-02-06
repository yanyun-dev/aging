/****************************************************************************
 * @file     uart.h
 * @brief    Peripheral and initialization routines for the application.
 * @date     13 February 2015
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

#ifndef UART_H
#define UART_H

typedef enum
{
	admin_state_motor_enabled = 1
} AdminStateBits;

typedef enum
{
	success,									/*!< UART message received successfully */
	host_write_error,							/*!< UART error when writing from host to target (receive message) */
	host_read_error,							/*!< UART error when reading from target to host (transmit message) */
	checksum_error,								/*!< UART checksum error */
} Uart_Status;

#endif
