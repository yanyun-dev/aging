/****************************************************************************
 * @file     version.h
 * @brief    Version number for the SIMPLE BLDC Firmware
 * @date     12 May 2015
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

#ifndef VERSION_H
#define VERSION_H

#define VERSION_TYPE				1
#define VERSION_MAJOR				5
#define VERSION_MINOR				0

#endif

/*
 * Version History
 *
 * Version 1.5.0 June 2016
 * 1. Added support for IAR Compiler
 * 2. (bldc_common.h) Changed the order of the #define checks so the EXTERN definition happens before the inclusion check takes place.
 * 3. (bldc_common.h) Moved the array definitions to their own c file - arrays.c
 * 4. (arrays.c) File created to move the commutation arrays out of the bldc_common.h file
 * 5. (all c files) added the #define INCLUDE_EXTERNS to properly select the bldc_common.h file state.
 *
 * Version 1.4.0 May 2016
 * 1. SOCBridge SPI Communications Speed Update - Changed from 8 bits top 16 bits packet - Changed SPICLK Freq from 2.7 MHz to 12.5 MHz.
 * 		(pac5xxx_SocBridge.h) added WL bit definition to the SOBCFG structure definition
 * 		(pac5xxx_driver_tile_soc.c) Modified the read and write functions from dual 8 bit packet to single 16 bit packet
 * 		(pac5xxx_driver_socbridge.c) Added the instruction to configure the SPI block for 16 bit packet size
 *
 * Version 1.3.0 March 2016
 *
 * 1. Added BEEP Support (new files beep_notes.c and diag_tunes.c)
 * 2. (uart.c) added parm_beep_freq, parm_beep_on and parm_beep_off.
 * 3. (sl_state_machine.c) added cases to support BEEP ON and BEEP OFF commands.
 * 4. (SineWaveLUT.c) Changed Sine Wave Commutation Look up table angle offset.
 * 		Previous version had PHASE U starting at 90 degrees offset.
 * 		Current version has PHASE U starting at offset 0
 * 5. (hs_state_machine.c) Added speed initialization to the hall sensor start state.
 *
 * Version 1.2.0 February 2016
 *
 * 1. Added support for flash_crc_test
 * 		(startup_pac5xxx_CooCox.c) updated for flash_crc_test
 * 		(main.c) Added CRC_Test_Check validation after initialization
 * 		(hsfunction.c) Added CRC_Test_Check to UpdateDebug
 * 		(sl_functions.c) Added CRC_Test_Check to UpdateDebug
 * 		(rc_functions.c) Added CRC_Test_Check to UpdateDebug
 *		(bldc_common.h) Added CRC_Test_Check flag to Application Status Bits
 * 2. Updated SDK Files (library was eliminated as SDK Source Code files are now included)
 * 3. (isr_TimerC.c) fixed a bug by removing ";" from the if statement making it validate as true always
 * 4. (uart.c) fixed a bug on parm_speed_Ki where the TD was not being updated properly
 *
 * Version 1.1.3 January 2016
 *
 * 1. (bldc_common.c) Changed GPIOB IRQ from using function calls to direct register access.
 * 2. (bldc_common.h) Changed uint_8 and uint16_t variables employed on TIMERC's ISR to uint32_t
 * 3. (hs_peripheral_init and sl_peripheral_init.c) added #if SPI_SUPPORT to PORTE enable function
 * 4. (uart.c) parm_speed_ki is not updating TD
 *
 *
 * Version 1.1.2 December 18, 2015
 *
 * 1. Added the file "spi.c" which offers support for SPI communications.
 * 2. init.c now includes SPI initialization code
 * 3. main.c offers an option to enable either UART or SPI. Only one should be chosen as both features are mutually exclusive
 *
 *Version 1.1.1 November 5, 2015
 *
 * 1. Added proper licensing information to fix16.c and fix16.h

 *Version 1.1.0 November 5, 2015
 *
 * 1. Added "bldc_hw_select.h" include file to select which HYDRA-Xxx board and which HYDRA-X BLDC/FOC head are being employed. Special configurations for Tiny BLDC and Tiny FOC are included.
 * 		a. Moved SINGLE_SHUNT, ADC_VIN_PC3, CHECK_VBAT, PAC5250 and PAC5223 #defines from bldc_common.h to this new file
 * 2. Trapezoidal commutation now supports both single shunt as well as three shunt current regulation.
 * 3. Added ADC memory location references as #define directives instead of PAC5XXX_ADC->AS0Rx.val references
 * 4. ADC_Init (init.c) is now divided between single shunt or 3 shunt modes.
 * 5. cafe_init (init.c) contains the differential amplifier configuration for both single shunt and 3 shunt modes.
 * 6. ADC_IRQHandler (isr_adc) contains a single ISR call for all FW variants, instead of one for sensorless and one for sensored.
 *
 * Version 1.0.0 September 29, 2015
 *
 * 1. Major Application Overhaul
 * 2. Integrated three Application FW's into one: BEMF Sensorless with UART Support, BEMF Sensorless with RC PWM Support and Hall Sensor BLDC
 * 3. app_select.h is used to select from the three apps.
 * 4. Timer ISR's were separated into different files
 * 5. sl_xxxxxx stands for sensorless with UART support
 * 6. hs_xxxxxx stands for Hall Sensor BLDC
 * 7. rc_xxxxxx stands for sensorless with RC PWM support
 *
 * Version 0.2.0 September 10, 2015
 *
 * 1. Added VBAT Check
 * 2. Moved TIMER B function to the main loop. Main loop is already running at 1 ms rate.
 * 	  Timer B is used on the RC PWM version of the FW. Timer B no longer used on this FW.
 * 3. Systick Handler is now priority 3. It was causing a problem with the systick being called during BEMF sensing.
 * 4. Removed __disable_irq() and __enable_irq() from the Systick ISR
 * 5. On TimerISR.c, TIMERC's ISR sample is only made once for all Switch Case statements.
 * 6. debug_array is now a fix16_t. debug_index is uint8_t.
 * 7. Cleanup to make compatible with the RC PWM THROTLE FW.
 * 8. Added PAC5250 support
 * 9. Fixed bug on UART.C for updating IQ_PID.MAX_VALUE. Previous version updates SPEED_PID.
 * 10. Fixed Dead Time parameters on sensorless.h. Parameters were at 25 which causes shoot-through.
 * 11. Fixed Over Current Limit from 200 (was an overflow) to 50.
 * 12. Added a closed loop flag.
 * 13. Control Loop only runs when the closed loop flag is set.
 * 14. VBAT Check only runs when the motor enabled flag is set.
 *
 * Version 0.1.0 June 26, 2015
 *
 * 1. Added interrupt enable/disable structure during Commutation Schedule Initialization (TimerC ISR on TimerISR.c)
 * 2. Added support for PLL = 100 MHz (init.c)
 * 3. Fixed Sine Wave Commutation PWM update in order to support 100 KHz PWM Switching Frequency (main.c)
 * 4. Fixed Angle Advanced Computation. Angle Advance Factor is computed once during UART update. (UART.c and TimerISR.c)
 * 5. Divided header file #defines. Motor specific definitions are not included within motordef.h
 * 6. parm_pwm_period on UART.c updates IQ PI TD parameter (UART.C)
 * 7. Moved ADCInit() to initialization. ADC runs at all times to measure VIN. Remove ADC Interrupt Disable during MotorDisable()
 *
 * Version 0.0.2 May 13, 2015
 *
 * 1. Improved current regulation by adding multiple current readings from ADC and averaging during ADC ISR.
 * 2. Divided Accel/Decel profiles between Open Loop and Closed Loop modes (cl_accel_period, cl_accel_increase, ol_accel_period, ol_accel_increase)
 * 3. Added compile time option to change motor rotation direction (BLDC_DIRECTION)
 * 4. Added Stall Detection option on TIMERC ISR ( STALL_DETECT)
 * 5. Other code clean up
 *
 * Version 0.0.1 Initial Release September 12, 2014
 *
 */
