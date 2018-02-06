/**************************************************************************//**
 * @file     pac5xxx_sdk_version.h
 * @brief    SDK Version
 *
 * @note
 * Copyright (C) 2015, Active-Semi International
 *
 * @par
 * THIS SOFTWARE IS SUBJECT TO A SOURCE CODE LICENSE AGREEMENT WHICH PROVIDES, 
 * AMONG OTHER THINGS:  (i) THAT IT CAN BE USED ONLY TO ADAPT THE LICENSEE'S 
 * APPLICATION TO PAC PROCESSORS SUPPLIED BY ACTIVE-SEMI INTERNATIONAL; 
 * (ii) THAT  IT IS PROVIDED "AS IS" WITHOUT WARRANTY;  (iii) THAT 
 * ACTIVE-SEMICONDUCTOR IS NOT LIABLE FOR ANY INDIRECT DAMAGES OR FOR DIRECT
 * DAMAGES EXCEEDING US$1,500;  AND (iv) THAT IT CAN BE DISCLOSED TO AND USED
 * ONLY BY CERTAIN AUTHORIZED PERSONS.
 *
 ******************************************************************************/

#ifndef PAC5XXX_SDK_VERSION_H
#define PAC5XXX_SDK_VERSION_H

#define SDK_VERSION_MAJOR           0
#define SDK_VERSION_MINOR           0
#define SDK_VERSION_BUGFIX          0
#define SDK_VERSION_RELEASE_STATE   0  //0=Released, 1=RC, 2=Beta, 3=Alpha
#define SDK_VERSION_RELEASE_NUM     0
#define SDK_VERSION_STR				"Based On 2.2.1"

#endif /* PAC5XXX_SDK_VERSION_H */

/*******************************************************************************

Change Log 

===========================
PAC52XX SDK vx.x.x (based on 2.2.1)
===========================
New Features, Improvements
--------------------------
- SOCBridge: Added 16-bit writes/reads; now addr, R/W, and data are written with one 16-bit value
- SOCBridge: Modified the clock to run at 12.5 Mhz when HCLK is 50 MHz; previously it was running at 2.27 MHz

Fixed Bugs
----------
- SPI: pac5xxx_spi_slave_config2(): corrected registers for clock polarity and phase
- I2C: pac5xxx_i2c_master_write() and pac5xxx_i2c_master_write_byte()
    Corrected MCTRL writes to XFER_TYPE and AUTONO_XFER bits
- I2C: removed unecessary function declarations
    pac5xxx_i2c_int_enable_MST_A_NACK_ACK()
    pac5xxx_i2c_int_enable_MST_D_NACK_ACK()
    
===========================
PAC52XX SDK v2.2.1
===========================
New Features, Improvements
--------------------------
- cleaned up file headers
- Added #ifndef HEADER_FILE_H for .h files that didn't have them
- Restored default STACK_SIZE to 0x90
- Removed Start Up delay in SystemInit()
- SocBridge_IRQHandler label from CoIDE now used for IAR and Keil also
- Removed pac5xxx_sys_pll_config_100mhz() 
	for proper ADC operation over temperature, the PLL should be <= 80 MHz

Fixed Bugs
----------
- pac5xxx_UART.h: fixed bug re-introduced in SDK v2.2 that affects the Receive FIFO threshold
- pac52XX.h: fixed issue with IAR #define PAC5XXX_OPTIMIZE_SPEED_ON
- restored pac5xxx_rtc_config(int ie_en, uint16_t cdv, RTCCTL_PS_Type prescale);
		to pac5xxx_rtc_config(int ie_en, uint32_t cdv, RTCCTL_PS_Type prescale);
		the cdv is a 24-bit number and requires uint32_t
- set correct value for __MPU_PRESENT = 0

===========================
PAC52XX SDK v2.2
===========================
New Features, Improvements
--------------------------
- created unified SDK for CoIDE, IAR, and KEIL
- restructured folders and created separate boot folders for each IDE
- renamed pac5xxx.h to pac52XX.h
----- Changes from CoIDE SDK v2.1-----
	- removed Software SOC Bridge functions
	- pac5xxx_driver_socbridge.h: function pac5xxx_socbridge_config()
		fixed issue where enable variable wasn't being applied to SOCBCTL.SSEN
	- pac5xxx_driver_spi.h: corrected pac5xxx_SPI_master_config() to pac5xxx_spi_master_config()
	- pac5xxx_driver_timer.h: added pac5xxx_timer_X_set_mode() functions for timers a-d
	- pac5xxx_UART.h added FD_CTL register structure
	- slight update to file headers
----- Changes from IAR SDK v2.1-----
	- pac5xxx_driver_adc.h - Removed duplicate functions:
		pac5xxx_adc_as0_seq_get_result
		pac5xxx_adc_as1_seq_get_result
	- pac5xxx_driver_config.h - commented out all RAM Func #defines so nothing in RAM by default
	- pac5xxx_driver_rtc.h -
		pac5xxx_rtc_config(int ie_en, uint32_t cdv, RTCCTL_PS_Type prescale); is now
		pac5xxx_rtc_config(int ie_en, uint16_t cdv, RTCCTL_PS_Type prescale);
	- pac5xxx_driver_rtc_config.c -
		pac5xxx_rtc_config(int ie_en, uint32_t cdv, RTCCTL_PS_Type pre_scaler) is now
		pac5xxx_rtc_config(int ie_en, uint16_t cdv, RTCCTL_PS_Type pre_scaler)
	- pac5xxx_driver_system_ccs_config.c fixed FCLK bug in pac5xxx_sys_ccs_config()
	- pac5xxx_driver_system_pll_config_100mhz.c - pac5xxx_sys_pll_config_100mhz() now
		uses 100 for PLLCTL_FDBK_DIV(100) instead of 125.
	- pac5xxx_UART.h 
		added MCR struct offset 0x10
		added MSR struct offset 0x18
----- Changes from Keil SDK v1.0.0 -----
	- Added RAMFUNCs to all functions
	- pac5xxx_driver_rtc.h -
		pac5xxx_rtc_config(int ie_en, uint32_t cdv, RTCCTL_PS_Type prescale); is now
		pac5xxx_rtc_config(int ie_en, uint16_t cdv, RTCCTL_PS_Type prescale);
	- pac5xxx_driver_rtc_config.c -
		pac5xxx_rtc_config(int ie_en, uint32_t cdv, RTCCTL_PS_Type pre_scaler) is now
		pac5xxx_rtc_config(int ie_en, uint16_t cdv, RTCCTL_PS_Type pre_scaler)	
	- pac5xxx_driver_socbridge.h: function pac5xxx_socbridge_config()
		fixed issue where enable variable wasn't being applied to SOCBCTL.SSEN
	- pac5xxx_driver_system_pll_config_100mhz.c - pac5xxx_sys_pll_config_100mhz() now
		uses 100 for PLLCTL_FDBK_DIV(100) instead of 125.		
    - pac5xxx_driver_uart.h: 
		added pac5xxx_uart_msr_X() functions for configuring Modem Status Register
		added scratchpad data read and write functions
	- pac5xxx_driver_uart_config.c: added pac5xxx_uart_config_MCR() function
	- pac5xxx_UART.h - added
		added MCR struct offset 0x10
		added MSR struct offset 0x18


*****************************************************************************************/