#include "bldc_common.h"


void peripheral_init(void)
{
		pac5xxx_sys_ccs_config(CCSCTL_CLKIN_CLKREF, CCSCTL_ACLKDIV_DIV1, CCSCTL_HCLKDIV_DIV2);
		pac5xxx_sys_pll_config(100);
		pac5xxx_sys_ccs_pll_select();
		pac5xxx_memctl_wait_state(FLASH_WSTATE_75MHZ_LT_HCLK_LTE_100MHZ);

		// Configure Timer A parameters. Timer A runs at PLL Frequency (selected above).
		pac5xxx_timer_clock_config(TimerA, TxCTL_CS_ACLK, TxCTL_PS_DIV1);
		app_pwm_period = PWM_SWITCH_FREQ;												// Number of KHz
		pwm_period_ticks = TIMER_A_FREQ_CNV / (app_pwm_period);							// 50,000 / # KHz
		pac5xxx_timer_base_config(TimerA, pwm_period_ticks, 0, TxCTL_MODE_UP, 0);		// Configure Timer
}
