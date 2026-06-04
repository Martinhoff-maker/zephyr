/* Copyright (c) 2024-2026 Silicon Laboratories Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT silabs_siwx91x_aon_clock_manager

#include <zephyr/dt-bindings/clock/silabs/siwx91x-clock.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/logging/log.h>

#include "si91x_device.h"

#include "clock_update.h"
#include "rsi_power_save.h"
#include "rsi_sysrtc.h"

#include "sl_si91x_power_manager.h"

// temporary until we have a better way to share these headers between clock control and the rest of
// the driver
#include "rsi_rom_ulpss_clk.h"
#include "rsi_rom_clks.h"
#include "sl_status.h"
#include "rsi_m4.h"
#include "rsi_ipmu.h"
#define INTF_PLL_FREQUENCY 160000000
#define SOC_PLL_FREQ       (180000000UL) // 180MHz default SoC PLL Clock as source to Processor

#define PLL_PREFETCH_LIMIT     (120000000UL) // 120MHz Limit for pll clock
#define MAX_PLL_FREQUENCY      (180000000UL) ///< Max PLL frequency is 180MHz
#define LF_FSM_CLOCK_FREQUENCY 32768
#define MANUAL_LOCK            1    // Manual lock enable
#define BYPASS_MANUAL_LOCK     1    // Bypass manual lock enable
#define SOC_PLL_MM_COUNT_LIMIT 0xA4 // Soc pll count limit

#define QSPI_DIV_FACTOR  1
#define QSPI_ODD_DIV_EN  0 // Odd division enable for QSPI clock
#define QSPI_SWALLO_EN   0 // Swallo enable for QSPI clock
#define QSPI2_DIV_FACTOR 1 // Division factor for QSPI2 clock

#define PS4_PERFORMANCE_MODE_SOC_FREQ  (180000000UL) // PS4 high power soc pll clock frequency
#define PS4_PERFORMANCE_MODE_INTF_FREQ (160000000UL) // PS4 high power intf pll clock frequency
#define PS4_POWERSAVE_MODE_FREQ        (100000000UL) // PS4 low power clock frequency
#define PS3_PERFORMANCE_MODE_FREQ      (80000000UL)  // PS3 high power clock frequency
#define PS3_POWERSAVE_MODE_FREQ        (40000000UL)  // PS3 low power clock frequency

LOG_MODULE_REGISTER(siwx91x_aon_clock, CONFIG_CLOCK_CONTROL_LOG_LEVEL);

typedef enum HP_REF_CLK {
	HP_REF_CLK_DISABLED = 0,
	HP_REF_ULP_MHZ_RC_BYP_CLK = 1,
	HP_REF_ULP_MHZ_RC_CLK = 2,
	HP_REF_EXT_40MHZ_CLK = 3,
} HP_REF_CLK_T;

typedef enum ULP_REF_CLK {
	ULP_REF_CLK_DISABLED = 0,
	ULP_REF_ULP_MHZ_RC_BYP_CLK = 1,
	ULP_REF_ULP_MHZ_RC_CLK = 2,
	ULP_REF_EXT_40MHZ_CLK = 3,
} ULP_REF_CLK_T;

typedef enum M4_SOC_CLK {
	M4_ULP_REF_CLK = 0,
	M4_RESERVED_CLK = 1,
	M4_SOC_PLL_CLK = 2,
	M4_MODEM_PLL_CLK1 = 3,
	M4_INTF_PLL_CLK = 4,
	M4_SLEEP_CLK = 5,
} M4_SOC_CLK_T;

typedef enum PLL_CLK {
	SOC_PLL_CLK = 0,
	INTF_PLL_CLK = 1,
	I2S_PLL_CLK = 2,
	MODEM_PLL_CLK = 3,
} PLL_CLK_T;

typedef enum PLL_REF_CLK {
	PLL_REF_CLK_XTAL = 0,
	PLL_REF_CLK_RC = 1,
} PLL_REF_CLK_T;

struct siwx91x_aon_clock_data {
	uint32_t enable;
};

static void siwx91x_aon_clock_request_xtal_to_nwp(void)
{
	/* Will be use in the futur when communication will be clearer*/
	__maybe_unused const struct device *nwp_dev = DEVICE_DT_GET(DT_NODELABEL(nwp));

	/* Should be something like this function but need to call NWP device API in the futur*/
	if (nwp_dev) {
		LOG_DBG("Requesting XTAL clock from NWP");
		sli_si91x_xtal_turn_on_request_from_m4_to_TA();
	} else {
		LOG_ERR("NWP device not found, cannot request XTAL clock");
	}
	return;
}

static int siwx91x_hp_ref_clk_config(HP_REF_CLK_T clkSource)
{
	switch (clkSource) {
	case HP_REF_CLK_DISABLED:
		MCU_FSM->MCU_FSM_REF_CLK_REG_b.M4SS_REF_CLK_SEL = clkSource;
		system_clocks.m4_ref_clock_source = clkSource;
		system_clocks.m4ss_ref_clk = 0;
		break;
	case HP_REF_ULP_MHZ_RC_BYP_CLK:
		MCU_FSM->MCU_FSM_REF_CLK_REG_b.M4SS_REF_CLK_SEL = clkSource;
		system_clocks.m4_ref_clock_source = clkSource;
		system_clocks.m4ss_ref_clk = system_clocks.byp_rc_ref_clock;
		break;
	case HP_REF_ULP_MHZ_RC_CLK:
		MCU_FSM->MCU_FSM_REF_CLK_REG_b.M4SS_REF_CLK_SEL = clkSource;
		system_clocks.m4_ref_clock_source = clkSource;
		system_clocks.m4ss_ref_clk = system_clocks.rc_mhz_clock;
		break;
	case HP_REF_EXT_40MHZ_CLK:
		MCU_FSM->MCU_FSM_REF_CLK_REG_b.M4SS_REF_CLK_SEL = clkSource;
		system_clocks.m4_ref_clock_source = clkSource;
		system_clocks.m4ss_ref_clk = system_clocks.rf_ref_clock;
		break;
	default:
		return -EINVAL;
	}

	/* wait for clock switched - before they are waiting for the pll - ulp ref clock change ?*/
	// while ((pCLK->PLL_STAT_REG_b.ULP_REF_CLK_SWITCHED) != true)
	// 	;
	while ((M4CLK->PLL_STAT_REG_b.M4_SOC_CLK_SWITCHED) != true)
		;

	return 0;
}

static int siwx91x_ulp_ref_clk_config(ULP_REF_CLK_T clkSource)
{
	switch (clkSource) {
	case ULP_REF_CLK_DISABLED:
		MCU_FSM->MCU_FSM_REF_CLK_REG_b.ULPSS_REF_CLK_SEL_b = clkSource;
		system_clocks.ulp_ref_clock_source = clkSource;
		system_clocks.ulpss_ref_clk = 0;
		break;
	case ULP_REF_ULP_MHZ_RC_BYP_CLK:
		MCU_FSM->MCU_FSM_REF_CLK_REG_b.ULPSS_REF_CLK_SEL_b = clkSource;
		system_clocks.ulp_ref_clock_source = clkSource;
		system_clocks.ulpss_ref_clk = system_clocks.byp_rc_ref_clock;
		break;
	case ULP_REF_ULP_MHZ_RC_CLK:
		MCU_FSM->MCU_FSM_REF_CLK_REG_b.ULPSS_REF_CLK_SEL_b = clkSource;
		system_clocks.ulp_ref_clock_source = clkSource;
		system_clocks.ulpss_ref_clk = system_clocks.rc_mhz_clock;
		break;
	case ULP_REF_EXT_40MHZ_CLK:
		MCU_FSM->MCU_FSM_REF_CLK_REG_b.ULPSS_REF_CLK_SEL_b = clkSource;
		system_clocks.ulp_ref_clock_source = clkSource;
		system_clocks.ulpss_ref_clk = system_clocks.rf_ref_clock;
		break;
	default:
		return -EINVAL;
	}

	/* not done before */
	while ((M4CLK->PLL_STAT_REG_b.ULP_REF_CLK_SWITCHED) != true)
		;

	return 0;
}

static int siwx91x_check_pll_lock(PLL_CLK_T clk)
{
	int err = 0;

	switch (clk) {
	case SOC_PLL_CLK:
		if (M4CLK->PLL_STAT_REG_b.SOCPLL_LOCK != 1) {
			err = -EINVAL;
		}
		break;
	case INTF_PLL_CLK:
		if (M4CLK->PLL_STAT_REG_b.INTFPLL_LOCK != 1) {
			err = -EINVAL;
		}
		break;
	case I2S_PLL_CLK:
		if (M4CLK->PLL_STAT_REG_b.I2SPLL_LOCK != 1) {
			err = -EINVAL;
		}
		break;
	case MODEM_PLL_CLK:
		if (M4CLK->PLL_STAT_REG_b.MODEMPLL_LOCK != 1) {
			err = -EINVAL;
		}
		break;
	}
	return err;
}

static int siwx91x_m4_soc_clk_config(M4_SOC_CLK_T clkSource, uint32_t divFactor)
{
	/* check valid parameters */
	if (divFactor >= SOC_MAX_CLK_DIVISION_FACTOR) {
		return -EINVAL;
	}

	/* Added for MCU 100 MHz variant mode setting
	 * Clock will be max/2 in this mode - Legacy
	 */
	// if (MCU_RET->CHIP_CONFIG_MCU_READ_b.LIMIT_M4_FREQ_110MHZ_b == 1) {
	//	divFactor = divFactor / 2;
	// }

	switch (clkSource) {

	case M4_ULP_REF_CLK:
		M4CLK->CLK_CONFIG_REG5_b.M4_SOC_CLK_SEL = clkSource;
		SystemCoreClock = system_clocks.m4ss_ref_clk;
		break;

	case M4_SOC_PLL_CLK:
		if (siwx91x_check_pll_lock(SOC_PLL_CLK) != 0) {
			return -EINVAL;
		}
		M4CLK->CLK_CONFIG_REG5_b.M4_SOC_CLK_SEL = clkSource;
		SystemCoreClock = system_clocks.soc_pll_clock;
		break;

	case M4_MODEM_PLL_CLK1:
		if (siwx91x_check_pll_lock(MODEM_PLL_CLK) != 0) {
			return -EINVAL;
		}
		M4CLK->CLK_CONFIG_REG5_b.M4_SOC_CLK_SEL = clkSource;
		SystemCoreClock = system_clocks.modem_pll_clock;
		break;

	case M4_INTF_PLL_CLK:
		if (siwx91x_check_pll_lock(INTF_PLL_CLK) != 0) {
			return -EINVAL;
		}
		M4CLK->CLK_CONFIG_REG5_b.M4_SOC_CLK_SEL = clkSource;
		SystemCoreClock = system_clocks.intf_pll_clock;
		break;

	case M4_SLEEP_CLK:
		/* Check clock is present is or not before switching */
		if (ULPCLK->M4LP_CTRL_REG_b.ULP_M4_CORE_CLK_ENABLE_b == 1) {
			/* Update the clock MUX */
			M4CLK->CLK_CONFIG_REG5_b.M4_SOC_CLK_SEL = clkSource;
		} else {
			return -EINVAL;
		}
		SystemCoreClock = system_clocks.sleep_clock;
		break;
	default:
		return -EINVAL;
	}

	/* wait for clock switched */
	while ((M4CLK->PLL_STAT_REG_b.M4_SOC_CLK_SWITCHED) != 1)
		;

	/* update the division factor */
	M4CLK->CLK_CONFIG_REG5_b.M4_SOC_CLK_DIV_FAC = (unsigned int)(divFactor & 0x3F);

	if (divFactor) {
		SystemCoreClock /= divFactor;
	}

	system_clocks.soc_clock = SystemCoreClock;

	return 0;
}

static int siwx91x_hp_set_pll_freq(PLL_CLK_T pll_clk, uint32_t pll_freq, PLL_REF_CLK_T pll_ref_clk)
{
	rsi_error_t ret = RSI_OK;

	// Return the error code if frequency is more than 180MHz
	if (pll_freq > MAX_PLL_FREQUENCY) {
		return SL_STATUS_INVALID_PARAMETER;
	}
	// Configure the registers for clock more than 120MHz in PS4
	if (pll_freq >= PLL_PREFETCH_LIMIT) {
		RSI_PS_PS4SetRegisters();
	}

	switch (pll_clk) {
	case SOC_PLL_CLK:
		// Configure SOC-PLL lock settings before configuring SOC PLL clock
		RSI_CLK_SocPllLockConfig(MANUAL_LOCK, BYPASS_MANUAL_LOCK, SOC_PLL_MM_COUNT_LIMIT);

		system_clocks.soc_pll_clock = pll_freq;

		/* Turn ON the SOC_PLL */
		RSI_CLK_SocPllTurnOn();

		if (pll_ref_clk == PLL_REF_CLK_XTAL) {
			siwx91x_aon_clock_request_xtal_to_nwp();
			PLL_REF_CLK_CONFIG_REG &= SELECT_XTAL_MHZ_CLOCK;
		}

		SPI_MEM_MAP_PLL(SOC_PLL_500_CTRL_REG9) = 0xD900;

		ret = clk_set_soc_pll_freq(M4CLK, pll_freq, 40000000);
		if (ret != RSI_OK) {
			return ret;
		}
		break;

	case INTF_PLL_CLK:
		system_clocks.intf_pll_clock = pll_freq;

		/* TurnON the INTF_PLL */
		RSI_CLK_IntfPLLTurnOn();

		if (pll_ref_clk == PLL_REF_CLK_XTAL) {
			siwx91x_aon_clock_request_xtal_to_nwp();
			PLL_REF_CLK_CONFIG_REG &= SELECT_XTAL_MHZ_CLOCK;
		}

		SPI_MEM_MAP_PLL(INTF_PLL_500_CTRL_REG9) = 0xD900;

		ret = clk_set_intf_pll_freq(M4CLK, pll_freq, 40000000);
		if (ret != RSI_OK) {
			return ret;
		}
		break;

	case I2S_PLL_CLK:
		system_clocks.i2s_pll_clock = pll_freq;
		/* TurnON the I2S_PLL */
		RSI_CLK_I2sPllTurnOn();

		/*  Notify NWP that M4 requires XTAL clock source */
		sli_si91x_xtal_turn_on_request_from_m4_to_TA();

		SPI_MEM_MAP_PLL(I2S_PLL_CTRL_REG9) = 0xD900;

		clk_set_i2s_pll_freq(M4CLK, pll_freq, 40000000);

		break;

	default:
		break;
	}

	return ret;
}

static int siwx91x_hp_set_m4_core_clk(M4_SOC_CLK_T clk_source, uint32_t pll_freq)
{
	int err;

	// PLL reference clock set to XTAL_CLK for PLL configuration
	uint32_t pll_ref_clk = PLL_REF_CLK_XTAL;

	// Configure the registers for clock less than 120MHz
	if (pll_freq < PLL_PREFETCH_LIMIT) {
		RSI_PS_PS4ClearRegisters();
	}

	// Changing M4 SOC clock to M4_ULP_REF_CLK
	err = siwx91x_m4_soc_clk_config(M4_ULP_REF_CLK, 0);

	// Configure the required PLL Clocks with desired frequency before configuring it to M4 Core
	if (clk_source == M4_ULP_REF_CLK) {
		UNUSED_PARAMETER(pll_freq);
		return err;
	} else if (clk_source == M4_INTF_PLL_CLK) {
		err = siwx91x_hp_set_pll_freq(INTF_PLL_CLK, pll_freq, pll_ref_clk);
	} else if (clk_source == M4_SOC_PLL_CLK) {
		err = siwx91x_hp_set_pll_freq(SOC_PLL_CLK, pll_freq, pll_ref_clk);
	}

	if (err != 0) {
		return err;
	}

	err = siwx91x_m4_soc_clk_config(clk_source, 0);

	if (!IS_ENABLED(CONFIG_TICKLESS_KERNEL)) {
		// Reconfigure the system tick timer after changing the core clock
		SysTick_Config(SystemCoreClock / 1000);
	}

	return err;
}

/*******************************************************************************
 * Configure clocks as per sleep state
 * Switch Subsystems' Ref clocks to MHz RC
 * Set M4 SOC and QSPI2 clock to Ref clock
 ******************************************************************************/
int config_sleep_clks(void)
{
	int ret = 0;

	// Change ref clocks to RC clock before moving to PS2/Sleep and not requested from PS2
	if (sl_si91x_power_manager_get_current_state() != SL_SI91X_POWER_MANAGER_PS2) {
		// Change Subsystems' ref clocks from 40MHz XTAL to MHz RC
		siwx91x_hp_ref_clk_config(HP_REF_ULP_MHZ_RC_CLK);
		siwx91x_ulp_ref_clk_config(ULP_REF_ULP_MHZ_RC_CLK);

		// Configure M4 source to ULP REF clock
		ret = siwx91x_hp_set_m4_core_clk(M4_ULP_REF_CLK, 0);
		if (ret != 0) {
			return ret;
		}

		RSI_CLK_QspiClkConfig(M4CLK, QSPI_ULPREFCLK, QSPI_SWALLO_EN, QSPI_ODD_DIV_EN,
				    QSPI_DIV_FACTOR);

		RSI_CLK_Qspi2ClkConfig(M4CLK, QSPI_ULPREFCLK, QSPI_SWALLO_EN, QSPI_ODD_DIV_EN,
				      QSPI_DIV_FACTOR);
	}

	return ret;
}

int sli_si91x_config_clocks_to_mhz_rc(void)
{
	if (!(M4_ULP_SLP_STATUS_REG & ULP_MODE_SWITCHED_NPSS)) {
		// Change Subsystems' ref clocks from 40MHz XTAL to MHz RC
		MCU_FSM->MCU_FSM_REF_CLK_REG_b.M4SS_REF_CLK_SEL = ULP_MHZ_RC_CLK;
		MCU_FSM->MCU_FSM_REF_CLK_REG_b.ULPSS_REF_CLK_SEL_b = ULPSS_ULP_MHZ_RC_CLK;
		/*wait for clock switched*/
		while ((M4CLK->PLL_STAT_REG_b.ULP_REF_CLK_SWITCHED) != true)
			;
		// Configure M4 source to ULP REF clock
		M4CLK->CLK_CONFIG_REG5_b.M4_SOC_CLK_SEL = M4_ULPREFCLK;

		clk_qspi_clk_config(M4CLK, QSPI_ULPREFCLK, QSPI_SWALLO_EN, QSPI_ODD_DIV_EN,
				    QSPI_DIV_FACTOR);

		clk_qspi_2_clk_config(M4CLK, QSPI_ULPREFCLK, QSPI_SWALLO_EN, QSPI_ODD_DIV_EN,
				      QSPI_DIV_FACTOR);
	}
	return 0;
}

sl_status_t sli_si91x_clock_manager_config_clks_on_ps_change(sl_power_state_t power_state,
							     boolean_t power_mode)
{
	sl_status_t sli_status = SL_STATUS_OK;
	uint32_t soc_pll_freq;
	QSPI_CLK_SRC_SEL_T qspi_clk_source = QSPI_ULPREFCLK;
	uint8_t qspi_div_fac = QSPI_DIV_FACTOR;
	uint32_t intf_pll_freq;

	switch (power_state) {
	case SL_SI91X_POWER_MANAGER_PS4:
		/* Configure Ref clocks to 40MHz crystal */
		siwx91x_aon_clock_request_xtal_to_nwp();
		siwx91x_hp_ref_clk_config(HP_REF_EXT_40MHZ_CLK);
		siwx91x_ulp_ref_clk_config(ULP_REF_EXT_40MHZ_CLK);

		// Set SOC PLL and configure M4 source to SOC PLL based on current state and mode
		soc_pll_freq = power_mode ? PS4_PERFORMANCE_MODE_SOC_FREQ : PS4_POWERSAVE_MODE_FREQ;

		siwx91x_hp_set_m4_core_clk(M4_SOC_PLL_CLK, soc_pll_freq);

		intf_pll_freq =
			power_mode ? PS4_PERFORMANCE_MODE_INTF_FREQ : PS4_POWERSAVE_MODE_FREQ;

		siwx91x_hp_set_pll_freq(INTF_PLL_CLK, intf_pll_freq, PLL_REF_CLK_XTAL);

		if (intf_pll_freq == PS4_PERFORMANCE_MODE_INTF_FREQ) {
			qspi_clk_source = QSPI_INTFPLLCLK;
		} else {
			qspi_clk_source = QSPI_ULPREFCLK;
		}

		clk_qspi_clk_config(M4CLK, qspi_clk_source, QSPI_SWALLO_EN, QSPI_ODD_DIV_EN,
				    qspi_div_fac);

		clk_qspi_2_clk_config(M4CLK, qspi_clk_source, QSPI_SWALLO_EN, QSPI_ODD_DIV_EN,
				      qspi_div_fac);

		break;

	case SL_SI91X_POWER_MANAGER_PS3:
		/* Configure Ref clocks to 40MHz crystal */
		siwx91x_aon_clock_request_xtal_to_nwp();
		siwx91x_hp_ref_clk_config(HP_REF_EXT_40MHZ_CLK);
		siwx91x_ulp_ref_clk_config(ULP_REF_EXT_40MHZ_CLK);

		// configure M4 source frequency based on current state and mode
		soc_pll_freq = power_mode ? PS3_PERFORMANCE_MODE_FREQ : PS3_POWERSAVE_MODE_FREQ;
		if (power_mode) {
			siwx91x_hp_set_m4_core_clk(M4_SOC_PLL_CLK, soc_pll_freq);
		} else {
			siwx91x_hp_set_m4_core_clk(M4_ULP_REF_CLK, 0);
			//siwx91x_hp_set_pll_freq(SOC_PLL_CLK, soc_pll_freq, PLL_REF_CLK_XTAL);
		}

		// Set INTF PLL based on current state and mode
		intf_pll_freq = power_mode ? PS3_PERFORMANCE_MODE_FREQ : PS3_POWERSAVE_MODE_FREQ;
		
		if (intf_pll_freq == PS3_PERFORMANCE_MODE_FREQ) {
			siwx91x_hp_set_pll_freq(INTF_PLL_CLK, intf_pll_freq, PLL_REF_CLK_XTAL);
			qspi_clk_source = QSPI_INTFPLLCLK;
			qspi_div_fac = 1;
		} else {
			qspi_clk_source = QSPI_ULPREFCLK;
			qspi_div_fac = 2;
		}

		clk_qspi_clk_config(M4CLK, qspi_clk_source, QSPI_SWALLO_EN,
							 QSPI_ODD_DIV_EN, qspi_div_fac);


		clk_qspi_2_clk_config(M4CLK, qspi_clk_source, QSPI_SWALLO_EN,
							   QSPI_ODD_DIV_EN, qspi_div_fac);

		break;

	case SL_SI91X_POWER_MANAGER_PS2:
		// Power modes are not applicable for PS2 state
		UNUSED_PARAMETER(power_mode);

		// Configures the clock with 20MHz
		RSI_IPMU_M20rcOsc_TrimEfuse();
		// Sets FSM HF frequency to 20MHz
		RSI_PS_FsmHfFreqConfig(20);
		// Updated the clock global variables
		RSI_PS_PS2UpdateClockVariable();

		// The remaining clock configurations are common for PS2 and Sleep states
		sli_status = config_sleep_clks();
		break;

	case SL_SI91X_POWER_MANAGER_SLEEP:
		// Power modes are not applicable for Sleep state
		UNUSED_PARAMETER(power_mode);

		// Configure clocks as per sleep state
		sli_status = config_sleep_clks();
		break;

	case SL_SI91X_POWER_MANAGER_PS1:
	case SL_SI91X_POWER_MANAGER_PS0:
	case SL_SI91X_POWER_MANAGER_STANDBY:
		// Not needed for these states
		sli_status = SL_STATUS_INVALID_STATE;
		break;

	default:
		// If reaches here, return error code
		sli_status = SL_STATUS_INVALID_PARAMETER;
		break;
	}

	return sli_status;
}

static int siwx91x_aon_clock_on(const struct device *dev, clock_control_subsys_t sys)
{
	struct siwx91x_aon_clock_data *data = dev->data;
	uintptr_t clockid = (uintptr_t)sys;

	switch (clockid) {
	case SIWX91X_CLK_WATCHDOG:
		/* Both SYSRTC and WDT use LF-FSM XTAL; this call waits for stabilization. */
		rsi_sysrtc_clk_set(RSI_SYSRTC_CLK_32kHz_Xtal, 0);
		break;
	case SIWX91X_CLK_RTC:
		/* Already done in sl_calendar_init(). */
		RSI_PS_NpssPeriPowerUp(SLPSS_PWRGATE_ULP_MCURTC | SLPSS_PWRGATE_ULP_TIMEPERIOD);
		break;
	default:
		return -EINVAL;
	}

	data->enable |= BIT(clockid);
	return 0;
}

static int siwx91x_aon_clock_off(const struct device *dev, clock_control_subsys_t sys)
{
	struct siwx91x_aon_clock_data *data = dev->data;
	uintptr_t clockid = (uintptr_t)sys;

	ARG_UNUSED(dev);

	switch (clockid) {
	case SIWX91X_CLK_WATCHDOG:
	case SIWX91X_CLK_RTC:
		/* Not supported. */
		return 0;
	default:
		return -EINVAL;
	}

	data->enable &= ~BIT(clockid);
	return 0;
}

static int siwx91x_aon_clock_get_rate(const struct device *dev, clock_control_subsys_t sys,
				      uint32_t *rate)
{
	uintptr_t clockid = (uintptr_t)sys;

	ARG_UNUSED(dev);

	switch (clockid) {
	case SIWX91X_CLK_WATCHDOG:
		*rate = LF_FSM_CLOCK_FREQUENCY;
		return 0;
	default:
		return -EINVAL;
	}
}

static int siwx91x_aon_clock_set_rate(const struct device *dev, clock_control_subsys_t sys,
				      clock_control_subsys_rate_t raw_rate)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(sys);
	ARG_UNUSED(raw_rate);
	return -EINVAL;
}

static enum clock_control_status siwx91x_aon_clock_get_status(const struct device *dev,
							      clock_control_subsys_t sys)
{
	struct siwx91x_aon_clock_data *data = dev->data;
	uintptr_t clockid = (uintptr_t)sys;

	if (data->enable & BIT(clockid)) {
		return CLOCK_CONTROL_STATUS_ON;
	}

	return CLOCK_CONTROL_STATUS_OFF;
}

static int siwx91x_aon_clock_init(const struct device *dev)
{
	ARG_UNUSED(dev);
	int ret;

	/*Updated the default SOC clock frequency - tend to be deleted */
	SystemCoreClock = DEFAULT_40MHZ_CLOCK;

	/*Initialize IPMU and MCU FSM blocks - Legacy Link*/
	RSI_Ipmu_Init();

	/* AON */
	/*Configuring the ULP reference clock to 40MHz, as this frequency is required by the
	 * temperature sensor for chip supply mode configuration.*/
	system_clocks.rf_ref_clock = DEFAULT_40MHZ_CLOCK;
	MCU_FSM->MCU_FSM_REF_CLK_REG_b.ULPSS_REF_CLK_SEL_b = ULPSS_40MHZ_CLK;
	system_clocks.ulp_ref_clock_source = ULPSS_40MHZ_CLK;
	system_clocks.ulpss_ref_clk = system_clocks.rf_ref_clock;

	/* maybe after sys init ?*/
	/* IPMU mode configuration based on temperature - Legacy */
	RSI_Configure_Ipmu_Mode();

	/* HP */
	/*Default clock mux configurations */
	M4CLK->CLK_ENABLE_SET_REG3 = M4_SOC_CLK_FOR_OTHER_ENABLE;

	/* AON */
	/* NWP clock is selected as 40MHZ clock from MCU -- why not the ULP_MHZ_BYPASS_RC ?*/
	MCU_FSM->MCU_FSM_REF_CLK_REG_b.TASS_REF_CLK_SEL = ULP_MHZ_RC_CLK;

	/* Configuring MCU FSM clock for BG_PMU */
	RSI_IPMU_ClockMuxSel(2);

	/* Configuring 32kHz Clock for LF-FSM  */
	/* AON */
	MCU_AON->MCUAON_KHZ_CLK_SEL_POR_RESET_STATUS_b.AON_KHZ_CLK_SEL = KHZ_XTAL_CLK_SEL;
	while (MCU_AON->MCUAON_KHZ_CLK_SEL_POR_RESET_STATUS_b.AON_KHZ_CLK_SEL_CLOCK_SWITCHED != 1)
		;

	/* Configuring RC-MHz Clock for HF-FSM */
	/* AON */
	MCU_FSM->MCU_FSM_CLKS_REG_b.HF_FSM_CLK_SELECT = FSM_MHZ_RC;
	while (MCU_FSM->MCU_FSM_CLKS_REG_b.HF_FSM_CLK_SWITCHED_SYNC != 1)
		;

	/* XTAL control pointed to Software and  XTAL is Turned-Off from M4 */
	/* old RSI_ConfigXtal*/
	BATT_FF->MCU_FSM_CTRL_BYPASS_b.MCU_XTAL_EN_40MHZ_BYPASS =
		(unsigned int)((XTAL_DISABLE_FROM_M4 & 0x1) & 0x01);
	BATT_FF->MCU_FSM_CTRL_BYPASS_b.MCU_XTAL_EN_40MHZ_BYPASS_CTRL =
		(unsigned int)((XTAL_IS_IN_SW_CTRL_FROM_M4 & 0x1) & 0x01);

	/* Before NWP is going to power save mode ,set m4ss_ref_clk_mux_ctrl ,
	tass_ref_clk_mux_ctrl, AON domain power supply controls from NWP to M4 */
	RSI_Set_Cntrls_To_M4();

	/*Update the system clock sources with source generating frequency*/
	system_clocks.m4ss_ref_clk = DEFAULT_40MHZ_CLOCK;
	system_clocks.ulpss_ref_clk = DEFAULT_40MHZ_CLOCK;
	system_clocks.soc_pll_clock = DEFAULT_SOC_PLL_CLOCK;
	system_clocks.modem_pll_clock = DEFAULT_MODEM_PLL_CLOCK;
	system_clocks.modem_pll_clock2 = DEFAULT_MODEM_PLL_CLOCK;
	system_clocks.intf_pll_clock = DEFAULT_INTF_PLL_CLOCK;
	system_clocks.soc_clock = DEFAULT_40MHZ_CLOCK;
	system_clocks.rc_32khz_clock = DEFAULT_32KHZ_RC_CLOCK;
	system_clocks.rc_mhz_clock = DEFAULT_MHZ_RC_CLOCK;
	system_clocks.ro_20mhz_clock = DEFAULT_20MHZ_RO_CLOCK;
	system_clocks.ro_32khz_clock = DEFAULT_32KHZ_RO_CLOCK;
	system_clocks.xtal_32khz_clock = DEFAULT_32KHZ_XTAL_CLOCK;
	system_clocks.doubler_clock = DEFAULT_DOUBLER_CLOCK;
	system_clocks.rf_ref_clock = DEFAULT_40MHZ_CLOCK;
	system_clocks.mems_ref_clock = DEFAULT_MEMS_REF_CLOCK;
	system_clocks.byp_rc_ref_clock = DEFAULT_MHZ_RC_CLOCK;
	system_clocks.i2s_pll_clock = DEFAULT_I2S_PLL_CLOCK;

	// need to properly check if the xtal is used in the dt.
	bool xtal_is_used = true;

	// in case we use the XTAL, request XTAL to the NWP
	if (xtal_is_used) {
		siwx91x_aon_clock_request_xtal_to_nwp();
	}

	/* Configure MCU_HP_REF_CLOCK  WTF ? want to set XTAL as HP_REF_CLOCK_SOURCE*/
	/* Since they are not configurin M4_SOC_CLK_SEL, M4_SOC_CLK_SEL will be zeroed and then with
	 * used the M4SS_REF_CLK_SEL to select the source for HP_REF_CLOCK. So configuring
	 * M4SS_REF_CLK_SEL to use XTAL will make HP_REF_CLOCK to use XTAL as source
	 */
	// RSI_CLK_M4ssRefClkConfig(M4CLK, EXT_40MHZ_CLK);
	siwx91x_hp_ref_clk_config(HP_REF_EXT_40MHZ_CLK);

	/* Configure MCU_ULP_REF_CLOCK*/
	// RSI_ULPSS_RefClkConfig(ULPSS_40MHZ_CLK);
	siwx91x_ulp_ref_clk_config(ULP_REF_EXT_40MHZ_CLK);

	// Core Clock runs at 180MHz SOC PLL Clock
	// This function configure the PLL to use the XTAL as reference and set the frequency of SOC
	// PLL 180MHz. It also set the M4 clock source to SOC PLL.
	ret = siwx91x_hp_set_m4_core_clk(M4_SOC_PLL_CLK, SOC_PLL_FREQ);
	if (ret != 0) {
		return ret;
	}

	ret = siwx91x_hp_set_pll_freq(INTF_PLL_CLK, INTF_PLL_FREQUENCY, PLL_REF_CLK_XTAL);
	if (ret != 0) {
		return ret;
	}

	/* need to clock the flash, maybe in the flash driver ?*/
	clk_qspi_clk_config(M4CLK, QSPI_INTFPLLCLK, QSPI_SWALLO_EN, QSPI_ODD_DIV_EN,
			    QSPI_DIV_FACTOR);
	/* needed to clock the psram, maybe in the psram driver ? */
	clk_qspi_2_clk_config(M4CLK, QSPI_INTFPLLCLK, QSPI_SWALLO_EN, QSPI_ODD_DIV_EN,
			      QSPI2_DIV_FACTOR);

	/* TODO: implement the clock out (meens using gpio driver to output the clock on a pin)
	 * sl_si91x_clock_manager_mcu_clk_out(sl_mcu_clk_out_config.pin_config,
	 *  					   sl_mcu_clk_out_config.clk_source,
	 *  					   sl_mcu_clk_out_config.div_factor);
	 */

	/* Use SoC PLL at configured frequency as core clock */
	ret = siwx91x_hp_set_m4_core_clk(M4_SOC_PLL_CLK,
					 DT_PROP(DT_PATH(cpus, cpu_0), clock_frequency));

	/* Since we are reconfiguring the INTF_PLL which is used by the Flash, need to have the
	 * following code in ram*/
	ret = siwx91x_hp_set_pll_freq(INTF_PLL_CLK, INTF_PLL_FREQUENCY, PLL_REF_CLK_XTAL);

	/* Change the QSPI clock source to INTF_PLL */
	clk_qspi_clk_config(M4CLK, QSPI_INTFPLLCLK, 0, 0, 1);

	return 0;
}

static DEVICE_API(clock_control, siwx91x_aon_clock_api) = {
	.on = siwx91x_aon_clock_on,
	.off = siwx91x_aon_clock_off,
	.get_rate = siwx91x_aon_clock_get_rate,
	.set_rate = siwx91x_aon_clock_set_rate,
	.get_status = siwx91x_aon_clock_get_status,
};

#define SIWX91X_AON_CLOCK_INIT(inst)                                                               \
	static struct siwx91x_aon_clock_data siwx91x_aon_clock_data_##inst;                        \
	DEVICE_DT_INST_DEFINE(inst, siwx91x_aon_clock_init, NULL, &siwx91x_aon_clock_data_##inst,  \
			      NULL, PRE_KERNEL_1, CONFIG_CLOCK_CONTROL_INIT_PRIORITY,              \
			      &siwx91x_aon_clock_api);

DT_INST_FOREACH_STATUS_OKAY(SIWX91X_AON_CLOCK_INIT)
