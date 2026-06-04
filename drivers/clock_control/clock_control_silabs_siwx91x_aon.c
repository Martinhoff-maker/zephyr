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

//temporary until we have a better way to share these headers between clock control and the rest of the driver
#include "rsi_rom_ulpss_clk.h"
#include "rsi_rom_clks.h"
#include "sl_status.h"
#include "sl_si91x_clock_manager.h"
#include "rsi_m4.h"
#include "rsi_ipmu.h"
#define INTF_PLL_FREQUENCY 160000000
#define SOC_PLL_FREQ           (180000000UL) // 180MHz default SoC PLL Clock as source to Processor


#define LF_FSM_CLOCK_FREQUENCY 32768


LOG_MODULE_REGISTER(siwx91x_aon_clock, CONFIG_CLOCK_CONTROL_LOG_LEVEL);

struct siwx91x_aon_clock_data {
	uint32_t enable;
};

static void siwx91x_aon_clock_request_xtal_to_nwp(void)
{
	/* Will be use in the futur when communication will be clearer*/
	__maybe_unused const struct device *nwp_dev = DEVICE_DT_GET(DT_NODELABEL(nwp));
	
	/* Should be something like this function but need to call NWP device API in the futur*/
	if(nwp_dev) {
		LOG_DBG("Requesting XTAL clock from NWP");
		sli_si91x_xtal_turn_on_request_from_m4_to_TA();
	} else {
		LOG_ERR("NWP device not found, cannot request XTAL clock");
	}
	return;
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

	// Enables interrupts by clearing the PRIMASK register, allowing the processor to handle IRQs
	__asm volatile("cpsie i" ::: "memory");

	/*Updated the default SOC clock frequency*/
	SystemCoreClock = DEFAULT_40MHZ_CLOCK;

	/*Initialize IPMU and MCU FSM blocks - Legacy Link*/
	RSI_Ipmu_Init();

	/*Configuring the ULP reference clock to 40MHz, as this frequency is required by the temperature sensor for chip supply mode configuration.*/
	system_clocks.rf_ref_clock = DEFAULT_40MHZ_CLOCK;
	MCU_FSM->MCU_FSM_REF_CLK_REG_b.ULPSS_REF_CLK_SEL_b = ULPSS_40MHZ_CLK;
      	system_clocks.ulp_ref_clock_source                 = ULPSS_40MHZ_CLK;
      	system_clocks.ulpss_ref_clk                        = system_clocks.rf_ref_clock;

	/* IPMU mode configuration based on temperature - Legacy */
	RSI_Configure_Ipmu_Mode();

	/*Default clock mux configurations */
	M4CLK->CLK_ENABLE_SET_REG3 = M4_SOC_CLK_FOR_OTHER_ENABLE;

	/* NWP clock is selected as 40MHZ clock from MCU */
	MCU_FSM->MCU_FSM_REF_CLK_REG_b.TASS_REF_CLK_SEL = ULP_MHZ_RC_CLK;
	/* Changing NPSS GPIO 0 mode to 0, to disable buck-boost enable mode*/
	MCU_RET->NPSS_GPIO_CNTRL[0].NPSS_GPIO_CTRLS_b.NPSS_GPIO_MODE = 0;

	/* Configuring MCU FSM clock for BG_PMU */
	RSI_IPMU_ClockMuxSel(2);

	/* Configuring 32kHz Clock for LF-FSM */
	RSI_PS_FsmLfClkSel(KHZ_XTAL_CLK_SEL);

	/* Configuring RC-MHz Clock for HF-FSM */
	RSI_PS_FsmHfClkSel(FSM_MHZ_RC);

	/* XTAL control pointed to Software and  XTAL is Turned-Off from M4 */
	RSI_ConfigXtal(XTAL_DISABLE_FROM_M4, XTAL_IS_IN_SW_CTRL_FROM_M4);

	/* Before NWP is going to power save mode ,set m4ss_ref_clk_mux_ctrl ,
	tass_ref_clk_mux_ctrl, AON domain power supply controls from NWP to M4 */
	RSI_Set_Cntrls_To_M4();

	/*Update the system clock sources with source generating frequency*/
	system_clocks.m4ss_ref_clk     = DEFAULT_40MHZ_CLOCK;
	system_clocks.ulpss_ref_clk    = DEFAULT_40MHZ_CLOCK;
	system_clocks.soc_pll_clock    = DEFAULT_SOC_PLL_CLOCK;
	system_clocks.modem_pll_clock  = DEFAULT_MODEM_PLL_CLOCK;
	system_clocks.modem_pll_clock2 = DEFAULT_MODEM_PLL_CLOCK;
	system_clocks.intf_pll_clock   = DEFAULT_INTF_PLL_CLOCK;
	system_clocks.soc_clock        = DEFAULT_40MHZ_CLOCK;
	system_clocks.rc_32khz_clock   = DEFAULT_32KHZ_RC_CLOCK;
	system_clocks.rc_mhz_clock     = DEFAULT_MHZ_RC_CLOCK;
	system_clocks.ro_20mhz_clock   = DEFAULT_20MHZ_RO_CLOCK;
	system_clocks.ro_32khz_clock   = DEFAULT_32KHZ_RO_CLOCK;
	system_clocks.xtal_32khz_clock = DEFAULT_32KHZ_XTAL_CLOCK;
	system_clocks.doubler_clock    = DEFAULT_DOUBLER_CLOCK;
	system_clocks.rf_ref_clock     = DEFAULT_40MHZ_CLOCK;
	system_clocks.mems_ref_clock   = DEFAULT_MEMS_REF_CLOCK;
	system_clocks.byp_rc_ref_clock = DEFAULT_MHZ_RC_CLOCK;
	system_clocks.i2s_pll_clock    = DEFAULT_I2S_PLL_CLOCK;

	sl_status_t status = SL_STATUS_OK;
	// need to properly check if the xtal is used in the dt.
	bool xtal_is_used = true;

	// in case we use the XTAL, request XTAL to the NWP
	if(xtal_is_used){
		siwx91x_aon_clock_request_xtal_to_nwp();
	}

	/* Configure MCU_HP_REF_CLOCK  WTF ? want to set XTAL as HP_REF_CLOCK_SOURCE*/
	/* Since theya re not configurin M4_SOC_CLK_SEL, M4_SOC_CLK_SEL will be zeroed and then with used the
	 * M4SS_REF_CLK_SEL to select the source for HP_REF_CLOCK. So configuring M4SS_REF_CLK_SEL to use XTAL
	 * will make HP_REF_CLOCK to use XTAL as source
	 */
	RSI_CLK_M4ssRefClkConfig(M4CLK, EXT_40MHZ_CLK);

	/* Configure MCU_ULP_REF_CLOCK*/
	RSI_ULPSS_RefClkConfig(ULPSS_40MHZ_CLK);


	// Core Clock runs at 180MHz SOC PLL Clock
	// This function configure the PLL to use the XTAL as reference and set the frequency of SOC PLL 180MHz.
	// It also set the M4 clock source to SOC PLL.
	status = sl_si91x_clock_manager_m4_set_core_clk(M4_SOCPLLCLK, SOC_PLL_FREQ);
	if (status != SL_STATUS_OK) {
	  return status;
	}

#ifdef SL_SI91X_REQUIRES_INTF_PLL
	  // Configuring the interface PLL clock to 160MHz used by the peripherals whose source clock is INTF_PLL
	  status = sl_si91x_clock_manager_set_pll_freq(INTF_PLL, INTF_PLL_FREQ, PLL_REF_CLK_VAL_XTAL);
	  if (status != SL_STATUS_OK) {
	    return status;
	  }
	// Configure QSPI clock with INTF PLL as input source
#if defined(CLOCK_ROMDRIVER_PRESENT)
  	ROMAPI_M4SS_CLK_API->clk_qspi_clk_config(pCLK, QSPI_INTFPLLCLK, QSPI_SWALLO_EN, QSPI_ODD_DIV_EN, QSPI_DIV_FACTOR);
#endif

#ifdef SLI_SI91X_MCU_PSRAM_PRESENT
  	// Configure QSPI2 clock with INTF PLL as input source
#if defined(CLOCK_ROMDRIVER_PRESENT)
 	 ROMAPI_M4SS_CLK_API->clk_qspi_2_clk_config(pCLK, QSPI_INTFPLLCLK, QSPI_SWALLO_EN, QSPI_ODD_DIV_EN, QSPI2_DIV_FACTOR);
#endif
#endif
#endif /* SL_SI91X_REQUIRES_INTF_PLL */
#if (defined(SL_SI91X_MCU_CLK_OUT_EN) && (SL_SI91X_MCU_CLK_OUT_EN == 1))
  	sl_si91x_clock_manager_mcu_clk_out(sl_mcu_clk_out_config.pin_config,
                                     sl_mcu_clk_out_config.clk_source,
                                     sl_mcu_clk_out_config.div_factor);
#endif

	/* Use SoC PLL at configured frequency as core clock */
	sl_si91x_clock_manager_m4_set_core_clk(M4_SOCPLLCLK,
					       DT_PROP(DT_PATH(cpus, cpu_0), clock_frequency));

	/* Use interface PLL at configured frequency as peripheral clock */
	sl_si91x_clock_manager_set_pll_freq(INFT_PLL, INTF_PLL_FREQUENCY, PLL_REF_CLK_VAL_XTAL);

	/* Change the QSPI clock source to INTF_PLL */
	RSI_CLK_QspiClkConfig(M4CLK, QSPI_INTFPLLCLK, 0, 0, 1);

	/* FIXME: Currently the clock consumer use clocks without power on them.
	 * This should be fixed in drivers. Meanwhile, get the list of required
	 * clocks using DT labels.
	 */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(ulpi2c), okay)
	siwx91x_aon_clock_on(dev, (clock_control_subsys_t)SIWX91X_CLK_ULP_I2C);
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c0), okay)
	siwx91x_aon_clock_on(dev, (clock_control_subsys_t)SIWX91X_CLK_I2C0);
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c1), okay)
	siwx91x_aon_clock_on(dev, (clock_control_subsys_t)SIWX91X_CLK_I2C1);
#endif

	return 0;
}

static DEVICE_API(clock_control, siwx91x_aon_clock_api) = {
	.on = siwx91x_aon_clock_on,
	.off = siwx91x_aon_clock_off,
	.get_rate = siwx91x_aon_clock_get_rate,
	.set_rate = siwx91x_aon_clock_set_rate,
	.get_status = siwx91x_aon_clock_get_status,
};

#define SIWX91X_AON_CLOCK_INIT(inst)                                                        \
	static struct siwx91x_aon_clock_data siwx91x_aon_clock_data_##inst;                 \
	DEVICE_DT_INST_DEFINE(inst, siwx91x_aon_clock_init, NULL,                            \
			      &siwx91x_aon_clock_data_##inst, NULL, PRE_KERNEL_1,       \
			      CONFIG_CLOCK_CONTROL_INIT_PRIORITY, &siwx91x_aon_clock_api);

DT_INST_FOREACH_STATUS_OKAY(SIWX91X_AON_CLOCK_INIT)
