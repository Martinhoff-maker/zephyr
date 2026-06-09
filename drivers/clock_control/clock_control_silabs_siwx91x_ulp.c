/* Copyright (c) 2024-2026 Silicon Laboratories Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#include "zephyr/devicetree.h"
#define DT_DRV_COMPAT silabs_siwx91x_ulp_clock_manager

#include <zephyr/dt-bindings/clock/silabs/siwx91x-clock.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/clock_control_silabs_siwx91x.h>
#include <zephyr/logging/log.h>
#include <errno.h>

#include "si91x_device.h"

#include "rsi_power_save.h"
#include "rsi_rom_ulpss_clk.h"
#include "clock_update.h"

LOG_MODULE_REGISTER(siwx91x_ulp_clock, CONFIG_CLOCK_CONTROL_LOG_LEVEL);

struct siwx91x_ulp_clock_config {
	ULPCLK_Type *ulpclk_reg;
	struct silabs_siwx91x_clock_control_config *ulp_clk_mux;
	size_t ulp_clk_mux_count;
};

/*
 * ULP Clock Register Value Mapping
 *
 * Maps ULP peripheral clocks to their available clock sources and corresponding register values.
 * Based on SiWx917 Family Reference Manual Section 6.14 MCU ULP Clock Architecture.
 */
const static struct {
	uint32_t clkid;
	uint32_t ref_clkid;
	uint32_t reg_value;
} clk_reg_map[] = {
	{SIWX91X_CLK_ULP_PROC, SIWX91X_CLK_ULP_REF, 0},
	{SIWX91X_CLK_ULP_PROC, SIWX91X_CLK_RC_KHZ, 2},
	{SIWX91X_CLK_ULP_PROC, SIWX91X_CLK_XTAL_KHZ, 3},
	{SIWX91X_CLK_ULP_PROC, SIWX91X_CLK_RC_MHZ, 4},
	{SIWX91X_CLK_ULP_PROC, SIWX91X_CLK_HPULP, 6},
	{SIWX91X_CLK_ULP_PROC, SIWX91X_CLK_INTF_PLL, 8},

	{SIWX91X_CLK_ULP_UART, SIWX91X_CLK_ULP_REF, 0},
	{SIWX91X_CLK_ULP_UART, SIWX91X_CLK_RC_KHZ, 2},
	{SIWX91X_CLK_ULP_UART, SIWX91X_CLK_XTAL_KHZ, 3},
	{SIWX91X_CLK_ULP_UART, SIWX91X_CLK_RC_MHZ, 4},
	{SIWX91X_CLK_ULP_UART, SIWX91X_CLK_HPULP, 6},
	{SIWX91X_CLK_ULP_UART, SIWX91X_CLK_INTF_PLL, 8},

	{SIWX91X_CLK_ULP_I2C, SIWX91X_CLK_ULP_REF, 0},
	{SIWX91X_CLK_ULP_I2C, SIWX91X_CLK_RC_KHZ, 2},
	{SIWX91X_CLK_ULP_I2C, SIWX91X_CLK_XTAL_KHZ, 3},
	{SIWX91X_CLK_ULP_I2C, SIWX91X_CLK_RC_MHZ, 4},
	{SIWX91X_CLK_ULP_I2C, SIWX91X_CLK_HPULP, 6},

	{SIWX91X_CLK_RTC, SIWX91X_CLK_ULP_REF, 0},
	{SIWX91X_CLK_RTC, SIWX91X_CLK_RC_KHZ, 2},
	{SIWX91X_CLK_RTC, SIWX91X_CLK_XTAL_KHZ, 3},
	{SIWX91X_CLK_RTC, SIWX91X_CLK_RC_MHZ, 4},
	{SIWX91X_CLK_RTC, SIWX91X_CLK_HPULP, 6},

	{SIWX91X_CLK_ULP_I2S, SIWX91X_CLK_ULP_REF, 0},
	{SIWX91X_CLK_ULP_I2S, SIWX91X_CLK_RC_KHZ, 2},
	{SIWX91X_CLK_ULP_I2S, SIWX91X_CLK_XTAL_KHZ, 3},
	{SIWX91X_CLK_ULP_I2S, SIWX91X_CLK_RC_MHZ, 4},
	{SIWX91X_CLK_ULP_I2S, SIWX91X_CLK_HPULP, 6},
	{SIWX91X_CLK_ULP_I2S, SIWX91X_CLK_I2S_PLL, 8},

	{SIWX91X_CLK_ADC, SIWX91X_CLK_ULP_REF, 0},
	{SIWX91X_CLK_ADC, SIWX91X_CLK_RC_KHZ, 2},
	{SIWX91X_CLK_ADC, SIWX91X_CLK_XTAL_KHZ, 3},
	{SIWX91X_CLK_ADC, SIWX91X_CLK_RC_MHZ, 4},
	{SIWX91X_CLK_ADC, SIWX91X_CLK_HPULP, 6},
	{SIWX91X_CLK_ADC, SIWX91X_CLK_I2S_PLL, 8},

	{SIWX91X_CLK_ULP_SSI, SIWX91X_CLK_ULP_REF, 0},
	{SIWX91X_CLK_ULP_SSI, SIWX91X_CLK_RC_KHZ, 2},
	{SIWX91X_CLK_ULP_SSI, SIWX91X_CLK_XTAL_KHZ, 3},
	{SIWX91X_CLK_ULP_SSI, SIWX91X_CLK_RC_MHZ, 4},
	{SIWX91X_CLK_ULP_SSI, SIWX91X_CLK_HPULP, 6},

	{SIWX91X_CLK_ULP_TIMER, SIWX91X_CLK_ULP_REF, 0},
	{SIWX91X_CLK_ULP_TIMER, SIWX91X_CLK_RC_KHZ, 2},
	{SIWX91X_CLK_ULP_TIMER, SIWX91X_CLK_XTAL_KHZ, 3},
	{SIWX91X_CLK_ULP_TIMER, SIWX91X_CLK_RC_MHZ, 4},
	{SIWX91X_CLK_ULP_TIMER, SIWX91X_CLK_HPULP, 6},

	{SIWX91X_CLK_SLP_SENSOR, SIWX91X_CLK_ULP_PROC, 0},
};

static uint32_t siwx91x_ulp_clock_get_reg(uint32_t clockid, uint32_t ref_clkid)
{
	for (size_t i = 0; i < ARRAY_SIZE(clk_reg_map); i++) {
		if (clockid == clk_reg_map[i].clkid && ref_clkid == clk_reg_map[i].ref_clkid) {
			return clk_reg_map[i].reg_value;
		}
	}
	return UINT32_MAX;
}

static uint32_t siwx91x_ulp_clock_get_ref_clock(uint32_t clockid, uint32_t reg_value)
{
	for (size_t i = 0; i < ARRAY_SIZE(clk_reg_map); i++) {
		if (clockid == clk_reg_map[i].clkid && reg_value == clk_reg_map[i].reg_value) {
			if (clk_reg_map[i].ref_clkid != 0U) {
				return clk_reg_map[i].ref_clkid;
			}
			if (reg_value == 6U) {
				return SIWX91X_CLK_HPULP;
			}
			return clk_reg_map[i].ref_clkid;
		}
	}
	return SIWX91X_CLK_INVALID;
}

static bool siwx91x_ulp_clk_valid(uint32_t clockid, uint32_t ref_clkid)
{
	return siwx91x_ulp_clock_get_reg(clockid, ref_clkid) != UINT32_MAX;
}

static bool siwx91x_ulp_clk_get_i2s_pll_status(void)
{
	const struct device *hp_clk_dev = DEVICE_DT_GET(DT_NODELABEL(hp_clock_manager));
	int ret;

	ret = clock_control_get_status(hp_clk_dev, (clock_control_subsys_t)SIWX91X_CLK_I2S_PLL);

	return (ret == CLOCK_CONTROL_STATUS_ON) ? true : false;
}

static void siwx91x_ulp_clk_wait_switched(ULPCLK_Type *ulpclk, uint32_t clockid)
{
	switch (clockid) {
	case SIWX91X_CLK_ULP_PROC:
		while (ulpclk->CLOCK_STAUS_REG_b.CLOCK_SWITCHED_PROC_CLK_b == 0U) {
		}
		break;
	case SIWX91X_CLK_ULP_UART:
		while (ulpclk->CLOCK_STAUS_REG_b.CLOCK_SWITCHED_UART_CLK_b == 0U) {
		}
		break;
	case SIWX91X_CLK_ULP_I2S:
		while (ulpclk->CLOCK_STAUS_REG_b.CLOCK_SWITCHED_I2S_CLK_b == 0U) {
		}
		break;
	case SIWX91X_CLK_ULP_TIMER:
		while (ulpclk->CLOCK_STAUS_REG_b.CLOCK_SWITCHED_TIMER_b == 0U) {
		}
		break;
	case SIWX91X_CLK_ULP_SSI:
		while (ulpclk->CLOCK_STAUS_REG_b.CLOCK_SWITCHED_SSI_b == 0U) {
		}
		break;
	case SIWX91X_CLK_ADC:
		while (ulpclk->CLOCK_STAUS_REG_b.CLOCK_SWITCHED_AUXADC_b == 0U) {
		}
		break;
	case SIWX91X_CLK_ULP_I2C:
		while (ulpclk->CLOCK_STAUS_REG_b.CLOCK_SWITCHED_I2C_b == 0U) {
		}
		break;
	default:
		break;
	}
}

static void siwx91x_ulp_clk_get_div_factor(const struct device *dev, uint32_t clockid,
					   uint32_t *div_factor)
{
	const struct siwx91x_ulp_clock_config *config = dev->config;

	if (div_factor == NULL) {
		return;
	}

	switch (clockid) {
	case SIWX91X_CLK_ULP_PROC:
		*div_factor = config->ulpclk_reg->ULP_TA_CLK_GEN_REG_b.ULP_PROC_CLK_DIV_FACTOR;
		break;
	case SIWX91X_CLK_ULP_UART:
		*div_factor = config->ulpclk_reg->ULP_UART_CLK_GEN_REG_b.ULP_UART_CLKDIV_FACTOR;
		break;
	case SIWX91X_CLK_ULP_I2S:
		*div_factor = config->ulpclk_reg->ULP_I2S_CLK_GEN_REG_b.ULP_I2S_CLKDIV_FACTOR;
		break;
	case SIWX91X_CLK_ULP_SSI:
		*div_factor = config->ulpclk_reg->ULP_I2C_SSI_CLK_GEN_REG_b.ULP_SSI_CLK_DIV_FACTOR;
		break;
	case SIWX91X_CLK_SLP_SENSOR:
		*div_factor = config->ulpclk_reg->SLP_SENSOR_CLK_REG_b.DIVISON_FACTOR;
		break;
	default:
		break;
	}

	return;
}

static int siwx91x_ulp_clk_apply_div_factor(const struct device *dev,
					    const struct silabs_siwx91x_clock_control_config *mux)
{
	const struct siwx91x_ulp_clock_config *config = dev->config;

	if (mux->clock_div == 0U) {
		return -EINVAL;
	}

	switch (mux->clkid) {
	case SIWX91X_CLK_ULP_PROC:
		config->ulpclk_reg->ULP_TA_CLK_GEN_REG_b.ULP_PROC_CLK_DIV_FACTOR =
			mux->clock_div & 0xFFU;
		break;
	case SIWX91X_CLK_ULP_UART:
		config->ulpclk_reg->ULP_UART_CLK_GEN_REG_b.ULP_UART_CLKDIV_FACTOR =
			mux->clock_div & 0x7U;
		break;
	case SIWX91X_CLK_ULP_I2S:
		config->ulpclk_reg->ULP_I2S_CLK_GEN_REG_b.ULP_I2S_CLKDIV_FACTOR =
			mux->clock_div & 0xFFU;
		break;
	case SIWX91X_CLK_ULP_SSI:
		config->ulpclk_reg->ULP_I2C_SSI_CLK_GEN_REG_b.ULP_SSI_CLK_DIV_FACTOR =
			mux->clock_div & 0x7FU;
		break;
	case SIWX91X_CLK_SLP_SENSOR:
		config->ulpclk_reg->SLP_SENSOR_CLK_REG_b.DIVISON_FACTOR = mux->clock_div & 0x7FU;
		break;
	default:
		LOG_WRN("ULP clkid %u: clock-div %u not supported in hardware", mux->clkid,
			mux->clock_div);
		return -EINVAL;
	}

	return 0;
}

static int siwx91x_ulp_clk_config(const struct device *dev,
				  struct silabs_siwx91x_clock_control_config *mux)
{
	const struct siwx91x_ulp_clock_config *cfg = dev->config;
	uint32_t reg_val = siwx91x_ulp_clock_get_reg(mux->clkid, mux->ref_clkid);

	if (reg_val == UINT32_MAX) {
		return -EINVAL;
	}

	/* 1 - Disable the clock, disable the dynamic clocking for the clock and configure clock
	 * routing */
	switch (mux->clkid) {
	case SIWX91X_CLK_ULP_PROC:
		cfg->ulpclk_reg->ULP_TA_CLK_GEN_REG_b.ULP_PROC_CLK_SEL = reg_val;
		break;
	case SIWX91X_CLK_ULP_DMA:
		RSI_ULPSS_PeripheralDisable(cfg->ulpclk_reg, ULP_UDMA_CLK);
		break;
	case SIWX91X_CLK_ULP_UART:
		RSI_ULPSS_PeripheralDisable(cfg->ulpclk_reg, ULP_UART_CLK);
		cfg->ulpclk_reg->ULP_UART_CLK_GEN_REG_b.ULP_UART_CLK_SEL = reg_val;
		break;
	case SIWX91X_CLK_ULP_GPIO:
		RSI_ULPSS_PeripheralDisable(cfg->ulpclk_reg, ULP_EGPIO_CLK);
		break;
	case SIWX91X_CLK_ULP_I2C:
		RSI_ULPSS_PeripheralDisable(cfg->ulpclk_reg, ULP_I2C_CLK);
		break;
	case SIWX91X_CLK_ULP_I2S:
		RSI_ULPSS_PeripheralDisable(cfg->ulpclk_reg, ULP_I2S_CLK);
		/* Only supported mode is master for the moment */
		cfg->ulpclk_reg->ULP_I2S_CLK_GEN_REG_b.ULP_I2S_MASTER_SLAVE_MODE_b = 0;
		cfg->ulpclk_reg->ULP_I2S_CLK_GEN_REG_b.ULP_I2S_CLK_SEL_b = reg_val;
		/* In case of I2S PLL, we need to check that the I2S PLL is available. 
		 * Few things to take care:
		 *  - I2S PLL is not supported in PS2 state. (PLL are shut down)
		 *  - When waking from PS4-SLEEP, I2S PLL needs to restart. That's why there is a possibility to bypass the I2S PLL while the I2S PLL 
		 */
		if (reg_val == 8) { /* SIWX91X_CLK_I2S_PLL */
			if (!siwx91x_ulp_clk_get_i2s_pll_status()) {
				LOG_ERR("I2S PLL clock is assigned to ULP_I2S but I2S PLL is not available");
			}
		}
		break;
	case SIWX91X_CLK_ADC:
		RSI_ULPSS_PeripheralDisable(cfg->ulpclk_reg, ULP_AUX_CLK);
		cfg->ulpclk_reg->ULP_AUXADC_CLK_GEN_REG_b.ULP_AUX_CLK_SEL = reg_val;
		break;
	case SIWX91X_CLK_ULP_SSI:
		RSI_ULPSS_PeripheralDisable(cfg->ulpclk_reg, ULP_SSI_CLK);
		cfg->ulpclk_reg->ULP_I2C_SSI_CLK_GEN_REG_b.ULP_SSI_CLK_SEL = reg_val;
		break;
	case SIWX91X_CLK_ULP_TIMER:
		RSI_ULPSS_PeripheralDisable(cfg->ulpclk_reg, ULP_TIMER_CLK);
		cfg->ulpclk_reg->ULP_TIMER_CLK_GEN_REG_b.ULP_TIMER_CLK_SEL = reg_val;
		break;
	case SIWX91X_CLK_SLP_SENSOR:
		/* Nothing to do, clock is hard wired to the ULP_PROC clock and there is no gate*/
		break;
	default:
		return -EINVAL;
	}

	/* 2 - Wait for clock switched */
	siwx91x_ulp_clk_wait_switched(cfg->ulpclk_reg, mux->clkid);

	/* 3 - Apply the clock division factor - historically after waiting that the clock have
	 * switched */
	siwx91x_ulp_clk_apply_div_factor(dev, mux);

	return 0;
}

static int siwx91x_ulp_clock_on(const struct device *dev, clock_control_subsys_t sys)
{
	const struct siwx91x_ulp_clock_config *config = dev->config;
	uint32_t clockid = (uint32_t)(uintptr_t)(sys);

	switch (clockid) {
	case SIWX91X_CLK_ULP_PROC:
		/* ULP_PROC is part of the Misc domain for power */
		config->ulpclk_reg->ULP_TA_CLK_GEN_REG_b.ULP2M4_A2A_BRDG_CLK_EN_b = 1;
		break;
	case SIWX91X_CLK_ULP_UART:
		RSI_PS_UlpssPeriPowerUp(ULPSS_PWRGATE_ULP_UART);
		RSI_ULPSS_PeripheralEnable(config->ulpclk_reg, ULP_UART_CLK, ENABLE_STATIC_CLK);
		break;
	case SIWX91X_CLK_ULP_I2C:
		RSI_PS_UlpssPeriPowerUp(ULPSS_PWRGATE_ULP_I2C);
		RSI_ULPSS_PeripheralEnable(config->ulpclk_reg, ULP_I2C_CLK, ENABLE_STATIC_CLK);
		break;
	case SIWX91X_CLK_ULP_DMA:
		RSI_PS_UlpssPeriPowerUp(ULPSS_PWRGATE_ULP_UDMA);
		RSI_ULPSS_PeripheralEnable(config->ulpclk_reg, ULP_UDMA_CLK, ENABLE_STATIC_CLK);
		break;
	case SIWX91X_CLK_ULP_I2S:
		RSI_PS_UlpssPeriPowerUp(ULPSS_PWRGATE_ULP_I2S);
		RSI_ULPSS_PeripheralEnable(config->ulpclk_reg, ULP_I2S_CLK, ENABLE_STATIC_CLK);
		break;
	case SIWX91X_CLK_ULP_GPIO:
		/* ULP GPIO is part of the Misc domain for power */
		RSI_ULPSS_PeripheralEnable(config->ulpclk_reg, ULP_EGPIO_CLK, ENABLE_STATIC_CLK);
		break;
	case SIWX91X_CLK_ADC:
		RSI_IPMU_PowerGateSet(AUXADC_PG_ENB);
		RSI_PS_UlpssPeriPowerUp(ULPSS_PWRGATE_ULP_AUX);
		RSI_ULPSS_PeripheralEnable(config->ulpclk_reg, ULP_AUX_CLK, ENABLE_STATIC_CLK);
		break;
	case SIWX91X_CLK_ULP_SSI:
		RSI_PS_UlpssPeriPowerUp(ULPSS_PWRGATE_ULP_SSI);
		RSI_ULPSS_PeripheralEnable(config->ulpclk_reg, ULP_SSI_CLK, ENABLE_STATIC_CLK);
		break;
	case SIWX91X_CLK_ULP_TIMER:
		/* ULP_TIMER is part of the Misc domain for power */
		RSI_ULPSS_PeripheralEnable(config->ulpclk_reg, ULP_TIMER_CLK, ENABLE_STATIC_CLK);
		break;
	case SIWX91X_CLK_SLP_SENSOR:
		config->ulpclk_reg->SLP_SENSOR_CLK_REG_b.ENABLE_b = 1;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int siwx91x_ulp_clock_off(const struct device *dev, clock_control_subsys_t sys)
{
	const struct siwx91x_ulp_clock_config *config = dev->config;
	uint32_t clockid = (uint32_t)(uintptr_t)(sys);

	/* Can't power down some peripheral because they are part of the Misc domain */

	switch (clockid) {
	case SIWX91X_CLK_ULP_PROC:
		/* ULP_PROC is part of the Misc domain for power */
		config->ulpclk_reg->ULP_TA_CLK_GEN_REG_b.ULP2M4_A2A_BRDG_CLK_EN_b = 0;
		break;
	case SIWX91X_CLK_ULP_GPIO:
		/* ULP_GPIO is part of the Misc domain for power */
		RSI_ULPSS_PeripheralDisable(config->ulpclk_reg, ULP_EGPIO_CLK);
		break;
	case SIWX91X_CLK_ULP_I2C:
		RSI_ULPSS_PeripheralDisable(config->ulpclk_reg, ULP_I2C_CLK);
		RSI_PS_UlpssPeriPowerDown(ULPSS_PWRGATE_ULP_I2C);
		break;
	case SIWX91X_CLK_ULP_SSI:
		RSI_ULPSS_PeripheralDisable(config->ulpclk_reg, ULP_SSI_CLK);
		RSI_PS_UlpssPeriPowerDown(ULPSS_PWRGATE_ULP_SSI);
		break;
	case SIWX91X_CLK_ULP_TIMER:
		/* ULP_TIMER is part of the Misc domain for power */
		RSI_ULPSS_PeripheralDisable(config->ulpclk_reg, ULP_TIMER_CLK);
		break;
	case SIWX91X_CLK_SLP_SENSOR:
		/* You don't want to do this, it will break the system */
		config->ulpclk_reg->SLP_SENSOR_CLK_REG_b.ENABLE_b = 0;
		break;
	case SIWX91X_CLK_ULP_DMA:
		RSI_ULPSS_PeripheralDisable(config->ulpclk_reg, ULP_UDMA_CLK);
		RSI_PS_UlpssPeriPowerDown(ULPSS_PWRGATE_ULP_UDMA);
		break;
	case SIWX91X_CLK_ULP_I2S:
		RSI_ULPSS_PeripheralDisable(config->ulpclk_reg, ULP_I2S_CLK);
		RSI_PS_UlpssPeriPowerDown(ULPSS_PWRGATE_ULP_I2S);
		break;
	case SIWX91X_CLK_ADC:
		RSI_PS_UlpssPeriPowerDown(ULPSS_PWRGATE_ULP_AUX);
		RSI_ULPSS_PeripheralDisable(config->ulpclk_reg, ULP_AUX_CLK);
		RSI_IPMU_PowerGateClr(AUXADC_PG_ENB);
		break;
	case SIWX91X_CLK_ULP_UART:
		RSI_ULPSS_PeripheralDisable(config->ulpclk_reg, ULP_UART_CLK);
		RSI_PS_UlpssPeriPowerDown(ULPSS_PWRGATE_ULP_UART);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int siwx91x_ulp_clock_get_rate(const struct device *dev, clock_control_subsys_t clk,
				      uint32_t *rate)
{
	const struct siwx91x_ulp_clock_config *config = dev->config;
	const struct device *parent_clk_dev;
	uint32_t ret = 0, reg = 0;
	uint32_t ref_clkid;
	uint32_t clockid = (uint32_t)(uintptr_t)(clk);
	uint32_t div_factor = 0;

	if (rate == NULL) {
		return -EINVAL;
	}

	*rate = 0;
	ref_clkid = SIWX91X_CLK_INVALID;

	switch (clockid) {
	case SIWX91X_CLK_ULP_PROC:
		reg = config->ulpclk_reg->ULP_TA_CLK_GEN_REG_b.ULP_PROC_CLK_SEL;
		break;
	case SIWX91X_CLK_ULP_UART:
		reg = config->ulpclk_reg->ULP_UART_CLK_GEN_REG_b.ULP_UART_CLK_SEL;
		break;
	case SIWX91X_CLK_ULP_I2C:
		/* ULP I2C is linked to the ULP_PROC clock*/
		ref_clkid = SIWX91X_CLK_ULP_PROC;
		break;
	case SIWX91X_CLK_ULP_I2S:
		reg = config->ulpclk_reg->ULP_I2S_CLK_GEN_REG_b.ULP_I2S_CLK_SEL_b;
		break;
	case SIWX91X_CLK_ULP_SSI:
		reg = config->ulpclk_reg->ULP_I2C_SSI_CLK_GEN_REG_b.ULP_SSI_CLK_SEL;
		break;
	case SIWX91X_CLK_ULP_DMA:
		/* UDMA is linked to the ULP_PROC clock*/
		ref_clkid = SIWX91X_CLK_ULP_PROC;
		break;
	case SIWX91X_CLK_ULP_GPIO:
		/* ULP GPIO clock is linked to the ULP_PROC clock*/
		ref_clkid = SIWX91X_CLK_ULP_PROC;
		break;
	case SIWX91X_CLK_ULP_TIMER:
		reg = config->ulpclk_reg->ULP_TIMER_CLK_GEN_REG_b.ULP_TIMER_CLK_SEL;
		break;
	case SIWX91X_CLK_ADC:
		reg = config->ulpclk_reg->ULP_AUXADC_CLK_GEN_REG_b.ULP_AUX_CLK_SEL;
		break;
	case SIWX91X_CLK_SLP_SENSOR:
		/* Slp sensor clock is linked to the ULP_PROC clock*/
		ref_clkid = SIWX91X_CLK_ULP_PROC;
		break;
	default:
		return -EINVAL;
	}

	/* Not directly connected to an oscillator */
	if (*rate == 0) {
		/* No fixed ref clock, get the ref clock from the register */
		if (ref_clkid == SIWX91X_CLK_INVALID) {
			ref_clkid = siwx91x_ulp_clock_get_ref_clock(clockid, reg);
			if (ref_clkid == SIWX91X_CLK_INVALID) {
				return -EINVAL;
			}
		}

		parent_clk_dev = siwx91x_clock_control_get_device(ref_clkid);
		if (parent_clk_dev == NULL) {
			return -EINVAL;
		}

		ret = clock_control_get_rate(parent_clk_dev, (clock_control_subsys_t)ref_clkid, rate);
		if (ret != 0) {
			return ret;
		}
	}

	/* Apply the divider factor if any */
	siwx91x_ulp_clk_get_div_factor(dev, clockid, &div_factor);
	if (div_factor != 0 && div_factor != 1) {
		*rate = *rate / div_factor;
	}

	return ret;
}

static int siwx91x_ulp_clock_set_rate(const struct device *dev, clock_control_subsys_t sys,
				      clock_control_subsys_rate_t raw_rate)
{
	uint32_t clockid = (uint32_t)(uintptr_t)(sys);
	uint32_t rate;
	uint32_t div;
	rsi_error_t ret;

	ARG_UNUSED(dev);

	if (raw_rate == NULL) {
		return -EINVAL;
	}

	rate = *(uint32_t *)raw_rate;
	if (rate == 0U) {
		return -EINVAL;
	}

	switch (clockid) {
	case SIWX91X_CLK_ULP_I2S:
		div = (DEFAULT_40MHZ_CLOCK * 2U) / rate;
		if (div == 0U) {
			div = 1U;
		}

		ret = RSI_ULPSS_UlpI2sClkConfig(
			ULPCLK, ULPCLK->ULP_I2S_CLK_GEN_REG_b.ULP_I2S_CLK_SEL_b, (uint16_t)div);
		if (ret != RSI_OK) {
			return -EIO;
		}

		return 0;
	default:
		return -ENOTSUP;
	}
}

static int siwx91x_ulp_clock_configure(const struct device *dev, clock_control_subsys_t sys,
				       void *data)
{
	uint32_t clockid = (uint32_t)(uintptr_t)(sys);
	struct silabs_siwx91x_clock_control_config *new_cfg = data;

	if (new_cfg == NULL) {
		return -EINVAL;
	}

	if (new_cfg->clkid != clockid) {
		return -EINVAL;
	}

	return siwx91x_ulp_clk_config(dev, new_cfg);
}

static enum clock_control_status siwx91x_ulp_clock_get_status(const struct device *dev,
							      clock_control_subsys_t sys)
{
	const struct siwx91x_ulp_clock_config *config = dev->config;
	uint32_t clockid = (uint32_t)(uintptr_t)(sys);
	uint32_t reg;

	/* Theoricaly, we need to verify that dynamic clocking is disabled before returning the
	 * status of a clock*/
	/* But in practice, it is not needed because the defined clocks are configured in static
	 * mode */

	switch (clockid) {
	case SIWX91X_CLK_ULP_PROC:
		reg = config->ulpclk_reg->ULP_TA_CLK_GEN_REG_b.ULP2M4_A2A_BRDG_CLK_EN_b;
		break;
	case SIWX91X_CLK_ULP_UART:
		reg = config->ulpclk_reg->ULP_MISC_SOFT_SET_REG_b.PCLK_ENABLE_UART_b &&
		      config->ulpclk_reg->ULP_MISC_SOFT_SET_REG_b.SCLK_ENABLE_UART_b;
		break;
	case SIWX91X_CLK_ULP_I2C:
		reg = config->ulpclk_reg->ULP_MISC_SOFT_SET_REG_b.PCLK_ENABLE_I2C_b;
		break;
	case SIWX91X_CLK_ULP_DMA:
		reg = config->ulpclk_reg->ULP_DYN_CLK_CTRL_DISABLE_b.UDMA_CLK_ENABLE_b;
		break;
	case SIWX91X_CLK_ULP_I2S:
		reg = config->ulpclk_reg->ULP_MISC_SOFT_SET_REG_b.CLK_ENABLE_I2S_b &&
		      config->ulpclk_reg->ULP_I2S_CLK_GEN_REG_b.ULP_I2S_CLK_EN_b;
		break;
	case SIWX91X_CLK_ADC:
		reg = config->ulpclk_reg->ULP_DYN_CLK_CTRL_DISABLE_b.AUX_CLK_EN_b;
		break;
	case SIWX91X_CLK_ULP_SSI:
		reg = config->ulpclk_reg->ULP_MISC_SOFT_SET_REG_b.SCLK_ENABLE_SSI_MASTER_b;
		break;
	case SIWX91X_CLK_ULP_TIMER:
		reg = config->ulpclk_reg->ULP_MISC_SOFT_SET_REG_b.CLK_ENABLE_TIMER_b;
		break;
	case SIWX91X_CLK_SLP_SENSOR:
		reg = config->ulpclk_reg->SLP_SENSOR_CLK_REG_b.ENABLE_b;
		break;
	default:
		return CLOCK_CONTROL_STATUS_UNKNOWN;
	}

	return reg ? CLOCK_CONTROL_STATUS_ON : CLOCK_CONTROL_STATUS_OFF;
}

static int siwx91x_ulp_apply_legacy_boot_defaults(const struct device *dev)
{
#ifdef SL_SI91X_ULP_STATE_ENABLE
	RSI_IPMU_M20rcOsc_TrimEfuse();
	RSI_PS_FsmHfFreqConfig(20);
	RSI_PS_PS2UpdateClockVariable();
#endif

	return 0;
}

static int siwx91x_ulp_clock_init(const struct device *dev)
{
	const struct siwx91x_ulp_clock_config *cfg = dev->config;
	int ret;

	/* Enable the Misc domain power - This is where we control the clocks for the ULP domain
	 * It is "on" by default at startup but we need to be sure it is enabled
	 * This also meens that ulp clock driver needs to be linked with ulp power domain in future
	 * If we don't do this, we will not be able to control the clocks for the ULP peripheral
	 * (clock_control_on() will not work).
	 * This is not needed for the moment because nobody explicitely shut down the ULP_MISC
	 * domain.
	 */
	RSI_PS_UlpssPeriPowerUp(ULPSS_PWRGATE_ULP_MISC);

	ret = siwx91x_ulp_apply_legacy_boot_defaults(dev);
	if (ret != 0) {
		return ret;
	}

	/* By default, a lot's of ULP clock are dynamic and then manage by the hardware itself.
	 * All the clock that are present in the device tree are put in static mode.
	 * Once a clock has been configured, it will always be in static mode, we are not allowing
	 * to change it back to dynamic mode in order to match hardware description of the clock in
	 * the dts.
	 */

	/* ULP_PROC is the bus clock for the ULP peripherals and SLP_SENSOR is the bus clock for the
	 * UULP peripherals. It is on by default at startup. That's why clock_control_on() is not
	 * necessary for these clocks.
	 * Hence it is possible to turn off those clock, This need to be done with precaution
	 * because it block all the ULP and UULP peripherals and break Peripheral that were directly
	 * mapped to th ebus clock (UDMA, I2C and ULP_GPIO).
	 */

	/* Note: ulpclk_reg->M4LP_CTRL_REG is not used for the moment. This register allow to give a
	 * clock to the M4 Core and M4 memories when M4_SOC_CLK_SEL switch to SLEEP_CLK. (HP clock
	 * driver). This is not used today and wiseconnect use it when transitioning from PS4 to
	 * PS2. When going to PS4-SLEEP, we manually switch the M4_SOC_CLK_SEL to MCUULP_REF_CLOCK.
	 */

	for (size_t i = 0; i < cfg->ulp_clk_mux_count; i++) {
		if (siwx91x_ulp_clk_valid(cfg->ulp_clk_mux[i].clkid,
					  cfg->ulp_clk_mux[i].ref_clkid)) {
			ret = siwx91x_ulp_clk_config(dev, &cfg->ulp_clk_mux[i]);
			if (ret != 0) {
				LOG_ERR("ULP mux %zu failed: %d", i, ret);
			}
		} else {
			LOG_ERR("Invalid ULP mux clockid %u ref %u", cfg->ulp_clk_mux[i].clkid,
				cfg->ulp_clk_mux[i].ref_clkid);
		}
	}

	LOG_INF("--------------------------------");
	LOG_INF("ULP clock initialized");
	LOG_INF("--------------------------------");
	uint32_t rate;
	siwx91x_ulp_clock_get_rate(dev, (clock_control_subsys_t)SIWX91X_CLK_ULP_PROC, &rate);
	LOG_INF("%s clock rate: %u", "ULP_PROC", rate);
	siwx91x_ulp_clock_get_rate(dev, (clock_control_subsys_t)SIWX91X_CLK_ULP_UART, &rate);
	LOG_INF("%s clock rate: %u", "ULP_UART", rate);
	siwx91x_ulp_clock_get_rate(dev, (clock_control_subsys_t)SIWX91X_CLK_ULP_I2C, &rate);
	LOG_INF("%s clock rate: %u", "ULP_I2C", rate);
	siwx91x_ulp_clock_get_rate(dev, (clock_control_subsys_t)SIWX91X_CLK_ULP_DMA, &rate);
	LOG_INF("%s clock rate: %u", "ULP_DMA", rate);
	siwx91x_ulp_clock_get_rate(dev, (clock_control_subsys_t)SIWX91X_CLK_ULP_I2S, &rate);
	LOG_INF("%s clock rate: %u", "ULP_I2S", rate);
	siwx91x_ulp_clock_get_rate(dev, (clock_control_subsys_t)SIWX91X_CLK_ULP_GPIO, &rate);
	LOG_INF("%s clock rate: %u", "ULP_GPIO", rate);
	siwx91x_ulp_clock_get_rate(dev, (clock_control_subsys_t)SIWX91X_CLK_ULP_SSI, &rate);
	LOG_INF("%s clock rate: %u", "ULP_SSI", rate);
	siwx91x_ulp_clock_get_rate(dev, (clock_control_subsys_t)SIWX91X_CLK_ULP_TIMER, &rate);
	LOG_INF("%s clock rate: %u", "ULP_TIMER", rate);
	siwx91x_ulp_clock_get_rate(dev, (clock_control_subsys_t)SIWX91X_CLK_SLP_SENSOR, &rate);
	LOG_INF("%s clock rate: %u", "SLP_SENSOR", rate);
	siwx91x_ulp_clock_get_rate(dev, (clock_control_subsys_t)SIWX91X_CLK_ADC, &rate);
	LOG_INF("%s clock rate: %u", "ADC", rate);

	return 0;
}

static DEVICE_API(clock_control, siwx91x_ulp_clock_api) = {
	.on = siwx91x_ulp_clock_on,
	.off = siwx91x_ulp_clock_off,
	.get_rate = siwx91x_ulp_clock_get_rate,
	.set_rate = siwx91x_ulp_clock_set_rate,
	.configure = siwx91x_ulp_clock_configure,
	.get_status = siwx91x_ulp_clock_get_status,
};

#define SIWX91X_ULP_CLOCK_INIT(inst)                                                               \
	static struct silabs_siwx91x_clock_control_config ulp_clk_mux_##inst[] = {                 \
		DT_INST_FOREACH_CHILD(inst, SIWX91X_CLOCK_MANAGER_CHILD_INIT)};                    \
	static const struct siwx91x_ulp_clock_config siwx91x_ulp_clock_config_##inst = {           \
		.ulpclk_reg = (ULPCLK_Type *)DT_INST_REG_ADDR_BY_NAME(inst, ulpclk),               \
		.ulp_clk_mux = ulp_clk_mux_##inst,                                                 \
		.ulp_clk_mux_count = ARRAY_SIZE(ulp_clk_mux_##inst)};                              \
	DEVICE_DT_INST_DEFINE(inst, siwx91x_ulp_clock_init, NULL, NULL,                            \
			      &siwx91x_ulp_clock_config_##inst, PRE_KERNEL_1,                      \
			      CONFIG_SIWX91X_ULP_CLOCK_INIT_PRIORITY, &siwx91x_ulp_clock_api);

DT_INST_FOREACH_STATUS_OKAY(SIWX91X_ULP_CLOCK_INIT)
