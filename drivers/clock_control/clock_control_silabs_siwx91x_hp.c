/* Copyright (c) 2024-2026 Silicon Laboratories Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT silabs_siwx91x_hp_clock_manager

#include <zephyr/dt-bindings/clock/silabs/siwx91x-clock.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/clock_control_silabs_siwx91x.h>
#include <zephyr/logging/log.h>
#include <errno.h>

#include "sl_si91x_power_manager.h"

#include "si91x_device.h"

#include "clock_update.h"
#include "rsi_pll.h"
#include "rsi_m4.h"
#include "rsi_power_save.h"
#include "rsi_rom_clks.h"

/*
 * QSPI / QSPI2 clock (RM: M4CLK QSPI clock config registers): source mux, integer divider,
 * optional odd-divide and "swallow" paths are programmed via WiseConnect helpers such as
 * clk_qspi_clk_config() / RSI_CLK_Qspi2ClkConfig(). For a future clock_control_set_rate() on
 * SIWX91X_CLK_QSPI, map target Hz to (source, div factor, QSPI_ODD_DIV_EN, QSPI_SWALLO_EN) using
 * the SoC RM tables; keep boot defaults in siwx91x_hp_apply_legacy_boot_defaults() until then.
 */
#define QSPI_DIV_FACTOR  1
#define QSPI_ODD_DIV_EN  0 /* Odd division enable for QSPI clock */
#define QSPI_SWALLO_EN   0 /* Swallow enable for QSPI clock */
#define QSPI2_DIV_FACTOR 1 /* Division factor for QSPI2 clock */

#define PLL_PREFETCH_LIMIT     (120000000UL) // 120MHz Limit for pll clock
#define MAX_PLL_FREQUENCY      (180000000UL) ///< Max PLL frequency is 180MHz
#define MANUAL_LOCK            1    // Manual lock enable
#define BYPASS_MANUAL_LOCK     1    // Bypass manual lock enable
#define SOC_PLL_MM_COUNT_LIMIT 0xA4 // Soc pll count limit

#define PS4_PERFORMANCE_MODE_SOC_FREQ  (180000000UL) // PS4 high power soc pll clock frequency
#define PS4_PERFORMANCE_MODE_INTF_FREQ (160000000UL) // PS4 high power intf pll clock frequency
#define PS4_POWERSAVE_MODE_FREQ        (100000000UL) // PS4 low power clock frequency
#define PS3_PERFORMANCE_MODE_FREQ      (80000000UL)  // PS3 high power clock frequency
#define PS3_POWERSAVE_MODE_FREQ        (40000000UL)  // PS3 low power clock frequency

LOG_MODULE_REGISTER(siwx91x_hp_clock, CONFIG_CLOCK_CONTROL_LOG_LEVEL);

void siwx91x_aon_clock_request_xtal_to_nwp(void);

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

struct siwx91x_hp_clock_config {
	M4CLK_Type *m4clk_reg;
	struct silabs_siwx91x_clock_control_config *hp_clk_mux;
	size_t hp_clk_mux_count;
};

struct siwx91x_hp_clock_data {
	uint32_t enable;
};

static const struct {
	uint32_t clkid;
	uint32_t ref_clkid;
	uint32_t reg_value;
} hp_clk_reg_map[] = {
	{ SIWX91X_CLK_QSPI, SIWX91X_CLK_ULP_REF,  QSPI_ULPREFCLK },
	{ SIWX91X_CLK_QSPI, SIWX91X_CLK_INTF_PLL, QSPI_INTFPLLCLK },
	{ SIWX91X_CLK_QSPI, SIWX91X_CLK_SOC_PLL,  QSPI_SOCPLLCLK },

	{ SIWX91X_CLK_UART0, SIWX91X_CLK_ULP_REF,  USART_ULPREFCLK },
	{ SIWX91X_CLK_UART0, SIWX91X_CLK_SOC_PLL,  USART_SOCPLLCLK },
	{ SIWX91X_CLK_UART0, SIWX91X_CLK_INTF_PLL, USART_INTFPLLCLK },

	{ SIWX91X_CLK_UART1, SIWX91X_CLK_ULP_REF,  USART_ULPREFCLK },
	{ SIWX91X_CLK_UART1, SIWX91X_CLK_SOC_PLL,  USART_SOCPLLCLK },
	{ SIWX91X_CLK_UART1, SIWX91X_CLK_INTF_PLL, USART_INTFPLLCLK },

	{ SIWX91X_CLK_I2C0, SIWX91X_CLK_ULP_REF,  SSI_ULPREFCLK },
	{ SIWX91X_CLK_I2C0, SIWX91X_CLK_SOC_PLL,  SSI_SOCPLLCLK },
	{ SIWX91X_CLK_I2C0, SIWX91X_CLK_INTF_PLL, SSI_INTFPLLCLK },

	{ SIWX91X_CLK_I2C1, SIWX91X_CLK_ULP_REF,  SSI_ULPREFCLK },
	{ SIWX91X_CLK_I2C1, SIWX91X_CLK_SOC_PLL,  SSI_SOCPLLCLK },
	{ SIWX91X_CLK_I2C1, SIWX91X_CLK_INTF_PLL, SSI_INTFPLLCLK },

	{ SIWX91X_CLK_GSPI, SIWX91X_CLK_ULP_REF,  SSI_ULPREFCLK },
	{ SIWX91X_CLK_GSPI, SIWX91X_CLK_SOC_PLL,  SSI_SOCPLLCLK },
	{ SIWX91X_CLK_GSPI, SIWX91X_CLK_INTF_PLL, SSI_INTFPLLCLK },

	{ SIWX91X_CLK_I2S0, SIWX91X_CLK_I2S_PLL, 0 },
	{ SIWX91X_CLK_I2S0, SIWX91X_CLK_SOC_PLL,  1 },

	{ SIWX91X_CLK_PWM, SIWX91X_CLK_SOC_PLL, 0 },
	{ SIWX91X_CLK_PWM, SIWX91X_CLK_INTF_PLL, 1 },

	{ SIWX91X_CLK_PLL_REF, SIWX91X_CLK_XTAL_MHZ, 0 },
	{ SIWX91X_CLK_PLL_REF, SIWX91X_CLK_RC_MHZ, 1 },
};

static int siwx91x_hp_apply_route_div(M4CLK_Type *m4clk,
				      const struct silabs_siwx91x_clock_control_config *mux);


static void siwx91x_hp_rate_apply_hw_div(M4CLK_Type *m4clk, uint32_t clockid, uint32_t *rate)
{
	uint32_t div;

	switch (clockid) {
	case SIWX91X_CLK_QSPI:
		div = m4clk->CLK_CONFIG_REG1_b.QSPI_CLK_DIV_FAC;
		break;
	case SIWX91X_CLK_UART0:
		div = m4clk->CLK_CONFIG_REG2_b.USART1_SCLK_DIV_FAC;
		break;
	case SIWX91X_CLK_UART1:
		div = m4clk->CLK_CONFIG_REG2_b.USART2_SCLK_DIV_FAC;
		break;
	case SIWX91X_CLK_I2C0:
	case SIWX91X_CLK_I2C1:
	case SIWX91X_CLK_GSPI:
		div = m4clk->CLK_CONFIG_REG1_b.SSI_MST_SCLK_DIV_FAC;
		break;
	case SIWX91X_CLK_I2S0:
		div = m4clk->CLK_CONFIG_REG5_b.I2S_CLK_DIV_FAC;
		break;
	case SIWX91X_CLK_PWM:
		div = m4clk->CLK_CONFIG_REG5_b.CT_CLK_DIV_FAC;
		break;
	default:
		return;
	}

	if (div != 0U) {
		*rate /= div;
	}
}

static uint32_t siwx91x_hp_clock_get_reg(uint32_t clockid, uint32_t ref_clkid)
{
	for (size_t i = 0; i < ARRAY_SIZE(hp_clk_reg_map); i++) {
		if (clockid == hp_clk_reg_map[i].clkid &&
		    ref_clkid == hp_clk_reg_map[i].ref_clkid) {
			return hp_clk_reg_map[i].reg_value;
		}
	}
	return UINT32_MAX;
}

static bool siwx91x_hp_clk_is_gate_only(uint32_t clockid)
{
	switch (clockid) {
	case SIWX91X_CLK_DMA0:
	case SIWX91X_CLK_GPDMA0:
	case SIWX91X_CLK_RNG:
		return true;
	default:
		return false;
	}
}

static int siwx91x_hp_pll_ref_select(uint32_t ref_clkid)
{
	switch (ref_clkid) {
	case SIWX91X_CLK_XTAL_MHZ:
		//siwx91x_aon_clock_request_xtal_to_nwp();
		PLL_REF_CLK_CONFIG_REG &= SELECT_XTAL_MHZ_CLOCK;
		return 0;
	case SIWX91X_CLK_RC_MHZ:
		PLL_REF_CLK_CONFIG_REG |= SELECT_RC_MHZ_CLOCK;
		return 0;
	default:
		return -EINVAL;
	}
}

static PLL_REF_CLK_T siwx91x_hp_pll_ref_to_hal(uint32_t ref_clkid)
{
	switch (ref_clkid) {
	case SIWX91X_CLK_RC_MHZ:
		return PLL_REF_CLK_RC;
	case SIWX91X_CLK_XTAL_MHZ:
	default:
		return PLL_REF_CLK_XTAL;
	}
}

static bool siwx91x_hp_clk_valid(uint32_t clockid, uint32_t ref_clkid)
{
	if (clockid == SIWX91X_CLK_PLL_REF) {
		return ref_clkid == SIWX91X_CLK_XTAL_MHZ || ref_clkid == SIWX91X_CLK_RC_MHZ;
	}

	if (siwx91x_hp_clk_is_gate_only(clockid)) {
		/* No source mux: only peripheral gate enable in clock_on(). */
		return ref_clkid == SIWX91X_CLK_INTF_PLL;
	}

	return siwx91x_hp_clock_get_reg(clockid, ref_clkid) != UINT32_MAX;
}

static uint32_t siwx91x_hp_clock_get_ref_clock(uint32_t clockid, uint32_t reg_value)
{
	for (size_t i = 0; i < ARRAY_SIZE(hp_clk_reg_map); i++) {
		if (clockid == hp_clk_reg_map[i].clkid &&
		    reg_value == hp_clk_reg_map[i].reg_value) {
			return hp_clk_reg_map[i].ref_clkid;
		}
	}
	return SIWX91X_CLK_INVALID;
}

static void siwx91x_hp_clk_wait_switched(M4CLK_Type *m4clk, uint32_t clockid)
{
	switch (clockid) {
	case SIWX91X_CLK_QSPI:
		while (m4clk->PLL_STAT_REG_b.QSPI_CLK_SWITCHED == 0U) {
		}
		break;
	case SIWX91X_CLK_UART0:
		while (m4clk->PLL_STAT_REG_b.USART1_SCLK_SWITCHED == 0U) {
		}
		break;
	case SIWX91X_CLK_UART1:
		while (m4clk->PLL_STAT_REG_b.USART2_SCLK_SWITCHED == 0U) {
		}
		break;
	case SIWX91X_CLK_I2C0:
	case SIWX91X_CLK_I2C1:
	case SIWX91X_CLK_GSPI:
		while (m4clk->PLL_STAT_REG_b.SSI_MST_SCLK_SWITCHED == 0U) {
		}
		break;
	case SIWX91X_CLK_I2S0:
		while (m4clk->PLL_STAT_REG_b.I2S_CLK_SWITCHED == 0U) {
		}
		break;
	default:
		break;
	}
}

static int siwx91x_hp_clk_config(const struct device *dev,
			       struct silabs_siwx91x_clock_control_config *mux)
{
	const struct siwx91x_hp_clock_config *cfg = dev->config;
	uint32_t reg_val;

	if (mux->clkid == SIWX91X_CLK_PLL_REF) {
		return siwx91x_hp_pll_ref_select(mux->ref_clkid);
	}

	if (siwx91x_hp_clk_is_gate_only(mux->clkid)) {
		return 0;
	}

	reg_val = siwx91x_hp_clock_get_reg(mux->clkid, mux->ref_clkid);

	if (reg_val == UINT32_MAX) {
		return -EINVAL;
	}

	switch (mux->clkid) {
	case SIWX91X_CLK_QSPI:
		cfg->m4clk_reg->CLK_CONFIG_REG1_b.QSPI_CLK_SEL = reg_val;
		break;
	case SIWX91X_CLK_UART0:
		cfg->m4clk_reg->CLK_CONFIG_REG2_b.USART1_SCLK_SEL = reg_val;
		break;
	case SIWX91X_CLK_UART1:
		cfg->m4clk_reg->CLK_CONFIG_REG2_b.USART2_SCLK_SEL = reg_val;
		break;
	case SIWX91X_CLK_I2C0:
		cfg->m4clk_reg->CLK_CONFIG_REG1_b.SSI_MST_SCLK_SEL = reg_val;
		break;
	case SIWX91X_CLK_I2C1:
		/* I2C1 shares SSI mux on this SoC; instance selected at enable time. */
		cfg->m4clk_reg->CLK_CONFIG_REG1_b.SSI_MST_SCLK_SEL = reg_val;
		break;
	case SIWX91X_CLK_GSPI:
		cfg->m4clk_reg->CLK_CONFIG_REG1_b.SSI_MST_SCLK_SEL = reg_val;
		break;
	case SIWX91X_CLK_I2S0:
		cfg->m4clk_reg->CLK_CONFIG_REG5_b.I2S_CLK_SEL = reg_val;
		break;
	case SIWX91X_CLK_PWM:
		cfg->m4clk_reg->CLK_CONFIG_REG5_b.CT_CLK_SEL = reg_val;
		break;
	default:
		return -EINVAL;
	}

	siwx91x_hp_clk_wait_switched(cfg->m4clk_reg, mux->clkid);

	return siwx91x_hp_apply_route_div(cfg->m4clk_reg, mux);
}

static int siwx91x_hp_apply_route_div(M4CLK_Type *m4clk,
				      const struct silabs_siwx91x_clock_control_config *mux)
{
	if (mux->clock_div == 0U) {
		return 0;
	}

	switch (mux->clkid) {
	case SIWX91X_CLK_QSPI:
		m4clk->CLK_CONFIG_REG1_b.QSPI_CLK_DIV_FAC = mux->clock_div & 0x3FU;
		return 0;
	case SIWX91X_CLK_UART0:
		m4clk->CLK_CONFIG_REG2_b.USART1_SCLK_DIV_FAC = mux->clock_div & 0xFU;
		return 0;
	case SIWX91X_CLK_UART1:
		m4clk->CLK_CONFIG_REG2_b.USART2_SCLK_DIV_FAC = mux->clock_div & 0xFU;
		return 0;
	case SIWX91X_CLK_I2C0:
	case SIWX91X_CLK_I2C1:
	case SIWX91X_CLK_GSPI:
		m4clk->CLK_CONFIG_REG1_b.SSI_MST_SCLK_DIV_FAC = mux->clock_div & 0xFU;
		return 0;
	case SIWX91X_CLK_I2S0:
		m4clk->CLK_CONFIG_REG5_b.I2S_CLK_DIV_FAC = mux->clock_div & 0x3FU;
		return 0;
	case SIWX91X_CLK_PWM:
		m4clk->CLK_CONFIG_REG5_b.CT_CLK_DIV_FAC = mux->clock_div & 0x3FU;
		return 0;
	default:
		LOG_WRN("HP clkid %u: clock-div %u not supported in hardware", mux->clkid,
			mux->clock_div);
		return 0;
	}
}

__maybe_unused static const struct device *aon_clk_dev = DEVICE_DT_GET(DT_NODELABEL(aon_clock_manager));



static int siwx91x_hp_set_pll_freq(PLL_CLK_T pll_clk, uint32_t pll_freq, PLL_REF_CLK_T pll_ref_clk);
static int siwx91x_hp_set_m4_core_clk(M4_SOC_CLK_T clk_source, uint32_t pll_freq);

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
		break;

	case M4_SOC_PLL_CLK:
		if (siwx91x_check_pll_lock(SOC_PLL_CLK) != 0) {
			return -EINVAL;
		}
		M4CLK->CLK_CONFIG_REG5_b.M4_SOC_CLK_SEL = clkSource;
		break;

	case M4_MODEM_PLL_CLK1:
		if (siwx91x_check_pll_lock(MODEM_PLL_CLK) != 0) {
			return -EINVAL;
		}
		M4CLK->CLK_CONFIG_REG5_b.M4_SOC_CLK_SEL = clkSource;
		break;

	case M4_INTF_PLL_CLK:
		if (siwx91x_check_pll_lock(INTF_PLL_CLK) != 0) {
			return -EINVAL;
		}
		M4CLK->CLK_CONFIG_REG5_b.M4_SOC_CLK_SEL = clkSource;
		break;

	case M4_SLEEP_CLK:
		/* Check clock is present is or not before switching */
		if (ULPCLK->M4LP_CTRL_REG_b.ULP_M4_CORE_CLK_ENABLE_b == 1) {
			/* Update the clock MUX */
			M4CLK->CLK_CONFIG_REG5_b.M4_SOC_CLK_SEL = clkSource;
		} else {
			return -EINVAL;
		}
		break;
	default:
		return -EINVAL;
	}

	/* wait for clock switched */
	while ((M4CLK->PLL_STAT_REG_b.M4_SOC_CLK_SWITCHED) != 1)
		;

	/* update the division factor */
	M4CLK->CLK_CONFIG_REG5_b.M4_SOC_CLK_DIV_FAC = (unsigned int)(divFactor & 0x3F);

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

		/* Turn ON the SOC_PLL */
		RSI_CLK_SocPllTurnOn();

		SPI_MEM_MAP_PLL(SOC_PLL_500_CTRL_REG9) = 0xD900;

		ret = RSI_CLK_SetSocPllFreq2(M4CLK, pll_freq, 40000000);
		if (ret != RSI_OK) {
			return ret;
		}
		break;

	case INTF_PLL_CLK:
		/* TurnON the INTF_PLL */
		RSI_CLK_IntfPLLTurnOn();

		SPI_MEM_MAP_PLL(INTF_PLL_500_CTRL_REG9) = 0xD900;

		ret = RSI_CLK_SetIntfPllFreq2(M4CLK, pll_freq, 40000000);
		if (ret != RSI_OK) {
			return ret;
		}
		break;

	case I2S_PLL_CLK:
		/* TurnON the I2S_PLL */
		RSI_CLK_I2sPllTurnOn();

		/*  Notify NWP that M4 requires XTAL clock source */
		//sli_si91x_xtal_turn_on_request_from_m4_to_TA();

		SPI_MEM_MAP_PLL(I2S_PLL_CTRL_REG9) = 0xD900;

		ret = RSI_CLK_SetI2sPllFreq2(M4CLK, pll_freq, 40000000);
		if (ret != RSI_OK) {
			return ret;
		}

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

sl_status_t sl_si91x_clock_manager_control_pll(PLL_TYPE_T pll_type, bool enable)
{
	sl_status_t status = SL_STATUS_OK;

	switch (pll_type) {
	case SOC_PLL:
		// Turn On/Off the SOC PLL
		enable ? RSI_CLK_SocPllTurnOn() : RSI_CLK_SocPllTurnOff();
		break;

	case INTF_PLL:
		// Turn On/Off the INTF PLL
		enable ? RSI_CLK_IntfPLLTurnOn() : RSI_CLK_IntfPLLTurnOff();
		break;

	case I2S_PLL:
		// Turn On/Off the I2S PLL
		enable ? RSI_CLK_I2sPllTurnOn() : RSI_CLK_I2sPllTurnOff();
		break;

	default:
		status = SL_STATUS_INVALID_PARAMETER;
		break;
	}

	return status;
}


/*******************************************************************************
 * Configure clocks as per sleep state
 * Switch Subsystems' Ref clocks to MHz RC
 * Set M4 SOC and QSPI2 clock to Ref clock
 ******************************************************************************/
int config_sleep_clks(void)
{
	struct silabs_siwx91x_clock_control_config hp_cl_cfg = {
		SIWX91X_CLK_HP_REF, SIWX91X_CLK_RC_MHZ, 0};
	struct silabs_siwx91x_clock_control_config ulp_cl_cfg = {
		SIWX91X_CLK_ULP_REF, SIWX91X_CLK_RC_MHZ, 0};
	int ret = 0;

	// Change ref clocks to RC clock before moving to PS2/Sleep and not requested from PS2
	if (sl_si91x_power_manager_get_current_state() != SL_SI91X_POWER_MANAGER_PS2) {
		// Change Subsystems' ref clocks from 40MHz XTAL to MHz RC
		clock_control_configure(aon_clk_dev, (clock_control_subsys_t)SIWX91X_CLK_HP_REF, &hp_cl_cfg);
		clock_control_configure(aon_clk_dev, (clock_control_subsys_t)SIWX91X_CLK_ULP_REF, &ulp_cl_cfg);
		//siwx91x_hp_ref_clk_config(HP_REF_ULP_MHZ_RC_CLK);
		//siwx91x_ulp_ref_clk_config(ULP_REF_ULP_MHZ_RC_CLK);

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

	struct silabs_siwx91x_clock_control_config hp_cl_cfg = {
		SIWX91X_CLK_HP_REF, SIWX91X_CLK_XTAL_MHZ, 0};
	struct silabs_siwx91x_clock_control_config ulp_cl_cfg = {
		SIWX91X_CLK_ULP_REF, SIWX91X_CLK_XTAL_MHZ, 0};

	switch (power_state) {
	case SL_SI91X_POWER_MANAGER_PS4:
		/* Configure Ref clocks to 40MHz crystal */
		siwx91x_aon_clock_request_xtal_to_nwp();
		clock_control_configure(aon_clk_dev, (clock_control_subsys_t)SIWX91X_CLK_HP_REF, &hp_cl_cfg);
		clock_control_configure(aon_clk_dev, (clock_control_subsys_t)SIWX91X_CLK_ULP_REF, &ulp_cl_cfg);

		//siwx91x_hp_ref_clk_config(HP_REF_EXT_40MHZ_CLK);
		//siwx91x_ulp_ref_clk_config(ULP_REF_EXT_40MHZ_CLK);

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

		// clk_qspi_2_clk_config(M4CLK, qspi_clk_source, QSPI_SWALLO_EN, QSPI_ODD_DIV_EN,
		// 		      qspi_div_fac);

		break;

	case SL_SI91X_POWER_MANAGER_PS3:
		/* Configure Ref clocks to 40MHz crystal */
		siwx91x_aon_clock_request_xtal_to_nwp();

		clock_control_configure(aon_clk_dev, (clock_control_subsys_t)SIWX91X_CLK_HP_REF, &hp_cl_cfg);
		clock_control_configure(aon_clk_dev, (clock_control_subsys_t)SIWX91X_CLK_ULP_REF, &ulp_cl_cfg);
		//siwx91x_hp_ref_clk_config(HP_REF_EXT_40MHZ_CLK);
		//siwx91x_ulp_ref_clk_config(ULP_REF_EXT_40MHZ_CLK);

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


		// clk_qspi_2_clk_config(M4CLK, qspi_clk_source, QSPI_SWALLO_EN,
		// 					   QSPI_ODD_DIV_EN, qspi_div_fac);

		break;

	case SL_SI91X_POWER_MANAGER_PS2:
		// Power modes are not applicable for PS2 state
		UNUSED_PARAMETER(power_mode);

		// Configures the clock with 20MHz
		RSI_IPMU_M20rcOsc_TrimEfuse();
		// Sets FSM HF frequency to 20MHz
		RSI_PS_FsmHfFreqConfig(20);
		// Updated the clock global variables
		//RSI_PS_PS2UpdateClockVariable();

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

static int siwx91x_hp_clock_on(const struct device *dev, clock_control_subsys_t sys)
{
	struct siwx91x_hp_clock_data *data = dev->data;
	uint32_t clockid = (uint32_t)(uintptr_t)(sys);

	switch (clockid) {
	case SIWX91X_CLK_UART0:
		RSI_PS_M4ssPeriPowerUp(M4SS_PWRGATE_ULP_EFUSE_PERI);
		/* RSI_CLK_UsartClkConfig() calls RSI_CLK_PeripheralClkEnable(). */
		RSI_CLK_UsartClkConfig(M4CLK, ENABLE_STATIC_CLK, 0, USART1, 0, 1);
		break;
	case SIWX91X_CLK_UART1:
		RSI_PS_M4ssPeriPowerUp(M4SS_PWRGATE_ULP_EFUSE_PERI);
		/* RSI_CLK_UsartClkConfig() calls RSI_CLK_PeripheralClkEnable(). */
		RSI_CLK_UsartClkConfig(M4CLK, ENABLE_STATIC_CLK, 0, USART2, 0, 1);
		break;
	case SIWX91X_CLK_I2C0:
		RSI_PS_M4ssPeriPowerUp(M4SS_PWRGATE_ULP_EFUSE_PERI);
		RSI_CLK_I2CClkConfig(M4CLK, true, 0);
		break;
	case SIWX91X_CLK_I2C1:
		RSI_PS_M4ssPeriPowerUp(M4SS_PWRGATE_ULP_EFUSE_PERI);
		RSI_CLK_I2CClkConfig(M4CLK, true, 1);
		break;
	case SIWX91X_CLK_DMA0:
		RSI_PS_M4ssPeriPowerUp(M4SS_PWRGATE_ULP_EFUSE_PERI);
		RSI_CLK_PeripheralClkEnable(M4CLK, UDMA_CLK, ENABLE_STATIC_CLK);
		break;
	case SIWX91X_CLK_PWM:
		RSI_PS_M4ssPeriPowerUp(M4SS_PWRGATE_ULP_EFUSE_PERI);
		RSI_CLK_PeripheralClkEnable(M4CLK, PWM_CLK, ENABLE_STATIC_CLK);
		break;
	case SIWX91X_CLK_GSPI:
		RSI_PS_M4ssPeriPowerUp(M4SS_PWRGATE_ULP_EFUSE_PERI);
		RSI_CLK_GspiClkConfig(M4CLK, GSPI_INTF_PLL_CLK);
		break;
	case SIWX91X_CLK_QSPI:
		RSI_CLK_PeripheralClkEnable(M4CLK, QSPI_CLK, ENABLE_STATIC_CLK);
		break;
	case SIWX91X_CLK_I2S0:
		RSI_PS_M4ssPeriPowerUp(M4SS_PWRGATE_ULP_EFUSE_PERI);
		break;
	case SIWX91X_CLK_STATIC_I2S0:
		MISC_CFG_MISC_CTRL1 |= (1 << 23);
		RSI_CLK_PeripheralClkEnable(M4CLK, I2SM_CLK, ENABLE_STATIC_CLK);
		break;
	case SIWX91X_CLK_GPDMA0:
		RSI_CLK_PeripheralClkEnable(M4CLK, RPDMA_CLK, ENABLE_STATIC_CLK);
		break;
	case SIWX91X_CLK_RNG:
		RSI_PS_M4ssPeriPowerUp(M4SS_PWRGATE_ULP_EFUSE_PERI);
		RSI_CLK_PeripheralClkEnable1(M4CLK, HWRNG_PCLK_ENABLE);
		break;
	default:
		return -EINVAL;
	}

	data->enable |= BIT(clockid);
	return 0;
}

static int siwx91x_hp_clock_off(const struct device *dev, clock_control_subsys_t sys)
{
	struct siwx91x_hp_clock_data *data = dev->data;
	uint32_t clockid = (uint32_t)(uintptr_t)(sys);

	switch (clockid) {
	case SIWX91X_CLK_UART0:
		RSI_CLK_PeripheralClkDisable(M4CLK, USART1_CLK);
		break;
	case SIWX91X_CLK_UART1:
		RSI_CLK_PeripheralClkDisable(M4CLK, USART2_CLK);
		break;
	case SIWX91X_CLK_DMA0:
		RSI_CLK_PeripheralClkDisable(M4CLK, UDMA_CLK);
		break;
	case SIWX91X_CLK_STATIC_I2S0:
		RSI_CLK_PeripheralClkDisable(M4CLK, I2SM_CLK);
		break;
	case SIWX91X_CLK_RNG:
		/* Not supported. */
		return 0;
	case SIWX91X_CLK_I2C0:
	case SIWX91X_CLK_I2C1:
		/* Not supported. */
		return 0;
	default:
		return -EINVAL;
	}

	data->enable &= ~BIT(clockid);
	return 0;
}



static int siwx91x_hp_get_m4_soc_rate(const struct device *dev, uint32_t *rate)
{
	const struct siwx91x_hp_clock_data *data = dev->data;
	uint32_t src = M4CLK->CLK_CONFIG_REG5_b.M4_SOC_CLK_SEL;

	switch (src) {
	case M4_ULPREFCLK:
		return clock_control_get_rate(aon_clk_dev, (clock_control_subsys_t)SIWX91X_CLK_HP_REF,
					      rate);
	case M4_SOCPLLCLK:
		*rate = data->soc_pll_hz;
		return 0;
	case M4_INTFPLLCLK:
		*rate = data->intf_pll_hz;
		return 0;
	default:
		return -EINVAL;
	}
}

static int siwx91x_hp_read_periph_rate(const struct device *dev, uint32_t clockid,
					uint32_t *rate)
{
	const struct siwx91x_hp_clock_config *cfg = dev->config;
	const struct siwx91x_hp_clock_data *data = dev->data;
	uint32_t reg;
	uint32_t ref_clkid;
	int ret;

	switch (clockid) {
	case SIWX91X_CLK_QSPI:
		reg = M4CLK->CLK_CONFIG_REG1_b.QSPI_CLK_SEL;
		break;
	case SIWX91X_CLK_UART0:
		reg = M4CLK->CLK_CONFIG_REG2_b.USART1_SCLK_SEL;
		break;
	case SIWX91X_CLK_UART1:
		reg = M4CLK->CLK_CONFIG_REG2_b.USART2_SCLK_SEL;
		break;
	case SIWX91X_CLK_I2C0:
	case SIWX91X_CLK_I2C1:
	case SIWX91X_CLK_GSPI:
		reg = M4CLK->CLK_CONFIG_REG1_b.SSI_MST_SCLK_SEL;
		break;
	case SIWX91X_CLK_I2S0:
		reg = M4CLK->CLK_CONFIG_REG5_b.I2S_CLK_SEL;
		break;
	case SIWX91X_CLK_PWM:
		reg = M4CLK->CLK_CONFIG_REG5_b.CT_CLK_SEL;
		break;
	default:
		return -EINVAL;
	}

	ref_clkid = siwx91x_hp_clock_get_ref_clock(clockid, reg);
	if (ref_clkid == SIWX91X_CLK_INVALID) {
		return -EINVAL;
	}

	switch (ref_clkid) {
	case SIWX91X_CLK_ULP_REF:
		ret = clock_control_get_rate(aon_clk_dev, (clock_control_subsys_t)SIWX91X_CLK_ULP_REF,
					     rate);
		break;
	case SIWX91X_CLK_SOC_PLL:
		*rate = data->soc_pll_hz;
		ret = 0;
		break;
	case SIWX91X_CLK_INTF_PLL:
		*rate = data->intf_pll_hz;
		ret = 0;
		break;
	case SIWX91X_CLK_I2S_PLL:
		*rate = data->i2s_pll_hz;
		ret = 0;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	if (ret == 0) {
		siwx91x_hp_rate_apply_hw_div(cfg->m4clk_reg, clockid, rate);
	}

	return ret;
}

static int siwx91x_hp_clock_get_rate(const struct device *dev, clock_control_subsys_t sys,
				     uint32_t *rate)
{
	const struct siwx91x_hp_clock_data *data = dev->data;
	uint32_t clockid = (uint32_t)(uintptr_t)(sys);
	uint32_t soc_rate;
	uint32_t div;

	if (rate == NULL) {
		return -EINVAL;
	}

	switch (clockid) {
	case SIWX91X_CLK_XTAL_MHZ:
		*rate = XTAL_FREQUENCY;
		return 0;
	case SIWX91X_CLK_SOC_PLL:
		*rate = data->soc_pll_hz;
		return 0;
	case SIWX91X_CLK_INTF_PLL:
		*rate = data->intf_pll_hz;
		return 0;
	case SIWX91X_CLK_I2S_PLL:
		*rate = data->i2s_pll_hz;
		return 0;
	case SIWX91X_CLK_HPULP:
		if (siwx91x_hp_get_m4_soc_rate(dev, &soc_rate) != 0) {
			return -EINVAL;
		}
		div = M4CLK->CLK_CONFIG_REG4_b.ULPSS_CLK_DIV_FAC;
		if (div == 0U) {
			*rate = soc_rate;
		} else if (M4CLK->CLK_CONFIG_REG5_b.ULPSS_ODD_DIV_SEL != 0U) {
			*rate = soc_rate / ((2U * div) + 1U);
		} else {
			*rate = soc_rate / div;
		}
		return 0;
	case SIWX91X_CLK_UART0:
	case SIWX91X_CLK_UART1:
	case SIWX91X_CLK_PWM:
	case SIWX91X_CLK_GSPI:
	case SIWX91X_CLK_QSPI:
	case SIWX91X_CLK_I2C0:
	case SIWX91X_CLK_I2C1:
	case SIWX91X_CLK_I2S0:
		return siwx91x_hp_read_periph_rate(dev, clockid, rate);
	default:
		return -EINVAL;
	}
}

static int siwx91x_hp_clock_set_rate(const struct device *dev, clock_control_subsys_t sys,
				     clock_control_subsys_rate_t raw_rate)
{
	struct siwx91x_hp_clock_data *hp_data = dev->data;
	uint32_t clockid = (uint32_t)(uintptr_t)(sys);
	uint32_t rate;
	rsi_error_t ret;

	if (raw_rate == NULL) {
		return -EINVAL;
	}

	rate = *(uint32_t *)raw_rate;
	if (rate == 0U) {
		return -EINVAL;
	}

	switch (clockid) {
	case SIWX91X_CLK_I2S0:
		ret = clk_set_i2s_pll_freq(M4CLK, rate, XTAL_FREQUENCY);
		if (ret != RSI_OK) {
			return -EIO;
		}
		hp_data->i2s_pll_hz = rate;

		ret = RSI_CLK_I2sClkConfig(M4CLK, I2S_PLLCLK, 0);
		if (ret != RSI_OK) {
			return -EIO;
		}
		return 0;
	case SIWX91X_CLK_SOC_PLL:
		if (rate > MAX_PLL_FREQUENCY) {
			return -EINVAL;
		}
		ret = siwx91x_hp_set_pll_freq(SOC_PLL_CLK, rate, PLL_REF_CLK_XTAL);
		if (ret != 0) {
			return -EIO;
		}
		hp_data->soc_pll_hz = rate;
		return 0;
	case SIWX91X_CLK_INTF_PLL:
		if (rate > MAX_PLL_FREQUENCY) {
			return -EINVAL;
		}
		ret = siwx91x_hp_set_pll_freq(INTF_PLL_CLK, rate, PLL_REF_CLK_XTAL);
		if (ret != 0) {
			return -EIO;
		}
		hp_data->intf_pll_hz = rate;
		return 0;
	case SIWX91X_CLK_I2S_PLL:
		ret = siwx91x_hp_set_pll_freq(I2S_PLL_CLK, rate, PLL_REF_CLK_XTAL);
		if (ret != 0) {
			return -EIO;
		}
		hp_data->i2s_pll_hz = rate;
		return 0;
	default:
		return -EINVAL;
	}
}

static int siwx91x_hp_clock_configure(const struct device *dev, clock_control_subsys_t sys,
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

	if (!siwx91x_hp_clk_valid(new_cfg->clkid, new_cfg->ref_clkid)) {
		return -EINVAL;
	}

	return siwx91x_hp_clk_config(dev, new_cfg);
}

static enum clock_control_status siwx91x_hp_clock_get_status(const struct device *dev,
							     clock_control_subsys_t sys)
{
	struct siwx91x_hp_clock_data *data = dev->data;
	uint32_t clockid = (uint32_t)(uintptr_t)(sys);

	if (data->enable & BIT(clockid)) {
		return CLOCK_CONTROL_STATUS_ON;
	}

	return CLOCK_CONTROL_STATUS_OFF;
}

static int siwx91x_hp_init_plls(const struct device *dev)
{
	struct siwx91x_hp_clock_data *data = dev->data;
	int ret;

	M4CLK->CLK_ENABLE_SET_REG3 = M4_SOC_CLK_FOR_OTHER_ENABLE;

	ret = siwx91x_hp_set_m4_core_clk(M4_SOC_PLL_CLK, data->soc_pll_hz);
	if (ret != 0) {
		return ret;
	}

	ret = siwx91x_hp_set_pll_freq(INTF_PLL_CLK, data->intf_pll_hz, pll_ref_clk);
	if (ret != 0) {
		return ret;
	}

	return siwx91x_hp_set_pll_freq(I2S_PLL_CLK, data->i2s_pll_hz, pll_ref_clk);
}

static int siwx91x_hp_clock_init(const struct device *dev)
{
	const struct siwx91x_hp_clock_config *cfg = dev->config;
	int ret;

	/* Set up hp clock routes */
	for (size_t i = 0; i < cfg->hp_clk_mux_count; i++) {
		struct silabs_siwx91x_clock_control_config *mux = &cfg->hp_clk_mux[i];

		if (!siwx91x_hp_clk_valid(mux->clkid, mux->ref_clkid)) {
			LOG_ERR("Invalid HP mux clockid %u ref %u", mux->clkid, mux->ref_clkid);
			return -EINVAL;
		}

		ret = siwx91x_hp_clk_config(dev, mux);
		if (ret != 0) {
			LOG_ERR("HP route %zu (clkid %u) failed: %d", i, mux->clkid, ret);
			return ret;
		}
	}

	/* 2 - Set up PLL frequencies */
	ret = siwx91x_hp_boot_init_core_plls(dev, pll_ref_clk);
	if (ret != 0) {
		LOG_ERR("HP core PLL boot init failed: %d", ret);
		return ret;
	}

	LOG_INF("--------------------------------");
	LOG_INF("HP clock initialized");
	LOG_INF("--------------------------------");

	uint32_t rate;
	siwx91x_hp_clock_get_rate(dev, (clock_control_subsys_t)SIWX91X_CLK_HPULP, &rate);
	LOG_INF("%s clock rate: %u", "HPULP", rate);
	siwx91x_hp_clock_get_rate(dev, (clock_control_subsys_t)SIWX91X_CLK_SOC_PLL, &rate);
	LOG_INF("%s clock rate: %u", "SOC_PLL", rate);
	siwx91x_hp_clock_get_rate(dev, (clock_control_subsys_t)SIWX91X_CLK_INTF_PLL, &rate);
	LOG_INF("%s clock rate: %u", "INTF_PLL", rate);
	siwx91x_hp_clock_get_rate(dev, (clock_control_subsys_t)SIWX91X_CLK_I2S_PLL, &rate);
	LOG_INF("%s clock rate: %u", "I2S_PLL", rate);
	siwx91x_hp_clock_get_rate(dev, (clock_control_subsys_t)SIWX91X_CLK_UART0, &rate);
	LOG_INF("%s clock rate: %u", "UART0", rate);
	siwx91x_hp_clock_get_rate(dev, (clock_control_subsys_t)SIWX91X_CLK_UART1, &rate);
	LOG_INF("%s clock rate: %u", "UART1", rate);
	siwx91x_hp_clock_get_rate(dev, (clock_control_subsys_t)SIWX91X_CLK_I2C0, &rate);
	LOG_INF("%s clock rate: %u", "I2C0", rate);
	siwx91x_hp_clock_get_rate(dev, (clock_control_subsys_t)SIWX91X_CLK_I2C1, &rate);
	LOG_INF("%s clock rate: %u", "I2C1", rate);
	siwx91x_hp_clock_get_rate(dev, (clock_control_subsys_t)SIWX91X_CLK_GSPI, &rate);
	LOG_INF("%s clock rate: %u", "GSPI", rate);
	siwx91x_hp_clock_get_rate(dev, (clock_control_subsys_t)SIWX91X_CLK_QSPI, &rate);
	LOG_INF("%s clock rate: %u", "QSPI", rate);
	siwx91x_hp_clock_get_rate(dev, (clock_control_subsys_t)SIWX91X_CLK_I2S0, &rate);
	LOG_INF("%s clock rate: %u", "I2S0", rate);
	siwx91x_hp_clock_get_rate(dev, (clock_control_subsys_t)SIWX91X_CLK_PWM, &rate);
	LOG_INF("%s clock rate: %u", "PWM", rate);

	return 0;
}

static DEVICE_API(clock_control, siwx91x_hp_clock_api) = {
	.on = siwx91x_hp_clock_on,
	.off = siwx91x_hp_clock_off,
	.get_rate = siwx91x_hp_clock_get_rate,
	.set_rate = siwx91x_hp_clock_set_rate,
	.configure = siwx91x_hp_clock_configure,
	.get_status = siwx91x_hp_clock_get_status,
};

#define SIWX91X_HP_CLOCK_INIT(inst)                                                                \
	static struct silabs_siwx91x_clock_control_config hp_clk_mux_##inst[] = {                  \
		DT_INST_FOREACH_CHILD(inst, SIWX91X_CLOCK_MANAGER_CHILD_INIT)};                    \
	static struct silabs_siwx91x_clock_control_pll_config pll_cfg_##inst[] = {                 \
		DT_INST_FOREACH_CHILD(inst, SIWX91X_CLOCK_MANAGER_PLL_INIT)};                      \
	static const struct siwx91x_hp_clock_config siwx91x_hp_clock_config_##inst = {             \
		.m4clk_reg = (M4CLK_Type *)DT_INST_REG_ADDR_BY_NAME(inst, m4clk),                  \
		.pll_reg = (PLL_Type *)DT_INST_REG_ADDR_BY_NAME(inst, pll),                        \
		.misc_config_reg =                                                                 \
			(MISC_CONFIG_Type *)DT_INST_REG_ADDR_BY_NAME(inst, misc_config),           \
		.hp_clk_mux = hp_clk_mux_##inst,                                                   \
		.hp_clk_mux_count = ARRAY_SIZE(hp_clk_mux_##inst),                                 \
		.pll_cfg = pll_cfg_##inst,                                                         \
		.pll_cfg_count = ARRAY_SIZE(pll_cfg_##inst)};                                      \
	static struct siwx91x_hp_clock_data siwx91x_hp_clock_data_##inst;                          \
	DEVICE_DT_INST_DEFINE(inst, siwx91x_hp_clock_init, NULL, &siwx91x_hp_clock_data_##inst,    \
			      &siwx91x_hp_clock_config_##inst, PRE_KERNEL_1,                       \
			      CONFIG_SIWX91X_HP_CLOCK_INIT_PRIORITY, &siwx91x_hp_clock_api);

DT_INST_FOREACH_STATUS_OKAY(SIWX91X_HP_CLOCK_INIT)
