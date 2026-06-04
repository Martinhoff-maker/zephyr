/* Copyright (c) 2024-2026 Silicon Laboratories Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT silabs_siwx91x_ulp_clock_manager

#include <zephyr/dt-bindings/clock/silabs/siwx91x-clock.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/logging/log.h>

#include "si91x_device.h"

#include "rsi_power_save.h"
#include "rsi_rom_ulpss_clk.h"
#include "clock_update.h"

LOG_MODULE_REGISTER(siwx91x_ulp_clock, CONFIG_CLOCK_CONTROL_LOG_LEVEL);

struct siwx91x_ulp_clock_data {
	uint32_t enable;
};

static int siwx91x_ulp_clock_on(const struct device *dev, clock_control_subsys_t sys)
{
	struct siwx91x_ulp_clock_data *data = dev->data;
	uintptr_t clockid = (uintptr_t)sys;

	switch (clockid) {
	case SIWX91X_CLK_ULP_UART:
		RSI_PS_UlpssPeriPowerUp(ULPSS_PWRGATE_ULP_UART);
		RSI_ULPSS_UlpUartClkConfig(ULPCLK, ENABLE_STATIC_CLK,
					   false, ULP_UART_ULP_MHZ_RC_CLK, 1);
		break;
	case SIWX91X_CLK_ULP_I2C:
		RSI_PS_UlpssPeriPowerUp(ULPSS_PWRGATE_ULP_I2C);
		RSI_ULPSS_PeripheralEnable(ULPCLK, ULP_I2C_CLK, ENABLE_STATIC_CLK);
		break;
	case SIWX91X_CLK_ULP_DMA:
		RSI_PS_UlpssPeriPowerUp(ULPSS_PWRGATE_ULP_UDMA);
		RSI_ULPSS_PeripheralEnable(ULPCLK, ULP_UDMA_CLK, ENABLE_STATIC_CLK);
		break;
	case SIWX91X_CLK_ULP_I2S:
		RSI_PS_UlpssPeriPowerUp(ULPSS_PWRGATE_ULP_I2S);
		break;
	case SIWX91X_CLK_STATIC_ULP_I2S:
		ULPCLK->ULP_I2S_CLK_GEN_REG_b.ULP_I2S_MASTER_SLAVE_MODE_b = 1;
		RSI_ULPSS_PeripheralEnable(ULPCLK, ULP_I2S_CLK, ENABLE_STATIC_CLK);
		break;
	case SIWX91X_CLK_ADC:
		/* Warning, DAC also uses these clocks. */
		RSI_IPMU_PowerGateSet(AUXADC_PG_ENB);
		RSI_PS_UlpssPeriPowerUp(ULPSS_PWRGATE_ULP_AUX);
		RSI_ULPSS_AuxClkConfig(ULPCLK, ENABLE_STATIC_CLK, ULP_AUX_REF_CLK);
		break;
	default:
		return -EINVAL;
	}

	data->enable |= BIT(clockid);
	return 0;
}

static int siwx91x_ulp_clock_off(const struct device *dev, clock_control_subsys_t sys)
{
	struct siwx91x_ulp_clock_data *data = dev->data;
	uintptr_t clockid = (uintptr_t)sys;

	switch (clockid) {
	case SIWX91X_CLK_ULP_I2C:
		RSI_ULPSS_PeripheralDisable(ULPCLK, ULP_I2C_CLK);
		break;
	case SIWX91X_CLK_ULP_DMA:
		RSI_ULPSS_PeripheralDisable(ULPCLK, ULP_UDMA_CLK);
		break;
	case SIWX91X_CLK_STATIC_ULP_I2S:
		RSI_ULPSS_PeripheralDisable(ULPCLK, ULP_I2S_CLK);
		break;
	case SIWX91X_CLK_ADC:
		/* Warning, DAC also uses these clocks. */
		RSI_PS_UlpssPeriPowerDown(ULPSS_PWRGATE_ULP_AUX);
		RSI_ULPSS_PeripheralDisable(ULPCLK, ULP_AUX_CLK);
		RSI_IPMU_PowerGateClr(AUXADC_PG_ENB);
		break;
	case SIWX91X_CLK_ULP_UART:
		/* Not supported. */
		return 0;
	default:
		return -EINVAL;
	}

	data->enable &= ~BIT(clockid);
	return 0;
}

static int siwx91x_ulp_clock_get_rate(const struct device *dev, clock_control_subsys_t sys,
					      uint32_t *rate)
{
	uintptr_t clockid = (uintptr_t)sys;

	ARG_UNUSED(dev);

	switch (clockid) {
	case SIWX91X_CLK_ULP_UART:
		*rate = RSI_CLK_GetBaseClock(ULPSS_UART);
		return 0;
	default:
		return -EINVAL;
	}
}

static int siwx91x_ulp_clock_set_rate(const struct device *dev, clock_control_subsys_t sys,
					      clock_control_subsys_rate_t raw_rate)
{
	uintptr_t clockid = (uintptr_t)sys;
	uint32_t rate = *(uint32_t *)raw_rate;
	int ret;

	ARG_UNUSED(dev);

	switch (clockid) {
	case SIWX91X_CLK_ULP_I2S:
		ret = RSI_ULPSS_UlpI2sClkConfig(ULPCLK,
						ULPCLK->ULP_I2S_CLK_GEN_REG_b.ULP_I2S_CLK_SEL_b,
						RSI_CLK_GetBaseClock(ULPSS_I2S) * 2 / rate);
		if (ret) {
			return -EIO;
		}
		return 0;
	default:
		return -EINVAL;
	}
}

static enum clock_control_status siwx91x_ulp_clock_get_status(const struct device *dev,
						       clock_control_subsys_t sys)
{
	struct siwx91x_ulp_clock_data *data = dev->data;
	uintptr_t clockid = (uintptr_t)sys;

	if (data->enable & BIT(clockid)) {
		return CLOCK_CONTROL_STATUS_ON;
	}

	return CLOCK_CONTROL_STATUS_OFF;
}

static int siwx91x_ulp_clock_init(const struct device *dev)
{

#ifdef SL_SI91X_ULP_STATE_ENABLE
	  //Trimming the RC_32MHz clock down to 20MHz, which is utilized in the PS2 state
	  RSI_IPMU_M20rcOsc_TrimEfuse();
	  // Sets FSM HF frequency to 20MHz
	  RSI_PS_FsmHfFreqConfig(20);
	  // Updated the clock global variables
	  RSI_PS_PS2UpdateClockVariable();
#endif

	RSI_ULPSS_RefClkConfig(ULPSS_40MHZ_CLK);

#if DT_NODE_HAS_STATUS(DT_NODELABEL(ulpi2c), okay)
	siwx91x_ulp_clock_on(dev, (clock_control_subsys_t)SIWX91X_CLK_ULP_I2C);
#endif

	return 0;
}

static DEVICE_API(clock_control, siwx91x_ulp_clock_api) = {
	.on = siwx91x_ulp_clock_on,
	.off = siwx91x_ulp_clock_off,
	.get_rate = siwx91x_ulp_clock_get_rate,
	.set_rate = siwx91x_ulp_clock_set_rate,
	.get_status = siwx91x_ulp_clock_get_status,
};

#define SIWX91X_ULP_CLOCK_INIT(inst)                                                        \
	static struct siwx91x_ulp_clock_data siwx91x_ulp_clock_data_##inst;                 \
	DEVICE_DT_INST_DEFINE(inst, siwx91x_ulp_clock_init, NULL,                            \
			      &siwx91x_ulp_clock_data_##inst, NULL, PRE_KERNEL_1,       \
			      CONFIG_CLOCK_CONTROL_INIT_PRIORITY, &siwx91x_ulp_clock_api);

DT_INST_FOREACH_STATUS_OKAY(SIWX91X_ULP_CLOCK_INIT)
