/* Copyright (c) 2024-2026 Silicon Laboratories Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#include "zephyr/sys/__assert.h"
#define DT_DRV_COMPAT silabs_siwx91x_aon_clock_manager

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/clock_control_silabs_siwx91x.h>
#include <zephyr/logging/log.h>

#include "si91x_device.h"

#include "rsi_power_save.h"
#include "rsi_sysrtc.h"

/* For sli_si91x_xtal_turn_on_request_from_m4_to_TA
 * Will be removed when the function will be implemented
 * in the NWP driver and we will call it through the NWP device API
 */
#include "rsi_m4.h"

LOG_MODULE_REGISTER(siwx91x_aon_clock, CONFIG_CLOCK_CONTROL_LOG_LEVEL);

struct siwx91x_aon_clock_config {
	MCU_FSM_Type *fsm_reg;
	MCU_AON_Type *aon_reg;
	struct silabs_siwx91x_clock_control_config *aon_clk_mux;
	size_t aon_clk_mux_count;
};

const static struct {
	uint32_t clkid;
	uint32_t ref_clkid;
	uint32_t reg_value;
} clk_reg_map[] = {
	/*clockid             ref_clkid             reg_value                              */
	/*   |                     |                      |                                  */
	{ SIWX91X_CLK_HP_REF,      0,                     0 }, 
	{ SIWX91X_CLK_HP_REF,      SIWX91X_CLK_RC_MHZ,    1 }, /* ULP_MHZ_RC_BYP_CLK         */
	{ SIWX91X_CLK_HP_REF,      SIWX91X_CLK_RC_MHZ,    2 }, /* ULP_MHZ_RC_CLK             */
	{ SIWX91X_CLK_HP_REF,      SIWX91X_CLK_XTAL_MHZ,  3 }, /* EXT_40MHZ_CLK              */
	{ SIWX91X_CLK_ULP_REF,     0,		          0 }, 
	{ SIWX91X_CLK_ULP_REF,     SIWX91X_CLK_RC_MHZ,    1 }, /* ULP_REF_ULP_MHZ_RC_BYP_CLK */
	{ SIWX91X_CLK_ULP_REF,     SIWX91X_CLK_RC_MHZ,    2 }, /* ULP_REF_ULP_MHZ_RC_CLK     */
	{ SIWX91X_CLK_ULP_REF,     SIWX91X_CLK_XTAL_MHZ,  3 }, /* ULPSS_40MHZ_CLK            */
	{ SIWX91X_CLK_UULP_HF_REF, 0,                     0 }, /* FSM_NO_CLOCK               */
	{ SIWX91X_CLK_UULP_HF_REF, SIWX91X_CLK_RC_MHZ,    2 }, /* FSM_MHZ_RC                 */
	{ SIWX91X_CLK_UULP_LF_REF, SIWX91X_CLK_RC_KHZ,    2 }, /* KHZ_RC_CLK_SEL             */
	{ SIWX91X_CLK_UULP_LF_REF, SIWX91X_CLK_XTAL_KHZ,  4 }, /* KHZ_XTAL_CLK_SEL           */
	{ SIWX91X_CLK_SYSRTC,      SIWX91X_CLK_RC_KHZ,    4 }, /* KHZ_RC_CLK_SEL             */
	{ SIWX91X_CLK_SYSRTC,      SIWX91X_CLK_XTAL_KHZ,  8 }, /* KHZ_XTAL_CLK_SEL           */
};

static uint32_t siwx91x_aon_clock_get_reg(uint32_t clockid, uint32_t ref_clkid)
{
	for (size_t i = 0; i < ARRAY_SIZE(clk_reg_map); i++) {
		if (clockid == clk_reg_map[i].clkid && ref_clkid == clk_reg_map[i].ref_clkid) {
			return clk_reg_map[i].reg_value;
		}
	}
	/* Need to return an proper error when no matching clock configuration is found - this
	 * should never happens if the clock configuration is valid so maybe assert ? 
	 */
	return 0;
}

static uint32_t siwx91x_aon_clock_get_ref_clock(uint32_t clockid, uint32_t reg_value)
{
	for (size_t i = 0; i < ARRAY_SIZE(clk_reg_map); i++) {
		if (clockid == clk_reg_map[i].clkid && reg_value == clk_reg_map[i].reg_value) {
			return clk_reg_map[i].ref_clkid;
		}
	}
	return SIWX91X_CLK_INVALID;
}

static bool siwx91x_aon_clk_valid(uint32_t clockid, uint32_t ref_clkid)
{
	for (size_t i = 0; i < ARRAY_SIZE(clk_reg_map); i++) {
		if (clockid == clk_reg_map[i].clkid && ref_clkid == clk_reg_map[i].ref_clkid) {
			return true;
		}
	}
	return false;
}

void siwx91x_aon_clock_request_xtal_to_nwp(void)
{
	__maybe_unused const struct device *nwp_dev = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(nwp));

	if (nwp_dev) {
		LOG_DBG("Requesting XTAL clock from NWP");
		/* need to call nwp api */
		sli_si91x_xtal_turn_on_request_from_m4_to_TA();
	}

	return;
}

int siwx91x_aon_clk_config(const struct device *dev,
			   struct silabs_siwx91x_clock_control_config *mux)
{
	const struct siwx91x_aon_clock_config *cfg = dev->config;

	switch (mux->clkid) {
	case SIWX91X_CLK_HP_REF:
		cfg->fsm_reg->MCU_FSM_REF_CLK_REG_b.M4SS_REF_CLK_SEL = siwx91x_aon_clock_get_reg(mux->clkid, mux->ref_clkid);

		/* In Wiseconnect, waiting for ulp ref clock change */
		/* M4CLK in Hardcoded because we cannot call HP clock driver since
		 * it is not initialized yet.
		 */
		while ((M4CLK->PLL_STAT_REG_b.M4_SOC_CLK_SWITCHED) != true)
			;
		
		break;
	case SIWX91X_CLK_ULP_REF:
		cfg->fsm_reg->MCU_FSM_REF_CLK_REG_b.ULPSS_REF_CLK_SEL_b = siwx91x_aon_clock_get_reg(mux->clkid, mux->ref_clkid);

		/* Not done in Wiseconnect */
		/* M4CLK in Hardcoded because we cannot call HP clock driver since
		 * it is not initialized yet.
		 */
		while ((M4CLK->PLL_STAT_REG_b.ULP_REF_CLK_SWITCHED) != true)
			;
		
		break;
	case SIWX91X_CLK_UULP_HF_REF:
		cfg->fsm_reg->MCU_FSM_CLKS_REG_b.HF_FSM_CLK_SELECT = siwx91x_aon_clock_get_reg(mux->clkid, mux->ref_clkid);

		while ((cfg->fsm_reg->MCU_FSM_CLKS_REG_b.HF_FSM_CLK_SWITCHED_SYNC) != true)
			;
		
		break;
	case SIWX91X_CLK_UULP_LF_REF:
		cfg->aon_reg->MCUAON_KHZ_CLK_SEL_POR_RESET_STATUS_b.AON_KHZ_CLK_SEL = siwx91x_aon_clock_get_reg(mux->clkid, mux->ref_clkid);

		while (cfg->aon_reg->MCUAON_KHZ_CLK_SEL_POR_RESET_STATUS_b.AON_KHZ_CLK_SEL_CLOCK_SWITCHED != 1)
			;
		
		break;
	case SIWX91X_CLK_SYSRTC:
		cfg->aon_reg->MCUAON_KHZ_CLK_SEL_POR_RESET_STATUS_b.AON_KHZ_CLK_SEL_SYSRTC =
			siwx91x_aon_clock_get_reg(mux->clkid, mux->ref_clkid);

		while (cfg->aon_reg->MCUAON_KHZ_CLK_SEL_POR_RESET_STATUS_b
			       .AON_KHZ_CLK_SEL_CLOCK_SWITCHED_SYSRTC != 1) {
			;
		}
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int siwx91x_aon_clock_on(const struct device *dev, clock_control_subsys_t sys)
{
	uint32_t clockid = (uint32_t)(uintptr_t)(sys);

	switch (clockid) {
	case SIWX91X_CLK_HP_REF:
	case SIWX91X_CLK_ULP_REF:
	case SIWX91X_CLK_UULP_HF_REF:
	case SIWX91X_CLK_UULP_LF_REF:
	case SIWX91X_CLK_SYSRTC:
	case SIWX91X_CLK_WATCHDOG:
	case SIWX91X_CLK_RTC:
		return -EALREADY;
	default:
		return -EINVAL;
	}
}

static int siwx91x_aon_clock_off(const struct device *dev, clock_control_subsys_t sys)
{
	uint32_t clockid = (uint32_t)(uintptr_t)(sys);
	const struct siwx91x_aon_clock_config *cfg = dev->config;

	ARG_UNUSED(dev);

	switch (clockid) {
	case SIWX91X_CLK_XTAL_MHZ:
		cfg->fsm_reg->MCU_FSM_CLK_ENS_AND_FIRST_BOOTUP_b.MCU_ULP_40MHZ_CLK_EN_b = 0;
		break;
	case SIWX91X_CLK_XTAL_KHZ:
		cfg->fsm_reg->MCU_FSM_CLK_ENS_AND_FIRST_BOOTUP_b.MCU_ULP_32KHZ_XTAL_CLK_EN_b = 0;
		break;
	case SIWX91X_CLK_RC_MHZ:
		cfg->fsm_reg->MCU_FSM_CLK_ENS_AND_FIRST_BOOTUP_b.MCU_ULP_MHZ_RC_CLK_EN_b = 0;
		break;
	case SIWX91X_CLK_RC_KHZ:
		cfg->fsm_reg->MCU_FSM_CLK_ENS_AND_FIRST_BOOTUP_b.MCU_ULP_32KHZ_RC_CLK_EN_b = 0;
		break;
	case SIWX91X_CLK_HP_REF:
	case SIWX91X_CLK_ULP_REF:
	case SIWX91X_CLK_UULP_HF_REF:
	case SIWX91X_CLK_UULP_LF_REF:
	case SIWX91X_CLK_SYSRTC: /* SYSRTC is an exception and is completely managed by sleeptimer */
	case SIWX91X_CLK_WATCHDOG: /* no IPMU init ?*/
	case SIWX91X_CLK_RTC: /* IPMU already done in sl_si91x_calendar_init(). */
		return -ENOTSUP;
	default:
		return -EINVAL;
	}

	return 0;
}

static int siwx91x_aon_clock_get_rate(const struct device *dev, clock_control_subsys_t clk_cfg,
				      uint32_t *rate)
{
	const struct siwx91x_aon_clock_config *cfg = dev->config;
	uint32_t clockid = (uint32_t)(clk_cfg);
	uint32_t ret, reg, ref_clkid;

	*rate = 0;
	ref_clkid = SIWX91X_CLK_INVALID;

	switch (clockid) {
	case SIWX91X_CLK_XTAL_MHZ:
		*rate = DT_PROP(DT_NODELABEL(xtal_mhz), clock_frequency);
		break;
	case SIWX91X_CLK_XTAL_KHZ:
		*rate = DT_PROP(DT_NODELABEL(xtal_khz), clock_frequency);
		break;
	case SIWX91X_CLK_RC_MHZ:
		*rate = DT_PROP(DT_NODELABEL(rc_mhz), clock_frequency);
		break;
	case SIWX91X_CLK_RC_KHZ:
		*rate = DT_PROP(DT_NODELABEL(rc_khz), clock_frequency);
		break;
	case SIWX91X_CLK_HP_REF:
		reg = cfg->fsm_reg->MCU_FSM_REF_CLK_REG_b.M4SS_REF_CLK_SEL;
		ref_clkid = siwx91x_aon_clock_get_ref_clock(clockid, reg);
		break;
	case SIWX91X_CLK_ULP_REF:
		reg = cfg->fsm_reg->MCU_FSM_REF_CLK_REG_b.ULPSS_REF_CLK_SEL_b;
		ref_clkid = siwx91x_aon_clock_get_ref_clock(clockid, reg);
		break;
	case SIWX91X_CLK_UULP_HF_REF:
		reg = cfg->fsm_reg->MCU_FSM_CLKS_REG_b.HF_FSM_CLK_SELECT;
		ref_clkid = siwx91x_aon_clock_get_ref_clock(clockid, reg);
		break;
	case SIWX91X_CLK_UULP_LF_REF:
		reg = cfg->aon_reg->MCUAON_KHZ_CLK_SEL_POR_RESET_STATUS_b.AON_KHZ_CLK_SEL;
		ref_clkid = siwx91x_aon_clock_get_ref_clock(clockid, reg);
		break;
	case SIWX91X_CLK_SYSRTC:
		reg = cfg->aon_reg->MCUAON_KHZ_CLK_SEL_POR_RESET_STATUS_b.AON_KHZ_CLK_SEL_SYSRTC;
		ref_clkid = siwx91x_aon_clock_get_ref_clock(SIWX91X_CLK_SYSRTC, reg);
		break;
	case SIWX91X_CLK_WATCHDOG:
	case SIWX91X_CLK_RTC:
		/* RTC and WATCHDOG are linked to UULP_LF_REF */
		ref_clkid = SIWX91X_CLK_UULP_LF_REF;
		break;
	default:
		return -EINVAL;
	}

	if (*rate == 0) {
		ret = siwx91x_aon_clock_get_rate(dev, (clock_control_subsys_t)(uintptr_t)ref_clkid, rate);
		if (ret) {
			return ret;
		}
	}

	return 0;
}

static int siwx91x_aon_clock_set_rate(const struct device *dev, clock_control_subsys_t sys,
				      clock_control_subsys_rate_t raw_rate)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(sys);
	ARG_UNUSED(raw_rate);
	return -ENOTSUP;	
}

static int siwx91x_aon_clock_configure(const struct device *dev, clock_control_subsys_t sys,
				       void *data)
{
	uint32_t clockid = (uint32_t)(uintptr_t)(sys);
	struct silabs_siwx91x_clock_control_config *cfg = data;

	if (cfg->clkid != clockid) {
		return -EINVAL;
	}

	if (!siwx91x_aon_clk_valid(cfg->clkid, cfg->ref_clkid)) {
		return -EINVAL;
	}

	return siwx91x_aon_clk_config(dev, cfg);
}

static enum clock_control_status siwx91x_aon_clock_get_status(const struct device *dev,
							      clock_control_subsys_t clk_cfg)
{
	const struct siwx91x_aon_clock_config *cfg = dev->config;
	uint32_t clkid = (uint32_t)(uintptr_t)(clk_cfg);
	uint32_t reg;

	switch (clkid) {
	case SIWX91X_CLK_XTAL_MHZ:
		reg = cfg->fsm_reg->MCU_FSM_CLK_ENS_AND_FIRST_BOOTUP_b.MCU_ULP_40MHZ_CLK_EN_b;
		break;
	case SIWX91X_CLK_XTAL_KHZ:
		reg = cfg->fsm_reg->MCU_FSM_CLK_ENS_AND_FIRST_BOOTUP_b.MCU_ULP_32KHZ_XTAL_CLK_EN_b;
		break;
	case SIWX91X_CLK_RC_MHZ:
		reg = cfg->fsm_reg->MCU_FSM_CLK_ENS_AND_FIRST_BOOTUP_b.MCU_ULP_MHZ_RC_CLK_EN_b;
		break;
	case SIWX91X_CLK_RC_KHZ:
		reg = cfg->fsm_reg->MCU_FSM_CLK_ENS_AND_FIRST_BOOTUP_b.MCU_ULP_32KHZ_RC_CLK_EN_b;
		break;
	case SIWX91X_CLK_HP_REF:
	case SIWX91X_CLK_ULP_REF:
	case SIWX91X_CLK_UULP_HF_REF:
	case SIWX91X_CLK_UULP_LF_REF:
	case SIWX91X_CLK_SYSRTC:
		reg = 1;
		break;
	default:
		return -EINVAL;
	}

	return reg ? CLOCK_CONTROL_STATUS_ON : CLOCK_CONTROL_STATUS_OFF;
}

static int siwx91x_aon_clock_init(const struct device *dev)
{
	const struct siwx91x_aon_clock_config *cfg = dev->config;

	siwx91x_aon_clock_request_xtal_to_nwp();

	for (size_t i = 0; i < cfg->aon_clk_mux_count; i++) {
		struct silabs_siwx91x_clock_control_config *mux = &cfg->aon_clk_mux[i];

		if (!siwx91x_aon_clk_valid(mux->clkid, mux->ref_clkid)) {
			__ASSERT(false, "Invalid AON route clkid %u ref %u", mux->clkid,
				 mux->ref_clkid);
			continue;
		}

		if (siwx91x_aon_clk_config(dev, mux) != 0) {
			LOG_ERR("AON route %zu (clkid %u) failed", i, mux->clkid);
			return -EIO;
		}
	}

	LOG_INF("AON clock initialized");
	//Print all clock rates with corresponding clockid
	uint32_t rate;
	siwx91x_aon_clock_get_rate(dev, (clock_control_subsys_t)SIWX91X_CLK_XTAL_MHZ, &rate);
	LOG_INF("%s clock rate: %u", "XTAL_MHZ", rate);
	siwx91x_aon_clock_get_rate(dev, (clock_control_subsys_t)SIWX91X_CLK_XTAL_KHZ, &rate);
	LOG_INF("%s clock rate: %u", "XTAL_KHZ", rate);
	siwx91x_aon_clock_get_rate(dev, (clock_control_subsys_t)SIWX91X_CLK_RC_MHZ, &rate);
	LOG_INF("%s clock rate: %u", "RC_MHZ", rate);
	siwx91x_aon_clock_get_rate(dev, (clock_control_subsys_t)SIWX91X_CLK_RC_KHZ, &rate);
	LOG_INF("%s clock rate: %u", "RC_KHZ", rate);
	siwx91x_aon_clock_get_rate(dev, (clock_control_subsys_t)SIWX91X_CLK_HP_REF, &rate);
	LOG_INF("%s clock rate: %u", "HP_REF", rate);
	siwx91x_aon_clock_get_rate(dev, (clock_control_subsys_t)SIWX91X_CLK_ULP_REF, &rate);
	LOG_INF("%s clock rate: %u", "ULP_REF", rate);
	siwx91x_aon_clock_get_rate(dev, (clock_control_subsys_t)SIWX91X_CLK_UULP_HF_REF, &rate);
	LOG_INF("%s clock rate: %u", "UULP_HF_REF", rate);
	siwx91x_aon_clock_get_rate(dev, (clock_control_subsys_t)SIWX91X_CLK_UULP_LF_REF, &rate);
	LOG_INF("%s clock rate: %u", "UULP_LF_REF", rate);
	siwx91x_aon_clock_get_rate(dev, (clock_control_subsys_t)SIWX91X_CLK_SYSRTC, &rate);
	LOG_INF("%s clock rate: %u", "SYSRTC", rate);
	siwx91x_aon_clock_get_rate(dev, (clock_control_subsys_t)SIWX91X_CLK_WATCHDOG, &rate);
	LOG_INF("%s clock rate: %u", "WATCHDOG", rate);
	siwx91x_aon_clock_get_rate(dev, (clock_control_subsys_t)SIWX91X_CLK_RTC, &rate);
	LOG_INF("%s clock rate: %u", "RTC", rate);

		return 0;
}

static DEVICE_API(clock_control, siwx91x_aon_clock_api) = {
	.on = siwx91x_aon_clock_on,
	.off = siwx91x_aon_clock_off,
	.get_rate = siwx91x_aon_clock_get_rate,
	.set_rate = siwx91x_aon_clock_set_rate,
	.configure = siwx91x_aon_clock_configure,
	.get_status = siwx91x_aon_clock_get_status,
};

#define SIWX91X_AON_CLOCK_INIT(inst)                                                               \
	static struct silabs_siwx91x_clock_control_config aon_clk_mux_##inst[] = {                 \
		DT_INST_FOREACH_CHILD(inst, SIWX91X_CLOCK_MANAGER_CHILD_INIT)};                    \
	static const struct siwx91x_aon_clock_config siwx91x_aon_clock_config_##inst = {           \
		.fsm_reg = (MCU_FSM_Type *)DT_INST_REG_ADDR_BY_NAME(inst, mcu_fsm),                \
		.aon_reg = (MCU_AON_Type *)DT_INST_REG_ADDR_BY_NAME(inst, mcu_aon),                \
		.aon_clk_mux = aon_clk_mux_##inst,                                                 \
		.aon_clk_mux_count = ARRAY_SIZE(aon_clk_mux_##inst)                                \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, siwx91x_aon_clock_init, NULL, NULL,                            \
			      &siwx91x_aon_clock_config_##inst, PRE_KERNEL_1,                      \
			      CONFIG_SIWX91X_AON_CLOCK_INIT_PRIORITY, &siwx91x_aon_clock_api);

DT_INST_FOREACH_STATUS_OKAY(SIWX91X_AON_CLOCK_INIT)
