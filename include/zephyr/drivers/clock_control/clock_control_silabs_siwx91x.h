/*
 * Copyright (c) 2024-2026 Silicon Laboratories Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * ============================================================================
 *  SiWG917 (SiWx91x) — clock control
 * ============================================================================
 *
 *  Three Zephyr clock providers (#clock-cells = <1>, one "clkid" per consumer).
 *  SIWX91X_CLK_* constants live in siwx91x-clock.h.
 *
 *      aon_clock_manager          hp_clock_manager           ulp_clock_manager
 *      (refs / NPSS)              (M4SS / HP)                (ULPSS)
 *      0x24048000                 0x46000000                 0x24041400
 *           |                          |                          |
 *           |                          |                          |
 *      HP_REF, ULP_REF             UART, QSPI, RNG            ULP_UART, ADC
 *      XTAL, RC, RTC/WDT           I2C, GSPI, DMA, PLL        ULP_I2C, ULP_DMA
 *
 *
 *  --- Device-tree (siwg917.dtsi) ---
 *
 *  Manager children use silabs,siwx91x-clock-route (one compatible for domain
 *  refs and peripheral defaults). The node label maps to clkid via
 *  SIWX91X_CLOCK_NODE_TO_ID() below (add a row when you add a labelled clock node).
 *
 *  Example:
 *
 *    ulp_clock_manager {
 *        mcuulp_ref_clock: mcuulp-ref {
 *            compatible = "silabs,siwx91x-clock-route";
 *            clocks = <&xtal_mhz>;
 *        };
 *        ulp_uart_mux: ulp-uart-mux {
 *            compatible = "silabs,siwx91x-clock-route";
 *            clocks = <&mcuulp_ref_clock>;
 *        };
 *    };
 *
 *    ulpuart {
 *        clocks = <&ulp_clock_manager SIWX91X_CLK_ULP_UART>;
 *    };
 *
 *  Consumers use a single clkid cell only — no ref cell in clocks = <>.
 *
 *
 *  --- Typical chain (ULP UART console) ---
 *
 *    xtal_mhz
 *       -> mcuulp_ref_clock     (clock-route: HP domain ref path)
 *       -> ulp_uart_mux         (clock-route: peripheral default source)
 *       -> ulp_clock_manager    (SIWX91X_CLK_ULP_UART)
 *       -> ulpuart               (clock_control_on)
 *
 *
 *  --- Boot vs runtime ---
 *
 *    AT BOOT (manager init, PRE_KERNEL_1)
 *    ------------------------------------
 *    DT_INST_FOREACH_CHILD(manager, SIWX91X_CLOCK_MANAGER_CHILD_INIT)
 *      each clock-route child -> clkid from label, ref from clocks =
 *
 *    AT RUNTIME (peripheral drivers)
 *    -------------------------------
 *    clock_control_on(dev, clkid)     enable gate / power / HAL
 *                                     (-EALREADY means already on, OK)
 *
 *    clock_control_configure(dev, clkid, &cfg)   change source (PM)
 *      cfg = struct silabs_siwx91x_clock_control_config { clkid, ref_clkid }
 *
 *
 *  --- Peripheral driver pattern (no SiLabs-specific macros) ---
 *
 *    .clock_dev    = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(inst)),
 *    .clock_subsys = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(inst, clkid),
 *
 *    ret = clock_control_on(cfg->clock_dev, cfg->clock_subsys);
 *    if (ret != 0 && ret != -EALREADY) {
 *        return ret;
 *    }
 *
 *    subsys is clkid cast to void* (no packing, no second DT cell).
 *
 *
 *  --- Gate-only clocks (no mux node in DT, HP) ---
 *
 *    RNG, DMA0, GPDMA0: no silabs,siwx91x-clock-route child.
 *    configure() is a no-op; on() only asserts the clock gate.
 *
 *
 *  --- clkid ownership ---
 *
 *    aon:  HP_REF, ULP_REF, UULP_HF/LF_REF, XTAL/RC, SYSRTC (RTC/WDT: fixed LF ref)
 *    hp:   UART0/1, I2C0/1, GSPI, QSPI, I2S0, STATIC_I2S0, PWM, DMA0, GPDMA0, RNG, PLLs
 *    ulp:  ULP_UART, ULP_I2C, ULP_DMA, ULP_I2S, STATIC_ULP_I2S, ADC, ULP_PROC
 *
 *
 *  --- Macros (this file) ---
 *
 *    SIWX91X_CLOCK_NODE_TO_ID    DT nodelabel -> SIWX91X_CLK_* (routes, refs, xtal, pll, ...)
 *    siwx91x_clock_control_get_device()  runtime clkid -> clock_control device (from DT table)
 *    SIWX91X_CLOCK_ROUTE_NODE_INIT / SIWX91X_CLOCK_MANAGER_CHILD_INIT
 *
 *  PLL outputs: silabs,siwx91x-pll (clocks = &hp_pll_ref_mux, shared ref).
 *  /clocks fixed-factor nodes unchanged.
 *
 *  Drivers:  clock_control_silabs_siwx91x_{hp,ulp,aon}.c
 *  DTS:      zephyr/dts/arm/silabs/siwg917.dtsi
 *
 * ============================================================================
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_SILABS_SIWX91X_H_
#define ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_SILABS_SIWX91X_H_

#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/dt-bindings/clock/silabs/siwx91x-clock.h>

/** Boot route (DT child) and clock_control_configure() runtime argument. */
struct silabs_siwx91x_clock_control_config {
	uint32_t clkid;
	uint32_t ref_clkid;
	uint32_t clock_div;
};

struct silabs_siwx91x_clock_control_pll_config {
	uint32_t clkid;
	uint32_t ref_clkid;
	uint32_t frequency;
};

#define SIWX91X_DT_CLOCK_STARTUP_TIME_US(node_id)                                              \
	DT_PROP_OR(node_id, silabs_startup_time_us, 0)

/*
 * DT nodelabel <-> SIWX91X_CLK_* (xtal/rc, domain refs, PLL outputs, clock-route children).
 * Add one DT_SAME_NODE line per labelled node and a matching SIWX91X_CLOCK_ROUTE_DEV()
 * in siwx91x_clk_dev_table[] below.
 */
#define SIWX91X_CLOCK_NODE_TO_ID(node)                                                             \
	(DT_SAME_NODE(DT_NODELABEL(xtal_mhz), node)               ? SIWX91X_CLK_XTAL_MHZ           \
	 : DT_SAME_NODE(DT_NODELABEL(xtal_khz), node)             ? SIWX91X_CLK_XTAL_KHZ           \
	 : DT_SAME_NODE(DT_NODELABEL(rc_mhz), node)               ? SIWX91X_CLK_RC_MHZ             \
	 : DT_SAME_NODE(DT_NODELABEL(rc_khz), node)               ? SIWX91X_CLK_RC_KHZ             \
	 : DT_SAME_NODE(DT_NODELABEL(mcuhp_ref_clock), node)      ? SIWX91X_CLK_HP_REF             \
	 : DT_SAME_NODE(DT_NODELABEL(mcuulp_ref_clock), node)     ? SIWX91X_CLK_ULP_REF            \
	 : DT_SAME_NODE(DT_NODELABEL(mcuuulp_hf_ref_clock), node) ? SIWX91X_CLK_UULP_HF_REF        \
	 : DT_SAME_NODE(DT_NODELABEL(mcuuulp_lf_ref_clock), node) ? SIWX91X_CLK_UULP_LF_REF        \
	 : DT_SAME_NODE(DT_NODELABEL(mcu_ulp_clock), node)        ? SIWX91X_CLK_ULP_PROC           \
	 : DT_SAME_NODE(DT_NODELABEL(pll_ref_mux), node)          ? SIWX91X_CLK_PLL_REF            \
	 : DT_SAME_NODE(DT_NODELABEL(soc_pll_clock), node)        ? SIWX91X_CLK_SOC_PLL            \
	 : DT_SAME_NODE(DT_NODELABEL(intf_pll_clock), node)       ? SIWX91X_CLK_INTF_PLL           \
	 : DT_SAME_NODE(DT_NODELABEL(i2s_pll_clock), node)        ? SIWX91X_CLK_I2S_PLL            \
	 : DT_SAME_NODE(DT_NODELABEL(ulp_uart_mux), node)         ? SIWX91X_CLK_ULP_UART           \
	 : DT_SAME_NODE(DT_NODELABEL(ulp_ssi_mux), node)          ? SIWX91X_CLK_ULP_SSI            \
	 : DT_SAME_NODE(DT_NODELABEL(ulp_i2s_mux), node)          ? SIWX91X_CLK_ULP_I2S            \
	 : DT_SAME_NODE(DT_NODELABEL(static_ulp_i2s_mux), node)   ? SIWX91X_CLK_STATIC_ULP_I2S     \
	 : DT_SAME_NODE(DT_NODELABEL(ulp_timer_mux), node)        ? SIWX91X_CLK_ULP_TIMER          \
	 : DT_SAME_NODE(DT_NODELABEL(ulp_slp_sensor_mux), node)   ? SIWX91X_CLK_SLP_SENSOR         \
	 : DT_SAME_NODE(DT_NODELABEL(adc_mux), node)              ? SIWX91X_CLK_ADC                \
	 : DT_SAME_NODE(DT_NODELABEL(uart0_mux), node)            ? SIWX91X_CLK_UART0              \
	 : DT_SAME_NODE(DT_NODELABEL(uart1_mux), node)            ? SIWX91X_CLK_UART1              \
	 : DT_SAME_NODE(DT_NODELABEL(i2c0_mux), node)             ? SIWX91X_CLK_I2C0               \
	 : DT_SAME_NODE(DT_NODELABEL(i2c1_mux), node)             ? SIWX91X_CLK_I2C1               \
	 : DT_SAME_NODE(DT_NODELABEL(gspi_mux), node)             ? SIWX91X_CLK_GSPI               \
	 : DT_SAME_NODE(DT_NODELABEL(qspi_mux), node)             ? SIWX91X_CLK_QSPI               \
	 : DT_SAME_NODE(DT_NODELABEL(i2s0_mux), node)             ? SIWX91X_CLK_I2S0               \
	 : DT_SAME_NODE(DT_NODELABEL(static_i2s0_mux), node)      ? SIWX91X_CLK_STATIC_I2S0        \
	 : DT_SAME_NODE(DT_NODELABEL(pwm_mux), node)              ? SIWX91X_CLK_PWM                \
	 : DT_SAME_NODE(DT_NODELABEL(hpulp_mux), node)            ? SIWX91X_CLK_HPULP              \
								  : SIWX91X_CLK_INVALID)

/*
 * clkid -> clock_control device (compile-time table, runtime lookup).
 * Route/PLL nodes: parent of the labelled node (the clock manager in siwg917.dtsi).
 */
#define SIWX91X_CLOCK_ROUTE_DEV(label)                                                             \
	[SIWX91X_CLOCK_NODE_TO_ID(DT_NODELABEL(label))] =                                          \
		DEVICE_DT_GET(DT_PARENT(DT_NODELABEL(label))),

#define SIWX91X_CLOCK_DEV_AON(clkid)                                                               \
	[clkid] = DEVICE_DT_GET(DT_NODELABEL(aon_clock_manager))

#define SIWX91X_CLOCK_DEV_HP(clkid)                                                                \
	[clkid] = DEVICE_DT_GET(DT_NODELABEL(hp_clock_manager))

#define SIWX91X_CLOCK_DEV_ULP(clkid)                                                               \
	[clkid] = DEVICE_DT_GET(DT_NODELABEL(ulp_clock_manager))

static const struct device *const siwx91x_clk_dev_table[SIWX91X_CLK_INVALID] = {
	/* AON manager children */
	SIWX91X_CLOCK_ROUTE_DEV(mcuhp_ref_clock)
	SIWX91X_CLOCK_ROUTE_DEV(mcuulp_ref_clock)
	SIWX91X_CLOCK_ROUTE_DEV(mcuuulp_hf_ref_clock)
	SIWX91X_CLOCK_ROUTE_DEV(mcuuulp_lf_ref_clock)
	/* ULP manager children */
	SIWX91X_CLOCK_ROUTE_DEV(mcu_ulp_clock)
	SIWX91X_CLOCK_ROUTE_DEV(ulp_uart_mux)
	SIWX91X_CLOCK_ROUTE_DEV(ulp_i2s_mux)
	SIWX91X_CLOCK_ROUTE_DEV(ulp_ssi_mux)
	SIWX91X_CLOCK_ROUTE_DEV(adc_mux)
	SIWX91X_CLOCK_ROUTE_DEV(ulp_timer_mux)
	SIWX91X_CLOCK_ROUTE_DEV(ulp_slp_sensor_mux)
	/* HP manager children */
	SIWX91X_CLOCK_ROUTE_DEV(pll_ref_mux)
	SIWX91X_CLOCK_ROUTE_DEV(uart0_mux)
	SIWX91X_CLOCK_ROUTE_DEV(uart1_mux)
	SIWX91X_CLOCK_ROUTE_DEV(i2c0_mux)
	SIWX91X_CLOCK_ROUTE_DEV(i2c1_mux)
	SIWX91X_CLOCK_ROUTE_DEV(gspi_mux)
	SIWX91X_CLOCK_ROUTE_DEV(qspi_mux)
	SIWX91X_CLOCK_ROUTE_DEV(i2s0_mux)
	SIWX91X_CLOCK_ROUTE_DEV(pwm_mux)
	SIWX91X_CLOCK_ROUTE_DEV(soc_pll_clock)
	SIWX91X_CLOCK_ROUTE_DEV(intf_pll_clock)
	SIWX91X_CLOCK_ROUTE_DEV(i2s_pll_clock)
	/* /clocks oscillators (get_rate in AON driver) */
	SIWX91X_CLOCK_DEV_AON(SIWX91X_CLK_XTAL_MHZ),
	SIWX91X_CLOCK_DEV_AON(SIWX91X_CLK_XTAL_KHZ),
	SIWX91X_CLOCK_DEV_AON(SIWX91X_CLK_RC_MHZ),
	SIWX91X_CLOCK_DEV_AON(SIWX91X_CLK_RC_KHZ),
	/* Gate-only or synthetic clkids */
	SIWX91X_CLOCK_DEV_ULP(SIWX91X_CLK_ULP_I2C),
	SIWX91X_CLOCK_DEV_ULP(SIWX91X_CLK_ULP_DMA),
	SIWX91X_CLOCK_DEV_HP(SIWX91X_CLK_DMA0),
	SIWX91X_CLOCK_DEV_AON(SIWX91X_CLK_WATCHDOG),
	SIWX91X_CLOCK_DEV_AON(SIWX91X_CLK_RTC),
	SIWX91X_CLOCK_DEV_HP(SIWX91X_CLK_STATIC_I2S0),
	SIWX91X_CLOCK_DEV_ULP(SIWX91X_CLK_STATIC_ULP_I2S),
	SIWX91X_CLOCK_DEV_HP(SIWX91X_CLK_GPDMA0),
	SIWX91X_CLOCK_DEV_HP(SIWX91X_CLK_RNG),
	SIWX91X_CLOCK_DEV_AON(SIWX91X_CLK_SYSRTC),
	SIWX91X_CLOCK_DEV_HP(SIWX91X_CLK_HPULP),
	SIWX91X_CLOCK_DEV_ULP(SIWX91X_CLK_ULP_GPIO),
};

static inline const struct device *siwx91x_clock_control_get_device(uint32_t clkid)
{
	if (clkid >= SIWX91X_CLK_INVALID) {
		return NULL;
	}

	return siwx91x_clk_dev_table[clkid];
}

#define SIWX91X_CLOCK_ROUTE_CLOCK_DIV(node)                                                        \
	COND_CODE_1(DT_NODE_HAS_PROP(node, clock_div), (DT_PROP(node, clock_div)), (0U))

#define SIWX91X_CLOCK_ROUTE_NODE_INIT(node)                                                        \
	{                                                                                          \
		.clkid = SIWX91X_CLOCK_NODE_TO_ID(node),                                           \
		.ref_clkid = SIWX91X_CLOCK_NODE_TO_ID(DT_CLOCKS_CTLR(node)),                       \
		.clock_div = SIWX91X_CLOCK_ROUTE_CLOCK_DIV(node),                                  \
	},

#define SIWX91X_CLOCK_PLL_NODE_INIT(node)                                                          \
	{                                                                                          \
		.clkid = SIWX91X_CLOCK_NODE_TO_ID(node),                                           \
		.ref_clkid = SIWX91X_CLOCK_NODE_TO_ID(DT_CLOCKS_CTLR(node)),                       \
		.frequency = DT_PROP(node, clock_frequency),                                       \
	},

#define SIWX91X_CLOCK_MANAGER_CHILD_INIT(node)                                                     \
	IF_ENABLED(DT_NODE_HAS_COMPAT(node, silabs_siwx91x_clock_route),                           \
		    (SIWX91X_CLOCK_ROUTE_NODE_INIT(node)))

#define SIWX91X_CLOCK_MANAGER_PLL_INIT(node)                                                       \
	IF_ENABLED(DT_NODE_HAS_COMPAT(node, silabs_siwx91x_pll),                                     \
		   (SIWX91X_CLOCK_PLL_NODE_INIT(node)))

#endif /* ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_SILABS_SIWX91X_H_ */
