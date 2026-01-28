/*
 * Copyright (c) 2025 Silicon Laboratories Inc.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Generic stub driver for clock control.
 * This file provides empty implementations of clock control functions.
 */

#include <zephyr/drivers/clock_control.h>

#define DT_DRV_COMPAT zephyr_clock_control_fake

static int clock_fake_init(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 0;
}

static DEVICE_API(clock_control, clock_fake_api) = {
	.on = NULL,
	.off = NULL,
	.get_rate = NULL,
	.set_rate = NULL,
	.get_status = NULL,
};

#define CLOCK_FAKE_INIT(inst)                                                              \
	DEVICE_DT_INST_DEFINE(inst, clock_fake_init, NULL, NULL, NULL, PRE_KERNEL_1,       \
			      CONFIG_CLOCK_CONTROL_INIT_PRIORITY, &clock_fake_api);

DT_INST_FOREACH_STATUS_OKAY(CLOCK_FAKE_INIT)
