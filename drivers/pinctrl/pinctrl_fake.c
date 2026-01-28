/*
 * Copyright (c) 2025 Silicon Laboratories Inc.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Generic fake driver for pinctrl.
 * This file provides empty implementations of pinctrl functions.
 */

#include <zephyr/drivers/pinctrl.h>

#define DT_DRV_COMPAT zephyr_pinctrl_fake

int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt, uintptr_t reg)
{
	ARG_UNUSED(pins);
	ARG_UNUSED(pin_cnt);
	ARG_UNUSED(reg);
	return 0;
}
