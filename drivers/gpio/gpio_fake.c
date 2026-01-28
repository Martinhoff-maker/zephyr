/*
 * Copyright (c) 2025 Silicon Laboratories Inc.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Generic stub driver for GPIO.
 * This file provides empty implementations of GPIO functions.
 */

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_utils.h>

#define DT_DRV_COMPAT zephyr_gpio_fake

static int gpio_fake_init(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 0;
}

static DEVICE_API(gpio, gpio_fake_api) = {
	.pin_configure = NULL,
	.port_get_raw = NULL,
	.port_set_masked_raw = NULL,
	.port_set_bits_raw = NULL,
	.port_clear_bits_raw = NULL,
	.port_toggle_bits = NULL,
	.pin_interrupt_configure = NULL,
	.manage_callback = NULL,
#ifdef CONFIG_GPIO_GET_DIRECTION
	.port_get_direction = NULL,
#endif
};

#define GPIO_PORT_FAKE_INIT(n)                                                                     \
	DEVICE_DT_DEFINE(n, gpio_fake_init, NULL, NULL, NULL, PRE_KERNEL_1,                        \
			 CONFIG_GPIO_INIT_PRIORITY, &gpio_fake_api);

#define GPIO_FAKE_INIT(inst)                                                                       \
	const struct device *ports_##inst[DT_INST_CHILD_NUM(inst)];                                \
	DEVICE_DT_INST_DEFINE(inst, gpio_fake_init, NULL, NULL, NULL, PRE_KERNEL_1,                \
			      CONFIG_GPIO_INIT_PRIORITY, &gpio_fake_api);                          \
	DT_INST_FOREACH_CHILD_STATUS_OKAY(inst, GPIO_PORT_FAKE_INIT);

DT_INST_FOREACH_STATUS_OKAY(GPIO_FAKE_INIT)
