/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @addtogroup t_driver_uart
 * @{
 * @defgroup t_uart_basic test_uart_basic_operations
 * @}
 */

#include "test_uart.h"
#include <zephyr/pm/device_runtime.h>

#ifdef CONFIG_SHELL
TC_CMD_DEFINE(test_uart_configure)
TC_CMD_DEFINE(test_uart_config_get)
TC_CMD_DEFINE(test_uart_poll_out)
TC_CMD_DEFINE(test_uart_poll_in)
#if CONFIG_UART_INTERRUPT_DRIVEN
TC_CMD_DEFINE(test_uart_fifo_read)
TC_CMD_DEFINE(test_uart_fifo_fill)
TC_CMD_DEFINE(test_uart_pending)
#endif

SHELL_CMD_REGISTER(test_uart_configure, NULL, NULL,
			TC_CMD_ITEM(test_uart_configure));
SHELL_CMD_REGISTER(test_uart_config_get, NULL, NULL,
			TC_CMD_ITEM(test_uart_config_get));
SHELL_CMD_REGISTER(test_uart_poll_in, NULL, NULL,
			TC_CMD_ITEM(test_uart_poll_in));
SHELL_CMD_REGISTER(test_uart_poll_out, NULL, NULL,
			TC_CMD_ITEM(test_uart_poll_out));
#if CONFIG_UART_INTERRUPT_DRIVEN
SHELL_CMD_REGISTER(test_uart_fifo_read, NULL, NULL,
			TC_CMD_ITEM(test_uart_fifo_read));
SHELL_CMD_REGISTER(test_uart_fifo_fill, NULL, NULL,
			TC_CMD_ITEM(test_uart_fifo_fill));
SHELL_CMD_REGISTER(test_uart_pending, NULL, NULL,
			TC_CMD_ITEM(test_uart_pending));
#endif


#endif

void *uart_basic_setup(void)
{
#if DT_NODE_HAS_COMPAT(DT_CHOSEN(zephyr_console), zephyr_cdc_acm_uart)
	const struct device *dev;
	uint32_t dtr = 0;

	dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

	if (!device_is_ready(dev)) {
		return NULL;
	}

	while (!dtr) {
		uart_line_ctrl_get(dev, UART_LINE_CTRL_DTR, &dtr);
		k_sleep(K_MSEC(100));
	}
#endif

	(void)pm_device_runtime_get(DEVICE_DT_GET(DT_CHOSEN(zephyr_console)));

	return NULL;
}
	
void uart_basic_teardown(void *args)
{
	(void)args;
	(void)pm_device_runtime_put(DEVICE_DT_GET(DT_CHOSEN(zephyr_console)));
}

#ifndef CONFIG_SHELL
ZTEST_SUITE(uart_basic_api, NULL, uart_basic_setup, NULL, NULL, uart_basic_teardown);

/* The UART pending test should be test finally. */
ZTEST_SUITE(uart_basic_api_pending, NULL, uart_basic_setup, NULL, NULL, uart_basic_teardown);
#endif
