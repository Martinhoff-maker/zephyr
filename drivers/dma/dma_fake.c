/*
 * Copyright (c) 2025 Silicon Laboratories Inc.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Generic fake driver for DMA.
 * This file provides empty implementations of DMA functions
 */

#include <zephyr/device.h>
#include <zephyr/drivers/dma.h>

#define DT_DRV_COMPAT zephyr_dma_fake

static int dma_fake_init(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 0;
}

static DEVICE_API(dma, dma_fake_api) = {
	.config = NULL,
	.reload = NULL,
	.start = NULL,
	.stop = NULL,
	.get_status = NULL,
	.chan_filter = NULL,
};

#define DMA_FAKE_INIT(inst)                          						   \
	DEVICE_DT_INST_DEFINE(inst, dma_fake_init, NULL, NULL, NULL, POST_KERNEL,                  \
			      CONFIG_DMA_INIT_PRIORITY, &dma_fake_api);

DT_INST_FOREACH_STATUS_OKAY(DMA_FAKE_INIT)

#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT zephyr_dma_fake_2cell

DT_INST_FOREACH_STATUS_OKAY(DMA_FAKE_INIT)
