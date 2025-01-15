/*
 * Copyright (c) 2025 Silicon Laboratories Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_DMA_DMA_SILABS_LDMA_H_
#define ZEPHYR_INCLUDE_DRIVERS_DMA_DMA_SILABS_LDMA_H_

#include <zephyr/drivers/dma.h>
#include <zephyr/kernel.h>
#include <zephyr/types.h>

/**
 * @brief Vendror-specific DMA function.
 *
 */

int silabs_ldma_append_block(const struct device *dev, uint32_t channel,
					struct dma_config *config);

#endif /* ZEPHYR_INCLUDE_DRIVERS_DMA_DMA_SILABS_LDMA_H_*/
