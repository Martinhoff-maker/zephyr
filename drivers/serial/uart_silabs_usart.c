/*
 * Copyright (c) 2024 Silicon Laboratories Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/clock_control_silabs.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>

#include <em_usart.h>

LOG_MODULE_REGISTER(uart_silabs, CONFIG_UART_LOG_LEVEL);

#ifdef CONFIG_UART_ASYNC_API
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/dma/dma_silabs_ldma.h>
#endif

#define DT_DRV_COMPAT silabs_usart

/**
 * @brief Config struct for UART
 */

#ifdef CONFIG_UART_ASYNC_API
struct uart_dma_channel {
	const struct device *dma_dev;
	uint32_t dma_channel;
	struct dma_config dma_cfg;
	uint8_t priority;
	int fifo_threshold;
	struct dma_block_config blk_cfg;
	uint8_t *buffer;
	size_t buffer_length;
	volatile size_t counter;
	size_t offset;
	int32_t timeout;
	bool enabled;
};
#endif
struct uart_silabs_config {
	const struct pinctrl_dev_config *pcfg;
	const struct device *clock_dev;
	const struct silabs_clock_control_cmu_config clock_cfg;
	USART_TypeDef *base;
	uint32_t baud_rate;
#if defined(CONFIG_UART_INTERRUPT_DRIVEN) || defined(CONFIG_UART_ASYNC_API)
	void (*irq_config_func)(const struct device *dev);
#endif
};

struct uart_silabs_data {
	struct uart_config uart_cfg;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_callback_user_data_t callback;
	void *cb_data;
#endif
#ifdef CONFIG_UART_ASYNC_API
	const struct device *uart_dev;
	uart_callback_t async_cb;
	void *async_user_data;
	struct uart_dma_channel dma_rx;
	struct uart_dma_channel dma_tx;
	uint8_t *rx_next_buffer;
	size_t rx_next_buffer_len;
#endif
};

static int uart_silabs_poll_in(const struct device *dev, unsigned char *c)
{
	const struct uart_silabs_config *config = dev->config;
	uint32_t flags = USART_StatusGet(config->base);

	if (flags & USART_STATUS_RXDATAV) {
		*c = USART_Rx(config->base);
		return 0;
	}

	return -1;
}

static void uart_silabs_poll_out(const struct device *dev, unsigned char c)
{
	const struct uart_silabs_config *config = dev->config;

	USART_Tx(config->base, c);
}

static int uart_silabs_err_check(const struct device *dev)
{
	const struct uart_silabs_config *config = dev->config;
	uint32_t flags = USART_IntGet(config->base);
	int err = 0;

	// TODO Oveflow might not be treated as error ?
	if (flags & USART_IF_RXOF) {
		err |= UART_ERROR_OVERRUN;
	}

	if (flags & USART_IF_PERR) {
		err |= UART_ERROR_PARITY;
	}

	if (flags & USART_IF_FERR) {
		err |= UART_ERROR_FRAMING;
	}

	USART_IntClear(config->base, USART_IF_RXOF | USART_IF_PERR | USART_IF_FERR);

	return err;
}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static int uart_silabs_fifo_fill(const struct device *dev, const uint8_t *tx_data, int len)
{
	const struct uart_silabs_config *config = dev->config;
	int num_tx = 0U;

	while ((len - num_tx > 0) && (config->base->STATUS & USART_STATUS_TXBL)) {

		config->base->TXDATA = (uint32_t)tx_data[num_tx++];
	}

	return num_tx;
}

static int uart_silabs_fifo_read(const struct device *dev, uint8_t *rx_data, const int len)
{
	const struct uart_silabs_config *config = dev->config;
	int num_rx = 0U;

	while ((len - num_rx > 0) && (config->base->STATUS & USART_STATUS_RXDATAV)) {

		rx_data[num_rx++] = (uint8_t)config->base->RXDATA;
	}

	return num_rx;
}

static void uart_silabs_irq_tx_enable(const struct device *dev)
{
	const struct uart_silabs_config *config = dev->config;
	uint32_t mask = USART_IEN_TXBL | USART_IEN_TXC;

	USART_IntEnable(config->base, mask);
}

static void uart_silabs_irq_tx_disable(const struct device *dev)
{
	const struct uart_silabs_config *config = dev->config;
	uint32_t mask = USART_IEN_TXBL | USART_IEN_TXC;

	USART_IntDisable(config->base, mask);
}

static int uart_silabs_irq_tx_complete(const struct device *dev)
{
	const struct uart_silabs_config *config = dev->config;
	uint32_t flags = USART_IntGet(config->base);

	USART_IntClear(config->base, USART_IF_TXC);

	return (flags & USART_IF_TXC) != 0U;
}

static int uart_silabs_irq_tx_ready(const struct device *dev)
{
	const struct uart_silabs_config *config = dev->config;
	uint32_t flags = USART_IntGetEnabled(config->base);

	return (flags & USART_IF_TXBL) != 0U;
}

static void uart_silabs_irq_rx_enable(const struct device *dev)
{
	const struct uart_silabs_config *config = dev->config;
	uint32_t mask = USART_IEN_RXDATAV;

	USART_IntEnable(config->base, mask);
}

static void uart_silabs_irq_rx_disable(const struct device *dev)
{
	const struct uart_silabs_config *config = dev->config;
	uint32_t mask = USART_IEN_RXDATAV;

	USART_IntDisable(config->base, mask);
}

static int uart_silabs_irq_rx_full(const struct device *dev)
{
	const struct uart_silabs_config *config = dev->config;
	uint32_t flags = USART_IntGet(config->base);

	return (flags & USART_IF_RXDATAV) != 0U;
}

static int uart_silabs_irq_rx_ready(const struct device *dev)
{
	const struct uart_silabs_config *config = dev->config;
	uint32_t mask = USART_IEN_RXDATAV;

	return (config->base->IEN & mask) && uart_silabs_irq_rx_full(dev);
}

static void uart_silabs_irq_err_enable(const struct device *dev)
{
	const struct uart_silabs_config *config = dev->config;

	USART_IntEnable(config->base, USART_IF_RXOF | USART_IF_PERR | USART_IF_FERR);
}

static void uart_silabs_irq_err_disable(const struct device *dev)
{
	const struct uart_silabs_config *config = dev->config;

	USART_IntDisable(config->base, USART_IF_RXOF | USART_IF_PERR | USART_IF_FERR);
}

static int uart_silabs_irq_is_pending(const struct device *dev)
{
	return uart_silabs_irq_tx_ready(dev) || uart_silabs_irq_rx_ready(dev);
}

static int uart_silabs_irq_update(const struct device *dev)
{
	return 1;
}

static void uart_silabs_irq_callback_set(const struct device *dev, uart_irq_callback_user_data_t cb,
					 void *cb_data)
{
	struct uart_silabs_data *data = dev->data;

	data->callback = cb;
	data->cb_data = cb_data;
}
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

#ifdef CONFIG_UART_ASYNC_API

static inline void async_user_callback(struct uart_silabs_data *data, struct uart_event *event)
{
	if (data->async_cb) {
		data->async_cb(data->uart_dev, event, data->async_user_data);
	}
}

static inline void async_evt_rx_rdy(struct uart_silabs_data *data)
{
	struct uart_event event = {.type = UART_RX_RDY,
				   .data.rx.buf = data->dma_rx.buffer,
				   .data.rx.len = data->dma_rx.counter - data->dma_rx.offset,
				   .data.rx.offset = data->dma_rx.offset};

	data->dma_rx.offset = data->dma_rx.counter;

	if (event.data.rx.len > 0) {
		async_user_callback(data, &event);
	}
}

static inline void async_evt_tx_done(struct uart_silabs_data *data)
{
	struct uart_event event = {.type = UART_TX_DONE,
				   .data.tx.buf = data->dma_tx.buffer,
				   .data.tx.len = data->dma_tx.counter};

	data->dma_tx.buffer_length = 0;
	data->dma_tx.counter = 0;

	async_user_callback(data, &event);
}

static inline void async_evt_tx_abort(struct uart_silabs_data *data)
{
	struct uart_event event = {.type = UART_TX_ABORTED,
				   .data.tx.buf = data->dma_tx.buffer,
				   .data.tx.len = data->dma_tx.counter};

	data->dma_tx.buffer_length = 0;
	data->dma_tx.counter = 0;

	async_user_callback(data, &event);
}

static inline void async_evt_rx_err(struct uart_silabs_data *data, int err_code)
{
	struct uart_event event = {.type = UART_RX_STOPPED,
				   .data.rx_stop.reason = err_code,
				   .data.rx_stop.data.len = data->dma_rx.counter,
				   .data.rx_stop.data.offset = 0,
				   .data.rx_stop.data.buf = data->dma_rx.buffer};

	async_user_callback(data, &event);
}

static inline void async_evt_rx_buf_release(struct uart_silabs_data *data)
{
	struct uart_event evt = {
		.type = UART_RX_BUF_RELEASED,
		.data.rx_buf.buf = data->dma_rx.buffer,
	};

	async_user_callback(data, &evt);
}

static inline void async_evt_rx_buf_request(struct uart_silabs_data *data)
{
	struct uart_event evt = {
		.type = UART_RX_BUF_REQUEST,
	};

	async_user_callback(data, &evt);
}

static int uart_silabs_async_callback_set(const struct device *dev, uart_callback_t callback,
					  void *user_data)
{
	struct uart_silabs_data *data = dev->data;

	data->async_cb = callback;
	data->async_user_data = user_data;

	return 0;
}

static void uart_silabs_dma_replace_buffer(const struct device *dev)
{
	struct uart_silabs_data *data = dev->data;

	data->dma_rx.offset = 0;
	data->dma_rx.counter = 0;
	data->dma_rx.buffer = data->rx_next_buffer;
	data->dma_rx.buffer_length = data->rx_next_buffer_len;
	data->rx_next_buffer = NULL;
	data->rx_next_buffer_len = 0;

	async_evt_rx_buf_request(data);
}

static void uart_silabs_dma_rx_flush(struct uart_silabs_data *data)
{
	struct dma_status stat;
	size_t rx_rcv_len;

	if (!dma_get_status(data->dma_rx.dma_dev, data->dma_rx.dma_channel, &stat)) {
		rx_rcv_len = data->dma_rx.buffer_length - stat.pending_length;
		if (rx_rcv_len > data->dma_rx.offset) {
			data->dma_rx.counter = rx_rcv_len;
			async_evt_rx_rdy(data);
		}
	}
}

void uart_silabs_dma_rx_cb(const struct device *dma_dev, void *user_data, uint32_t channel,
			   int status)
{
	const struct device *uart_dev = user_data;
	struct uart_silabs_data *data = uart_dev->data;
	struct uart_event disabled_event = {.type = UART_RX_DISABLED};

	if (status < 0) {
		async_evt_rx_err(data, status);
		return;
	}

	data->dma_rx.counter = data->dma_rx.buffer_length;

	async_evt_rx_rdy(data);

	if (data->rx_next_buffer != NULL) {
		async_evt_rx_buf_release(data);
		uart_silabs_dma_replace_buffer(uart_dev);
	} else {
		dma_stop(data->dma_rx.dma_dev, data->dma_rx.dma_channel);
		data->dma_rx.enabled = false;
		async_evt_rx_buf_release(data);
		async_user_callback(data, &disabled_event);
	}
}

void uart_silabs_dma_tx_cb(const struct device *dma_dev, void *user_data, uint32_t channel,
			   int status)
{
	const struct device *uart_dev = user_data;
	struct uart_silabs_data *data = uart_dev->data;

	dma_stop(data->dma_tx.dma_dev, data->dma_tx.dma_channel);
	data->dma_tx.enabled = false;
}

static int uart_silabs_async_tx(const struct device *dev, const uint8_t *tx_data, size_t buf_size,
				int32_t timeout)
{
	const struct uart_silabs_config *config = dev->config;
	struct uart_silabs_data *data = dev->data;
	int ret;

	if (data->dma_tx.dma_dev == NULL) {
		return -ENODEV;
	}

	if (data->dma_tx.buffer_length != 0) {
		return -EBUSY;
	}

	data->dma_tx.buffer = (uint8_t *)tx_data;
	data->dma_tx.buffer_length = buf_size;
	data->dma_tx.timeout = timeout;
	data->dma_tx.blk_cfg.source_address = (uint32_t)data->dma_tx.buffer;
	data->dma_tx.blk_cfg.block_size = data->dma_tx.buffer_length;

	USART_IntClear(config->base, USART_IF_TXC | USART_IF_TCMP2);
	USART_IntEnable(config->base, USART_IF_TXC);
	if (timeout >= 0) {
		USART_IntEnable(config->base, USART_IF_TCMP2);
	}

	ret = dma_config(data->dma_tx.dma_dev, data->dma_tx.dma_channel, &data->dma_tx.dma_cfg);
	if (ret) {
		LOG_ERR("dma tx config error!");
		return ret;
	}

	ret = dma_start(data->dma_tx.dma_dev, data->dma_tx.dma_channel);
	if (ret) {
		LOG_ERR("UART err: TX DMA start failed!");
		return ret;
	}

	data->dma_tx.enabled = true;

	return 0;
}

static int uart_silabs_async_tx_abort(const struct device *dev)
{
	const struct uart_silabs_config *config = dev->config;
	struct uart_silabs_data *data = dev->data;
	size_t tx_buffer_length = data->dma_tx.buffer_length;
	struct dma_status stat;

	if (tx_buffer_length == 0) {
		return -EFAULT;
	}

	USART_IntClear(config->base, USART_IF_TXC | USART_IF_TCMP2);
	USART_IntDisable(config->base, USART_IF_TXC);
	USART_IntDisable(config->base, USART_IF_TCMP2);

	if (!dma_get_status(data->dma_tx.dma_dev, data->dma_tx.dma_channel, &stat)) {
		data->dma_tx.counter = tx_buffer_length - stat.pending_length;
	}

	dma_stop(data->dma_tx.dma_dev, data->dma_tx.dma_channel);
	data->dma_tx.enabled = false;

	async_evt_tx_abort(data);

	return 0;
}

static int uart_silabs_async_rx_enable(const struct device *dev, uint8_t *rx_buf, size_t buf_size,
				       int32_t timeout)
{
	const struct uart_silabs_config *config = dev->config;
	struct uart_silabs_data *data = dev->data;
	int ret;

	if (data->dma_rx.dma_dev == NULL) {
		return -ENODEV;
	}

	if (data->dma_rx.enabled) {
		LOG_WRN("RX was already enabled");
		return -EBUSY;
	}

	data->dma_rx.offset = 0;
	data->dma_rx.buffer = rx_buf;
	data->dma_rx.buffer_length = buf_size;
	data->dma_rx.counter = 0;
	data->dma_rx.timeout = timeout;
	data->dma_rx.blk_cfg.block_size = buf_size;
	data->dma_rx.blk_cfg.dest_address = (uint32_t)data->dma_rx.buffer;

	ret = dma_config(data->dma_rx.dma_dev, data->dma_rx.dma_channel, &data->dma_rx.dma_cfg);

	if (ret != 0) {
		LOG_ERR("UART ERR: RX DMA config failed!");
		return -EINVAL;
	}

	if (dma_start(data->dma_rx.dma_dev, data->dma_rx.dma_channel)) {
		LOG_ERR("UART ERR: RX DMA start failed!");
		return -EFAULT;
	}

	USART_IntClear(config->base, USART_IF_RXOF | USART_IF_TCMP1);
	USART_IntEnable(config->base, USART_IF_RXOF);

	if (timeout >= 0) {
		USART_IntEnable(config->base, USART_IF_TCMP1);
	}

	data->dma_rx.enabled = true;

	async_evt_rx_buf_request(data);

	return ret;
}

static int uart_silabs_async_rx_disable(const struct device *dev)
{
	const struct uart_silabs_config *config = dev->config;
	USART_TypeDef *usart = config->base;
	struct uart_silabs_data *data = dev->data;
	struct uart_event disabled_event = {.type = UART_RX_DISABLED};

	if (!data->dma_rx.enabled) {
		async_user_callback(data, &disabled_event);
		return -EFAULT;
	}

	dma_stop(data->dma_rx.dma_dev, data->dma_rx.dma_channel);

	USART_IntClear(usart, USART_IF_RXOF | USART_IF_TCMP1);
	USART_IntDisable(usart, USART_IF_RXOF);
	USART_IntDisable(usart, USART_IF_TCMP1);

	if (!(data->dma_rx.enabled)) {
		usart->CMD = USART_CMD_CLEARRX;
	}

	uart_silabs_dma_rx_flush(data);

	async_evt_rx_buf_release(data);

	if (data->rx_next_buffer) {
		struct uart_event rx_next_buf_release_evt = {
			.type = UART_RX_BUF_RELEASED,
			.data.rx_buf.buf = data->rx_next_buffer,
		};
		async_user_callback(data, &rx_next_buf_release_evt);
	}

	data->rx_next_buffer = NULL;
	data->rx_next_buffer_len = 0;

	data->dma_rx.enabled = false;

	async_user_callback(data, &disabled_event);

	return 0;
}

static int uart_silabs_async_rx_buf_rsp(const struct device *dev, uint8_t *buf, size_t len)
{
	struct uart_silabs_data *data = dev->data;
	unsigned int key;
	int ret;

	key = irq_lock();

	if (data->rx_next_buffer != NULL) {
		return -EBUSY;
	} else if (!data->dma_rx.enabled) {
		return -EACCES;
	} else {
		data->rx_next_buffer = buf;
		data->rx_next_buffer_len = len;
		data->dma_rx.blk_cfg.dest_address = (uint32_t)buf;
		data->dma_rx.blk_cfg.block_size = len;
	}

	irq_unlock(key);

	ret = silabs_ldma_append_block(data->dma_rx.dma_dev, data->dma_rx.dma_channel,
				       &data->dma_rx.dma_cfg);
	if (ret != 0) {
		LOG_ERR("UART ERR: RX DMA append failed!");
		return -EINVAL;
	}

	return ret;
}

static int uart_silabs_async_init(const struct device *dev)
{
	const struct uart_silabs_config *config = dev->config;
	USART_TypeDef *usart = config->base;
	struct uart_silabs_data *data = dev->data;

	data->uart_dev = dev;

	if (data->dma_rx.dma_dev != NULL) {
		if (!device_is_ready(data->dma_rx.dma_dev)) {
			return -ENODEV;
		}
	}

	if (data->dma_tx.dma_dev != NULL) {
		if (!device_is_ready(data->dma_tx.dma_dev)) {
			return -ENODEV;
		}
	}

	data->dma_rx.enabled = false;
	data->dma_tx.enabled = false;

	memset(&data->dma_rx.blk_cfg, 0, sizeof(data->dma_rx.blk_cfg));
	data->dma_rx.blk_cfg.source_address = (uintptr_t)&(usart->RXDATA);
	data->dma_rx.blk_cfg.dest_address = 0;
	data->dma_rx.blk_cfg.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	data->dma_rx.blk_cfg.dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	data->dma_rx.dma_cfg.complete_callback_en = 1;
	data->dma_rx.dma_cfg.channel_priority = 3;
	data->dma_rx.dma_cfg.head_block = &data->dma_rx.blk_cfg;
	data->dma_rx.dma_cfg.user_data = (void *)dev;
	data->rx_next_buffer = NULL;
	data->rx_next_buffer_len = 0;

	memset(&data->dma_tx.blk_cfg, 0, sizeof(data->dma_tx.blk_cfg));
	data->dma_tx.blk_cfg.dest_address = (uintptr_t)&(usart->TXDATA);
	data->dma_tx.blk_cfg.source_address = 0;
	data->dma_tx.blk_cfg.source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	data->dma_tx.blk_cfg.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	data->dma_tx.dma_cfg.complete_callback_en = 1;
	data->dma_tx.dma_cfg.head_block = &data->dma_tx.blk_cfg;
	data->dma_tx.dma_cfg.user_data = (void *)dev;

	config->base->CMD = USART_CMD_CLEARRX | USART_CMD_CLEARTX;
	config->base->TIMECMP1 = USART_TIMECMP1_TSTOP_RXACT | USART_TIMECMP1_TSTART_RXEOF |
				 USART_TIMECMP1_RESTARTEN | (0xff << _USART_TIMECMP1_TCMPVAL_SHIFT);
	config->base->TIMECMP2 = USART_TIMECMP2_TSTOP_TXST | USART_TIMECMP2_TSTART_TXEOF |
				 USART_TIMECMP2_RESTARTEN | (0xff << _USART_TIMECMP2_TCMPVAL_SHIFT);

	USART_Enable(config->base, usartEnable);

	return 0;
}

#endif /* CONFIG_UART_ASYNC_API */

static void uart_silabs_isr(const struct device *dev)
{
	struct uart_silabs_data *data = dev->data;
#ifdef CONFIG_UART_ASYNC_API
	const struct uart_silabs_config *config = dev->config;
	USART_TypeDef *usart = config->base;
	struct dma_status stat;
#endif
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	if (data->callback) {
		data->callback(dev, data->cb_data);
	}
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
#ifdef CONFIG_UART_ASYNC_API
	if (usart->IF & USART_IF_TCMP1) {
		uart_silabs_dma_rx_flush(data);

		usart->TIMECMP1 &= ~_USART_TIMECMP1_TSTART_MASK;
		usart->TIMECMP1 |= USART_TIMECMP1_TSTART_RXEOF;

		USART_IntClear(usart, USART_IF_TCMP1);

		return;
	}
	if (usart->IF & USART_IF_RXOF) {
		LOG_DBG("Rx overflow interrupt occurred");

		// TODO: handle overflow error
		USART_IntClear(usart, USART_IF_RXOF);
		return;
	}
	if (usart->IF & USART_IF_TXC) {
		if (!dma_get_status(data->dma_tx.dma_dev, data->dma_tx.dma_channel, &stat)) {
			data->dma_tx.counter = data->dma_tx.buffer_length - stat.pending_length;
		}

		if (data->dma_tx.counter == data->dma_tx.buffer_length) {

			USART_IntDisable(config->base, USART_IF_TXC);
			USART_IntDisable(config->base, USART_IF_TCMP2);
			USART_IntClear(usart, USART_IF_TXC | USART_IF_TCMP2);

			usart->TIMECMP2 &= ~_USART_TIMECMP2_TSTART_MASK;
			usart->TIMECMP2 |= USART_TIMECMP2_TSTART_DISABLE;
		}

		async_evt_tx_done(data);
	}
	if (usart->IF & USART_IF_TCMP2) {

		if (dma_get_status(data->dma_tx.dma_dev, data->dma_tx.dma_channel, &stat) == 0) {
			size_t tx_snd_len = data->dma_tx.counter - stat.pending_length;
			data->dma_tx.counter = tx_snd_len;
			async_evt_tx_abort(data);
		}

		usart->TIMECMP2 &= ~_USART_TIMECMP2_TSTART_MASK;
		usart->TIMECMP2 |= USART_TIMECMP2_TSTART_DISABLE;

		uart_silabs_async_tx_abort(dev);

		USART_IntClear(usart, USART_IF_TCMP2);

		return;
	}

	uart_silabs_err_check(dev);
#endif /* CONFIG_UART_ASYNC_API */
}

static inline USART_Parity_TypeDef uart_silabs_usart_cfg2ll_parity(
	enum uart_config_parity parity)
{
	switch (parity) {
	case UART_CFG_PARITY_ODD:
		return usartOddParity;
	case UART_CFG_PARITY_EVEN:
		return usartEvenParity;
	case UART_CFG_PARITY_NONE:
	default:
		return usartNoParity;
	}
}

static inline USART_Stopbits_TypeDef uart_silabs_usart_cfg2ll_stopbits(
	enum uart_config_stop_bits sb)
{
	switch (sb) {
	case UART_CFG_STOP_BITS_0_5:
		return usartStopbits0p5;
	case UART_CFG_STOP_BITS_1:
		return usartStopbits1;
	case UART_CFG_STOP_BITS_2:
		return usartStopbits2;
	case UART_CFG_STOP_BITS_1_5:
		return usartStopbits1p5;
	default:
		return usartStopbits1;
	}
}

static inline USART_Databits_TypeDef uart_silabs_usart_cfg2ll_databits(
	enum uart_config_data_bits db, enum uart_config_parity p)
{
	switch (db) {
	case UART_CFG_DATA_BITS_7:
		if (p == UART_CFG_PARITY_NONE) {
			return usartDatabits7;
		} else {
			return usartDatabits8;
		}
	case UART_CFG_DATA_BITS_9:
		return usartDatabits9;
	case UART_CFG_DATA_BITS_8:
	default:
		if (p == UART_CFG_PARITY_NONE) {
			return usartDatabits8;
		} else {
			return usartDatabits9;
		}
		return usartDatabits8;
	}
}

static inline USART_HwFlowControl_TypeDef uart_silabs_usart_cfg2ll_hwctrl(
	enum uart_config_flow_control fc)
{
	if (fc == UART_CFG_FLOW_CTRL_RTS_CTS) {
		return usartHwFlowControlCtsAndRts;
	}

	return usartHwFlowControlNone;
}

/**
 * @brief Main initializer for UART
 *
 * @param dev UART device to be initialized
 * @return int 0
 */
static int uart_silabs_init(const struct device *dev)
{
	int err;
	const struct uart_silabs_config *config = dev->config;
	struct uart_silabs_data *data = dev->data;
	struct uart_config *uart_cfg = &data->uart_cfg;
	USART_InitAsync_TypeDef usartInit = USART_INITASYNC_DEFAULT;

	/* The peripheral and gpio clock are already enabled from soc and gpio driver */
	/* Enable USART clock */
	err = clock_control_on(config->clock_dev, (clock_control_subsys_t)&config->clock_cfg);
	if (err < 0) {
		return err;
	}

	err = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (err < 0) {
		return err;
	}

	/* Init USART */
	usartInit.baudrate = uart_cfg->baudrate;
	usartInit.parity = uart_silabs_usart_cfg2ll_parity(uart_cfg->parity);
	usartInit.stopbits = uart_silabs_usart_cfg2ll_stopbits(uart_cfg->stop_bits);
	usartInit.databits = uart_silabs_usart_cfg2ll_databits(uart_cfg->data_bits,
								 uart_cfg->parity);
	usartInit.hwFlowControl = uart_silabs_usart_cfg2ll_hwctrl(uart_cfg->flow_ctrl);

	USART_InitAsync(config->base, &usartInit);

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	config->irq_config_func(dev);
	USART_Enable(config->base, usartEnable);
#endif

#ifdef CONFIG_UART_ASYNC_API
	config->irq_config_func(dev);
	return uart_silabs_async_init(dev);
#else
	return 0;
#endif
}

#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
static int uart_silabs_configure(const struct device *dev,
				const struct uart_config *cfg)
{
	const struct uart_silabs_config *config = dev->config;
	struct uart_silabs_data *data = dev->data;
	struct uart_config *uart_cfg = &data->uart_cfg;
	USART_InitAsync_TypeDef usartInit = USART_INITASYNC_DEFAULT;

#ifdef CONFIG_UART_ASYNC_API
	if(data->dma_rx.enabled || data->dma_tx.enabled)
	{
		return -EBUSY;
	}
#endif

	if ((cfg->parity == UART_CFG_PARITY_MARK) ||
	    (cfg->parity == UART_CFG_PARITY_SPACE)) {
		return -ENOTSUP;
	}

	if (cfg->flow_ctrl == UART_CFG_FLOW_CTRL_DTR_DSR || cfg->flow_ctrl == UART_CFG_FLOW_CTRL_RS485) {
		return -ENOTSUP;
	}

	*uart_cfg = *cfg;

	usartInit.baudrate = uart_cfg->baudrate;
	usartInit.parity = uart_silabs_usart_cfg2ll_parity(uart_cfg->parity);
	usartInit.stopbits = uart_silabs_usart_cfg2ll_stopbits(uart_cfg->stop_bits);
	usartInit.databits = uart_silabs_usart_cfg2ll_databits(uart_cfg->data_bits,
								 uart_cfg->parity);
	usartInit.hwFlowControl = uart_silabs_usart_cfg2ll_hwctrl(uart_cfg->flow_ctrl);

	USART_Enable(config->base, usartDisable);

	USART_InitAsync(config->base, &usartInit);

	USART_Enable(config->base, usartEnable);

	return 0;
};

static int uart_silabs_config_get(const struct device *dev,
				 struct uart_config *cfg)
{
	struct uart_silabs_data *data = dev->data;

	*cfg = data->uart_cfg;

	return 0;
}
#endif /* CONFIG_UART_USE_RUNTIME_CONFIGURE */

#ifdef CONFIG_PM_DEVICE
static int uart_silabs_pm_action(const struct device *dev, enum pm_device_action action)
{
	__maybe_unused const struct uart_silabs_config *config = dev->config;

	switch (action) {
	case PM_DEVICE_ACTION_SUSPEND:
#ifdef USART_STATUS_TXIDLE
		/* Wait for TX FIFO to flush before suspending */
		while (!(USART_StatusGet(config->base) & USART_STATUS_TXIDLE)) {
		}
#endif
		break;

	case PM_DEVICE_ACTION_RESUME:
		break;

	default:
		return -ENOTSUP;
	}

	return 0;
}
#endif

static DEVICE_API(uart, uart_silabs_driver_api) = {
	.poll_in = uart_silabs_poll_in,
	.poll_out = uart_silabs_poll_out,
	.err_check = uart_silabs_err_check,
#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
	.configure = uart_silabs_configure,
	.config_get = uart_silabs_config_get,
#endif
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill = uart_silabs_fifo_fill,
	.fifo_read = uart_silabs_fifo_read,
	.irq_tx_enable = uart_silabs_irq_tx_enable,
	.irq_tx_disable = uart_silabs_irq_tx_disable,
	.irq_tx_complete = uart_silabs_irq_tx_complete,
	.irq_tx_ready = uart_silabs_irq_tx_ready,
	.irq_rx_enable = uart_silabs_irq_rx_enable,
	.irq_rx_disable = uart_silabs_irq_rx_disable,
	.irq_rx_ready = uart_silabs_irq_rx_ready,
	.irq_err_enable = uart_silabs_irq_err_enable,
	.irq_err_disable = uart_silabs_irq_err_disable,
	.irq_is_pending = uart_silabs_irq_is_pending,
	.irq_update = uart_silabs_irq_update,
	.irq_callback_set = uart_silabs_irq_callback_set,
#endif
#ifdef CONFIG_UART_ASYNC_API
	.callback_set = uart_silabs_async_callback_set,
	.tx = uart_silabs_async_tx,
	.tx_abort = uart_silabs_async_tx_abort,
	.rx_enable = uart_silabs_async_rx_enable,
	.rx_disable = uart_silabs_async_rx_disable,
	.rx_buf_rsp = uart_silabs_async_rx_buf_rsp,
#endif
};

#ifdef CONFIG_UART_ASYNC_API

#define UART_DMA_CHANNEL_INIT(index, dir)                                                          \
	.dma_##dir = {                                                                             \
		.dma_dev = DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(index, dir)),                   \
		.dma_channel = DT_INST_DMAS_CELL_BY_NAME(index, dir, channel),                     \
		.dma_cfg =                                                                         \
			{                                                                          \
				.dma_slot = DT_INST_DMAS_CELL_BY_NAME(index, dir, slot),           \
				.channel_direction =                                               \
					DT_INST_DMAS_CELL_BY_NAME(index, dir, channel_config),     \
				.source_data_size = 1,                                             \
				.dest_data_size = 1,                                               \
				.source_burst_length = 1,                                          \
				.dest_burst_length = 1,                                            \
				.dma_callback = uart_silabs_dma_##dir##_cb,                        \
			},                                                                         \
		.fifo_threshold = 0,                                                               \
	}
#else

#define UART_DMA_CHANNEL_INIT(index, dir) 0

#endif

#if defined(CONFIG_UART_INTERRUPT_DRIVEN) || defined(CONFIG_UART_ASYNC_API)
#define SILABS_USART_IRQ_HANDLER_DECL(idx)                                                         \
	static void usart_silabs_config_func_##idx(const struct device *dev)
#define SILABS_USART_IRQ_HANDLER_FUNC(idx) .irq_config_func = usart_silabs_config_func_##idx,
#define SILABS_USART_IRQ_HANDLER(idx)                                                              \
	static void usart_silabs_config_func_##idx(const struct device *dev)                       \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(idx, rx, irq),                                     \
			    DT_INST_IRQ_BY_NAME(idx, rx, priority), uart_silabs_isr,               \
			    DEVICE_DT_INST_GET(idx), 0);                                           \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(idx, tx, irq),                                     \
			    DT_INST_IRQ_BY_NAME(idx, tx, priority), uart_silabs_isr,               \
			    DEVICE_DT_INST_GET(idx), 0);                                           \
                                                                                                   \
		irq_enable(DT_INST_IRQ_BY_NAME(idx, rx, irq));                                     \
		irq_enable(DT_INST_IRQ_BY_NAME(idx, tx, irq));                                     \
	}
#else
#define SILABS_USART_IRQ_HANDLER_DECL(idx)
#define SILABS_USART_IRQ_HANDLER_FUNC(idx)
#define SILABS_USART_IRQ_HANDLER(idx)
#endif

#define GET_SILABS_USART_CLOCK(idx)                                                                \
	.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(idx)),                                      \
	.clock_cfg = SILABS_DT_INST_CLOCK_CFG(idx),

#define SILABS_USART_INIT(idx)                                                                     \
	PINCTRL_DT_INST_DEFINE(idx);                                                               \
	SILABS_USART_IRQ_HANDLER_DECL(idx);                                                        \
	PM_DEVICE_DT_INST_DEFINE(idx, uart_silabs_pm_action);                                      \
                                                                                                   \
	static const struct uart_silabs_config usart_silabs_cfg_##idx = {                          \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(idx),                                       \
		.base = (USART_TypeDef *)DT_INST_REG_ADDR(idx),                                    \
		GET_SILABS_USART_CLOCK(idx) SILABS_USART_IRQ_HANDLER_FUNC(idx)};                   \
                                                                                                   \
	static struct uart_silabs_data usart_silabs_data_##idx = {                                 \
		.uart_cfg =                                                                        \
			{                                                                          \
				.baudrate = DT_INST_PROP(idx, current_speed),                      \
				.parity = DT_INST_ENUM_IDX(idx, parity),                           \
				.stop_bits = DT_INST_ENUM_IDX(idx, stop_bits),                     \
				.data_bits = DT_INST_ENUM_IDX(idx, data_bits),                     \
				.flow_ctrl = DT_INST_PROP(idx, hw_flow_control)                    \
						     ? UART_CFG_FLOW_CTRL_RTS_CTS                  \
						     : UART_CFG_FLOW_CTRL_NONE,                    \
			},                                                                         \
		UART_DMA_CHANNEL_INIT(idx, rx),                                                    \
		UART_DMA_CHANNEL_INIT(idx, tx)};                                                   \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(idx, uart_silabs_init, PM_DEVICE_DT_INST_GET(idx),                   \
			      &usart_silabs_data_##idx, &usart_silabs_cfg_##idx, PRE_KERNEL_1,     \
			      CONFIG_SERIAL_INIT_PRIORITY, &uart_silabs_driver_api);               \
                                                                                                   \
	SILABS_USART_IRQ_HANDLER(idx)

DT_INST_FOREACH_STATUS_OKAY(SILABS_USART_INIT)
