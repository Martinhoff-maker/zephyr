/*
 * Copyright (c) 2024 Silicon Laboratories Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/clock_control_silabs.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/irq.h>
#include <zephyr/pm/device.h>
#include <em_usart.h>

#ifdef CONFIG_UART_ASYNC_API
#include <zephyr/drivers/dma.h>
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
	bool src_addr_increment;
	bool dst_addr_increment;
	int fifo_threshold;
	struct dma_block_config blk_cfg;
	uint8_t *buffer;
	size_t buffer_length;
	size_t offset;
	volatile size_t counter;
	int32_t timeout;
	struct k_work_delayable timeout_work;
	bool enabled;
};
#endif
struct uart_silabs_config {
	const struct pinctrl_dev_config *pcfg;
	const struct device *clock_dev;
	const struct silabs_clock_control_cmu_config clock_cfg;
	USART_TypeDef *base;
	uint32_t baud_rate;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	void (*irq_config_func)(const struct device *dev);
#endif
};

struct uart_silabs_data {
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

static void uart_silabs_isr(const struct device *dev)
{
	struct uart_silabs_data *data = dev->data;
#ifdef  CONFIG_UART_INTERRUPT_DRIVEN
	if (data->callback) {
		data->callback(dev, data->cb_data);
	}
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
#ifdef CONFIG_UART_ASYNC_API

	//IT TIMCMP1 : Rx start receiving but no more receive til 255 baud time
	if (data.uart_dev->IF & USART_IF_TCMP1) {
		LOG_DBG("TCMP1 interrupt occurred");
        	//uart_handle_rx_dma_timeout(uart_ctxt);

		if (dma_get_status(data->dma_rx.dma_dev, data->dma_rx.dma_channel, &stat) == 0) {
			size_t rx_rcv_len = data->dma_rx.buffer_length - stat.pending_length;
			if (rx_rcv_len > data->dma_rx.offset) {
				data->dma_rx.counter = rx_rcv_len;
				async_evt_rx_rdy(data);
			}
		}

		// Pourquoi set ces registre de nouveau ?
		data.uart_dev->TIMECMP1 &= ~_USART_TIMECMP1_TSTART_MASK;
        	data.uart_dev->TIMECMP1 |= USART_TIMECMP1_TSTART_RXEOF;

        	USART_IntClear(data.uart_dev, USART_IF_TCMP1);

        	return;
    	}
	// Rx overflow
    	if (data.uart_dev->IF & USART_IF_RXOF) {
		LOG_DBG("Rx overflow interrupt occurred");
        	//uart_handle_rx_overflow(uart_ctxt);
        	USART_IntClear(data.uart_dev, USART_IF_RXOF);
        	return;
    	}

	// Tx Complete
	if (data.uart_dev->IF & USART_IF_TXC) {
		LOG_DBG("Tc Complete interrupt occurred");

		// disable TCMP2 because TX is either abort (by user or timeout) or complete
		data.uart_dev->TIMECMP2 &= ~_USART_TIMECMP2_TSTART_MASK;
        	data.uart_dev->TIMECMP2 |= USART_TIMECMP2_TSTART_DISABLE;

		async_evt_tx_done(data);
		
		USART_IntClear(data.uart_dev, data.uart_dev)
	}
	if (data.uart_dev->IF & USART_IF_TCMP2) {
		LOG_DBG("TCMP2 interrupt occurred");

		// Si transfert completé
		if (dma_get_status(data->dma_rx.dma_dev, data->dma_rx.dma_channel, &stat) == 0) {
			size_t rx_rcv_len = data->dma_rx.buffer_length - stat.pending_length;
			if (rx_rcv_len > data->dma_rx.offset) {
				data->dma_rx.counter = rx_rcv_len;
				async_evt_rx_rdy(data);
			}
		} else {

		}

		// disable TCMP2 because TX is either abort (by user or timeout) or complete
		data.uart_dev->TIMECMP2 &= ~_USART_TIMECMP2_TSTART_MASK;
        	data.uart_dev->TIMECMP2 |= USART_TIMECMP2_TSTART_DISABLE;

		uart_silabs_async_tx_abort()

        	USART_IntClear(data.uart_dev, USART_IF_TCMP2);

        	return;
    	}
	
	//data flush RXNE ?

	/* Clear errors */
	uart_silabs_err_check(dev);
#endif /* CONFIG_UART_ASYNC_API */

}

#ifdef CONFIG_UART_ASYNC_API

static inline void async_user_callback(struct uart_silabs_data *data,
				struct uart_event *event)
{
	if (data->async_cb) {
		data->async_cb(data->uart_dev, event, data->async_user_data);
	}
}

static inline void async_evt_tx_done(struct uart_silabs_data *data)
{
	LOG_DBG("tx done: %d", data->dma_tx.counter);

	struct uart_event event = {
		.type = UART_TX_DONE,
		.data.tx.buf = data->dma_tx.buffer,
		.data.tx.len = data->dma_tx.counter
	};

	/* Reset tx buffer */
	data->dma_tx.buffer_length = 0;
	data->dma_tx.counter = 0;

	async_user_callback(data, &event);
}

static inline void async_evt_tx_abort(struct uart_silabs_data *data)
{
	LOG_DBG("tx abort: %d", data->dma_tx.counter);

	struct uart_event event = {
		.type = UART_TX_ABORTED,
		.data.tx.buf = data->dma_tx.buffer,
		.data.tx.len = data->dma_tx.counter
	};

	/* Reset tx buffer */
	data->dma_tx.buffer_length = 0;
	data->dma_tx.counter = 0;

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

static int uart_silabs_async_callback_set(const struct device *dev,
					 uart_callback_t callback,
					 void *user_data)
{
	struct uart_silabs_data *data = dev->data;

	data->async_cb = callback;
	data->async_user_data = user_data;

	return 0;
}

static int uart_silabs_async_rx_disable(const struct device *dev)
{
	const struct uart_silabs_config *config = dev->config;
	USART_TypeDef *usart = config->usart;
	struct uart_silabs_data *data = dev->data;
	struct uart_event disabled_event = {
		.type = UART_RX_DISABLED
	};

	if (!data->dma_rx.enabled) {
		async_user_callback(data, &disabled_event);
		return -EFAULT;
	}

	// Disable Rx related IT
	USART_IntClear(config->base, USART_IF_RXOF | USART_IF_TCMP1);
    	USART_IntDisable(config->base, USART_IF_RXOF);
    	USART_IntDisable(config->base, USART_IF_TCMP1);

	// Flush data that is already in the current buffer
	uart_silabs_dma_rx_flush(dev);

	// Release buffer
	async_evt_rx_buf_release(data);

	//Need to disable USART rx but how to only disable for dma and not for interrupt driven 
	USART_Enable()

	dma_stop(data->dma_rx.dma_dev, data->dma_rx.dma_channel);

	// Release next buffer is already provided
	if (data->rx_next_buffer) {
		struct uart_event rx_next_buf_release_evt = {
			.type = UART_RX_BUF_RELEASED,
			.data.rx_buf.buf = data->rx_next_buffer,
		};
		async_user_callback(data, &rx_next_buf_release_evt);
	}

	data->rx_next_buffer = NULL;
	data->rx_next_buffer_len = 0;

	/* When async rx is disabled, enable interruptible instance of uart to function normally */
	//LL_USART_EnableIT_RXNE(usart);

	LOG_DBG("rx: disabled");

	async_user_callback(data, &disabled_event);

	return 0;
}

void uart_silabs_dma_tx_cb(const struct device *dma_dev, void *user_data,
			       uint32_t channel, int status)
{
	const struct device *uart_dev = user_data;
	struct uart_stm32_data *data = uart_dev->data;
	struct dma_status stat;
	unsigned int key = irq_lock();

	/* Disable TX */
	uart_silabs_dma_tx_disable(uart_dev);

	(void)k_work_cancel_delayable(&data->dma_tx.timeout_work);

	if (!dma_get_status(data->dma_tx.dma_dev,
				data->dma_tx.dma_channel, &stat)) {
		data->dma_tx.counter = data->dma_tx.buffer_length -
					stat.pending_length;
	}

	data->dma_tx.buffer_length = 0;

	irq_unlock(key);
}

static void uart_silabs_dma_replace_buffer(const struct device *dev)
{
	const struct uart_stm32_config *config = dev->config;
	USART_TypeDef *usart = config->usart;
	struct uart_stm32_data *data = dev->data;

	/* Replace the buffer and reload the DMA */
	LOG_DBG("Replacing RX buffer: %d", data->rx_next_buffer_len);

	/* reload DMA */
	data->dma_rx.offset = 0;
	data->dma_rx.counter = 0;
	data->dma_rx.buffer = data->rx_next_buffer;
	data->dma_rx.buffer_length = data->rx_next_buffer_len;
	data->dma_rx.blk_cfg.block_size = data->dma_rx.buffer_length;
	data->dma_rx.blk_cfg.dest_address = (uint32_t)data->dma_rx.buffer;
	data->rx_next_buffer = NULL;
	data->rx_next_buffer_len = 0;

	dma_reload(data->dma_rx.dma_dev, data->dma_rx.dma_channel,
			data->dma_rx.blk_cfg.source_address,
			data->dma_rx.blk_cfg.dest_address,
			data->dma_rx.blk_cfg.block_size);

	dma_start(data->dma_rx.dma_dev, data->dma_rx.dma_channel);

	LL_USART_ClearFlag_IDLE(usart);

	/* Request next buffer */
	async_evt_rx_buf_request(data);
}

void uart_silabs_dma_rx_cb(const struct device *dma_dev, void *user_data,
			       uint32_t channel, int status)
{
	const struct device *uart_dev = user_data;
	struct uart_silabs_data *data = uart_dev->data;

	if (status < 0) {
		async_evt_rx_err(data, status);
		return;
	}

	// need to stop TCMP1

	/* true since this functions occurs when buffer if full */
	data->dma_rx.counter = data->dma_rx.buffer_length;

	async_evt_rx_rdy(data);

	if (data->rx_next_buffer != NULL) {
		async_evt_rx_buf_release(data);

		/* replace the buffer when the current
		 * is full and not the same as the next
		 * one.
		 */
		uart_stm32_dma_replace_buffer(uart_dev);
	} else {
		/* Buffer full without valid next buffer,
		 * an UART_RX_DISABLED event must be generated,
		 * but uart_stm32_async_rx_disable() cannot be
		 * called in ISR context. So force the RX timeout
		 * to minimum value and let the RX timeout to do the job.
		 */
		k_work_reschedule(&data->dma_rx.timeout_work, K_TICKS(1));
	}
}

static int uart_silabs_async_tx(const struct device *dev,
		const uint8_t *tx_data, size_t buf_size, int32_t timeout)
{
	const struct uart_silabs_config *config = dev->config;
	USART_TypeDef *usart = config->usart;
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

	LOG_DBG("tx: l=%d", data->dma_tx.buffer_length);

	config->base->CMD = USART_CMD_CLEARTX;

	USART_TIMECMP2_TSTART_TXEOF | \

	USART_IntClear(config->base, USART_IF_TXC | USART_IF_TCMP2);
	USART_IntEnable(config->base, USART_IF_TXC);
	USART_IntEnable(config->base, USART_IF_TCMP2);
	
	ret = dma_config(data->dma_tx.dma_dev, data->dma_tx.dma_channel,
				&data->dma_tx.dma_cfg);
	if (ret) {
		LOG_ERR("dma tx config error!");
		return ret;
	}

	ret = dma_start(data->dma_tx.dma_dev, data->dma_tx.dma_channel);
	if (ret) {
		LOG_ERR("UART err: TX DMA start failed!");
		return ret;
	}

	USART_Enable(config->base, usartEnableTx);
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

	// TODO: fonctionnalité à rajouter dans le driver dma (LDMA_TransferRemainingCount)
	if (!dma_get_status(data->dma_tx.dma_dev,
				data->dma_tx.dma_channel, &stat)) {
		data->dma_tx.counter = tx_buffer_length - stat.pending_length;
	}

	// TODO : mettre à jour la quantité de data déjà envoyé 

	dma_stop(data->dma_tx.dma_dev, data->dma_tx.dma_channel);
	async_evt_tx_abort(data);

	return 0;
}

static int uart_silabs_async_rx_enable(const struct device *dev,
		uint8_t *rx_buf, size_t buf_size, int32_t timeout)
{
	const struct uart_silabs_config *config = dev->config;
	USART_TypeDef *usart = config->usart;
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

	//Gérer le timeout de l'API avec avec le baudrate et le tcmp1

	/* Disable RX interrupts to let DMA to handle it */
	// IT_RXDATAV ???
	//LL_USART_DisableIT_RXNE(usart);

	data->dma_rx.blk_cfg.block_size = buf_size;
	data->dma_rx.blk_cfg.dest_address = (uint32_t)data->dma_rx.buffer;

	ret = dma_config(data->dma_rx.dma_dev, data->dma_rx.dma_channel,
				&data->dma_rx.dma_cfg);

	if (ret != 0) {
		LOG_ERR("UART ERR: RX DMA config failed!");
		return -EINVAL;
	}

	if (dma_start(data->dma_rx.dma_dev, data->dma_rx.dma_channel)) {
		LOG_ERR("UART ERR: RX DMA start failed!");
		return -EFAULT;
	}

	// Clear Rx buffr
	config->base->CMD = USART_CMD_CLEARRX

	//Clear IT
	USART_IntClear(config->base, USART_IF_RXOF | USART_IF_TCMP1);
    	//USART_IntEnable(uart_ctxt->hw_regs, USART_IF_RXDATAV);
    	USART_IntEnable(config->base, USART_IF_RXOF);
    	USART_IntEnable(config->base, USART_IF_TCMP1);
    	USART_Enable(config->base, usartEnableRx);

	data->dma_rx.enabled = true;

	// Enable more Error IT ?

	/* Request next buffer */
	async_evt_rx_buf_request(data);

	LOG_DBG("async rx enabled");

	return ret;
}

static int uart_silabs_async_rx_buf_rsp(const struct device *dev, uint8_t *buf,
				       size_t len)
{
	struct uart_silabs_data *data = dev->data;
	unsigned int key;
	int err = 0;

	LOG_DBG("replace buffer (%d)", len);

	key = irq_lock();

	if (data->rx_next_buffer != NULL) {
		err = -EBUSY;
	} else if (!data->dma_rx.enabled) {
		err = -EACCES;
	} else {
		data->rx_next_buffer = buf;
		data->rx_next_buffer_len = len;
	}

	irq_unlock(key);

	return err;
}

static int uart_silabs_async_init(const struct device *dev)
{
	const struct uart_silabs_config *config = dev->config;
	USART_TypeDef *usart = config->usart;
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
	dma_stop(&data->dma_rx.dma_dev);
	dma_stop(&data->dma_tx.dma_dev);

	memset(&data->dma_rx.blk_cfg, 0, sizeof(data->dma_rx.blk_cfg));
	data->dma_rx.blk_cfg.source_address = &(usart->RXDATA);
	data->dma_rx.blk_cfg.dest_address = 0; /* dest not ready */
	data->dma_rx.blk_cfg.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	data->dma_rx.blk_cfg.dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	data->dma_rx.blk_cfg.source_reload_en  = 0;
	data->dma_rx.blk_cfg.dest_reload_en = 0;
	data->dma_rx.blk_cfg.fifo_mode_control = data->dma_rx.fifo_threshold;
	// data->dma_rx.dma_cfg.complete_callback_en = 1; ????
	data->dma_rx.dma_cfg.head_block = &data->dma_rx.blk_cfg;
	data->dma_rx.dma_cfg.user_data = (void *)dev;
	data->rx_next_buffer = NULL;
	data->rx_next_buffer_len = 0;

	memset(&data->dma_tx.blk_cfg, 0, sizeof(data->dma_tx.blk_cfg));
	data->dma_tx.blk_cfg.dest_address = &(usart->TXDATA)
	data->dma_tx.blk_cfg.source_address = 0; /* not ready */
	data->dma_tx.blk_cfg.source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	data->dma_tx.blk_cfg.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	data->dma_tx.blk_cfg.fifo_mode_control = data->dma_tx.fifo_threshold;
	// data->dma_tx.dma_cfg.complete_callback_en = 1; ????
	data->dma_tx.dma_cfg.head_block = &data->dma_tx.blk_cfg;
	data->dma_tx.dma_cfg.user_data = (void *)dev;

	config->base->CMD = USART_CMD_CLEARRX | USART_CMD_CLEARTX; //really needed ? already cleared in USART_AsyncInit
    	config->base->TIMECMP1 = USART_TIMECMP1_TSTOP_RXACT | \
                                  USART_TIMECMP1_TSTART_RXEOF | \
                                  USART_TIMECMP1_RESTARTEN | \
                                  (0xff << _USART_TIMECMP1_TCMPVAL_SHIFT);
	config->base->TIMECMP2 = USART_TIMECMP2_TSTOP_TXST | \
                                  USART_TIMECMP1_TSTART_TXEOF | \
                                  USART_TIMECMP2_RESTARTEN | \
                                  (0xff << _USART_TIMECMP2_TCMPVAL_SHIFT);

	return 0;
}

#endif /* CONFIG_UART_ASYNC_API */


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
	usartInit.enable = usartDisable;
	usartInit.baudrate = config->baud_rate;
	USART_InitAsync(config->base, &usartInit);

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	config->irq_config_func(dev);
#endif

#ifdef CONFIG_UART_ASYNC_API
	return uart_silabs_async_init(dev);
#else
	USART_Enable(config->base, usartEnable);
	return 0;
#endif
}

// rajouter un if def sur pm device ? ou un is_define(?) dans la def du driver. 
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
	.dma_dev = DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(index, dir)),                           \
	.dma_channel = DT_INST_DMAS_CELL_BY_NAME(index, dir, channel),                             \
	.dma_cfg =                                                                                 \
		{                                                                                  \
			.dma_slot = DT_INST_DMAS_CELL_BY_NAME(index, dir, slot),                   \
			.channel_direction =                                                       \
				DT_INST_DMAS_CELL_BY_NAME(index, dir, channel_config),             \
			.channel_priority = 0 .source_data_size = 1,                               \
			.dest_data_size = 1 .source_burst_length = 1, /* SINGLE transfer */        \
			.dest_burst_length = 1,                                                    \
			.block_count = 1,                                                          \
			.dma_callback = uart_silabs_dma_##dir##_cb,                                \
	},                                                                                         \
	.src_addr_increment = 0, .dst_addr_increment = 0, .fifo_threshold = 0,

#endif

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
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
		GET_SILABS_USART_CLOCK(idx).baud_rate = DT_INST_PROP(idx, current_speed),          \
		SILABS_USART_IRQ_HANDLER_FUNC(idx)};                                               \
                                                                                                   \
	static struct uart_silabs_data usart_silabs_data_##idx = {                                 \
		UART_DMA_CHANNEL(idx, rx), UART_DMA_CHANNEL(idx, tx)};                             \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(idx, uart_silabs_init, PM_DEVICE_DT_INST_GET(idx),                   \
			      &usart_silabs_data_##idx, &usart_silabs_cfg_##idx, PRE_KERNEL_1,     \
			      CONFIG_SERIAL_INIT_PRIORITY, &uart_silabs_driver_api);               \
                                                                                                   \
	SILABS_USART_IRQ_HANDLER(idx)

DT_INST_FOREACH_STATUS_OKAY(SILABS_USART_INIT)
