/*
 * Copyright (C) 2005-2007 Jan Kiszka <jan.kiszka@web.de>.
 *
 * Xenomai is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Xenomai is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Xenomai; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#include <linux/version.h>
#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <asm/io.h>

#include <rtdm/serial.h>
#include <rtdm/driver.h>

MODULE_DESCRIPTION("RTDM-based driver for 16550A UARTs");
MODULE_AUTHOR("Jan Kiszka <jan.kiszka@web.de>");
MODULE_VERSION("1.5.2");
MODULE_LICENSE("GPL");

#define RT_16550_DRIVER_NAME	"xeno_16550A"

#define MAX_DEVICES		8

#define IN_BUFFER_SIZE		4096
#define OUT_BUFFER_SIZE		4096

#define DEFAULT_BAUD_BASE	115200
#define DEFAULT_TX_FIFO		16

#define PARITY_MASK		0x03
#define DATA_BITS_MASK		0x03
#define STOP_BITS_MASK		0x01
#define FIFO_MASK		0xC0
#define EVENT_MASK		0x0F

#define LCR_DLAB		0x80

#define FCR_FIFO		0x01
#define FCR_RESET_RX		0x02
#define FCR_RESET_TX		0x04

#define IER_RX			0x01
#define IER_TX			0x02
#define IER_STAT		0x04
#define IER_MODEM		0x08

#define IIR_MODEM		0x00
#define IIR_PIRQ		0x01
#define IIR_TX			0x02
#define IIR_RX			0x04
#define IIR_STAT		0x06
#define IIR_MASK		0x07

#define RHR			0	/* Receive Holding Buffer */
#define THR			0	/* Transmit Holding Buffer */
#define DLL			0	/* Divisor Latch LSB */
#define IER			1	/* Interrupt Enable Register */
#define DLM			1	/* Divisor Latch MSB */
#define IIR			2	/* Interrupt Id Register */
#define FCR			2	/* Fifo Control Register */
#define LCR			3	/* Line Control Register */
#define MCR			4	/* Modem Control Register */
#define LSR			5	/* Line Status Register */
#define MSR			6	/* Modem Status Register */

struct rt_16550_context {
	struct rtser_config config;	/* current device configuration */

	rtdm_irq_t irq_handle;		/* device IRQ handle */
	rtdm_lock_t lock;		/* lock to protect context struct */

	unsigned long base_addr;	/* hardware IO base address */
#ifdef CONFIG_XENO_DRIVERS_16550A_ANY
	int io_mode;			/* hardware IO-access mode */
#endif
	int tx_fifo;			/* cached global tx_fifo[<device>] */

	int in_head;			/* RX ring buffer, head pointer */
	int in_tail;			/* RX ring buffer, tail pointer */
	size_t in_npend;		/* pending bytes in RX ring */
	int in_nwait;			/* bytes the user waits for */
	rtdm_event_t in_event;		/* raised to unblock reader */
	char in_buf[IN_BUFFER_SIZE];	/* RX ring buffer */
	volatile unsigned long in_lock;	/* single-reader lock */
	uint64_t *in_history;		/* RX timestamp buffer */

	int out_head;			/* TX ring buffer, head pointer */
	int out_tail;			/* TX ring buffer, tail pointer */
	size_t out_npend;		/* pending bytes in TX ring */
	rtdm_event_t out_event;		/* raised to unblock writer */
	char out_buf[OUT_BUFFER_SIZE];	/* TX ring buffer */
	rtdm_mutex_t out_lock;		/* single-writer mutex */

	uint64_t last_timestamp;	/* timestamp of last event */
	int ioc_events;			/* recorded events */
	rtdm_event_t ioc_event;		/* raised to unblock event waiter */
	volatile unsigned long ioc_event_lock;	/* single-waiter lock */

	int ier_status;			/* IER cache */
	int mcr_status;			/* MCR cache */
	int status;			/* cache for LSR + soft-states */
	int saved_errors;		/* error cache for RTIOC_GET_STATUS */
};

static const struct rtser_config default_config = {
	0xFFFF, RTSER_DEF_BAUD, RTSER_DEF_PARITY, RTSER_DEF_BITS,
	RTSER_DEF_STOPB, RTSER_DEF_HAND, RTSER_DEF_FIFO_DEPTH, 0,
	RTSER_DEF_TIMEOUT, RTSER_DEF_TIMEOUT, RTSER_DEF_TIMEOUT,
	RTSER_DEF_TIMESTAMP_HISTORY, RTSER_DEF_EVENT_MASK, RTSER_DEF_RS485
};

static struct rtdm_device *device[MAX_DEVICES];

static unsigned int irq[MAX_DEVICES];
static unsigned long irqtype[MAX_DEVICES] = {
	[0 ... MAX_DEVICES-1] = RTDM_IRQTYPE_SHARED | RTDM_IRQTYPE_EDGE
};
static unsigned int baud_base[MAX_DEVICES];
static int tx_fifo[MAX_DEVICES];

module_param_array(irq, uint, NULL, 0400);
module_param_array(baud_base, uint, NULL, 0400);
module_param_array(tx_fifo, int, NULL, 0400);

MODULE_PARM_DESC(irq, "IRQ numbers of the serial devices");
MODULE_PARM_DESC(baud_base, "Maximum baud rate of the serial device "
		 "(internal clock rate / 16)");
MODULE_PARM_DESC(tx_fifo, "Transmitter FIFO size");

#include "16550A_io.h"
#include "16550A_pnp.h"
#include "16550A_pci.h"

static inline int rt_16550_rx_interrupt(struct rt_16550_context *ctx,
					uint64_t * timestamp)
{
	unsigned long base = ctx->base_addr;
	int mode = rt_16550_io_mode_from_ctx(ctx);
	int rbytes = 0;
	int lsr = 0;
	int c;

	do {
		c = rt_16550_reg_in(mode, base, RHR);	/* read input char */

		ctx->in_buf[ctx->in_tail] = c;
		if (ctx->in_history)
			ctx->in_history[ctx->in_tail] = *timestamp;
		ctx->in_tail = (ctx->in_tail + 1) & (IN_BUFFER_SIZE - 1);

		if (++ctx->in_npend > IN_BUFFER_SIZE) {
			lsr |= RTSER_SOFT_OVERRUN_ERR;
			ctx->in_npend--;
		}

		rbytes++;
		lsr &= ~RTSER_LSR_DATA;
		lsr |= (rt_16550_reg_in(mode, base, LSR) &
			(RTSER_LSR_DATA | RTSER_LSR_OVERRUN_ERR |
			 RTSER_LSR_PARITY_ERR | RTSER_LSR_FRAMING_ERR |
			 RTSER_LSR_BREAK_IND));
	} while (lsr & RTSER_LSR_DATA);

	/* save new errors */
	ctx->status |= lsr;

	/* If we are enforcing the RTSCTS control flow and the input
	   buffer is busy above the specified high watermark, clear
	   RTS. */
/*	if (uart->i_count >= uart->config.rts_hiwm &&
	    (uart->config.handshake & RT_UART_RTSCTS) != 0 &&
	    (uart->modem & MCR_RTS) != 0) {
		uart->modem &= ~MCR_RTS;
		rt_16550_reg_out(mode, base, MCR, uart->modem);
	}*/

	return rbytes;
}

static inline void rt_16550_tx_interrupt(struct rt_16550_context *ctx)
{
	int c;
	int count;
	unsigned long base = ctx->base_addr;
	int mode = rt_16550_io_mode_from_ctx(ctx);

/*	if (uart->modem & MSR_CTS)*/
	{
		for (count = ctx->tx_fifo;
		     (count > 0) && (ctx->out_npend > 0);
		     count--, ctx->out_npend--) {
			c = ctx->out_buf[ctx->out_head++];
			rt_16550_reg_out(mode, base, THR, c);
			ctx->out_head &= (OUT_BUFFER_SIZE - 1);
		}
	}
}

static inline void rt_16550_stat_interrupt(struct rt_16550_context *ctx)
{
	unsigned long base = ctx->base_addr;
	int mode = rt_16550_io_mode_from_ctx(ctx);

	ctx->status |= (rt_16550_reg_in(mode, base, LSR) &
			(RTSER_LSR_OVERRUN_ERR | RTSER_LSR_PARITY_ERR |
			 RTSER_LSR_FRAMING_ERR | RTSER_LSR_BREAK_IND));
}

static int rt_16550_interrupt(rtdm_irq_t * irq_context)
{
	struct rt_16550_context *ctx;
	unsigned long base;
	int mode;
	int iir;
	uint64_t timestamp = rtdm_clock_read();
	int rbytes = 0;
	int events = 0;
	int modem;
	int ret = RTDM_IRQ_NONE;

	ctx = rtdm_irq_get_arg(irq_context, struct rt_16550_context);
	base = ctx->base_addr;
	mode = rt_16550_io_mode_from_ctx(ctx);

	rtdm_lock_get(&ctx->lock);

	while (1) {
		iir = rt_16550_reg_in(mode, base, IIR) & IIR_MASK;
		if (iir & IIR_PIRQ)
			break;

		if (iir == IIR_RX) {
			rbytes += rt_16550_rx_interrupt(ctx, &timestamp);
			events |= RTSER_EVENT_RXPEND;
		} else if (iir == IIR_STAT)
			rt_16550_stat_interrupt(ctx);
		else if (iir == IIR_TX)
			rt_16550_tx_interrupt(ctx);
		else if (iir == IIR_MODEM) {
			modem = rt_16550_reg_in(mode, base, MSR);
			if (modem & (modem << 4))
				events |= RTSER_EVENT_MODEMHI;
			if ((modem ^ 0xF0) & (modem << 4))
				events |= RTSER_EVENT_MODEMLO;
		}

		ret = RTDM_IRQ_HANDLED;
	}

	if (ctx->in_nwait > 0) {
		if ((ctx->in_nwait <= rbytes) || ctx->status) {
			ctx->in_nwait = 0;
			rtdm_event_signal(&ctx->in_event);
		} else
			ctx->in_nwait -= rbytes;
	}

	if (ctx->status) {
		events |= RTSER_EVENT_ERRPEND;
		ctx->ier_status &= ~IER_STAT;
	}

	if (events & ctx->config.event_mask) {
		int old_events = ctx->ioc_events;

		ctx->last_timestamp = timestamp;
		ctx->ioc_events = events;

		if (!old_events)
			rtdm_event_signal(&ctx->ioc_event);
	}

	if ((ctx->ier_status & IER_TX) && (ctx->out_npend == 0)) {
		/* mask transmitter empty interrupt */
		ctx->ier_status &= ~IER_TX;

		rtdm_event_signal(&ctx->out_event);
	}

	/* update interrupt mask */
	rt_16550_reg_out(mode, base, IER, ctx->ier_status);

	rtdm_lock_put(&ctx->lock);

	return ret;
}

static int rt_16550_set_config(struct rt_16550_context *ctx,
			       const struct rtser_config *config,
			       uint64_t **in_history_ptr)
{
	rtdm_lockctx_t lock_ctx;
	unsigned long base = ctx->base_addr;
	int mode = rt_16550_io_mode_from_ctx(ctx);
	int err = 0;

	/* make line configuration atomic and IRQ-safe */
	rtdm_lock_get_irqsave(&ctx->lock, lock_ctx);

	if (config->config_mask & RTSER_SET_BAUD) {
		int dev_id = rtdm_fd_minor(rtdm_private_to_fd(ctx));
		int baud_div;

		ctx->config.baud_rate = config->baud_rate;
		baud_div = (baud_base[dev_id] + (ctx->config.baud_rate>>1)) /
			ctx->config.baud_rate;
		rt_16550_reg_out(mode, base, LCR, LCR_DLAB);
		rt_16550_reg_out(mode, base, DLL, baud_div & 0xff);
		rt_16550_reg_out(mode, base, DLM, baud_div >> 8);
	}

	if (config->config_mask & RTSER_SET_PARITY)
		ctx->config.parity = config->parity & PARITY_MASK;
	if (config->config_mask & RTSER_SET_DATA_BITS)
		ctx->config.data_bits = config->data_bits & DATA_BITS_MASK;
	if (config->config_mask & RTSER_SET_STOP_BITS)
		ctx->config.stop_bits = config->stop_bits & STOP_BITS_MASK;

	if (config->config_mask & (RTSER_SET_PARITY |
				   RTSER_SET_DATA_BITS |
				   RTSER_SET_STOP_BITS |
				   RTSER_SET_BAUD)) {
		rt_16550_reg_out(mode, base, LCR,
				 (ctx->config.parity << 3) |
				 (ctx->config.stop_bits << 2) |
				 ctx->config.data_bits);
		ctx->status = 0;
		ctx->ioc_events &= ~RTSER_EVENT_ERRPEND;
	}

	if (config->config_mask & RTSER_SET_FIFO_DEPTH) {
		ctx->config.fifo_depth = config->fifo_depth & FIFO_MASK;
		rt_16550_reg_out(mode, base, FCR,
				 FCR_FIFO | FCR_RESET_RX | FCR_RESET_TX);
		rt_16550_reg_out(mode, base, FCR,
				 FCR_FIFO | ctx->config.fifo_depth);
	}

	rtdm_lock_put_irqrestore(&ctx->lock, lock_ctx);

	/* Timeout manipulation is not atomic. The user is supposed to take
	   care not to use and change timeouts at the same time. */
	if (config->config_mask & RTSER_SET_TIMEOUT_RX)
		ctx->config.rx_timeout = config->rx_timeout;
	if (config->config_mask & RTSER_SET_TIMEOUT_TX)
		ctx->config.tx_timeout = config->tx_timeout;
	if (config->config_mask & RTSER_SET_TIMEOUT_EVENT)
		ctx->config.event_timeout = config->event_timeout;

	if (config->config_mask & RTSER_SET_TIMESTAMP_HISTORY) {
		/* change timestamp history atomically */
		rtdm_lock_get_irqsave(&ctx->lock, lock_ctx);

		if (config->timestamp_history & RTSER_RX_TIMESTAMP_HISTORY) {
			if (!ctx->in_history) {
				ctx->in_history = *in_history_ptr;
				*in_history_ptr = NULL;
				if (!ctx->in_history)
					err = -ENOMEM;
			}
		} else {
			*in_history_ptr = ctx->in_history;
			ctx->in_history = NULL;
		}

		rtdm_lock_put_irqrestore(&ctx->lock, lock_ctx);
	}

	if (config->config_mask & RTSER_SET_EVENT_MASK) {
		/* change event mask atomically */
		rtdm_lock_get_irqsave(&ctx->lock, lock_ctx);

		ctx->config.event_mask = config->event_mask & EVENT_MASK;
		ctx->ioc_events = 0;

		if ((config->event_mask & RTSER_EVENT_RXPEND) &&
		    (ctx->in_npend > 0))
			ctx->ioc_events |= RTSER_EVENT_RXPEND;

		if ((config->event_mask & RTSER_EVENT_ERRPEND)
		    && ctx->status)
			ctx->ioc_events |= RTSER_EVENT_ERRPEND;

		if (config->event_mask & (RTSER_EVENT_MODEMHI | RTSER_EVENT_MODEMLO))
			/* enable modem status interrupt */
			ctx->ier_status |= IER_MODEM;
		else
			/* disable modem status interrupt */
			ctx->ier_status &= ~IER_MODEM;
		rt_16550_reg_out(mode, base, IER, ctx->ier_status);

		rtdm_lock_put_irqrestore(&ctx->lock, lock_ctx);
	}

	if (config->config_mask & RTSER_SET_HANDSHAKE) {
		/* change handshake atomically */
		rtdm_lock_get_irqsave(&ctx->lock, lock_ctx);

		ctx->config.handshake = config->handshake;

		switch (ctx->config.handshake) {
		case RTSER_RTSCTS_HAND:
			// ...?

		default:	/* RTSER_NO_HAND */
			ctx->mcr_status =
			    RTSER_MCR_DTR | RTSER_MCR_RTS | RTSER_MCR_OUT2;
			break;
		}
		rt_16550_reg_out(mode, base, MCR, ctx->mcr_status);

		rtdm_lock_put_irqrestore(&ctx->lock, lock_ctx);
	}

	return err;
}

void rt_16550_cleanup_ctx(struct rt_16550_context *ctx)
{
	rtdm_event_destroy(&ctx->in_event);
	rtdm_event_destroy(&ctx->out_event);
	rtdm_event_destroy(&ctx->ioc_event);
	rtdm_mutex_destroy(&ctx->out_lock);
}

int rt_16550_open(struct rtdm_fd *fd, int oflags)
{
	struct rt_16550_context *ctx;
	int dev_id = rtdm_fd_minor(fd);
	int err;
	uint64_t *dummy;
	rtdm_lockctx_t lock_ctx;

	ctx = rtdm_fd_to_private(fd);

	/* IPC initialisation - cannot fail with used parameters */
	rtdm_lock_init(&ctx->lock);
	rtdm_event_init(&ctx->in_event, 0);
	rtdm_event_init(&ctx->out_event, 0);
	rtdm_event_init(&ctx->ioc_event, 0);
	rtdm_mutex_init(&ctx->out_lock);

	rt_16550_init_io_ctx(dev_id, ctx);

	ctx->tx_fifo = tx_fifo[dev_id];

	ctx->in_head = 0;
	ctx->in_tail = 0;
	ctx->in_npend = 0;
	ctx->in_nwait = 0;
	ctx->in_lock = 0;
	ctx->in_history = NULL;

	ctx->out_head = 0;
	ctx->out_tail = 0;
	ctx->out_npend = 0;

	ctx->ioc_events = 0;
	ctx->ioc_event_lock = 0;
	ctx->status = 0;
	ctx->saved_errors = 0;

	rt_16550_set_config(ctx, &default_config, &dummy);

	err = rtdm_irq_request(&ctx->irq_handle, irq[dev_id],
			rt_16550_interrupt, irqtype[dev_id],
			rtdm_fd_device(fd)->name, ctx);
	if (err) {
		/* reset DTR and RTS */
		rt_16550_reg_out(rt_16550_io_mode_from_ctx(ctx), ctx->base_addr,
				 MCR, 0);

		rt_16550_cleanup_ctx(ctx);

		return err;
	}

	rtdm_lock_get_irqsave(&ctx->lock, lock_ctx);

	/* enable interrupts */
	ctx->ier_status = IER_RX;
	rt_16550_reg_out(rt_16550_io_mode_from_ctx(ctx), ctx->base_addr, IER,
			 IER_RX);

	rtdm_lock_put_irqrestore(&ctx->lock, lock_ctx);

	return 0;
}

void rt_16550_close(struct rtdm_fd *fd)
{
	struct rt_16550_context *ctx;
	unsigned long base;
	int mode;
	uint64_t *in_history;
	rtdm_lockctx_t lock_ctx;

	ctx = rtdm_fd_to_private(fd);
	base = ctx->base_addr;
	mode = rt_16550_io_mode_from_ctx(ctx);

	rtdm_lock_get_irqsave(&ctx->lock, lock_ctx);

	/* reset DTR and RTS */
	rt_16550_reg_out(mode, base, MCR, 0);

	/* mask all UART interrupts and clear pending ones. */
	rt_16550_reg_out(mode, base, IER, 0);
	rt_16550_reg_in(mode, base, IIR);
	rt_16550_reg_in(mode, base, LSR);
	rt_16550_reg_in(mode, base, RHR);
	rt_16550_reg_in(mode, base, MSR);

	in_history = ctx->in_history;
	ctx->in_history = NULL;

	rtdm_lock_put_irqrestore(&ctx->lock, lock_ctx);

	rtdm_irq_free(&ctx->irq_handle);

	rt_16550_cleanup_ctx(ctx);

	kfree(in_history);
}

int rt_16550_ioctl(struct rtdm_fd *fd, unsigned int request, void *arg)
{
	rtdm_lockctx_t lock_ctx;
	struct rt_16550_context *ctx;
	int err = 0;
	unsigned long base;
	int mode;

	ctx = rtdm_fd_to_private(fd);
	base = ctx->base_addr;
	mode = rt_16550_io_mode_from_ctx(ctx);

	switch (request) {
	case RTSER_RTIOC_GET_CONFIG:
		if (rtdm_fd_is_user(fd))
			err =
			    rtdm_safe_copy_to_user(fd, arg,
						   &ctx->config,
						   sizeof(struct
							  rtser_config));
		else
			memcpy(arg, &ctx->config,
			       sizeof(struct rtser_config));
		break;

	case RTSER_RTIOC_SET_CONFIG: {
		struct rtser_config *config;
		struct rtser_config config_buf;
		uint64_t *hist_buf = NULL;

		config = (struct rtser_config *)arg;

		if (rtdm_fd_is_user(fd)) {
			err =
			    rtdm_safe_copy_from_user(fd, &config_buf,
						     arg,
						     sizeof(struct
							    rtser_config));
			if (err)
				return err;

			config = &config_buf;
		}

		if ((config->config_mask & RTSER_SET_BAUD) &&
		    (config->baud_rate >
			    baud_base[rtdm_fd_minor(fd)] ||
			    config->baud_rate <= 0))
			/* invalid baudrate for this port */
			return -EINVAL;

		if (config->config_mask & RTSER_SET_TIMESTAMP_HISTORY) {
			/*
			 * Reflect the call to non-RT as we will likely
			 * allocate or free the buffer.
			 */
			if (rtdm_in_rt_context())
				return -ENOSYS;

			if (config->timestamp_history &
			    RTSER_RX_TIMESTAMP_HISTORY)
				hist_buf = kmalloc(IN_BUFFER_SIZE *
						   sizeof(nanosecs_abs_t),
						   GFP_KERNEL);
		}

		rt_16550_set_config(ctx, config, &hist_buf);

		if (hist_buf)
			kfree(hist_buf);

		break;
	}

	case RTSER_RTIOC_GET_STATUS: {
		int status;

		rtdm_lock_get_irqsave(&ctx->lock, lock_ctx);

		status = ctx->saved_errors | ctx->status;
		ctx->status = 0;
		ctx->saved_errors = 0;
		ctx->ioc_events &= ~RTSER_EVENT_ERRPEND;

		rtdm_lock_put_irqrestore(&ctx->lock, lock_ctx);

		if (rtdm_fd_is_user(fd)) {
			struct rtser_status status_buf;

			status_buf.line_status =
			    rt_16550_reg_in(mode, base, LSR) | status;
			status_buf.modem_status =
			    rt_16550_reg_in(mode, base, MSR);

			err =
			    rtdm_safe_copy_to_user(fd, arg,
						   &status_buf,
						   sizeof(struct
							  rtser_status));
		} else {
			((struct rtser_status *)arg)->line_status =
			    rt_16550_reg_in(mode, base, LSR) | status;
			((struct rtser_status *)arg)->modem_status =
			    rt_16550_reg_in(mode, base, MSR);
		}
		break;
	}

	case RTSER_RTIOC_GET_CONTROL:
		if (rtdm_fd_is_user(fd))
			err =
			    rtdm_safe_copy_to_user(fd, arg,
						   &ctx->mcr_status,
						   sizeof(int));
		else
			*(int *)arg = ctx->mcr_status;

		break;

	case RTSER_RTIOC_SET_CONTROL: {
		int new_mcr = (long)arg;

		rtdm_lock_get_irqsave(&ctx->lock, lock_ctx);
		ctx->mcr_status = new_mcr;
		rt_16550_reg_out(mode, base, MCR, new_mcr);
		rtdm_lock_put_irqrestore(&ctx->lock, lock_ctx);
		break;
	}

	case RTSER_RTIOC_WAIT_EVENT: {
		struct rtser_event ev = { .rxpend_timestamp = 0 };
		rtdm_toseq_t timeout_seq;

		if (!rtdm_in_rt_context())
			return -ENOSYS;

		/* Only one waiter allowed, stop any further attempts here. */
		if (test_and_set_bit(0, &ctx->ioc_event_lock))
			return -EBUSY;

		rtdm_toseq_init(&timeout_seq, ctx->config.event_timeout);

		rtdm_lock_get_irqsave(&ctx->lock, lock_ctx);

		while (!ctx->ioc_events) {
			/* Only enable error interrupt
			   when the user waits for it. */
			if (ctx->config.event_mask & RTSER_EVENT_ERRPEND) {
				ctx->ier_status |= IER_STAT;
				rt_16550_reg_out(mode, base, IER,
						 ctx->ier_status);
			}

			rtdm_lock_put_irqrestore(&ctx->lock, lock_ctx);

			err = rtdm_event_timedwait(&ctx->ioc_event,
						   ctx->config.event_timeout,
						   &timeout_seq);
			if (err) {
				/* Device has been closed? */
				if (err == -EIDRM)
					err = -EBADF;
				goto wait_unlock_out;
			}

			rtdm_lock_get_irqsave(&ctx->lock, lock_ctx);
		}

		ev.events = ctx->ioc_events;
		ctx->ioc_events &=
		    ~(RTSER_EVENT_MODEMHI | RTSER_EVENT_MODEMLO);

		ev.last_timestamp = ctx->last_timestamp;
		ev.rx_pending = ctx->in_npend;

		if (ctx->in_history)
			ev.rxpend_timestamp = ctx->in_history[ctx->in_head];

		rtdm_lock_put_irqrestore(&ctx->lock, lock_ctx);

		if (rtdm_fd_is_user(fd))
			err =
			    rtdm_safe_copy_to_user(fd, arg, &ev,
						   sizeof(struct
							  rtser_event));
			else
				memcpy(arg, &ev, sizeof(struct rtser_event));

	      wait_unlock_out:
		/* release the simple event waiter lock */
		clear_bit(0, &ctx->ioc_event_lock);
		break;
	}

	case RTSER_RTIOC_BREAK_CTL: {
		int lcr = ((long)arg & RTSER_BREAK_SET) << 6;

		rtdm_lock_get_irqsave(&ctx->lock, lock_ctx);

		lcr |=
		    (ctx->config.parity << 3) | (ctx->config.stop_bits << 2) |
		    ctx->config.data_bits;

		rt_16550_reg_out(mode, base, LCR, lcr);

		rtdm_lock_put_irqrestore(&ctx->lock, lock_ctx);
		break;
	}

	case RTIOC_PURGE: {
		int fcr = 0;

		rtdm_lock_get_irqsave(&ctx->lock, lock_ctx);
		if ((long)arg & RTDM_PURGE_RX_BUFFER) {
			ctx->in_head = 0;
			ctx->in_tail = 0;
			ctx->in_npend = 0;
			ctx->status = 0;
			fcr |= FCR_FIFO | FCR_RESET_RX;
			rt_16550_reg_in(mode, base, RHR);
		}
		if ((long)arg & RTDM_PURGE_TX_BUFFER) {
			ctx->out_head = 0;
			ctx->out_tail = 0;
			ctx->out_npend = 0;
			fcr |= FCR_FIFO | FCR_RESET_TX;
		}
		if (fcr) {
			rt_16550_reg_out(mode, base, FCR, fcr);
			rt_16550_reg_out(mode, base, FCR,
					 FCR_FIFO | ctx->config.fifo_depth);
		}
		rtdm_lock_put_irqrestore(&ctx->lock, lock_ctx);
		break;
	}

	default:
		err = -ENOTTY;
	}

	return err;
}

ssize_t rt_16550_read(struct rtdm_fd *fd, void *buf, size_t nbyte)
{
	struct rt_16550_context *ctx;
	rtdm_lockctx_t lock_ctx;
	size_t read = 0;
	int pending;
	int block;
	int subblock;
	int in_pos;
	char *out_pos = (char *)buf;
	rtdm_toseq_t timeout_seq;
	ssize_t ret = -EAGAIN;	/* for non-blocking read */
	int nonblocking;

	if (nbyte == 0)
		return 0;

	if (rtdm_fd_is_user(fd) && !rtdm_rw_user_ok(fd, buf, nbyte))
		return -EFAULT;

	ctx = rtdm_fd_to_private(fd);

	rtdm_toseq_init(&timeout_seq, ctx->config.rx_timeout);

	/* non-blocking is handled separately here */
	nonblocking = (ctx->config.rx_timeout < 0);

	/* only one reader allowed, stop any further attempts here */
	if (test_and_set_bit(0, &ctx->in_lock))
		return -EBUSY;

	rtdm_lock_get_irqsave(&ctx->lock, lock_ctx);

	while (1) {
		/* switch on error interrupt - the user is ready to listen */
		if ((ctx->ier_status & IER_STAT) == 0) {
			ctx->ier_status |= IER_STAT;
			rt_16550_reg_out(rt_16550_io_mode_from_ctx(ctx),
					 ctx->base_addr, IER,
					 ctx->ier_status);
		}

		if (ctx->status) {
			if (ctx->status & RTSER_LSR_BREAK_IND)
				ret = -EPIPE;
			else
				ret = -EIO;
			ctx->saved_errors = ctx->status &
			    (RTSER_LSR_OVERRUN_ERR | RTSER_LSR_PARITY_ERR |
			     RTSER_LSR_FRAMING_ERR | RTSER_SOFT_OVERRUN_ERR);
			ctx->status = 0;
			break;
		}

		pending = ctx->in_npend;

		if (pending > 0) {
			block = subblock = (pending <= nbyte) ? pending : nbyte;
			in_pos = ctx->in_head;

			rtdm_lock_put_irqrestore(&ctx->lock, lock_ctx);

			/* Do we have to wrap around the buffer end? */
			if (in_pos + subblock > IN_BUFFER_SIZE) {
				/* Treat the block between head and buffer end
				   separately. */
				subblock = IN_BUFFER_SIZE - in_pos;

				if (rtdm_fd_is_user(fd)) {
					if (rtdm_copy_to_user
					    (fd, out_pos,
					     &ctx->in_buf[in_pos],
					     subblock) != 0) {
						ret = -EFAULT;
						goto break_unlocked;
					}
				} else
					memcpy(out_pos, &ctx->in_buf[in_pos],
					       subblock);

				read += subblock;
				out_pos += subblock;

				subblock = block - subblock;
				in_pos = 0;
			}

			if (rtdm_fd_is_user(fd)) {
				if (rtdm_copy_to_user(fd, out_pos,
						      &ctx->in_buf[in_pos],
						      subblock) != 0) {
					ret = -EFAULT;
					goto break_unlocked;
				}
			} else
				memcpy(out_pos, &ctx->in_buf[in_pos], subblock);

			read += subblock;
			out_pos += subblock;
			nbyte -= block;

			rtdm_lock_get_irqsave(&ctx->lock, lock_ctx);

			ctx->in_head =
			    (ctx->in_head + block) & (IN_BUFFER_SIZE - 1);
			if ((ctx->in_npend -= block) == 0)
				ctx->ioc_events &= ~RTSER_EVENT_RXPEND;

			if (nbyte == 0)
				break; /* All requested bytes read. */

			continue;
		}

		if (nonblocking)
			/* ret was set to EAGAIN in case of a real
			   non-blocking call or contains the error
			   returned by rtdm_event_wait[_until] */
			break;

		ctx->in_nwait = nbyte;

		rtdm_lock_put_irqrestore(&ctx->lock, lock_ctx);

		ret = rtdm_event_timedwait(&ctx->in_event,
					   ctx->config.rx_timeout,
					   &timeout_seq);
		if (ret < 0) {
			if (ret == -EIDRM) {
				/* Device has been closed -
				   return immediately. */
				return -EBADF;
			}

			rtdm_lock_get_irqsave(&ctx->lock, lock_ctx);

			nonblocking = 1;
			if (ctx->in_npend > 0) {
				/* Final turn: collect pending bytes
				   before exit. */
				continue;
			}

			ctx->in_nwait = 0;
			break;
		}

		rtdm_lock_get_irqsave(&ctx->lock, lock_ctx);
	}

	rtdm_lock_put_irqrestore(&ctx->lock, lock_ctx);

break_unlocked:
	/* Release the simple reader lock, */
	clear_bit(0, &ctx->in_lock);

	if ((read > 0) && ((ret == 0) || (ret == -EAGAIN) ||
			   (ret == -ETIMEDOUT) || (ret == -EINTR)))
		ret = read;

	return ret;
}

ssize_t rt_16550_write(struct rtdm_fd *fd, const void *buf, size_t nbyte)
{
	struct rt_16550_context *ctx;
	rtdm_lockctx_t lock_ctx;
	size_t written = 0;
	int free;
	int block;
	int subblock;
	int out_pos;
	char *in_pos = (char *)buf;
	rtdm_toseq_t timeout_seq;
	ssize_t ret;

	if (nbyte == 0)
		return 0;

	if (rtdm_fd_is_user(fd) && !rtdm_read_user_ok(fd, buf, nbyte))
		return -EFAULT;

	ctx = rtdm_fd_to_private(fd);

	rtdm_toseq_init(&timeout_seq, ctx->config.rx_timeout);

	/* Make write operation atomic. */
	ret = rtdm_mutex_timedlock(&ctx->out_lock, ctx->config.rx_timeout,
				   &timeout_seq);
	if (ret)
		return ret;

	while (nbyte > 0) {
		rtdm_lock_get_irqsave(&ctx->lock, lock_ctx);

		free = OUT_BUFFER_SIZE - ctx->out_npend;

		if (free > 0) {
			block = subblock = (nbyte <= free) ? nbyte : free;
			out_pos = ctx->out_tail;

			rtdm_lock_put_irqrestore(&ctx->lock, lock_ctx);

			/* Do we have to wrap around the buffer end? */
			if (out_pos + subblock > OUT_BUFFER_SIZE) {
				/* Treat the block between head and buffer
				   end separately. */
				subblock = OUT_BUFFER_SIZE - out_pos;

				if (rtdm_fd_is_user(fd)) {
					if (rtdm_copy_from_user
					    (fd,
					     &ctx->out_buf[out_pos],
					     in_pos, subblock) != 0) {
						ret = -EFAULT;
						break;
					}
				} else
					memcpy(&ctx->out_buf[out_pos], in_pos,
					       subblock);

				written += subblock;
				in_pos += subblock;

				subblock = block - subblock;
				out_pos = 0;
			}

			if (rtdm_fd_is_user(fd)) {
				if (rtdm_copy_from_user
				    (fd, &ctx->out_buf[out_pos],
				     in_pos, subblock) != 0) {
					ret = -EFAULT;
					break;
				}
			} else
				memcpy(&ctx->out_buf[out_pos], in_pos, block);

			written += subblock;
			in_pos += subblock;
			nbyte -= block;

			rtdm_lock_get_irqsave(&ctx->lock, lock_ctx);

			ctx->out_tail =
			    (ctx->out_tail + block) & (OUT_BUFFER_SIZE - 1);
			ctx->out_npend += block;

			/* unmask tx interrupt */
			ctx->ier_status |= IER_TX;
			rt_16550_reg_out(rt_16550_io_mode_from_ctx(ctx),
					 ctx->base_addr, IER,
					 ctx->ier_status);

			rtdm_lock_put_irqrestore(&ctx->lock, lock_ctx);
			continue;
		}

		rtdm_lock_put_irqrestore(&ctx->lock, lock_ctx);

		ret =
		    rtdm_event_timedwait(&ctx->out_event,
					 ctx->config.tx_timeout,
					 &timeout_seq);
		if (ret < 0) {
			if (ret == -EIDRM) {
				/* Device has been closed -
				   return immediately. */
				return -EBADF;
			}
			if (ret == -EWOULDBLOCK) {
				/* Fix error code for non-blocking mode. */
				ret = -EAGAIN;
			}
			break;
		}
	}

	rtdm_mutex_unlock(&ctx->out_lock);

	if ((written > 0) && ((ret == 0) || (ret == -EAGAIN) ||
			      (ret == -ETIMEDOUT) || (ret == -EINTR)))
		ret = written;

	return ret;
}

static struct rtdm_driver uart16550A_driver = {
	.profile_info		= RTDM_PROFILE_INFO(uart16550A,
						    RTDM_CLASS_SERIAL,
						    RTDM_SUBCLASS_16550A,
						    RTSER_PROFILE_VER),
	.device_flags		= RTDM_NAMED_DEVICE | RTDM_EXCLUSIVE,
	.device_count		= MAX_DEVICES,
	.context_size		= sizeof(struct rt_16550_context),
	.ops = {
		.open		= rt_16550_open,
		.close		= rt_16550_close,
		.ioctl_rt	= rt_16550_ioctl,
		.ioctl_nrt	= rt_16550_ioctl,
		.read_rt	= rt_16550_read,
		.write_rt	= rt_16550_write,
	},
};

void rt_16550_exit(void);

int __init rt_16550_init(void)
{
	struct rtdm_device *dev;
	unsigned long base;
	char *name;
	int mode;
	int err;
	int i;

	if (!realtime_core_enabled())
		return 0;

	rt_16550_pnp_init();
	rt_16550_pci_init();

	for (i = 0; i < MAX_DEVICES; i++) {
		if (!rt_16550_addr_param(i))
			continue;

		err = -EINVAL;
		if (!irq[i] || !rt_16550_addr_param_valid(i))
			goto cleanup_out;

		dev = kmalloc(sizeof(struct rtdm_device) +
			      RTDM_MAX_DEVNAME_LEN, GFP_KERNEL);
		err = -ENOMEM;
		if (!dev)
			goto cleanup_out;

		dev->driver = &uart16550A_driver;
		dev->label = "rtser%d";
		name = (char *)(dev + 1);
		ksformat(name, RTDM_MAX_DEVNAME_LEN, dev->label, i);

		err = rt_16550_init_io(i, name);
		if (err)
			goto kfree_out;

		if (baud_base[i] == 0)
			baud_base[i] = DEFAULT_BAUD_BASE;

		if (tx_fifo[i] == 0)
			tx_fifo[i] = DEFAULT_TX_FIFO;

		/* Mask all UART interrupts and clear pending ones. */
		base = rt_16550_base_addr(i);
		mode = rt_16550_io_mode(i);
		rt_16550_reg_out(mode, base, IER, 0);
		rt_16550_reg_in(mode, base, IIR);
		rt_16550_reg_in(mode, base, LSR);
		rt_16550_reg_in(mode, base, RHR);
		rt_16550_reg_in(mode, base, MSR);

		err = rtdm_dev_register(dev);

		if (err)
			goto release_io_out;

		device[i] = dev;
	}

	return 0;

      release_io_out:
	rt_16550_release_io(i);

      kfree_out:
	kfree(dev);

      cleanup_out:
	rt_16550_exit();

	return err;
}

void rt_16550_exit(void)
{
	int i;

	if (!realtime_core_enabled())
		return;

	for (i = 0; i < MAX_DEVICES; i++)
		if (device[i]) {
			rtdm_dev_unregister(device[i]);
			rt_16550_release_io(i);
			kfree(device[i]);
		}

	rt_16550_pci_cleanup();
	rt_16550_pnp_cleanup();
}

module_init(rt_16550_init);
module_exit(rt_16550_exit);
