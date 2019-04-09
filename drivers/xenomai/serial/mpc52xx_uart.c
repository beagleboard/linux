/*
 * Copyright (C) 2011 Wolfgang Grandegger <wg@denx.de>.
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
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/of_platform.h>
#include <linux/io.h>

#include <asm/mpc52xx.h>
#include <asm/mpc52xx_psc.h>

#include <rtdm/serial.h>
#include <rtdm/driver.h>

MODULE_DESCRIPTION("RTDM-based driver for MPC52xx UARTs");
MODULE_AUTHOR("Wolfgang Grandegger <wg@denx.de>");
MODULE_VERSION("1.0.0");
MODULE_LICENSE("GPL");

#define RT_MPC52XX_UART_DRVNAM	"xeno_mpc52xx_uart"

#define IN_BUFFER_SIZE		512
#define OUT_BUFFER_SIZE		512

#define PARITY_MASK		0x03
#define DATA_BITS_MASK		0x03
#define STOP_BITS_MASK		0x01
#define FIFO_MASK		0xC0
#define EVENT_MASK		0x0F


struct rt_mpc52xx_uart_port {
	const struct device *dev;
	struct mpc52xx_psc __iomem *psc;
	struct mpc52xx_psc_fifo __iomem *fifo;
	unsigned int uartclk;
	int irq;
	int num;
};

struct rt_mpc52xx_uart_ctx {
	struct rtser_config config;	/* current device configuration */

	rtdm_irq_t irq_handle;		/* device IRQ handle */
	rtdm_lock_t lock;		/* lock to protect context struct */

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


	int mcr_status;			/* emulated MCR cache */
	int status;			/* cache for LSR + soft-states */
	int saved_errors;		/* error cache for RTIOC_GET_STATUS */

	unsigned int imr_status;	/* interrupt mask register cache */
	int tx_empty;			/* shift register empty flag */

	struct rt_mpc52xx_uart_port *port; /* Port related data */
};

static const struct rtser_config default_config = {
	.config_mask = 0xFFFF,
	.baud_rate = RTSER_DEF_BAUD,
	.parity = RTSER_DEF_PARITY,
	.data_bits = RTSER_DEF_BITS,
	.stop_bits = RTSER_DEF_STOPB,
	.handshake = RTSER_DEF_HAND,
	.fifo_depth = RTSER_DEF_FIFO_DEPTH,
	.rx_timeout = RTSER_DEF_TIMEOUT,
	.tx_timeout = RTSER_DEF_TIMEOUT,
	.event_timeout = RTSER_DEF_TIMEOUT,
	.timestamp_history = RTSER_DEF_TIMESTAMP_HISTORY,
	.event_mask = RTSER_DEF_EVENT_MASK,
	.rs485 = RTSER_DEF_RS485,
};

/* lookup table for matching device nodes to index numbers */
static struct device_node *rt_mpc52xx_uart_nodes[MPC52xx_PSC_MAXNUM];

static inline void psc_fifo_init(struct rt_mpc52xx_uart_ctx *ctx)
{
	out_8(&ctx->port->fifo->rfcntl, 0x00);
	out_be16(&ctx->port->fifo->rfalarm, 0x1ff);
	out_8(&ctx->port->fifo->tfcntl, 0x07);
	out_be16(&ctx->port->fifo->tfalarm, 0x80);
}

static inline int psc_raw_rx_rdy(struct rt_mpc52xx_uart_ctx *ctx)
{
	return in_be16(&ctx->port->psc->mpc52xx_psc_status) &
		MPC52xx_PSC_SR_RXRDY;
}

static inline int psc_raw_tx_rdy(struct rt_mpc52xx_uart_ctx *ctx)
{
	return in_be16(&ctx->port->psc->mpc52xx_psc_status) &
		MPC52xx_PSC_SR_TXRDY;
}

static inline int psc_rx_rdy(struct rt_mpc52xx_uart_ctx *ctx)
{
	return in_be16(&ctx->port->psc->mpc52xx_psc_isr) &
		ctx->imr_status & MPC52xx_PSC_IMR_RXRDY;
}

static int psc_tx_rdy(struct rt_mpc52xx_uart_ctx *ctx)
{
	return in_be16(&ctx->port->psc->mpc52xx_psc_isr) &
		ctx->imr_status & MPC52xx_PSC_IMR_TXRDY;
}

static inline int psc_tx_empty(struct rt_mpc52xx_uart_ctx *ctx)
{
	return in_be16(&ctx->port->psc->mpc52xx_psc_status) &
		MPC52xx_PSC_SR_TXEMP;
}

static inline void psc_start_tx(struct rt_mpc52xx_uart_ctx *ctx)
{
	ctx->imr_status |= MPC52xx_PSC_IMR_TXRDY;
	out_be16(&ctx->port->psc->mpc52xx_psc_imr, ctx->imr_status);
}

static inline void psc_stop_tx(struct rt_mpc52xx_uart_ctx *ctx)
{
	ctx->imr_status &= ~MPC52xx_PSC_IMR_TXRDY;
	out_be16(&ctx->port->psc->mpc52xx_psc_imr, ctx->imr_status);
}

static inline void psc_stop_rx(struct rt_mpc52xx_uart_ctx *ctx)
{
	ctx->imr_status &= ~MPC52xx_PSC_IMR_RXRDY;
	out_be16(&ctx->port->psc->mpc52xx_psc_imr, ctx->imr_status);
}

static inline void psc_write_char(struct rt_mpc52xx_uart_ctx *ctx,
				  unsigned char c)
{
	out_8(&ctx->port->psc->mpc52xx_psc_buffer_8, c);
}

static inline unsigned char psc_read_char(struct rt_mpc52xx_uart_ctx *ctx)
{
	return in_8(&ctx->port->psc->mpc52xx_psc_buffer_8);
}

static inline void psc_disable_ints(struct rt_mpc52xx_uart_ctx *ctx)
{
	ctx->imr_status = 0;
	out_be16(&ctx->port->psc->mpc52xx_psc_imr, ctx->imr_status);
}

static void psc_set_mcr(struct rt_mpc52xx_uart_ctx *ctx,
			unsigned int mcr)
{
	if (mcr & RTSER_MCR_RTS)
		out_8(&ctx->port->psc->op1, MPC52xx_PSC_OP_RTS);
	else
		out_8(&ctx->port->psc->op0, MPC52xx_PSC_OP_RTS);
}

/* FIXME: status interrupts not yet handled properly */
static unsigned int psc_get_msr(struct rt_mpc52xx_uart_ctx *ctx)
{
	unsigned int msr = RTSER_MSR_DSR;
	u8 status = in_8(&ctx->port->psc->mpc52xx_psc_ipcr);

	if (!(status & MPC52xx_PSC_CTS))
		msr |= RTSER_MSR_CTS;
	if (!(status & MPC52xx_PSC_DCD))
		msr |= RTSER_MSR_DCD;

	return msr;
}

static void psc_enable_ms(struct rt_mpc52xx_uart_ctx *ctx)
{
	struct mpc52xx_psc *psc = ctx->port->psc;

	/* clear D_*-bits by reading them */
	in_8(&psc->mpc52xx_psc_ipcr);
	/* enable CTS and DCD as IPC interrupts */
	out_8(&psc->mpc52xx_psc_acr, MPC52xx_PSC_IEC_CTS | MPC52xx_PSC_IEC_DCD);

	ctx->imr_status |= MPC52xx_PSC_IMR_IPC;
	out_be16(&psc->mpc52xx_psc_imr, ctx->imr_status);
}

static void psc_disable_ms(struct rt_mpc52xx_uart_ctx *ctx)
{
	struct mpc52xx_psc *psc = ctx->port->psc;

	/* disable CTS and DCD as IPC interrupts */
	out_8(&psc->mpc52xx_psc_acr, 0);

	ctx->imr_status &= ~MPC52xx_PSC_IMR_IPC;
	out_be16(&psc->mpc52xx_psc_imr, ctx->imr_status);
}

static struct of_device_id mpc5200_gpio_ids[] = {
	{ .compatible = "fsl,mpc5200-gpio", },
	{ .compatible = "mpc5200-gpio", },
	{}
};

static void rt_mpc52xx_uart_init_hw(struct rt_mpc52xx_uart_port *port)
{
	struct mpc52xx_gpio __iomem *gpio;
	struct device_node *gpio_np;
	u32 port_config;

	if (port->num == 6) {
		gpio_np = of_find_matching_node(NULL, mpc5200_gpio_ids);
		gpio = of_iomap(gpio_np, 0);
		of_node_put(gpio_np);
		if (!gpio) {
			dev_err(port->dev, "PSC%d port_config: "
				"couldn't map gpio ids\n", port->num);
			return;
		}
		port_config = in_be32(&gpio->port_config);
		port_config &= 0xFF0FFFFF; /* port config for PSC6 */
		port_config |= 0x00500000;
		dev_dbg(port->dev, "PSC%d port_config: old:%x new:%x\n",
			port->num, in_be32(&gpio->port_config), port_config);
		out_be32(&gpio->port_config, port_config);
		iounmap(gpio);
	}
}

static inline void rt_mpc52xx_uart_put_char(struct rt_mpc52xx_uart_ctx *ctx,
					    uint64_t *timestamp,
					    unsigned char ch)
{
	ctx->in_buf[ctx->in_tail] = ch;
	if (ctx->in_history)
		ctx->in_history[ctx->in_tail] = *timestamp;
	ctx->in_tail = (ctx->in_tail + 1) & (IN_BUFFER_SIZE - 1);

	if (++ctx->in_npend > IN_BUFFER_SIZE) {
		ctx->status |= RTSER_SOFT_OVERRUN_ERR;
		ctx->in_npend--;
	}
}

static inline int rt_mpc52xx_uart_rx_interrupt(struct rt_mpc52xx_uart_ctx *ctx,
					       uint64_t *timestamp)
{
	int rbytes = 0;
	int psc_status;

	psc_status = in_be16(&ctx->port->psc->mpc52xx_psc_status);
	while (psc_status & MPC52xx_PSC_SR_RXRDY) {
		/* read input character */
		rt_mpc52xx_uart_put_char(ctx, timestamp, psc_read_char(ctx));
		rbytes++;

		/* save new errors */
		if (psc_status & (MPC52xx_PSC_SR_OE | MPC52xx_PSC_SR_PE |
				  MPC52xx_PSC_SR_FE | MPC52xx_PSC_SR_RB)) {
			if (psc_status & MPC52xx_PSC_SR_PE)
				ctx->status |= RTSER_LSR_PARITY_ERR;
			if (psc_status & MPC52xx_PSC_SR_FE)
				ctx->status |= RTSER_LSR_FRAMING_ERR;
			if (psc_status & MPC52xx_PSC_SR_RB)
				ctx->status |= RTSER_LSR_BREAK_IND;

			/*
			 * Overrun is special, since it's reported
			 * immediately, and doesn't affect the current
			 * character.
			 */
			if (psc_status & MPC52xx_PSC_SR_OE) {
				ctx->status |= RTSER_LSR_OVERRUN_ERR;
				rt_mpc52xx_uart_put_char(ctx, timestamp, 0);
				rbytes++;
			}

			/* Clear error condition */
			out_8(&ctx->port->psc->command,
			      MPC52xx_PSC_RST_ERR_STAT);
		}

		psc_status = in_be16(&ctx->port->psc->mpc52xx_psc_status);
	};

	return rbytes;
}

static inline int rt_mpc52xx_uart_tx_interrupt(struct rt_mpc52xx_uart_ctx *ctx)
{
	while (psc_raw_tx_rdy(ctx) && (ctx->out_npend > 0)) {
		if (ctx->config.rs485 &&
		    (ctx->mcr_status & RTSER_MCR_RTS) == 0) {
			/* switch RTS */
			ctx->mcr_status |= RTSER_MCR_RTS;
			dev_dbg(ctx->port->dev, "Set RTS, mcr_status=%#x\n",
				ctx->mcr_status);
			psc_set_mcr(ctx, ctx->mcr_status);
		}
		if (ctx->config.rs485 ||
		    ((ctx->config.event_mask & RTSER_EVENT_TXEMPTY) &&
		     (ctx->imr_status & MPC52xx_PSC_IMR_TXEMP) == 0)) {
			/* enable tx-empty interrupt */
			ctx->imr_status |= MPC52xx_PSC_IMR_TXEMP;
			dev_dbg(ctx->port->dev, "Enable TXEMP interrupt, "
				"imr_status=%#x\n", ctx->imr_status);
			out_be16(&ctx->port->psc->mpc52xx_psc_imr,
				 ctx->imr_status);
		}

		psc_write_char(ctx, ctx->out_buf[ctx->out_head++]);
		ctx->out_head &= OUT_BUFFER_SIZE - 1;
		ctx->out_npend--;
	}

	return ctx->out_npend;
}

static int rt_mpc52xx_uart_interrupt(rtdm_irq_t *irq_context)
{
	struct rt_mpc52xx_uart_ctx *ctx;
	uint64_t timestamp = rtdm_clock_read();
	int rbytes = 0;
	int events = 0;
	int ret = RTDM_IRQ_NONE;
	int goon = 1;
	int n;

	ctx = rtdm_irq_get_arg(irq_context, struct rt_mpc52xx_uart_ctx);

	rtdm_lock_get(&ctx->lock);

	while (goon) {
		goon = 0;
		if (psc_rx_rdy(ctx)) {
			dev_dbg(ctx->port->dev, "RX interrupt\n");
			n = rt_mpc52xx_uart_rx_interrupt(ctx, &timestamp);
			if (n) {
				rbytes += n;
				events |= RTSER_EVENT_RXPEND;
			}
		}
		if (psc_tx_rdy(ctx))
			goon |= rt_mpc52xx_uart_tx_interrupt(ctx);

		if (psc_tx_empty(ctx)) {
			if (ctx->config.rs485 &&
			    (ctx->mcr_status & RTSER_MCR_RTS)) {
				/* reset RTS */
				ctx->mcr_status &= ~RTSER_MCR_RTS;
				dev_dbg(ctx->port->dev, "Reset RTS, "
					"mcr_status=%#x\n", ctx->mcr_status);
				psc_set_mcr(ctx, ctx->mcr_status);
			}
			/* disable tx-empty interrupt */
			ctx->imr_status &= ~MPC52xx_PSC_IMR_TXEMP;
			dev_dbg(ctx->port->dev, "Disable TXEMP interrupt, "
				"imr_status=%#x\n", ctx->imr_status);
			out_be16(&ctx->port->psc->mpc52xx_psc_imr,
				 ctx->imr_status);

			events |= RTSER_EVENT_TXEMPTY;
			ctx->tx_empty = 1;
		}

		if (ctx->config.event_mask &
		    (RTSER_EVENT_MODEMHI | RTSER_EVENT_MODEMLO)) {
			u8 status = in_8(&ctx->port->psc->mpc52xx_psc_ipcr);

			if (status & MPC52xx_PSC_D_DCD)
				events |= (status & MPC52xx_PSC_DCD) ?
					RTSER_EVENT_MODEMLO :
					RTSER_EVENT_MODEMHI;
			if (status & MPC52xx_PSC_D_CTS)
				events |= (status & MPC52xx_PSC_CTS) ?
					RTSER_EVENT_MODEMLO :
					RTSER_EVENT_MODEMHI;
			dev_dbg(ctx->port->dev, "Modem line changed, "
				"events=%#x\n", events);
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

	if (ctx->status)
		events |= RTSER_EVENT_ERRPEND;

	if (events & ctx->config.event_mask) {
		int old_events = ctx->ioc_events;

		ctx->last_timestamp = timestamp;
		ctx->ioc_events = events;

		if (!old_events)
			rtdm_event_signal(&ctx->ioc_event);
	}

	if ((ctx->imr_status & MPC52xx_PSC_IMR_TXRDY) &&
	    (ctx->out_npend == 0)) {
		psc_stop_tx(ctx);
		rtdm_event_signal(&ctx->out_event);
	}

	rtdm_lock_put(&ctx->lock);

	return ret;
}


static int rt_mpc52xx_uart_set_config(struct rt_mpc52xx_uart_ctx *ctx,
				      const struct rtser_config *config,
				      uint64_t **in_history_ptr)
{
	rtdm_lockctx_t lock_ctx;
	int err = 0;

	/* make line configuration atomic and IRQ-safe */
	rtdm_lock_get_irqsave(&ctx->lock, lock_ctx);

	if (config->config_mask & RTSER_SET_BAUD)
		ctx->config.baud_rate = config->baud_rate;
	if (config->config_mask & RTSER_SET_PARITY)
		ctx->config.parity = config->parity & PARITY_MASK;
	if (config->config_mask & RTSER_SET_DATA_BITS)
		ctx->config.data_bits = config->data_bits & DATA_BITS_MASK;
	if (config->config_mask & RTSER_SET_STOP_BITS)
		ctx->config.stop_bits = config->stop_bits & STOP_BITS_MASK;
	if (config->config_mask & RTSER_SET_HANDSHAKE)
		ctx->config.handshake = config->handshake;

	if (config->config_mask & (RTSER_SET_PARITY |
				   RTSER_SET_DATA_BITS | RTSER_SET_STOP_BITS |
				   RTSER_SET_BAUD | RTSER_SET_HANDSHAKE)) {
		struct mpc52xx_psc *psc = ctx->port->psc;
		unsigned char mr1 = 0, mr2 = 0;
		unsigned int divisor;
		u16 prescaler;

		switch (ctx->config.data_bits) {
		case RTSER_5_BITS:
			mr1 |= MPC52xx_PSC_MODE_5_BITS;
			break;
		case RTSER_6_BITS:
			mr1 |= MPC52xx_PSC_MODE_6_BITS;
			break;
		case RTSER_7_BITS:
			mr1 |= MPC52xx_PSC_MODE_7_BITS;
			break;
		case RTSER_8_BITS:
		default:
			mr1 |= MPC52xx_PSC_MODE_8_BITS;
			break;
		}

		switch (ctx->config.parity) {
		case RTSER_ODD_PARITY:
			mr1 |= MPC52xx_PSC_MODE_PARODD;
			break;
		case RTSER_EVEN_PARITY:
			mr1 |= MPC52xx_PSC_MODE_PAREVEN;
			break;
		case RTSER_NO_PARITY:
		default:
			mr1 |= MPC52xx_PSC_MODE_PARNONE;
			break;
		}

		if (ctx->config.stop_bits == RTSER_2_STOPB)
			mr2 |= (ctx->config.data_bits == RTSER_5_BITS) ?
				MPC52xx_PSC_MODE_ONE_STOP_5_BITS :
				MPC52xx_PSC_MODE_TWO_STOP;
		else
			mr2 |= MPC52xx_PSC_MODE_ONE_STOP;

		if (ctx->config.handshake == RTSER_RTSCTS_HAND) {
			mr1 |= MPC52xx_PSC_MODE_RXRTS;
			mr2 |= MPC52xx_PSC_MODE_TXCTS;
		} else if (config->config_mask & RTSER_SET_HANDSHAKE) {
			ctx->mcr_status =
				RTSER_MCR_DTR | RTSER_MCR_RTS | RTSER_MCR_OUT2;
			psc_set_mcr(ctx, ctx->mcr_status);
		}

		/* Reset the TX & RX */
		out_8(&psc->command, MPC52xx_PSC_RST_RX);
		out_8(&psc->command, MPC52xx_PSC_RST_TX);

		/* Send new mode settings */
		out_8(&psc->command, MPC52xx_PSC_SEL_MODE_REG_1);
		out_8(&psc->mode, mr1);
		out_8(&psc->mode, mr2);

		/* Set baudrate */
		divisor = (ctx->port->uartclk + 16 * ctx->config.baud_rate) /
			(32 * ctx->config.baud_rate);
		prescaler = 0xdd00;
		out_be16(&psc->mpc52xx_psc_clock_select, prescaler);
		out_8(&psc->ctur, divisor >> 8);
		out_8(&psc->ctlr, divisor & 0xff);

		dev_info(ctx->port->dev,
			 "mr1=%#x mr2=%#x baud=%d divisor=%d prescaler=%x\n",
			 mr1, mr2, ctx->config.baud_rate, divisor, prescaler);

		/* Reenable TX & RX */
		out_8(&psc->command, MPC52xx_PSC_TX_ENABLE);
		out_8(&psc->command, MPC52xx_PSC_RX_ENABLE);

		/* Enable RX */
		ctx->imr_status |= MPC52xx_PSC_IMR_RXRDY;
		out_be16(&ctx->port->psc->mpc52xx_psc_imr, ctx->imr_status);

		ctx->status = 0;
		ctx->ioc_events &= ~RTSER_EVENT_ERRPEND;

	}

	if (config->config_mask & RTSER_SET_RS485) {
		ctx->config.rs485 = config->rs485;
		if (config->rs485) {
			/* reset RTS */
			ctx->mcr_status &= ~RTSER_MCR_RTS;
			dev_dbg(ctx->port->dev, "Reset RTS, mcr_status=%#x\n",
				ctx->mcr_status);
			psc_set_mcr(ctx, ctx->mcr_status);
		}
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

		if ((config->event_mask & RTSER_EVENT_ERRPEND) &&
		    ctx->status)
			ctx->ioc_events |= RTSER_EVENT_ERRPEND;

		if ((config->event_mask & RTSER_EVENT_TXEMPTY) &&
		    !ctx->out_npend && ctx->tx_empty)
			ctx->ioc_events |= RTSER_EVENT_TXEMPTY;

		if (config->event_mask &
		    (RTSER_EVENT_MODEMHI | RTSER_EVENT_MODEMLO))
			psc_enable_ms(ctx);
		else
			psc_disable_ms(ctx);

		rtdm_lock_put_irqrestore(&ctx->lock, lock_ctx);
	}

	return err;
}

void rt_mpc52xx_uart_cleanup_ctx(struct rt_mpc52xx_uart_ctx *ctx)
{
	rtdm_event_destroy(&ctx->in_event);
	rtdm_event_destroy(&ctx->out_event);
	rtdm_event_destroy(&ctx->ioc_event);
	rtdm_mutex_destroy(&ctx->out_lock);
}

static int rt_mpc52xx_uart_open(struct rtdm_fd *fd, int oflags)
{
	struct rt_mpc52xx_uart_ctx *ctx;
	rtdm_lockctx_t lock_ctx;
	uint64_t *dummy;
	int err;

	ctx = rtdm_fd_to_private(fd);
	ctx->port = (struct rt_mpc52xx_uart_port *)rtdm_fd_device(fd)->device_data;

	/* IPC initialisation - cannot fail with used parameters */
	rtdm_lock_init(&ctx->lock);
	rtdm_event_init(&ctx->in_event, 0);
	rtdm_event_init(&ctx->out_event, 0);
	rtdm_event_init(&ctx->ioc_event, 0);
	rtdm_mutex_init(&ctx->out_lock);

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

	rtdm_lock_get_irqsave(&ctx->lock, lock_ctx);

	psc_disable_ints(ctx);

	/* Reset/activate the port, clear and enable interrupts */
	out_8(&ctx->port->psc->command, MPC52xx_PSC_RST_RX);
	out_8(&ctx->port->psc->command, MPC52xx_PSC_RST_TX);

	out_be32(&ctx->port->psc->sicr, 0);	/* UART mode DCD ignored */

	psc_fifo_init(ctx);

	out_8(&ctx->port->psc->command, MPC52xx_PSC_TX_ENABLE);
	out_8(&ctx->port->psc->command, MPC52xx_PSC_RX_ENABLE);

	rtdm_lock_put_irqrestore(&ctx->lock, lock_ctx);

	rt_mpc52xx_uart_set_config(ctx, &default_config, &dummy);

	err = rtdm_irq_request(&ctx->irq_handle, ctx->port->irq,
			       rt_mpc52xx_uart_interrupt, 0,
			       rtdm_fd_device(fd)->name, ctx);
	if (err) {
		psc_set_mcr(ctx, 0);
		rt_mpc52xx_uart_cleanup_ctx(ctx);

		return err;
	}

	return 0;
}

static void rt_mpc52xx_uart_close(struct rtdm_fd *fd)
{
	struct rt_mpc52xx_uart_ctx *ctx;
	uint64_t *in_history;
	rtdm_lockctx_t lock_ctx;

	ctx = rtdm_fd_to_private(fd);

	rtdm_lock_get_irqsave(&ctx->lock, lock_ctx);

	/* reset DTR and RTS */
	psc_set_mcr(ctx, 0);

	psc_disable_ints(ctx);

	in_history = ctx->in_history;
	ctx->in_history = NULL;

	rtdm_lock_put_irqrestore(&ctx->lock, lock_ctx);

	rtdm_irq_free(&ctx->irq_handle);

	rt_mpc52xx_uart_cleanup_ctx(ctx);

	kfree(in_history);
}

static int rt_mpc52xx_uart_ioctl(struct rtdm_fd *fd,
				 unsigned int request, void *arg)
{
	rtdm_lockctx_t lock_ctx;
	struct rt_mpc52xx_uart_ctx *ctx;
	int err = 0;

	ctx = rtdm_fd_to_private(fd);

	switch (request) {
	case RTSER_RTIOC_GET_CONFIG:
		if (rtdm_fd_is_user(fd))
			err = rtdm_safe_copy_to_user(fd, arg,
						     &ctx->config,
						     sizeof(struct
							    rtser_config));
		else
			memcpy(arg, &ctx->config, sizeof(struct rtser_config));
		break;

	case RTSER_RTIOC_SET_CONFIG: {
		struct rtser_config *config;
		struct rtser_config config_buf;
		uint64_t *hist_buf = NULL;

		config = (struct rtser_config *)arg;

		if (rtdm_fd_is_user(fd)) {
			err = rtdm_safe_copy_from_user(fd, &config_buf,
						       arg,
						       sizeof(struct
							      rtser_config));
			if (err)
				return err;

			config = &config_buf;
		}

		if ((config->config_mask & RTSER_SET_BAUD) &&
		    (config->baud_rate <= 0))
			/* invalid baudrate for this port */
			return -EINVAL;

		if (config->config_mask & RTSER_SET_TIMESTAMP_HISTORY) {
			/*
			 * Reflect the call to non-RT as we will likely
			 * allocate or free the buffer.
			 */
			if (rtdm_in_rt_context())
				return -ENOSYS;

			if (config->timestamp_history & RTSER_RX_TIMESTAMP_HISTORY)
				hist_buf = kmalloc(IN_BUFFER_SIZE *
						   sizeof(nanosecs_abs_t),
						   GFP_KERNEL);
		}

		rt_mpc52xx_uart_set_config(ctx, config, &hist_buf);

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

			status_buf.line_status = status;
			status_buf.modem_status = psc_get_msr(ctx);

			err = rtdm_safe_copy_to_user(fd, arg,
						     &status_buf,
						     sizeof(struct
							    rtser_status));
		} else {
			((struct rtser_status *)arg)->line_status = status;
			((struct rtser_status *)arg)->modem_status =
				psc_get_msr(ctx);
		}
		break;
	}

	case RTSER_RTIOC_GET_CONTROL:
		if (rtdm_fd_is_user(fd))
			err = rtdm_safe_copy_to_user(fd, arg,
						     &ctx->mcr_status,
						     sizeof(int));
		else
			*(int *)arg = ctx->mcr_status;

		break;

	case RTSER_RTIOC_SET_CONTROL: {
		int new_mcr = (long)arg;

		if ((new_mcr & RTSER_MCR_RTS) != RTSER_MCR_RTS)
			dev_warn(ctx->port->dev,
				 "MCR: Only RTS is supported\n");
		rtdm_lock_get_irqsave(&ctx->lock, lock_ctx);
		ctx->mcr_status = new_mcr & RTSER_MCR_RTS;
		psc_set_mcr(ctx, ctx->mcr_status);
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
		ctx->ioc_events &= ~(RTSER_EVENT_MODEMHI | RTSER_EVENT_MODEMLO);

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
		rtdm_lock_get_irqsave(&ctx->lock, lock_ctx);
		if ((long)arg & RTSER_BREAK_SET)
			out_8(&ctx->port->psc->command,
			      MPC52xx_PSC_START_BRK);
		else
			out_8(&ctx->port->psc->command,
			      MPC52xx_PSC_STOP_BRK);
		rtdm_lock_put_irqrestore(&ctx->lock, lock_ctx);
		break;
	}

#ifdef ISREADY
	case RTIOC_PURGE: {
		int fcr = 0;

		rtdm_lock_get_irqsave(&ctx->lock, lock_ctx);
		if ((long)arg & RTDM_PURGE_RX_BUFFER) {
			ctx->in_head = 0;
			ctx->in_tail = 0;
			ctx->in_npend = 0;
			ctx->status = 0;
			fcr |= FCR_FIFO | FCR_RESET_RX;
			rt_mpc52xx_uart_reg_in(mode, base, RHR);
		}
		if ((long)arg & RTDM_PURGE_TX_BUFFER) {
			ctx->out_head = 0;
			ctx->out_tail = 0;
			ctx->out_npend = 0;
			fcr |= FCR_FIFO | FCR_RESET_TX;
		}
		if (fcr) {
			rt_mpc52xx_uart_reg_out(mode, base, FCR, fcr);
			rt_mpc52xx_uart_reg_out(mode, base, FCR,
					 FCR_FIFO | ctx->config.fifo_depth);
		}
		rtdm_lock_put_irqrestore(&ctx->lock, lock_ctx);
		break;
	}
#endif

	default:
		err = -ENOTTY;
	}

	return err;
}

static ssize_t rt_mpc52xx_uart_read(struct rtdm_fd *fd, void *buf,
				    size_t nbyte)
{
	struct rt_mpc52xx_uart_ctx *ctx;
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

static ssize_t rt_mpc52xx_uart_write(struct rtdm_fd *fd,
				     const void *buf,
				     size_t nbyte)
{
	struct rt_mpc52xx_uart_ctx *ctx;
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

			/* Mark shift register not empty */
			ctx->ioc_events &= ~RTSER_EVENT_TXEMPTY;
			ctx->tx_empty = 0;

			psc_start_tx(ctx);

			rtdm_lock_put_irqrestore(&ctx->lock, lock_ctx);
			continue;
		}

		rtdm_lock_put_irqrestore(&ctx->lock, lock_ctx);

		ret = rtdm_event_timedwait(&ctx->out_event,
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

static struct rtdm_driver mpc52xx_uart_driver = {
	.profile_info		= RTDM_PROFILE_INFO(imx_uart,
						    RTDM_CLASS_SERIAL,
						    RTDM_SUBCLASS_16550A,
						    RTSER_PROFILE_VER),
	.device_count		= MPC52xx_PSC_MAXNUM,
	.device_flags		= RTDM_NAMED_DEVICE | RTDM_EXCLUSIVE,
	.context_size		= sizeof(struct rt_mpc52xx_uart_ctx),
	.ops = {
		.open		= rt_mpc52xx_uart_open,
		.close		= rt_mpc52xx_uart_close,
		.ioctl_rt	= rt_mpc52xx_uart_ioctl,
		.ioctl_nrt	= rt_mpc52xx_uart_ioctl,
		.read_rt	= rt_mpc52xx_uart_read,
		.write_rt	= rt_mpc52xx_uart_write,
	},
};

static int rt_mpc52xx_uart_of_probe(struct platform_device *op)
{
	struct rt_mpc52xx_uart_port *port;
	struct rtdm_device *dev;
	struct resource res;
	int ret, idx;

	dev_dbg(&op->dev, "mpc52xx_uart_probe(op=%p)\n", op);

	/* Check validity & presence */
	for (idx = 0; idx < MPC52xx_PSC_MAXNUM; idx++)
		if (rt_mpc52xx_uart_nodes[idx] == op->dev.of_node)
			break;
	if (idx >= MPC52xx_PSC_MAXNUM)
		return -EINVAL;

	port = kmalloc(sizeof(*port), GFP_KERNEL);
	if (!port) {
		dev_err(&op->dev, "Could allocate port space\n");
		return -ENOMEM;
	}
	port->dev = &op->dev;

	/*
	 * Set the uart clock to the input clock of the psc, the different
	 * prescalers are taken into account in the set_baudrate() methods
	 * of the respective chip
	 */
	port->uartclk = mpc5xxx_get_bus_frequency(op->dev.of_node);
	if (port->uartclk == 0) {
		dev_err(&op->dev, "Could not find uart clock frequency\n");
		ret = -EINVAL;
		goto out_kfree_port;
	}

	/* Fetch register locations */
	ret = of_address_to_resource(op->dev.of_node, 0, &res);
	if (ret) {
		dev_err(&op->dev, "Could not get resources\n");
		goto out_kfree_port;
	}
	port->num = ((res.start >> 8) & 0xf) / 2;
	if (port->num < 6)
		port->num++;

	if (!request_mem_region(res.start, resource_size(&res),
				RT_MPC52XX_UART_DRVNAM)) {
		ret = -EBUSY;
		goto out_kfree_port;
	}

	port->psc = ioremap(res.start, resource_size(&res));
	if (!port->psc) {
		dev_err(&op->dev, "Could not map PSC registers\n");
		ret = -ENOMEM;
		goto out_release_mem_region;
	}
	port->fifo = (struct mpc52xx_psc_fifo __iomem *)(port->psc + 1);

	port->irq = irq_of_parse_and_map(op->dev.of_node, 0);
	if (port->irq <= 0) {
		dev_err(&op->dev, "Could not get irq\n");
		ret = -ENODEV;
		goto out_iounmap;
	}

	dev = kmalloc(sizeof(struct rtdm_device), GFP_KERNEL);
	if (!dev) {
		dev_err(&op->dev, "Could allocate device context\n");
		ret = -ENOMEM;
		goto out_dispose_irq_mapping;
	}

	dev->driver = &mpc52xx_uart_driver;
	dev->label = "rtserPSC%d";
	dev->device_data = port;

	rt_mpc52xx_uart_init_hw(port);

	ret = rtdm_dev_register(dev);
	if (ret)
		goto out_kfree_dev;

	dev_set_drvdata(&op->dev, dev);

	dev_info(&op->dev, "%s on PSC%d at 0x%p, irq=%d, clk=%i\n",
		 dev->name, port->num, port->psc, port->irq,
		 port->uartclk);

	return 0;

out_kfree_dev:
	kfree(dev);
out_dispose_irq_mapping:
	irq_dispose_mapping(port->irq);
out_iounmap:
	iounmap(port->psc);
out_release_mem_region:
	release_mem_region(res.start, resource_size(&res));
out_kfree_port:
	kfree(port);

	return ret;
}

static int rt_mpc52xx_uart_of_remove(struct platform_device *op)
{
	struct rtdm_device *dev = dev_get_drvdata(&op->dev);
	struct rt_mpc52xx_uart_port *port = dev->device_data;
	struct resource res;

	dev_set_drvdata(&op->dev, NULL);

	rtdm_dev_unregister(dev);
	irq_dispose_mapping(port->irq);
	iounmap(port->psc);
	if (!of_address_to_resource(op->dev.of_node, 0, &res))
		release_mem_region(res.start, resource_size(&res));
	kfree(port);
	kfree(dev);

	return 0;
}

static struct of_device_id rt_mpc52xx_uart_of_match[] = {
	{ .compatible = "fsl,mpc5200b-psc-uart", },
	{ .compatible = "fsl,mpc5200-psc-uart", },
	{},
};
MODULE_DEVICE_TABLE(of, rt_mpc52xx_uart_of_match);

static struct platform_driver rt_mpc52xx_uart_of_driver = {
	.probe = rt_mpc52xx_uart_of_probe,
	.remove	=  rt_mpc52xx_uart_of_remove,
	.driver = {
		.name = "rt-mpc52xx-psc-uart",
		.owner = THIS_MODULE,
		.of_match_table = rt_mpc52xx_uart_of_match,
	},
};

static void rt_mpc52xx_uart_of_enumerate(void)
{
	struct device_node *np;
	int idx = 0;

	/* Assign index to each PSC in device tree line the linux driver does */
	for_each_matching_node(np, rt_mpc52xx_uart_of_match) {
		of_node_get(np);
		rt_mpc52xx_uart_nodes[idx] = np;
		idx++;
	}
}

static int __init rt_mpc52xx_uart_init(void)
{
	int ret;

	if (!realtime_core_enabled())
		return 0;

	printk(KERN_INFO "RTserial: MPC52xx PSC UART driver\n");

	rt_mpc52xx_uart_of_enumerate();

	ret = platform_driver_register(&rt_mpc52xx_uart_of_driver);
	if (ret) {
		printk(KERN_ERR
		       "%s; Could not register  driver (err=%d)\n",
		       __func__, ret);
		return ret;
	}

	return 0;
}

static void __exit rt_mpc52xx_uart_exit(void)
{
	if (realtime_core_enabled())
		platform_driver_unregister(&rt_mpc52xx_uart_of_driver);
}

module_init(rt_mpc52xx_uart_init);
module_exit(rt_mpc52xx_uart_exit);
