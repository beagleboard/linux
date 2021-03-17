/*
 * Copyright 2012 Wolfgang Grandegger <wg@denx.de>
 *
 * Derived from the Linux IMX UART driver (drivers/tty/serial/imx.c)
 * and 16650A RTserial driver.
 *
 * Copyright (C) 2005-2007 Jan Kiszka <jan.kiszka@web.de>.
 * Copyright (C) 2004 Pengutronix
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/tty.h>
#include <linux/string.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/serial.h>
#include <linux/console.h>
#include <linux/platform_device.h>
#include <linux/sysrq.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/rational.h>
#include <linux/io.h>
#include <asm/irq.h>
#include <asm/dma.h>
#include <asm/div64.h>
#include <linux/platform_data/serial-imx.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_device.h>

#include <rtdm/serial.h>
#include <rtdm/driver.h>

MODULE_AUTHOR("Wolfgang Grandegger <wg@denx.de>");
MODULE_DESCRIPTION("RTDM-based driver for IMX UARTs");
MODULE_VERSION("1.0.0");
MODULE_LICENSE("GPL");

#define DRIVER_NAME	"xeno_imx_uart"

/* Register definitions */
#define URXD0	0x0  /* Receiver Register */
#define URTX0	0x40 /* Transmitter Register */
#define UCR1	0x80 /* Control Register 1 */
#define UCR2	0x84 /* Control Register 2 */
#define UCR3	0x88 /* Control Register 3 */
#define UCR4	0x8c /* Control Register 4 */
#define UFCR	0x90 /* FIFO Control Register */
#define USR1	0x94 /* Status Register 1 */
#define USR2	0x98 /* Status Register 2 */
#define UESC	0x9c /* Escape Character Register */
#define UTIM	0xa0 /* Escape Timer Register */
#define UBIR	0xa4 /* BRM Incremental Register */
#define UBMR	0xa8 /* BRM Modulator Register */
#define UBRC	0xac /* Baud Rate Count Register */
#define MX2_ONEMS 0xb0 /* One Millisecond register */
#define IMX1_UTS 0xd0 /* UART Test Register on i.mx1 */
#define IMX21_UTS 0xb4 /* UART Test Register on all other i.mx*/



/* UART Control Register Bit Fields.*/
#define URXD_CHARRDY	(1<<15)
#define URXD_ERR	(1<<14)
#define URXD_OVRRUN	(1<<13)
#define URXD_FRMERR	(1<<12)
#define URXD_BRK	(1<<11)
#define URXD_PRERR	(1<<10)
#define UCR1_ADEN	(1<<15) /* Auto dectect interrupt */
#define UCR1_ADBR	(1<<14) /* Auto detect baud rate */
#define UCR1_TRDYEN	(1<<13) /* Transmitter ready interrupt enable */
#define UCR1_IDEN	(1<<12) /* Idle condition interrupt */
#define UCR1_RRDYEN	(1<<9)	/* Recv ready interrupt enable */
#define UCR1_RDMAEN	(1<<8)	/* Recv ready DMA enable */
#define UCR1_IREN	(1<<7)	/* Infrared interface enable */
#define UCR1_TXMPTYEN	(1<<6)	/* Transimitter empty interrupt enable */
#define UCR1_RTSDEN	(1<<5)	/* RTS delta interrupt enable */
#define UCR1_SNDBRK	(1<<4)	/* Send break */
#define UCR1_TDMAEN	(1<<3)	/* Transmitter ready DMA enable */
#define MX1_UCR1_UARTCLKEN	(1<<2)	/* UART clock enabled, mx1 only */
#define UCR1_DOZE	(1<<1)	/* Doze */
#define UCR1_UARTEN	(1<<0)	/* UART enabled */
#define UCR2_ESCI	(1<<15) /* Escape seq interrupt enable */
#define UCR2_IRTS	(1<<14) /* Ignore RTS pin */
#define UCR2_CTSC	(1<<13) /* CTS pin control */
#define UCR2_CTS	(1<<12) /* Clear to send */
#define UCR2_ESCEN	(1<<11) /* Escape enable */
#define UCR2_PREN	(1<<8)	/* Parity enable */
#define UCR2_PROE	(1<<7)	/* Parity odd/even */
#define UCR2_STPB	(1<<6)	/* Stop */
#define UCR2_WS		(1<<5)	/* Word size */
#define UCR2_RTSEN	(1<<4)	/* Request to send interrupt enable */
#define UCR2_ATEN	(1<<3)	/* Aging Timer Enable */
#define UCR2_TXEN	(1<<2)	/* Transmitter enabled */
#define UCR2_RXEN	(1<<1)	/* Receiver enabled */
#define UCR2_SRST	(1<<0)	/* SW reset */
#define UCR3_DTREN	(1<<13) /* DTR interrupt enable */
#define UCR3_PARERREN	(1<<12) /* Parity enable */
#define UCR3_FRAERREN	(1<<11) /* Frame error interrupt enable */
#define UCR3_DSR	(1<<10) /* Data set ready */
#define UCR3_DCD	(1<<9)	/* Data carrier detect */
#define UCR3_RI		(1<<8)	/* Ring indicator */
#define UCR3_ADNIMP	(1<<7)	/* Autobaud Detection Not Improved */
#define UCR3_RXDSEN	(1<<6)	/* Receive status interrupt enable */
#define UCR3_AIRINTEN	(1<<5)	/* Async IR wake interrupt enable */
#define UCR3_AWAKEN	(1<<4)	/* Async wake interrupt enable */
#define UCR3_DTRDEN	(1<<3)	/* Data Terminal Ready Delta Enable. */
#define MX1_UCR3_REF25		(1<<3)	/* Ref freq 25 MHz, only on mx1 */
#define MX1_UCR3_REF30		(1<<2)	/* Ref Freq 30 MHz, only on mx1 */
#define MX2_UCR3_RXDMUXSEL	(1<<2)	/* RXD Muxed Input Select, on mx2/mx3 */
#define UCR3_INVT	(1<<1)	/* Inverted Infrared transmission */
#define UCR3_BPEN	(1<<0)	/* Preset registers enable */
#define UCR4_CTSTL_SHF	10	/* CTS trigger level shift */
#define UCR4_CTSTL_MASK	0x3F	/* CTS trigger is 6 bits wide */
#define UCR4_INVR	(1<<9)	/* Inverted infrared reception */
#define UCR4_ENIRI	(1<<8)	/* Serial infrared interrupt enable */
#define UCR4_WKEN	(1<<7)	/* Wake interrupt enable */
#define UCR4_REF16	(1<<6)	/* Ref freq 16 MHz */
#define UCR4_IRSC	(1<<5)	/* IR special case */
#define UCR4_TCEN	(1<<3)	/* Transmit complete interrupt enable */
#define UCR4_BKEN	(1<<2)	/* Break condition interrupt enable */
#define UCR4_OREN	(1<<1)	/* Receiver overrun interrupt enable */
#define UCR4_DREN	(1<<0)	/* Recv data ready interrupt enable */
#define UFCR_RXTL_SHF	0	/* Receiver trigger level shift */
#define UFCR_RFDIV	(7<<7)	/* Reference freq divider mask */
#define UFCR_RFDIV_REG(x)	(((x) < 7 ? 6 - (x) : 6) << 7)
#define UFCR_TXTL_SHF	10	/* Transmitter trigger level shift */
#define UFCR_DCEDTE	(1<<6)
#define USR1_PARITYERR	(1<<15) /* Parity error interrupt flag */
#define USR1_RTSS	(1<<14) /* RTS pin status */
#define USR1_TRDY	(1<<13) /* Transmitter ready interrupt/dma flag */
#define USR1_RTSD	(1<<12) /* RTS delta */
#define USR1_ESCF	(1<<11) /* Escape seq interrupt flag */
#define USR1_FRAMERR	(1<<10) /* Frame error interrupt flag */
#define USR1_RRDY	(1<<9)	/* Receiver ready interrupt/dma flag */
#define USR1_AGTIM	(1<<8)	/* Ageing Timer Interrupt Flag */
#define USR1_DTRD	(1<<7)	/* DTR Delta */
#define USR1_RXDS	(1<<6)	/* Receiver idle interrupt flag */
#define USR1_AIRINT	(1<<5)	/* Async IR wake interrupt flag */
#define USR1_AWAKE	(1<<4)	/* Async wake interrupt flag */
#define USR2_ADET	(1<<15) /* Auto baud rate detect complete */
#define USR2_TXFE	(1<<14) /* Transmit buffer FIFO empty */
#define USR2_DTRF	(1<<13) /* DTR edge interrupt flag */
#define USR2_IDLE	(1<<12) /* Idle condition */
#define USR2_RIDELT	(1<<10) /* Ring Indicator Delta */
#define USR2_RIIN	(1<<9)	/* Ring Indicator Input */
#define USR2_IRINT	(1<<8)	/* Serial infrared interrupt flag */
#define USR2_WAKE	(1<<7)	/* Wake */
#define USR2_DCDDELT	(1<<6)	/* Data Carrier Detect Delta */
#define USR2_DCDIN	(1<<5)	/* Data Carrier Detect Input */
#define USR2_RTSF	(1<<4)	/* RTS edge interrupt flag */
#define USR2_TXDC	(1<<3)	/* Transmitter complete */
#define USR2_BRCD	(1<<2)	/* Break condition */
#define USR2_ORE	(1<<1)	/* Overrun error */
#define USR2_RDR	(1<<0)	/* Recv data ready */
#define UTS_FRCPERR	(1<<13) /* Force parity error */
#define UTS_LOOP	(1<<12) /* Loop tx and rx */
#define UTS_TXEMPTY	(1<<6)	/* TxFIFO empty */
#define UTS_RXEMPTY	(1<<5)	/* RxFIFO empty */
#define UTS_TXFULL	(1<<4)	/* TxFIFO full */
#define UTS_RXFULL	(1<<3)	/* RxFIFO full */
#define UTS_SOFTRST	(1<<0)	/* Software reset */

#define IN_BUFFER_SIZE		4096
#define OUT_BUFFER_SIZE		4096

#define TX_FIFO_SIZE		32

#define PARITY_MASK		0x03
#define DATA_BITS_MASK		0x03
#define STOP_BITS_MASK		0x01
#define FIFO_MASK		0xC0
#define EVENT_MASK		0x0F

#define IER_RX			0x01
#define IER_TX			0x02
#define IER_STAT		0x04
#define IER_MODEM		0x08

#define IMX_ISR_PASS_LIMIT	256
#define UART_CREAD_BIT		256

#define RT_IMX_UART_MAX		5

static int tx_fifo[RT_IMX_UART_MAX];
module_param_array(tx_fifo, int, NULL, 0400);
MODULE_PARM_DESC(tx_fifo, "Transmitter FIFO size");

/* i.MX21 type uart runs on all i.mx except i.MX1 and i.MX6q */
enum imx_uart_type {
	IMX1_UART,
	IMX21_UART,
	IMX53_UART,
	IMX6Q_UART,
};

/* device type dependent stuff */
struct imx_uart_data {
	unsigned int uts_reg;
	enum imx_uart_type devtype;
};


struct rt_imx_uart_port {
	unsigned char __iomem *membase;	/* read/write[bwl] */
	resource_size_t mapbase;	/* for ioremap */
	unsigned int irq;		/* irq number */
	int tx_fifo;			/* TX fifo size*/
	unsigned int have_rtscts;
	unsigned int use_dcedte;
	unsigned int use_hwflow;
	struct clk *clk_ipg;		/* clock id for UART clock */
	struct clk *clk_per;		/* clock id for UART clock */
	const struct imx_uart_data *devdata;
	unsigned int uartclk;		/* base uart clock */
	struct rtdm_device rtdm_dev;	/* RTDM device structure */
};


static struct imx_uart_data imx_uart_devdata[] = {
	[IMX1_UART] = {
		.uts_reg = IMX1_UTS,
		.devtype = IMX1_UART,
	},
	[IMX21_UART] = {
		.uts_reg = IMX21_UTS,
		.devtype = IMX21_UART,
	},
	[IMX53_UART] = {
		.uts_reg = IMX21_UTS,
		.devtype = IMX53_UART,
	},
	[IMX6Q_UART] = {
		.uts_reg = IMX21_UTS,
		.devtype = IMX6Q_UART,
	},
};

static const struct platform_device_id rt_imx_uart_id_table[] = {
	{
		.name = "imx1-uart",
		.driver_data = (kernel_ulong_t) &imx_uart_devdata[IMX1_UART],
	}, {
		.name = "imx21-uart",
		.driver_data = (kernel_ulong_t) &imx_uart_devdata[IMX21_UART],
	}, {
		.name = "imx53-uart",
		.driver_data = (kernel_ulong_t) &imx_uart_devdata[IMX53_UART],
	}, {
		.name = "imx6q-uart",
		.driver_data = (kernel_ulong_t) &imx_uart_devdata[IMX6Q_UART],
	}, {
		/* sentinel */
	}
};
MODULE_DEVICE_TABLE(platform, rt_imx_uart_id_table);

static const struct of_device_id rt_imx_uart_dt_ids[] = {
	{
		.compatible = "fsl,imx6q-uart",
		.data = &imx_uart_devdata[IMX6Q_UART], },
	{
		.compatible = "fsl,imx53-uart",
		.data = &imx_uart_devdata[IMX53_UART], },
	{
		.compatible = "fsl,imx1-uart",
		.data = &imx_uart_devdata[IMX1_UART], },
	{
		.compatible = "fsl,imx21-uart",
		.data = &imx_uart_devdata[IMX21_UART], },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, rt_imx_uart_dt_ids);

struct rt_imx_uart_ctx {
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

	int ier_status;			/* IER cache */
	int mcr_status;			/* MCR cache */
	int status;			/* cache for LSR + soft-states */
	int saved_errors;		/* error cache for RTIOC_GET_STATUS */

	/*
	 * The port structure holds all the information about the UART
	 * port like base address, and so on.
	 */
	struct rt_imx_uart_port *port;
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
};

static void rt_imx_uart_stop_tx(struct rt_imx_uart_ctx *ctx)
{
	unsigned long temp;

	temp = readl(ctx->port->membase + UCR1);
	writel(temp & ~UCR1_TXMPTYEN, ctx->port->membase + UCR1);
}

static void rt_imx_uart_start_tx(struct rt_imx_uart_ctx *ctx)
{
	unsigned long temp;

	temp = readl(ctx->port->membase + UCR1);
	writel(temp | UCR1_TXMPTYEN, ctx->port->membase + UCR1);
}

static void rt_imx_uart_enable_ms(struct rt_imx_uart_ctx *ctx)
{
	unsigned long ucr3;

	/*
	 * RTS interrupt is enabled only if we are using interrupt-driven
	 * software controlled hardware flow control
	 */
	if (!ctx->port->use_hwflow) {
		unsigned long ucr1 = readl(ctx->port->membase + UCR1);

		ucr1 |= UCR1_RTSDEN;
		writel(ucr1, ctx->port->membase + UCR1);
	}
	ucr3 = readl(ctx->port->membase + UCR3);
	ucr3 |= UCR3_DTREN;
	if (ctx->port->use_dcedte) /* DTE mode */
		ucr3 |= UCR3_DCD | UCR3_RI;
	writel(ucr3, ctx->port->membase + UCR3);
}

static int rt_imx_uart_rx_chars(struct rt_imx_uart_ctx *ctx,
				uint64_t *timestamp)
{
	unsigned int rx, temp;
	int rbytes = 0;
	int lsr = 0;

	while (readl(ctx->port->membase + USR2) & USR2_RDR) {
		rx = readl(ctx->port->membase + URXD0);
		temp = readl(ctx->port->membase + USR2);
		if (temp & USR2_BRCD) {
			writel(USR2_BRCD, ctx->port->membase + USR2);
			lsr |= RTSER_LSR_BREAK_IND;
		}

		if (rx & (URXD_PRERR | URXD_OVRRUN | URXD_FRMERR)) {
			if (rx & URXD_PRERR)
				lsr |= RTSER_LSR_PARITY_ERR;
			else if (rx & URXD_FRMERR)
				lsr |= RTSER_LSR_FRAMING_ERR;
			if (rx & URXD_OVRRUN)
				lsr |= RTSER_LSR_OVERRUN_ERR;
		}

		/* save received character */
		ctx->in_buf[ctx->in_tail] = rx & 0xff;
		if (ctx->in_history)
			ctx->in_history[ctx->in_tail] = *timestamp;
		ctx->in_tail = (ctx->in_tail + 1) & (IN_BUFFER_SIZE - 1);

		if (unlikely(ctx->in_npend >= IN_BUFFER_SIZE))
			lsr |= RTSER_SOFT_OVERRUN_ERR;
		else
			ctx->in_npend++;

		rbytes++;
	}

	/* save new errors */
	ctx->status |= lsr;

	return rbytes;
}

static void rt_imx_uart_tx_chars(struct rt_imx_uart_ctx *ctx)
{
	int ch;
	unsigned int uts_reg = ctx->port->devdata->uts_reg;

	while (ctx->out_npend > 0 &&
	       !(readl(ctx->port->membase + uts_reg) & UTS_TXFULL)) {
		ch = ctx->out_buf[ctx->out_head++];
		writel(ch, ctx->port->membase + URTX0);
		ctx->out_head &= (OUT_BUFFER_SIZE - 1);
		ctx->out_npend--;
	}
}

static int rt_imx_uart_modem_status(struct rt_imx_uart_ctx *ctx,
				     unsigned int usr1,
				     unsigned int usr2)
{
	int events = 0;

	/* Clear the status bits that triggered the interrupt */
	writel(usr1, ctx->port->membase + USR1);
	writel(usr2, ctx->port->membase + USR2);

	if (ctx->port->use_dcedte) { /* DTE mode */
		if (usr2 & USR2_DCDDELT)
			events |= !(usr2 & USR2_DCDIN) ?
				RTSER_EVENT_MODEMHI : RTSER_EVENT_MODEMLO;
	}
	if (!ctx->port->use_hwflow && (usr1 & USR1_RTSD)) {
		events |= (usr1 & USR1_RTSS) ?
			RTSER_EVENT_MODEMHI : RTSER_EVENT_MODEMLO;
	}

	return events;
}

static int rt_imx_uart_int(rtdm_irq_t *irq_context)
{
	uint64_t timestamp = rtdm_clock_read();
	struct rt_imx_uart_ctx *ctx;
	unsigned int usr1, usr2, ucr1;
	int rbytes = 0, events = 0;
	int ret = RTDM_IRQ_NONE;

	ctx = rtdm_irq_get_arg(irq_context, struct rt_imx_uart_ctx);

	rtdm_lock_get(&ctx->lock);

	usr1 = readl(ctx->port->membase + USR1);
	usr2 = readl(ctx->port->membase + USR2);
	ucr1 = readl(ctx->port->membase + UCR1);

	/*
	 * Read if there is data available
	 */
	if (usr1 & USR1_RRDY) {
		if (likely(ucr1 & UCR1_RRDYEN)) {
			rbytes = rt_imx_uart_rx_chars(ctx, &timestamp);
			events |= RTSER_EVENT_RXPEND;
		}
		ret = RTDM_IRQ_HANDLED;
	}

	/*
	 * Send data if there is data to be sent
	 */
	if (usr1 & USR1_TRDY) {
		if (likely(ucr1 & UCR1_TXMPTYEN))
			rt_imx_uart_tx_chars(ctx);
		ret = RTDM_IRQ_HANDLED;
	}

	/*
	 * Handle modem status events
	 */
	if ((usr1 & (USR1_RTSD | USR1_DTRD)) ||
	    (usr2 & (USR2_DCDDELT | USR2_RIDELT))) {
		events |= rt_imx_uart_modem_status(ctx, usr1, usr2);
		ret = RTDM_IRQ_HANDLED;
	}

	if (ctx->in_nwait > 0) {
		if ((ctx->in_nwait <= rbytes) || ctx->status) {
			ctx->in_nwait = 0;
			rtdm_event_signal(&ctx->in_event);
		} else {
			ctx->in_nwait -= rbytes;
		}
	}

	if (ctx->status) {
		events |= RTSER_EVENT_ERRPEND;
#ifdef FIXME
		ctx->ier_status &= ~IER_STAT;
#endif
	}

	if (events & ctx->config.event_mask) {
		int old_events = ctx->ioc_events;

		ctx->last_timestamp = timestamp;
		ctx->ioc_events = events;

		if (!old_events)
			rtdm_event_signal(&ctx->ioc_event);
	}

	if ((ctx->ier_status & IER_TX) && (ctx->out_npend == 0)) {
		rt_imx_uart_stop_tx(ctx);
		ctx->ier_status &= ~IER_TX;
		rtdm_event_signal(&ctx->out_event);
	}

	rtdm_lock_put(&ctx->lock);

	if (ret != RTDM_IRQ_HANDLED)
		pr_warn("%s: unhandled interrupt\n", __func__);
	return ret;
}

static unsigned int rt_imx_uart_get_msr(struct rt_imx_uart_ctx *ctx)
{
	unsigned long usr1 = readl(ctx->port->membase + USR1);
	unsigned long usr2 = readl(ctx->port->membase + USR2);
	unsigned int msr = 0;

	if (usr1 & USR1_RTSD)
		msr |= RTSER_MSR_DCTS;
	if (usr1 & USR1_DTRD)
		msr |= RTSER_MSR_DDSR;
	if (usr2 & USR2_RIDELT)
		msr |= RTSER_MSR_TERI;
	if (usr2 & USR2_DCDDELT)
		msr |= RTSER_MSR_DDCD;

	if (usr1 & USR1_RTSS)
		msr |= RTSER_MSR_CTS;

	if (ctx->port->use_dcedte) { /* DTE mode */
		if (!(usr2 & USR2_DCDIN))
			msr |= RTSER_MSR_DCD;
		if (!(usr2 & USR2_RIIN))
			msr |= RTSER_MSR_RI;
	}

	return msr;
}

static void rt_imx_uart_set_mcr(struct rt_imx_uart_ctx *ctx,
				unsigned int mcr)
{
	unsigned int uts_reg = ctx->port->devdata->uts_reg;
	unsigned long ucr2 = readl(ctx->port->membase + UCR2);
	unsigned long ucr3 = readl(ctx->port->membase + UCR3);
	unsigned long uts = readl(ctx->port->membase + uts_reg);

	if (mcr & RTSER_MCR_RTS) {
		/*
		 * Return to hardware-driven hardware flow control if the
		 * option is enabled
		 */
		if (ctx->port->use_hwflow) {
			ucr2 |= UCR2_CTSC;
		} else {
			ucr2 |= UCR2_CTS;
			ucr2 &= ~UCR2_CTSC;
		}
	} else {
		ucr2 &= ~(UCR2_CTS | UCR2_CTSC);
	}
	writel(ucr2, ctx->port->membase + UCR2);

	if (mcr & RTSER_MCR_DTR)
		ucr3 |= UCR3_DSR;
	else
		ucr3 &= ~UCR3_DSR;
	writel(ucr3, ctx->port->membase + UCR3);

	if (mcr & RTSER_MCR_LOOP)
		uts |= UTS_LOOP;
	else
		uts &= ~UTS_LOOP;
	writel(uts, ctx->port->membase + uts_reg);
}

static void rt_imx_uart_break_ctl(struct rt_imx_uart_ctx *ctx,
				  int break_state)
{
	unsigned long ucr1 = readl(ctx->port->membase + UCR1);

	if (break_state == RTSER_BREAK_SET)
		ucr1 |= UCR1_SNDBRK;
	else
		ucr1 &= ~UCR1_SNDBRK;
	writel(ucr1, ctx->port->membase + UCR1);
}

static int rt_imx_uart_set_config(struct rt_imx_uart_ctx *ctx,
				  const struct rtser_config *config,
				  uint64_t **in_history_ptr)
{
	rtdm_lockctx_t lock_ctx;
	int err = 0;

	rtdm_lock_get_irqsave(&ctx->lock, lock_ctx);

	if (config->config_mask & RTSER_SET_BAUD)
		ctx->config.baud_rate = config->baud_rate;
	if (config->config_mask & RTSER_SET_DATA_BITS)
		ctx->config.data_bits = config->data_bits & DATA_BITS_MASK;
	if (config->config_mask & RTSER_SET_PARITY)
		ctx->config.parity = config->parity & PARITY_MASK;
	if (config->config_mask & RTSER_SET_STOP_BITS)
		ctx->config.stop_bits = config->stop_bits & STOP_BITS_MASK;

	/* Timeout manipulation is not atomic. The user is supposed to take
	 * care not to use and change timeouts at the same time.
	 */
	if (config->config_mask & RTSER_SET_TIMEOUT_RX)
		ctx->config.rx_timeout = config->rx_timeout;
	if (config->config_mask & RTSER_SET_TIMEOUT_TX)
		ctx->config.tx_timeout = config->tx_timeout;
	if (config->config_mask & RTSER_SET_TIMEOUT_EVENT)
		ctx->config.event_timeout = config->event_timeout;

	if (config->config_mask & RTSER_SET_TIMESTAMP_HISTORY) {
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
	}

	if (config->config_mask & RTSER_SET_EVENT_MASK) {
		ctx->config.event_mask = config->event_mask & EVENT_MASK;
		ctx->ioc_events = 0;

		if ((config->event_mask & RTSER_EVENT_RXPEND) &&
		    (ctx->in_npend > 0))
			ctx->ioc_events |= RTSER_EVENT_RXPEND;

		if ((config->event_mask & RTSER_EVENT_ERRPEND)
		    && ctx->status)
			ctx->ioc_events |= RTSER_EVENT_ERRPEND;
	}

	if (config->config_mask & RTSER_SET_HANDSHAKE) {
		ctx->config.handshake = config->handshake;

		switch (ctx->config.handshake) {
		case RTSER_RTSCTS_HAND:
			/* ...? */

		default:	/* RTSER_NO_HAND */
			ctx->mcr_status = RTSER_MCR_RTS | RTSER_MCR_OUT1;
			break;
		}
		rt_imx_uart_set_mcr(ctx, ctx->mcr_status);
	}

	/* configure hardware with new parameters */
	if (config->config_mask & (RTSER_SET_BAUD |
				   RTSER_SET_PARITY |
				   RTSER_SET_DATA_BITS |
				   RTSER_SET_STOP_BITS |
				   RTSER_SET_EVENT_MASK |
				   RTSER_SET_HANDSHAKE)) {
		struct rt_imx_uart_port *port = ctx->port;
		unsigned int ucr2, old_ucr1, old_txrxen, old_ucr2;
		unsigned int baud = ctx->config.baud_rate;
		unsigned int div, ufcr;
		unsigned long num, denom;
		uint64_t tdiv64;

		if (ctx->config.data_bits == RTSER_8_BITS)
			ucr2 = UCR2_WS | UCR2_SRST | UCR2_IRTS;
		else
			ucr2 = UCR2_SRST | UCR2_IRTS;

		if (ctx->config.handshake == RTSER_RTSCTS_HAND) {
			if (port->have_rtscts) {
				ucr2 &= ~UCR2_IRTS;
				ucr2 |= UCR2_CTSC;
			}
		}

		if (ctx->config.stop_bits == RTSER_2_STOPB)
			ucr2 |= UCR2_STPB;
		if (ctx->config.parity == RTSER_ODD_PARITY ||
		    ctx->config.parity == RTSER_EVEN_PARITY) {
			ucr2 |= UCR2_PREN;
			if (ctx->config.parity == RTSER_ODD_PARITY)
				ucr2 |= UCR2_PROE;
		}

		/*
		 * disable interrupts and drain transmitter
		 */
		old_ucr1 = readl(port->membase + UCR1);
		old_ucr1 &= ~UCR1_RTSDEN; /* reset in  rt_imx_uart_enable_ms()*/
		writel(old_ucr1 & ~(UCR1_TXMPTYEN | UCR1_RRDYEN),
		       port->membase + UCR1);
		old_ucr2 = readl(port->membase + USR2);
		writel(old_ucr2 & ~UCR2_ATEN, port->membase + USR2);
		while (!(readl(port->membase + USR2) & USR2_TXDC))
			barrier();

		/* then, disable everything */
		old_txrxen = readl(port->membase + UCR2);
		writel(old_txrxen & ~(UCR2_TXEN | UCR2_RXEN),
		       port->membase + UCR2);
		old_txrxen &= (UCR2_TXEN | UCR2_RXEN);
		div = port->uartclk / (baud * 16);
		if (div > 7)
			div = 7;
		if (!div)
			div = 1;

		rational_best_approximation(16 * div * baud, port->uartclk,
					    1 << 16, 1 << 16, &num, &denom);

		tdiv64 = port->uartclk;
		tdiv64 *= num;
		do_div(tdiv64, denom * 16 * div);

		num -= 1;
		denom -= 1;

		ufcr = readl(port->membase + UFCR);
		ufcr = (ufcr & (~UFCR_RFDIV)) | UFCR_RFDIV_REG(div);

		if (port->use_dcedte)
			ufcr |= UFCR_DCEDTE;

		writel(ufcr, port->membase + UFCR);

		writel(num, port->membase + UBIR);
		writel(denom, port->membase + UBMR);

		writel(port->uartclk / div / 1000, port->membase + MX2_ONEMS);

		writel(old_ucr1, port->membase + UCR1);

		/* set the parity, stop bits and data size */
		writel(ucr2 | old_txrxen, port->membase + UCR2);

		if (config->event_mask &
		    (RTSER_EVENT_MODEMHI | RTSER_EVENT_MODEMLO))
			rt_imx_uart_enable_ms(ctx);

		ctx->status = 0;
		ctx->ioc_events &= ~RTSER_EVENT_ERRPEND;
	}

	rtdm_lock_put_irqrestore(&ctx->lock, lock_ctx);

	return err;
}

void rt_imx_uart_cleanup_ctx(struct rt_imx_uart_ctx *ctx)
{
	rtdm_event_destroy(&ctx->in_event);
	rtdm_event_destroy(&ctx->out_event);
	rtdm_event_destroy(&ctx->ioc_event);
	rtdm_mutex_destroy(&ctx->out_lock);
}

#define TXTL 2 /* reset default */
#define RXTL 1 /* reset default */

static int rt_imx_uart_setup_ufcr(struct rt_imx_uart_port *port)
{
	unsigned int val;
	unsigned int ufcr_rfdiv;

	/* set receiver / transmitter trigger level.
	 * RFDIV is set such way to satisfy requested uartclk value
	 */
	val = TXTL << 10 | RXTL;
	ufcr_rfdiv = (clk_get_rate(port->clk_per) + port->uartclk / 2) /
		port->uartclk;

	if (!ufcr_rfdiv)
		ufcr_rfdiv = 1;

	val |= UFCR_RFDIV_REG(ufcr_rfdiv);

	writel(val, port->membase + UFCR);

	return 0;
}

/* half the RX buffer size */
#define CTSTL 16

static void uart_reset(struct rt_imx_uart_port *port)
{
	unsigned int uts_reg = port->devdata->uts_reg;
	int n = 100;
	u32 temp;

	/* Reset fifo's and state machines */
	temp = readl(port->membase + UCR2);
	temp &= ~UCR2_SRST;
	writel(temp, port->membase + UCR2);
	n = 100;
	while (!(readl(port->membase + uts_reg) & UTS_SOFTRST) && --n > 0)
		udelay(1);
}

static int rt_imx_uart_open(struct rtdm_fd *fd, int oflags)
{
	struct rt_imx_uart_ctx *ctx;
	struct rt_imx_uart_port *port;
	rtdm_lockctx_t lock_ctx;
	unsigned long temp;
	uint64_t *dummy;

	ctx = rtdm_fd_to_private(fd);
	ctx->port = (struct rt_imx_uart_port *)rtdm_fd_device(fd)->device_data;

	port = ctx->port;

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

	/*
	 * disable the DREN bit (Data Ready interrupt enable) before
	 * requesting IRQs
	 */
	temp = readl(port->membase + UCR4);

	/* set the trigger level for CTS */
	temp &= ~(UCR4_CTSTL_MASK << UCR4_CTSTL_SHF);
	temp |= CTSTL << UCR4_CTSTL_SHF;
	writel(temp & ~UCR4_DREN, port->membase + UCR4);

	uart_reset(port);

	rtdm_lock_get_irqsave(&ctx->lock, lock_ctx);

	/*
	 * Finally, clear status and enable interrupts
	 */
	writel(USR1_RTSD | USR1_DTRD, port->membase + USR1);
	writel(USR2_ORE, port->membase + USR2);

	temp = readl(port->membase + UCR1) & ~UCR1_RRDYEN;
	temp |= UCR1_UARTEN;
	if (port->have_rtscts)
		temp |= UCR1_RTSDEN;
	writel(temp, port->membase + UCR1);

	temp = readl(port->membase + UCR4);
	temp |= UCR4_OREN;
	writel(temp, port->membase + UCR4);

	temp = readl(port->membase + UCR2) & ~(UCR2_ATEN|UCR2_RTSEN);
	temp |= (UCR2_RXEN | UCR2_TXEN);
	if (!port->have_rtscts)
		temp |= UCR2_IRTS;
	writel(temp, port->membase + UCR2);

	temp = readl(port->membase + UCR3);
	temp |= MX2_UCR3_RXDMUXSEL;
	writel(temp, port->membase + UCR3);

	temp = readl(port->membase + UCR1);
	temp |= UCR1_RRDYEN;
	writel(temp, port->membase + UCR1);

	temp = readl(port->membase + UCR2);
	temp |= UCR2_ATEN;
	writel(temp, port->membase + UCR2);

	rtdm_lock_put_irqrestore(&ctx->lock, lock_ctx);

	rt_imx_uart_set_config(ctx, &default_config, &dummy);

	rt_imx_uart_setup_ufcr(port);

	return rtdm_irq_request(&ctx->irq_handle,
				port->irq, rt_imx_uart_int, 0,
				rtdm_fd_device(fd)->name, ctx);
}

void rt_imx_uart_close(struct rtdm_fd *fd)
{
	struct rt_imx_uart_port *port;
	struct rt_imx_uart_ctx *ctx;
	rtdm_lockctx_t lock_ctx;
	unsigned long temp;

	ctx = rtdm_fd_to_private(fd);
	port = ctx->port;

	rtdm_lock_get_irqsave(&ctx->lock, lock_ctx);

	temp = readl(port->membase + UCR2);
	temp &= ~(UCR2_ATEN|UCR2_RTSEN|UCR2_RXEN|UCR2_TXEN|UCR2_IRTS);
	writel(temp, port->membase + UCR2);
	/*
	 * Disable all interrupts, port and break condition, then
	 * reset.
	 */
	temp = readl(port->membase + UCR1);
	temp &= ~(UCR1_TXMPTYEN | UCR1_RRDYEN | UCR1_RTSDEN | UCR1_UARTEN);
	writel(temp, port->membase + UCR1);

	rtdm_lock_put_irqrestore(&ctx->lock, lock_ctx);

	rtdm_irq_free(&ctx->irq_handle);

	uart_reset(port);

	rt_imx_uart_cleanup_ctx(ctx);
	kfree(ctx->in_history);
}

static int rt_imx_uart_ioctl(struct rtdm_fd *fd,
			     unsigned int request, void *arg)
{
	rtdm_lockctx_t lock_ctx;
	struct rt_imx_uart_ctx *ctx;
	int err = 0;

	ctx = rtdm_fd_to_private(fd);

	switch (request) {
	case RTSER_RTIOC_GET_CONFIG:
		if (rtdm_fd_is_user(fd))
			err =
			    rtdm_safe_copy_to_user(fd, arg,
						   &ctx->config,
						   sizeof(struct rtser_config));
		else
			memcpy(arg, &ctx->config,
			       sizeof(struct rtser_config));
		break;

	case RTSER_RTIOC_SET_CONFIG: {
		struct rtser_config *config;
		struct rtser_config config_buf;
		uint64_t *hist_buf = NULL;

		/*
		 * We may call regular kernel services ahead, ask for
		 * re-entering secondary mode if need be.
		 */
		if (rtdm_in_rt_context())
			return -ENOSYS;

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
		    (config->baud_rate > clk_get_rate(ctx->port->clk_per) / 16 ||
		     config->baud_rate <= 0))
			/* invalid baudrate for this port */
			return -EINVAL;

		if (config->config_mask & RTSER_SET_TIMESTAMP_HISTORY) {
			if (config->timestamp_history &
						RTSER_RX_TIMESTAMP_HISTORY)
				hist_buf = kmalloc(IN_BUFFER_SIZE *
						   sizeof(nanosecs_abs_t),
						   GFP_KERNEL);
		}

		rt_imx_uart_set_config(ctx, config, &hist_buf);

		if (hist_buf)
			kfree(hist_buf);
		break;
	}

	case RTSER_RTIOC_GET_STATUS: {
		int status, msr;

		rtdm_lock_get_irqsave(&ctx->lock, lock_ctx);

		status = ctx->saved_errors | ctx->status;
		ctx->status = 0;
		ctx->saved_errors = 0;
		ctx->ioc_events &= ~RTSER_EVENT_ERRPEND;

		msr = rt_imx_uart_get_msr(ctx);

		rtdm_lock_put_irqrestore(&ctx->lock, lock_ctx);

		if (rtdm_fd_is_user(fd)) {
			struct rtser_status status_buf;


			status_buf.line_status = status;
			status_buf.modem_status = msr;
			err =
			    rtdm_safe_copy_to_user(fd, arg,
						   &status_buf,
						   sizeof(struct
							  rtser_status));
		} else {
			((struct rtser_status *)arg)->line_status = 0;
			((struct rtser_status *)arg)->modem_status = msr;
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
		rt_imx_uart_set_mcr(ctx, new_mcr);
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
			 * when the user waits for it.
			 */
			if (ctx->config.event_mask & RTSER_EVENT_ERRPEND) {
				ctx->ier_status |= IER_STAT;
#ifdef FIXME
				rt_imx_uart_reg_out(mode, base, IER,
						 ctx->ier_status);
#endif
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
		rtdm_lock_get_irqsave(&ctx->lock, lock_ctx);
		rt_imx_uart_break_ctl(ctx, (int)arg);
		rtdm_lock_put_irqrestore(&ctx->lock, lock_ctx);
		break;
	}

#ifdef FIXME
	case RTIOC_PURGE: {
		int fcr = 0;

		rtdm_lock_get_irqsave(&ctx->lock, lock_ctx);
		if ((long)arg & RTDM_PURGE_RX_BUFFER) {
			ctx->in_head = 0;
			ctx->in_tail = 0;
			ctx->in_npend = 0;
			ctx->status = 0;
			fcr |= FCR_FIFO | FCR_RESET_RX;
			rt_imx_uart_reg_in(mode, base, RHR);
		}
		if ((long)arg & RTDM_PURGE_TX_BUFFER) {
			ctx->out_head = 0;
			ctx->out_tail = 0;
			ctx->out_npend = 0;
			fcr |= FCR_FIFO | FCR_RESET_TX;
		}
		if (fcr) {
			rt_imx_uart_reg_out(mode, base, FCR, fcr);
			rt_imx_uart_reg_out(mode, base, FCR,
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

ssize_t rt_imx_uart_read(struct rtdm_fd *fd, void *buf, size_t nbyte)
{
	struct rt_imx_uart_ctx *ctx;
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
				 * separately.
				 */
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
			ctx->in_npend -= block;
			if (ctx->in_npend == 0)
				ctx->ioc_events &= ~RTSER_EVENT_RXPEND;

			if (nbyte == 0)
				break; /* All requested bytes read. */

			continue;
		}

		if (nonblocking)
			/* ret was set to EAGAIN in case of a real
			 * non-blocking call or contains the error
			 * returned by rtdm_event_wait[_until]
			 */
			break;

		ctx->in_nwait = nbyte;

		rtdm_lock_put_irqrestore(&ctx->lock, lock_ctx);

		ret = rtdm_event_timedwait(&ctx->in_event,
					   ctx->config.rx_timeout,
					   &timeout_seq);
		if (ret < 0) {
			if (ret == -EIDRM) {
				/* Device has been closed -
				 * return immediately.
				 */
				return -EBADF;
			}

			rtdm_lock_get_irqsave(&ctx->lock, lock_ctx);

			nonblocking = 1;
			if (ctx->in_npend > 0) {
				/* Final turn: collect pending bytes
				 * before exit.
				 */
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
			   (ret == -ETIMEDOUT)))
		ret = read;

	return ret;
}

static ssize_t rt_imx_uart_write(struct rtdm_fd *fd, const void *buf,
				size_t nbyte)
{
	struct rt_imx_uart_ctx *ctx;
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
				 * end separately.
				 */
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

			ctx->ier_status |= IER_TX;
			rt_imx_uart_start_tx(ctx);

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
				 * return immediately.
				 */
				ret = -EBADF;
			}
			break;
		}
	}

	rtdm_mutex_unlock(&ctx->out_lock);

	if ((written > 0) && ((ret == 0) || (ret == -EAGAIN) ||
			      (ret == -ETIMEDOUT)))
		ret = written;

	return ret;
}

static struct rtdm_driver imx_uart_driver = {
	.profile_info		= RTDM_PROFILE_INFO(imx_uart,
						    RTDM_CLASS_SERIAL,
						    RTDM_SUBCLASS_16550A,
						    RTSER_PROFILE_VER),
	.device_count		= RT_IMX_UART_MAX,
	.device_flags		= RTDM_NAMED_DEVICE | RTDM_EXCLUSIVE,
	.context_size		= sizeof(struct rt_imx_uart_ctx),
	.ops = {
		.open		= rt_imx_uart_open,
		.close		= rt_imx_uart_close,
		.ioctl_rt	= rt_imx_uart_ioctl,
		.ioctl_nrt	= rt_imx_uart_ioctl,
		.read_rt	= rt_imx_uart_read,
		.write_rt	= rt_imx_uart_write,
	},
};


#ifdef CONFIG_OF

/*
 * This function returns 1 iff pdev isn't a device instatiated by dt, 0 iff it
 * could successfully get all information from dt or a negative errno.
 */
static int rt_imx_uart_probe_dt(struct rt_imx_uart_port *port,
				struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	const struct of_device_id *of_id =
			of_match_device(rt_imx_uart_dt_ids, &pdev->dev);
	int ret;

	if (!np)
		/* no device tree device */
		return 1;

	ret = of_alias_get_id(np, "serial");
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to get alias id, errno %d\n", ret);
		return ret;
	}

	pdev->id = ret;

	if (of_get_property(np, "uart-has-rtscts", NULL) ||
	    of_get_property(np, "fsl,uart-has-rtscts", NULL) /* deprecated */)
		port->have_rtscts = 1;
	if (of_get_property(np, "fsl,irda-mode", NULL))
		dev_warn(&pdev->dev, "IRDA not yet supported\n");

	if (of_get_property(np, "fsl,dte-mode", NULL))
		port->use_dcedte = 1;

	port->devdata = of_id->data;

	return 0;
}
#else
static inline int rt_imx_uart_probe_dt(struct rt_imx_uart_port *port,
				       struct platform_device *pdev)
{
	return 1;
}
#endif

static void rt_imx_uart_probe_pdata(struct rt_imx_uart_port *port,
				    struct platform_device *pdev)
{
	struct imxuart_platform_data *pdata = dev_get_platdata(&pdev->dev);

	port->devdata = (struct imx_uart_data  *) pdev->id_entry->driver_data;

	if (!pdata)
		return;

	if (pdata->flags & IMXUART_HAVE_RTSCTS)
		port->have_rtscts = 1;
}

static int rt_imx_uart_probe(struct platform_device *pdev)
{
	struct rtdm_device *dev;
	struct rt_imx_uart_port *port;
	struct resource *res;
	int ret;

	port = devm_kzalloc(&pdev->dev, sizeof(*port), GFP_KERNEL);
	if (!port)
		return -ENOMEM;

	ret = rt_imx_uart_probe_dt(port, pdev);
	if (ret > 0)
		rt_imx_uart_probe_pdata(port, pdev);
	else if (ret < 0)
		return ret;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	port->irq = platform_get_irq(pdev, 0);

	if (port->irq <= 0)
		return -ENODEV;

	port->membase = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(port->membase))
		return PTR_ERR(port->membase);

	dev = &port->rtdm_dev;
	dev->driver = &imx_uart_driver;
	dev->label = "rtser%d";
	dev->device_data = port;

	if (!tx_fifo[pdev->id] || tx_fifo[pdev->id] > TX_FIFO_SIZE)
		port->tx_fifo = TX_FIFO_SIZE;
	else
		port->tx_fifo = tx_fifo[pdev->id];

	port->clk_ipg = devm_clk_get(&pdev->dev, "ipg");
	if (IS_ERR(port->clk_ipg))
		return PTR_ERR(port->clk_ipg);

	port->clk_per = devm_clk_get(&pdev->dev, "per");
	if (IS_ERR(port->clk_per))
		return PTR_ERR(port->clk_per);

	clk_prepare_enable(port->clk_ipg);
	clk_prepare_enable(port->clk_per);
	port->uartclk = clk_get_rate(port->clk_per);

	port->use_hwflow = 1;

	ret = rtdm_dev_register(dev);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, port);

	pr_info("%s on IMX UART%d: membase=0x%p irq=%d uartclk=%d\n",
	       dev->name, pdev->id, port->membase, port->irq, port->uartclk);
	return 0;
}

static int rt_imx_uart_remove(struct platform_device *pdev)
{
	struct imxuart_platform_data *pdata;
	struct rt_imx_uart_port *port = platform_get_drvdata(pdev);
	struct rtdm_device *dev = &port->rtdm_dev;

	pdata = pdev->dev.platform_data;
	platform_set_drvdata(pdev, NULL);

	clk_disable_unprepare(port->clk_ipg);
	clk_disable_unprepare(port->clk_per);
	rtdm_dev_unregister(dev);

	return 0;
}

static struct platform_driver rt_imx_uart_driver = {
	.probe = rt_imx_uart_probe,
	.remove	= rt_imx_uart_remove,
	.id_table = rt_imx_uart_id_table,
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = rt_imx_uart_dt_ids,
	},
	.prevent_deferred_probe = true,
};


static int __init rt_imx_uart_init(void)
{
	int ret;

	if (!rtdm_available())
		return -ENODEV;

	ret = platform_driver_register(&rt_imx_uart_driver);
	if (ret) {
		pr_err("%s; Could not register  driver (err=%d)\n",
			__func__, ret);
	}

	return ret;
}

static void __exit rt_imx_uart_exit(void)
{
	platform_driver_unregister(&rt_imx_uart_driver);
}

module_init(rt_imx_uart_init);
module_exit(rt_imx_uart_exit);
