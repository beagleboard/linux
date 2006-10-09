/*
 * linux/drivers/i2c/i2c-omap.c
 *
 * TI OMAP I2C master mode driver
 *
 * Copyright (C) 2003 MontaVista Software, Inc.
 * Copyright (C) 2004 Texas Instruments.
 *
 * Updated to work with multiple I2C interfaces on 24xx by
 * Tony Lindgren <tony@atomide.com> and Imre Deak <imre.deak@nokia.com>
 * Copyright (C) 2005 Nokia Corporation
 *
 * Cleaned up by Juha Yrjölä <juha.yrjola@nokia.com>
 *
 * ----------------------------------------------------------------------------
 * This file was highly leveraged from i2c-elektor.c:
 *
 * Copyright 1995-97 Simon G. Vogl
 *           1998-99 Hans Berglund
 *
 * With some changes from Kyösti Mälkki <kmalkki@cc.hut.fi> and even
 * Frodo Looijaard <frodol@dds.nl>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

// #define DEBUG

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/completion.h>
#include <linux/platform_device.h>
#include <linux/clk.h>

#include <asm/io.h>

/* ----- global defines ----------------------------------------------- */
static const char driver_name[] = "i2c_omap";

#define MODULE_NAME "OMAP I2C"
#define OMAP_I2C_TIMEOUT (msecs_to_jiffies(1000)) /* timeout waiting for the controller to respond */

#define DEFAULT_OWN		1	/* default own I2C address */
#define MAX_MESSAGES		65536	/* max number of messages */

#define OMAP_I2C_REV_REG		0x00
#define OMAP_I2C_IE_REG			0x04
#define OMAP_I2C_STAT_REG		0x08
#define OMAP_I2C_IV_REG			0x0c
#define OMAP_I2C_SYSS_REG		0x10
#define OMAP_I2C_BUF_REG		0x14
#define OMAP_I2C_CNT_REG		0x18
#define OMAP_I2C_DATA_REG		0x1c
#define OMAP_I2C_SYSC_REG		0x20
#define OMAP_I2C_CON_REG		0x24
#define OMAP_I2C_OA_REG			0x28
#define OMAP_I2C_SA_REG			0x2c
#define OMAP_I2C_PSC_REG		0x30
#define OMAP_I2C_SCLL_REG		0x34
#define OMAP_I2C_SCLH_REG		0x38
#define OMAP_I2C_SYSTEST_REG		0x3c

/* I2C Interrupt Enable Register (OMAP_I2C_IE): */
#define OMAP_I2C_IE_XRDY	(1 << 4)	/* TX data ready int enable */
#define OMAP_I2C_IE_RRDY	(1 << 3)	/* RX data ready int enable */
#define OMAP_I2C_IE_ARDY	(1 << 2)	/* Access ready int enable */
#define OMAP_I2C_IE_NACK	(1 << 1)	/* No ack interrupt enable */
#define OMAP_I2C_IE_AL		(1 << 0)	/* Arbitration lost int ena */

/* I2C Status Register (OMAP_I2C_STAT): */
#define OMAP_I2C_STAT_SBD	(1 << 15)	/* Single byte data */
#define OMAP_I2C_STAT_BB	(1 << 12)	/* Bus busy */
#define OMAP_I2C_STAT_ROVR	(1 << 11)	/* Receive overrun */
#define OMAP_I2C_STAT_XUDF	(1 << 10)	/* Transmit underflow */
#define OMAP_I2C_STAT_AAS	(1 << 9)	/* Address as slave */
#define OMAP_I2C_STAT_AD0	(1 << 8)	/* Address zero */
#define OMAP_I2C_STAT_XRDY	(1 << 4)	/* Transmit data ready */
#define OMAP_I2C_STAT_RRDY	(1 << 3)	/* Receive data ready */
#define OMAP_I2C_STAT_ARDY	(1 << 2)	/* Register access ready */
#define OMAP_I2C_STAT_NACK	(1 << 1)	/* No ack interrupt enable */
#define OMAP_I2C_STAT_AL	(1 << 0)	/* Arbitration lost int ena */

/* I2C Buffer Configuration Register (OMAP_I2C_BUF): */
#define OMAP_I2C_BUF_RDMA_EN	(1 << 15)	/* RX DMA channel enable */
#define OMAP_I2C_BUF_XDMA_EN	(1 << 7)	/* TX DMA channel enable */

/* I2C Configuration Register (OMAP_I2C_CON): */
#define OMAP_I2C_CON_EN		(1 << 15)	/* I2C module enable */
#define OMAP_I2C_CON_BE		(1 << 14)	/* Big endian mode */
#define OMAP_I2C_CON_STB	(1 << 11)	/* Start byte mode (master) */
#define OMAP_I2C_CON_MST	(1 << 10)	/* Master/slave mode */
#define OMAP_I2C_CON_TRX	(1 << 9)	/* TX/RX mode (master only) */
#define OMAP_I2C_CON_XA		(1 << 8)	/* Expand address */
#define OMAP_I2C_CON_RM		(1 << 2)	/* Repeat mode (master only) */
#define OMAP_I2C_CON_STP	(1 << 1)	/* Stop cond (master only) */
#define OMAP_I2C_CON_STT	(1 << 0)	/* Start condition (master) */

/* I2C System Test Register (OMAP_I2C_SYSTEST): */
#define OMAP_I2C_SYSTEST_ST_EN		(1 << 15)	/* System test enable */
#define OMAP_I2C_SYSTEST_FREE		(1 << 14)	/* Free running mode */
#define OMAP_I2C_SYSTEST_TMODE_MASK	(3 << 12)	/* Test mode select */
#define OMAP_I2C_SYSTEST_TMODE_SHIFT	(12)		/* Test mode select */
#define OMAP_I2C_SYSTEST_SCL_I		(1 << 3)	/* SCL line sense in */
#define OMAP_I2C_SYSTEST_SCL_O		(1 << 2)	/* SCL line drive out */
#define OMAP_I2C_SYSTEST_SDA_I		(1 << 1)	/* SDA line sense in */
#define OMAP_I2C_SYSTEST_SDA_O		(1 << 0)	/* SDA line drive out */

/* I2C System Status register (OMAP_I2C_SYSS): */
#define OMAP_I2C_SYSS_RDONE		1		/* Reset Done */

/* I2C System Configuration Register (OMAP_I2C_SYSC): */
#define OMAP_I2C_SYSC_SRST		(1 << 1)	/* Soft Reset */

/* REVISIT: Use platform_data instead of module parameters */
static int clock = 100;	/* Default: Fast Mode = 400 KHz, Standard = 100 KHz */
module_param(clock, int, 0);
MODULE_PARM_DESC(clock, "Set I2C clock in kHz: 100 or 400 (Fast Mode)");

static int own;
module_param(own, int, 0);
MODULE_PARM_DESC(own, "Address of OMAP I2C master (0 for default == 1)");

struct omap_i2c_dev {
	struct device		*dev;
	void __iomem		*base;		/* virtual */
	int			irq;
	struct clk		*iclk;		/* Interface clock */
	struct clk		*fclk;		/* Functional clock */
	struct completion	cmd_complete;
	u16			cmd_err;
	u8			*buf;
	size_t			buf_len;
	struct i2c_adapter	adapter;
	unsigned		rev1:1;
	u8			own_address;
};

static inline void omap_i2c_write_reg(struct omap_i2c_dev *i2c_dev,
				      int reg, u16 val)
{
	__raw_writew(val, i2c_dev->base + reg);
}

static inline u16 omap_i2c_read_reg(struct omap_i2c_dev *i2c_dev, int reg)
{
	return __raw_readw(i2c_dev->base + reg);
}

static int omap_i2c_get_clocks(struct omap_i2c_dev *dev)
{
	if (cpu_is_omap16xx() || cpu_is_omap24xx()) {
		dev->iclk = clk_get(dev->dev, "i2c_ick");
		if (IS_ERR(dev->iclk))
			return -ENODEV;
	}

	dev->fclk = clk_get(dev->dev, "i2c_fck");
	if (IS_ERR(dev->fclk)) {
		if (dev->iclk != NULL)
			clk_put(dev->iclk);
		return -ENODEV;
	}

	return 0;
}

static void omap_i2c_put_clocks(struct omap_i2c_dev *dev)
{
	clk_put(dev->fclk);
	dev->fclk = NULL;
	if (dev->iclk != NULL) {
		clk_put(dev->iclk);
		dev->iclk = NULL;
	}
}

static void omap_i2c_enable_clocks(struct omap_i2c_dev *dev)
{
	if (dev->iclk != NULL)
		clk_enable(dev->iclk);
	clk_enable(dev->fclk);
}

static void omap_i2c_disable_clocks(struct omap_i2c_dev *dev)
{
	if (dev->iclk != NULL)
		clk_disable(dev->iclk);
	clk_disable(dev->fclk);
}

static void omap_i2c_reset(struct omap_i2c_dev *dev)
{
	u16 psc;
	unsigned long fclk_rate;

	if (!dev->rev1) {
		omap_i2c_write_reg(dev, OMAP_I2C_SYSC_REG, OMAP_I2C_SYSC_SRST);
		/* For some reason we need to set the EN bit before the
		 * reset done bit gets set. */
		omap_i2c_write_reg(dev, OMAP_I2C_CON_REG, OMAP_I2C_CON_EN);
		while (!(omap_i2c_read_reg(dev, OMAP_I2C_SYSS_REG) & 0x01));
	}
	omap_i2c_write_reg(dev, OMAP_I2C_CON_REG, 0);

	if (cpu_class_is_omap1()) {
		struct clk *armxor_ck;
		unsigned long armxor_rate;

		armxor_ck = clk_get(NULL, "armxor_ck");
		if (IS_ERR(armxor_ck)) {
			printk(KERN_WARNING "i2c: Could not get armxor_ck\n");
			armxor_rate = 12000000;
		} else {
			armxor_rate = clk_get_rate(armxor_ck);
			clk_put(armxor_ck);
		}

		if (armxor_rate > 16000000)
			psc = (armxor_rate + 8000000) / 12000000;
		else
			psc = 0;

		fclk_rate = armxor_rate;
	} else if (cpu_class_is_omap2()) {
		fclk_rate = 12000000;
		psc = 0;
	}

	/* Setup clock prescaler to obtain approx 12MHz I2C module clock: */
	omap_i2c_write_reg(dev, OMAP_I2C_PSC_REG, psc);

	/* Program desired operating rate */
	fclk_rate /= (psc + 1) * 1000;
	if (psc > 2)
		psc = 2;

	omap_i2c_write_reg(dev, OMAP_I2C_SCLL_REG,
			   fclk_rate / (clock * 2) - 7 + psc);
	omap_i2c_write_reg(dev, OMAP_I2C_SCLH_REG,
			   fclk_rate / (clock * 2) - 7 + psc);

	/* Set Own Address: */
	omap_i2c_write_reg(dev, OMAP_I2C_OA_REG, dev->own_address);

	/* Take the I2C module out of reset: */
	omap_i2c_write_reg(dev, OMAP_I2C_CON_REG, OMAP_I2C_CON_EN);

	/* Enable interrupts */
	omap_i2c_write_reg(dev, OMAP_I2C_IE_REG,
			   (OMAP_I2C_IE_XRDY | OMAP_I2C_IE_RRDY |
			    OMAP_I2C_IE_ARDY | OMAP_I2C_IE_NACK |
			    OMAP_I2C_IE_AL));
}

/*
 * Waiting on Bus Busy
 */
static int omap_i2c_wait_for_bb(struct omap_i2c_dev *dev)
{
	unsigned long timeout;

	timeout = jiffies + OMAP_I2C_TIMEOUT;
	while (omap_i2c_read_reg(dev, OMAP_I2C_STAT_REG) & OMAP_I2C_STAT_BB) {
		if (time_after(jiffies, timeout)) {
			dev_warn(dev->dev, "timeout waiting for bus ready\n");
			return -ETIMEDOUT;
		}
		msleep(1);
	}

	return 0;
}

/*
 * Low level master read/write transaction.
 */
static int omap_i2c_xfer_msg(struct i2c_adapter *adap,
			     struct i2c_msg *msg, int stop)
{
	struct omap_i2c_dev *dev = i2c_get_adapdata(adap);
	int r;
	u16 w;
	u8 zero_byte = 0;

	dev_dbg(dev->dev, "addr: 0x%04x, len: %d, flags: 0x%x, stop: %d\n",
		msg->addr, msg->len, msg->flags, stop);

	omap_i2c_write_reg(dev, OMAP_I2C_SA_REG, msg->addr);

	/* Sigh, seems we can't do zero length transactions. Thus, we
	 * can't probe for devices w/o actually sending/receiving at least
	 * a single byte. So we'll set count to 1 for the zero length
	 * transaction case and hope we don't cause grief for some
	 * arbitrary device due to random byte write/read during
	 * probes.
	 */
	/* REVISIT: Could the STB bit of I2C_CON be used with probing? */
	if (msg->len == 0) {
		dev->buf = &zero_byte;
		dev->buf_len = 1;
	} else {
		dev->buf = msg->buf;
		dev->buf_len = msg->len;
	}
	omap_i2c_write_reg(dev, OMAP_I2C_CNT_REG, dev->buf_len);

	init_completion(&dev->cmd_complete);
	dev->cmd_err = 0;

	w = OMAP_I2C_CON_EN | OMAP_I2C_CON_MST | OMAP_I2C_CON_STT;
	if (msg->flags & I2C_M_TEN)
		w |= OMAP_I2C_CON_XA;
	if (!(msg->flags & I2C_M_RD))
		w |= OMAP_I2C_CON_TRX;
	if (stop)
		w |= OMAP_I2C_CON_STP;
	omap_i2c_write_reg(dev, OMAP_I2C_CON_REG, w);

	r = wait_for_completion_timeout(&dev->cmd_complete,
					OMAP_I2C_TIMEOUT);
	dev->buf_len = 0;
	if (r < 0)
		return r;
	if (r == 0) {
		dev_err(dev->dev, "controller timed out\n");
		omap_i2c_reset(dev);
		return -ETIMEDOUT;
	}

	if (likely(!dev->cmd_err))
		return 0;

	/* We have an error */
	if (dev->cmd_err & OMAP_I2C_STAT_NACK) {
		if (msg->flags & I2C_M_IGNORE_NAK)
			return 0;
		if (stop) {
			u16 w;

			w = omap_i2c_read_reg(dev, OMAP_I2C_CON_REG);
			w |= OMAP_I2C_CON_STP;
			omap_i2c_write_reg(dev, OMAP_I2C_CON_REG, w);
		}
		return -EREMOTEIO;
	}
	if (dev->cmd_err & (OMAP_I2C_STAT_AL | OMAP_I2C_STAT_ROVR |
			    OMAP_I2C_STAT_XUDF))
		omap_i2c_reset(dev);
	return -EIO;
}


/*
 * Prepare controller for a transaction and call omap_i2c_xfer_msg
 * to do the work during IRQ processing.
 */
static int
omap_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg msgs[], int num)
{
	struct omap_i2c_dev *dev = i2c_get_adapdata(adap);
	int i;
	int r = 0;

	if (num < 1 || num > MAX_MESSAGES)
		return -EINVAL;

	/* Check for valid parameters in messages */
	for (i = 0; i < num; i++)
		if (msgs[i].buf == NULL)
			return -EINVAL;

	omap_i2c_enable_clocks(dev);

	/* REVISIT: initialize and use adap->retries */
	if ((r = omap_i2c_wait_for_bb(dev)) < 0)
		goto out;

	for (i = 0; i < num; i++) {
		dev_dbg(dev->dev, "msg: %d, addr: 0x%04x, len: %d, flags: 0x%x\n",
			i, msgs[i].addr, msgs[i].len, msgs[i].flags);
		r = omap_i2c_xfer_msg(adap, &msgs[i], (i == (num - 1)));
		if (r != 0)
			break;
	}

	if (r == 0)
		r = num;
out:
	omap_i2c_disable_clocks(dev);
	return r;
}

static u32
omap_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static inline void
omap_i2c_complete_cmd(struct omap_i2c_dev *dev, u16 err)
{
	dev->cmd_err |= err;
	complete(&dev->cmd_complete);
}

static inline void
omap_i2c_ack_stat(struct omap_i2c_dev *dev, u16 stat)
{
	omap_i2c_write_reg(dev, OMAP_I2C_STAT_REG, stat);
}

#ifdef CONFIG_ARCH_OMAP15XX
static irqreturn_t
omap_i2c_rev1_isr(int this_irq, void *dev_id, struct pt_regs *regs)
{
	struct omap_i2c_dev *dev = dev_id;
	u16 iv, w;

	iv = omap_i2c_read_reg(dev, OMAP_I2C_IV_REG);
	switch (iv) {
	case 0x00:	/* None */
		break;
	case 0x01:	/* Arbitration lost */
		dev_err(dev->dev, "Arbitration lost\n");
		omap_i2c_complete_cmd(dev, OMAP_I2C_STAT_AL);
		break;
	case 0x02:	/* No acknowledgement */
		omap_i2c_complete_cmd(dev, OMAP_I2C_STAT_NACK);
		omap_i2c_write_reg(dev, OMAP_I2C_CON_REG, OMAP_I2C_CON_STP);
		break;
	case 0x03:	/* Register access ready */
		omap_i2c_complete_cmd(dev, 0);
		break;
	case 0x04:	/* Receive data ready */
		if (dev->buf_len) {
			w = omap_i2c_read_reg(dev, OMAP_I2C_DATA_REG);
			*dev->buf++ = w;
			dev->buf_len--;
			if (dev->buf_len) {
				*dev->buf++ = w >> 8;
				dev->buf_len--;
			}
		} else
			dev_err(dev->dev, "RRDY IRQ while no data requested\n");
		break;
	case 0x05:	/* Transmit data ready */
		if (dev->buf_len) {
			w = *dev->buf++;
			dev->buf_len--;
			if (dev->buf_len) {
				w |= *dev->buf++ << 8;
				dev->buf_len--;
			}
			omap_i2c_write_reg(dev, OMAP_I2C_DATA_REG, w);
		} else
			dev_err(dev->dev, "XRDY IRQ while no data to send\n");
		break;
	default:
		return IRQ_NONE;
	}

	return IRQ_HANDLED;
}
#endif

static irqreturn_t
omap_i2c_isr(int this_irq, void *dev_id, struct pt_regs *regs)
{
	struct omap_i2c_dev *dev = dev_id;
	u16 bits;
	u16 stat, w;
	int count = 0;

	bits = omap_i2c_read_reg(dev, OMAP_I2C_IE_REG);
	while ((stat = (omap_i2c_read_reg(dev, OMAP_I2C_STAT_REG))) & bits) {
		dev_dbg(dev->dev, "IRQ (ISR = 0x%04x)\n", stat);
		if (count++ == 100) {
			dev_warn(dev->dev, "Too much work in one IRQ\n");
			break;
		}

		omap_i2c_write_reg(dev, OMAP_I2C_STAT_REG, stat);

		if (stat & OMAP_I2C_STAT_ARDY) {
			omap_i2c_complete_cmd(dev, 0);
			continue;
		}
		if (stat & OMAP_I2C_STAT_RRDY) {
			w = omap_i2c_read_reg(dev, OMAP_I2C_DATA_REG);
			if (dev->buf_len) {
				*dev->buf++ = w;
				dev->buf_len--;
				if (dev->buf_len) {
					*dev->buf++ = w >> 8;
					dev->buf_len--;
				}
			} else
				dev_err(dev->dev, "RRDY IRQ while no data requested\n");
			omap_i2c_ack_stat(dev, OMAP_I2C_STAT_RRDY);
			continue;
		}
		if (stat & OMAP_I2C_STAT_XRDY) {
			int bail_out = 0;

			w = 0;
			if (dev->buf_len) {
				w = *dev->buf++;
				dev->buf_len--;
				if (dev->buf_len) {
					w |= *dev->buf++ << 8;
					dev->buf_len--;
				}
			} else
				dev_err(dev->dev, "XRDY IRQ while no data to send\n");
#if 0
			if (!(stat & OMAP_I2C_STAT_BB)) {
				dev_warn(dev->dev, "XRDY while bus not busy\n");
				bail_out = 1;
			}
#endif
			omap_i2c_write_reg(dev, OMAP_I2C_DATA_REG, w);
			omap_i2c_ack_stat(dev, OMAP_I2C_STAT_XRDY);
			if (bail_out)
				omap_i2c_complete_cmd(dev, 1 << 15);
			continue;
		}
		if (stat & OMAP_I2C_STAT_ROVR) {
			dev_err(dev->dev, "Receive overrun\n");
			dev->cmd_err |= OMAP_I2C_STAT_ROVR;
		}
		if (stat & OMAP_I2C_STAT_XUDF) {
			dev_err(dev->dev, "Transmit overflow\n");
			dev->cmd_err |= OMAP_I2C_STAT_XUDF;
		}
		if (stat & OMAP_I2C_STAT_NACK) {
			omap_i2c_complete_cmd(dev, OMAP_I2C_STAT_NACK);
			omap_i2c_write_reg(dev, OMAP_I2C_CON_REG, OMAP_I2C_CON_STP);
		}
		if (stat & OMAP_I2C_STAT_AL) {
			dev_err(dev->dev, "Arbitration lost\n");
			omap_i2c_complete_cmd(dev, OMAP_I2C_STAT_AL);
		}
	}

	return count ? IRQ_HANDLED : IRQ_NONE;
}

static struct i2c_algorithm omap_i2c_algo = {
	.master_xfer	= omap_i2c_xfer,
	.functionality	= omap_i2c_func,
};

static int
omap_i2c_probe(struct platform_device *pdev)
{
	struct omap_i2c_dev	*dev;
	struct i2c_adapter	*adap;
	struct resource		*mem, *irq;
	int r;

	/* NOTE: driver uses the static register mapping */
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		dev_err(&pdev->dev, "no mem resource?\n");
		return -ENODEV;
	}
	irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!irq) {
		dev_err(&pdev->dev, "no irq resource?\n");
		return -ENODEV;
	}

	r = (int) request_mem_region(mem->start, (mem->end - mem->start) + 1,
			driver_name);
	if (!r) {
		dev_err(&pdev->dev, "I2C region already claimed\n");
		return -EBUSY;
	}

	if (clock > 200)
		clock = 400;	/* Fast mode */
	else
		clock = 100;	/* Standard mode */

	dev = kzalloc(sizeof(struct omap_i2c_dev), GFP_KERNEL);
	if (!dev) {
		r = -ENOMEM;
		goto do_release_region;
	}

	/* FIXME: Get own address from platform_data */
	if (own >= 1 && own < 0x7f)
		dev->own_address = own;
	else
		own = DEFAULT_OWN;

	dev->dev = &pdev->dev;
	dev->irq = irq->start;
	dev->base = (void __iomem *) IO_ADDRESS(mem->start);
	platform_set_drvdata(pdev, dev);

	if ((r = omap_i2c_get_clocks(dev)) != 0)
		goto do_free_mem;

	omap_i2c_enable_clocks(dev);

#ifdef CONFIG_ARCH_OMAP15XX
	dev->rev1 = omap_i2c_read_reg(dev, OMAP_I2C_REV_REG) < 0x20;
#endif

	/* reset ASAP, clearing any IRQs */
	omap_i2c_reset(dev);

#ifdef CONFIG_ARCH_OMAP15XX
	r = request_irq(dev->irq, dev->rev1 ? omap_i2c_rev1_isr : omap_i2c_isr,
			0, driver_name, dev);
#else
	r = request_irq(dev->irq, omap_i2c_isr, 0, driver_name, dev);
#endif
	if (r) {
		dev_err(dev->dev, "failure requesting irq %i\n", dev->irq);
		goto do_unuse_clocks;
	}
	r = omap_i2c_read_reg(dev, OMAP_I2C_REV_REG) & 0xff;
	dev_info(dev->dev, "bus %d rev%d.%d at %d kHz\n",
		 pdev->id - 1, r >> 4, r & 0xf, clock);

	adap = &dev->adapter;
	i2c_set_adapdata(adap, dev);
	adap->owner = THIS_MODULE;
	adap->class = I2C_CLASS_HWMON;
	strncpy(adap->name, "OMAP I2C adapter", sizeof(adap->name));
	adap->algo = &omap_i2c_algo;
	adap->dev.parent = &pdev->dev;

	/* i2c device drivers may be active on return from add_adapter() */
	r = i2c_add_adapter(adap);
	if (r) {
		dev_err(dev->dev, "failure adding adapter\n");
		goto do_free_irq;
	}

	omap_i2c_disable_clocks(dev);

	return 0;

do_free_irq:
	free_irq(dev->irq, dev);
do_unuse_clocks:
	omap_i2c_disable_clocks(dev);
	omap_i2c_put_clocks(dev);
do_free_mem:
	kfree(dev);
do_release_region:
	omap_i2c_write_reg(dev, OMAP_I2C_CON_REG, 0);
	release_mem_region(mem->start, (mem->end - mem->start) + 1);

	return r;
}

static int
omap_i2c_remove(struct platform_device *pdev)
{
	struct omap_i2c_dev	*dev = platform_get_drvdata(pdev);
	struct resource		*mem;

	free_irq(dev->irq, dev);
	i2c_del_adapter(&dev->adapter);
	omap_i2c_write_reg(dev, OMAP_I2C_CON_REG, 0);
	omap_i2c_put_clocks(dev);
	kfree(dev);
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(mem->start, (mem->end - mem->start) + 1);
	return 0;
}

static struct platform_driver omap_i2c_driver = {
	.probe		= omap_i2c_probe,
	.remove		= omap_i2c_remove,
	.driver 	= {
		.name	= (char *)driver_name,
	},
};

/* I2C may be needed to bring up other drivers */
static int __init
omap_i2c_init_driver(void)
{
	return platform_driver_register(&omap_i2c_driver);
}
subsys_initcall(omap_i2c_init_driver);

static void __exit omap_i2c_exit_driver(void)
{
	platform_driver_unregister(&omap_i2c_driver);
}
module_exit(omap_i2c_exit_driver);

MODULE_AUTHOR("MontaVista Software, Inc. (and others)");
MODULE_DESCRIPTION("TI OMAP I2C bus adapter");
MODULE_LICENSE("GPL");
