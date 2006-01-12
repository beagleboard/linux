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
 * ----------------------------------------------------------------------------
 * This file was highly leveraged from i2c-elektor.c:
 *
 * Copyright 1995-97 Simon G. Vogl
 *           1998-99 Hans Berglund
 *
 * With some changes from Kysti M�kki <kmalkki@cc.hut.fi> and even
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

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>

#include <asm/io.h>
#include <asm/hardware/clock.h>

/* ----- global defines ----------------------------------------------- */
static const char driver_name[] = "i2c_omap";

#define MODULE_NAME "OMAP I2C"
#define OMAP_I2C_TIMEOUT (1*HZ)	/* timeout waiting for an I2C transaction */

#define pr_err(format, arg...) \
		printk(KERN_ERR MODULE_NAME " ERROR: " format "\n", ## arg )

#define DEFAULT_OWN		1	/* default own I2C address */
#define MAX_MESSAGES		65536	/* max number of messages */

#define OMAP_I2C_REV		0x00
#define OMAP_I2C_IE		0x04
#define OMAP_I2C_STAT		0x08
#define OMAP_I2C_IV		0x0c
#define OMAP_I2C_SYSS		0x10
#define OMAP_I2C_BUF		0x14
#define OMAP_I2C_CNT		0x18
#define OMAP_I2C_DATA		0x1c
#define OMAP_I2C_SYSC		0x20
#define OMAP_I2C_CON		0x24
#define OMAP_I2C_OA		0x28
#define OMAP_I2C_SA		0x2c
#define OMAP_I2C_PSC		0x30
#define OMAP_I2C_SCLL		0x34
#define OMAP_I2C_SCLH		0x38
#define OMAP_I2C_SYSTEST	0x3c

/* I2C Interrupt Enable Register (OMAP_I2C_IE): */
#define OMAP_I2C_IE_XRDY_IE	(1 << 4)	/* TX data ready int enable */
#define OMAP_I2C_IE_RRDY_IE	(1 << 3)	/* RX data ready int enable */
#define OMAP_I2C_IE_ARDY_IE	(1 << 2)	/* Access ready int enable */
#define OMAP_I2C_IE_NACK_IE	(1 << 1)	/* No ack interrupt enable */
#define OMAP_I2C_IE_AL_IE	(1 << 0)	/* Arbitration lost int ena */

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
#define OMAP_I2C_CON_RST	(0 << 15)	/* I2C module reset */
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

/* ------- debugging ---------------------------------------------------*/

#define I2C_OMAP_DEBUG
#ifdef I2c_OMAP_DEBUG
static int i2c_debug;

module_param(i2c_debug, int, 0);
MODULE_PARM_DESC(i2c_debug,
		"debug level - 0 off; 1 normal; 2,3 more verbose; "
		"9 omap-protocol");

#define DEB(level, format, arg...) do { \
	if (i2c_debug >= level) \
		printk(KERN_DEBUG MODULE_NAME " DEBUG: " format "\n", ## arg); \
	} while(0)

#define DEB0(format, arg...) DEB(0, format, arg)
#define DEB1(format, arg...) DEB(1, format, arg)
#define DEB2(format, arg...) DEB(2, format, arg)
#define DEB3(format, arg...) DEB(3, format, arg)
#define DEB9(format, arg...) DEB(9, format, arg)

#else

#define DEB0(fmt, args...)
#define DEB1(fmt, args...)
#define DEB2(fmt, args...)
#define DEB3(fmt, args...)
#define DEB9(fmt, args...)

#endif

static int clock = 100;	/* Default: Fast Mode = 400 KHz, Standard = 100 KHz */
module_param(clock, int, 0);
MODULE_PARM_DESC(clock, "Set I2C clock in KHz: 100 or 400 (Fast Mode)");

static int own;
module_param(own, int, 0);
MODULE_PARM_DESC(own, "Address of OMAP i2c master (0 for default == 1)");

struct omap_i2c_dev {
	struct device		*dev;
	void __iomem		*base;		/* virtual */
	int			irq;
	struct clk		*iclk;		/* Interface clock */
	struct clk		*fclk;		/* Functional clock */
	int			cmd_complete, cmd_err;
	wait_queue_head_t	cmd_wait;
	u8			*buf;
	size_t			buf_len;
	struct i2c_adapter	adapter;
	int			rev1;
};

static void inline omap_i2c_write(struct omap_i2c_dev *i2c_dev,
				  int val, int reg)
{
	__raw_writew(val, i2c_dev->base + reg);
}

static int inline omap_i2c_read(struct omap_i2c_dev *i2c_dev, int reg)
{
	return __raw_readw(i2c_dev->base + reg);
}

static void omap_i2c_reset(struct omap_i2c_dev *omap_i2c_dev)
{
	unsigned long timeout;
	u16 psc = 0;
	unsigned long fclk_rate;

	/* Soft reset, hard reset needed only for rev1 */
	if (!omap_i2c_dev->rev1)
		omap_i2c_write(omap_i2c_dev, OMAP_I2C_SYSC_SRST, OMAP_I2C_SYSC);
	else
		omap_i2c_write(omap_i2c_dev, OMAP_I2C_CON_RST, OMAP_I2C_CON);

	fclk_rate = 12000000;

	if (!cpu_is_omap24xx()) {
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

		fclk_rate = armxor_rate;
	}

	/* Setup clock prescaler to obtain approx 12MHz I2C module clock: */
	omap_i2c_write(omap_i2c_dev, psc, OMAP_I2C_PSC);

	/* Program desired operating rate */
	fclk_rate /= (psc + 1) * 1000;
	if (psc > 2)
		psc = 2;
	omap_i2c_write(omap_i2c_dev,
			fclk_rate / (clock * 2) - 7 + psc, OMAP_I2C_SCLL);
	omap_i2c_write(omap_i2c_dev,
			fclk_rate / (clock * 2) - 7 + psc, OMAP_I2C_SCLH);

	/* Set Own Address: */
	omap_i2c_write(omap_i2c_dev, own, OMAP_I2C_OA);

	/* Enable interrupts */
	omap_i2c_write(omap_i2c_dev,
			(OMAP_I2C_IE_XRDY_IE | OMAP_I2C_IE_RRDY_IE |
		         OMAP_I2C_IE_ARDY_IE | OMAP_I2C_IE_NACK_IE |
		         OMAP_I2C_IE_AL_IE),
			OMAP_I2C_IE);

	/* Take the I2C module out of reset: */
	omap_i2c_write(omap_i2c_dev, OMAP_I2C_CON_EN, OMAP_I2C_CON);

	if (!omap_i2c_dev->rev1){
		timeout = jiffies + OMAP_I2C_TIMEOUT;
		while (!(omap_i2c_read(omap_i2c_dev, OMAP_I2C_SYSS) &
					OMAP_I2C_SYSS_RDONE)) {
			if (time_after(jiffies, timeout)) {
				pr_err("timeout waiting for I2C reset");
				break;
			}
			msleep(1);
		}
	}
}

/*
 * Waiting on Bus Busy
 */
static int
omap_i2c_wait_for_bb(struct omap_i2c_dev *omap_i2c_dev, char allow_sleep)
{
	unsigned long timeout;

	timeout = jiffies + OMAP_I2C_TIMEOUT;
	while (omap_i2c_read(omap_i2c_dev, OMAP_I2C_STAT) & OMAP_I2C_STAT_BB) {
		if (time_after(jiffies, timeout)) {
			printk(KERN_WARNING "timeout waiting for bus ready\n");
			return -ETIMEDOUT;
		}
		if (allow_sleep)
			msleep(1);
	}

	return 0;
}

/*
 * Low level master read/write transaction.
 */
static int
omap_i2c_xfer_msg(struct i2c_adapter *adap, struct i2c_msg *msg, int stop)
{
	struct omap_i2c_dev *omap_i2c_dev = i2c_get_adapdata(adap);
	u8 zero_byte = 0;
	int r;
	u16 w;

	DEB2("addr: 0x%04x, len: %d, flags: 0x%x, stop: %d\n",
	     msg->addr, msg->len, msg->flags, stop);

	omap_i2c_write(omap_i2c_dev, msg->addr, OMAP_I2C_SA);

	/* Sigh, seems we can't do zero length transactions. Thus, we
	 * can't probe for devices w/o actually sending/receiving at least
	 * a single byte. So we'll set count to 1 for the zero length
	 * transaction case and hope we don't cause grief for some
	 * arbitrary device due to random byte write/read during
	 * probes.
	 */
	if (msg->len == 0) {
		omap_i2c_dev->buf = &zero_byte;
		omap_i2c_dev->buf_len = 1;
	} else {
		omap_i2c_dev->buf = msg->buf;
		omap_i2c_dev->buf_len = msg->len;
	}
	omap_i2c_write(omap_i2c_dev, omap_i2c_dev->buf_len, OMAP_I2C_CNT);
	omap_i2c_dev->cmd_complete = 0;
	omap_i2c_dev->cmd_err = 0;
	w = OMAP_I2C_CON_EN | OMAP_I2C_CON_MST | OMAP_I2C_CON_STT;
	if (msg->flags & I2C_M_TEN)
		w |= OMAP_I2C_CON_XA;
	if (!(msg->flags & I2C_M_RD))
		w |= OMAP_I2C_CON_TRX;
	if (stop)
		w |= OMAP_I2C_CON_STP;
	omap_i2c_write(omap_i2c_dev, w, OMAP_I2C_CON);

	r = wait_event_interruptible_timeout(omap_i2c_dev->cmd_wait,
					     omap_i2c_dev->cmd_complete,
					     OMAP_I2C_TIMEOUT);

	omap_i2c_dev->buf_len = 0;
	if (r < 0)
		return r;
	if (!omap_i2c_dev->cmd_complete) {
		omap_i2c_reset(omap_i2c_dev);
		return -ETIMEDOUT;
	}
	if (!omap_i2c_dev->cmd_err)
		return msg->len;

	/* We have an error */
	if (omap_i2c_dev->cmd_err & OMAP_I2C_STAT_NACK) {
		if (msg->flags & I2C_M_IGNORE_NAK)
			return msg->len;
		if (stop)
			omap_i2c_write(omap_i2c_dev,
				omap_i2c_read(omap_i2c_dev, OMAP_I2C_CON) |
						OMAP_I2C_CON_STP,
				OMAP_I2C_CON);
		return -EREMOTEIO;
	}
	if ((OMAP_I2C_STAT_AL | OMAP_I2C_STAT_ROVR | OMAP_I2C_STAT_XUDF)
			& omap_i2c_dev->cmd_err) {
		omap_i2c_reset(omap_i2c_dev);
		return -EIO;
	}
	return msg->len;
}

/*
 * Prepare controller for a transaction and call omap_i2c_xfer_msg
 * to do the work during IRQ processing.
 */
static int
omap_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg msgs[], int num)
{
	struct omap_i2c_dev *omap_i2c_dev = i2c_get_adapdata(adap);
	int i;
	int r = 0;

	DEB1("msgs: %d\n", num);

	if (num < 1 || num > MAX_MESSAGES)
		return -EINVAL;

	/* Check for valid parameters in messages */
	for (i = 0; i < num; i++)
		if (msgs[i].buf == NULL)
			return -EINVAL;

// REVISIT: initialize and use adap->retries

	if ((r = omap_i2c_wait_for_bb(omap_i2c_dev, 1)) < 0)
		return r;

	for (i = 0; i < num; i++) {
		DEB2("msg: %d, addr: 0x%04x, len: %d, flags: 0x%x\n",
		     i, msgs[i].addr, msgs[i].len, msgs[i].flags);

		r = omap_i2c_xfer_msg(adap, &msgs[i], (i == (num - 1)));

		DEB2("r: %d\n", r);

		if (r != msgs[i].len)
			break;
	}

	if (r >= 0 && num > 1)
		r = num;

	DEB2("r: %d\n", r);

	return r;
}

static u32
omap_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static inline void
omap_i2c_complete_cmd(struct omap_i2c_dev *dev)
{
	dev->cmd_complete = 1;
	wake_up(&dev->cmd_wait);
}

static irqreturn_t
omap_i2c_isr(int this_irq, void *dev_id, struct pt_regs *regs)
{
	struct omap_i2c_dev *omap_i2c_dev = dev_id;
	u16 bits;
	u16 stat, w;
	int count = 0;
	u16 iv_read;

	bits = omap_i2c_read(omap_i2c_dev, OMAP_I2C_IE);
	while ((stat = omap_i2c_read(omap_i2c_dev, OMAP_I2C_STAT)) & bits) {

		if (count++ == 100) {
			printk(KERN_WARNING "Too much work in one IRQ\n");
			break;
		}

		omap_i2c_write(omap_i2c_dev, stat, OMAP_I2C_STAT);
		if (stat & OMAP_I2C_STAT_ARDY) {
			omap_i2c_complete_cmd(omap_i2c_dev);
			omap_i2c_write(omap_i2c_dev, OMAP_I2C_STAT_ARDY,
				       OMAP_I2C_STAT);
			if (omap_i2c_dev->rev1)
				iv_read = omap_i2c_read(omap_i2c_dev,
							OMAP_I2C_IV);
			continue;
		}
		if (stat & OMAP_I2C_STAT_RRDY) {
			w = omap_i2c_read(omap_i2c_dev, OMAP_I2C_DATA);
			if (omap_i2c_dev->buf_len) {
				*omap_i2c_dev->buf++ = w;
				omap_i2c_dev->buf_len--;
				if (omap_i2c_dev->buf_len) {
					*omap_i2c_dev->buf++ = w >> 8;
					omap_i2c_dev->buf_len--;
				}
				if (omap_i2c_dev->rev1 &&
				    !omap_i2c_dev->buf_len)
					omap_i2c_complete_cmd(omap_i2c_dev);
			} else
				pr_err("RRDY IRQ while no data requested");
			omap_i2c_write(omap_i2c_dev, OMAP_I2C_STAT_RRDY,
					OMAP_I2C_STAT);
			if (omap_i2c_dev->rev1)
				iv_read = omap_i2c_read(omap_i2c_dev,
							OMAP_I2C_IV);
			continue;
		}
		if (stat & OMAP_I2C_STAT_XRDY) {
			w = 0;
			if (omap_i2c_dev->buf_len) {
				w = *omap_i2c_dev->buf++;
				omap_i2c_dev->buf_len--;
				if (omap_i2c_dev->buf_len) {
					w |= *omap_i2c_dev->buf++ << 8;
					omap_i2c_dev->buf_len--;
				}
			} else {
				pr_err("XRDY IRQ while no data to send");
				/* FIXME: shouldn't we bail out here? */
			}
			omap_i2c_write(omap_i2c_dev, w, OMAP_I2C_DATA);
			/* We have to make sure the XRDY bit is reset */
			omap_i2c_write(omap_i2c_dev, OMAP_I2C_STAT_XRDY,
					OMAP_I2C_STAT);
			if (omap_i2c_dev->rev1) {
				iv_read = omap_i2c_read(omap_i2c_dev,
							OMAP_I2C_IV);
				if (!omap_i2c_dev->buf_len)
					omap_i2c_complete_cmd(omap_i2c_dev);
			}
			continue;
		}
		if (stat & OMAP_I2C_STAT_ROVR) {
			pr_err("Receive overrun\n");
			omap_i2c_dev->cmd_err |= OMAP_I2C_STAT_ROVR;
		}
		if (stat & OMAP_I2C_STAT_XUDF) {
			pr_err("Transmit overflow\n");
			omap_i2c_dev->cmd_err |= OMAP_I2C_STAT_XUDF;
		}
		if (stat & OMAP_I2C_STAT_NACK) {
			omap_i2c_dev->cmd_err |= OMAP_I2C_STAT_NACK;
			omap_i2c_complete_cmd(omap_i2c_dev);
			omap_i2c_write(omap_i2c_dev, OMAP_I2C_CON_STP,
				       OMAP_I2C_CON);
		}
		if (stat & OMAP_I2C_STAT_AL) {
			pr_err("Arbitration lost\n");
			omap_i2c_dev->cmd_err |= OMAP_I2C_STAT_AL;
			omap_i2c_complete_cmd(omap_i2c_dev);
		}
		if (omap_i2c_dev->rev1)
			iv_read = omap_i2c_read(omap_i2c_dev, OMAP_I2C_IV);

	}
	return IRQ_HANDLED;
}

static struct i2c_algorithm omap_i2c_algo = {
	.master_xfer	= omap_i2c_xfer,
	.functionality	= omap_i2c_func,
};

#ifdef CONFIG_ARCH_OMAP24XX
static int omap_i2c_24xx_get_clocks(struct omap_i2c_dev *omap_i2c_dev, int bus)
{
	if (!cpu_is_omap24xx())
		return 0;

	omap_i2c_dev->iclk = clk_get(NULL,
		bus == 1 ? "i2c1_ick" : "i2c2_ick");
	if (IS_ERR(omap_i2c_dev->iclk)) {
		return -ENODEV;
	}

	omap_i2c_dev->fclk = clk_get(NULL,
		bus == 1 ? "i2c1_fck" : "i2c2_fck");
	if (IS_ERR(omap_i2c_dev->fclk)) {
		clk_put(omap_i2c_dev->fclk);
		return -ENODEV;
	}

	return 0;
}

static void omap_i2c_24xx_put_clocks(struct omap_i2c_dev *omap_i2c_dev)
{
	if (cpu_is_omap24xx()) {
		clk_put(omap_i2c_dev->fclk);
		clk_put(omap_i2c_dev->iclk);
	}
}

static void omap_i2c_24xx_enable_clocks(struct omap_i2c_dev *omap_i2c_dev,
					int enable)
{
	if (cpu_is_omap24xx()) {
		if (enable) {
			clk_enable(omap_i2c_dev->iclk);
			clk_enable(omap_i2c_dev->fclk);
		} else {
			clk_disable(omap_i2c_dev->iclk);
			clk_disable(omap_i2c_dev->fclk);
		}
	}
}

#else
#define omap_i2c_24xx_get_clocks(x, y)		0
#define omap_i2c_24xx_enable_clocks(x, y)	do {} while (0)
#define omap_i2c_24xx_put_clocks(x)		do {} while (0)
#endif

static int
omap_i2c_probe(struct platform_device *pdev)
{
	struct omap_i2c_dev	*omap_i2c_dev;
	struct i2c_adapter	*adap;
	struct resource		*mem, *irq;
	int r;

	/* NOTE: driver uses the static register mapping */
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		pr_err("%s: no mem resource?\n", driver_name);
		return -ENODEV;
	}
	irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!irq) {
		pr_err("%s: no irq resource?\n", driver_name);
		return -ENODEV;
	}

	r = (int) request_mem_region(mem->start, (mem->end - mem->start) + 1,
			driver_name);
	if (!r) {
		pr_err("%s: I2C region already claimed\n", driver_name);
		return -EBUSY;
	}

	if (clock > 200)
		clock = 400;	/* Fast mode */
	else
		clock = 100;	/* Standard mode */

	if (own < 1 || own > 0x7f)
		own = DEFAULT_OWN;

	omap_i2c_dev = kzalloc(sizeof(struct omap_i2c_dev), GFP_KERNEL);
	if (!omap_i2c_dev) {
		r = -ENOMEM;
		goto do_release_region;
	}

	omap_i2c_dev->dev = &pdev->dev;
	omap_i2c_dev->irq = irq->start;
	omap_i2c_dev->base = (void __iomem *)IO_ADDRESS(mem->start);
	platform_set_drvdata(pdev, omap_i2c_dev);
	init_waitqueue_head(&omap_i2c_dev->cmd_wait);

	if ((r = omap_i2c_24xx_get_clocks(omap_i2c_dev, pdev->id)) != 0)
		goto do_free_mem;

	omap_i2c_24xx_enable_clocks(omap_i2c_dev, 1);

#ifdef CONFIG_ARCH_OMAP15XX
	omap_i2c_dev->rev1 = omap_i2c_read(omap_i2c_dev, OMAP_I2C_REV) < 0x20;
#endif

	/* reset ASAP, clearing any IRQs */
	omap_i2c_reset(omap_i2c_dev);

	r = request_irq(omap_i2c_dev->irq, omap_i2c_isr, 0,
			driver_name, omap_i2c_dev);
	if (r) {
		pr_err("%s: failure requesting irq %i\n",
			 driver_name, omap_i2c_dev->irq);
		goto do_unuse_clocks;
	}
	r = omap_i2c_read(omap_i2c_dev, OMAP_I2C_REV) & 0xff;
	pr_info("%s: bus %d rev%d.%d at %d KHz\n", driver_name,
			pdev->id - 1, r >> 4, r & 0xf, clock);

	adap = &omap_i2c_dev->adapter;
	i2c_set_adapdata(adap, omap_i2c_dev);
	adap->owner = THIS_MODULE;
	adap->class = I2C_CLASS_HWMON;
	strncpy(adap->name, "OMAP I2C adapter", sizeof(adap->name));
	adap->algo = &omap_i2c_algo;
	adap->dev.parent = &pdev->dev;
	/* i2c device drivers may be active on return from add_adapter() */
	r = i2c_add_adapter(adap);
	if (r) {
		pr_err("%s: failure adding adapter\n", driver_name);
		goto do_free_irq;
	}

	return 0;

do_free_irq:
	free_irq(omap_i2c_dev->irq, omap_i2c_dev);
do_unuse_clocks:
	omap_i2c_24xx_enable_clocks(omap_i2c_dev, 0);
	omap_i2c_24xx_put_clocks(omap_i2c_dev);
do_free_mem:
	kfree(omap_i2c_dev);
do_release_region:
	omap_i2c_write(omap_i2c_dev, 0, OMAP_I2C_CON);
	release_mem_region(mem->start, (mem->end - mem->start) + 1);

	return r;
}

static int
omap_i2c_remove(struct platform_device *pdev)
{
	struct omap_i2c_dev	*omap_i2c_dev = platform_get_drvdata(pdev);
	struct resource		*mem;

	omap_i2c_write(omap_i2c_dev, 0, OMAP_I2C_CON);
	i2c_del_adapter(&omap_i2c_dev->adapter);
	free_irq(omap_i2c_dev->irq, omap_i2c_dev);
	omap_i2c_24xx_enable_clocks(omap_i2c_dev, 0);
	omap_i2c_24xx_put_clocks(omap_i2c_dev);
	kfree(omap_i2c_dev);
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

/* i2c may be needed to bring up other drivers */
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
