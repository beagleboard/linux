/*
 * linux/drivers/i2c/i2c-omap.c
 *
 * TI OMAP I2C unified algorith+adapter driver (inspired by i2c-ibm_iic.c and i2c-omap1510.c)
 *
 * Copyright (C) 2003 MontaVista Software, Inc.
 *
 * Copyright (C) 2004 Texas Instruments.
 *
 * ----------------------------------------------------------------------------
 * This file was highly leveraged from i2c-elektor.c, which was created
 * by Simon G. Vogl and Hans Berglund:
 *
 *
 * Copyright 1995-97 Simon G. Vogl
 *           1998-99 Hans Berglund
 *
 * With some changes from Kysti Mï¿½kki <kmalkki@cc.hut.fi> and even
 * Frodo Looijaard <frodol@dds.nl>
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
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 * ----------------------------------------------------------------------------
 Modifications:
 ver. 1.1: Nov 2003, MontaVista Software
 - added DPM support
 ver. 1.2: Feb 2004, Texas Instruments
 - Ported to 2.6 kernel (Feb 2004)
 - Added support for I2C_M_IGNORE_NAK option.
 ver. 1.3: Mar 2004, Juha Yrjölä <juha.yrjola@nokia.com>
 - Cleaned up
 ver. 1.4: Aug 2004, Thiago Radicchi <trr@dcc.ufmg.br>  DCC-UFMG / iNdT
 - Updated omap_i2c_isr to remove messages of too much work in one IRQ,
   by reading the interrupt vector, as specified on ref [1]
 ver. 1.5: Oct 2004, Tuukka Tikkanen <tuukka.tikkanen@elektrobit.com>
 - Changed clock handling
 *
 * REFERENCES:
 *
 * 1.   OMAP5910 Dual-Core Processor Inter-Integrated Circuit (I2C)
 *      Controller Reference Guide
 *	Document number: spru681
 *      Date: October 2003
 *	http://www-s.ti.com/sc/psheets/spru681/spru681.pdf
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/version.h>
#include <linux/i2c.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <asm/hardware/clock.h>

#include <asm/uaccess.h>
#include <asm/hardware/clock.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <asm/arch/hardware.h>
#include <linux/interrupt.h>
#include <linux/moduleparam.h>
#include <asm/arch/mux.h>
#include <linux/err.h>
#include "i2c-omap.h"

/* ----- global defines ----------------------------------------------- */
#define MODULE_NAME "OMAP I2C"
#define OMAP_I2C_TIMEOUT (1*HZ)	/* timeout waiting for an I2C transaction */

#define	I2C_OMAP_DEBUG
#ifdef I2C_OMAP_DEBUG
static int i2c_debug = 0;
#define DEB0(format, arg...)	printk(KERN_DEBUG MODULE_NAME " DEBUG: " format "\n",  ## arg )
#define DEB1(format, arg...)	\
	if (i2c_debug>=1) {	\
		printk(KERN_DEBUG MODULE_NAME " DEBUG: " format "\n",  ## arg ); \
	}
#define DEB2(format, arg...)	\
	if (i2c_debug>=2) {	\
		printk(KERN_DEBUG MODULE_NAME " DEBUG: " format "\n",  ## arg ); \
	}
#define DEB3(format, arg...)	\
	if (i2c_debug>=3) {	\
		printk(KERN_DEBUG MODULE_NAME " DEBUG: " format "\n",  ## arg ); \
	}
#define DEB9(format, arg...)	\
	/* debug the protocol by showing transferred bits */	\
	if (i2c_debug>=9) {	\
		printk(KERN_DEBUG MODULE_NAME " DEBUG: " format "\n",  ## arg ); \
	}
#else
#define DEB0(fmt, args...)
#define DEB1(fmt, args...)
#define DEB2(fmt, args...)
#define DEB3(fmt, args...)
#define DEB9(fmt, args...)
#endif

#define err(format, arg...) printk(KERN_ERR MODULE_NAME " ERROR: " format "\n",  ## arg )
#define info(format, arg...) printk(KERN_INFO MODULE_NAME ": " format "\n",  ## arg )
#define warn(format, arg...) printk(KERN_WARNING MODULE_NAME " WARNING: " format "\n",  ## arg )
#define emerg(format, arg...) printk(KERN_EMERG MODULE_NAME " EMERGENCY: " format "\n",  ## arg )

#ifdef CONFIG_ARCH_OMAP1510
#define omap_i2c_rev1()		(readw(OMAP_I2C_REV) < 0x20)
#else
#define omap_i2c_rev1()		0
#endif

#define DEFAULT_OWN	1	/*default own I2C address */
#define MAX_MESSAGES	65536	/* max number of messages */

static int clock = 100;		/* Default: Fast Mode = 400 KHz, Standard Mode = 100 KHz */
static int own;
static int i2c_scan;		/* have a look at what's hanging 'round */

static struct omap_i2c_dev {
        int cmd_complete, cmd_err;
        wait_queue_head_t cmd_wait;
	u8 *buf;
	size_t buf_len;
} omap_i2c_dev;


static int omap_i2c_reset(void)
{
	unsigned long timeout;
	u16 psc;
	struct clk *armxor_ck;
	unsigned long armxor_rate;

	if(!cpu_is_omap1510()) {

		writew(OMAP_I2C_SYSC_SRST, OMAP_I2C_SYSC);	/*soft reset */
	}
	else {
		writew(OMAP_I2C_CON_RST, OMAP_I2C_CON);		/* reset */
	}

	armxor_ck = clk_get(0, "armxor_ck");
	if (IS_ERR(armxor_ck)) {
		printk(KERN_WARNING "i2c: Could not obtain armxor_ck rate.\n");
		armxor_rate = 12000000;
	} else {
		armxor_rate = clk_get_rate(armxor_ck);
		clk_put(armxor_ck);
	}

	if (armxor_rate <= 16000000)
		psc = 0;
	else
		psc = (armxor_rate + 8000000) / 12000000;

	/* Setup clock prescaler to obtain approx 12MHz I2C module clock: */
	writew(psc, OMAP_I2C_PSC);

	/* Program desired operating rate */
	armxor_rate /= (psc + 1) * 1000;
	if (psc > 2)
		psc = 2;
	writew(armxor_rate / (clock * 2) - 7 + psc, OMAP_I2C_SCLL);
	writew(armxor_rate / (clock * 2) - 7 + psc, OMAP_I2C_SCLH);


	/* Set Own Address: */
	writew(own, OMAP_I2C_OA);

	/* Enable interrupts */
	writew((OMAP_I2C_IE_XRDY_IE | OMAP_I2C_IE_RRDY_IE | OMAP_I2C_IE_ARDY_IE |
		OMAP_I2C_IE_NACK_IE | OMAP_I2C_IE_AL_IE), OMAP_I2C_IE);

	/* Take the I2C module out of reset: */
	writew(OMAP_I2C_CON_EN, OMAP_I2C_CON);

	if(!cpu_is_omap1510()){
		timeout = jiffies + OMAP_I2C_TIMEOUT;
		while (!(readw(OMAP_I2C_SYSS) & OMAP_I2C_SYSS_RDONE)) {
			if (time_after(jiffies, timeout)) {
				err("timeout waiting for I2C reset complete");
				return -EFAULT;
			}
			schedule_timeout(1);
		}
	}

	return 0;

}

/*
 * Waiting on Bus Busy
 */
static int
omap_i2c_wait_for_bb(char allow_sleep)
{
	unsigned long timeout;

	timeout = jiffies + OMAP_I2C_TIMEOUT;
	while (readw(OMAP_I2C_STAT) & OMAP_I2C_STAT_BB) {
		if (time_after(jiffies, timeout)) {
			warn("timeout waiting for bus ready");
			return -ETIMEDOUT;
		}
		if (allow_sleep)
			schedule_timeout(1);
	}

	return 0;
}

/*
 * Low level master read/write transaction.
 */
static int
omap_i2c_xfer_msg(struct i2c_adapter *adap, struct i2c_msg *msg, int stop)
{
        struct omap_i2c_dev *dev = i2c_get_adapdata(adap);
        u8 zero_byte = 0;
	int r;
	u16 w;

	DEB2("addr: 0x%04x, len: %d, flags: 0x%x, stop: %d",
	     msg->addr, msg->len, msg->flags, stop);

	writew(msg->addr, OMAP_I2C_SA);

	/* Sigh, seems we can't do zero length transactions. Thus, we
	 * can't probe for devices w/o actually sending/receiving at least
	 * a single byte. So we'll set count to 1 for the zero length
	 * transaction case and hope we don't cause grief for some
	 * arbitrary device due to random byte write/read during
	 * probes.
	 */
	if (msg->len == 0) {
		dev->buf = &zero_byte;
                dev->buf_len = 1;
	} else {
		dev->buf = msg->buf;
		dev->buf_len = msg->len;
	}
	writew(dev->buf_len, OMAP_I2C_CNT);
        dev->cmd_complete = 0;
        dev->cmd_err = 0;
	w = OMAP_I2C_CON_EN | OMAP_I2C_CON_MST | OMAP_I2C_CON_STT;
	if (msg->flags & I2C_M_TEN)
		w |= OMAP_I2C_CON_XA;
	if (!(msg->flags & I2C_M_RD))
		w |= OMAP_I2C_CON_TRX;
	if (stop)
		w |= OMAP_I2C_CON_STP;
	writew(w, OMAP_I2C_CON);

	r = wait_event_interruptible_timeout(dev->cmd_wait,
					     dev->cmd_complete,
					     OMAP_I2C_TIMEOUT);
	dev->buf_len = 0;
	if (r < 0)
                return r;
	if (!dev->cmd_complete) {
                omap_i2c_reset();
		return -ETIMEDOUT;
	}
	if (!dev->cmd_err)
		return msg->len;

	/* We have an error */
	if (dev->cmd_err & OMAP_I2C_STAT_NACK) {
		if (msg->flags & I2C_M_IGNORE_NAK)
                        return msg->len;
		if (stop)
			writew(readw(OMAP_I2C_CON) | OMAP_I2C_CON_STP, OMAP_I2C_CON);
		return -EREMOTEIO;
	}
	if (dev->cmd_err & OMAP_I2C_STAT_AL ||
	    dev->cmd_err & OMAP_I2C_STAT_ROVR ||
	    dev->cmd_err & OMAP_I2C_STAT_XUDF) {
		omap_i2c_reset();
		return -EIO;
	}
        return msg->len;
}

/*
 * Prepare controller for a transaction and call omap_i2c_rxbytes
 * to do the work.
 */
static int
omap_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg msgs[], int num)
{
	int i;
	int r = 0;

	DEB1("msgs: %d", num);

	if (num < 1 || num > MAX_MESSAGES)
		return -EINVAL;

	/* Check for valid parameters in messages */
	for (i = 0; i < num; i++)
		if (msgs[i].buf == NULL)
			return -EINVAL;

	if ((r = omap_i2c_wait_for_bb(1)) < 0)
		return r;

	for (i = 0; i < num; i++) {
		DEB2("msg: %d, addr: 0x%04x, len: %d, flags: 0x%x",
		     i, msgs[i].addr, msgs[i].len, msgs[i].flags);

		r = omap_i2c_xfer_msg(adap, &msgs[i], (i == (num - 1)));

		DEB2("r: %d", r);

		if (r != msgs[i].len)
			break;
	}

	if (r >= 0 && num > 1)
		r = num;

	DEB1("r: %d", r);

	return r;
}

/*
 * Sanity check for the adapter hardware - check the reaction of
 * the bus lines only if it seems to be idle.
 *
 * Scan the I2C bus for valid 7 bit addresses
 * (ie things that ACK on 1byte read)
 * if i2c_debug is off we print everything on one line.
 * if i2c_debug is on we do a newline per print so we don't
 * clash too much with printf's in the other functions.
 * TODO: check for 10-bit mode and never run as a slave.
 */
static int
omap_i2c_scan_bus(struct i2c_adapter *adap)
{
	int found = 0;
	int i;
	struct i2c_msg msg;
	char data[1];

	info("scanning for active I2C devices on the bus...");

	for (i = 1; i < 0x7f; i++) {
		if (readw(OMAP_I2C_OA) == i)
			continue;

		msg.addr = i;
		msg.buf = data;
		msg.len = 0;
		msg.flags = I2C_M_RD;

		if (omap_i2c_xfer(adap, &msg, 1) == 0) {
			info("I2C device 0x%02x found", i);
			found++;
		}
	}

	if (!found)
		info("found nothing");

	return found;
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
	struct omap_i2c_dev *dev = dev_id;
        u16 bits;
	u16 stat, w;
        int count = 0;
	u16 iv_read;

        bits = readw(OMAP_I2C_IE);
	while ((stat = readw(OMAP_I2C_STAT)) & bits) {
		
		if (count++ == 100) {
			warn("Too much work in one IRQ");
			break;
		}

		writew(stat, OMAP_I2C_STAT);
		if (stat & OMAP_I2C_STAT_ARDY) {
			omap_i2c_complete_cmd(dev);
			writew(OMAP_I2C_STAT_ARDY, OMAP_I2C_STAT);
			if (omap_i2c_rev1())
				iv_read = readw(OMAP_I2C_IV);
			continue;
		}
		if (stat & OMAP_I2C_STAT_RRDY) {
                        w = readw(OMAP_I2C_DATA);
			if (dev->buf_len) {
				*dev->buf++ = w;
				dev->buf_len--;
				if (dev->buf_len) {
					*dev->buf++ = w >> 8;
					dev->buf_len--;
				}
				if (omap_i2c_rev1() && !dev->buf_len)
					omap_i2c_complete_cmd(dev);
			} else
				err("RRDY IRQ while no data requested");
			writew(OMAP_I2C_STAT_RRDY, OMAP_I2C_STAT);
			if (omap_i2c_rev1())
				iv_read = readw(OMAP_I2C_IV);
			continue;
		}
		if (stat & OMAP_I2C_STAT_XRDY) {
                        w = 0;
			if (dev->buf_len) {
				w = *dev->buf++;
				dev->buf_len--;
				if (dev->buf_len) {
					w |= *dev->buf++ << 8;
					dev->buf_len--;
				}
			} else {
				err("XRDY IRQ while no data to send");
			}
			writew(w, OMAP_I2C_DATA);
                        /* We have to make sure the XRDY bit is reset */
			writew(OMAP_I2C_STAT_XRDY, OMAP_I2C_STAT);
			if (omap_i2c_rev1()) {
				iv_read = readw(OMAP_I2C_IV);
				if (!dev->buf_len)
					omap_i2c_complete_cmd(dev);
			}
			continue;
		}
		if (stat & OMAP_I2C_STAT_ROVR) {
			warn("Receive overrun");
                        dev->cmd_err |= OMAP_I2C_STAT_ROVR;
		}
		if (stat & OMAP_I2C_STAT_XUDF) {
			warn("Transmit overflow");
                        dev->cmd_err |= OMAP_I2C_STAT_XUDF;
		}
		if (stat & OMAP_I2C_STAT_NACK) {
                        dev->cmd_err |= OMAP_I2C_STAT_NACK;
                        omap_i2c_complete_cmd(dev);
			writew(OMAP_I2C_CON_STP, OMAP_I2C_CON);
		}
		if (stat & OMAP_I2C_STAT_AL) {
                        warn("Arbitration lost");
			dev->cmd_err |= OMAP_I2C_STAT_AL;
                        omap_i2c_complete_cmd(dev);
		}
		if (omap_i2c_rev1())
			iv_read = readw(OMAP_I2C_IV);

	}
	return IRQ_HANDLED;
}

static int omap_i2c_remove(struct device *dev)
{
        return 0;
}

static void omap_i2c_device_release(struct device *dev)
{
        /* Nothing */
}

static struct i2c_algorithm omap_i2c_algo = {
	.name = "OMAP I2C algorithm",
	.id = I2C_ALGO_EXP,
	.master_xfer = omap_i2c_xfer,
	.smbus_xfer = NULL,
	.slave_send = NULL,
	.slave_recv = NULL,
	.algo_control = NULL,
	.functionality = omap_i2c_func,
};

static struct i2c_adapter omap_i2c_adap = {
	.owner = THIS_MODULE,
	.name = "OMAP I2C adapter",
	.id = I2C_ALGO_EXP,	/* REVISIT: register for id */
	.algo = &omap_i2c_algo,
	.algo_data = NULL,
	.client_register = NULL,
	.client_unregister = NULL,
};

static struct device_driver omap_i2c_driver = {
        .name           = "omap_i2c",
        .bus            = &platform_bus_type,
        .remove         = omap_i2c_remove,
};

static struct platform_device omap_i2c_device = {
        .name           = "i2c",
        .id             = -1,
        .dev = {
                .driver         = &omap_i2c_driver,
                .release        = omap_i2c_device_release,
        },
};

static int __init
omap_i2c_init(void)
{
	int r;

	info("Driver ver. 1.3");
	DEB0("%s %s", __TIME__, __DATE__);

	if (clock > 200)
		clock = 400;	/*Fast mode */
	else
		clock = 100;	/*Standard mode */

	if (own < 1 || own > 0x7f)
		own = DEFAULT_OWN;

        memset(&omap_i2c_dev, 0, sizeof(omap_i2c_dev));
	init_waitqueue_head(&omap_i2c_dev.cmd_wait);

	r = (int) request_region(OMAP_I2C_BASE, OMAP_I2C_IOSIZE, MODULE_NAME);
	if (!r) {
		err("I2C is already in use");
		return -ENODEV;
	}

	r = request_irq(INT_I2C, omap_i2c_isr, 0, MODULE_NAME, &omap_i2c_dev);
	if (r) {
		err("failed to request I2C IRQ");
                goto do_release_region;
	}

	i2c_set_adapdata(&omap_i2c_adap, &omap_i2c_dev);
	r = i2c_add_adapter(&omap_i2c_adap);
	if (r) {
		err("failed to add adapter");
                goto do_free_irq;
		return r;
	}

	/* configure I/O pin multiplexing */
	/* FIXME: This should be done in bootloader */
	omap_cfg_reg(I2C_SCL);
	omap_cfg_reg(I2C_SDA);

	omap_i2c_reset();

	if (i2c_scan)
		omap_i2c_scan_bus(&omap_i2c_adap);
	if(driver_register(&omap_i2c_driver) != 0)
		printk(KERN_ERR "Driver register failed for omap_i2c\n");
	if(platform_device_register(&omap_i2c_device) != 0) {
		printk(KERN_ERR "Device register failed for i2c\n");
		driver_unregister(&omap_i2c_driver);
	}

	return 0;

do_free_irq:
        free_irq(INT_I2C, &omap_i2c_dev);
do_release_region:
        release_region(OMAP_I2C_BASE, OMAP_I2C_IOSIZE);

	return r;
}

static void __exit
omap_i2c_exit(void)
{
	i2c_del_adapter(&omap_i2c_adap);
	writew(0, OMAP_I2C_CON);
	free_irq(INT_I2C, &omap_i2c_dev);
	release_region(OMAP_I2C_BASE, OMAP_I2C_IOSIZE);
        driver_unregister(&omap_i2c_driver);
        platform_device_unregister(&omap_i2c_device);
}

MODULE_AUTHOR("MontaVista Software, Inc.");
MODULE_DESCRIPTION("TI OMAP I2C bus adapter");
MODULE_LICENSE("GPL");

module_param(clock, int, 0);
MODULE_PARM_DESC(clock,
		 "Set I2C clock in KHz: 100 (Standard Mode) or 400 (Fast Mode)");

module_param(own, int, 0);

module_param(i2c_scan, int, 0);
MODULE_PARM_DESC(i2c_scan, "Scan for active I2C clients on the bus");

#ifdef I2C_OMAP_DEBUG
module_param(i2c_debug, int, 0);
MODULE_PARM_DESC(i2c_debug,
		 "debug level - 0 off; 1 normal; 2,3 more verbose; "
		 "9 omap-protocol");
#endif

/* i2c may be needed to bring up other drivers */
subsys_initcall(omap_i2c_init);
module_exit(omap_i2c_exit);
