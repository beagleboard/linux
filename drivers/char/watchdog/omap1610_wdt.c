/*
 * linux/drivers/char/omap1610_wdt.c
 *
 * Watchdog driver for the TI OMAP
 *
 * Author: MontaVista Software, Inc.
 *         <gdavis@mvista.com> or <source@mvista.com>
 *
 * 2003 (c) MontaVista Software, Inc. This file is licensed under the
 * terms of the GNU General Public License version 2. This program is
 * licensed "as is" without any warranty of any kind, whether express
 * or implied.
 *
 * History:
 *
 * 20030527: George G. Davis <gdavis@mvista.com>
 *	Initially based on linux-2.4.19-rmk7-pxa1/drivers/char/sa1100_wdt.c
 *	(c) Copyright 2000 Oleg Drokin <green@crimea.edu>
 *	Based on SoftDog driver by Alan Cox <alan@redhat.com>
 *
 * Copyright (c) 2004 Texas Instruments.
 *
 * 1. Modified to support OMAP1610 32-KHz watchdog timer
 * 2. Ported to 2.6 kernel
 *
 *
 * TODO:
 * 1. Need to disable watchdog when entering chip idle mode.
 */

#include <linux/module.h>
#include <linux/config.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/miscdevice.h>
#include <linux/watchdog.h>
#include <linux/reboot.h>
#include <linux/smp_lock.h>
#include <linux/init.h>
#include <linux/err.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/hardware.h>
#include <asm/bitops.h>
#include <asm/hardware/clock.h>

#include <linux/moduleparam.h>
#include "omap1610_wdt.h"

static int timer_margin;	/* in seconds */
static int pre_margin;
static int omap_wdt_users;
static struct clk *armwdt_ck = 0;
static struct miscdevice omap_wdt_miscdev; /* Forward declaration */

static unsigned int wdt_trgr_pattern = 0x1234;

static void
omap_wdt_ping(void)
{
	while ((omap_readl(OMAP_WATCHDOG_WPS)) & 0x08) ;	/* wait for posted write to complete */
	wdt_trgr_pattern = ~wdt_trgr_pattern;
	omap_writel(wdt_trgr_pattern, (OMAP_WATCHDOG_TGR));
	while ((omap_readl(OMAP_WATCHDOG_WPS)) & 0x08) ;	/* wait for posted write to complete */
	return;
}

static void
omap_wdt_enable(void)
{
	/* Sequence to enable the watchdog */
	omap_writel(0xBBBB, OMAP_WATCHDOG_SPR);
	while ((omap_readl(OMAP_WATCHDOG_WPS)) & 0x10) ;
	omap_writel(0x4444, OMAP_WATCHDOG_SPR);
	while ((omap_readl(OMAP_WATCHDOG_WPS)) & 0x10) ;
	return;
}

static void
omap_wdt_disable(void)
{
	/* sequence required to disable watchdog */
	omap_writel(0xAAAA, OMAP_WATCHDOG_SPR);	/* TIMER_MODE */
	while (omap_readl(OMAP_WATCHDOG_WPS) & 0x10) ;
	omap_writel(0x5555, OMAP_WATCHDOG_SPR);	/* TIMER_MODE */
	while (omap_readl(OMAP_WATCHDOG_WPS) & 0x10) ;
	return;
}

/*
 *	Allow only one person to hold it open
 */

static int
omap_wdt_open(struct inode *inode, struct file *file)
{
	if (test_and_set_bit(1, (unsigned long *) &omap_wdt_users))
		return -EBUSY;

	if (armwdt_ck == 0 || IS_ERR(armwdt_ck)) {
		armwdt_ck = clk_get(omap_wdt_miscdev.dev, "armwdt_ck");
		if (IS_ERR(armwdt_ck)) {
			omap_wdt_users = 0;
			return PTR_ERR(armwdt_ck);
		}
		clk_use(armwdt_ck);	/* Enable the clock */
	}

	omap_wdt_enable();

	return 0;
}

static int
omap_wdt_release(struct inode *inode, struct file *file)
{
	/*
	 *      Shut off the timer.
	 *      Lock it in if it's a module and we defined ...NOWAYOUT
	 */
#ifndef CONFIG_WATCHDOG_NOWAYOUT
	omap_wdt_disable();
	clk_unuse(armwdt_ck);	/* Disable the clock */
	clk_put(armwdt_ck);
	armwdt_ck = 0;
#endif
	omap_wdt_users = 0;
	return 0;
}

static ssize_t
omap_wdt_write(struct file *file, const char *data, size_t len, loff_t * ppos)
{
	/*  Can't seek (pwrite) on this device  */
	if (ppos != &file->f_pos)
		return -ESPIPE;

	/* Refresh LOAD_TIME. */
	if (len) {
		omap_wdt_ping();
		return 1;
	}
	return 0;
}

static int
omap_wdt_ioctl(struct inode *inode, struct file *file,
	       unsigned int cmd, unsigned long arg)
{
	int new_margin;
	static struct watchdog_info ident = {
		.identity = "OMAP Watchdog",
		.options = WDIOF_SETTIMEOUT,
		.firmware_version = 0,
	};

	switch (cmd) {
	default:
		return -ENOIOCTLCMD;
	case WDIOC_GETSUPPORT:
		return copy_to_user((struct watchdog_info *) arg, &ident,
				    sizeof (ident));
	case WDIOC_GETSTATUS:
		return put_user(0, (int *) arg);
	case WDIOC_GETBOOTSTATUS:
		return put_user(omap_readw(ARM_SYSST), (int *) arg);
	case WDIOC_KEEPALIVE:
		omap_wdt_ping();
		return 0;
	case WDIOC_SETTIMEOUT:
		if (get_user(new_margin, (int *) arg))
			return -EFAULT;
		if ((new_margin < TIMER_MARGIN_MIN)
		    || (new_margin > TIMER_MARGIN_MAX))
			timer_margin = TIMER_MARGIN_MAX;	/* default timeout */
		else
			timer_margin = new_margin;
		pre_margin = GET_WLDR_VAL(timer_margin);

		omap_wdt_disable();

		while ((omap_readl(OMAP_WATCHDOG_WPS)) & 0x04) ;	/* wait for posted write to complete */
		omap_writel(pre_margin, (OMAP_WATCHDOG_LDR));
		while ((omap_readl(OMAP_WATCHDOG_WPS)) & 0x04) ;	/* wait for posted write to complete */

		omap_wdt_enable();

		omap_wdt_ping();
		/* Fall */
	case WDIOC_GETTIMEOUT:
		return put_user(timer_margin, (int *) arg);
	}
}

static struct file_operations omap_wdt_fops = {
	.owner = THIS_MODULE,
	.write = omap_wdt_write,
	.ioctl = omap_wdt_ioctl,
	.open = omap_wdt_open,
	.release = omap_wdt_release,
};

static struct miscdevice omap_wdt_miscdev = {
	.minor = WATCHDOG_MINOR,
	.name = "omap_wdt",
	.fops = &omap_wdt_fops
};

static int __init
omap_wdt_init(void)
{
	int ret;

	ret = misc_register(&omap_wdt_miscdev);

	if (ret)
		return ret;

	omap_wdt_disable();

	if (timer_margin < 1 || timer_margin > 32)
		timer_margin = 32;

	printk(KERN_INFO "%s: TI OMAP Watchdog Timer: timer margin %d sec\n",
	       omap_wdt_miscdev.name, timer_margin);

	return 0;
}

static void __exit
omap_wdt_exit(void)
{
	misc_deregister(&omap_wdt_miscdev);
}

module_init(omap_wdt_init);
module_exit(omap_wdt_exit);

MODULE_AUTHOR("George G. Davis");
MODULE_LICENSE("GPL");
module_param(timer_margin, int, 0);
