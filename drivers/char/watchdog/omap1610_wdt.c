/*
 * linux/drivers/char/omap1610_wdt.c
 *
 * Watchdog driver for the TI OMAP 16xx 32KHz (non-secure) watchdog
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
 *	1. Modified to support OMAP1610 32-KHz watchdog timer
 *	2. Ported to 2.6 kernel
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
#include <linux/moduleparam.h>

#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/hardware.h>
#include <asm/bitops.h>
#include <asm/hardware/clock.h>

#include "omap1610_wdt.h"


static unsigned timer_margin;
module_param(timer_margin, uint, 0);
MODULE_PARM_DESC(timer_margin, "initial watchdog timeout (in seconds)");

static int omap_wdt_users;
static struct clk *armwdt_ck = NULL;

static unsigned int wdt_trgr_pattern = 0x1234;

static void
omap_wdt_ping(void)
{
	while ((omap_readl(OMAP_WATCHDOG_WPS)) & 0x08) ;	/* wait for posted write to complete */
	wdt_trgr_pattern = ~wdt_trgr_pattern;
	omap_writel(wdt_trgr_pattern, (OMAP_WATCHDOG_TGR));
	while ((omap_readl(OMAP_WATCHDOG_WPS)) & 0x08) ;	/* wait for posted write to complete */
	/* reloaded WCRR from WLDR */
}

static void
omap_wdt_enable(void)
{
	/* Sequence to enable the watchdog */
	omap_writel(0xBBBB, OMAP_WATCHDOG_SPR);
	while ((omap_readl(OMAP_WATCHDOG_WPS)) & 0x10) ;
	omap_writel(0x4444, OMAP_WATCHDOG_SPR);
	while ((omap_readl(OMAP_WATCHDOG_WPS)) & 0x10) ;
}

static void
omap_wdt_disable(void)
{
	/* sequence required to disable watchdog */
	omap_writel(0xAAAA, OMAP_WATCHDOG_SPR);	/* TIMER_MODE */
	while (omap_readl(OMAP_WATCHDOG_WPS) & 0x10) ;
	omap_writel(0x5555, OMAP_WATCHDOG_SPR);	/* TIMER_MODE */
	while (omap_readl(OMAP_WATCHDOG_WPS) & 0x10) ;
}

static void
omap_wdt_adjust_timeout(unsigned new_timeout)
{
	if (new_timeout < TIMER_MARGIN_MIN)
		new_timeout = TIMER_MARGIN_DEFAULT;
	if (new_timeout > TIMER_MARGIN_MAX)
		new_timeout = TIMER_MARGIN_MAX;
	timer_margin = new_timeout;
}

static void
omap_wdt_set_timeout(void)
{
	u32	pre_margin = GET_WLDR_VAL(timer_margin);

	/* just count up at 32 KHz */
	while (omap_readl(OMAP_WATCHDOG_WPS) & 0x04) continue;
	omap_writel(pre_margin, OMAP_WATCHDOG_LDR);
	while (omap_readl(OMAP_WATCHDOG_WPS) & 0x04) continue;
}


/*
 *	Allow only one task to hold it open
 */

static int
omap_wdt_open(struct inode *inode, struct file *file)
{
	if (test_and_set_bit(1, (unsigned long *) &omap_wdt_users))
		return -EBUSY;

	clk_use(armwdt_ck);	/* Enable the clock */

	/* initialize prescaler */
	while (omap_readl(OMAP_WATCHDOG_WPS) & 0x01) continue;
	omap_writel((1 << 5) | (PTV << 2), OMAP_WATCHDOG_CNTRL);
	while (omap_readl(OMAP_WATCHDOG_WPS) & 0x01) continue;

	omap_wdt_set_timeout();
	omap_wdt_enable();
	return 0;
}

static int
omap_wdt_release(struct inode *inode, struct file *file)
{
	/*
	 *      Shut off the timer unless NOWAYOUT is defined.
	 */
#ifndef CONFIG_WATCHDOG_NOWAYOUT
	omap_wdt_disable();
	clk_unuse(armwdt_ck);	/* Disable the clock */
	clk_put(armwdt_ck);
	armwdt_ck = NULL;
#else
	printk(KERN_CRIT "omap1610_wdt: Unexpected close, not stopping!\n");
#endif
	omap_wdt_users = 0;
	return 0;
}

static ssize_t
omap_wdt_write(struct file *file, const char __user *data,
		size_t len, loff_t * ppos)
{
	/* Refresh LOAD_TIME. */
	if (len)
		omap_wdt_ping();
	return len;
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
		return copy_to_user((struct watchdog_info __user *) arg, &ident,
				    sizeof (ident));
	case WDIOC_GETSTATUS:
		return put_user(0, (int __user *) arg);
	case WDIOC_GETBOOTSTATUS:
		return put_user(omap_readw(ARM_SYSST), (int __user *) arg);
	case WDIOC_KEEPALIVE:
		omap_wdt_ping();
		return 0;
	case WDIOC_SETTIMEOUT:
		if (get_user(new_margin, (int __user *) arg))
			return -EFAULT;
		omap_wdt_adjust_timeout(new_margin);

		omap_wdt_disable();
		omap_wdt_set_timeout();
		omap_wdt_enable();

		omap_wdt_ping();
		/* Fall */
	case WDIOC_GETTIMEOUT:
		return put_user(timer_margin, (int __user *) arg);
	}
}

static struct file_operations omap_wdt_fops = {
	.owner		= THIS_MODULE,
	.write		= omap_wdt_write,
	.ioctl		= omap_wdt_ioctl,
	.open		= omap_wdt_open,
	.release	= omap_wdt_release,
};

static struct miscdevice omap_wdt_miscdev = {
	.minor		= WATCHDOG_MINOR,
	.name		= "watchdog",
	.fops		= &omap_wdt_fops
};

static int __init
omap_wdt_init(void)
{
	int ret;

	omap_wdt_users = 0;
	armwdt_ck = clk_get(NULL, "armwdt_ck");
	if (IS_ERR(armwdt_ck)) {
		ret = PTR_ERR(armwdt_ck);
		armwdt_ck = NULL;
		goto fail;
	}

	omap_wdt_disable();
	omap_wdt_adjust_timeout(timer_margin);

	ret = misc_register(&omap_wdt_miscdev);
	if (ret)
		goto fail;

	pr_info("OMAP Watchdog Timer: initial timeout %d sec\n", timer_margin);

	/* autogate OCP interface clock */
	omap_writel(0x01, OMAP_WATCHDOG_SYS_CONFIG);
	return 0;

fail:
	if (armwdt_ck)
		clk_put(armwdt_ck);
	return ret;
}

static void __exit
omap_wdt_exit(void)
{
	misc_deregister(&omap_wdt_miscdev);
	clk_put(armwdt_ck);
}

module_init(omap_wdt_init);
module_exit(omap_wdt_exit);

MODULE_AUTHOR("George G. Davis");
MODULE_LICENSE("GPL");
MODULE_ALIAS_MISCDEV(WATCHDOG_MINOR);
