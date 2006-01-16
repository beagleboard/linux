/*
 * linux/drivers/ssi/omap-tsc2101.c
 *
 * TSC2101 codec interface driver for the OMAP platform
 *
 * Copyright (C) 2004 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * History:
 *
 * 2004/11/07   Nishanth Menon - Modified for common hooks for Audio and Touchscreen
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/clk.h>

#include <asm/system.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <asm/mach-types.h>
#include <asm/hardware.h>

#include <asm/arch/mux.h>
#include <asm/arch/io.h>
#include <asm/arch/hardware.h>
#include <asm/hardware/tsc2101.h>
#include <asm/arch/gpioexpander.h>

#include "omap-tsc2101.h"

#if CONFIG_ARCH_OMAP16XX
#include <../drivers/ssi/omap-uwire.h>
#else
#error "Unsupported configuration"
#endif

#define SPIO 1

static int count;
static spinlock_t tsc2101_lock = SPIN_LOCK_UNLOCKED;
static struct clk  * tsc2101_mclk_ck;

static int omap_tsc2101_configure(void);

/* FIXME: add driver model usage to powerdown the tsc2101 on suspend */
/* Clock  -Hard coding for the time being */
#define CLK_SOFT_REQ_REG_BASE  (0xFFFE0800+0x34)
#define SOFT_COM_MCK0_REQ_MASK (0x1<<6)

int omap_tsc2101_enable(void)
{
	int ret = 0;

	spin_lock(&tsc2101_lock);
	if (count++ == 0) {
		int ret = 0;
		/* set the Mux to provide MCLK to TSC2101 */
		if (machine_is_omap_h3()) {
			ret = omap_cfg_reg(V5_1710_MCLK_ON);
		} else {
			if (machine_is_omap_h2()) {
				ret = omap_cfg_reg(R10_1610_MCLK_ON);
			}
		}

		/* Get the MCLK */
		tsc2101_mclk_ck = clk_get(NULL, "mclk");
		if (NULL == tsc2101_mclk_ck) {
			printk(KERN_ERR "Unable to get the clock MCLK!!!\n");;
			ret = -EPERM;
			goto done;
		}
		if (clk_set_rate(tsc2101_mclk_ck, 12000000)) {
			printk(KERN_ERR "Unable to set rate to the MCLK!!!\n");;
			ret = -EPERM;
			goto done;
		}
		clk_enable(tsc2101_mclk_ck);

		ret = omap_tsc2101_configure();

		/* Lock the module */
		if (!ret && !try_module_get(THIS_MODULE)) {
			printk(KERN_CRIT "Failed to get TSC module\n");
			ret = -ESTALE;
		}
	}

done:
	spin_unlock(&tsc2101_lock);
	return ret;
}

void omap_tsc2101_disable(void)
{
	spin_lock(&tsc2101_lock);
	if (--count == 0) {
		int ret = 0;
		/* Remove the Mux to Stop MCLK to TSC2101 */
		if (machine_is_omap_h3()) {
			ret = omap_cfg_reg(V5_1710_MCLK_OFF);
		} else {
			if (machine_is_omap_h2()) {
				ret = omap_cfg_reg(R10_1610_MCLK_OFF);
			}
		}

		/* Release the MCLK */
		clk_disable(tsc2101_mclk_ck);
		clk_put(tsc2101_mclk_ck);
		tsc2101_mclk_ck = NULL;

		module_put(THIS_MODULE);
	}
	spin_unlock(&tsc2101_lock);
}

void omap_tsc2101_write(int page, u8 address, u16 data)
{

	int ret = 0;

	if (machine_is_omap_h2()) {
		ret =
		    omap_uwire_data_transfer(1, 
					     (((page) << 11) | (address << 5)),
					     16, 0, NULL, 1);
		if (ret) {
			printk(KERN_ERR
			       "uwire-write returned error for address %x\n",
			       address);
			return;
		}
		ret = omap_uwire_data_transfer(1, data, 16, 0, NULL, 0);
		if (ret) {
			printk(KERN_ERR
			       "uwire-write returned error for address %x\n",
			       address);
			return;
		}
	}
	if (machine_is_omap_h3()) {

		ret =
		    omap_uwire_data_transfer(0, ((page << 11) | (address << 5)),
					     16, 0, NULL, 1);
		if (ret) {
			printk(KERN_ERR
			       "uwire-write returned error for address %x\n",
			       address);
			return;
		}
		ret = omap_uwire_data_transfer(0, data, 16, 0, NULL, 0);
		if (ret) {
			printk(KERN_ERR
			       "uwire-write returned error for address %x\n",
			       address);
			return;
		}
	}

}

void omap_tsc2101_reads(int page, u8 startaddress, u16 * data, int numregs)
{
	int cs = 0, i;
	if (machine_is_omap_h2()) {
		cs = 1;
	}
	if (machine_is_omap_h3()) {
		cs = 0;
	}
	(void)omap_uwire_data_transfer(cs, (0x8000 | (page << 11)
					    | (startaddress << 5)),
				       16, 0, NULL, 1);
	for (i = 0; i < (numregs - 1); i++, data++) {
		omap_uwire_data_transfer(cs, 0, 0, 16, data, 1);
	}
	omap_uwire_data_transfer(cs, 0, 0, 16, data, 0);
}

u16 omap_tsc2101_read(int page, u8 address)
{
	u16 ret;
	omap_tsc2101_reads(page, address, &ret, 1);
	return ret;
}

/* FIXME: adapt clock divisors for uwire to current ARM xor clock rate */
static int omap_tsc2101_configure(void)
{
	unsigned long uwire_flags = 0;

#ifdef CONFIG_MACH_OMAP_H3
	int err = 0;
	u8 ioExpanderVal = 0;

	if ((err = read_gpio_expa(&ioExpanderVal, 0x24))) {
		printk(" Error reading from I/O EXPANDER \n");
		return err;
	}
	ioExpanderVal |= 0x8;

	if ((err = write_gpio_expa(ioExpanderVal, 0x24))) {
		printk(KERN_ERR ": Error writing to I/O EXPANDER \n");
		return err;
	}
#endif

	if (machine_is_omap_h2()) {
		uwire_flags = UWIRE_READ_RISING_EDGE | UWIRE_WRITE_RISING_EDGE;
		omap_cfg_reg(N15_1610_UWIRE_CS1);
		omap_uwire_configure_mode(1, uwire_flags);
	}
	if (machine_is_omap_h3()) {
		uwire_flags = UWIRE_READ_RISING_EDGE | UWIRE_WRITE_RISING_EDGE;
		omap_cfg_reg(N14_1610_UWIRE_CS0);
		omap_uwire_configure_mode(0, uwire_flags);
	}

	/* Configure MCLK enable */
	omap_writel(omap_readl(PU_PD_SEL_2) | (1 << 22), PU_PD_SEL_2);	

	return 0;
}

EXPORT_SYMBOL(omap_tsc2101_enable);
EXPORT_SYMBOL(omap_tsc2101_read);
EXPORT_SYMBOL(omap_tsc2101_reads);
EXPORT_SYMBOL(omap_tsc2101_write);
EXPORT_SYMBOL(omap_tsc2101_disable);

MODULE_AUTHOR("Texas Instruments");
MODULE_DESCRIPTION
    ("Glue audio driver for the TI OMAP1610/OMAP1710 TSC2101 codec.");
MODULE_LICENSE("GPL");
