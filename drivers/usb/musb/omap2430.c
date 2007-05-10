/*
 * Copyright (C) 2005-2006 by Texas Instruments
 *
 * This file is part of the Inventra Controller Driver for Linux.
 *
 * The Inventra Controller Driver for Linux is free software; you
 * can redistribute it and/or modify it under the terms of the GNU
 * General Public License version 2 as published by the Free Software
 * Foundation.
 *
 * The Inventra Controller Driver for Linux is distributed in
 * the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public
 * License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with The Inventra Controller Driver for Linux ; if not,
 * write to the Free Software Foundation, Inc., 59 Temple Place,
 * Suite 330, Boston, MA  02111-1307  USA
 *
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/clk.h>

#include <asm/io.h>
#include <asm/mach-types.h>
#include <asm/arch/hardware.h>
#include <asm/arch/mux.h>

#include "musbdefs.h"
#include "omap2430.h"


static int dma_off;

void musb_platform_enable(struct musb *musb)
{
	if (is_dma_capable() && dma_off)
		printk(KERN_WARNING "%s %s: dma not reactivated\n",
				__FILE__, __FUNCTION__);
	else
		dma_off = 1;
}

void musb_platform_disable(struct musb *musb)
{
	if (is_dma_capable()) {
		printk(KERN_WARNING "%s %s: dma still active\n",
				__FILE__, __FUNCTION__);
		dma_off = 1;
	}
}

static void omap_vbus_power(struct musb *musb, int is_on, int sleeping)
{
}

static void omap_set_vbus(struct musb *musb, int is_on)
{
	WARN_ON(is_on && is_peripheral_active(musb));
	return omap_vbus_power(musb, is_on, 0);
}

int __init musb_platform_init(struct musb *musb)
{
	/* Erratum - reset value of STP has pull-down.
	   Change it to pull-up. */
	omap_cfg_reg(AE5_2430_USB0HS_STP);

	/* start clock */
	musb->clock = clk_get((struct device *)musb->controller, "usbhs_ick");
	clk_enable(musb->clock);

	omap_writel(omap_readl(OTG_INTERFSEL) | (1<<0), OTG_INTERFSEL);
	omap_writel(omap_readl(OTG_SYSCONFIG) |
		    ((1 << 12) | (1 << 3) | (1 << 2)),
		    OTG_SYSCONFIG);

	pr_debug("HS USB OTG: revision 0x%x, sysconfig 0x%02x, "
			"sysstatus 0x%x, intrfsel 0x%x, simenable  0x%x\n",
			omap_readl(OTG_REVISION), omap_readl(OTG_SYSCONFIG),
			omap_readl(OTG_SYSSTATUS), omap_readl(OTG_INTERFSEL),
			omap_readl(OTG_SIMENABLE));

	musb->board_set_vbus = omap_set_vbus;
	omap_vbus_power(musb, musb->board_mode == MUSB_HOST, 1);

	return 0;
}

int musb_platform_exit(struct musb *musb)
{
	omap_vbus_power(musb, 0 /*off*/, 1);
	clk_disable(musb->clock);

	return 0;
}
