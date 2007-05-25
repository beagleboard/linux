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

#ifdef CONFIG_ARCH_OMAP3430
#define	get_cpu_rev()	2
#endif


void musb_platform_enable(struct musb *musb)
{
}
void musb_platform_disable(struct musb *musb)
{
}
static void omap_vbus_power(struct musb *musb, int is_on, int sleeping)
{
}

static void omap_set_vbus(struct musb *musb, int is_on)
{
	u8		devctl;
	/* HDRC controls CPEN, but beware current surges during device
	 * connect.  They can trigger transient overcurrent conditions
	 * that must be ignored.
	 */

	devctl = musb_readb(musb->pRegs, MGC_O_HDRC_DEVCTL);

	if (is_on) {
		musb->is_active = 1;
		musb->xceiv.default_a = 1;
		musb->xceiv.state = OTG_STATE_A_WAIT_VRISE;
		devctl |= MGC_M_DEVCTL_SESSION;

		MUSB_HST_MODE(musb);
	} else {
		musb->is_active = 0;

		/* NOTE:  we're skipping A_WAIT_VFALL -> A_IDLE and
		 * jumping right to B_IDLE...
		 */

		musb->xceiv.default_a = 0;
		musb->xceiv.state = OTG_STATE_B_IDLE;
		devctl &= ~MGC_M_DEVCTL_SESSION;

		MUSB_DEV_MODE(musb);
	}
	musb_writeb(musb->pRegs, MGC_O_HDRC_DEVCTL, devctl);

	DBG(1, "VBUS %s, devctl %02x "
		/* otg %3x conf %08x prcm %08x */ "\n",
		otg_state_string(musb),
		musb_readb(musb->pRegs, MGC_O_HDRC_DEVCTL));
}
static int omap_set_power(struct otg_transceiver *x, unsigned mA)
{
	return 0;
}

int musb_platform_resume(struct musb *musb);

int __init musb_platform_init(struct musb *musb)
{
#if defined(CONFIG_ARCH_OMAP2430)
	omap_cfg_reg(AE5_2430_USB0HS_STP);
	/* get the clock */
	musb->clock = clk_get((struct device *)musb->controller, "usbhs_ick");
#else
	musb->clock = clk_get((struct device *)musb->controller, "hsusb_ick");
#endif
	if(IS_ERR(musb->clock))
		return PTR_ERR(musb->clock);

	musb_platform_resume(musb);

	OTG_INTERFSEL_REG |= ULPI_12PIN;

	pr_debug("HS USB OTG: revision 0x%x, sysconfig 0x%02x, "
			"sysstatus 0x%x, intrfsel 0x%x, simenable  0x%x\n",
			OTG_REVISION_REG, OTG_SYSCONFIG_REG, OTG_SYSSTATUS_REG,
			OTG_INTERFSEL_REG, OTG_SIMENABLE_REG);

	omap_vbus_power(musb, musb->board_mode == MUSB_HOST, 1);


	if (is_host_enabled(musb))
		musb->board_set_vbus = omap_set_vbus;
	if (is_peripheral_enabled(musb))
		musb->xceiv.set_power = omap_set_power;

	return 0;
}

int musb_platform_suspend(struct musb *musb)
{
	/* in any role */
	OTG_FORCESTDBY_REG &= ~ENABLEFORCE; /* disable MSTANDBY */
	OTG_SYSCONFIG_REG &= FORCESTDBY;	/* enable force standby */
	OTG_SYSCONFIG_REG &= ~AUTOIDLE;		/* disable auto idle */
	OTG_SYSCONFIG_REG |= SMARTIDLE;		/* enable smart idle */
	OTG_FORCESTDBY_REG |= ENABLEFORCE; /* enable MSTANDBY */
	OTG_SYSCONFIG_REG |= AUTOIDLE;		/* enable auto idle */

	clk_disable(musb->clock);
	return 0;
}

int musb_platform_resume(struct musb *musb)
{
	clk_enable(musb->clock);

	OTG_FORCESTDBY_REG &= ~ENABLEFORCE; /* disable MSTANDBY */
	OTG_SYSCONFIG_REG |= SMARTSTDBY;	/* enable smart standby */
	OTG_SYSCONFIG_REG &= ~AUTOIDLE;		/* disable auto idle */
	OTG_SYSCONFIG_REG |= SMARTIDLE;		/* enable smart idle */
	OTG_SYSCONFIG_REG |= AUTOIDLE;		/* enable auto idle */

	return 0;
}


int musb_platform_exit(struct musb *musb)
{

	omap_vbus_power(musb, 0 /*off*/, 1);

	musb_platform_suspend(musb);

	clk_put(musb->clock);
	musb->clock = 0;

	return 0;
}
