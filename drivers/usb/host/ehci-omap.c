/*
 * ehci-omap.c - driver for USBHOST on OMAP 34xx processor
 *
 * Bus Glue for OMAP34xx USBHOST 3 port EHCI controller
 * Tested on OMAP3430 ES2.0 SDP
 *
 * Copyright (C) 2007-2008 Texas Instruments, Inc.
 *	Author: Vikram Pandita <vikram.pandita@ti.com>
 *
 * Based on "ehci-fsl.c" and "ehci-au1xxx.c" ehci glue layers
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

#include <linux/platform_device.h>
#include <linux/clk.h>
#include <mach/gpio.h>

#include "ehci-omap.h"


#ifdef CONFIG_OMAP_EHCI_PHY_MODE
/* EHCI connected to External PHY */

/* External USB connectivity board: 750-2083-001
 * Connected to OMAP3430 SDP
 * The board has Port1 and Port2 connected to ISP1504 in 12-pin ULPI mode
 */

/* ISSUE1:
 *      ISP1504 for input clocking mode needs special reset handling
 *	Hold the PHY in reset by asserting RESET_N signal
 *	Then start the 60Mhz clock input to PHY
 *	Release the reset after a delay -
 *		to get the PHY state machine in working state
 */
#define EXTERNAL_PHY_RESET
#define	EXT_PHY_RESET_GPIO_PORT1	(57)
#define	EXT_PHY_RESET_GPIO_PORT2	(61)
#define	EXT_PHY_RESET_DELAY		(10)

/* ISSUE2:
 * USBHOST supports External charge pump PHYs only
 * Use the VBUS from Port1 to power VBUS of Port2 externally
 * So use Port2 as the working ULPI port
 */
#define VBUS_INTERNAL_CHARGEPUMP_HACK

#endif /* CONFIG_OMAP_EHCI_PHY_MODE */

/*-------------------------------------------------------------------------*/

/* Define USBHOST clocks for clock management */
struct ehci_omap_clock_defs {
	struct clk	*usbhost_ick_clk;
	struct clk	*usbhost2_120m_fck_clk;
	struct clk	*usbhost1_48m_fck_clk;
	struct clk	*usbtll_fck_clk;
	struct clk	*usbtll_ick_clk;
};

/* Clock names as per clock framework: May change so keep as #defs */
#define USBHOST_ICKL		"usbhost_ick"
#define USBHOST_120M_FCLK	"usbhost_120m_fck"
#define USBHOST_48M_FCLK	"usbhost_48m_fck"
#define USBHOST_TLL_ICKL	"usbtll_ick"
#define USBHOST_TLL_FCLK	"usbtll_fck"
/*-------------------------------------------------------------------------*/


#ifndef CONFIG_OMAP_EHCI_PHY_MODE

static void omap_usb_utmi_init(struct usb_hcd *hcd, u8 tll_channel_mask)
{
	int i;

	/* Use UTMI Ports of TLL */
	omap_writel((1 << OMAP_UHH_HOSTCONFIG_ULPI_BYPASS_SHIFT)|
			(1<<OMAP_UHH_HOSTCONFIG_INCR4_BURST_EN_SHIFT)|
			(1<<OMAP_UHH_HOSTCONFIG_INCR8_BURST_EN_SHIFT)|
			(1<<OMAP_UHH_HOSTCONFIG_INCR16_BURST_EN_SHIFT)|
			(0<<OMAP_UHH_HOSTCONFIG_INCRX_ALIGN_EN_SHIFT),
						OMAP_UHH_HOSTCONFIG);
	/* Enusre bit is set */
	while (!(omap_readl(OMAP_UHH_HOSTCONFIG)
			& (1 << OMAP_UHH_HOSTCONFIG_ULPI_BYPASS_SHIFT)))
		cpu_relax();

	dev_dbg(hcd->self.controller, "\nEntered UTMI MODE: success\n");

	/* Program the 3 TLL channels upfront */

	for (i = 0; i < OMAP_TLL_CHANNEL_COUNT; i++) {

		/* Disable AutoIdle */
		omap_writel(omap_readl(OMAP_TLL_CHANNEL_CONF(i)) &
			    ~(1<<OMAP_TLL_CHANNEL_CONF_UTMIAUTOIDLE_SHIFT),
			    OMAP_TLL_CHANNEL_CONF(i));
		/* Disable BitStuffing */
		omap_writel(omap_readl(OMAP_TLL_CHANNEL_CONF(i)) &
			    ~(1<<OMAP_TLL_CHANNEL_CONF_ULPINOBITSTUFF_SHIFT),
			    OMAP_TLL_CHANNEL_CONF(i));
		/* SDR Mode */
		omap_writel(omap_readl(OMAP_TLL_CHANNEL_CONF(i)) &
			    ~(1<<OMAP_TLL_CHANNEL_CONF_ULPIDDRMODE_SHIFT),
			    OMAP_TLL_CHANNEL_CONF(i));

	}

	/* Program Common TLL register */
	omap_writel((1 << OMAP_TLL_SHARED_CONF_FCLK_IS_ON_SHIFT) |
			(1 << OMAP_TLL_SHARED_CONF_USB_DIVRATION_SHIFT) |
			(0 << OMAP_TLL_SHARED_CONF_USB_180D_SDR_EN_SHIFT) |
			(0 << OMAP_TLL_SHARED_CONF_USB_90D_DDR_EN_SHFT),
				OMAP_TLL_SHARED_CONF);

	/* Enable channels now */
	for (i = 0; i < OMAP_TLL_CHANNEL_COUNT; i++) {

		/* Enable only the channel that is needed */
		if (!(tll_channel_mask & 1<<i))
			continue;

		omap_writel(omap_readl(OMAP_TLL_CHANNEL_CONF(i)) |
			    (1<<OMAP_TLL_CHANNEL_CONF_CHANEN_SHIFT),
			    OMAP_TLL_CHANNEL_CONF(i));

		omap_writeb(0xBE, OMAP_TLL_ULPI_SCRATCH_REGISTER(i));
		dev_dbg(hcd->self.controller, "\nULPI_SCRATCH_REG[ch=%d]"
			"= 0x%02x\n",
			i+1, omap_readb(OMAP_TLL_ULPI_SCRATCH_REGISTER(i)));
	}
}

#else
# define omap_usb_utmi_init(x, y)	0
#endif


/* omap_start_ehc
 *	- Start the TI USBHOST controller
 */
static int omap_start_ehc(struct platform_device *dev, struct usb_hcd *hcd)
{
	struct ehci_omap_clock_defs *ehci_clocks;

	dev_dbg(hcd->self.controller, "starting TI EHCI USB Controller\n");

	ehci_clocks = (struct ehci_omap_clock_defs *)(
				((char *)hcd_to_ehci(hcd)) +
					sizeof(struct ehci_hcd));

	/* Start DPLL5 Programming:
	 * Clock Framework is not doing this now:
	 * This will be done in clock framework later
	 */
	/* Enable DPLL 5 : Based on Input of 13Mhz*/
	cm_write_mod_reg((12 << OMAP3430ES2_PERIPH2_DPLL_DIV_SHIFT)|
			(120 << OMAP3430ES2_PERIPH2_DPLL_MULT_SHIFT),
			PLL_MOD, OMAP3430ES2_CM_CLKSEL4);

	cm_write_mod_reg(1 << OMAP3430ES2_DIV_120M_SHIFT,
			PLL_MOD, OMAP3430ES2_CM_CLKSEL5);

	cm_write_mod_reg((7 << OMAP3430ES2_PERIPH2_DPLL_FREQSEL_SHIFT) |
			(7 << OMAP3430ES2_EN_PERIPH2_DPLL_SHIFT),
			PLL_MOD, OMAP3430ES2_CM_CLKEN2);

	while (!(cm_read_mod_reg(PLL_MOD, CM_IDLEST2) &
				OMAP3430ES2_ST_PERIPH2_CLK_MASK))
		dev_dbg(hcd->self.controller,
			"idlest2 = 0x%x\n",
			cm_read_mod_reg(PLL_MOD, CM_IDLEST2));
	/* End DPLL5 programming */


	/* PRCM settings for USBHOST:
	 * Interface clk un-related to domain transition
	 */
	cm_write_mod_reg(0 << OMAP3430ES2_AUTO_USBHOST_SHIFT,
				OMAP3430ES2_USBHOST_MOD, CM_AUTOIDLE);

	/* Disable sleep dependency with MPU and IVA */
	cm_write_mod_reg((0 << OMAP3430ES2_EN_MPU_SHIFT) |
				(0 << OMAP3430ES2_EN_IVA2_SHIFT),
				OMAP3430ES2_USBHOST_MOD, OMAP3430_CM_SLEEPDEP);

	/* Disable Automatic transition of clock */
	cm_write_mod_reg(0 << OMAP3430ES2_CLKTRCTRL_USBHOST_SHIFT,
				OMAP3430ES2_USBHOST_MOD, CM_CLKSTCTRL);

	/* Enable Clocks for USBHOST */
	ehci_clocks->usbhost_ick_clk = clk_get(&dev->dev,
						USBHOST_ICKL);
	if (IS_ERR(ehci_clocks->usbhost_ick_clk))
		return PTR_ERR(ehci_clocks->usbhost_ick_clk);
	clk_enable(ehci_clocks->usbhost_ick_clk);


	ehci_clocks->usbhost2_120m_fck_clk = clk_get(&dev->dev,
							USBHOST_120M_FCLK);
	if (IS_ERR(ehci_clocks->usbhost2_120m_fck_clk))
		return PTR_ERR(ehci_clocks->usbhost2_120m_fck_clk);
	clk_enable(ehci_clocks->usbhost2_120m_fck_clk);

	ehci_clocks->usbhost1_48m_fck_clk = clk_get(&dev->dev,
						USBHOST_48M_FCLK);
	if (IS_ERR(ehci_clocks->usbhost1_48m_fck_clk))
		return PTR_ERR(ehci_clocks->usbhost1_48m_fck_clk);
	clk_enable(ehci_clocks->usbhost1_48m_fck_clk);


#ifdef EXTERNAL_PHY_RESET
	/* Refer: ISSUE1 */
	gpio_request(EXT_PHY_RESET_GPIO_PORT1, "USB1 PHY reset");
	gpio_direction_output(EXT_PHY_RESET_GPIO_PORT1, 0);
	gpio_request(EXT_PHY_RESET_GPIO_PORT2, "USB2 PHY reset");
	gpio_direction_output(EXT_PHY_RESET_GPIO_PORT2, 0);
	/* Hold the PHY in RESET for enough time till DIR is high */
	udelay(EXT_PHY_RESET_DELAY);
#endif

	/* Configure TLL for 60Mhz clk for ULPI */
	ehci_clocks->usbtll_fck_clk = clk_get(&dev->dev, USBHOST_TLL_FCLK);
	if (IS_ERR(ehci_clocks->usbtll_fck_clk))
		return PTR_ERR(ehci_clocks->usbtll_fck_clk);
	clk_enable(ehci_clocks->usbtll_fck_clk);

	ehci_clocks->usbtll_ick_clk = clk_get(&dev->dev, USBHOST_TLL_ICKL);
	if (IS_ERR(ehci_clocks->usbtll_ick_clk))
		return PTR_ERR(ehci_clocks->usbtll_ick_clk);
	clk_enable(ehci_clocks->usbtll_ick_clk);

	/* Disable Auto Idle of USBTLL */
	cm_write_mod_reg((0 << OMAP3430ES2_AUTO_USBTLL_SHIFT),
				CORE_MOD, CM_AUTOIDLE3);

	/* Wait for TLL to be Active */
	while ((cm_read_mod_reg(CORE_MOD, OMAP2430_CM_IDLEST3)
			& (1 << OMAP3430ES2_ST_USBTLL_SHIFT)))
		cpu_relax();

	/* perform TLL soft reset, and wait until reset is complete */
	omap_writel(1 << OMAP_USBTLL_SYSCONFIG_SOFTRESET_SHIFT,
			OMAP_USBTLL_SYSCONFIG);
	/* Wait for TLL reset to complete */
	while (!(omap_readl(OMAP_USBTLL_SYSSTATUS)
			& (1 << OMAP_USBTLL_SYSSTATUS_RESETDONE_SHIFT)))
		cpu_relax();

	dev_dbg(hcd->self.controller, "TLL RESET DONE\n");

	/* (1<<3) = no idle mode only for initial debugging */
	omap_writel((1 << OMAP_USBTLL_SYSCONFIG_ENAWAKEUP_SHIFT) |
			(1 << OMAP_USBTLL_SYSCONFIG_SIDLEMODE_SHIFT) |
			(1 << OMAP_USBTLL_SYSCONFIG_CACTIVITY_SHIFT),
			OMAP_USBTLL_SYSCONFIG);


	/* Put UHH in NoIdle/NoStandby mode */
	omap_writel((0 << OMAP_UHH_SYSCONFIG_AUTOIDLE_SHIFT) |
			(1 << OMAP_UHH_SYSCONFIG_ENAWAKEUP_SHIFT) |
			(1 << OMAP_UHH_SYSCONFIG_SIDLEMODE_SHIFT) |
			(1 << OMAP_UHH_SYSCONFIG_CACTIVITY_SHIFT) |
			(1 << OMAP_UHH_SYSCONFIG_MIDLEMODE_SHIFT),
			OMAP_UHH_SYSCONFIG);

#ifdef CONFIG_OMAP_EHCI_PHY_MODE
	/* Bypass the TLL module for PHY mode operation */
	omap_writel((0 << OMAP_UHH_HOSTCONFIG_ULPI_BYPASS_SHIFT)|
			(1<<OMAP_UHH_HOSTCONFIG_INCR4_BURST_EN_SHIFT)|
			(1<<OMAP_UHH_HOSTCONFIG_INCR8_BURST_EN_SHIFT)|
			(1<<OMAP_UHH_HOSTCONFIG_INCR16_BURST_EN_SHIFT)|
			(0<<OMAP_UHH_HOSTCONFIG_INCRX_ALIGN_EN_SHIFT),
						OMAP_UHH_HOSTCONFIG);
	/* Ensure that BYPASS is set */
	while (omap_readl(OMAP_UHH_HOSTCONFIG)
			& (1 << OMAP_UHH_HOSTCONFIG_ULPI_BYPASS_SHIFT))
		cpu_relax();

	dev_dbg(hcd->self.controller, "Entered ULPI PHY MODE: success\n");

#else
	/* Enable UTMI mode for all 3 TLL channels */
	omap_usb_utmi_init(hcd,
		OMAP_TLL_CHANNEL_1_EN_MASK |
		OMAP_TLL_CHANNEL_2_EN_MASK |
		OMAP_TLL_CHANNEL_3_EN_MASK
		);
#endif

#ifdef EXTERNAL_PHY_RESET
	/* Refer ISSUE1:
	 * Hold the PHY in RESET for enough time till PHY is settled and ready
	 */
	udelay(EXT_PHY_RESET_DELAY);
	gpio_set_value(EXT_PHY_RESET_GPIO_PORT1, 1);
	gpio_set_value(EXT_PHY_RESET_GPIO_PORT2, 1);
#endif

#ifdef VBUS_INTERNAL_CHARGEPUMP_HACK
	/* Refer ISSUE2: LINK assumes external charge pump */

	/* use Port1 VBUS to charge externally Port2:
	 *	So for PHY mode operation use Port2 only
	 */
	omap_writel((0xA << EHCI_INSNREG05_ULPI_REGADD_SHIFT) |/* OTG ctrl reg*/
			(2 << EHCI_INSNREG05_ULPI_OPSEL_SHIFT) |/*   Write */
			(1 << EHCI_INSNREG05_ULPI_PORTSEL_SHIFT) |/* Port1 */
			(1 << EHCI_INSNREG05_ULPI_CONTROL_SHIFT) |/* Start */
			(0x26),
			EHCI_INSNREG05_ULPI);

	while (!(omap_readl(EHCI_INSNREG05_ULPI)
			& (1<<EHCI_INSNREG05_ULPI_CONTROL_SHIFT)))
		cpu_relax();

#endif

	return 0;
}

/*-------------------------------------------------------------------------*/

static void omap_stop_ehc(struct platform_device *dev, struct usb_hcd *hcd)
{
	struct ehci_omap_clock_defs *ehci_clocks;

	ehci_clocks = (struct ehci_omap_clock_defs *)
			(((char *)hcd_to_ehci(hcd)) + sizeof(struct ehci_hcd));

	dev_dbg(hcd->self.controller, "stopping TI EHCI USB Controller\n");

	/* Reset OMAP modules for insmod/rmmod to work */
	omap_writel((1<<1), OMAP_UHH_SYSCONFIG);
	while (!(omap_readl(OMAP_UHH_SYSSTATUS) & (1<<0)))
		cpu_relax();
	while (!(omap_readl(OMAP_UHH_SYSSTATUS) & (1<<1)))
		cpu_relax();
	while (!(omap_readl(OMAP_UHH_SYSSTATUS) & (1<<2)))
		cpu_relax();
	dev_dbg(hcd->self.controller,
		"UHH RESET DONE OMAP_UHH_SYSSTATUS %x !!\n",
			omap_readl(OMAP_UHH_SYSSTATUS));

	omap_writel((1<<1), OMAP_USBTLL_SYSCONFIG);
	while (!(omap_readl(OMAP_USBTLL_SYSSTATUS) & (1<<0)))
		cpu_relax();
	dev_dbg(hcd->self.controller, "TLL RESET DONE\n");

	if (ehci_clocks->usbtll_fck_clk != NULL) {
		clk_disable(ehci_clocks->usbtll_fck_clk);
		clk_put(ehci_clocks->usbtll_fck_clk);
		ehci_clocks->usbtll_fck_clk = NULL;
	}

	if (ehci_clocks->usbhost_ick_clk != NULL) {
		clk_disable(ehci_clocks->usbhost_ick_clk);
		clk_put(ehci_clocks->usbhost_ick_clk);
		ehci_clocks->usbhost_ick_clk = NULL;
	}

	if (ehci_clocks->usbhost1_48m_fck_clk != NULL) {
		clk_disable(ehci_clocks->usbhost1_48m_fck_clk);
		clk_put(ehci_clocks->usbhost1_48m_fck_clk);
		ehci_clocks->usbhost1_48m_fck_clk = NULL;
	}

	if (ehci_clocks->usbhost2_120m_fck_clk != NULL) {
		clk_disable(ehci_clocks->usbhost2_120m_fck_clk);
		clk_put(ehci_clocks->usbhost2_120m_fck_clk);
		ehci_clocks->usbhost2_120m_fck_clk = NULL;
	}

	if (ehci_clocks->usbtll_ick_clk != NULL) {
		clk_disable(ehci_clocks->usbtll_ick_clk);
		clk_put(ehci_clocks->usbtll_ick_clk);
		ehci_clocks->usbtll_ick_clk = NULL;
	}


#ifdef EXTERNAL_PHY_RESET
	gpio_free(EXT_PHY_RESET_GPIO_PORT1);
	gpio_free(EXT_PHY_RESET_GPIO_PORT2);
#endif

	dev_dbg(hcd->self.controller,
		"Clock to USB host has been disabled\n");
}

static const struct hc_driver ehci_omap_hc_driver;

/*-------------------------------------------------------------------------*/
/* configure so an HC device and id are always provided */
/* always called with process context; sleeping is OK */

/**
 * ehci_hcd_omap_drv_probe - initialize TI-based HCDs
 * Context: !in_interrupt()
 *
 * Allocates basic resources for this USB host controller, and
 * then invokes the start() method for the HCD associated with it
 * through the hotplug entry's driver_data.
 *
 */
static int ehci_hcd_omap_drv_probe(struct platform_device *dev)
{
	int retval = 0;
	struct usb_hcd *hcd;
	struct ehci_hcd *ehci;

	dev_dbg(&dev->dev, "ehci_hcd_omap_drv_probe()\n");

	if (usb_disabled())
		return -ENODEV;

	if (dev->resource[1].flags != IORESOURCE_IRQ) {
		dev_dbg(&dev->dev, "resource[1] is not IORESOURCE_IRQ\n");
		retval = -ENOMEM;
	}

	hcd = usb_create_hcd(&ehci_omap_hc_driver, &dev->dev,
			dev_name(&dev->dev));
	if (!hcd)
		return -ENOMEM;

	retval = omap_start_ehc(dev, hcd);
	if (retval)
		return retval;

	hcd->rsrc_start = 0;
	hcd->rsrc_len = 0;
	hcd->rsrc_start = dev->resource[0].start;
	hcd->rsrc_len = dev->resource[0].end - dev->resource[0].start + 1;

	hcd->regs = ioremap(hcd->rsrc_start, hcd->rsrc_len);
	if (!hcd->regs) {
		dev_err(&dev->dev, "ioremap failed\n");
		return -ENOMEM;
	}

	ehci = hcd_to_ehci(hcd);
	ehci->caps = hcd->regs;

	ehci->sbrn = 0x20;

	ehci->regs = hcd->regs + HC_LENGTH(readl(&ehci->caps->hc_capbase));
	/* cache this readonly data; minimize chip reads */
	ehci->hcs_params = readl(&ehci->caps->hcs_params);

	/* SET 1 micro-frame Interrupt interval */
	writel(readl(&ehci->regs->command) | (1<<16), &ehci->regs->command);

	retval = usb_add_hcd(hcd, dev->resource[1].start,
				IRQF_DISABLED | IRQF_SHARED);
	if (retval == 0)
		return retval;

	dev_dbg(hcd->self.controller, "ERR: add_hcd\n");
	omap_stop_ehc(dev, hcd);
	iounmap(hcd->regs);
	usb_put_hcd(hcd);

	return retval;
}

/*-------------------------------------------------------------------------*/

/* may be called without controller electrically present */
/* may be called with controller, bus, and devices active */

/**
 * ehci_hcd_omap_drv_remove - shutdown processing for EHCI HCDs
 * @dev: USB Host Controller being removed
 * Context: !in_interrupt()
 *
 * Reverses the effect of usb_ehci_hcd_omap_probe(), first invoking
 * the HCD's stop() method.  It is always called from a thread
 * context, normally "rmmod", "apmd", or something similar.
 *
 */
static int ehci_hcd_omap_drv_remove(struct platform_device *dev)
{
	struct usb_hcd *hcd = platform_get_drvdata(dev);

	dev_dbg(&dev->dev, "ehci_hcd_omap_drv_remove()\n");

	iounmap(hcd->regs);
	usb_remove_hcd(hcd);
	usb_put_hcd(hcd);
	omap_stop_ehc(dev, hcd);

	return 0;
}

/*-------------------------------------------------------------------------*/
#ifdef CONFIG_PM
static int omap_ehci_bus_suspend(struct usb_hcd *hcd)
{
	return ehci_bus_suspend(hcd);
}

static int omap_ehci_bus_resume(struct usb_hcd *hcd)
{
	return ehci_bus_resume(hcd);
}
#endif
/*-------------------------------------------------------------------------*/

static const struct hc_driver ehci_omap_hc_driver = {
	.description = hcd_name,
	.product_desc = "OMAP-EHCI Host Controller",
	.hcd_priv_size = sizeof(struct ehci_hcd)
				+ sizeof(struct ehci_omap_clock_defs),

	/*
	 * generic hardware linkage
	 */
	.irq = ehci_irq,
	.flags = HCD_MEMORY | HCD_USB2,

	/*
	 * basic lifecycle operations
	 */
	.reset = ehci_init,
	.start = ehci_run,
	.stop = ehci_stop,
	.shutdown = ehci_shutdown,

	/*
	 * managing i/o requests and associated device resources
	 */
	.urb_enqueue = ehci_urb_enqueue,
	.urb_dequeue = ehci_urb_dequeue,
	.endpoint_disable = ehci_endpoint_disable,

	/*
	 * scheduling support
	 */
	.get_frame_number = ehci_get_frame,

	/*
	 * root hub support
	 */
	.hub_status_data = ehci_hub_status_data,
	.hub_control = ehci_hub_control,
#ifdef	CONFIG_PM
	.bus_suspend = omap_ehci_bus_suspend,
	.bus_resume = omap_ehci_bus_resume,
#endif
};

/*-------------------------------------------------------------------------*/
MODULE_ALIAS("platform:omap-ehci");
static struct platform_driver ehci_hcd_omap_driver = {
	.probe = ehci_hcd_omap_drv_probe,
	.remove = ehci_hcd_omap_drv_remove,
	.shutdown = usb_hcd_platform_shutdown,
	/*.suspend      = ehci_hcd_omap_drv_suspend, */
	/*.resume       = ehci_hcd_omap_drv_resume, */
	.driver = {
		.name = "ehci-omap",
		.bus = &platform_bus_type
	}
};
