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

#include <linux/config.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/delay.h>
#include <linux/clk.h>

#include <asm/io.h>
#include <asm/arch/hardware.h>
#include <asm/arch/memory.h>
// #include <asm/arch/gpio.h>
#include <asm/mach-types.h>

#include "musbdefs.h"


#ifdef CONFIG_ARCH_DAVINCI

#ifdef CONFIG_MACH_DAVINCI_EVM
#include <asm/arch/i2c-client.h>
#endif

#include "davinci.h"
#endif

#ifdef CONFIG_USB_TI_CPPI_DMA
#include "cppi_dma.h"
#endif


static inline void phy_on(void)
{
	/* start the on-chip PHY and its PLL */
	__raw_writel(USBPHY_SESNDEN | USBPHY_VBDTCTEN | USBPHY_PHYPLLON,
			IO_ADDRESS(USBPHY_CTL_PADDR));
	while ((__raw_readl(IO_ADDRESS(USBPHY_CTL_PADDR))
			& USBPHY_PHYCLKGD) == 0)
		cpu_relax();
}

static inline void phy_off(void)
{
	/* powerdown the on-chip PHY and its oscillator */
	__raw_writel(USBPHY_OSCPDWN | USBPHY_PHYSPDWN,
			IO_ADDRESS(USBPHY_CTL_PADDR));
}

static int dma_off = 1;

void musb_platform_enable(struct musb *musb)
{
	u32	tmp, old, val;

	/* workaround:  setup irqs through both register sets */
	tmp = (musb->wEndMask & DAVINCI_USB_TX_ENDPTS_MASK)
			<< DAVINCI_USB_TXINT_SHIFT;
	musb_writel(musb->ctrl_base, DAVINCI_USB_INT_MASK_SET_REG, tmp);
	old = tmp;
	tmp = (musb->wEndMask & (0xfffe & DAVINCI_USB_RX_ENDPTS_MASK))
			<< DAVINCI_USB_RXINT_SHIFT;
	musb_writel(musb->ctrl_base, DAVINCI_USB_INT_MASK_SET_REG, tmp);
	tmp |= old;

	val = ~MGC_M_INTR_SOF;
	tmp |= ((val & 0x01ff) << DAVINCI_USB_USBINT_SHIFT);
	musb_writel(musb->ctrl_base, DAVINCI_USB_INT_MASK_SET_REG, tmp);

	if (is_dma_capable() && !dma_off)
		printk(KERN_WARNING "%s %s: dma not reactivated\n",
				__FILE__, __FUNCTION__);
	else
		dma_off = 0;
}

/*
 * Disable the HDRC and flush interrupts
 */
void musb_platform_disable(struct musb *musb)
{
	/* because we don't set CTRLR.UINT, "important" to:
	 *  - not read/write INTRUSB/INTRUSBE
	 *  - (except during initial setup, as workaround)
	 *  - use INTSETR/INTCLRR instead
	 */
	musb_writel(musb->ctrl_base, DAVINCI_USB_INT_MASK_CLR_REG,
			  DAVINCI_USB_USBINT_MASK
			| DAVINCI_USB_TXINT_MASK
			| DAVINCI_USB_RXINT_MASK);
	musb_writeb(musb->pRegs, MGC_O_HDRC_DEVCTL, 0);
	musb_writel(musb->ctrl_base, DAVINCI_USB_EOI_REG, 0);

	if (is_dma_capable() && !dma_off)
		WARN("dma still active\n");
}


/* REVISIT this file shouldn't modify the OTG state machine ...
 *
 * The OTG infrastructure needs updating, to include things like
 * offchip DRVVBUS support and replacing MGC_OtgMachineInputs with
 * musb struct members (so e.g. vbus_state vanishes).
 */
static int vbus_state = -1;

#ifdef CONFIG_USB_MUSB_HDRC_HCD
#define	portstate(stmt)		stmt
#else
#define	portstate(stmt)
#endif

static void session(struct musb *musb, int is_on)
{
	void	*__iomem mregs = musb->pRegs;
	u8	devctl = musb_readb(mregs, MGC_O_HDRC_DEVCTL);

	/* NOTE: after drvvbus off the state _could_ be A_IDLE;
	 * but the silicon seems to couple vbus to "ID grounded".
	 */
	devctl |= MGC_M_DEVCTL_SESSION;
	if (is_on) {
		musb->xceiv.state = OTG_STATE_A_WAIT_BCON;
		portstate(musb->port1_status |= USB_PORT_STAT_POWER);
	} else {
		musb->xceiv.state = OTG_STATE_B_IDLE;
		portstate(musb->port1_status &= ~USB_PORT_STAT_POWER);
	}
	musb_writeb(mregs, MGC_O_HDRC_DEVCTL, devctl);
}


/* VBUS SWITCHING IS BOARD-SPECIFIC */

#ifdef CONFIG_MACH_DAVINCI_EVM

/* I2C operations are always synchronous, and require a task context.
 * With unloaded systems, using the shared workqueue seems to suffice
 * to satisfy the 100msec A_WAIT_VRISE timeout...
 */
static void evm_deferred_drvvbus(void *_musb)
{
	struct musb	*musb = _musb;
	int		is_on = (musb->xceiv.state == OTG_STATE_A_WAIT_VRISE);

	davinci_i2c_expander_op(0x3a, USB_DRVVBUS, !is_on);
	vbus_state = is_on;
	session(musb, is_on);
}
DECLARE_WORK(evm_vbus_work, evm_deferred_drvvbus, 0);

#endif

static void davinci_vbus_power(struct musb *musb, int is_on, int sleeping)
{
	if (is_on)
		is_on = 1;

	if (vbus_state == is_on)
		return;

	if (is_on) {
		musb->xceiv.state = OTG_STATE_A_WAIT_VRISE;
		MUSB_HST_MODE(musb);
	} else {
		switch (musb->xceiv.state) {
		case OTG_STATE_UNDEFINED:
		case OTG_STATE_B_IDLE:
			MUSB_DEV_MODE(musb);
			musb->xceiv.state = OTG_STATE_B_IDLE;
			break;
		case OTG_STATE_A_IDLE:
			break;
		default:
			musb->xceiv.state = OTG_STATE_A_WAIT_VFALL;
			break;
		}
	}

#ifdef CONFIG_MACH_DAVINCI_EVM
	if (machine_is_davinci_evm()) {
#ifdef CONFIG_MACH_DAVINCI_EVM_OTG
		/* modified EVM board switching VBUS with GPIO(6) not I2C
		 * NOTE:  PINMUX0.RGB888 (bit23) must be clear
		 */
		if (is_on)
			gpio_set(GPIO(6));
		else
			gpio_clear(GPIO(6));
#else
		if (sleeping)
			davinci_i2c_expander_op(0x3a, USB_DRVVBUS, !is_on);
		else
			schedule_work(&evm_vbus_work);
#endif
	}
#endif
	if (sleeping) {
		vbus_state = is_on;
		session(musb, is_on);
	}

	DBG(2, "VBUS power %s, %s\n", is_on ? "on" : "off",
		sleeping ? "immediate" : "deferred");
}

static void davinci_set_vbus(struct musb *musb, int is_on)
{
	return davinci_vbus_power(musb, is_on, 0);
}

static irqreturn_t davinci_interrupt(int irq, void *__hci, struct pt_regs *r)
{
	unsigned long	flags;
	irqreturn_t	retval = IRQ_NONE;
	struct musb	*musb = __hci;
	void		*__iomem tibase = musb->ctrl_base;
	u32		tmp;

	spin_lock_irqsave(&musb->Lock, flags);

	/* NOTE: DaVinci shadows the Mentor IRQs.  Don't manage them through
	 * the Mentor registers (except for setup), use the TI ones and EOI.
	 *
	 * Docs describe irq "vector" registers asociated with the CPPI and
	 * USB EOI registers.  These hold a bitmask corresponding to the
	 * current IRQ, not an irq handler address.  Would using those bits
	 * resolve some of the races observed in this dispatch code??
	 */

#ifdef CONFIG_USB_TI_CPPI_DMA
	/* CPPI interrupts share the same IRQ line, but have their own
	 * mask, state, "vector", and EOI registers.
	 */
	{
		u32 cppi_tx = musb_readl(tibase, DAVINCI_TXCPPI_MASKED_REG);
		u32 cppi_rx = musb_readl(tibase, DAVINCI_RXCPPI_MASKED_REG);

		if (cppi_tx || cppi_rx) {
			DBG(4, "<== CPPI IRQ t%x r%x\n", cppi_tx, cppi_rx);
			cppi_completion(musb, cppi_rx, cppi_tx);
			retval = IRQ_HANDLED;
		}
	}
#endif

	/* ack and handle non-CPPI interrupts */
	tmp = musb_readl(tibase, DAVINCI_USB_INT_SRC_MASKED_REG);
	musb_writel(tibase, DAVINCI_USB_INT_SRC_CLR_REG, tmp);

	musb->int_rx = (tmp & DAVINCI_USB_RXINT_MASK)
			>> DAVINCI_USB_RXINT_SHIFT;
	musb->int_tx = (tmp & DAVINCI_USB_TXINT_MASK)
			>> DAVINCI_USB_TXINT_SHIFT;
	musb->int_usb = (tmp & DAVINCI_USB_USBINT_MASK)
			>> DAVINCI_USB_USBINT_SHIFT;
	musb->int_regs = r;

	if (tmp & (1 << (8 + DAVINCI_USB_USBINT_SHIFT))) {
		int	drvvbus = musb_readl(tibase, DAVINCI_USB_STAT_REG);

		/* NOTE:  this must complete poweron within 100 msec */
		davinci_vbus_power(musb, drvvbus, 0);
		DBG(2, "DRVVBUS %d (state %d)\n", drvvbus, musb->xceiv.state);
		retval = IRQ_HANDLED;
	}

	if (musb->int_tx || musb->int_rx || musb->int_usb)
		retval |= musb_interrupt(musb);

	/* irq stays asserted until EOI is written */
	musb_writel(tibase, DAVINCI_USB_EOI_REG, 0);

	musb->int_regs = NULL;
	spin_unlock_irqrestore(&musb->Lock, flags);

	/* REVISIT we sometimes get unhandled IRQs
	 * (e.g. ep0).  not clear why...
	 */
	if (retval != IRQ_HANDLED)
		DBG(5, "unhandled? %08x\n", tmp);
	return IRQ_HANDLED;
}

int __devinit musb_platform_init(struct musb *musb)
{
	void	*__iomem tibase = musb->ctrl_base;
	u32	revision;

	musb->pRegs += DAVINCI_BASE_OFFSET;
#if 0
	/* REVISIT there's something odd about clocking, this
	 * didn't appear do the job ...
	 */
	musb->clock = clk_get(pDevice, "usb");
	if (IS_ERR(musb->clock))
		return PTR_ERR(musb->clock);

	status = clk_enable(musb->clock);
	if (status < 0)
		return -ENODEV;
#endif

	/* returns zero if e.g. not clocked */
	revision = musb_readl(tibase, DAVINCI_USB_VERSION_REG);
	if (revision == 0)
		return -ENODEV;

	/* note that transceiver issues make us want to charge
	 * VBUS only when the PHY PLL is not active.
	 */
#ifdef CONFIG_MACH_DAVINCI_EVM
	evm_vbus_work.data = musb;
#endif
	davinci_vbus_power(musb, musb->board_mode == MUSB_HOST, 1);

	if (is_host_enabled(musb))
		musb->board_set_vbus = davinci_set_vbus;

	/* reset the controller */
	musb_writel(tibase, DAVINCI_USB_CTRL_REG, 0x1);

	/* start the on-chip PHY and its PLL */
	phy_on();

	msleep(5);

	/* NOTE:  irqs are in mixed mode, not bypass to pure-musb */
	pr_debug("DaVinci OTG revision %08x phy %03x control %02x\n",
		revision,
		__raw_readl((void *__iomem) IO_ADDRESS(USBPHY_CTL_PADDR)),
		musb_readb(tibase, DAVINCI_USB_CTRL_REG));

	musb->isr = davinci_interrupt;
	return 0;
}

int musb_platform_exit(struct musb *musb)
{
	phy_off();
	davinci_vbus_power(musb, 0 /*off*/, 1);
	return 0;
}
