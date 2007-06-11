/*
 * TUSB6010 USB 2.0 OTG Dual Role controller
 *
 * Copyright (C) 2006 Nokia Corporation
 * Jarkko Nikula <jarkko.nikula@nokia.com>
 * Tony Lindgren <tony@atomide.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Notes:
 * - Driver assumes that interface to external host (main CPU) is
 *   configured for NOR FLASH interface instead of VLYNQ serial
 *   interface.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/usb.h>
#include <linux/irq.h>
#include <linux/platform_device.h>

#include "musbdefs.h"

/*
 * Checks the revision. We need to use the DMA register as 3.0 does not
 * have correct versions for TUSB_PRCM_REV or TUSB_INT_CTRL_REV.
 */
static u8 tusb_get_revision(struct musb *musb)
{
	void __iomem	*base = musb->ctrl_base;

	return musb_readl(base, TUSB_DMA_CTRL_REV) & 0xff;
}

#define TUSB_REV_MAJOR(reg_val)		((reg_val >> 4) & 0xf)
#define TUSB_REV_MINOR(reg_val)		(reg_val & 0xf)

static int __init tusb_print_revision(struct musb *musb)
{
	void __iomem	*base = musb->ctrl_base;

	pr_info("tusb: Revisions: %s%i.%i %s%i.%i %s%i.%i %s%i.%i\n",
		"prcm",
		TUSB_REV_MAJOR(musb_readl(base, TUSB_PRCM_REV)),
		TUSB_REV_MINOR(musb_readl(base, TUSB_PRCM_REV)),
		"int",
		TUSB_REV_MAJOR(musb_readl(base, TUSB_INT_CTRL_REV)),
		TUSB_REV_MINOR(musb_readl(base, TUSB_INT_CTRL_REV)),
		"gpio",
		TUSB_REV_MAJOR(musb_readl(base, TUSB_GPIO_REV)),
		TUSB_REV_MINOR(musb_readl(base, TUSB_GPIO_REV)),
		"dma",
		TUSB_REV_MAJOR(musb_readl(base, TUSB_DMA_CTRL_REV)),
		TUSB_REV_MINOR(musb_readl(base, TUSB_DMA_CTRL_REV)));

	return TUSB_REV_MAJOR(musb_readl(base, TUSB_INT_CTRL_REV));
}

#define WBUS_QUIRK_MASK	(TUSB_PHY_OTG_CTRL_TESTM2 | TUSB_PHY_OTG_CTRL_TESTM1	\
				| TUSB_PHY_OTG_CTRL_TESTM0)

/*
 * Workaround for spontaneous WBUS wake-up issue #2 for tusb3.0.
 * Disables power detection in PHY for the duration of idle.
 */
static void tusb_wbus_quirk(struct musb *musb, int enabled)
{
	void __iomem	*base = musb->ctrl_base;
	static u32	phy_otg_ena = 0, phy_otg_ctrl = 0;
	u32		int_src, tmp;

	if (enabled) {
		phy_otg_ena = musb_readl(base, TUSB_PHY_OTG_CTRL_ENABLE);
		phy_otg_ctrl = musb_readl(base, TUSB_PHY_OTG_CTRL);
		tmp = TUSB_PHY_OTG_CTRL_WRPROTECT
				| phy_otg_ena | WBUS_QUIRK_MASK;
		musb_writel(base, TUSB_PHY_OTG_CTRL_ENABLE, tmp);
		tmp = phy_otg_ctrl & ~WBUS_QUIRK_MASK;
		tmp |= TUSB_PHY_OTG_CTRL_WRPROTECT | TUSB_PHY_OTG_CTRL_TESTM2;
		musb_writel(base, TUSB_PHY_OTG_CTRL, tmp);
		DBG(2, "Enabled tusb wbus quirk ena %08x ctrl %08x\n",
			musb_readl(base, TUSB_PHY_OTG_CTRL_ENABLE),
			musb_readl(base, TUSB_PHY_OTG_CTRL));
	} else if (musb_readl(base, TUSB_PHY_OTG_CTRL_ENABLE)
					& TUSB_PHY_OTG_CTRL_TESTM2) {
		tmp = TUSB_PHY_OTG_CTRL_WRPROTECT
				| phy_otg_ena | WBUS_QUIRK_MASK;
		musb_writel(base, TUSB_PHY_OTG_CTRL_ENABLE, tmp);
		tmp = TUSB_PHY_OTG_CTRL_WRPROTECT | phy_otg_ctrl;
		musb_writel(base, TUSB_PHY_OTG_CTRL, tmp);
		tmp = TUSB_PHY_OTG_CTRL_WRPROTECT | phy_otg_ena;
		musb_writel(base, TUSB_PHY_OTG_CTRL_ENABLE, tmp);
		DBG(2, "Disabled tusb wbus quirk ena %08x ctrl %08x\n",
			musb_readl(base, TUSB_PHY_OTG_CTRL_ENABLE),
			musb_readl(base, TUSB_PHY_OTG_CTRL));
		phy_otg_ena = 0;
		phy_otg_ctrl = 0;
	}

	int_src = musb_readl(base, TUSB_INT_SRC);
	if (int_src & TUSB_INT_SRC_ID_STATUS_CHNG)
		musb_writel(base, TUSB_INT_SRC_CLEAR,
				TUSB_INT_SRC_ID_STATUS_CHNG);
}

/*
 * TUSB 6010 may use a parallel bus that doesn't support byte ops;
 * so both loading and unloading FIFOs need explicit byte counts.
 */

static inline void
tusb_fifo_write_unaligned(void __iomem *fifo, const u8 *buf, u16 len)
{
	u32		val;
	int		i;

	if (len > 4) {
		for (i = 0; i < (len >> 2); i++) {
			memcpy(&val, buf, 4);
			musb_writel(fifo, 0, val);
			buf += 4;
		}
		len %= 4;
	}
	if (len > 0) {
		/* Write the rest 1 - 3 bytes to FIFO */
		memcpy(&val, buf, len);
		musb_writel(fifo, 0, val);
	}
}

static inline void tusb_fifo_read_unaligned(void __iomem *fifo,
						void __iomem *buf, u16 len)
{
	u32		val;
	int		i;

	if (len > 4) {
		for (i = 0; i < (len >> 2); i++) {
			val = musb_readl(fifo, 0);
			memcpy(buf, &val, 4);
			buf += 4;
		}
		len %= 4;
	}
	if (len > 0) {
		/* Read the rest 1 - 3 bytes from FIFO */
		val = musb_readl(fifo, 0);
		memcpy(buf, &val, len);
	}
}

void musb_write_fifo(struct musb_hw_ep *hw_ep, u16 len, const u8 *buf)
{
	void __iomem	*ep_conf = hw_ep->conf;
	void __iomem	*fifo = hw_ep->fifo;
	u8		epnum = hw_ep->bLocalEnd;

	prefetch(buf);

	DBG(4, "%cX ep%d fifo %p count %d buf %p\n",
			'T', epnum, fifo, len, buf);

	if (epnum)
		musb_writel(ep_conf, TUSB_EP_TX_OFFSET,
			TUSB_EP_CONFIG_XFR_SIZE(len));
	else
		musb_writel(ep_conf, 0, TUSB_EP0_CONFIG_DIR_TX |
			TUSB_EP0_CONFIG_XFR_SIZE(len));

	if (likely((0x01 & (unsigned long) buf) == 0)) {

		/* Best case is 32bit-aligned destination address */
		if ((0x02 & (unsigned long) buf) == 0) {
			if (len >= 4) {
				writesl(fifo, buf, len >> 2);
				buf += (len & ~0x03);
				len &= 0x03;
			}
		} else {
			if (len >= 2) {
				u32 val;
				int i;

				/* Cannot use writesw, fifo is 32-bit */
				for (i = 0; i < (len >> 2); i++) {
					val = (u32)(*(u16 *)buf);
					buf += 2;
					val |= (*(u16 *)buf) << 16;
					buf += 2;
					musb_writel(fifo, 0, val);
				}
				len &= 0x03;
			}
		}
	}

	if (len > 0)
		tusb_fifo_write_unaligned(fifo, buf, len);
}

void musb_read_fifo(struct musb_hw_ep *hw_ep, u16 len, u8 *buf)
{
	void __iomem	*ep_conf = hw_ep->conf;
	void __iomem	*fifo = hw_ep->fifo;
	u8		epnum = hw_ep->bLocalEnd;

	DBG(4, "%cX ep%d fifo %p count %d buf %p\n",
			'R', epnum, fifo, len, buf);

	if (epnum)
		musb_writel(ep_conf, TUSB_EP_RX_OFFSET,
			TUSB_EP_CONFIG_XFR_SIZE(len));
	else
		musb_writel(ep_conf, 0, TUSB_EP0_CONFIG_XFR_SIZE(len));

	if (likely((0x01 & (unsigned long) buf) == 0)) {

		/* Best case is 32bit-aligned destination address */
		if ((0x02 & (unsigned long) buf) == 0) {
			if (len >= 4) {
				readsl(fifo, buf, len >> 2);
				buf += (len & ~0x03);
				len &= 0x03;
			}
		} else {
			if (len >= 2) {
				u32 val;
				int i;

				/* Cannot use readsw, fifo is 32-bit */
				for (i = 0; i < (len >> 2); i++) {
					val = musb_readl(fifo, 0);
					*(u16 *)buf = (u16)(val & 0xffff);
					buf += 2;
					*(u16 *)buf = (u16)(val >> 16);
					buf += 2;
				}
				len &= 0x03;
			}
		}
	}

	if (len > 0)
		tusb_fifo_read_unaligned(fifo, buf, len);
}

#ifdef CONFIG_USB_GADGET_MUSB_HDRC

/* This is used by gadget drivers, and OTG transceiver logic, allowing
 * at most mA current to be drawn from VBUS during a Default-B session
 * (that is, while VBUS exceeds 4.4V).  In Default-A (including pure host
 * mode), or low power Default-B sessions, something else supplies power.
 * Caller must take care of locking.
 */
static int tusb_draw_power(struct otg_transceiver *x, unsigned mA)
{
	struct musb	*musb = container_of(x, struct musb, xceiv);
	void __iomem	*base = musb->ctrl_base;
	u32		reg;

	/* tps65030 seems to consume max 100mA, with maybe 60mA available
	 * (measured on one board) for things other than tps and tusb.
	 *
	 * Boards sharing the CPU clock with CLKIN will need to prevent
	 * certain idle sleep states while the USB link is active.
	 *
	 * REVISIT we could use VBUS to supply only _one_ of { 1.5V, 3.3V }.
	 * The actual current usage would be very board-specific.  For now,
	 * it's simpler to just use an aggregate (also board-specific).
	 */
	if (x->default_a || mA < (musb->min_power << 1))
		mA = 0;

	reg = musb_readl(base, TUSB_PRCM_MNGMT);
	if (mA) {
		if (musb->set_clock)
			musb->set_clock(musb->clock, 1);
		musb->is_bus_powered = 1;
		reg |= TUSB_PRCM_MNGMT_15_SW_EN | TUSB_PRCM_MNGMT_33_SW_EN;
	} else {
		musb->is_bus_powered = 0;
		reg &= ~(TUSB_PRCM_MNGMT_15_SW_EN | TUSB_PRCM_MNGMT_33_SW_EN);
		if (musb->set_clock)
			musb->set_clock(musb->clock, 0);
	}
	musb_writel(base, TUSB_PRCM_MNGMT, reg);

	DBG(2, "draw max %d mA VBUS\n", mA);
	return 0;
}

#else
#define tusb_draw_power	NULL
#endif

/* workaround for issue 13:  change clock during chip idle
 * (to be fixed in rev3 silicon) ... symptoms include disconnect
 * or looping suspend/resume cycles
 */
static void tusb_set_clock_source(struct musb *musb, unsigned mode)
{
	void __iomem	*base = musb->ctrl_base;
	u32		reg;

	reg = musb_readl(base, TUSB_PRCM_CONF);
	reg &= ~TUSB_PRCM_CONF_SYS_CLKSEL(0x3);

	/* 0 = refclk (clkin, XI)
	 * 1 = PHY 60 MHz (internal PLL)
	 * 2 = not supported
	 * 3 = what?
	 */
	if (mode > 0)
		reg |= TUSB_PRCM_CONF_SYS_CLKSEL(mode & 0x3);

	musb_writel(base, TUSB_PRCM_CONF, reg);

	// FIXME tusb6010_platform_retime(mode == 0);
}

/*
 * Idle TUSB6010 until next wake-up event; NOR access always wakes.
 * Other code ensures that we idle unless we're connected _and_ the
 * USB link is not suspended ... and tells us the relevant wakeup
 * events.  SW_EN for voltage is handled separately.
 */
static void tusb_allow_idle(struct musb *musb, u32 wakeup_enables)
{
	void __iomem	*base = musb->ctrl_base;
	u32		reg;

	if ((wakeup_enables & TUSB_PRCM_WBUS)
			&& (tusb_get_revision(musb) == TUSB_REV_30))
		tusb_wbus_quirk(musb, 1);

	tusb_set_clock_source(musb, 0);

	wakeup_enables |= TUSB_PRCM_WNORCS;
	musb_writel(base, TUSB_PRCM_WAKEUP_MASK, ~wakeup_enables);

	/* REVISIT writeup of WID implies that if WID set and ID is grounded,
	 * TUSB_PHY_OTG_CTRL.TUSB_PHY_OTG_CTRL_OTG_ID_PULLUP must be cleared.
	 * Presumably that's mostly to save power, hence WID is immaterial ...
	 */

	reg = musb_readl(base, TUSB_PRCM_MNGMT);
	/* issue 4: when driving vbus, use hipower (vbus_det) comparator */
	if (is_host_active(musb)) {
		reg |= TUSB_PRCM_MNGMT_OTG_VBUS_DET_EN;
		reg &= ~TUSB_PRCM_MNGMT_OTG_SESS_END_EN;
	} else {
		reg |= TUSB_PRCM_MNGMT_OTG_SESS_END_EN;
		reg &= ~TUSB_PRCM_MNGMT_OTG_VBUS_DET_EN;
	}
	reg |= TUSB_PRCM_MNGMT_PM_IDLE | TUSB_PRCM_MNGMT_DEV_IDLE;
	musb_writel(base, TUSB_PRCM_MNGMT, reg);

	DBG(6, "idle, wake on %02x\n", wakeup_enables);
}

/*
 * Updates cable VBUS status. Caller must take care of locking.
 */
int musb_platform_get_vbus_status(struct musb *musb)
{
	void __iomem	*base = musb->ctrl_base;
	u32		otg_stat, prcm_mngmt;
	int		ret = 0;

	otg_stat = musb_readl(base, TUSB_DEV_OTG_STAT);
	prcm_mngmt = musb_readl(base, TUSB_PRCM_MNGMT);

	/* Temporarily enable VBUS detection if it was disabled for
	 * suspend mode. Unless it's enabled otg_stat and devctl will
	 * not show correct VBUS state.
	 */
	if (!(prcm_mngmt & TUSB_PRCM_MNGMT_OTG_VBUS_DET_EN)) {
		u32 tmp = prcm_mngmt;
		tmp |= TUSB_PRCM_MNGMT_OTG_VBUS_DET_EN;
		musb_writel(base, TUSB_PRCM_MNGMT, tmp);
		otg_stat = musb_readl(base, TUSB_DEV_OTG_STAT);
		musb_writel(base, TUSB_PRCM_MNGMT, prcm_mngmt);
	}

	if (otg_stat & TUSB_DEV_OTG_STAT_VBUS_SENSE)
		ret = 1;

	return ret;
}

static struct timer_list musb_idle_timer;

static void musb_do_idle(unsigned long _musb)
{
	struct musb	*musb = (void *)_musb;
	unsigned long	flags;

	spin_lock_irqsave(&musb->Lock, flags);
	if (!musb->is_active) {
		u32	wakeups;

#ifdef CONFIG_USB_MUSB_HDRC_HCD
		/* wait until khubd handles port change status */
		if (is_host_active(musb) && (musb->port1_status >> 16))
			goto done;
#endif

#ifdef CONFIG_USB_GADGET_MUSB_HDRC
		if (is_peripheral_enabled(musb) && !musb->pGadgetDriver)
			wakeups = 0;
		else {
			wakeups = TUSB_PRCM_WHOSTDISCON
					| TUSB_PRCM_WBUS
					| TUSB_PRCM_WVBUS;
			if (is_otg_enabled(musb))
				wakeups |= TUSB_PRCM_WID;
		}
#else
		wakeups = TUSB_PRCM_WHOSTDISCON | TUSB_PRCM_WBUS;
#endif
		tusb_allow_idle(musb, wakeups);
	}
done:
	spin_unlock_irqrestore(&musb->Lock, flags);
}

/*
 * Maybe put TUSB6010 into idle mode mode depending on USB link status,
 * like "disconnected" or "suspended".  We'll be woken out of it by
 * connect, resume, or disconnect.
 *
 * Needs to be called as the last function everywhere where there is
 * register access to TUSB6010 because of NOR flash wake-up.
 * Caller should own controller spinlock.
 *
 * Delay because peripheral enables D+ pullup 3msec after SE0, and
 * we don't want to treat that full speed J as a wakeup event.
 * ... peripherals must draw only suspend current after 10 msec.
 */
void musb_platform_try_idle(struct musb *musb)
{
	if (musb->is_active)
		del_timer(&musb_idle_timer);
	else
		mod_timer(&musb_idle_timer, jiffies + msecs_to_jiffies(3));
}

/* ticks of 60 MHz clock */
#define DEVCLOCK		60000000
#define OTG_TIMER_MS(msecs)	((msecs) \
		? (TUSB_DEV_OTG_TIMER_VAL((DEVCLOCK/1000)*(msecs)) \
				| TUSB_DEV_OTG_TIMER_ENABLE) \
		: 0)

static void tusb_source_power(struct musb *musb, int is_on)
{
	void __iomem	*base = musb->ctrl_base;
	u32		conf, prcm, timer;
	u8		devctl;

	/* HDRC controls CPEN, but beware current surges during device
	 * connect.  They can trigger transient overcurrent conditions
	 * that must be ignored.
	 */

	prcm = musb_readl(base, TUSB_PRCM_MNGMT);
	conf = musb_readl(base, TUSB_DEV_CONF);
	devctl = musb_readb(musb->pRegs, MGC_O_HDRC_DEVCTL);

	if (is_on) {
		musb->is_active = 1;
		timer = OTG_TIMER_MS(OTG_TIME_A_WAIT_VRISE);

		musb->xceiv.default_a = 1;
		musb->xceiv.state = OTG_STATE_A_WAIT_VRISE;
		devctl |= MGC_M_DEVCTL_SESSION;

		conf |= TUSB_DEV_CONF_USB_HOST_MODE;
		MUSB_HST_MODE(musb);
	} else {
		musb->is_active = 0;
		timer = 0;

		/* NOTE:  we're skipping A_WAIT_VFALL -> A_IDLE and
		 * jumping right to B_IDLE...
		 */

		musb->xceiv.default_a = 0;
		musb->xceiv.state = OTG_STATE_B_IDLE;
		devctl &= ~MGC_M_DEVCTL_SESSION;

		conf &= ~TUSB_DEV_CONF_USB_HOST_MODE;
		MUSB_DEV_MODE(musb);
	}
	prcm &= ~(TUSB_PRCM_MNGMT_15_SW_EN | TUSB_PRCM_MNGMT_33_SW_EN);

	musb_writel(base, TUSB_PRCM_MNGMT, prcm);
	musb_writel(base, TUSB_DEV_OTG_TIMER, timer);
	musb_writel(base, TUSB_DEV_CONF, conf);
	musb_writeb(musb->pRegs, MGC_O_HDRC_DEVCTL, devctl);

	DBG(1, "VBUS %s, devctl %02x otg %3x conf %08x prcm %08x\n",
		otg_state_string(musb),
		musb_readb(musb->pRegs, MGC_O_HDRC_DEVCTL),
		musb_readl(base, TUSB_DEV_OTG_STAT),
		conf, prcm);
}

/*
 * Sets the mode to OTG, peripheral or host by changing the ID detection.
 * Caller must take care of locking.
 *
 * Note that if a mini-A cable is plugged in the ID line will stay down as
 * the weak ID pull-up is not able to pull the ID up.
 *
 * REVISIT: It would be possible to add support for changing between host
 * and peripheral modes in non-OTG configurations by reconfiguring hardware
 * and then setting musb->board_mode. For now, only support OTG mode.
 */
void musb_platform_set_mode(struct musb *musb, u8 musb_mode)
{
	void __iomem	*base = musb->ctrl_base;
	u32		otg_stat, phy_otg_ena, phy_otg_ctrl, dev_conf;
	int		vbus = 0;

	if (musb->board_mode != MUSB_OTG) {
		ERR("Changing mode currently only supported in OTG mode\n");
		return;
	}

	otg_stat = musb_readl(base, TUSB_DEV_OTG_STAT);
	phy_otg_ena = musb_readl(base, TUSB_PHY_OTG_CTRL_ENABLE);
	phy_otg_ctrl = musb_readl(base, TUSB_PHY_OTG_CTRL);
	dev_conf = musb_readl(base, TUSB_DEV_CONF);

	switch (musb_mode) {

#ifdef CONFIG_USB_MUSB_HDRC_HCD
	case MUSB_HOST:		/* Disable PHY ID detect, ground ID */
		if (!(otg_stat & TUSB_DEV_OTG_STAT_ID_STATUS)) {
			ERR("Already in host mode otg_stat: %08x\n", otg_stat);
			return;
		}
		phy_otg_ena |= TUSB_PHY_OTG_CTRL_OTG_ID_PULLUP;
		phy_otg_ctrl &= ~TUSB_PHY_OTG_CTRL_OTG_ID_PULLUP;
		dev_conf |= TUSB_DEV_CONF_ID_SEL;
		dev_conf &= ~TUSB_DEV_CONF_SOFT_ID;
		vbus = 1;
		break;
#endif

#ifdef CONFIG_USB_GADGET_MUSB_HDRC
	case MUSB_PERIPHERAL:	/* Disable PHY ID detect, keep ID pull-up on */
		if (otg_stat & TUSB_DEV_OTG_STAT_ID_STATUS) {
			ERR("Already in peripheral mode otg_stat: %08x\n",
				otg_stat);
			return;
		}
		phy_otg_ena |= TUSB_PHY_OTG_CTRL_OTG_ID_PULLUP;
		phy_otg_ctrl |= TUSB_PHY_OTG_CTRL_OTG_ID_PULLUP;
		dev_conf |= (TUSB_DEV_CONF_ID_SEL | TUSB_DEV_CONF_SOFT_ID);
		break;
#endif

#ifdef CONFIG_USB_MUSB_OTG
	case MUSB_OTG:		/* Use PHY ID detection */
		phy_otg_ena &= ~TUSB_PHY_OTG_CTRL_OTG_ID_PULLUP;
		phy_otg_ctrl &= ~TUSB_PHY_OTG_CTRL_OTG_ID_PULLUP;
		dev_conf &= ~(TUSB_DEV_CONF_ID_SEL | TUSB_DEV_CONF_SOFT_ID);
		break;
#endif

	default:
		DBG(2, "Trying to set unknown mode %i\n", musb_mode);
	}

	musb_writel(base, TUSB_PHY_OTG_CTRL_ENABLE,
			TUSB_PHY_OTG_CTRL_WRPROTECT | phy_otg_ena);
	musb_writel(base, TUSB_PHY_OTG_CTRL,
			TUSB_PHY_OTG_CTRL_WRPROTECT | phy_otg_ctrl);
	musb_writel(base, TUSB_DEV_CONF, dev_conf);

	msleep(1);
	otg_stat = musb_readl(base, TUSB_DEV_OTG_STAT);
	if ((musb_mode == MUSB_PERIPHERAL) &&
		!(otg_stat & TUSB_DEV_OTG_STAT_ID_STATUS))
			ERR("Cannot be peripheral with mini-A cable "
			"otg_stat: %08x\n", otg_stat);
}

static inline void
tusb_otg_ints(struct musb *musb, u32 int_src, void __iomem *base)
{
	u32	otg_stat = musb_readl(base, TUSB_DEV_OTG_STAT);

	/* ID pin */
	if ((int_src & TUSB_INT_SRC_ID_STATUS_CHNG)) {
		int	default_a;

		if (is_otg_enabled(musb))
			default_a = !(otg_stat & TUSB_DEV_OTG_STAT_ID_STATUS);
		else
			default_a = is_host_enabled(musb);
		DBG(2, "Default-%c\n", default_a ? 'A' : 'B');
		musb->xceiv.default_a = default_a;
		tusb_source_power(musb, default_a);
	}

	/* VBUS state change */
	if (int_src & TUSB_INT_SRC_VBUS_SENSE_CHNG) {

		/* B-dev state machine:  no vbus ~= disconnect */
		if ((is_otg_enabled(musb) && !musb->xceiv.default_a)
				|| !is_host_enabled(musb)) {
#ifdef CONFIG_USB_MUSB_HDRC_HCD
			// ? musb_root_disconnect(musb);
			musb->port1_status &=
				~(USB_PORT_STAT_CONNECTION
				| USB_PORT_STAT_ENABLE
				| USB_PORT_STAT_LOW_SPEED
				| USB_PORT_STAT_HIGH_SPEED
				| USB_PORT_STAT_TEST
				);
#endif

			if (otg_stat & TUSB_DEV_OTG_STAT_SESS_END) {
				if (musb->xceiv.state != OTG_STATE_B_IDLE) {
					/* INTR_DISCONNECT can hide... */
					musb->xceiv.state = OTG_STATE_B_IDLE;
					musb->int_usb |= MGC_M_INTR_DISCONNECT;
				}
				musb->is_active = 0;
			}
			DBG(2, "vbus change, %s, otg %03x\n",
				otg_state_string(musb), otg_stat);
			schedule_work(&musb->irq_work);

		} else /* A-dev state machine */ {
			DBG(4, "vbus change, %s, otg %03x\n",
				otg_state_string(musb), otg_stat);

			switch (musb->xceiv.state) {
			case OTG_STATE_A_WAIT_VRISE:
				/* ignore; A-session-valid < VBUS_VALID/2,
				 * we monitor this with the timer
				 */
				break;
			case OTG_STATE_A_WAIT_VFALL:
				/* REVISIT this irq triggers during short
				 * spikes causet by enumeration ...
				 */
				if (musb->vbuserr_retry) {
					musb->vbuserr_retry--;
					tusb_source_power(musb, 1);
				}
				break;
			default:
				break;
			}
		}
	}

	/* OTG timer expiration */
	if (int_src & TUSB_INT_SRC_OTG_TIMEOUT) {
		u8	devctl;

		DBG(4, "%s timer, %03x\n", otg_state_string(musb), otg_stat);

		switch (musb->xceiv.state) {
		case OTG_STATE_A_WAIT_VRISE:
			/* VBUS has probably been valid for a while now,
			 * but may well have bounced out of range a bit
			 */
			devctl = musb_readb(musb->pRegs, MGC_O_HDRC_DEVCTL);
			if (otg_stat & TUSB_DEV_OTG_STAT_VBUS_VALID) {
				u32	timer;

				if ((devctl & MGC_M_DEVCTL_VBUS)
						!= MGC_M_DEVCTL_VBUS) {
					DBG(2, "devctl %02x\n", devctl);
					break;
				}
				musb->xceiv.state = OTG_STATE_A_WAIT_BCON;

				/* REVISIT: if nothing is connected yet,
				 * mark controller as inactive so that
				 * we can suspend the TUSB chip.
				 */

				/* timeout 0 == infinite (like non-OTG hosts) */
				timer = OTG_TIMER_MS(OTG_TIME_A_WAIT_BCON);
				if (timer)
					musb_writel(base, TUSB_DEV_OTG_TIMER,
							timer);
			} else {
				/* REVISIT report overcurrent to hub? */
				ERR("vbus too slow, devctl %02x\n", devctl);
				tusb_source_power(musb, 0);
			}
			break;
		case OTG_STATE_A_WAIT_BCON:
			if (OTG_TIME_A_WAIT_BCON)
				tusb_source_power(musb, 0);
			break;
		case OTG_STATE_A_SUSPEND:
			break;
		case OTG_STATE_B_WAIT_ACON:
			break;
		default:
			break;
		}
	}
}

static irqreturn_t tusb_interrupt(int irq, void *__hci)
{
	struct musb	*musb = __hci;
	void __iomem	*base = musb->ctrl_base;
	unsigned long	flags;
	u32		int_mask, int_src;

	spin_lock_irqsave(&musb->Lock, flags);

	/* Mask all interrupts to allow using both edge and level GPIO irq */
	int_mask = musb_readl(base, TUSB_INT_MASK);
	musb_writel(base, TUSB_INT_MASK, ~TUSB_INT_MASK_RESERVED_BITS);

	int_src = musb_readl(base, TUSB_INT_SRC) & ~TUSB_INT_SRC_RESERVED_BITS;
	DBG(3, "TUSB IRQ %08x\n", int_src);

	musb->int_usb = (u8) int_src;

	/* Acknowledge wake-up source interrupts */
	if (int_src & TUSB_INT_SRC_DEV_WAKEUP) {
		u32	reg;
		u32	i;

		if (tusb_get_revision(musb) == TUSB_REV_30)
			tusb_wbus_quirk(musb, 0);

		/* there are issues re-locking the PLL on wakeup ... */

		/* work around issue 8 */
		for (i = 0xf7f7f7; i > 0xf7f7f7 - 1000; i--) {
			musb_writel(base, TUSB_SCRATCH_PAD, 0);
			musb_writel(base, TUSB_SCRATCH_PAD, i);
			reg = musb_readl(base, TUSB_SCRATCH_PAD);
			if (reg == i)
				break;
			DBG(6, "TUSB NOR not ready\n");
		}

		/* work around issue 13 (2nd half) */
		tusb_set_clock_source(musb, 1);

		reg = musb_readl(base, TUSB_PRCM_WAKEUP_SOURCE);
		musb_writel(base, TUSB_PRCM_WAKEUP_CLEAR, reg);
		if (reg & ~TUSB_PRCM_WNORCS) {
			musb->is_active = 1;
			schedule_work(&musb->irq_work);
		}
		DBG(3, "wake %sactive %02x\n",
				musb->is_active ? "" : "in", reg);

		// REVISIT host side TUSB_PRCM_WHOSTDISCON, TUSB_PRCM_WBUS
	}

	/* OTG state change reports (annoyingly) not issued by Mentor core */
	if (int_src & (TUSB_INT_SRC_VBUS_SENSE_CHNG
				| TUSB_INT_SRC_OTG_TIMEOUT
				| TUSB_INT_SRC_ID_STATUS_CHNG))
		tusb_otg_ints(musb, int_src, base);

	/* TX dma callback must be handled here, RX dma callback is
	 * handled in tusb_omap_dma_cb.
	 */
	if ((int_src & TUSB_INT_SRC_TXRX_DMA_DONE)) {
		u32	dma_src = musb_readl(base, TUSB_DMA_INT_SRC);
		u32	real_dma_src = musb_readl(base, TUSB_DMA_INT_MASK);

		DBG(3, "DMA IRQ %08x\n", dma_src);
		real_dma_src = ~real_dma_src & dma_src;
		if (tusb_dma_omap() && real_dma_src) {
			int	tx_source = (real_dma_src & 0xffff);
			int	i;

			for (i = 1; i <= 15; i++) {
				if (tx_source & (1 << i)) {
					DBG(3, "completing ep%i %s\n", i, "tx");
					musb_dma_completion(musb, i, 1);
				}
			}
		}
		musb_writel(base, TUSB_DMA_INT_CLEAR, dma_src);
	}

	/* EP interrupts. In OCP mode tusb6010 mirrors the MUSB * interrupts */
	if (int_src & (TUSB_INT_SRC_USB_IP_TX | TUSB_INT_SRC_USB_IP_RX)) {
		u32	musb_src = musb_readl(base, TUSB_USBIP_INT_SRC);

		musb_writel(base, TUSB_USBIP_INT_CLEAR, musb_src);
		musb->int_rx = (((musb_src >> 16) & 0xffff) << 1);
		musb->int_tx = (musb_src & 0xffff);
	} else
		musb->int_rx = musb->int_tx = 0;

	if (int_src & (TUSB_INT_SRC_USB_IP_TX | TUSB_INT_SRC_USB_IP_RX | 0xff))
		musb_interrupt(musb);

	/* Acknowledge TUSB interrupts. Clear only non-reserved bits */
	musb_writel(base, TUSB_INT_SRC_CLEAR,
		int_src & ~TUSB_INT_MASK_RESERVED_BITS);

	musb_platform_try_idle(musb);

	musb_writel(base, TUSB_INT_MASK, int_mask);
	spin_unlock_irqrestore(&musb->Lock, flags);

	return IRQ_HANDLED;
}

static int dma_off;

/*
 * Enables TUSB6010. Caller must take care of locking.
 * REVISIT:
 * - Check what is unnecessary in MGC_HdrcStart()
 */
void musb_platform_enable(struct musb * musb)
{
	void __iomem	*base = musb->ctrl_base;

	/* Setup TUSB6010 main interrupt mask. Enable all interrupts except SOF.
	 * REVISIT: Enable and deal with TUSB_INT_SRC_USB_IP_SOF */
	musb_writel(base, TUSB_INT_MASK, TUSB_INT_SRC_USB_IP_SOF);

	/* Setup TUSB interrupt, disable DMA and GPIO interrupts */
	musb_writel(base, TUSB_USBIP_INT_MASK, 0);
	musb_writel(base, TUSB_DMA_INT_MASK, 0x7fffffff);
	musb_writel(base, TUSB_GPIO_INT_MASK, 0x1ff);

	/* Clear all subsystem interrups */
	musb_writel(base, TUSB_USBIP_INT_CLEAR, 0x7fffffff);
	musb_writel(base, TUSB_DMA_INT_CLEAR, 0x7fffffff);
	musb_writel(base, TUSB_GPIO_INT_CLEAR, 0x1ff);

	/* Acknowledge pending interrupt(s) */
	musb_writel(base, TUSB_INT_SRC_CLEAR, ~TUSB_INT_MASK_RESERVED_BITS);

	/* Only 0 clock cycles for minimum interrupt de-assertion time and
	 * interrupt polarity active low seems to work reliably here */
	musb_writel(base, TUSB_INT_CTRL_CONF,
			TUSB_INT_CTRL_CONF_INT_RELCYC(0));

	set_irq_type(musb->nIrq, IRQ_TYPE_LEVEL_LOW);

	/* maybe force into the Default-A OTG state machine */
	if (!(musb_readl(base, TUSB_DEV_OTG_STAT)
			& TUSB_DEV_OTG_STAT_ID_STATUS))
		musb_writel(base, TUSB_INT_SRC_SET,
				TUSB_INT_SRC_ID_STATUS_CHNG);

	if (is_dma_capable() && dma_off)
		printk(KERN_WARNING "%s %s: dma not reactivated\n",
				__FILE__, __FUNCTION__);
	else
		dma_off = 1;
}

/*
 * Disables TUSB6010. Caller must take care of locking.
 */
void musb_platform_disable(struct musb *musb)
{
	void __iomem	*base = musb->ctrl_base;

	/* FIXME stop DMA, IRQs, timers, ... */

	/* disable all IRQs */
	musb_writel(base, TUSB_INT_MASK, ~TUSB_INT_MASK_RESERVED_BITS);
	musb_writel(base, TUSB_USBIP_INT_MASK, 0);
	musb_writel(base, TUSB_DMA_INT_MASK, 0x7fffffff);
	musb_writel(base, TUSB_GPIO_INT_MASK, 0x1ff);

	del_timer(&musb_idle_timer);

	if (is_dma_capable() && !dma_off) {
		printk(KERN_WARNING "%s %s: dma still active\n",
				__FILE__, __FUNCTION__);
		dma_off = 1;
	}
}

/*
 * Sets up TUSB6010 CPU interface specific signals and registers
 * Note: Settings optimized for OMAP24xx
 */
static void __init tusb_setup_cpu_interface(struct musb *musb)
{
	void __iomem	*base = musb->ctrl_base;

	/*
	 * Disable GPIO[5:0] pullups (used as output DMA requests)
	 * Don't disable GPIO[7:6] as they are needed for wake-up.
	 */
	musb_writel(base, TUSB_PULLUP_1_CTRL, 0x0000003F);

	/* Disable all pullups on NOR IF, DMAREQ0 and DMAREQ1 */
	musb_writel(base, TUSB_PULLUP_2_CTRL, 0x01FFFFFF);

	/* Turn GPIO[5:0] to DMAREQ[5:0] signals */
	musb_writel(base, TUSB_GPIO_CONF, TUSB_GPIO_CONF_DMAREQ(0x3f));

	/* Burst size 16x16 bits, all six DMA requests enabled, DMA request
	 * de-assertion time 2 system clocks p 62 */
	musb_writel(base, TUSB_DMA_REQ_CONF,
		TUSB_DMA_REQ_CONF_BURST_SIZE(2) |
		TUSB_DMA_REQ_CONF_DMA_REQ_EN(0x3f) |
		TUSB_DMA_REQ_CONF_DMA_REQ_ASSER(2));

	/* Set 0 wait count for synchronous burst access */
	musb_writel(base, TUSB_WAIT_COUNT, 1);
}

static int __init tusb_start(struct musb *musb)
{
	void __iomem	*base = musb->ctrl_base;
	int		ret = 0;
	unsigned long	flags;
	u32		reg;

	if (musb->board_set_power)
		ret = musb->board_set_power(1);
	if (ret != 0) {
		printk(KERN_ERR "tusb: Cannot enable TUSB6010\n");
		return ret;
	}

	spin_lock_irqsave(&musb->Lock, flags);

	if (musb_readl(base, TUSB_PROD_TEST_RESET) !=
		TUSB_PROD_TEST_RESET_VAL) {
		printk(KERN_ERR "tusb: Unable to detect TUSB6010\n");
		goto err;
	}

	ret = tusb_print_revision(musb);
	if (ret < 2) {
		printk(KERN_ERR "tusb: Unsupported TUSB6010 revision %i\n",
				ret);
		goto err;
	}

	/* The uint bit for "USB non-PDR interrupt enable" has to be 1 when
	 * NOR FLASH interface is used */
	musb_writel(base, TUSB_VLYNQ_CTRL, 8);

	/* Select PHY free running 60MHz as a system clock */
	tusb_set_clock_source(musb, 1);

	/* VBus valid timer 1us, disable DFT/Debug and VLYNQ clocks for
	 * power saving, enable VBus detect and session end comparators,
	 * enable IDpullup, enable VBus charging */
	musb_writel(base, TUSB_PRCM_MNGMT,
		TUSB_PRCM_MNGMT_VBUS_VALID_TIMER(0xa) |
		TUSB_PRCM_MNGMT_VBUS_VALID_FLT_EN |
		TUSB_PRCM_MNGMT_OTG_SESS_END_EN |
		TUSB_PRCM_MNGMT_OTG_VBUS_DET_EN |
		TUSB_PRCM_MNGMT_OTG_ID_PULLUP);
	tusb_setup_cpu_interface(musb);

	/* simplify:  always sense/pullup ID pins, as if in OTG mode */
	reg = musb_readl(base, TUSB_PHY_OTG_CTRL);
	reg |= TUSB_PHY_OTG_CTRL_WRPROTECT | TUSB_PHY_OTG_CTRL_OTG_ID_PULLUP;
	musb_writel(base, TUSB_PHY_OTG_CTRL, reg);

	reg = musb_readl(base, TUSB_PHY_OTG_CTRL_ENABLE);
	reg |= TUSB_PHY_OTG_CTRL_WRPROTECT | TUSB_PHY_OTG_CTRL_OTG_ID_PULLUP;
	musb_writel(base, TUSB_PHY_OTG_CTRL_ENABLE, reg);

	spin_unlock_irqrestore(&musb->Lock, flags);

	return 0;

err:
	spin_unlock_irqrestore(&musb->Lock, flags);

	if (musb->board_set_power)
		musb->board_set_power(0);

	return -ENODEV;
}

int __init musb_platform_init(struct musb *musb)
{
	struct platform_device	*pdev;
	struct resource		*mem;
	void __iomem		*sync;
	int			ret;

	pdev = to_platform_device(musb->controller);

	/* dma address for async dma */
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	musb->async = mem->start;

	/* dma address for sync dma */
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!mem) {
		pr_debug("no sync dma resource?\n");
		return -ENODEV;
	}
	musb->sync = mem->start;

	sync = ioremap(mem->start, mem->end - mem->start + 1);
	if (!sync) {
		pr_debug("ioremap for sync failed\n");
		return -ENOMEM;
	}
	musb->sync_va = sync;

	/* Offsets from base: VLYNQ at 0x000, MUSB regs at 0x400,
	 * FIFOs at 0x600, TUSB at 0x800
	 */
	musb->pRegs += TUSB_BASE_OFFSET;

	ret = tusb_start(musb);
	if (ret) {
		printk(KERN_ERR "Could not start tusb6010 (%d)\n",
				ret);
		return -ENODEV;
	}
	musb->isr = tusb_interrupt;

	if (is_host_enabled(musb))
		musb->board_set_vbus = tusb_source_power;
	if (is_peripheral_enabled(musb))
		musb->xceiv.set_power = tusb_draw_power;

	setup_timer(&musb_idle_timer, musb_do_idle, (unsigned long) musb);

	return ret;
}

int musb_platform_exit(struct musb *musb)
{
	del_timer_sync(&musb_idle_timer);

	if (musb->board_set_power)
		musb->board_set_power(0);

	iounmap(musb->sync_va);

	return 0;
}
