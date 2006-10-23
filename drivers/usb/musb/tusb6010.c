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
 * TUSB 6010 may use a parallel bus that doesn't support byte ops;
 * so both loading and unloading FIFOs need explicit byte counts.
 */

void musb_write_fifo(struct musb_hw_ep *hw_ep, u16 len, const u8 *buf)
{
	void __iomem	*ep_conf = hw_ep->conf;
	void __iomem	*fifo = hw_ep->fifo;
	u8		epnum = hw_ep->bLocalEnd;
	u8		*bufp = (u8 *)buf;
	int		i, remain;
	u32		val;

	prefetch(bufp);

	DBG(4, "%cX ep%d fifo %p count %d buf %p\n",
			'T', epnum, fifo, len, bufp);

	if (epnum)
		musb_writel(ep_conf, TUSB_EP_TX_OFFSET,
			TUSB_EP_CONFIG_XFR_SIZE(len));
	else
		musb_writel(ep_conf, 0, TUSB_EP0_CONFIG_DIR_TX |
			TUSB_EP0_CONFIG_XFR_SIZE(len));

	/* Write 32-bit blocks from buffer to FIFO
	 * REVISIT: Optimize for burst ... writesl/writesw
	 */
	if (len >= 4) {
		if (((unsigned long)bufp & 0x3) == 0) {
			for (i = 0; i < (len / 4); i++ ) {
				val = *(u32 *)bufp;
				bufp += 4;
				musb_writel(fifo, 0, val);
			}
		} else if (((unsigned long)bufp & 0x2) == 0x2) {
			for (i = 0; i < (len / 4); i++ ) {
				val = (u32)(*(u16 *)bufp);
				bufp += 2;
				val |= (*(u16 *)bufp) << 16;
				bufp += 2;
				musb_writel(fifo, 0, val);
			}
		} else {
			for (i = 0; i < (len / 4); i++ ) {
				memcpy(&val, bufp, 4);
				bufp += 4;
				musb_writel(fifo, 0, val);
			}
		}
		remain = len - (i * 4);
	} else
		remain = len;

	if (remain) {
		/* Write rest of 1-3 bytes from buffer into FIFO */
		memcpy(&val, bufp, remain);
		musb_writel(fifo, 0, val);
	}
}

void musb_read_fifo(struct musb_hw_ep *hw_ep, u16 len, u8 *buf)
{
	void __iomem	*ep_conf = hw_ep->conf;
	void __iomem	*fifo = hw_ep->fifo;
	u8		epnum = hw_ep->bLocalEnd;
	u8		*bufp = (u8 *)buf;
	int		i, remain;
	u32		val;

	DBG(4, "%cX ep%d fifo %p count %d buf %p\n",
			'R', epnum, fifo, len, bufp);

	if (epnum)
		musb_writel(ep_conf, TUSB_EP_RX_OFFSET,
			TUSB_EP_CONFIG_XFR_SIZE(len));
	else
		musb_writel(ep_conf, 0, TUSB_EP0_CONFIG_XFR_SIZE(len));

	/* Read 32-bit blocks from FIFO to buffer
	 * REVISIT: Optimize for burst ... writesl/writesw
	 */
	if (len >= 4) {
		if (((unsigned long)bufp & 0x3) == 0) {
			for (i = 0; i < (len / 4); i++) {
				val = musb_readl(fifo, 0);
				*(u32 *)bufp = val;
				bufp += 4;
			}
		} else if (((unsigned long)bufp & 0x2) == 0x2) {
			for (i = 0; i < (len / 4); i++) {
				val = musb_readl(fifo, 0);
				*(u16 *)bufp = (u16)(val & 0xffff);
				bufp += 2;
				*(u16 *)bufp = (u16)(val >> 16);
				bufp += 2;
			}
		} else {
			for (i = 0; i < (len / 4); i++) {
				val = musb_readl(fifo, 0);
				memcpy(bufp, &val, 4);
				bufp += 4;
			}
		}
		remain = len - (i * 4);
	} else
		remain = len;

	if (remain) {
		/* Read rest of 1-3 bytes from FIFO */
		val = musb_readl(fifo, 0);
		memcpy(bufp, &val, remain);
	}
}

/* This is used by gadget drivers, and OTG transceiver logic, allowing
 * at most mA current to be drawn from VBUS during a Default-B session
 * (that is, while VBUS exceeds 4.4V).  In Default-A (including pure host
 * mode), or low power Default-B sessions, something else supplies power.
 */
static int tusb_set_power(struct otg_transceiver *x, unsigned mA)
{
	struct musb	*musb = container_of(x, struct musb, xceiv);
	void __iomem	*base = musb->ctrl_base;
	u32		reg;

	/* tps65030 seems to consume max 100mA, with maybe 60mA available
	 * (measured on one board) for things other than tps and tusb.
	 *
	 * REVISIT we could use VBUS to supply only _one_ of { 1.5V, 3.3V }.
	 * The actual current usage would be very board-specific.  For now,
	 * it's simpler to just use an aggregate (also board-specific).
	 */
	if (x->default_a || mA < (musb->min_power << 1))
		mA = 0;

	reg = musb_readl(base, TUSB_PRCM_MNGMT);
	if (mA)
		reg |= TUSB_PRCM_MNGMT_15_SW_EN | TUSB_PRCM_MNGMT_33_SW_EN;
	else
		reg &= ~(TUSB_PRCM_MNGMT_15_SW_EN | TUSB_PRCM_MNGMT_33_SW_EN);
	musb_writel(base, TUSB_PRCM_MNGMT, reg);

	DBG(2, "draw max %d mA VBUS\n", mA);
	return 0;
}

/* workaround for issue 13:  change clock during chip idle
 * (to be fixed in rev3 silicon) ... symptoms include disconnect
 * or looping suspend/resume cycles
 */
static void tusb_set_clock_source(struct musb *musb, int mode)
{
	void __iomem	*base = musb->ctrl_base;
	u32		reg;

	reg = musb_readl(base, TUSB_PRCM_CONF);
	reg &= ~TUSB_PRCM_CONF_SYS_CLKSEL(0x3);

	if (mode > 0)
		reg |= TUSB_PRCM_CONF_SYS_CLKSEL(mode & 0x3);

	musb_writel(base, TUSB_PRCM_CONF, reg);
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

	DBG(2, "idle, wake on %02x\n", wakeup_enables);
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

static void tusb_set_vbus(struct musb *musb, int is_on)
{
	void __iomem	*base = musb->ctrl_base;
	u32		conf, prcm, timer;
	u8		devctl;

	/* we control CPEN in software not hardware ... */

	prcm = musb_readl(base, TUSB_PRCM_MNGMT);
	conf = musb_readl(base, TUSB_DEV_CONF);
	devctl = musb_readb(musb->pRegs, MGC_O_HDRC_DEVCTL);

	if (is_on) {
		musb->is_active = 1;
		prcm |= TUSB_PRCM_MNGMT_5V_CPEN;
		timer = OTG_TIMER_MS(OTG_TIME_A_WAIT_VRISE);

		musb->xceiv.default_a = 1;
		musb->xceiv.state = OTG_STATE_A_WAIT_VRISE;

		conf |= TUSB_DEV_CONF_USB_HOST_MODE;

	} else {
		prcm &= ~TUSB_PRCM_MNGMT_5V_CPEN;
		timer = 0;

		if (musb->xceiv.default_a) {
			musb->xceiv.state = OTG_STATE_A_WAIT_VFALL;
			devctl &= ~MGC_M_DEVCTL_SESSION;
		} else {
			musb->xceiv.state = OTG_STATE_B_IDLE;
			musb->is_active = 0;
		}
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
		tusb_set_vbus(musb, default_a);
	}

	/* VBUS state change */
	if (int_src & TUSB_INT_SRC_VBUS_SENSE_CHNG) {

		/* B-dev state machine:  no vbus ~= disconnect */
		if ((is_otg_enabled(musb) && !musb->xceiv.default_a)
				|| !is_host_enabled(musb)) {

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
				/* REVISIT this irq triggers at too high a
				 * voltage ... we probably need to use the
				 * OTG timer to wait for session end.
				 */
				if (musb->vbuserr_retry) {
					musb->vbuserr_retry--;
					tusb_set_vbus(musb, 1);
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
			/* VBUS has probably been valid for a while now */
			devctl = musb_readb(musb->pRegs, MGC_O_HDRC_DEVCTL);
			if (otg_stat & TUSB_DEV_OTG_STAT_VBUS_VALID) {
				if ((devctl & MGC_M_DEVCTL_VBUS)
						!= MGC_M_DEVCTL_VBUS) {
					DBG(2, "devctl %02x\n", devctl);
					break;
				}

				/* request a session, then DEVCTL_HM will
				 * be set by the controller
				 */
				devctl |= MGC_M_DEVCTL_SESSION;
				musb_writeb(musb->pRegs, MGC_O_HDRC_DEVCTL,
						devctl);
				musb->xceiv.state = OTG_STATE_A_WAIT_BCON;

				/* timeout 0 == infinite (like non-OTG hosts) */
				if (OTG_TIME_A_WAIT_BCON)
					musb_writel(base, TUSB_DEV_OTG_TIMER,
						OTG_TIMER_MS(OTG_TIME_A_WAIT_BCON));
				else
					musb_writel(base, TUSB_DEV_OTG_TIMER, 0);
			} else {
				ERR("vbus too slow, devctl %02x\n", devctl);
				tusb_set_vbus(musb, 0);
			}
			break;
		case OTG_STATE_A_WAIT_BCON:
			if (OTG_TIME_A_WAIT_BCON)
				tusb_set_vbus(musb, 0);
			break;
		case OTG_STATE_A_SUSPEND:
			break;
		case OTG_STATE_B_WAIT_ACON:
			break;
		default:
			break;
		}
		musb_writel(base, TUSB_DEV_OTG_TIMER, 0);
	}
}

static irqreturn_t tusb_interrupt(int irq, void *__hci, struct pt_regs *r)
{
	struct musb	*musb = __hci;
	void __iomem	*base = musb->ctrl_base;
	unsigned long	flags;
	u32		int_src;

	spin_lock_irqsave(&musb->Lock, flags);

	int_src = musb_readl(base, TUSB_INT_SRC) & ~TUSB_INT_SRC_RESERVED_BITS;
	DBG(3, "TUSB IRQ %08x\n", int_src);

	musb->int_regs = r;
	musb->int_usb = (u8) int_src;

	/* Acknowledge wake-up source interrupts */
	if (int_src & TUSB_INT_SRC_DEV_WAKEUP) {
		u32	reg;
		u32	i;

		/* there are issues re-locking the PLL on wakeup ... */

		/* work around issue 8 */
		for (i = 0xf7f7f7; i > 0xf7f7f7 - 1000; i--) {
			musb_writel(base, TUSB_SCRATCH_PAD, 0);
			musb_writel(base, TUSB_SCRATCH_PAD, i);
			reg = musb_readl(base, TUSB_SCRATCH_PAD);
			if (reg == i)
				break;
			DBG(1, "TUSB NOR not ready\n");
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

	musb->int_regs = NULL;
	musb_platform_try_idle(musb);
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
	/* FIXME stop DMA, IRQs, timers, ... */

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
static void tusb_setup_cpu_interface(struct musb *musb)
{
	void __iomem	*base = musb->ctrl_base;

	/* Disable GPIO[7:0] pullups (used as output DMA requests) */
	musb_writel(base, TUSB_PULLUP_1_CTRL, 0x000000FF);
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

#define TUSB_REV_MAJOR(reg_val)		((reg_val >> 4) & 0xf)
#define TUSB_REV_MINOR(reg_val)		(reg_val & 0xf)

static int tusb_print_revision(struct musb *musb)
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

static int tusb_start(struct musb *musb)
{
	void __iomem	*base = musb->ctrl_base;
	int		ret = -1;
	unsigned long	flags;
	u32		reg;

	if (musb->board_set_power)
		ret = musb->board_set_power(1);
	if (ret != 0) {
		printk(KERN_ERR "tusb: Cannot enable TUSB6010\n");
		goto err;
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
	musb_writel(base, TUSB_PRCM_CONF, //FIXME: CPEN should not be needed!
			TUSB_PRCM_CONF_SFW_CPEN | TUSB_PRCM_CONF_SYS_CLKSEL(1));

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
	if (musb->board_set_power)
		musb->board_set_power(0);

	return -ENODEV;
}

int __devinit musb_platform_init(struct musb *musb)
{
	struct platform_device	*pdev;
	struct resource		*mem;
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
		musb->board_set_vbus = tusb_set_vbus;
	if (is_peripheral_enabled(musb))
		musb->xceiv.set_power = tusb_set_power;

	setup_timer(&musb_idle_timer, musb_do_idle, (unsigned long) musb);

	return ret;
}

int musb_platform_exit(struct musb *musb)
{
	if (musb->board_set_power)
		musb->board_set_power(0);

	return 0;
}
