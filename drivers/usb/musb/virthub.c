/*****************************************************************
 * Copyright 2005 Mentor Graphics Corporation
 * Copyright (C) 2005-2006 by Texas Instruments
 * Copyright (C) 2006 by Nokia Corporation
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
 * ANY DOWNLOAD, USE, REPRODUCTION, MODIFICATION OR DISTRIBUTION
 * OF THIS DRIVER INDICATES YOUR COMPLETE AND UNCONDITIONAL ACCEPTANCE
 * OF THOSE TERMS.THIS DRIVER IS PROVIDED "AS IS" AND MENTOR GRAPHICS
 * MAKES NO WARRANTIES, EXPRESS OR IMPLIED, RELATED TO THIS DRIVER.
 * MENTOR GRAPHICS SPECIFICALLY DISCLAIMS ALL IMPLIED WARRANTIES
 * OF MERCHANTABILITY; FITNESS FOR A PARTICULAR PURPOSE AND
 * NON-INFRINGEMENT.  MENTOR GRAPHICS DOES NOT PROVIDE SUPPORT
 * SERVICES OR UPDATES FOR THIS DRIVER, EVEN IF YOU ARE A MENTOR
 * GRAPHICS SUPPORT CUSTOMER.
 ******************************************************************/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/time.h>
#include <linux/timer.h>

#include <asm/unaligned.h>

#include "musbdefs.h"


static void musb_port_suspend(struct musb *musb, u8 bSuspend)
{
	u8		power;
	void __iomem	*pBase = musb->pRegs;

	if (!is_host_active(musb))
		return;

	/* NOTE:  this doesn't necessarily put PHY into low power mode,
	 * turning off its clock; that's a function of PHY integration and
	 * MGC_M_POWER_ENSUSPEND.  PHY may need a clock (sigh) to detect
	 * SE0 changing to connect (J) or wakeup (K) states.
	 */
	power = musb_readb(pBase, MGC_O_HDRC_POWER);
	if (bSuspend) {
		int retries = 10000;

		power &= ~MGC_M_POWER_RESUME;
		power |= MGC_M_POWER_SUSPENDM;
		musb_writeb(pBase, MGC_O_HDRC_POWER, power);

		/* Needed for OPT A tests */
		power = musb_readb(pBase, MGC_O_HDRC_POWER);
		while (power & MGC_M_POWER_SUSPENDM) {
			power = musb_readb(pBase, MGC_O_HDRC_POWER);
			if (retries-- < 1)
				break;
		}

		DBG(3, "Root port suspended, power %02x\n", power);

		musb->port1_status |= USB_PORT_STAT_SUSPEND;
		switch (musb->xceiv.state) {
		case OTG_STATE_A_HOST:
			musb->xceiv.state = OTG_STATE_A_SUSPEND;
			musb->is_active = is_otg_enabled(musb)
					&& musb->xceiv.host->b_hnp_enable;
			musb_platform_try_idle(musb, 0);
			break;
		case OTG_STATE_B_HOST:
			musb->xceiv.state = OTG_STATE_B_PERIPHERAL;
			MUSB_DEV_MODE(musb);
			/* REVISIT restore setting of MGC_M_DEVCTL_HR */
			break;
		default:
			DBG(1, "bogus rh suspend? %s\n",
				otg_state_string(musb));
		}
	} else if (power & MGC_M_POWER_SUSPENDM) {
		power &= ~MGC_M_POWER_SUSPENDM;
		power |= MGC_M_POWER_RESUME;
		musb_writeb(pBase, MGC_O_HDRC_POWER, power);

		DBG(3, "Root port resuming, power %02x\n", power);

		/* later, GetPortStatus will stop RESUME signaling */
		musb->port1_status |= MUSB_PORT_STAT_RESUME;
		musb->rh_timer = jiffies + msecs_to_jiffies(20);
	}
}

static void musb_port_reset(struct musb *musb, u8 bReset)
{
	u8		power;
	void __iomem	*pBase = musb->pRegs;

#ifdef CONFIG_USB_MUSB_OTG
	/* REVISIT this looks wrong for HNP */
	u8 devctl = musb_readb(pBase, MGC_O_HDRC_DEVCTL);

	if (musb->bDelayPortPowerOff || !(devctl & MGC_M_DEVCTL_HM)) {
		return;
	}
#endif

	if (!is_host_active(musb))
		return;

	/* NOTE:  caller guarantees it will turn off the reset when
	 * the appropriate amount of time has passed
	 */
	power = musb_readb(pBase, MGC_O_HDRC_POWER);
	if (bReset) {
		musb->bIgnoreDisconnect = TRUE;
		power &= 0xf0;
		musb_writeb(pBase, MGC_O_HDRC_POWER,
				power | MGC_M_POWER_RESET);

		musb->port1_status |= USB_PORT_STAT_RESET;
		musb->port1_status &= ~USB_PORT_STAT_ENABLE;
		musb->rh_timer = jiffies + msecs_to_jiffies(50);
	} else {
		DBG(4, "root port reset stopped\n");
		musb_writeb(pBase, MGC_O_HDRC_POWER,
				power & ~MGC_M_POWER_RESET);

		musb->bIgnoreDisconnect = FALSE;

		power = musb_readb(pBase, MGC_O_HDRC_POWER);
		if (power & MGC_M_POWER_HSMODE) {
			DBG(4, "high-speed device connected\n");
			musb->port1_status |= USB_PORT_STAT_HIGH_SPEED;
		}

		musb->port1_status &= ~USB_PORT_STAT_RESET;
		musb->port1_status |= USB_PORT_STAT_ENABLE
					| (USB_PORT_STAT_C_RESET << 16)
					| (USB_PORT_STAT_C_ENABLE << 16);
		usb_hcd_poll_rh_status(musb_to_hcd(musb));

		musb->vbuserr_retry = VBUSERR_RETRY_COUNT;
	}
}

void musb_root_disconnect(struct musb *musb)
{
	musb->port1_status = (1 << USB_PORT_FEAT_POWER)
			| (1 << USB_PORT_FEAT_C_CONNECTION);

	usb_hcd_poll_rh_status(musb_to_hcd(musb));
	musb->is_active = 0;

	switch (musb->xceiv.state) {
	case OTG_STATE_A_HOST:
	case OTG_STATE_A_SUSPEND:
		musb->xceiv.state = OTG_STATE_A_WAIT_BCON;
		break;
	case OTG_STATE_A_WAIT_VFALL:
		musb->xceiv.state = OTG_STATE_B_IDLE;
		break;
	default:
		DBG(1, "host disconnect (%s)\n", otg_state_string(musb));
	}
}


/*---------------------------------------------------------------------*/

int musb_hub_status_data(struct usb_hcd *hcd, char *buf)
{
	struct musb	*musb = hcd_to_musb(hcd);
	int		retval = 0;

	/* called in_irq() via usb_hcd_poll_rh_status() */
	if (musb->port1_status & 0xffff0000) {
		*buf = 0x02;
		retval = 1;
	}
	return retval;
}

int musb_hub_control(
	struct usb_hcd	*hcd,
	u16		typeReq,
	u16		wValue,
	u16		wIndex,
	char		*buf,
	u16		wLength)
{
	struct musb	*musb = hcd_to_musb(hcd);
	u32		temp;
	int		retval = 0;
	unsigned long	flags;

	if (unlikely(!test_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags)))
		return -ESHUTDOWN;

	/* hub features:  always zero, setting is a NOP
	 * port features: reported, sometimes updated when host is active
	 * no indicators
	 */
	spin_lock_irqsave(&musb->Lock, flags);
	switch (typeReq) {
	case ClearHubFeature:
	case SetHubFeature:
		switch (wValue) {
		case C_HUB_OVER_CURRENT:
		case C_HUB_LOCAL_POWER:
			break;
		default:
			goto error;
		}
		break;
	case ClearPortFeature:
		if (wIndex != 1)
			goto error;

		switch (wValue) {
		case USB_PORT_FEAT_ENABLE:
			break;
		case USB_PORT_FEAT_SUSPEND:
			musb_port_suspend(musb, FALSE);
			break;
		case USB_PORT_FEAT_POWER:
			if (!(is_otg_enabled(musb) && hcd->self.is_b_host))
				musb_set_vbus(musb, 0);
			break;
		case USB_PORT_FEAT_C_CONNECTION:
		case USB_PORT_FEAT_C_ENABLE:
		case USB_PORT_FEAT_C_OVER_CURRENT:
		case USB_PORT_FEAT_C_RESET:
		case USB_PORT_FEAT_C_SUSPEND:
			break;
		default:
			goto error;
		}
		DBG(5, "clear feature %d\n", wValue);
		musb->port1_status &= ~(1 << wValue);
		break;
	case GetHubDescriptor:
		{
		struct usb_hub_descriptor *desc = (void *)buf;

		desc->bDescLength = 9;
		desc->bDescriptorType = 0x29;
		desc->bNbrPorts = 1;
		desc->wHubCharacteristics = __constant_cpu_to_le16(
				  0x0001	/* per-port power switching */
				| 0x0010	/* no overcurrent reporting */
				);
		desc->bPwrOn2PwrGood = 5;	/* msec/2 */
		desc->bHubContrCurrent = 0;

		/* workaround bogus struct definition */
		desc->DeviceRemovable[0] = 0x02;	/* port 1 */
		desc->DeviceRemovable[1] = 0xff;
		}
		break;
	case GetHubStatus:
		temp = 0;
		*(__le32 *) buf = cpu_to_le32 (temp);
		break;
	case GetPortStatus:
		if (wIndex != 1)
			goto error;

		/* finish RESET signaling? */
		if ((musb->port1_status & USB_PORT_STAT_RESET)
				&& time_after(jiffies, musb->rh_timer))
			musb_port_reset(musb, FALSE);

		/* finish RESUME signaling? */
		if ((musb->port1_status & MUSB_PORT_STAT_RESUME)
				&& time_after(jiffies, musb->rh_timer)) {
			u8		power;

			power = musb_readb(musb->pRegs, MGC_O_HDRC_POWER);
			power &= ~MGC_M_POWER_RESUME;
			DBG(4, "root port resume stopped, power %02x\n",
					power);
			musb_writeb(musb->pRegs, MGC_O_HDRC_POWER, power);

			/* ISSUE:  DaVinci (RTL 1.300) disconnects after
			 * resume of high speed peripherals (but not full
			 * speed ones).
			 */

			musb->is_active = 1;
			musb->port1_status &= ~(USB_PORT_STAT_SUSPEND
					| MUSB_PORT_STAT_RESUME);
			musb->port1_status |= USB_PORT_STAT_C_SUSPEND << 16;
			usb_hcd_poll_rh_status(musb_to_hcd(musb));
			/* NOTE: it might really be A_WAIT_BCON ... */
			musb->xceiv.state = OTG_STATE_A_HOST;
		}

		put_unaligned(cpu_to_le32(musb->port1_status & ~MUSB_PORT_STAT_RESUME),
				(__le32 *) buf);

		/* port change status is more interesting */
		DBG(get_unaligned((u16*)(buf+2)) ? 2 : 5, "port status %08x\n",
				musb->port1_status);
		break;
	case SetPortFeature:
		if ((wIndex & 0xff) != 1)
			goto error;

		switch (wValue) {
		case USB_PORT_FEAT_POWER:
			/* NOTE: this controller has a strange state machine
			 * that involves "requesting sessions" according to
			 * magic side effects from incompletely-described
			 * rules about startup...
			 *
			 * This call is what really starts the host mode; be
			 * very careful about side effects if you reorder any
			 * initialization logic, e.g. for OTG, or change any
			 * logic relating to VBUS power-up.
			 */
			if (!(is_otg_enabled(musb) && hcd->self.is_b_host))
				musb_start(musb);
			break;
		case USB_PORT_FEAT_RESET:
			musb_port_reset(musb, TRUE);
			break;
		case USB_PORT_FEAT_SUSPEND:
			musb_port_suspend(musb, TRUE);
			break;
		case USB_PORT_FEAT_TEST:
			if (unlikely(is_host_active(musb)))
				goto error;

			wIndex >>= 8;
			switch (wIndex) {
			case 1:
				pr_debug("TEST_J\n");
				temp = MGC_M_TEST_J;
				break;
			case 2:
				pr_debug("TEST_K\n");
				temp = MGC_M_TEST_K;
				break;
			case 3:
				pr_debug("TEST_SE0_NAK\n");
				temp = MGC_M_TEST_SE0_NAK;
				break;
			case 4:
				pr_debug("TEST_PACKET\n");
				temp = MGC_M_TEST_PACKET;
				musb_load_testpacket(musb);
				break;
			case 5:
				pr_debug("TEST_FORCE_ENABLE\n");
				temp = MGC_M_TEST_FORCE_HOST
					| MGC_M_TEST_FORCE_HS;

				musb_writeb(musb->pRegs, MGC_O_HDRC_DEVCTL, MGC_M_DEVCTL_SESSION);
				break;
			case 6:
				pr_debug("TEST_FIFO_ACCESS\n");
				temp = MGC_M_TEST_FIFO_ACCESS;
				break;
			default:
				goto error;
			}
			musb_writeb(musb->pRegs, MGC_O_HDRC_TESTMODE, temp);
			break;
		default:
			goto error;
		}
		DBG(5, "set feature %d\n", wValue);
		musb->port1_status |= 1 << wValue;
		break;

	default:
error:
		/* "protocol stall" on error */
		retval = -EPIPE;
	}
	spin_unlock_irqrestore(&musb->Lock, flags);
	return retval;
}
