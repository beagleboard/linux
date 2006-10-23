/******************************************************************
 * Copyright 2005 Mentor Graphics Corporation
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

/* OTG state machine status 8-mar:
 *
 *   - on DaVinci
 *        + EVM gamma boards have troublesome C133, preventing
 *          conformant timings for A_WAIT_VFALL transitions
 *        + ID-pin based role initialization and VBUS switching
 *          seems partly functional ... seems to bypass this code.
 *        + haven't tried HNP or SRP.
 *
 *   - needs updating along the lines of <linux/usb_otg.h>
 *
 *   - TUSB has a hardware OTG timer, unclear how much of this would
 *     ever be needed for it ...
 *
 *   - doesn't yet use all the linux 2.6.10 usbcore hooks for OTG, but
 *     some of the conversion (and consequent shrinkage) has begun.
 *
 *   - it's not clear if any version of this code ever have passed
 *     the USB-IF OTG tests even at full speed; presumably not.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/smp_lock.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/time.h>
#include <linux/timer.h>

#include "musbdefs.h"
#include "otg.h"


static void otg_set_session(struct musb *musb, u8 bSession)
{
	void __iomem *pBase = musb->pRegs;
	u8 devctl = musb_readb(pBase, MGC_O_HDRC_DEVCTL);

	DBG(2, "<==\n");

	/* REVISIT unclear what this should do, but this looks
	 * like the wrong thing ... the OTG machine should never
	 * shut down so long as both host and peripheral drivers
	 * are active. you'd think the docs would help...
	 */
	if (bSession) {
		devctl |= MGC_M_DEVCTL_SESSION;
		musb_writeb(pBase, MGC_O_HDRC_DEVCTL, devctl);
	} else {
		//devctl &= ~MGC_M_DEVCTL_SESSION;
		musb_root_disconnect(musb);
		//musb_writeb(pBase, MGC_O_HDRC_DEVCTL, devctl);
	}
}

#if 0
static void otg_request_session(struct musb *musb)
{
	void __iomem *pBase = musb->pRegs;
	u8 devctl;

	DBG(2, "<==\n");
	devctl = musb_readb(pBase, MGC_O_HDRC_DEVCTL);
	devctl |= MGC_M_DEVCTL_SESSION;
	musb_writeb(pBase, MGC_O_HDRC_DEVCTL, devctl);
}
#endif

/* caller has irqlocked musb,
 * and if host or peripheral needs to be shut down, already did that.
 */
static void otg_state_changed(struct musb *musb, enum usb_otg_state state)
{
	/* caller should pass the timeout here */
	unsigned long	timer = 0;

	if (state == musb->OtgMachine.bState)
		return;

	DBG(1, "%d --> %d\n", musb->OtgMachine.bState, state);
	musb->OtgMachine.bState = state;

	/* OTG timeouts the hardware doesn't handle:
	 *  - ...
	 */

	switch (state) {
	case OTG_STATE_A_IDLE:
	case OTG_STATE_A_HOST:
	case OTG_STATE_B_HOST:
		MUSB_HST_MODE(musb);
		break;

	case OTG_STATE_B_IDLE:
	case OTG_STATE_B_PERIPHERAL:
	case OTG_STATE_A_PERIPHERAL:
		MUSB_DEV_MODE(musb);
		break;

	default:
		DBG(1, "state change to %d?\n", state);
		/* REVISIT this "otg" mode is goofy; just switch between
		 * default-A and default-B state machines, they already
		 * include disconnect-equivalent states (IDLE).
		 */
		MUSB_OTG_MODE(musb);
		break;
	}

	if (timer)
		mod_timer(&musb->OtgMachine.Timer, jiffies + timer);
	else
		del_timer(&musb->OtgMachine.Timer);

	/* FIXME  the otg state implies MUSB_MODE().  Properly track
	 * xceiv.state, then remove OtgMachine.bState and MUSB_MODE...
	 */
	DBG(2, "==> OTG state %d(%d), mode %s\n",
			state, musb->xceiv.state,
			MUSB_MODE(musb));
}


/**
 * Timer expiration function to complete the interrupt URB on changes
 * @param ptr standard expiration param (hub pointer)
 */
static void otg_timeout(unsigned long ptr)
{
	struct otg_machine	*pMachine = (void *) ptr;
	void __iomem	*mregs;
	u8		devctl;
	struct musb	*musb = pMachine->musb;
	unsigned long	flags;

	DBG(0, "** TIMEOUT ** state %d(%d)\n",
			pMachine->bState, pMachine->musb->xceiv.state);

	/* REVISIT:  a few of these cases _require_ (per the OTG spec)
	 * some sort of user notification, such as turning on an LED
	 * or displaying a message on the screen; INFO() not enough.
	 */

	spin_lock_irqsave(&musb->Lock, flags);
	switch (pMachine->bState) {
	case OTG_STATE_B_SRP_INIT:
		INFO("SRP failed\n");
		otg_set_session(pMachine->musb, FALSE);
		otg_state_changed(pMachine->musb, OTG_STATE_B_IDLE);
		break;

	case OTG_STATE_B_WAIT_ACON:
		INFO("No response from A-device\n");
		mregs = pMachine->musb->pRegs;
		devctl = musb_readb(mregs, MGC_O_HDRC_DEVCTL);
		musb_writeb(mregs, MGC_O_HDRC_DEVCTL,
				devctl & ~MGC_M_DEVCTL_HR);
		otg_set_session(pMachine->musb, TRUE);
		otg_state_changed(pMachine->musb, OTG_STATE_B_PERIPHERAL);
		break;

	case OTG_STATE_A_WAIT_BCON:
		/* REVISIT we'd like to force the VBUS-off path here... */
		INFO("No response from B-device\n");
		otg_set_session(pMachine->musb, FALSE);
		/* transition via OTG_STATE_A_WAIT_VFALL */
		otg_state_changed(pMachine->musb, OTG_STATE_A_IDLE);
		break;

	case OTG_STATE_A_SUSPEND:
		/* FIXME b-dev HNP is _optional_ so this is no error */
		INFO("No B-device HNP response\n");
		otg_set_session(pMachine->musb, FALSE);
		/* transition via OTG_STATE_A_WAIT_VFALL */
		otg_state_changed(pMachine->musb, OTG_STATE_A_IDLE);
		break;

	default:
		WARN("timeout in state %d, now what?\n", pMachine->bState);
	}
	musb_platform_try_idle(musb);
	spin_unlock_irqrestore(&musb->Lock, flags);
}

void MGC_OtgMachineInit(struct otg_machine *pMachine, struct musb *musb)
{
	memset(pMachine, 0, sizeof *pMachine);
	spin_lock_init(&pMachine->Lock);
	pMachine->musb = musb;

	init_timer(&pMachine->Timer);
	pMachine->Timer.function = otg_timeout;
	pMachine->Timer.data = (unsigned long)pMachine;

	pMachine->bState = OTG_STATE_B_IDLE;
	pMachine->bRequest = MGC_OTG_REQUEST_UNKNOWN;
}

void MGC_OtgMachineDestroy(struct otg_machine *pMachine)
{
	/* stop timer */
	del_timer_sync(&pMachine->Timer);
}

/* caller has irqlocked musb */
void MGC_OtgMachineInputsChanged(struct otg_machine *pMachine,
				const MGC_OtgMachineInputs * pInputs)
{

	DBG(2, "<== bState %d(%d)%s%s%s%s%s%s\n",
			pMachine->bState, pMachine->musb->xceiv.state,
			pInputs->bSession ? ", sess" : "",
			pInputs->bSuspend ? ", susp" : "",
			pInputs->bConnection ? ", bcon" : "",
			pInputs->bReset ? ", reset" : "",
			pInputs->bConnectorId ? ", B-Dev" : ", A-Dev",
			pInputs->bVbusError ? ", vbus_error" : "");

	if (pInputs->bVbusError) {
		/* transition via OTG_STATE_VBUS_ERR and
		 * then OTG_STATE_A_WAIT_VFALL
		 */
		otg_state_changed(pMachine->musb, OTG_STATE_A_IDLE);
		return;
	}

	switch (pMachine->bState) {
	case OTG_STATE_B_IDLE:
		if (pInputs->bSession && pInputs->bConnectorId) {
			/* WRONG:  if VBUS is below session threshold,
			 * it's still B_IDLE
			 */
			otg_state_changed(pMachine->musb,
					OTG_STATE_B_PERIPHERAL);
		}
		break;
	case OTG_STATE_A_IDLE:
		if (pInputs->bConnection) {
			/*
			 * SKIP a state because connect IRQ comes so quickly
			 * after setting session,
			 * and only happens in host mode
			 */
			otg_state_changed(pMachine->musb, OTG_STATE_A_HOST);
		} else if (pInputs->bSession) {
			otg_state_changed(pMachine->musb,
					OTG_STATE_A_WAIT_BCON);
			mod_timer(&pMachine->Timer, jiffies
				+ msecs_to_jiffies(MGC_OTG_T_A_WAIT_BCON));
		}
		break;

	case OTG_STATE_B_SRP_INIT:
		if (pInputs->bReset) {
			otg_state_changed(pMachine->musb,
					OTG_STATE_B_PERIPHERAL);
		} else if (pInputs->bConnection) {
			/* FIXME bogus:  there is no such transition!!! */
			otg_state_changed(pMachine->musb,
					pInputs->bConnectorId
						? OTG_STATE_B_HOST
						: OTG_STATE_A_HOST);
		}
		break;

	case OTG_STATE_B_PERIPHERAL:
		if (!pInputs->bSession) {
			otg_state_changed(pMachine->musb, OTG_STATE_B_IDLE);
		}

		/* FIXME nothing ever sets bRequest ... */
		if ((MGC_OTG_REQUEST_START_BUS == pMachine->bRequest)
				&& pMachine->musb->g.b_hnp_enable) {
			otg_state_changed(pMachine->musb,
					OTG_STATE_B_WAIT_ACON);
			//mdelay(10);
			//otg_set_session(pMachine->musb, FALSE);
			mod_timer(&pMachine->Timer, jiffies
				+ msecs_to_jiffies(MGC_OTG_T_B_ASE0_BRST));
		}
		break;

	case OTG_STATE_B_WAIT_ACON:
		if (pInputs->bConnection) {
			otg_state_changed(pMachine->musb, OTG_STATE_B_HOST);
		} else if (!pInputs->bSession) {
			otg_state_changed(pMachine->musb, OTG_STATE_B_IDLE);
		} else if (!pInputs->bSuspend) {
			otg_state_changed(pMachine->musb,
					OTG_STATE_B_PERIPHERAL);
		}
		break;

	case OTG_STATE_B_HOST:
		if (!pInputs->bConnection) {
			otg_state_changed(pMachine->musb, OTG_STATE_B_IDLE);
		} else if (pInputs->bConnection && !pInputs->bReset) {
			/* REVISIT seems incomplete */
		}
		break;

	case OTG_STATE_A_WAIT_BCON:
		if (pInputs->bConnection) {
			otg_state_changed(pMachine->musb, OTG_STATE_A_HOST);
		} else if (pInputs->bReset) {
			/* FIXME there is no such transition */
			otg_state_changed(pMachine->musb,
					OTG_STATE_A_PERIPHERAL);
		}
		break;

	case OTG_STATE_A_HOST:
		if (!pInputs->bConnection) {
			otg_state_changed(pMachine->musb,
					OTG_STATE_A_WAIT_BCON);
			mod_timer(&pMachine->Timer, jiffies
				+ msecs_to_jiffies(MGC_OTG_T_A_WAIT_BCON));
		} else if (pInputs->bConnection && !pInputs->bReset) {
			/* REVISIT seems incomplete */
		}
		break;

	case OTG_STATE_A_SUSPEND:
		if (!pInputs->bSuspend) {
			otg_state_changed(pMachine->musb, OTG_STATE_A_HOST);
		} else if (!pInputs->bConnection) {
			if (musb_to_hcd(pMachine->musb)->self.b_hnp_enable) {
				otg_state_changed(pMachine->musb,
						OTG_STATE_A_PERIPHERAL);
			} else {
				otg_state_changed(pMachine->musb,
						OTG_STATE_A_WAIT_BCON);
				mod_timer(&pMachine->Timer, jiffies
					+ msecs_to_jiffies(
						MGC_OTG_T_A_WAIT_BCON));
			}
		}
		break;

	case OTG_STATE_A_PERIPHERAL:
		if (!pInputs->bSession) {
			/* transition via OTG_STATE_A_WAIT_VFALL */
			otg_state_changed(pMachine->musb, OTG_STATE_A_IDLE);
		} else if (pInputs->bSuspend) {
			otg_state_changed(pMachine->musb,
					OTG_STATE_A_WAIT_BCON);
			mod_timer(&pMachine->Timer, jiffies
				+ msecs_to_jiffies(MGC_OTG_T_A_WAIT_BCON));
		}
		break;

	default:
		WARN("event in state %d, now what?\n", pMachine->bState);
	}
}
