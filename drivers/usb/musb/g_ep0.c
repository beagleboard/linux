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

#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/timer.h>
#include <linux/spinlock.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/interrupt.h>

#include "musbdefs.h"

/* ep0 is always musb->aLocalEnd[0].ep_in */
#define	next_ep0_request(musb)	next_in_request(&(musb)->aLocalEnd[0])

/*
 * Locking note:  we use only the controller lock, for simpler correctness.
 * It's always held with IRQs blocked.
 *
 * It protects the ep0 request queue as well as ep0_state, not just the
 * controller and indexed registers.  And that lock stays held unless it
 * needs to be dropped to allow reentering this driver ... like upcalls to
 * the gadget driver, or adjusting endpoint halt status.
 */

static char *decode_ep0stage(u8 stage)
{
	switch(stage) {
	case MGC_END0_STAGE_SETUP:	return "idle";
	case MGC_END0_STAGE_TX:		return "in";
	case MGC_END0_STAGE_RX:		return "out";
	case MGC_END0_STAGE_ACKWAIT:	return "wait";
	case MGC_END0_STAGE_STATUSIN:	return "in/status";
	case MGC_END0_STAGE_STATUSOUT:	return "out/status";
	default:			return "?";
	}
}

/* handle a standard GET_STATUS request
 * Context:  caller holds controller lock
 */
static int service_tx_status_request(
	struct musb *musb,
	const struct usb_ctrlrequest *pControlRequest)
{
	void __iomem	*pBase = musb->pRegs;
	int handled = 1;
	u8 bResult[2], bEnd = 0;
	const u8 bRecip = pControlRequest->bRequestType & USB_RECIP_MASK;

	bResult[1] = 0;

	switch (bRecip) {
	case USB_RECIP_DEVICE:
		bResult[0] = musb->is_self_powered << USB_DEVICE_SELF_POWERED;
		bResult[0] |= musb->may_wakeup << USB_DEVICE_REMOTE_WAKEUP;
#ifdef CONFIG_USB_MUSB_OTG
		if (musb->g.is_otg) {
			bResult[0] |= musb->g.b_hnp_enable
				<< USB_DEVICE_B_HNP_ENABLE;
			bResult[0] |= musb->g.a_alt_hnp_support
				<< USB_DEVICE_A_ALT_HNP_SUPPORT;
			bResult[0] |= musb->g.a_hnp_support
				<< USB_DEVICE_A_HNP_SUPPORT;
		}
#endif
		break;

	case USB_RECIP_INTERFACE:
		bResult[0] = 0;
		break;

	case USB_RECIP_ENDPOINT: {
		int		is_in;
		struct musb_ep	*ep;
		u16		tmp;
		void __iomem	*regs;

		bEnd = (u8) pControlRequest->wIndex;
		if (!bEnd) {
			bResult[0] = 0;
			break;
		}

		is_in = bEnd & USB_DIR_IN;
		if (is_in) {
			bEnd &= 0x0f;
			ep = &musb->aLocalEnd[bEnd].ep_in;
		} else {
			ep = &musb->aLocalEnd[bEnd].ep_out;
		}
		regs = musb->aLocalEnd[bEnd].regs;

		if (bEnd >= MUSB_C_NUM_EPS || !ep->desc) {
			handled = -EINVAL;
			break;
		}

		MGC_SelectEnd(pBase, bEnd);
		if (is_in)
			tmp = musb_readw(regs, MGC_O_HDRC_TXCSR)
						& MGC_M_TXCSR_P_SENDSTALL;
		else
			tmp = musb_readw(regs, MGC_O_HDRC_RXCSR)
						& MGC_M_RXCSR_P_SENDSTALL;
		MGC_SelectEnd(pBase, 0);

		bResult[0] = tmp ? 1 : 0;
		} break;

	default:
		/* class, vendor, etc ... delegate */
		handled = 0;
		break;
	}

	/* fill up the fifo; caller updates csr0 */
	if (handled > 0) {
		u16	len = le16_to_cpu(pControlRequest->wLength);

		if (len > 2)
			len = 2;
		musb_write_fifo(&musb->aLocalEnd[0], len, bResult);
	}

	return handled;
}

/*
 * handle a control-IN request, the end0 buffer contains the current request
 * that is supposed to be a standard control request. Assumes the fifo to
 * be at least 2 bytes long.
 *
 * @return 0 if the request was NOT HANDLED,
 * < 0 when error
 * > 0 when the request is processed
 *
 * Context:  caller holds controller lock
 */
static int
service_in_request(struct musb *musb,
		const struct usb_ctrlrequest *pControlRequest)
{
	int handled = 0;	/* not handled */

	if ((pControlRequest->bRequestType & USB_TYPE_MASK)
			== USB_TYPE_STANDARD) {
		switch (pControlRequest->bRequest) {
		case USB_REQ_GET_STATUS:
			handled = service_tx_status_request(musb,
					pControlRequest);
			break;

		/* case USB_REQ_SYNC_FRAME: */

		default:
			break;
		}
	}
	return handled;
}

/*
 * Context:  caller holds controller lock
 */
static void musb_g_ep0_giveback(struct musb *musb, struct usb_request *req)
{
	musb->ep0_state = MGC_END0_STAGE_SETUP;
	musb_g_giveback(&musb->aLocalEnd[0].ep_in, req, 0);
}

/*
 * Tries to start B-device HNP negotiation if enabled via sysfs
 */
static inline void musb_try_b_hnp_enable(struct musb *musb)
{
	void __iomem	*pBase = musb->pRegs;
	u8		devctl;

	DBG(1, "HNP: Setting HR\n");
	devctl = musb_readb(pBase, MGC_O_HDRC_DEVCTL);
	musb_writeb(pBase, MGC_O_HDRC_DEVCTL, devctl | MGC_M_DEVCTL_HR);
}

/*
 * Handle all control requests with no DATA stage, including standard
 * requests such as:
 * USB_REQ_SET_CONFIGURATION, USB_REQ_SET_INTERFACE, unrecognized
 *	always delegated to the gadget driver
 * USB_REQ_SET_ADDRESS, USB_REQ_CLEAR_FEATURE, USB_REQ_SET_FEATURE
 *	always handled here, except for class/vendor/... features
 *
 * Context:  caller holds controller lock
 */
static int
service_zero_data_request(struct musb *musb,
		struct usb_ctrlrequest *pControlRequest)
__releases(musb->Lock)
__acquires(musb->Lock)
{
	int handled = -EINVAL;
	void __iomem *pBase = musb->pRegs;
	const u8 bRecip = pControlRequest->bRequestType & USB_RECIP_MASK;

	/* the gadget driver handles everything except what we MUST handle */
	if ((pControlRequest->bRequestType & USB_TYPE_MASK)
			== USB_TYPE_STANDARD) {
		switch (pControlRequest->bRequest) {
		case USB_REQ_SET_ADDRESS:
			/* change it after the status stage */
			musb->bSetAddress = TRUE;
			musb->bAddress = (u8) (pControlRequest->wValue & 0x7f);
			handled = 1;
			break;

		case USB_REQ_CLEAR_FEATURE:
			switch (bRecip) {
			case USB_RECIP_DEVICE:
				if (pControlRequest->wValue
						!= USB_DEVICE_REMOTE_WAKEUP)
					break;
				musb->may_wakeup = 0;
				handled = 1;
				break;
			case USB_RECIP_INTERFACE:
				break;
			case USB_RECIP_ENDPOINT:{
				const u8 bEnd = pControlRequest->wIndex & 0x0f;
				struct musb_ep *pEnd;

				if (bEnd == 0
						|| bEnd >= MUSB_C_NUM_EPS
						|| pControlRequest->wValue
							!= USB_ENDPOINT_HALT)
					break;

				if (pControlRequest->wIndex & USB_DIR_IN)
					pEnd = &musb->aLocalEnd[bEnd].ep_in;
				else
					pEnd = &musb->aLocalEnd[bEnd].ep_out;
				if (!pEnd->desc)
					break;

				/* REVISIT do it directly, no locking games */
				spin_unlock(&musb->Lock);
				musb_gadget_set_halt(&pEnd->end_point, 0);
				spin_lock(&musb->Lock);

				/* select ep0 again */
				MGC_SelectEnd(pBase, 0);
				handled = 1;
				} break;
			default:
				/* class, vendor, etc ... delegate */
				handled = 0;
				break;
			}
			break;

		case USB_REQ_SET_FEATURE:
			switch (bRecip) {
			case USB_RECIP_DEVICE:
				handled = 1;
				switch (pControlRequest->wValue) {
				case USB_DEVICE_REMOTE_WAKEUP:
					musb->may_wakeup = 1;
					break;
				case USB_DEVICE_TEST_MODE:
					if (musb->g.speed != USB_SPEED_HIGH)
						goto stall;
					if (pControlRequest->wIndex & 0xff)
						goto stall;

					switch (pControlRequest->wIndex >> 8) {
					case 1:
						pr_debug("TEST_J\n");
						/* TEST_J */
						musb->bTestModeValue =
							MGC_M_TEST_J;
						break;
					case 2:
						/* TEST_K */
						pr_debug("TEST_K\n");
						musb->bTestModeValue =
							MGC_M_TEST_K;
						break;
					case 3:
						/* TEST_SE0_NAK */
						pr_debug("TEST_SE0_NAK\n");
						musb->bTestModeValue =
							MGC_M_TEST_SE0_NAK;
						break;
					case 4:
						/* TEST_PACKET */
						pr_debug("TEST_PACKET\n");
						musb->bTestModeValue =
							MGC_M_TEST_PACKET;
						break;
					default:
						goto stall;
					}

					/* enter test mode after irq */
					if (handled > 0)
						musb->bTestMode = TRUE;
					break;
#ifdef CONFIG_USB_MUSB_OTG
				case USB_DEVICE_B_HNP_ENABLE:
					if (!musb->g.is_otg)
						goto stall;
					musb->g.b_hnp_enable = 1;
					musb_try_b_hnp_enable(musb);
					break;
				case USB_DEVICE_A_HNP_SUPPORT:
					if (!musb->g.is_otg)
						goto stall;
					musb->g.a_hnp_support = 1;
					break;
				case USB_DEVICE_A_ALT_HNP_SUPPORT:
					if (!musb->g.is_otg)
						goto stall;
					musb->g.a_alt_hnp_support = 1;
					break;
#endif
stall:
				default:
					handled = -EINVAL;
					break;
				}
				break;

			case USB_RECIP_INTERFACE:
				break;

			case USB_RECIP_ENDPOINT:{
				const u8		bEnd =
					pControlRequest->wIndex & 0x0f;
				struct musb_ep		*pEnd;
				struct musb_hw_ep	*ep;
				void __iomem		*regs;
				int			is_in;
				u16			csr;

				if (bEnd == 0
						|| bEnd >= MUSB_C_NUM_EPS
						|| pControlRequest->wValue
							!= USB_ENDPOINT_HALT)
					break;

				ep = musb->aLocalEnd + bEnd;
				regs = ep->regs;
				is_in = pControlRequest->wIndex & USB_DIR_IN;
				if (is_in)
					pEnd = &ep->ep_in;
				else
					pEnd = &ep->ep_out;
				if (!pEnd->desc)
					break;

				MGC_SelectEnd(pBase, bEnd);
				if (is_in) {
					csr = musb_readw(regs,
							MGC_O_HDRC_TXCSR);
					if (csr & MGC_M_TXCSR_FIFONOTEMPTY)
						csr |= MGC_M_TXCSR_FLUSHFIFO;
					csr |= MGC_M_TXCSR_P_SENDSTALL
						| MGC_M_TXCSR_CLRDATATOG
						| MGC_M_TXCSR_P_WZC_BITS;
					musb_writew(regs, MGC_O_HDRC_TXCSR,
							csr);
				} else {
					csr = musb_readw(regs,
							MGC_O_HDRC_RXCSR);
					csr |= MGC_M_RXCSR_P_SENDSTALL
						| MGC_M_RXCSR_FLUSHFIFO
						| MGC_M_RXCSR_CLRDATATOG
						| MGC_M_TXCSR_P_WZC_BITS;
					musb_writew(regs, MGC_O_HDRC_RXCSR,
							csr);
				}

				/* select ep0 again */
				MGC_SelectEnd(pBase, 0);
				handled = 1;
				} break;

			default:
				/* class, vendor, etc ... delegate */
				handled = 0;
				break;
			}
			break;
		default:
			/* delegate SET_CONFIGURATION, etc */
			handled = 0;
		}
	} else
		handled = 0;
	return handled;
}

/* we have an ep0out data packet
 * Context:  caller holds controller lock
 */
static void ep0_rxstate(struct musb *this)
{
	void __iomem		*regs = this->control_ep->regs;
	struct usb_request	*req;
	u16			tmp;

	req = next_ep0_request(this);

	/* read packet and ack; or stall because of gadget driver bug:
	 * should have provided the rx buffer before setup() returned.
	 */
	if (req) {
		void		*buf = req->buf + req->actual;
		unsigned	len = req->length - req->actual;

		/* read the buffer */
		tmp = musb_readb(regs, MGC_O_HDRC_COUNT0);
		if (tmp > len) {
			req->status = -EOVERFLOW;
			tmp = len;
		}
		musb_read_fifo(&this->aLocalEnd[0], tmp, buf);
		req->actual += tmp;
		tmp = MGC_M_CSR0_P_SVDRXPKTRDY;
		if (tmp < 64 || req->actual == req->length) {
			this->ep0_state = MGC_END0_STAGE_STATUSIN;
			tmp |= MGC_M_CSR0_P_DATAEND;
		} else
			req = NULL;
	} else
		tmp = MGC_M_CSR0_P_SVDRXPKTRDY | MGC_M_CSR0_P_SENDSTALL;
	musb_writew(regs, MGC_O_HDRC_CSR0, tmp);


	/* NOTE:  we "should" hold off reporting DATAEND and going to
	 * STATUSIN until after the completion handler decides whether
	 * to issue a stall instead, since this hardware can do that.
	 */
	if (req)
		musb_g_ep0_giveback(this, req);
}

/*
 * transmitting to the host (IN), this code might be called from IRQ
 * and from kernel thread.
 *
 * Context:  caller holds controller lock
 */
static void ep0_txstate(struct musb *musb)
{
	void __iomem		*regs = musb->control_ep->regs;
	struct usb_request	*pRequest = next_ep0_request(musb);
	u16			wCsrVal = MGC_M_CSR0_TXPKTRDY;
	u8			*pFifoSource;
	u8			wFifoCount;

	if (!pRequest) {
		// WARN_ON(1);
		DBG(2, "odd; csr0 %04x\n", musb_readw(regs, MGC_O_HDRC_CSR0));
		return;
	}

	/* load the data */
	pFifoSource = (u8 *) pRequest->buf + pRequest->actual;
	wFifoCount = min((unsigned) MGC_END0_FIFOSIZE,
		pRequest->length - pRequest->actual);
	musb_write_fifo(&musb->aLocalEnd[0], wFifoCount, pFifoSource);
	pRequest->actual += wFifoCount;

	/* update the flags */
	if (wFifoCount < MUSB_MAX_END0_PACKET
			|| pRequest->actual == pRequest->length) {
		musb->ep0_state = MGC_END0_STAGE_STATUSOUT;
		wCsrVal |= MGC_M_CSR0_P_DATAEND;
	} else
		pRequest = NULL;

	/* send it out, triggering a "txpktrdy cleared" irq */
	musb_writew(regs, MGC_O_HDRC_CSR0, wCsrVal);

	/* report completions as soon as the fifo's loaded; there's no
	 * win in waiting till this last packet gets acked.  (other than
	 * very precise fault reporting, needed by USB TMC; possible with
	 * this hardware, but not usable from portable gadget drivers.)
	 */
	if (pRequest)
		musb_g_ep0_giveback(musb, pRequest);
}

/*
 * Read a SETUP packet (struct usb_ctrlrequest) from the hardware.
 * Fields are left in USB byte-order.
 *
 * Context:  caller holds controller lock.
 */
static void
musb_read_setup(struct musb *musb, struct usb_ctrlrequest *req)
{
	struct usb_request	*r;
	void __iomem		*regs = musb->control_ep->regs;

	musb_read_fifo(&musb->aLocalEnd[0], sizeof *req, (u8 *)req);

	/* NOTE:  earlier 2.6 versions changed setup packets to host
	 * order, but now USB packets always stay in USB byte order.
	 */
	DBG(3, "SETUP req%02x.%02x v%04x i%04x l%d\n",
		req->bRequestType,
		req->bRequest,
		le16_to_cpu(req->wValue),
		le16_to_cpu(req->wIndex),
		le16_to_cpu(req->wLength));

	/* clean up any leftover transfers */
	r = next_ep0_request(musb);
	if (r)
		musb_g_ep0_giveback(musb, r);

	/* For zero-data requests we want to delay the STATUS stage to
	 * avoid SETUPEND errors.  If we read data (OUT), delay accepting
	 * packets until there's a buffer to store them in.
	 *
	 * If we write data, the controller acts happier if we enable
	 * the TX FIFO right away, and give the controller a moment
	 * to switch modes...
	 */
	musb->bSetAddress = FALSE;
	musb->ackpend = MGC_M_CSR0_P_SVDRXPKTRDY;
	if (req->wLength == 0) {
		if (req->bRequestType & USB_DIR_IN)
			musb->ackpend |= MGC_M_CSR0_TXPKTRDY;
		musb->ep0_state = MGC_END0_STAGE_ACKWAIT;
	} else if (req->bRequestType & USB_DIR_IN) {
		musb->ep0_state = MGC_END0_STAGE_TX;
		musb_writew(regs, MGC_O_HDRC_CSR0, MGC_M_CSR0_P_SVDRXPKTRDY);
		while ((musb_readw(regs, MGC_O_HDRC_CSR0)
				& MGC_M_CSR0_RXPKTRDY) != 0)
			cpu_relax();
		musb->ackpend = 0;
	} else
		musb->ep0_state = MGC_END0_STAGE_RX;
}

static int
forward_to_driver(struct musb *musb,
		const struct usb_ctrlrequest *pControlRequest)
__releases(musb->Lock)
__acquires(musb->Lock)
{
	int retval;
	if (!musb->pGadgetDriver)
		return -EOPNOTSUPP;
	spin_unlock(&musb->Lock);
	retval = musb->pGadgetDriver->setup(&musb->g, pControlRequest);
	spin_lock(&musb->Lock);
	return retval;
}

/*
 * Handle peripheral ep0 interrupt
 *
 * Context: irq handler; we won't re-enter the driver that way.
 */
irqreturn_t musb_g_ep0_irq(struct musb *musb)
{
	u16		wCsrVal;
	u16		wCount;
	void __iomem	*pBase = musb->pRegs;
	void __iomem	*regs = musb->aLocalEnd[0].regs;
	irqreturn_t	retval = IRQ_NONE;

	MGC_SelectEnd(pBase, 0);	/* select ep0 */
	wCsrVal = musb_readw(regs, MGC_O_HDRC_CSR0);
	wCount = musb_readb(regs, MGC_O_HDRC_COUNT0);

	DBG(4, "csr %04x, count %d, myaddr %d, ep0stage %s\n",
			wCsrVal, wCount,
			musb_readb(pBase, MGC_O_HDRC_FADDR),
			decode_ep0stage(musb->ep0_state));

	/* I sent a stall.. need to acknowledge it now.. */
	if (wCsrVal & MGC_M_CSR0_P_SENTSTALL) {
		musb_writew(regs, MGC_O_HDRC_CSR0,
				wCsrVal & ~MGC_M_CSR0_P_SENTSTALL);
		retval = IRQ_HANDLED;
		musb->ep0_state = MGC_END0_STAGE_SETUP;
		wCsrVal = musb_readw(regs, MGC_O_HDRC_CSR0);
	}

	/* request ended "early" */
	if (wCsrVal & MGC_M_CSR0_P_SETUPEND) {
		musb_writew(regs, MGC_O_HDRC_CSR0, MGC_M_CSR0_P_SVDSETUPEND);
		retval = IRQ_HANDLED;
		musb->ep0_state = MGC_END0_STAGE_SETUP;
		wCsrVal = musb_readw(regs, MGC_O_HDRC_CSR0);
		/* NOTE:  request may need completion */
	}

	/* docs from Mentor only describe tx, rx, and idle/setup states.
	 * we need to handle nuances around status stages, and also the
	 * case where status and setup stages come back-to-back ...
	 */
	switch (musb->ep0_state) {

	case MGC_END0_STAGE_TX:
		/* irq on clearing txpktrdy */
		if ((wCsrVal & MGC_M_CSR0_TXPKTRDY) == 0) {
			ep0_txstate(musb);
			retval = IRQ_HANDLED;
		}
		break;

	case MGC_END0_STAGE_RX:
		/* irq on set rxpktrdy */
		if (wCsrVal & MGC_M_CSR0_RXPKTRDY) {
			ep0_rxstate(musb);
			retval = IRQ_HANDLED;
		}
		break;

	case MGC_END0_STAGE_STATUSIN:
		/* end of sequence #2 (OUT/RX state) or #3 (no data) */

		/* update address (if needed) only @ the end of the
		 * status phase per usb spec, which also guarantees
		 * we get 10 msec to receive this irq... until this
		 * is done we won't see the next packet.
		 */
		if (musb->bSetAddress) {
			musb->bSetAddress = FALSE;
			musb_writeb(pBase, MGC_O_HDRC_FADDR, musb->bAddress);
		}

		/* enter test mode if needed (exit by reset) */
		else if (musb->bTestMode) {
			DBG(1, "entering TESTMODE\n");

			if (MGC_M_TEST_PACKET == musb->bTestModeValue)
				musb_load_testpacket(musb);

			musb_writeb(pBase, MGC_O_HDRC_TESTMODE,
					musb->bTestModeValue);
		}
		/* FALLTHROUGH */

	case MGC_END0_STAGE_STATUSOUT:
		/* end of sequence #1: write to host (TX state) */
		{
			struct usb_request	*req;

			req = next_ep0_request(musb);
			if (req)
				musb_g_ep0_giveback(musb, req);
		}
		retval = IRQ_HANDLED;
		musb->ep0_state = MGC_END0_STAGE_SETUP;
		/* FALLTHROUGH */

	case MGC_END0_STAGE_SETUP:
		if (wCsrVal & MGC_M_CSR0_RXPKTRDY) {
			struct usb_ctrlrequest	setup;
			int			handled = 0;

			if (wCount != 8) {
				ERR("SETUP packet len %d != 8 ?\n", wCount);
				break;
			}
			musb_read_setup(musb, &setup);
			retval = IRQ_HANDLED;

			/* sometimes the RESET won't be reported */
			if (unlikely(musb->g.speed == USB_SPEED_UNKNOWN)) {
				u8	power;

				printk(KERN_NOTICE "%s: peripheral reset "
						"irq lost!\n",
						musb_driver_name);
				power = musb_readb(pBase, MGC_O_HDRC_POWER);
				musb->g.speed = (power & MGC_M_POWER_HSMODE)
					? USB_SPEED_HIGH : USB_SPEED_FULL;

			}

			switch (musb->ep0_state) {

			/* sequence #3 (no data stage), includes requests
			 * we can't forward (notably SET_ADDRESS and the
			 * device/endpoint feature set/clear operations)
			 * plus SET_CONFIGURATION and others we must
			 */
			case MGC_END0_STAGE_ACKWAIT:
				handled = service_zero_data_request(
						musb, &setup);

				/* status stage might be immediate */
				if (handled > 0) {
					musb->ackpend |= MGC_M_CSR0_P_DATAEND;
					musb->ep0_state =
						MGC_END0_STAGE_STATUSIN;
				}
				break;

			/* sequence #1 (IN to host), includes GET_STATUS
			 * requests that we can't forward, GET_DESCRIPTOR
			 * and others that we must
			 */
			case MGC_END0_STAGE_TX:
				handled = service_in_request(musb, &setup);
				if (handled > 0) {
					musb->ackpend = MGC_M_CSR0_TXPKTRDY
						| MGC_M_CSR0_P_DATAEND;
					musb->ep0_state =
						MGC_END0_STAGE_STATUSOUT;
				}
				break;

			/* sequence #2 (OUT from host), always forward */
			default:		/* MGC_END0_STAGE_RX */
				break;
			}

			DBG(3, "handled %d, csr %04x, ep0stage %s\n",
				handled, wCsrVal,
				decode_ep0stage(musb->ep0_state));

			/* unless we need to delegate this to the gadget
			 * driver, we know how to wrap this up:  csr0 has
			 * not yet been written.
			 */
			if (handled < 0)
				goto stall;
			else if (handled > 0)
				goto finish;

			handled = forward_to_driver(musb, &setup);
			if (handled < 0) {
				MGC_SelectEnd(pBase, 0);
stall:
				DBG(3, "stall (%d)\n", handled);
				musb->ackpend |= MGC_M_CSR0_P_SENDSTALL;
				musb->ep0_state = MGC_END0_STAGE_SETUP;
finish:
				musb_writew(regs, MGC_O_HDRC_CSR0,
						musb->ackpend);
				musb->ackpend = 0;
			}
		}
		break;

	case MGC_END0_STAGE_ACKWAIT:
		/* This should not happen. But happens with tusb6010 with
		 * g_file_storage and high speed. Do nothing.
		 */
		retval = IRQ_HANDLED;
		break;

	default:
		/* "can't happen" */
		WARN_ON(1);
		musb_writew(regs, MGC_O_HDRC_CSR0, MGC_M_CSR0_P_SENDSTALL);
		musb->ep0_state = MGC_END0_STAGE_SETUP;
		break;
	}

	return retval;
}


static int
musb_g_ep0_enable(struct usb_ep *ep, const struct usb_endpoint_descriptor *desc)
{
	/* always enabled */
	return -EINVAL;
}

static int musb_g_ep0_disable(struct usb_ep *e)
{
	/* always enabled */
	return -EINVAL;
}

static void *musb_g_ep0_alloc_buffer(struct usb_ep *ep, unsigned bytes,
			dma_addr_t * dma, gfp_t gfp_flags)
{
	*dma = DMA_ADDR_INVALID;
	return kmalloc(bytes, gfp_flags);
}

static void musb_g_ep0_free_buffer(struct usb_ep *ep, void *address,
			dma_addr_t dma, unsigned bytes)
{
	kfree(address);
}

static int
musb_g_ep0_queue(struct usb_ep *e, struct usb_request *r, gfp_t gfp_flags)
{
	struct musb_ep		*ep;
	struct musb_request	*req;
	struct musb		*musb;
	int			status;
	unsigned long		lockflags;
	void __iomem		*regs;

	if (!e || !r)
		return -EINVAL;

	ep = to_musb_ep(e);
	musb = ep->pThis;
	regs = musb->control_ep->regs;

	req = to_musb_request(r);
	req->musb = musb;
	req->request.actual = 0;
	req->request.status = -EINPROGRESS;
	req->bTx = ep->is_in;

	spin_lock_irqsave(&musb->Lock, lockflags);

	if (!list_empty(&ep->req_list)) {
		status = -EBUSY;
		goto cleanup;
	}

	switch (musb->ep0_state) {
	case MGC_END0_STAGE_RX:		/* control-OUT data */
	case MGC_END0_STAGE_TX:		/* control-IN data */
	case MGC_END0_STAGE_ACKWAIT:	/* zero-length data */
		status = 0;
		break;
	default:
		DBG(1, "ep0 request queued in state %d\n",
				musb->ep0_state);
		status = -EINVAL;
		goto cleanup;
	}

	/* add request to the list */
	list_add_tail(&(req->request.list), &(ep->req_list));

	DBG(3, "queue to %s (%s), length=%d\n",
			ep->name, ep->is_in ? "IN/TX" : "OUT/RX",
			req->request.length);

	MGC_SelectEnd(musb->pRegs, 0);

	/* sequence #1, IN ... start writing the data */
	if (musb->ep0_state == MGC_END0_STAGE_TX)
		ep0_txstate(musb);

	/* sequence #3, no-data ... issue IN status */
	else if (musb->ep0_state == MGC_END0_STAGE_ACKWAIT) {
		if (req->request.length)
			status = -EINVAL;
		else {
			musb->ep0_state = MGC_END0_STAGE_STATUSIN;
			musb_writew(regs, MGC_O_HDRC_CSR0,
					musb->ackpend | MGC_M_CSR0_P_DATAEND);
			musb->ackpend = 0;
			musb_g_ep0_giveback(ep->pThis, r);
		}

	/* else for sequence #2 (OUT), caller provides a buffer
	 * before the next packet arrives.  deferred responses
	 * (after SETUP is acked) are racey.
	 */
	} else if (musb->ackpend) {
		musb_writew(regs, MGC_O_HDRC_CSR0, musb->ackpend);
		musb->ackpend = 0;
	}

cleanup:
	spin_unlock_irqrestore(&musb->Lock, lockflags);
	return status;
}

static int
musb_g_ep0_dequeue(struct usb_ep *ep, struct usb_request *req)
{
	/* we just won't support this */
	return -EINVAL;
}

static int musb_g_ep0_halt(struct usb_ep *e, int value)
{
	struct musb_ep		*ep;
	struct musb		*musb;
	void __iomem		*base, *regs;
	unsigned long		flags;
	int			status;
	u16			csr;

	if (!e || !value)
		return -EINVAL;

	ep = to_musb_ep(e);
	musb = ep->pThis;
	base = musb->pRegs;
	regs = musb->control_ep->regs;

	spin_lock_irqsave(&musb->Lock, flags);

	if (!list_empty(&ep->req_list)) {
		status = -EBUSY;
		goto cleanup;
	}

	switch (musb->ep0_state) {
	case MGC_END0_STAGE_TX:		/* control-IN data */
	case MGC_END0_STAGE_ACKWAIT:	/* STALL for zero-length data */
	case MGC_END0_STAGE_RX:		/* control-OUT data */
		status = 0;

		MGC_SelectEnd(base, 0);
		csr = musb_readw(regs, MGC_O_HDRC_CSR0);
		csr |= MGC_M_CSR0_P_SENDSTALL;
		musb_writew(regs, MGC_O_HDRC_CSR0, csr);
		musb->ep0_state = MGC_END0_STAGE_SETUP;
		break;
	default:
		DBG(1, "ep0 can't halt in state %d\n", musb->ep0_state);
		status = -EINVAL;
	}

cleanup:
	spin_unlock_irqrestore(&musb->Lock, flags);
	return status;
}

const struct usb_ep_ops musb_g_ep0_ops = {
	.enable		= musb_g_ep0_enable,
	.disable	= musb_g_ep0_disable,
	.alloc_request	= musb_alloc_request,
	.free_request	= musb_free_request,
	.alloc_buffer	= musb_g_ep0_alloc_buffer,
	.free_buffer	= musb_g_ep0_free_buffer,
	.queue		= musb_g_ep0_queue,
	.dequeue	= musb_g_ep0_dequeue,
	.set_halt	= musb_g_ep0_halt,
	.fifo_status	= NULL,
	.fifo_flush	= NULL,
};
