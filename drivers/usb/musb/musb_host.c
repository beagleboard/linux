/******************************************************************
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
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/list.h>

#include "musbdefs.h"
#include "musb_host.h"


/* MUSB HOST status 22-mar-2006
 *
 * - There's still lots of partial code duplication for fault paths, so
 *   they aren't handled as consistently as they need to be.
 *
 * - PIO mostly behaved when last tested.
 *     + including ep0, with all usbtest cases 9, 10
 *     + usbtest 14 (ep0out) doesn't seem to run at all
 *     + double buffered OUT/TX endpoints saw stalls(!) with certain usbtest
 *       configurations, but otherwise double buffering passes basic tests.
 *     + for 2.6.N, for N > ~10, needs API changes for hcd framework.
 *
 * - DMA (CPPI) ... partially behaves, not currently recommended
 *     + about 1/15 the speed of typical EHCI implementations (PCI)
 *     + RX, all too often reqpkt seems to misbehave after tx
 *     + TX, no known issues (other than evident silicon issue)
 *
 * - DMA (Mentor/OMAP) ...has at least toggle update problems
 *
 * - Still no traffic scheduling code to make NAKing for bulk or control
 *   transfers unable to starve other requests; or to make efficient use
 *   of hardware with periodic transfers.  (Note that network drivers
 *   commonly post bulk reads that stay pending for a long time; these
 *   would make very visible trouble.)
 *
 * - Not tested with HNP, but some SRP paths seem to behave.
 *
 * NOTE 24-August:
 *
 * - Bulk traffic finally uses both sides of hardware ep1, freeing up an
 *   extra endpoint for periodic use enabling hub + keybd + mouse.  That
 *   mostly works, except that with "usbnet" it's easy to trigger cases
 *   with "ping" where RX loses.  (a) ping to davinci, even "ping -f",
 *   fine; but (b) ping _from_ davinci, even "ping -c 1", ICMP RX loses
 *   although ARP RX wins.  (That test was done with a full speed link.)
 */


/*
 * NOTE on endpoint usage:
 *
 * CONTROL transfers all go through ep0.  BULK ones go through dedicated IN
 * and OUT endpoints ... hardware is dedicated for those "async" queue(s).
 *
 * (Yes, bulk _could_ use more of the endpoints than that, and would even
 * benefit from it ... one remote device may easily be NAKing while others
 * need to perform transfers in that same direction.  The same thing could
 * be done in software though, assuming dma cooperates.)
 *
 * INTERUPPT and ISOCHRONOUS transfers are scheduled to the other endpoints.
 * So far that scheduling is both dumb and optimistic:  the endpoint will be
 * "claimed" until its software queue is no longer refilled.  No multiplexing
 * of transfers between endpoints, or anything clever.
 */


/*************************** Forwards ***************************/

static void musb_ep_program(struct musb *pThis, u8 bEnd,
			struct urb *pUrb, unsigned int nOut,
			u8 * pBuffer, u32 dwLength);

/*
 * Clear TX fifo. Needed to avoid BABBLE errors.
 */
static inline void musb_h_tx_flush_fifo(struct musb_hw_ep *ep)
{
	void __iomem	*epio = ep->regs;
	u16		csr;
	int		retries = 1000;

	csr = musb_readw(epio, MGC_O_HDRC_TXCSR);
	while (csr & MGC_M_TXCSR_FIFONOTEMPTY) {
		DBG(5, "Host TX FIFONOTEMPTY csr: %02x\n", csr);
		csr |= MGC_M_TXCSR_FLUSHFIFO;
		musb_writew(epio, MGC_O_HDRC_TXCSR, csr);
		csr = musb_readw(epio, MGC_O_HDRC_TXCSR);
		if (retries-- < 1) {
			ERR("Could not flush host TX fifo: csr: %04x\n", csr);
			return;
		}
		mdelay(1);
	}
}

/*
 * Start transmit. Caller is responsible for locking shared resources.
 * pThis must be locked.
 */
static inline void musb_h_tx_start(struct musb_hw_ep *ep)
{
	u16	txcsr;

	/* NOTE: no locks here; caller should lock and select EP */
	if (ep->bLocalEnd) {
		txcsr = musb_readw(ep->regs, MGC_O_HDRC_TXCSR);
		txcsr |= MGC_M_TXCSR_TXPKTRDY | MGC_M_TXCSR_H_WZC_BITS;
		musb_writew(ep->regs, MGC_O_HDRC_TXCSR, txcsr);
	} else {
		txcsr = MGC_M_CSR0_H_SETUPPKT | MGC_M_CSR0_TXPKTRDY;
		musb_writew(ep->regs, MGC_O_HDRC_CSR0, txcsr);
	}

}

static inline void cppi_host_txdma_start(struct musb_hw_ep *ep)
{
	u16	txcsr;

	/* NOTE: no locks here; caller should lock and select EP */
	txcsr = musb_readw(ep->regs, MGC_O_HDRC_TXCSR);
	txcsr |= MGC_M_TXCSR_DMAENAB | MGC_M_TXCSR_H_WZC_BITS;
	musb_writew(ep->regs, MGC_O_HDRC_TXCSR, txcsr);
}

/*
 * Start the URB at the front of an endpoint's queue
 * end must be claimed from the caller.
 *
 * Context: controller locked, irqs blocked
 */
static void
musb_start_urb(struct musb *musb, int is_in, struct musb_qh *qh)
{
	u16			wFrame;
	u32			dwLength;
	void			*pBuffer;
	void __iomem		*pBase =  musb->pRegs;
	struct urb		*urb = next_urb(qh);
	struct musb_hw_ep	*pEnd = qh->hw_ep;
	unsigned		nPipe = urb->pipe;
	u8			bAddress = usb_pipedevice(nPipe);
	int			bEnd = pEnd->bLocalEnd;

	/* initialize software qh state */
	qh->offset = 0;
	qh->segsize = 0;

	/* gather right source of data */
	switch (qh->type) {
	case USB_ENDPOINT_XFER_CONTROL:
		/* control transfers always start with SETUP */
		is_in = 0;
		pEnd->out_qh = qh;
		musb->bEnd0Stage = MGC_END0_START;
		pBuffer = urb->setup_packet;
		dwLength = 8;
		break;
	case USB_ENDPOINT_XFER_ISOC:
		qh->iso_idx = 0;
		qh->frame = 0;
		pBuffer = urb->transfer_buffer + urb->iso_frame_desc[0].offset;
		dwLength = urb->iso_frame_desc[0].length;
		break;
	default:		/* bulk, interrupt */
		pBuffer = urb->transfer_buffer;
		dwLength = urb->transfer_buffer_length;
	}

	DBG(4, "qh %p urb %p dev%d ep%d%s%s, hw_ep %d, %p/%d\n",
			qh, urb, bAddress, qh->epnum,
			is_in ? "in" : "out",
			({char *s; switch (qh->type) {
			case USB_ENDPOINT_XFER_CONTROL:	s = ""; break;
			case USB_ENDPOINT_XFER_BULK:	s = "-bulk"; break;
			case USB_ENDPOINT_XFER_ISOC:	s = "-iso"; break;
			default:			s = "-intr"; break;
			}; s;}),
			bEnd, pBuffer, dwLength);

	/* Configure endpoint */
	if (is_in || pEnd->bIsSharedFifo)
		pEnd->in_qh = qh;
	else
		pEnd->out_qh = qh;
	musb_ep_program(musb, bEnd, urb, !is_in, pBuffer, dwLength);

	/* transmit may have more work: start it when it is time */
	if (is_in)
		return;

	/* determine if the time is right for a periodic transfer */
	switch (qh->type) {
	case USB_ENDPOINT_XFER_ISOC:
	case USB_ENDPOINT_XFER_INT:
		DBG(3, "check whether there's still time for periodic Tx\n");
		qh->iso_idx = 0;
		wFrame = musb_readw(pBase, MGC_O_HDRC_FRAME);
		/* FIXME this doesn't implement that scheduling policy ...
		 * or handle framecounter wrapping
		 */
		if ((urb->transfer_flags & URB_ISO_ASAP)
				|| (wFrame >= urb->start_frame)) {
			/* REVISIT the SOF irq handler shouldn't duplicate
			 * this code; and we don't init urb->start_frame...
			 */
			qh->frame = 0;
			goto start;
		} else {
			qh->frame = urb->start_frame;
			/* enable SOF interrupt so we can count down */
DBG(1,"SOF for %d\n", bEnd);
#if 1 // ifndef	CONFIG_ARCH_DAVINCI
			musb_writeb(pBase, MGC_O_HDRC_INTRUSBE, 0xff);
#endif
		}
		break;
	default:
start:
		DBG(4, "Start TX%d %s\n", bEnd,
			pEnd->tx_channel ? "dma" : "pio");

		if (!pEnd->tx_channel)
			musb_h_tx_start(pEnd);
		else if (is_cppi_enabled() || tusb_dma_omap())
			cppi_host_txdma_start(pEnd);
	}
}

/* caller owns controller lock, irqs are blocked */
static void
__musb_giveback(struct musb *musb, struct urb *urb, int status)
__releases(musb->Lock)
__acquires(musb->Lock)
{
	if ((urb->transfer_flags & URB_SHORT_NOT_OK)
			&& (urb->actual_length < urb->transfer_buffer_length)
			&& status == 0
			&& usb_pipein(urb->pipe))
		status = -EREMOTEIO;

	spin_lock(&urb->lock);
	urb->hcpriv = NULL;
	if (urb->status == -EINPROGRESS)
		urb->status = status;
	spin_unlock(&urb->lock);

	DBG(({ int level; switch (urb->status) {
				case 0:
					level = 4;
					break;
				/* common/boring faults */
				case -EREMOTEIO:
				case -ESHUTDOWN:
				case -ECONNRESET:
				case -EPIPE:
					level = 3;
					break;
				default:
					level = 2;
					break;
				}; level; }),
			"complete %p (%d), dev%d ep%d%s, %d/%d\n",
			urb, urb->status,
			usb_pipedevice(urb->pipe),
			usb_pipeendpoint(urb->pipe),
			usb_pipein(urb->pipe) ? "in" : "out",
			urb->actual_length, urb->transfer_buffer_length
			);

	spin_unlock(&musb->Lock);
	usb_hcd_giveback_urb(musb_to_hcd(musb), urb);
	spin_lock(&musb->Lock);
}

/* for bulk/interrupt endpoints only */
static inline void musb_save_toggle(struct musb_hw_ep *ep, int is_in, struct urb *urb)
{
	struct usb_device	*udev = urb->dev;
	u16			csr;
	void __iomem		*epio = ep->regs;
	struct musb_qh		*qh;

	/* FIXME:  the current Mentor DMA code seems to have
	 * problems getting toggle correct.
	 */

	if (is_in || ep->bIsSharedFifo)
		qh = ep->in_qh;
	else
		qh = ep->out_qh;

	if (!is_in) {
		csr = musb_readw(epio, MGC_O_HDRC_TXCSR);
		usb_settoggle(udev, qh->epnum, 1,
			(csr & MGC_M_TXCSR_H_DATATOGGLE)
				? 1 : 0);
	} else {
		csr = musb_readw(epio, MGC_O_HDRC_RXCSR);
		usb_settoggle(udev, qh->epnum, 0,
			(csr & MGC_M_RXCSR_H_DATATOGGLE)
				? 1 : 0);
	}
}

/* caller owns controller lock, irqs are blocked */
static struct musb_qh *
musb_giveback(struct musb_qh *qh, struct urb *urb, int status)
{
	int			is_in;
	struct musb_hw_ep	*ep = qh->hw_ep;
	struct musb		*musb = ep->musb;
	int			ready = qh->is_ready;

	if (ep->bIsSharedFifo)
		is_in = 1;
	else
		is_in = usb_pipein(urb->pipe);

	/* save toggle eagerly, for paranoia */
	switch (qh->type) {
	case USB_ENDPOINT_XFER_BULK:
	case USB_ENDPOINT_XFER_INT:
		musb_save_toggle(ep, is_in, urb);
		break;
	case USB_ENDPOINT_XFER_ISOC:
		if (status == 0 && urb->error_count)
			status = -EXDEV;
		break;
	}

	qh->is_ready = 0;
	__musb_giveback(musb, urb, status);
	qh->is_ready = ready;

	/* reclaim resources (and bandwidth) ASAP; deschedule it, and
	 * invalidate qh as soon as list_empty(&hep->urb_list)
	 */
	if (list_empty(&qh->hep->urb_list)) {
		struct list_head	*head;

		if (is_in)
			ep->rx_reinit = 1;
		else
			ep->tx_reinit = 1;

		/* clobber old pointers to this qh */
		if (is_in || ep->bIsSharedFifo)
			ep->in_qh = NULL;
		else
			ep->out_qh = NULL;
		qh->hep->hcpriv = NULL;

		switch (qh->type) {

		case USB_ENDPOINT_XFER_ISOC:
		case USB_ENDPOINT_XFER_INT:
			/* this is where periodic bandwidth should be
			 * de-allocated if it's tracked and allocated;
			 * and where we'd update the schedule tree...
			 */
			musb->periodic[ep->bLocalEnd] = NULL;
			kfree(qh);
			qh = NULL;
			break;

		case USB_ENDPOINT_XFER_CONTROL:
		case USB_ENDPOINT_XFER_BULK:
			/* fifo policy for these lists, except that NAKing
			 * should rotate a qh to the end (for fairness).
			 */
			head = qh->ring.prev;
			list_del(&qh->ring);
			kfree(qh);
			qh = first_qh(head);
			break;
		}
	}
	return qh;
}

/*
 * Advance this hardware endpoint's queue, completing the specified urb and
 * advancing to either the next urb queued to that qh, or else invalidating
 * that qh and advancing to the next qh scheduled after the current one.
 *
 * Context: caller owns controller lock, irqs are blocked
 */
static void
musb_advance_schedule(struct musb *pThis, struct urb *urb,
		struct musb_hw_ep *pEnd, int is_in)
{
	struct musb_qh	*qh;

	if (is_in || pEnd->bIsSharedFifo)
		qh = pEnd->in_qh;
	else
		qh = pEnd->out_qh;
	qh = musb_giveback(qh, urb, 0);

	if (qh && qh->is_ready && !list_empty(&qh->hep->urb_list)) {
		DBG(4, "... next ep%d %cX urb %p\n",
				pEnd->bLocalEnd, is_in ? 'R' : 'T',
				next_urb(qh));
		musb_start_urb(pThis, is_in, qh);
	}
}

static inline u16 musb_h_flush_rxfifo(struct musb_hw_ep *hw_ep, u16 csr)
{
	/* we don't want fifo to fill itself again;
	 * ignore dma (various models),
	 * leave toggle alone (may not have been saved yet)
	 */
	csr |= MGC_M_RXCSR_FLUSHFIFO | MGC_M_RXCSR_RXPKTRDY;
	csr &= ~( MGC_M_RXCSR_H_REQPKT
		| MGC_M_RXCSR_H_AUTOREQ
		| MGC_M_RXCSR_AUTOCLEAR
		);

	/* write 2x to allow double buffering */
	musb_writew(hw_ep->regs, MGC_O_HDRC_RXCSR, csr);
	musb_writew(hw_ep->regs, MGC_O_HDRC_RXCSR, csr);

	/* flush writebuffer */
	return musb_readw(hw_ep->regs, MGC_O_HDRC_RXCSR);
}

/*
 * PIO RX for a packet (or part of it).
 */
static u8 musb_host_packet_rx(struct musb *pThis, struct urb *pUrb,
		u8 bEnd, u8 bIsochError)
{
	u16 wRxCount;
	u8 *pBuffer;
	u16 wCsr;
	u8 bDone = FALSE;
	u32			length;
	int			do_flush = 0;
	struct musb_hw_ep	*pEnd = pThis->aLocalEnd + bEnd;
	void __iomem		*epio = pEnd->regs;
	struct musb_qh		*qh = pEnd->in_qh;
	int			nPipe = pUrb->pipe;
	void			*buffer = pUrb->transfer_buffer;

	// MGC_SelectEnd(pBase, bEnd);
	wRxCount = musb_readw(epio, MGC_O_HDRC_RXCOUNT);
	DBG(3, "RX%d count %d, buffer %p len %d/%d\n", bEnd, wRxCount,
			pUrb->transfer_buffer, qh->offset,
			pUrb->transfer_buffer_length);

	/* unload FIFO */
	if (usb_pipeisoc(nPipe)) {
		int					status = 0;
		struct usb_iso_packet_descriptor	*d;

		if (bIsochError) {
			status = -EILSEQ;
			pUrb->error_count++;
		}

		d = pUrb->iso_frame_desc + qh->iso_idx;
		pBuffer = buffer + d->offset;
		length = d->length;
		if (wRxCount > length) {
			if (status == 0) {
				status = -EOVERFLOW;
				pUrb->error_count++;
			}
			DBG(2, "** OVERFLOW %d into %d\n", wRxCount, length);
			do_flush = 1;
		} else
			length = wRxCount;
		pUrb->actual_length += length;
		d->actual_length = length;

		d->status = status;

		/* see if we are done */
		bDone = (++qh->iso_idx >= pUrb->number_of_packets);
	} else {
		/* non-isoch */
		pBuffer = buffer + qh->offset;
		length = pUrb->transfer_buffer_length - qh->offset;
		if (wRxCount > length) {
			if (pUrb->status == -EINPROGRESS)
				pUrb->status = -EOVERFLOW;
			DBG(2, "** OVERFLOW %d into %d\n", wRxCount, length);
			do_flush = 1;
		} else
			length = wRxCount;
		pUrb->actual_length += length;
		qh->offset += length;

		/* see if we are done */
		bDone = (pUrb->actual_length == pUrb->transfer_buffer_length)
			|| (wRxCount < qh->maxpacket)
			|| (pUrb->status != -EINPROGRESS);
		if (bDone
				&& (pUrb->status == -EINPROGRESS)
				&& (pUrb->transfer_flags & URB_SHORT_NOT_OK)
				&& (pUrb->actual_length
					< pUrb->transfer_buffer_length))
			pUrb->status = -EREMOTEIO;
	}

	musb_read_fifo(pEnd, length, pBuffer);

	wCsr = musb_readw(epio, MGC_O_HDRC_RXCSR);
	wCsr |= MGC_M_RXCSR_H_WZC_BITS;
	if (unlikely(do_flush))
		musb_h_flush_rxfifo(pEnd, wCsr);
	else {
		/* REVISIT this assumes AUTOCLEAR is never set */
		wCsr &= ~(MGC_M_RXCSR_RXPKTRDY | MGC_M_RXCSR_H_REQPKT);
		if (!bDone)
			wCsr |= MGC_M_RXCSR_H_REQPKT;
		musb_writew(epio, MGC_O_HDRC_RXCSR, wCsr);
	}

	return bDone;
}

/* we don't always need to reinit a given side of an endpoint...
 * when we do, use tx/rx reinit routine and then construct a new CSR
 * to address data toggle, NYET, and DMA or PIO.
 *
 * it's possible that driver bugs (especially for DMA) or aborting a
 * transfer might have left the endpoint busier than it should be.
 * the busy/not-empty tests are basically paranoia.
 */
static void
musb_rx_reinit(struct musb *musb, struct musb_qh *qh, struct musb_hw_ep *ep)
{
	u16	csr;

	/* NOTE:  we know the "rx" fifo reinit never triggers for ep0.
	 * That always uses tx_reinit since ep0 repurposes TX register
	 * offsets; the initial SETUP packet is also a kind of OUT.
	 */

	/* if programmed for Tx, put it in RX mode */
	if (ep->bIsSharedFifo) {
		csr = musb_readw(ep->regs, MGC_O_HDRC_TXCSR);
		if (csr & MGC_M_TXCSR_MODE) {
			musb_h_tx_flush_fifo(ep);
			musb_writew(ep->regs, MGC_O_HDRC_TXCSR,
					MGC_M_TXCSR_FRCDATATOG);
		}
		/* clear mode (and everything else) to enable Rx */
		musb_writew(ep->regs, MGC_O_HDRC_TXCSR, 0);

	/* scrub all previous state, clearing toggle */
	} else {
		csr = musb_readw(ep->regs, MGC_O_HDRC_RXCSR);
		if (csr & MGC_M_RXCSR_RXPKTRDY)
			WARN("rx%d, packet/%d ready?\n", ep->bLocalEnd,
				musb_readw(ep->regs, MGC_O_HDRC_RXCOUNT));

		musb_h_flush_rxfifo(ep, MGC_M_RXCSR_CLRDATATOG);
	}

	/* target addr and (for multipoint) hub addr/port */
	if (musb->bIsMultipoint) {
		musb_writeb(ep->target_regs, MGC_O_HDRC_RXFUNCADDR,
			qh->addr_reg);
		musb_writeb(ep->target_regs, MGC_O_HDRC_RXHUBADDR,
			qh->h_addr_reg);
		musb_writeb(ep->target_regs, MGC_O_HDRC_RXHUBPORT,
			qh->h_port_reg);
	} else
		musb_writeb(musb->pRegs, MGC_O_HDRC_FADDR, qh->addr_reg);

	/* protocol/endpoint, interval/NAKlimit, i/o size */
	musb_writeb(ep->regs, MGC_O_HDRC_RXTYPE, qh->type_reg);
	musb_writeb(ep->regs, MGC_O_HDRC_RXINTERVAL, qh->intv_reg);
	/* NOTE: bulk combining rewrites high bits of maxpacket */
	musb_writew(ep->regs, MGC_O_HDRC_RXMAXP, qh->maxpacket);

	ep->rx_reinit = 0;
}


/*
 * Program an HDRC endpoint as per the given URB
 * Context: irqs blocked, controller lock held
 */
static void musb_ep_program(struct musb *pThis, u8 bEnd,
			struct urb *pUrb, unsigned int is_out,
			u8 * pBuffer, u32 dwLength)
{
	struct dma_controller	*pDmaController;
	struct dma_channel	*pDmaChannel;
	u8			bDmaOk;
	void __iomem		*pBase = pThis->pRegs;
	struct musb_hw_ep	*pEnd = pThis->aLocalEnd + bEnd;
	void __iomem		*epio = pEnd->regs;
	struct musb_qh		*qh;
	u16			wPacketSize;

	if (!is_out || pEnd->bIsSharedFifo)
		qh = pEnd->in_qh;
	else
		qh = pEnd->out_qh;

	wPacketSize = qh->maxpacket;

	DBG(3, "%s hw%d urb %p spd%d dev%d ep%d%s "
				"h_addr%02x h_port%02x bytes %d\n",
			is_out ? "-->" : "<--",
			bEnd, pUrb, pUrb->dev->speed,
			qh->addr_reg, qh->epnum, is_out ? "out" : "in",
			qh->h_addr_reg, qh->h_port_reg,
			dwLength);

	MGC_SelectEnd(pBase, bEnd);

	/* candidate for DMA? */
	pDmaController = pThis->pDmaController;
	if (is_dma_capable() && bEnd && pDmaController) {
		pDmaChannel = is_out ? pEnd->tx_channel : pEnd->rx_channel;
		if (!pDmaChannel) {
			pDmaChannel = pDmaController->channel_alloc(
					pDmaController, pEnd, is_out);
			if (is_out)
				pEnd->tx_channel = pDmaChannel;
			else
				pEnd->rx_channel = pDmaChannel;
		}
	} else
		pDmaChannel = NULL;

	/* make sure we clear DMAEnab, autoSet bits from previous run */

	/* OUT/transmit/EP0 or IN/receive? */
	if (is_out) {
		u16	wCsr;
		u16	wIntrTxE;
		u16	wLoadCount;

		wCsr = musb_readw(epio, MGC_O_HDRC_TXCSR);

		/* disable interrupt in case we flush */
		wIntrTxE = musb_readw(pBase, MGC_O_HDRC_INTRTXE);
		musb_writew(pBase, MGC_O_HDRC_INTRTXE, wIntrTxE & ~(1 << bEnd));

		/* general endpoint setup */
		if (bEnd) {
			u16	csr = wCsr;

			/* ASSERT:  TXCSR_DMAENAB was already cleared */

			/* flush all old state, set default */
			musb_h_tx_flush_fifo(pEnd);
			csr &= ~(MGC_M_TXCSR_H_NAKTIMEOUT
					| MGC_M_TXCSR_DMAMODE
					| MGC_M_TXCSR_FRCDATATOG
					| MGC_M_TXCSR_H_RXSTALL
					| MGC_M_TXCSR_H_ERROR
					| MGC_M_TXCSR_TXPKTRDY
					);
			csr |= MGC_M_TXCSR_MODE;

			if (usb_gettoggle(pUrb->dev,
					qh->epnum, 1))
				csr |= MGC_M_TXCSR_H_WR_DATATOGGLE
					| MGC_M_TXCSR_H_DATATOGGLE;
			else
				csr |= MGC_M_TXCSR_CLRDATATOG;

			/* twice in case of double packet buffering */
			musb_writew(epio, MGC_O_HDRC_TXCSR, csr);
			/* REVISIT may need to clear FLUSHFIFO ... */
			musb_writew(epio, MGC_O_HDRC_TXCSR, csr);
			wCsr = musb_readw(epio, MGC_O_HDRC_TXCSR);
		} else {
			/* endpoint 0: just flush */
			musb_writew(epio, MGC_O_HDRC_CSR0,
				wCsr | MGC_M_CSR0_FLUSHFIFO);
			musb_writew(epio, MGC_O_HDRC_CSR0,
				wCsr | MGC_M_CSR0_FLUSHFIFO);
		}

		/* target addr and (for multipoint) hub addr/port */
		if (pThis->bIsMultipoint) {
			musb_writeb(pBase,
				MGC_BUSCTL_OFFSET(bEnd, MGC_O_HDRC_TXFUNCADDR),
				qh->addr_reg);
			musb_writeb(pBase,
				MGC_BUSCTL_OFFSET(bEnd, MGC_O_HDRC_TXHUBADDR),
				qh->h_addr_reg);
			musb_writeb(pBase,
				MGC_BUSCTL_OFFSET(bEnd, MGC_O_HDRC_TXHUBPORT),
				qh->h_port_reg);
/* FIXME if !bEnd, do the same for RX ... */
		} else
			musb_writeb(pBase, MGC_O_HDRC_FADDR, qh->addr_reg);

		/* protocol/endpoint/interval/NAKlimit */
		if (bEnd) {
			musb_writeb(epio, MGC_O_HDRC_TXTYPE, qh->type_reg);
			if (can_bulk_split(pThis, qh->type))
				musb_writew(epio, MGC_O_HDRC_TXMAXP,
					wPacketSize
					| ((pEnd->wMaxPacketSizeTx /
						wPacketSize) - 1) << 11);
			else
				musb_writew(epio, MGC_O_HDRC_TXMAXP,
					wPacketSize);
			musb_writeb(epio, MGC_O_HDRC_TXINTERVAL, qh->intv_reg);
		} else {
			musb_writeb(epio, MGC_O_HDRC_NAKLIMIT0, qh->intv_reg);
			if (pThis->bIsMultipoint)
				musb_writeb(epio, MGC_O_HDRC_TYPE0,
						qh->type_reg);
		}

		if (can_bulk_split(pThis, qh->type))
			wLoadCount = min((u32) pEnd->wMaxPacketSizeTx,
						dwLength);
		else
			wLoadCount = min((u32) wPacketSize, dwLength);

#ifdef CONFIG_USB_INVENTRA_DMA
		if (pDmaChannel) {

			/* clear previous state */
			wCsr = musb_readw(epio, MGC_O_HDRC_TXCSR);
			wCsr &= ~(MGC_M_TXCSR_AUTOSET
				| MGC_M_TXCSR_DMAMODE
				| MGC_M_TXCSR_DMAENAB);
                        wCsr |= MGC_M_TXCSR_MODE;
			musb_writew(epio, MGC_O_HDRC_TXCSR,
				wCsr | MGC_M_TXCSR_MODE);

			qh->segsize = min(dwLength, pDmaChannel->dwMaxLength);

			if (qh->segsize <= wPacketSize)
				pDmaChannel->bDesiredMode = 0;
			else
				pDmaChannel->bDesiredMode = 1;


			if (pDmaChannel->bDesiredMode == 0) {
				wCsr &= ~(MGC_M_TXCSR_AUTOSET
					| MGC_M_TXCSR_DMAMODE);
				wCsr |= (MGC_M_TXCSR_DMAENAB);
					// against programming guide
			} else
				wCsr |= (MGC_M_TXCSR_AUTOSET
					| MGC_M_TXCSR_DMAENAB
					| MGC_M_TXCSR_DMAMODE);

			musb_writew(epio, MGC_O_HDRC_TXCSR, wCsr);

			bDmaOk = pDmaController->channel_program(
					pDmaChannel, wPacketSize,
					pDmaChannel->bDesiredMode,
					pUrb->transfer_dma,
					qh->segsize);
			if (bDmaOk) {
				wLoadCount = 0;
			} else {
				pDmaController->channel_release(pDmaChannel);
				if (is_out)
					pEnd->tx_channel = NULL;
				else
					pEnd->rx_channel = NULL;
				pDmaChannel = NULL;
			}
		}
#endif

		/* candidate for DMA */
		if ((is_cppi_enabled() || tusb_dma_omap()) && pDmaChannel) {

			/* program endpoint CSRs first, then setup DMA.
			 * assume CPPI setup succeeds.
			 * defer enabling dma.
			 */
			wCsr = musb_readw(epio, MGC_O_HDRC_TXCSR);
			wCsr &= ~(MGC_M_TXCSR_AUTOSET
					| MGC_M_TXCSR_DMAMODE
					| MGC_M_TXCSR_DMAENAB);
			wCsr |= MGC_M_TXCSR_MODE;
			musb_writew(epio, MGC_O_HDRC_TXCSR,
				wCsr | MGC_M_TXCSR_MODE);

			pDmaChannel->dwActualLength = 0L;
			qh->segsize = dwLength;

			/* TX uses "rndis" mode automatically, but needs help
			 * to identify the zero-length-final-packet case.
			 */
			bDmaOk = pDmaController->channel_program(
					pDmaChannel, wPacketSize,
					(pUrb->transfer_flags
							& URB_ZERO_PACKET)
						== URB_ZERO_PACKET,
					pUrb->transfer_dma,
					qh->segsize);
			if (bDmaOk) {
				wLoadCount = 0;
			} else {
				pDmaController->channel_release(pDmaChannel);
				pDmaChannel = pEnd->tx_channel = NULL;

				/* REVISIT there's an error path here that
				 * needs handling:  can't do dma, but
				 * there's no pio buffer address...
				 */
			}
		}

		if (wLoadCount) {
			/* ASSERT:  TXCSR_DMAENAB was already cleared */

			/* PIO to load FIFO */
			qh->segsize = wLoadCount;
			musb_write_fifo(pEnd, wLoadCount, pBuffer);
			wCsr = musb_readw(epio, MGC_O_HDRC_TXCSR);
			wCsr &= ~(MGC_M_TXCSR_DMAENAB
				| MGC_M_TXCSR_DMAMODE
				| MGC_M_TXCSR_AUTOSET);
			/* write CSR */
			wCsr |= MGC_M_TXCSR_MODE;

			if (bEnd)
				musb_writew(epio, MGC_O_HDRC_TXCSR, wCsr);
		}

		/* re-enable interrupt */
		musb_writew(pBase, MGC_O_HDRC_INTRTXE, wIntrTxE);

	/* IN/receive */
	} else {
		u16	csr;

		if (pEnd->rx_reinit) {
			musb_rx_reinit(pThis, qh, pEnd);

			/* init new state: toggle and NYET, maybe DMA later */
			if (usb_gettoggle(pUrb->dev, qh->epnum, 0))
				csr = MGC_M_RXCSR_H_WR_DATATOGGLE
					| MGC_M_RXCSR_H_DATATOGGLE;
			else
				csr = 0;
			if (qh->type == USB_ENDPOINT_XFER_INT)
				csr |= MGC_M_RXCSR_DISNYET;

		} else {
			csr = musb_readw(pEnd->regs, MGC_O_HDRC_RXCSR);

			if (csr & (MGC_M_RXCSR_RXPKTRDY
					| MGC_M_RXCSR_DMAENAB
					| MGC_M_RXCSR_H_REQPKT))
				ERR("broken !rx_reinit, ep%d csr %04x\n",
						pEnd->bLocalEnd, csr);

			/* scrub any stale state, leaving toggle alone */
			csr &= MGC_M_RXCSR_DISNYET;
		}

		/* kick things off */

		if ((is_cppi_enabled() || tusb_dma_omap()) && pDmaChannel) {
			/* candidate for DMA */
			if (pDmaChannel) {
				pDmaChannel->dwActualLength = 0L;
				qh->segsize = dwLength;

				/* AUTOREQ is in a DMA register */
				musb_writew(pEnd->regs, MGC_O_HDRC_RXCSR, csr);
				csr = musb_readw(pEnd->regs,
						MGC_O_HDRC_RXCSR);

				/* unless caller treats short rx transfers as
				 * errors, we dare not queue multiple transfers.
				 */
				bDmaOk = pDmaController->channel_program(
						pDmaChannel, wPacketSize,
						!(pUrb->transfer_flags
							& URB_SHORT_NOT_OK),
						pUrb->transfer_dma,
						qh->segsize);
				if (!bDmaOk) {
					pDmaController->channel_release(
							pDmaChannel);
					pDmaChannel = pEnd->rx_channel = NULL;
				} else
					csr |= MGC_M_RXCSR_DMAENAB;
			}
		}

		csr |= MGC_M_RXCSR_H_REQPKT;
		DBG(7, "RXCSR%d := %04x\n", bEnd, csr);
		musb_writew(pEnd->regs, MGC_O_HDRC_RXCSR, csr);
		csr = musb_readw(pEnd->regs, MGC_O_HDRC_RXCSR);
	}
}


/*
 * Service the default endpoint (ep0) as host.
 * Return TRUE until it's time to start the status stage.
 */
static int musb_h_ep0_continue(struct musb *pThis,
				u16 wCount, struct urb *pUrb)
{
	int			 bMore = FALSE;
	u8 *pFifoDest = NULL;
	u16 wFifoCount = 0;
	struct musb_hw_ep	*pEnd = pThis->control_ep;
	struct musb_qh		*qh = pEnd->in_qh;
	struct usb_ctrlrequest	*pRequest;

	switch (pThis->bEnd0Stage) {
	case MGC_END0_IN:
		pFifoDest = pUrb->transfer_buffer + pUrb->actual_length;
		wFifoCount = min(wCount, ((u16) (pUrb->transfer_buffer_length
					- pUrb->actual_length)));
		if (wFifoCount < wCount)
			pUrb->status = -EOVERFLOW;

		musb_read_fifo(pEnd, wFifoCount, pFifoDest);

		pUrb->actual_length += wFifoCount;
		if (wCount < qh->maxpacket) {
			/* always terminate on short read; it's
			 * rarely reported as an error.
			 */
		} else if (pUrb->actual_length <
				pUrb->transfer_buffer_length)
			bMore = TRUE;
		break;
	case MGC_END0_START:
		pRequest = (struct usb_ctrlrequest *) pUrb->setup_packet;

		if (!pRequest->wLength) {
			DBG(4, "start no-DATA\n");
			break;
		} else if (pRequest->bRequestType & USB_DIR_IN) {
			DBG(4, "start IN-DATA\n");
			pThis->bEnd0Stage = MGC_END0_IN;
			bMore = TRUE;
			break;
		} else {
			DBG(4, "start OUT-DATA\n");
			pThis->bEnd0Stage = MGC_END0_OUT;
			bMore = TRUE;
		}
		/* FALLTHROUGH */
	case MGC_END0_OUT:
		wFifoCount = min(qh->maxpacket, ((u16)
				(pUrb->transfer_buffer_length
				- pUrb->actual_length)));

		if (wFifoCount) {
			pFifoDest = (u8 *) (pUrb->transfer_buffer
					+ pUrb->actual_length);
			DBG(3, "Sending %d bytes to %p\n",
					wFifoCount, pFifoDest);
			musb_write_fifo(pEnd, wFifoCount, pFifoDest);

			pUrb->actual_length += wFifoCount;
			bMore = TRUE;
		}
		break;
	default:
		ERR("bogus ep0 stage %d\n", pThis->bEnd0Stage);
		break;
	}

	return bMore;
}

/*
 * Handle default endpoint interrupt as host. Only called in IRQ time
 * from the LinuxIsr() interrupt service routine.
 *
 * called with controller irqlocked
 */
irqreturn_t musb_h_ep0_irq(struct musb *pThis)
{
	struct urb		*pUrb;
	u16			wCsrVal, wCount;
	int			status = 0;
	void __iomem		*pBase = pThis->pRegs;
	struct musb_hw_ep	*pEnd = pThis->control_ep;
	void __iomem		*epio = pEnd->regs;
	struct musb_qh		*qh = pEnd->in_qh;
	u8			bComplete = FALSE;
	irqreturn_t		retval = IRQ_NONE;

	/* ep0 only has one queue, "in" */
	pUrb = next_urb(qh);

	MGC_SelectEnd(pBase, 0);
	wCsrVal = musb_readw(epio, MGC_O_HDRC_CSR0);
	wCount = (wCsrVal & MGC_M_CSR0_RXPKTRDY)
			? musb_readb(epio, MGC_O_HDRC_COUNT0)
			: 0;

	DBG(4, "<== csr0 %04x, qh %p, count %d, urb %p, stage %d\n",
		wCsrVal, qh, wCount, pUrb, pThis->bEnd0Stage);

	/* if we just did status stage, we are done */
	if (MGC_END0_STATUS == pThis->bEnd0Stage) {
		retval = IRQ_HANDLED;
		bComplete = TRUE;
	}

	/* prepare status */
	if (wCsrVal & MGC_M_CSR0_H_RXSTALL) {
		DBG(6, "STALLING ENDPOINT\n");
		status = -EPIPE;

	} else if (wCsrVal & MGC_M_CSR0_H_ERROR) {
		DBG(2, "no response, csr0 %04x\n", wCsrVal);
		status = -EPROTO;

	} else if (wCsrVal & MGC_M_CSR0_H_NAKTIMEOUT) {
		DBG(2, "control NAK timeout\n");

		/* NOTE:  this code path would be a good place to PAUSE a
		 * control transfer, if another one is queued, so that
		 * ep0 is more likely to stay busy.
		 *
		 * if (qh->ring.next != &musb->control), then
		 * we have a candidate... NAKing is *NOT* an error
		 */
		musb_writew(epio, MGC_O_HDRC_CSR0, 0);
		retval = IRQ_HANDLED;
	}

	if (status) {
		DBG(6, "aborting\n");
		retval = IRQ_HANDLED;
		if (pUrb)
			pUrb->status = status;
		bComplete = TRUE;

		/* use the proper sequence to abort the transfer */
		if (wCsrVal & MGC_M_CSR0_H_REQPKT) {
			wCsrVal &= ~MGC_M_CSR0_H_REQPKT;
			musb_writew(epio, MGC_O_HDRC_CSR0, wCsrVal);
			wCsrVal &= ~MGC_M_CSR0_H_NAKTIMEOUT;
			musb_writew(epio, MGC_O_HDRC_CSR0, wCsrVal);
		} else {
			wCsrVal |= MGC_M_CSR0_FLUSHFIFO;
			musb_writew(epio, MGC_O_HDRC_CSR0, wCsrVal);
			musb_writew(epio, MGC_O_HDRC_CSR0, wCsrVal);
			wCsrVal &= ~MGC_M_CSR0_H_NAKTIMEOUT;
			musb_writew(epio, MGC_O_HDRC_CSR0, wCsrVal);
		}

		musb_writeb(epio, MGC_O_HDRC_NAKLIMIT0, 0);

		/* clear it */
		musb_writew(epio, MGC_O_HDRC_CSR0, 0);
	}

	if (unlikely(!pUrb)) {
		/* stop endpoint since we have no place for its data, this
		 * SHOULD NEVER HAPPEN! */
		ERR("no URB for end 0\n");

		musb_writew(epio, MGC_O_HDRC_CSR0, MGC_M_CSR0_FLUSHFIFO);
		musb_writew(epio, MGC_O_HDRC_CSR0, MGC_M_CSR0_FLUSHFIFO);
		musb_writew(epio, MGC_O_HDRC_CSR0, 0);

		goto done;
	}

	if (!bComplete) {
		/* call common logic and prepare response */
		if (musb_h_ep0_continue(pThis, wCount, pUrb)) {
			/* more packets required */
			wCsrVal = (MGC_END0_IN == pThis->bEnd0Stage)
				?  MGC_M_CSR0_H_REQPKT : MGC_M_CSR0_TXPKTRDY;
		} else {
			/* data transfer complete; perform status phase */
			if (usb_pipeout(pUrb->pipe)
					|| !pUrb->transfer_buffer_length)
				wCsrVal = MGC_M_CSR0_H_STATUSPKT
					| MGC_M_CSR0_H_REQPKT;
			else
				wCsrVal = MGC_M_CSR0_H_STATUSPKT
					| MGC_M_CSR0_TXPKTRDY;

			/* flag status stage */
			pThis->bEnd0Stage = MGC_END0_STATUS;

			DBG(5, "ep0 STATUS, csr %04x\n", wCsrVal);

		}
		musb_writew(epio, MGC_O_HDRC_CSR0, wCsrVal);
		retval = IRQ_HANDLED;
	} else
		pThis->bEnd0Stage = MGC_END0_IDLE;

	/* call completion handler if done */
	if (bComplete)
		musb_advance_schedule(pThis, pUrb, pEnd, 1);
done:
	return retval;
}


#ifdef CONFIG_USB_INVENTRA_DMA

/* Host side TX (OUT) using Mentor DMA works as follows:
	submit_urb ->
		- if queue was empty, Program Endpoint
		- ... which starts DMA to fifo in mode 1 or 0

	DMA Isr (transfer complete) -> TxAvail()
		- Stop DMA (~DmaEnab)	(<--- Alert ... currently happens
					only in musb_cleanup_urb)
		- TxPktRdy has to be set in mode 0 or for
			short packets in mode 1.
*/

#endif

/* Service a Tx-Available or dma completion irq for the endpoint */
void musb_host_tx(struct musb *pThis, u8 bEnd)
{
	int			nPipe;
	u8			bDone = FALSE;
	u16			wTxCsrVal;
	size_t			wLength = 0;
	u8			*pBuffer = NULL;
	struct urb		*pUrb;
	struct musb_hw_ep	*pEnd = pThis->aLocalEnd + bEnd;
	void __iomem		*epio = pEnd->regs;
	struct musb_qh		*qh = pEnd->out_qh;
	u32			status = 0;
	void __iomem		*pBase = pThis->pRegs;
	struct dma_channel	*dma;

	pUrb = next_urb(qh);

	MGC_SelectEnd(pBase, bEnd);
	wTxCsrVal = musb_readw(epio, MGC_O_HDRC_TXCSR);

	/* with CPPI, DMA sometimes triggers "extra" irqs */
	if (!pUrb) {
		DBG(4, "extra TX%d ready, csr %04x\n", bEnd, wTxCsrVal);
		goto finish;
	}

	nPipe = pUrb->pipe;
	dma = is_dma_capable() ? pEnd->tx_channel : NULL;
	DBG(4, "OUT/TX%d end, csr %04x%s\n", bEnd, wTxCsrVal,
			dma ? ", dma" : "");

	/* check for errors */
	if (wTxCsrVal & MGC_M_TXCSR_H_RXSTALL) {
		/* dma was disabled, fifo flushed */
		DBG(3, "TX end %d stall\n", bEnd);

		/* stall; record URB status */
		status = -EPIPE;

	} else if (wTxCsrVal & MGC_M_TXCSR_H_ERROR) {
		/* (NON-ISO) dma was disabled, fifo flushed */
		DBG(3, "TX 3strikes on ep=%d\n", bEnd);

		status = -ETIMEDOUT;

	} else if (wTxCsrVal & MGC_M_TXCSR_H_NAKTIMEOUT) {
		DBG(6, "TX end=%d device not responding\n", bEnd);

		/* NOTE:  this code path would be a good place to PAUSE a
		 * transfer, if there's some other (nonperiodic) tx urb
		 * that could use this fifo.  (dma complicates it...)
		 *
		 * if (bulk && qh->ring.next != &musb->out_bulk), then
		 * we have a candidate... NAKing is *NOT* an error
		 */
		MGC_SelectEnd(pBase, bEnd);
		musb_writew(epio, MGC_O_HDRC_CSR0,
				MGC_M_TXCSR_H_WZC_BITS
				| MGC_M_TXCSR_TXPKTRDY);
		goto finish;
	}

	if (status) {
		if (dma_channel_status(dma) == MGC_DMA_STATUS_BUSY) {
			dma->bStatus = MGC_DMA_STATUS_CORE_ABORT;
			(void) pThis->pDmaController->channel_abort(dma);
		}

		/* do the proper sequence to abort the transfer in the
		 * usb core; the dma engine should already be stopped.
		 */
		musb_h_tx_flush_fifo(pEnd);
		wTxCsrVal &= ~(MGC_M_TXCSR_AUTOSET
				| MGC_M_TXCSR_DMAENAB
				| MGC_M_TXCSR_H_ERROR
				| MGC_M_TXCSR_H_RXSTALL
				| MGC_M_TXCSR_H_NAKTIMEOUT
				);

		MGC_SelectEnd(pBase, bEnd);
		musb_writew(epio, MGC_O_HDRC_TXCSR, wTxCsrVal);
		/* REVISIT may need to clear FLUSHFIFO ... */
		musb_writew(epio, MGC_O_HDRC_TXCSR, wTxCsrVal);
		musb_writeb(epio, MGC_O_HDRC_TXINTERVAL, 0);

		bDone = TRUE;
	}

	/* second cppi case */
	if (dma_channel_status(dma) == MGC_DMA_STATUS_BUSY) {
		DBG(4, "extra TX%d ready, csr %04x\n", bEnd, wTxCsrVal);
		goto finish;

	}

	/* REVISIT this looks wrong... */
	if (!status || dma || usb_pipeisoc(nPipe)) {
		if (dma)
			wLength = dma->dwActualLength;
		else
			wLength = qh->segsize;
		qh->offset += wLength;

		if (usb_pipeisoc(nPipe)) {
			struct usb_iso_packet_descriptor	*d;

			d = pUrb->iso_frame_desc + qh->iso_idx;
			d->actual_length = qh->segsize;
			if (++qh->iso_idx >= pUrb->number_of_packets) {
				bDone = TRUE;
			} else if (!dma) {
				d++;
				pBuffer = pUrb->transfer_buffer + d->offset;
				wLength = d->length;
			}
		} else if (dma) {
			bDone = TRUE;
		} else {
			/* see if we need to send more data, or ZLP */
			if (qh->segsize < qh->maxpacket)
				bDone = TRUE;
			else if (qh->offset == pUrb->transfer_buffer_length
					&& !(pUrb-> transfer_flags
							& URB_ZERO_PACKET))
				bDone = TRUE;
			if (!bDone) {
				pBuffer = pUrb->transfer_buffer
						+ qh->offset;
				wLength = pUrb->transfer_buffer_length
						- qh->offset;
			}
		}
	}

	/* urb->status != -EINPROGRESS means request has been faulted,
	 * so we must abort this transfer after cleanup
	 */
	if (pUrb->status != -EINPROGRESS) {
		bDone = TRUE;
		if (status == 0)
			status = pUrb->status;
	}

	if (bDone) {
		/* set status */
		pUrb->status = status;
		pUrb->actual_length = qh->offset;
		musb_advance_schedule(pThis, pUrb, pEnd, USB_DIR_OUT);

	} else if (!(wTxCsrVal & MGC_M_TXCSR_DMAENAB)) {
		// WARN_ON(!pBuffer);

		/* REVISIT:  some docs say that when pEnd->tx_double_buffered,
		 * (and presumably, fifo is not half-full) we should write TWO
		 * packets before updating TXCSR ... other docs disagree ...
		 */
		/* PIO:  start next packet in this URB */
		wLength = min(qh->maxpacket, (u16) wLength);
		musb_write_fifo(pEnd, wLength, pBuffer);
		qh->segsize = wLength;

		MGC_SelectEnd(pBase, bEnd);
		musb_writew(epio, MGC_O_HDRC_TXCSR,
				MGC_M_TXCSR_H_WZC_BITS | MGC_M_TXCSR_TXPKTRDY);
	} else
		DBG(1, "not complete, but dma enabled?\n");

finish:
	return;
}


#ifdef CONFIG_USB_INVENTRA_DMA

/* Host side RX (IN) using Mentor DMA works as follows:
	submit_urb ->
		- if queue was empty, ProgramEndpoint
		- first IN token is sent out (by setting ReqPkt)
	LinuxIsr -> RxReady()
	/\	=> first packet is received
	|	- Set in mode 0 (DmaEnab, ~ReqPkt)
	|		-> DMA Isr (transfer complete) -> RxReady()
	|		    - Ack receive (~RxPktRdy), turn off DMA (~DmaEnab)
	|		    - if urb not complete, send next IN token (ReqPkt)
	|			   |		else complete urb.
	|			   |
	---------------------------
 *
 * Nuances of mode 1:
 *	For short packets, no ack (+RxPktRdy) is sent automatically
 *	(even if AutoClear is ON)
 *	For full packets, ack (~RxPktRdy) and next IN token (+ReqPkt) is sent
 *	automatically => major problem, as collecting the next packet becomes
 *	difficult. Hence mode 1 is not used.
 *
 * REVISIT
 *	All we care about at this driver level is that
 *       (a) all URBs terminate with REQPKT cleared and fifo(s) empty;
 *       (b) termination conditions are: short RX, or buffer full;
 *       (c) fault modes include
 *           - iff URB_SHORT_NOT_OK, short RX status is -EREMOTEIO.
 *             (and that endpoint's dma queue stops immediately)
 *           - overflow (full, PLUS more bytes in the terminal packet)
 *
 *	So for example, usb-storage sets URB_SHORT_NOT_OK, and would
 *	thus be a great candidate for using mode 1 ... for all but the
 *	last packet of one URB's transfer.
 */

#endif

/*
 * Service an RX interrupt for the given IN endpoint; docs cover bulk, iso,
 * and high-bandwidth IN transfer cases.
 */
void musb_host_rx(struct musb *pThis, u8 bEnd)
{
	struct urb		*pUrb;
	struct musb_hw_ep	*pEnd = pThis->aLocalEnd + bEnd;
	void __iomem		*epio = pEnd->regs;
	struct musb_qh		*qh = pEnd->in_qh;
	size_t			xfer_len;
	void __iomem		*pBase = pThis->pRegs;
	int			nPipe;
	u16			wRxCsrVal, wVal;
	u8			bIsochError = FALSE;
	u8			bDone = FALSE;
	u32			status;
	struct dma_channel	*dma;

	MGC_SelectEnd(pBase, bEnd);

	pUrb = next_urb(qh);
	dma = is_dma_capable() ? pEnd->rx_channel : NULL;
	status = 0;
	xfer_len = 0;

	wVal = wRxCsrVal = musb_readw(epio, MGC_O_HDRC_RXCSR);

	if (unlikely(!pUrb)) {
		/* REVISIT -- THIS SHOULD NEVER HAPPEN ... but, at least
		 * usbtest #11 (unlinks) triggers it regularly, sometimes
		 * with fifo full.  (Only with DMA??)
		 */
		DBG(3, "BOGUS RX%d ready, csr %04x, count %d\n", bEnd, wVal,
			musb_readw(epio, MGC_O_HDRC_RXCOUNT));
		musb_h_flush_rxfifo(pEnd, MGC_M_RXCSR_CLRDATATOG);
		return;
	}

	nPipe = pUrb->pipe;

	DBG(5, "<== hw %d rxcsr %04x, urb actual %d (+dma %zd)\n",
		bEnd, wRxCsrVal, pUrb->actual_length,
		dma ? dma->dwActualLength : 0);

	/* check for errors, concurrent stall & unlink is not really
	 * handled yet! */
	if (wRxCsrVal & MGC_M_RXCSR_H_RXSTALL) {
		DBG(3, "RX end %d STALL\n", bEnd);

		/* stall; record URB status */
		status = -EPIPE;

	} else if (wRxCsrVal & MGC_M_RXCSR_H_ERROR) {
		DBG(3, "end %d RX proto error\n", bEnd);

		status = -EPROTO;
		musb_writeb(epio, MGC_O_HDRC_RXINTERVAL, 0);

	} else if (wRxCsrVal & MGC_M_RXCSR_DATAERROR) {

		if (USB_ENDPOINT_XFER_ISOC != qh->type) {
			/* NOTE this code path would be a good place to PAUSE a
			 * transfer, if there's some other (nonperiodic) rx urb
			 * that could use this fifo.  (dma complicates it...)
			 *
			 * if (bulk && qh->ring.next != &musb->in_bulk), then
			 * we have a candidate... NAKing is *NOT* an error
			 */
			DBG(6, "RX end %d NAK timeout\n", bEnd);
			MGC_SelectEnd(pBase, bEnd);
			musb_writew(epio, MGC_O_HDRC_RXCSR,
					MGC_M_RXCSR_H_WZC_BITS
					| MGC_M_RXCSR_H_REQPKT);

			goto finish;
		} else {
			DBG(4, "RX end %d ISO data error\n", bEnd);
			/* packet error reported later */
			bIsochError = TRUE;
		}
	}

	/* faults abort the transfer */
	if (status) {
		/* clean up dma and collect transfer count */
		if (dma_channel_status(dma) == MGC_DMA_STATUS_BUSY) {
			dma->bStatus = MGC_DMA_STATUS_CORE_ABORT;
			(void) pThis->pDmaController->channel_abort(dma);
			xfer_len = dma->dwActualLength;
		}
		musb_h_flush_rxfifo(pEnd, 0);
		musb_writeb(epio, MGC_O_HDRC_RXINTERVAL, 0);
		bDone = TRUE;
		goto finish;
	}

	if (unlikely(dma_channel_status(dma) == MGC_DMA_STATUS_BUSY)) {
		/* SHOULD NEVER HAPPEN ... but at least DaVinci has done it */
		ERR("RX%d dma busy, csr %04x\n", bEnd, wRxCsrVal);
		goto finish;
	}

	/* thorough shutdown for now ... given more precise fault handling
	 * and better queueing support, we might keep a DMA pipeline going
	 * while processing this irq for earlier completions.
	 */

	/* FIXME this is _way_ too much in-line logic for Mentor DMA */

#ifndef CONFIG_USB_INVENTRA_DMA
	if (wRxCsrVal & MGC_M_RXCSR_H_REQPKT)  {
		/* REVISIT this happened for a while on some short reads...
		 * the cleanup still needs investigation... looks bad...
		 * and also duplicates dma cleanup code above ... plus,
		 * shouldn't this be the "half full" double buffer case?
		 */
		if (dma_channel_status(dma) == MGC_DMA_STATUS_BUSY) {
			dma->bStatus = MGC_DMA_STATUS_CORE_ABORT;
			(void) pThis->pDmaController->channel_abort(dma);
			xfer_len = dma->dwActualLength;
			bDone = TRUE;
		}

		DBG(2, "RXCSR%d %04x, reqpkt, len %zd%s\n", bEnd, wRxCsrVal,
				xfer_len, dma ? ", dma" : "");
		wRxCsrVal &= ~MGC_M_RXCSR_H_REQPKT;

		MGC_SelectEnd(pBase, bEnd);
		musb_writew(epio, MGC_O_HDRC_RXCSR,
				MGC_M_RXCSR_H_WZC_BITS | wRxCsrVal);
	}
#endif
	if (dma && (wRxCsrVal & MGC_M_RXCSR_DMAENAB)) {
		xfer_len = dma->dwActualLength;

		wVal &= ~(MGC_M_RXCSR_DMAENAB
			| MGC_M_RXCSR_H_AUTOREQ
			| MGC_M_RXCSR_AUTOCLEAR
			| MGC_M_RXCSR_RXPKTRDY);
		musb_writew(pEnd->regs, MGC_O_HDRC_RXCSR, wVal);

#ifdef CONFIG_USB_INVENTRA_DMA
		/* bDone if pUrb buffer is full or short packet is recd */
		bDone = ((pUrb->actual_length + xfer_len) >=
				pUrb->transfer_buffer_length)
			|| (dma->dwActualLength & (qh->maxpacket - 1));

		/* send IN token for next packet, without AUTOREQ */
		if (!bDone) {
			wVal |= MGC_M_RXCSR_H_REQPKT;
			musb_writew(epio, MGC_O_HDRC_RXCSR,
				MGC_M_RXCSR_H_WZC_BITS | wVal);
		}

		DBG(4, "ep %d dma %s, rxcsr %04x, rxcount %d\n", bEnd,
			bDone ? "off" : "reset",
			musb_readw(epio, MGC_O_HDRC_RXCSR),
			musb_readw(epio, MGC_O_HDRC_RXCOUNT));
#else
		bDone = TRUE;
#endif
	} else if (pUrb->status == -EINPROGRESS) {
		/* if no errors, be sure a packet is ready for unloading */
		if (unlikely(!(wRxCsrVal & MGC_M_RXCSR_RXPKTRDY))) {
			status = -EPROTO;
			ERR("Rx interrupt with no errors or packet!\n");

			// FIXME this is another "SHOULD NEVER HAPPEN"

// SCRUB (RX)
			/* do the proper sequence to abort the transfer */
			MGC_SelectEnd(pBase, bEnd);
			wVal &= ~MGC_M_RXCSR_H_REQPKT;
			musb_writew(epio, MGC_O_HDRC_RXCSR, wVal);
			goto finish;
		}

		/* we are expecting IN packets */
#ifdef CONFIG_USB_INVENTRA_DMA
		if (dma) {
			struct dma_controller	*c;
			u16			wRxCount;
			int			status;

			wRxCount = musb_readw(epio, MGC_O_HDRC_RXCOUNT);

			DBG(2, "RX%d count %d, buffer 0x%x len %d/%d\n",
					bEnd, wRxCount,
					pUrb->transfer_dma
						+ pUrb->actual_length,
					qh->offset,
					pUrb->transfer_buffer_length);

			c = pThis->pDmaController;

			dma->bDesiredMode = 0;
#ifdef USE_MODE1
			/* because of the issue below, mode 1 will
			 * only rarely behave with correct semantics.
			 */
			if ((pUrb->transfer_flags &
						URB_SHORT_NOT_OK)
				&& (pUrb->transfer_buffer_length -
						pUrb->actual_length)
					> qh->maxpacket)
				dma->bDesiredMode = 1;
#endif

/* Disadvantage of using mode 1:
 *	It's basically usable only for mass storage class; essentially all
 *	other protocols also terminate transfers on short packets.
 *
 * Details:
 *	An extra IN token is sent at the end of the transfer (due to AUTOREQ)
 *	If you try to use mode 1 for (transfer_buffer_length - 512), and try
 *	to use the extra IN token to grab the last packet using mode 0, then
 *	the problem is that you cannot be sure when the device will send the
 *	last packet and RxPktRdy set. Sometimes the packet is recd too soon
 *	such that it gets lost when RxCSR is re-set at the end of the mode 1
 *	transfer, while sometimes it is recd just a little late so that if you
 *	try to configure for mode 0 soon after the mode 1 transfer is
 *	completed, you will find rxcount 0. Okay, so you might think why not
 *	wait for an interrupt when the pkt is recd. Well, you won't get any!
 */

			wVal = musb_readw(epio, MGC_O_HDRC_RXCSR);
			wVal &= ~MGC_M_RXCSR_H_REQPKT;

			if (dma->bDesiredMode == 0)
				wVal &= ~MGC_M_RXCSR_H_AUTOREQ;
			else
				wVal |= MGC_M_RXCSR_H_AUTOREQ;
			wVal |= MGC_M_RXCSR_AUTOCLEAR | MGC_M_RXCSR_DMAENAB;

			musb_writew(epio, MGC_O_HDRC_RXCSR,
				MGC_M_RXCSR_H_WZC_BITS | wVal);

			/* REVISIT if when actual_length != 0,
			 * transfer_buffer_length needs to be
			 * adjusted first...
			 */
			status = c->channel_program(
				dma, qh->maxpacket,
				dma->bDesiredMode,
				pUrb->transfer_dma
					+ pUrb->actual_length,
				(dma->bDesiredMode == 0)
					? wRxCount
					: pUrb->transfer_buffer_length);

			if (!status) {
				c->channel_release(dma);
				dma = pEnd->rx_channel = NULL;
				/* REVISIT reset CSR */
			}
		}
#endif	/* Mentor DMA */

		if (!dma) {
			bDone = musb_host_packet_rx(pThis, pUrb,
					bEnd, bIsochError);
			DBG(6, "read %spacket\n", bDone ? "last " : "");
		}
	}

finish:
	pUrb->actual_length += xfer_len;
	qh->offset += xfer_len;
	if (bDone) {
		if (pUrb->status == -EINPROGRESS)
			pUrb->status = status;
		musb_advance_schedule(pThis, pUrb, pEnd, USB_DIR_IN);
	}
}

/* schedule nodes correspond to peripheral endpoints, like an OHCI QH.
 * the software schedule associates multiple such nodes with a given
 * host side hardware endpoint + direction; scheduling may activate
 * that hardware endpoint.
 */
static int musb_schedule(
	struct musb		*musb,
	struct musb_qh		*qh,
	int			is_in)
{
	int			idle;
	int			wBestDiff;
	int			nBestEnd, nEnd;
	struct musb_hw_ep	*hw_ep = NULL;
	struct list_head	*head = NULL;

	/* use fixed hardware for control and bulk */
	switch (qh->type) {
	case USB_ENDPOINT_XFER_CONTROL:
		head = &musb->control;
		hw_ep = musb->control_ep;
		break;
	case USB_ENDPOINT_XFER_BULK:
		hw_ep = musb->bulk_ep;
		if (is_in)
			head = &musb->in_bulk;
		else
			head = &musb->out_bulk;
		break;
	}
	if (head) {
		idle = list_empty(head);
		list_add_tail(&qh->ring, head);
		goto success;
	}

	/* else, periodic transfers get muxed to other endpoints */

	/* FIXME this doesn't consider direction, so it can only
	 * work for one half of the endpoint hardware, and assumes
	 * the previous cases handled all non-shared endpoints...
	 */

	/* we know this qh hasn't been scheduled, so all we need to do
	 * is choose which hardware endpoint to put it on ...
	 *
	 * REVISIT what we really want here is a regular schedule tree
	 * like e.g. OHCI uses, but for now musb->periodic is just an
	 * array of the _single_ logical endpoint associated with a
	 * given physical one (identity mapping logical->physical).
	 *
	 * that simplistic approach makes TT scheduling a lot simpler;
	 * there is none, and thus none of its complexity...
	 */
	wBestDiff = 4096;
	nBestEnd = -1;

	for (nEnd = 1; nEnd < musb->bEndCount; nEnd++) {
		int	diff;

		if (musb->periodic[nEnd])
			continue;
		hw_ep = &musb->aLocalEnd[nEnd];
		if (hw_ep == musb->bulk_ep)
			continue;

		if (is_in)
			diff = hw_ep->wMaxPacketSizeRx - qh->maxpacket;
		else
			diff = hw_ep->wMaxPacketSizeTx - qh->maxpacket;

		if (diff > 0 && wBestDiff > diff) {
			wBestDiff = diff;
			nBestEnd = nEnd;
		}
	}
	if (nBestEnd < 0)
		return -ENOSPC;

	idle = 1;
	hw_ep = musb->aLocalEnd + nBestEnd;
	musb->periodic[nBestEnd] = qh;
	DBG(4, "qh %p periodic slot %d\n", qh, nBestEnd);
success:
	qh->hw_ep = hw_ep;
	qh->hep->hcpriv = qh;
	if (idle)
		musb_start_urb(musb, is_in, qh);
	return 0;
}

static int musb_urb_enqueue(
	struct usb_hcd			*hcd,
	struct usb_host_endpoint	*hep,
	struct urb			*urb,
	gfp_t				mem_flags)
{
	unsigned long			flags;
	struct musb			*musb = hcd_to_musb(hcd);
	struct musb_qh			*qh = hep->hcpriv;
	struct usb_endpoint_descriptor	*epd = &hep->desc;
	int				status;
	unsigned			type_reg;
	unsigned			interval;

	/* host role must be active */
	if (!is_host_active(musb) || !musb->is_active)
		return -ENODEV;

	/* DMA mapping was already done, if needed, and this urb is on
	 * hep->urb_list ... so there's little to do unless hep wasn't
	 * yet scheduled onto a live qh.
	 *
	 * REVISIT best to keep hep->hcpriv valid until the endpoint gets
	 * disabled, testing for empty qh->ring and avoiding qh setup costs
	 * except for the first urb queued after a config change.
	 */
	if (qh) {
		urb->hcpriv = qh;
		return 0;
	}

	/* Allocate and initialize qh, minimizing the work done each time
	 * hw_ep gets reprogrammed, or with irqs blocked.  Then schedule it.
	 *
	 * REVISIT consider a dedicated qh kmem_cache, so it's harder
	 * for bugs in other kernel code to break this driver...
	 */
	qh = kzalloc(sizeof *qh, mem_flags);
	if (!qh)
		return -ENOMEM;

	qh->hep = hep;
	qh->dev = urb->dev;
	INIT_LIST_HEAD(&qh->ring);
	qh->is_ready = 1;

	qh->maxpacket = le16_to_cpu(epd->wMaxPacketSize);

	/* no high bandwidth support yet */
	if (qh->maxpacket & ~0x7ff) {
		status = -EMSGSIZE;
		goto done;
	}

	qh->epnum = epd->bEndpointAddress & USB_ENDPOINT_NUMBER_MASK;
	qh->type = epd->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK;

	/* NOTE: urb->dev->devnum is wrong during SET_ADDRESS */
	qh->addr_reg = (u8) usb_pipedevice(urb->pipe);

	/* precompute rxtype/txtype/type0 register */
	type_reg = (qh->type << 4) | qh->epnum;
	switch (urb->dev->speed) {
	case USB_SPEED_LOW:
		type_reg |= 0xc0;
		break;
	case USB_SPEED_FULL:
		type_reg |= 0x80;
		break;
	default:
		type_reg |= 0x40;
	}
	qh->type_reg = type_reg;

	/* precompute rxinterval/txinterval register */
	interval = min((u8)16, epd->bInterval);	/* log encoding */
	switch (qh->type) {
	case USB_ENDPOINT_XFER_INT:
		/* fullspeed uses linear encoding */
		if (USB_SPEED_FULL == urb->dev->speed) {
			interval = epd->bInterval;
			if (!interval)
				interval = 1;
		}
		/* FALLTHROUGH */
	case USB_ENDPOINT_XFER_ISOC:
		/* iso always uses log encoding */
		break;
	default:
		/* REVISIT we actually want to use NAK limits, hinting to the
		 * transfer scheduling logic to try some other qh, e.g. try
		 * for 2 msec first:
		 *
		 * interval = (USB_SPEED_HIGH == pUrb->dev->speed) ? 16 : 2;
		 *
		 * The downside of disabling this is that transfer scheduling
		 * gets VERY unfair for nonperiodic transfers; a misbehaving
		 * peripheral could make that hurt.  Or for reads, one that's
		 * perfectly normal:  network and other drivers keep reads
		 * posted at all times, having one pending for a week should
		 * be perfectly safe.
		 *
		 * The upside of disabling it is avoidng transfer scheduling
		 * code to put this aside for while.
		 */
		interval = 0;
	}
	qh->intv_reg = interval;

	/* precompute addressing for external hub/tt ports */
	if (musb->bIsMultipoint) {
		struct usb_device	*parent = urb->dev->parent;

		if (parent != hcd->self.root_hub) {
			qh->h_addr_reg = (u8) parent->devnum;

			/* set up tt info if needed */
			if (urb->dev->tt) {
				qh->h_port_reg = (u8) urb->dev->ttport;
				qh->h_addr_reg |= 0x80;
			}
		}
	}

	/* invariant: hep->hcpriv is null OR the qh that's already scheduled.
	 * until we get real dma queues (with an entry for each urb/buffer),
	 * we only have work to do in the former case.
	 */
	spin_lock_irqsave(&musb->Lock, flags);
	if (hep->hcpriv) {
		/* some concurrent activity submitted another urb to hep...
		 * odd, rare, error prone, but legal.
		 */
		kfree(qh);
		status = 0;
	} else
		status = musb_schedule(musb, qh,
				epd->bEndpointAddress & USB_ENDPOINT_DIR_MASK);

	if (status == 0) {
		urb->hcpriv = qh;
		/* FIXME set urb->start_frame for iso/intr, it's tested in
		 * musb_start_urb(), but otherwise only konicawc cares ...
		 */
	}
	spin_unlock_irqrestore(&musb->Lock, flags);

done:
	if (status != 0)
		kfree(qh);
	return status;
}


/*
 * abort a transfer that's at the head of a hardware queue.
 * called with controller locked, irqs blocked
 * that hardware queue advances to the next transfer, unless prevented
 */
static int musb_cleanup_urb(struct urb *urb, struct musb_qh *qh, int is_in)
{
	struct musb_hw_ep	*ep = qh->hw_ep;
	void __iomem		*epio = ep->regs;
	unsigned		hw_end = ep->bLocalEnd;
	void __iomem		*regs = ep->musb->pRegs;
	u16			csr;
	int			status = 0;

	MGC_SelectEnd(regs, hw_end);

	if (is_dma_capable()) {
		struct dma_channel	*dma;

		dma = is_in ? ep->rx_channel : ep->tx_channel;
		if (dma) {
			status = ep->musb->pDmaController->channel_abort(dma);
			DBG(status ? 1 : 3,
				"abort %cX%d DMA for urb %p --> %d\n",
				is_in ? 'R' : 'T', ep->bLocalEnd,
				urb, status);
			urb->actual_length += dma->dwActualLength;
		}
	}

	/* turn off DMA requests, discard state, stop polling ... */
	if (is_in) {
		/* giveback saves bulk toggle */
		csr = musb_h_flush_rxfifo(ep, 0);

		/* REVISIT we still get an irq; should likely clear the
		 * endpoint's irq status here to avoid bogus irqs.
		 * clearing that status is platform-specific...
		 */
	} else {
		musb_h_tx_flush_fifo(ep);
		csr = musb_readw(epio, MGC_O_HDRC_TXCSR);
		csr &= ~( MGC_M_TXCSR_AUTOSET
			| MGC_M_TXCSR_DMAENAB
			| MGC_M_TXCSR_H_RXSTALL
			| MGC_M_TXCSR_H_NAKTIMEOUT
			| MGC_M_TXCSR_H_ERROR
			| MGC_M_TXCSR_TXPKTRDY
			);
		musb_writew(epio, MGC_O_HDRC_TXCSR, csr);
		/* REVISIT may need to clear FLUSHFIFO ... */
		musb_writew(epio, MGC_O_HDRC_TXCSR, csr);
		/* flush cpu writebuffer */
		csr = musb_readw(epio, MGC_O_HDRC_TXCSR);
	}
	if (status == 0)
		musb_advance_schedule(ep->musb, urb, ep, is_in);
	return status;
}

static int musb_urb_dequeue(struct usb_hcd *hcd, struct urb *urb)
{
	struct musb		*musb = hcd_to_musb(hcd);
	struct musb_qh		*qh;
	struct list_head	*sched;
	struct urb		*tmp;
	unsigned long		flags;
	int			status = -ENOENT;

	DBG(4, "urb=%p, dev%d ep%d%s\n", urb,
			usb_pipedevice(urb->pipe),
			usb_pipeendpoint(urb->pipe),
			usb_pipein(urb->pipe) ? "in" : "out");

	spin_lock_irqsave(&musb->Lock, flags);

	/* make sure the urb is still queued and not completed */
	spin_lock(&urb->lock);
	qh = urb->hcpriv;
	if (qh) {
		struct usb_host_endpoint	*hep;

		hep = qh->hep;
		list_for_each_entry(tmp, &hep->urb_list, urb_list) {
			if (urb == tmp) {
				status = 0;
				break;
			}
		}
	}
	spin_unlock(&urb->lock);

	/* already completed */
	if (!qh) {
		status = 0;
		goto done;
	}

	/* still queued but not found on the list */
	if (status)
		goto done;

	/* Any URB not actively programmed into endpoint hardware can be
	 * immediately given back.  Such an URB must be at the head of its
	 * endpoint queue, unless someday we get real DMA queues.  And even
	 * then, it might not be known to the hardware...
	 *
	 * Otherwise abort current transfer, pending dma, etc.; urb->status
	 * has already been updated.  This is a synchronous abort; it'd be
	 * OK to hold off until after some IRQ, though.
	 */
	if (!qh->is_ready || urb->urb_list.prev != &qh->hep->urb_list)
		status = -EINPROGRESS;
	else {
		switch (qh->type) {
		case USB_ENDPOINT_XFER_CONTROL:
			sched = &musb->control;
			break;
		case USB_ENDPOINT_XFER_BULK:
			if (usb_pipein(urb->pipe))
				sched = &musb->in_bulk;
			else
				sched = &musb->out_bulk;
			break;
		default:
			/* REVISIT when we get a schedule tree, periodic
			 * transfers won't always be at the head of a
			 * singleton queue...
			 */
			sched = NULL;
			break;
		}
	}

	/* NOTE:  qh is invalid unless !list_empty(&hep->urb_list) */
	if (status < 0 || (sched && qh != first_qh(sched))) {
		int	ready = qh->is_ready;

		status = 0;
		qh->is_ready = 0;
		__musb_giveback(musb, urb, 0);
		qh->is_ready = ready;
	} else
		status = musb_cleanup_urb(urb, qh, urb->pipe & USB_DIR_IN);
done:
	spin_unlock_irqrestore(&musb->Lock, flags);
	return status;
}

/* disable an endpoint */
static void
musb_h_disable(struct usb_hcd *hcd, struct usb_host_endpoint *hep)
{
	u8			epnum = hep->desc.bEndpointAddress;
	unsigned long		flags;
	struct musb		*musb = hcd_to_musb(hcd);
	u8			is_in = epnum & USB_DIR_IN;
	struct musb_qh		*qh = hep->hcpriv;
	struct urb		*urb, *tmp;
	struct list_head	*sched;

	if (!qh)
		return;

	spin_lock_irqsave(&musb->Lock, flags);

	switch (qh->type) {
	case USB_ENDPOINT_XFER_CONTROL:
		sched = &musb->control;
		break;
	case USB_ENDPOINT_XFER_BULK:
		if (is_in)
			sched = &musb->in_bulk;
		else
			sched = &musb->out_bulk;
		break;
	default:
		/* REVISIT when we get a schedule tree, periodic transfers
		 * won't always be at the head of a singleton queue...
		 */
		sched = NULL;
		break;
	}

	/* NOTE:  qh is invalid unless !list_empty(&hep->urb_list) */

	/* kick first urb off the hardware, if needed */
	qh->is_ready = 0;
	if (!sched || qh == first_qh(sched)) {
		urb = next_urb(qh);

		/* make software (then hardware) stop ASAP */
		spin_lock(&urb->lock);
		if (urb->status == -EINPROGRESS)
			urb->status = -ESHUTDOWN;
		spin_unlock(&urb->lock);

		/* cleanup */
		musb_cleanup_urb(urb, qh, urb->pipe & USB_DIR_IN);
	} else
		urb = NULL;

	/* then just nuke all the others */
	list_for_each_entry_safe_from(urb, tmp, &hep->urb_list, urb_list)
		musb_giveback(qh, urb, -ESHUTDOWN);

	spin_unlock_irqrestore(&musb->Lock, flags);
}

static int musb_h_get_frame_number(struct usb_hcd *hcd)
{
	struct musb	*musb = hcd_to_musb(hcd);

	return musb_readw(musb->pRegs, MGC_O_HDRC_FRAME);
}

static int musb_h_start(struct usb_hcd *hcd)
{
	struct musb	*musb = hcd_to_musb(hcd);

	/* NOTE: musb_start() is called when the hub driver turns
	 * on port power, or when (OTG) peripheral starts.
	 */
	hcd->state = HC_STATE_RUNNING;
	musb->port1_status = 0;
	return 0;
}

static void musb_h_stop(struct usb_hcd *hcd)
{
	musb_stop(hcd_to_musb(hcd));
	hcd->state = HC_STATE_HALT;
}

static int musb_bus_suspend(struct usb_hcd *hcd)
{
	struct musb	*musb = hcd_to_musb(hcd);

	if (is_host_active(musb) && musb->is_active)
		return -EBUSY;
	else
		return 0;
}

static int musb_bus_resume(struct usb_hcd *hcd)
{
	/* resuming child port does the work */
	return 0;
}

const struct hc_driver musb_hc_driver = {
	.description		= "musb-hcd",
	.product_desc		= "MUSB HDRC host driver",
	.hcd_priv_size		= sizeof (struct musb),
	.flags			= HCD_USB2 | HCD_MEMORY,

	/* not using irq handler or reset hooks from usbcore, since
	 * those must be shared with peripheral code for OTG configs
	 */

	.start			= musb_h_start,
	.stop			= musb_h_stop,

	.get_frame_number	= musb_h_get_frame_number,

	.urb_enqueue		= musb_urb_enqueue,
	.urb_dequeue		= musb_urb_dequeue,
	.endpoint_disable	= musb_h_disable,

	.hub_status_data	= musb_hub_status_data,
	.hub_control		= musb_hub_control,
	.bus_suspend		= musb_bus_suspend,
	.bus_resume		= musb_bus_resume,
//	.start_port_reset	= NULL,
//	.hub_irq_enable		= NULL,
};
