/******************************************************************
 * Copyright 2005 Mentor Graphics Corporation
 * Copyright (C) 2005-2007 by Texas Instruments
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

/*
 * Implementation for the DMA controller within the MUSBMHDRC.
 */

#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include "musbdefs.h"

#if defined(CONFIG_ARCH_OMAP2430) || defined(CONFIG_ARCH_OMAP3430)
#include "omap2430.h"
#endif

#define MGC_O_HSDMA_BASE		0x200
#define MGC_O_HSDMA_INTR		(MGC_O_HSDMA_BASE + 0)
#define MGC_O_HSDMA_CONTROL		0x4
#define MGC_O_HSDMA_ADDRESS		0x8
#define MGC_O_HSDMA_COUNT		0xc

#define MGC_HSDMA_CHANNEL_OFFSET(_bChannel, _bOffset)		\
		(MGC_O_HSDMA_BASE + (_bChannel << 4) + _bOffset)

/* control register (16-bit): */
#define MGC_S_HSDMA_ENABLE		0
#define MGC_S_HSDMA_TRANSMIT		1
#define MGC_S_HSDMA_MODE1		2
#define MGC_S_HSDMA_IRQENABLE		3
#define MGC_S_HSDMA_ENDPOINT		4
#define MGC_S_HSDMA_BUSERROR		8
#define MGC_S_HSDMA_BURSTMODE		9
#define MGC_M_HSDMA_BURSTMODE		(3 << MGC_S_HSDMA_BURSTMODE)
#define MGC_HSDMA_BURSTMODE_UNSPEC	0
#define MGC_HSDMA_BURSTMODE_INCR4	1
#define MGC_HSDMA_BURSTMODE_INCR8	2
#define MGC_HSDMA_BURSTMODE_INCR16	3

#define MGC_HSDMA_CHANNELS		8

struct musb_dma_controller;

struct musb_dma_channel {
	struct dma_channel		Channel;
	struct musb_dma_controller	*pController;
	u32				dwStartAddress;
	u32				dwCount;
	u16				wMaxPacketSize;
	u8				bIndex;
	u8				bEnd;
	u8				bTransmit;
};

struct musb_dma_controller {
	struct dma_controller		Controller;
	struct musb_dma_channel		aChannel[MGC_HSDMA_CHANNELS];
	void 				*pDmaPrivate;
	void __iomem 			*pCoreBase;
	u8 				bChannelCount;
	u8 				bmUsedChannels;
	u8				irq;
};

static int dma_controller_start(struct dma_controller *c)
{
	/* nothing to do */
	return 0;
}

static void dma_channel_release(struct dma_channel *pChannel);

static int dma_controller_stop(struct dma_controller *c)
{
	struct musb_dma_controller *pController =
		container_of(c, struct musb_dma_controller, Controller);
	struct musb *pThis = (struct musb *) pController->pDmaPrivate;
	struct dma_channel *pChannel;
	u8 bBit;

	if (pController->bmUsedChannels != 0) {
		dev_err(pThis->controller,
			"Stopping DMA controller while channel active\n");

		for (bBit = 0; bBit < MGC_HSDMA_CHANNELS; bBit++) {
			if (pController->bmUsedChannels & (1 << bBit)) {
				pChannel = &(pController->aChannel[bBit].Channel);
				dma_channel_release(pChannel);

				if (!pController->bmUsedChannels)
					break;
			}
		}
	}
	return 0;
}

static struct dma_channel* dma_channel_allocate(struct dma_controller *c,
				struct musb_hw_ep *hw_ep, u8 bTransmit)
{
	u8 bBit;
	struct dma_channel *pChannel = NULL;
	struct musb_dma_channel *pImplChannel = NULL;
	struct musb_dma_controller *pController =
			container_of(c, struct musb_dma_controller, Controller);

	for (bBit = 0; bBit < MGC_HSDMA_CHANNELS; bBit++) {
		if (!(pController->bmUsedChannels & (1 << bBit))) {
			pController->bmUsedChannels |= (1 << bBit);
			pImplChannel = &(pController->aChannel[bBit]);
			pImplChannel->pController = pController;
			pImplChannel->bIndex = bBit;
			pImplChannel->bEnd = hw_ep->bLocalEnd;
			pImplChannel->bTransmit = bTransmit;
			pChannel = &(pImplChannel->Channel);
			pChannel->pPrivateData = pImplChannel;
			pChannel->bStatus = MGC_DMA_STATUS_FREE;
			pChannel->dwMaxLength = 0x10000;
			/* Tx => mode 1; Rx => mode 0 */
			pChannel->bDesiredMode = bTransmit;
			pChannel->dwActualLength = 0;
			break;
		}
	}
	return pChannel;
}

static void dma_channel_release(struct dma_channel *pChannel)
{
	struct musb_dma_channel *pImplChannel =
		(struct musb_dma_channel *) pChannel->pPrivateData;

	pChannel->dwActualLength = 0;
	pImplChannel->dwStartAddress = 0;
	pImplChannel->dwCount = 0;

	pImplChannel->pController->bmUsedChannels &=
		~(1 << pImplChannel->bIndex);

	pChannel->bStatus = MGC_DMA_STATUS_UNKNOWN;
}

static void configure_channel(struct dma_channel *pChannel,
				u16 wPacketSize, u8 bMode,
				dma_addr_t dma_addr, u32 dwLength)
{
	struct musb_dma_channel *pImplChannel =
		(struct musb_dma_channel *) pChannel->pPrivateData;
	struct musb_dma_controller *pController = pImplChannel->pController;
	u8 *pBase = pController->pCoreBase;
	u8 bChannel = pImplChannel->bIndex;
	u16 wCsr = 0;

	DBG(4, "%p, pkt_sz %d, addr 0x%x, len %d, mode %d\n",
	    pChannel, wPacketSize, dma_addr, dwLength, bMode);

	if (bMode) {
		wCsr |= 1 << MGC_S_HSDMA_MODE1;
		if (dwLength < wPacketSize) {
			return FALSE;
		}
		if (wPacketSize >= 64) {
			wCsr |=
			    MGC_HSDMA_BURSTMODE_INCR16 << MGC_S_HSDMA_BURSTMODE;
		} else if (wPacketSize >= 32) {
			wCsr |=
			    MGC_HSDMA_BURSTMODE_INCR8 << MGC_S_HSDMA_BURSTMODE;
		} else if (wPacketSize >= 16) {
			wCsr |=
			    MGC_HSDMA_BURSTMODE_INCR4 << MGC_S_HSDMA_BURSTMODE;
		}
	}

	wCsr |= (pImplChannel->bEnd << MGC_S_HSDMA_ENDPOINT)
		| (1 << MGC_S_HSDMA_ENABLE)
		| (1 << MGC_S_HSDMA_IRQENABLE)
		| (pImplChannel->bTransmit ? (1 << MGC_S_HSDMA_TRANSMIT) : 0);

	/* address/count */
	musb_writel(pBase,
		    MGC_HSDMA_CHANNEL_OFFSET(bChannel, MGC_O_HSDMA_ADDRESS),
		    dma_addr);
	musb_writel(pBase,
		    MGC_HSDMA_CHANNEL_OFFSET(bChannel, MGC_O_HSDMA_COUNT),
		    dwLength);

	/* control (this should start things) */
	musb_writew(pBase,
		    MGC_HSDMA_CHANNEL_OFFSET(bChannel, MGC_O_HSDMA_CONTROL),
		    wCsr);
}

static int dma_channel_program(struct dma_channel * pChannel,
				u16 wPacketSize, u8 bMode,
				dma_addr_t dma_addr, u32 dwLength)
{
	struct musb_dma_channel *pImplChannel =
			(struct musb_dma_channel *) pChannel->pPrivateData;

	DBG(2, "ep%d-%s pkt_sz %d, dma_addr 0x%x length %d, mode %d\n",
		pImplChannel->bEnd,
		pImplChannel->bTransmit ? "Tx" : "Rx",
		wPacketSize, dma_addr, dwLength, bMode);

	BUG_ON(pChannel->bStatus == MGC_DMA_STATUS_UNKNOWN ||
		pChannel->bStatus == MGC_DMA_STATUS_BUSY);

	pChannel->dwActualLength = 0;
	pImplChannel->dwStartAddress = dma_addr;
	pImplChannel->dwCount = dwLength;
	pImplChannel->wMaxPacketSize = wPacketSize;
	pChannel->bStatus = MGC_DMA_STATUS_BUSY;

	if ((bMode == 1) && (dwLength >= wPacketSize)) {
		configure_channel(pChannel, wPacketSize, 1, dma_addr,
				  dwLength);
	} else
		configure_channel(pChannel, wPacketSize, 0, dma_addr,
				  dwLength);

	return TRUE;
}

static int dma_channel_abort(struct dma_channel *pChannel)
{
	struct musb_dma_channel *pImplChannel =
		(struct musb_dma_channel *) pChannel->pPrivateData;
	u8 bChannel = pImplChannel->bIndex;
	u8 *pBase = pImplChannel->pController->pCoreBase;
	u16 csr;

	if (pChannel->bStatus == MGC_DMA_STATUS_BUSY) {
		if (pImplChannel->bTransmit) {

			csr = musb_readw(pBase,
				MGC_END_OFFSET(pImplChannel->bEnd,MGC_O_HDRC_TXCSR));
			csr &= ~(MGC_M_TXCSR_AUTOSET |
				 MGC_M_TXCSR_DMAENAB |
				 MGC_M_TXCSR_DMAMODE);
			musb_writew(pBase,
					MGC_END_OFFSET(pImplChannel->bEnd,MGC_O_HDRC_TXCSR),
					csr);
		}
		else {
			csr = musb_readw(pBase,
				MGC_END_OFFSET(pImplChannel->bEnd,MGC_O_HDRC_RXCSR));
			csr &= ~(MGC_M_RXCSR_AUTOCLEAR |
				 MGC_M_RXCSR_DMAENAB |
				 MGC_M_RXCSR_DMAMODE);
			musb_writew(pBase,
					MGC_END_OFFSET(pImplChannel->bEnd,MGC_O_HDRC_RXCSR),
					csr);
		}

		musb_writew(pBase,
		   MGC_HSDMA_CHANNEL_OFFSET(bChannel, MGC_O_HSDMA_CONTROL), 0);
		musb_writel(pBase,
		   MGC_HSDMA_CHANNEL_OFFSET(bChannel, MGC_O_HSDMA_ADDRESS), 0);
		musb_writel(pBase,
		   MGC_HSDMA_CHANNEL_OFFSET(bChannel, MGC_O_HSDMA_COUNT), 0);

		pChannel->bStatus = MGC_DMA_STATUS_FREE;
	}
	return 0;
}

static irqreturn_t dma_controller_irq(int irq, void *pPrivateData)
{
	struct musb_dma_controller *pController =
		(struct musb_dma_controller *)pPrivateData;
	struct musb_dma_channel *pImplChannel;
	u8 *pBase = pController->pCoreBase;
	struct dma_channel *pChannel;
	u8 bChannel;
	u16 wCsr;
	u32 dwAddress;
	u8 bIntr;
	irqreturn_t retval = IRQ_NONE;

	bIntr = musb_readb(pBase, MGC_O_HSDMA_INTR);
	if (!bIntr)
		goto done;

	for (bChannel = 0; bChannel < MGC_HSDMA_CHANNELS; bChannel++) {
		if (bIntr & (1 << bChannel)) {
			pImplChannel = (struct musb_dma_channel *)
					&(pController->aChannel[bChannel]);
			pChannel = &pImplChannel->Channel;

			wCsr = musb_readw(pBase,
				       MGC_HSDMA_CHANNEL_OFFSET(bChannel,
							MGC_O_HSDMA_CONTROL));

			if (wCsr & (1 << MGC_S_HSDMA_BUSERROR)) {
				pImplChannel->Channel.bStatus =
				    MGC_DMA_STATUS_BUS_ABORT;
			} else {
				dwAddress = musb_readl(pBase,
						MGC_HSDMA_CHANNEL_OFFSET(
							bChannel,
							MGC_O_HSDMA_ADDRESS));
				pChannel->dwActualLength =
				    dwAddress - pImplChannel->dwStartAddress;

				DBG(2, "ch %p, 0x%x -> 0x%x (%d / %d) %s\n",
				    pChannel, pImplChannel->dwStartAddress,
				    dwAddress, pChannel->dwActualLength,
				    pImplChannel->dwCount,
				    (pChannel->dwActualLength <
					pImplChannel->dwCount) ?
					"=> reconfig 0": "=> complete");

				u8 devctl = musb_readb(pBase,
						MGC_O_HDRC_DEVCTL);

				pChannel->bStatus = MGC_DMA_STATUS_FREE;

				/* completed */
				if ((devctl & MGC_M_DEVCTL_HM)
				    && (pImplChannel->bTransmit)
				    && ((pChannel->bDesiredMode == 0)
					|| (pChannel->dwActualLength &
					    (pImplChannel->wMaxPacketSize - 1)))
				   ) {
					/* Send out the packet */
					MGC_SelectEnd(pBase,
						pImplChannel->bEnd);
					musb_writew(pBase,
						MGC_END_OFFSET(pImplChannel->bEnd,MGC_O_HDRC_TXCSR),
						MGC_M_TXCSR_TXPKTRDY);
				} else
					musb_dma_completion(
						pController->pDmaPrivate,
						pImplChannel->bEnd,
						pImplChannel->bTransmit);
			}
		}
	}
	retval = IRQ_HANDLED;
done:
	return retval;
}

void dma_controller_destroy(struct dma_controller *c)
{
	struct musb_dma_controller *pController =
		(struct musb_dma_controller *) c->pPrivateData;

	if (!pController)
		return;

	if (pController->irq)
		free_irq(pController->irq, c);

	kfree(pController);
	c->pPrivateData = NULL;
}

struct dma_controller *__init
dma_controller_create(struct musb *pThis, void __iomem *pCoreBase)
{
	struct musb_dma_controller *pController;
	struct device *dev = pThis->controller;
	struct platform_device *pdev = to_platform_device(dev);
	int irq = platform_get_irq(pdev, 1);

	if (irq == 0) {
		dev_err(dev, "No DMA interrupt line!\n");
		return NULL;
	}

	if (!(pController = kzalloc(sizeof(struct musb_dma_controller),
				GFP_KERNEL)))
		return NULL;

	pController->bChannelCount = MGC_HSDMA_CHANNELS;
	pController->pDmaPrivate = pThis;
	pController->pCoreBase = pCoreBase;

	pController->Controller.pPrivateData = pController;
	pController->Controller.start = dma_controller_start;
	pController->Controller.stop = dma_controller_stop;
	pController->Controller.channel_alloc = dma_channel_allocate;
	pController->Controller.channel_release = dma_channel_release;
	pController->Controller.channel_program = dma_channel_program;
	pController->Controller.channel_abort = dma_channel_abort;

	if (request_irq(irq, dma_controller_irq, IRQF_DISABLED,
			pThis->controller->bus_id, &pController->Controller)) {
		dev_err(dev, "request_irq %d failed!\n", irq);
		dma_controller_destroy(&pController->Controller);
		return NULL;
	}

	pController->irq = irq;

	return &pController->Controller;
}
