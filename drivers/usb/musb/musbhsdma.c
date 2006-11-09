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

/*
 * DMA implementation for high-speed controllers.
 */

#include "musbdefs.h"


/****************************** CONSTANTS ********************************/

#define MGC_O_HSDMA_BASE    0x200
#define MGC_O_HSDMA_INTR    0x200

#define MGC_O_HSDMA_CONTROL 4
#define MGC_O_HSDMA_ADDRESS 8
#define MGC_O_HSDMA_COUNT   0xc

#define MGC_HSDMA_CHANNEL_OFFSET(_bChannel, _bOffset)		\
		(MGC_O_HSDMA_BASE + (_bChannel << 4) + _bOffset)

/* control register (16-bit): */
#define MGC_S_HSDMA_ENABLE	0
#define MGC_S_HSDMA_TRANSMIT	1
#define MGC_S_HSDMA_MODE1	2
#define MGC_S_HSDMA_IRQENABLE	3
#define MGC_S_HSDMA_ENDPOINT	4
#define MGC_S_HSDMA_BUSERROR	8
#define MGC_S_HSDMA_BURSTMODE	9
#define MGC_M_HSDMA_BURSTMODE	(3 << MGC_S_HSDMA_BURSTMODE)
#define MGC_HSDMA_BURSTMODE_UNSPEC  0
#define MGC_HSDMA_BURSTMODE_INCR4   1
#define MGC_HSDMA_BURSTMODE_INCR8   2
#define MGC_HSDMA_BURSTMODE_INCR16  3

#define MGC_HSDMA_CHANNELS 8

/******************************* Types ********************************/

struct _MGC_HsDmaController;

typedef struct {
	struct dma_channel Channel;
	struct _MGC_HsDmaController *pController;
	u32 dwStartAddress;
	u32 dwCount;
	u8 bIndex;
	u8 bEnd;
	u8 bTransmit;
} MGC_HsDmaChannel;

struct hsdma {
	struct dma_controller Controller;
	MGC_HsDmaChannel aChannel[MGC_HSDMA_CHANNELS];
	void *pDmaPrivate;
	void __iomem *pCoreBase;
	u8 bChannelCount;
	u8 bmUsedChannels;
};

/* FIXME remove typedef noise */
typedef struct hsdma MGC_HsDmaController;

/****************************** FUNCTIONS ********************************/

static int MGC_HsDmaStartController(struct dma_controller *c)
{
	/* nothing to do */
	return 0;
}

static int MGC_HsDmaStopController(struct dma_controller *c)
{
	/* nothing to do */
	return 0;
}

static struct dma_channel *MGC_HsDmaAllocateChannel(
		struct dma_controller *c,
		struct musb_hw_ep *hw_ep,
		u8 bTransmit)
{
	u8 bBit;
	struct dma_channel *pChannel = NULL;
	MGC_HsDmaChannel *pImplChannel = NULL;
	MGC_HsDmaController *pController;

	pcontroller = container_of(c, struct hsdma, Controller);
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

static void MGC_HsDmaReleaseChannel(struct dma_channel *pChannel)
{
	MGC_HsDmaChannel *pImplChannel =
	    (MGC_HsDmaChannel *) pChannel->pPrivateData;

	pImplChannel->pController->bmUsedChannels &=
	    ~(1 << pImplChannel->bIndex);
	pChannel->bStatus = MGC_DMA_STATUS_FREE;
}

static void clear_state(struct dma_channel *pChannel)
{
	MGC_HsDmaChannel *pImplChannel =
	    (MGC_HsDmaChannel *) pChannel->pPrivateData;
	MGC_HsDmaController *pController = pImplChannel->pController;
	u8 *pBase = pController->pCoreBase;
	u8 bChannel = pImplChannel->bIndex;

	musb_writew(pBase,
		    MGC_HSDMA_CHANNEL_OFFSET(bChannel, MGC_O_HSDMA_CONTROL),
		    0);
	musb_writel(pBase,
		    MGC_HSDMA_CHANNEL_OFFSET(bChannel, MGC_O_HSDMA_ADDRESS),
		    0);
	musb_writel(pBase,
		    MGC_HSDMA_CHANNEL_OFFSET(bChannel, MGC_O_HSDMA_COUNT),
		    0);

	pChannel->dwActualLength = 0L;
	pImplChannel->dwStartAddress = 0;
	pImplChannel->dwCount = 0;
}

static u8 configure_channel(struct dma_channel *pChannel,
				  u16 wPacketSize, u8 bMode,
				  dma_addr_t dma_addr, u32 dwLength)
{
	MGC_HsDmaChannel *pImplChannel =
	    (MGC_HsDmaChannel *) pChannel->pPrivateData;
	MGC_HsDmaController *pController = pImplChannel->pController;
	u8 *pBase = pController->pCoreBase;
	u8 bChannel = pImplChannel->bIndex;
	u16 wCsr = 0;

	DBG(2, "%p, pkt_sz %d, addr 0x%x, len %d, mode %d\n",
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

	return TRUE;
}

static int MGC_HsDmaProgramChannel(struct dma_channel * pChannel,
				  u16 wPacketSize, u8 bMode,
				  dma_addr_t dma_addr, u32 dwLength)
{
	MGC_HsDmaChannel *pImplChannel =
	    (MGC_HsDmaChannel *) pChannel->pPrivateData;

	DBG(2, "pkt_sz %d, dma_addr 0x%x length %d, mode %d\n",
	       wPacketSize, dma_addr, dwLength, bMode);

	BUG_ON(pChannel->bStatus != MGC_DMA_STATUS_FREE);

	pChannel->dwActualLength = 0L;
	pImplChannel->dwStartAddress = dma_addr;
	pImplChannel->dwCount = dwLength;

	pChannel->bStatus = MGC_DMA_STATUS_BUSY;

	if ((bMode == 1) && (dwLength >= wPacketSize)) {

#if 0
		/* mode 1 sends an extra IN token at the end of
		 * full packet transfer in host Rx
		 */
		if (dwLength % wPacketSize == 0)
			dwLength -= wPacketSize;

		/* mode 1 doesn't give an interrupt on short packet */
		configure_channel(pChannel, wPacketSize, 1, dma_addr,
				  dwLength & ~(wPacketSize - 1));
		/* the rest (<= pkt_size) will be transferred in mode 0 */
#endif

		configure_channel(pChannel, wPacketSize, 1, dma_addr,
				  dwLength);

	} else
		configure_channel(pChannel, wPacketSize, 0, dma_addr,
				  dwLength);

	return TRUE;
}

// REVISIT...
static int MGC_HsDmaAbortChannel(struct dma_channel *pChannel)
{
	clear_state(pChannel);
	pChannel->bStatus = MGC_DMA_STATUS_FREE;
	return 0;
}

static irqreturn_t hsdma_irq(int irq, void *pPrivateData)
{
	u8 bChannel;
	u16 wCsr;
	u32 dwAddress;
	MGC_HsDmaChannel *pImplChannel;
	MGC_HsDmaController *pController = pPrivateData;
	u8 *pBase = pController->pCoreBase;
	struct dma_channel *pChannel;
	u8 bIntr = musb_readb(pBase, MGC_O_HSDMA_INTR);

	if (!bIntr)
		return IRQ_NONE;

	for (bChannel = 0; bChannel < MGC_HSDMA_CHANNELS; bChannel++) {
		if (bIntr & (1 << bChannel)) {

			pImplChannel = (MGC_HsDmaChannel *)
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
						       MGC_HSDMA_CHANNEL_OFFSET
						       (bChannel,
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
#if 0
				if (pChannel->dwActualLength <
				    pImplChannel->dwCount) {
					/* mode 1 sends an extra IN request if
					the last packet is a complete packet */
					u16 newcsr = MGC_ReadCsr16(pBase,
							MGC_O_HDRC_RXCSR,
							pImplChannel->bEnd);
					newcsr &= ~(MGC_M_RXCSR_H_AUTOREQ |
						    MGC_M_RXCSR_H_REQPKT);
					MGC_WriteCsr16(pBase, MGC_O_HDRC_RXCSR,
						       pImplChannel->bEnd,
						       MGC_M_RXCSR_H_WZC_BITS |
								newcsr);

					configure_channel(pChannel,
						pImplChannel->wMaxPacketSize,
						0, dwAddress,
						pImplChannel->dwCount -
						    pChannel->dwActualLength);
				}
				else
#endif
				{
					pChannel->bStatus = MGC_DMA_STATUS_FREE;
					/* completed */
					musb_dma_completion(
						pController->pDmaPrivate,
						pImplChannel->bEnd,
						pImplChannel->bTransmit);
				}
			}
		}
	}
	return IRQ_HANDLED;
}

static void hsdma_controller_destroy(struct dma_controller *pController)
{
	MGC_HsDmaController *pHsController = pController->pPrivateData;

	if (pHsController) {
		pHsController->Controller.pPrivateData = NULL;
		kfree(pHsController);
	}
}

static struct dma_controller *
hsdma_controller_new(struct musb *pThis, void __iomem *pCoreBase)
{
	MGC_HsDmaController *pController;
	struct device *dev = pThis->controller;
	struct platform_device *pdev = to_platform_device(dev);
	int irq = platform_get_irq(pdev, 1);

	if (irq == 0) {
		dev_err(dev, "No DMA interrupt line!\n");
		return NULL;
	}

	if (!(pController = kzalloc(sizeof(MGC_HsDmaController), GFP_KERNEL)))
		return NULL;

	pController->bChannelCount = MGC_HSDMA_CHANNELS;
	pController->pDmaPrivate = pThis;
	pController->pCoreBase = pCoreBase;

	pController->Controller.pPrivateData = pController;
	pController->Controller.start = MGC_HsDmaStartController;
	pController->Controller.stop = MGC_HsDmaStopController;
	pController->Controller.channel_alloc = MGC_HsDmaAllocateChannel;
	pController->Controller.channel_release = MGC_HsDmaReleaseChannel;
	pController->Controller.channel_program = MGC_HsDmaProgramChannel;
	pController->Controller.channel_abort = MGC_HsDmaAbortChannel;

	if (request_irq(irq, hsdma_irq, SA_INTERRUPT,
			pThis->controller->bus_id, &pController->Controller)) {
		dev_err(dev, "request_irq %d failed!\n", irq);
		hsdma_controller_destroy(&pController->Controller);
		return NULL;
	}

	return &pController->Controller;
}

const struct dma_controller_factory dma_controller_factory = {
	.create =	hsdma_controller_new,
	.destroy =	hsdma_controller_destroy,
};
