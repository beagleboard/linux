/* Copyright (C) 2005-2006 by Texas Instruments */

#ifndef _CPPI_DMA_H_
#define _CPPI_DMA_H_

#include <linux/slab.h>
#include <linux/list.h>
#include <linux/smp_lock.h>
#include <linux/errno.h>
#include <linux/dmapool.h>

#include "dma.h"
#include "musbdefs.h"
#include "davinci.h"


/* hOptions bit masks for CPPI BDs */
#define CPPI_SOP_SET	((u32)(1 << 31))
#define CPPI_EOP_SET	((u32)(1 << 30))
#define CPPI_OWN_SET	((u32)(1 << 29))	/* owned by cppi */
#define CPPI_EOQ_MASK	((u32)(1 << 28))
#define CPPI_ZERO_SET	((u32)(1 << 23))	/* rx saw zlp; tx issues one */
#define CPPI_RXABT_MASK	((u32)(1 << 19))	/* need more rx buffers */

#define CPPI_RECV_PKTLEN_MASK 0xFFFF
#define CPPI_BUFFER_LEN_MASK 0xFFFF

#define CPPI_TEAR_READY ((u32)(1 << 31))

/* CPPI data structure definitions */

#define	CPPI_DESCRIPTOR_ALIGN	16	// bytes; 5-dec docs say 4-byte align

struct cppi_descriptor {
	/* Hardware Overlay */
	u32 hNext;     /**< Next(hardware) Buffer Descriptor Pointer */
	u32 buffPtr;	   /**<Buffer Pointer (dma_addr_t) */
	u32 bOffBLen;	    /**<Buffer_offset16,buffer_length16 */
	u32 hOptions;	    /**<Option fields for SOP,EOP etc*/

	struct cppi_descriptor *next;
	dma_addr_t dma;		/* address of this descriptor */

	/* for Rx Desc, track original Buffer len to detect short packets */
	u32 enqBuffLen;
} __attribute__ ((aligned(CPPI_DESCRIPTOR_ALIGN)));


/* forward declaration for CppiDmaController structure */
struct cppi;

/**
 *  Channel Control Structure
 *
 * CPPI  Channel Control structure. Using he same for Tx/Rx. If need be
 * derive out of this later.
 */
struct cppi_channel {
	/* First field must be dma_channel for easy type casting
	 * FIXME just use container_of() and be typesafe instead!
	 */
	struct dma_channel Channel;

	/* back pointer to the Dma Controller structure */
	struct cppi		*pController;

	/* which direction of which endpoint? */
	struct musb_hw_ep	*pEndPt;
	u8			bTransmit;
	u8			chNo;

	/* DMA modes:  RNDIS or "transparent" */
	u8			bLastModeRndis;

	/* book keeping for current transfer request */
	dma_addr_t		startAddr;
	u32			transferSize;
	u32			pktSize;
	u32			currOffset;	/* requested segments */
	u32			actualLen;	/* completed (Channel.actual) */

	void __iomem		*stateRam;	/* CPPI state */

	/* BD management fields */
	struct cppi_descriptor	*bdPoolHead;
	struct cppi_descriptor	*activeQueueHead;
	struct cppi_descriptor	*activeQueueTail;
	struct cppi_descriptor	*lastHwBDProcessed;

	/* use tx_complete in host role to track endpoints waiting for
	 * FIFONOTEMPTY to clear.
	 */
	struct list_head	tx_complete;
};

/**
 *  CPPI Dma Controller Object
 *
 *  CPPI Dma controller object.Encapsulates all bookeeping and Data
 *  structures pertaining to the CPPI Dma Controller.
 */
struct cppi {
	struct dma_controller		Controller;
	struct musb			*musb;
	void __iomem			*pCoreBase;

	struct cppi_channel		txCppi[MUSB_C_NUM_EPT - 1];
	struct cppi_channel		rxCppi[MUSB_C_NUM_EPR - 1];

	struct dma_pool			*pool;

	struct list_head		tx_complete;
};

/* irq handling hook */
extern void cppi_completion(struct musb *, u32 rx, u32 tx);

#endif				/* end of ifndef _CPPI_DMA_H_ */
