/*
 * Copyright (C) 2005-2006 by Texas Instruments
 * Copyright (c) 2008, MontaVista Software, Inc. <source@mvista.com>
 *
 * This file implements a DMA interface using TI's CPPI 4.1 DMA.
 *
 * This program is free software; you can distribute it and/or modify it
 * under the terms of the GNU General Public License (Version 2) as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place - Suite 330, Boston MA 02111-1307, USA.
 *
 */

#include <linux/errno.h>
#include <linux/dma-mapping.h>

#include "cppi41.h"

#include "musb_core.h"
#include "musb_dma.h"
#include "cppi41_dma.h"

/* Configuration */
#define USB_CPPI41_DESC_SIZE_SHIFT 6
#define USB_CPPI41_DESC_ALIGN	(1 << USB_CPPI41_DESC_SIZE_SHIFT)
#define USB_CPPI41_CH_NUM_PD	64	/* 4K bulk data at full speed */
#define USB_CPPI41_MAX_PD	(USB_CPPI41_CH_NUM_PD * USB_CPPI41_NUM_CH)

#undef DEBUG_CPPI_TD
#undef USBDRV_DEBUG

#ifdef USBDRV_DEBUG
#define dprintk(x, ...) printk(x, ## __VA_ARGS__)
#else
#define dprintk(x, ...)
#endif

/*
 * Data structure definitions
 */

/*
 * USB Packet Descriptor
 */
struct usb_pkt_desc;

struct usb_pkt_desc {
	/* Hardware descriptor fields from this point */
	struct cppi41_host_pkt_desc hw_desc;
	/* Protocol specific data */
	dma_addr_t dma_addr;
	struct usb_pkt_desc *next_pd_ptr;
	u8 ch_num;
	u8 ep_num;
};

/**
 * struct cppi41_channel - DMA Channel Control Structure
 *
 * Using the same for Tx/Rx.
 */
struct cppi41_channel {
	struct dma_channel channel;

	struct cppi41_dma_ch_obj dma_ch_obj; /* DMA channel object */
	struct cppi41_queue src_queue;	/* Tx queue or Rx free descriptor/ */
					/* buffer queue */
	struct cppi41_queue_obj queue_obj; /* Tx queue object or Rx free */
					/* descriptor/buffer queue object */

	u32 tag_info;			/* Tx PD Tag Information field */

	/* Which direction of which endpoint? */
	struct musb_hw_ep *end_pt;
	u8 transmit;
	u8 ch_num;			/* Channel number of Tx/Rx 0..3 */

	/* DMA mode: "transparent", RNDIS, CDC, or Generic RNDIS */
	u8 dma_mode;
	u8 autoreq;

	/* Book keeping for the current transfer request */
	dma_addr_t start_addr;
	u32 length;
	u32 curr_offset;
	u16 pkt_size;
	u8  transfer_mode;
	u8  zlp_queued;
};

/**
 * struct cppi41 - CPPI 4.1 DMA Controller Object
 *
 * Encapsulates all book keeping and data structures pertaining to
 * the CPPI 1.4 DMA controller.
 */
struct cppi41 {
	struct dma_controller controller;
	struct musb *musb;

	struct cppi41_channel tx_cppi_ch[USB_CPPI41_NUM_CH];
	struct cppi41_channel rx_cppi_ch[USB_CPPI41_NUM_CH];

	struct usb_pkt_desc *pd_pool_head; /* Free PD pool head */
	dma_addr_t pd_mem_phys;		/* PD memory physical address */
	void *pd_mem;			/* PD memory pointer */
	u8 pd_mem_rgn;			/* PD memory region number */

	u16 teardownQNum;		/* Teardown completion queue number */
	struct cppi41_queue_obj queue_obj; /* Teardown completion queue */
					/* object */
	u32 pkt_info;			/* Tx PD Packet Information field */
};

#ifdef DEBUG_CPPI_TD
static void print_pd_list(struct usb_pkt_desc *pd_pool_head)
{
	struct usb_pkt_desc *curr_pd = pd_pool_head;
	int cnt = 0;

	while (curr_pd != NULL) {
		if (cnt % 8 == 0)
			dprintk("\n%02x ", cnt);
		cnt++;
		dprintk(" %p", curr_pd);
		curr_pd = curr_pd->next_pd_ptr;
	}
	dprintk("\n");
}
#endif

static struct usb_pkt_desc *usb_get_free_pd(struct cppi41 *cppi)
{
	struct usb_pkt_desc *free_pd = cppi->pd_pool_head;

	if (free_pd != NULL) {
		cppi->pd_pool_head = free_pd->next_pd_ptr;
		free_pd->next_pd_ptr = NULL;
	}
	return free_pd;
}

static void usb_put_free_pd(struct cppi41 *cppi, struct usb_pkt_desc *free_pd)
{
	free_pd->next_pd_ptr = cppi->pd_pool_head;
	cppi->pd_pool_head = free_pd;
}

/**
 * cppi41_controller_start - start DMA controller
 * @controller: the controller
 *
 * This function initializes the CPPI 4.1 Tx/Rx channels.
 */
static int __init cppi41_controller_start(struct dma_controller *controller)
{
	struct cppi41 *cppi;
	struct cppi41_channel *cppi_ch;
	void __iomem *reg_base;
	struct usb_pkt_desc *curr_pd;
	unsigned long pd_addr;
	int i;

	cppi = container_of(controller, struct cppi41, controller);

	/*
	 * TODO: We may need to check USB_CPPI41_MAX_PD here since CPPI 4.1
	 * requires the descriptor count to be a multiple of 2 ^ 5 (i.e. 32).
	 * Similarly, the descriptor size should also be a multiple of 32.
	 */

	/*
	 * Allocate free packet descriptor pool for all Tx/Rx endpoints --
	 * dma_alloc_coherent()  will return a page aligned address, so our
	 * alignment requirement will be honored.
	 */
	cppi->pd_mem = dma_alloc_coherent(cppi->musb->controller,
					  USB_CPPI41_MAX_PD *
					  USB_CPPI41_DESC_ALIGN,
					  &cppi->pd_mem_phys,
					  GFP_KERNEL | GFP_DMA);
	if (cppi->pd_mem == NULL) {
		DBG(1, "ERROR: packet descriptor memory allocation failed\n");
		return 0;
	}
	if (cppi41_mem_rgn_alloc(usb_cppi41_info.q_mgr, cppi->pd_mem_phys,
				 USB_CPPI41_DESC_SIZE_SHIFT,
				 get_count_order(USB_CPPI41_MAX_PD),
				 &cppi->pd_mem_rgn)) {
		DBG(1, "ERROR: queue manager memory region allocation "
		    "failed\n");
		goto free_pds;
	}

	/* Allocate the teardown completion queue */
	if (cppi41_queue_alloc(CPPI41_UNASSIGNED_QUEUE,
			       0, &cppi->teardownQNum)) {
		DBG(1, "ERROR: teardown completion queue allocation failed\n");
		goto free_mem_rgn;
	}
	DBG(4, "Allocated teardown completion queue %d in queue manager 0\n",
	    cppi->teardownQNum);

	if (cppi41_queue_init(&cppi->queue_obj, 0, cppi->teardownQNum)) {
		DBG(1, "ERROR: teardown completion queue initialization "
		    "failed\n");
		goto free_queue;
	}

	/*
	 * "Slice" PDs one-by-one from the big chunk and
	 * add them to the free pool.
	 */
	curr_pd = (struct usb_pkt_desc *)cppi->pd_mem;
	pd_addr = cppi->pd_mem_phys;
	for (i = 0; i < USB_CPPI41_MAX_PD; i++) {
		curr_pd->dma_addr = pd_addr;

		usb_put_free_pd(cppi, curr_pd);
		curr_pd = (struct usb_pkt_desc *)((char *)curr_pd +
						  USB_CPPI41_DESC_ALIGN);
		pd_addr += USB_CPPI41_DESC_ALIGN;
	}

	/* Configure the Tx channels */
	for (i = 0, cppi_ch = cppi->tx_cppi_ch;
	     i < ARRAY_SIZE(cppi->tx_cppi_ch); ++i, ++cppi_ch) {
		const struct cppi41_tx_ch *tx_info;

		memset(cppi_ch, 0, sizeof(struct cppi41_channel));
		cppi_ch->transmit = 1;
		cppi_ch->ch_num = i;
		cppi_ch->channel.private_data = cppi;

		/*
		 * Extract the CPPI 4.1 DMA Tx channel configuration and
		 * construct/store the Tx PD tag info field for later use...
		 */
		tx_info = cppi41_dma_block[usb_cppi41_info.dma_block].tx_ch_info
			  + usb_cppi41_info.ep_dma_ch[i];
		cppi_ch->src_queue = tx_info->tx_queue[0];
		cppi_ch->tag_info = (tx_info->port_num <<
				     CPPI41_SRC_TAG_PORT_NUM_SHIFT) |
				    (tx_info->ch_num <<
				     CPPI41_SRC_TAG_CH_NUM_SHIFT) |
				    (tx_info->sub_ch_num <<
				     CPPI41_SRC_TAG_SUB_CH_NUM_SHIFT);
	}

	/* Configure the Rx channels */
	for (i = 0, cppi_ch = cppi->rx_cppi_ch;
	     i < ARRAY_SIZE(cppi->rx_cppi_ch); ++i, ++cppi_ch) {
		memset(cppi_ch, 0, sizeof(struct cppi41_channel));
		cppi_ch->ch_num = i;
		cppi_ch->channel.private_data = cppi;
	}

	/* Construct/store Tx PD packet info field for later use */
	cppi->pkt_info = (CPPI41_PKT_TYPE_USB << CPPI41_PKT_TYPE_SHIFT) |
			 (CPPI41_RETURN_LINKED << CPPI41_RETURN_POLICY_SHIFT) |
			 (usb_cppi41_info.q_mgr << CPPI41_RETURN_QMGR_SHIFT) |
			 (usb_cppi41_info.tx_comp_q[0] <<
			  CPPI41_RETURN_QNUM_SHIFT);

	/* Do necessary configuartion in hardware to get started */
	reg_base = cppi->musb->ctrl_base;

	/* Disable auto request mode */
	musb_writel(reg_base, USB_AUTOREQ_REG, 0);

	/* Disable the CDC/RNDIS modes */
	musb_writel(reg_base, USB_MODE_REG, 0);

	return 1;

 free_queue:
	if (cppi41_queue_free(0, cppi->teardownQNum))
		DBG(1, "ERROR: failed to free teardown completion queue\n");

 free_mem_rgn:
	if (cppi41_mem_rgn_free(usb_cppi41_info.q_mgr, cppi->pd_mem_rgn))
		DBG(1, "ERROR: failed to free queue manager memory region\n");

 free_pds:
	dma_free_coherent(cppi->musb->controller,
			  USB_CPPI41_MAX_PD * USB_CPPI41_DESC_ALIGN,
			  cppi->pd_mem, cppi->pd_mem_phys);

	return 0;
}

/**
 * cppi41_controller_stop - stop DMA controller
 * @controller: the controller
 *
 * De-initialize the DMA Controller as necessary.
 */
static int cppi41_controller_stop(struct dma_controller *controller)
{
	struct cppi41 *cppi;
	void __iomem *reg_base;

	cppi = container_of(controller, struct cppi41, controller);

	/* Free the teardown completion queue */
	if (cppi41_queue_free(usb_cppi41_info.q_mgr, cppi->teardownQNum))
		DBG(1, "ERROR: failed to free teardown completion queue\n");

	/*
	 * Free the packet descriptor region allocated
	 * for all Tx/Rx channels.
	 */
	if (cppi41_mem_rgn_free(usb_cppi41_info.q_mgr, cppi->pd_mem_rgn))
		DBG(1, "ERROR: failed to free queue manager memory region\n");

	dma_free_coherent(cppi->musb->controller,
			  USB_CPPI41_MAX_PD * USB_CPPI41_DESC_ALIGN,
			  cppi->pd_mem, cppi->pd_mem_phys);

	reg_base = cppi->musb->ctrl_base;

	/* Disable auto request mode */
	musb_writel(reg_base, USB_AUTOREQ_REG, 0);

	/* Disable the CDC/RNDIS modes */
	musb_writel(reg_base, USB_MODE_REG, 0);

	return 1;
}

/**
 * cppi41_channel_alloc - allocate a CPPI channel for DMA.
 * @controller: the controller
 * @ep:		the endpoint
 * @is_tx:	1 for Tx channel, 0 for Rx channel
 *
 * With CPPI, channels are bound to each transfer direction of a non-control
 * endpoint, so allocating (and deallocating) is mostly a way to notice bad
 * housekeeping on the software side.  We assume the IRQs are always active.
 */
static struct dma_channel *cppi41_channel_alloc(struct dma_controller
						*controller,
						struct musb_hw_ep *ep, u8 is_tx)
{
	struct cppi41 *cppi;
	struct cppi41_channel  *cppi_ch;
	u32 ch_num, ep_num = ep->epnum;

	cppi = container_of(controller, struct cppi41, controller);

	/* Remember, ep_num: 1 .. Max_EP, and CPPI ch_num: 0 .. Max_EP - 1 */
	ch_num = ep_num - 1;

	if (ep_num > USB_CPPI41_NUM_CH) {
		DBG(1, "No %cx DMA channel for EP%d\n",
		    is_tx ? 'T' : 'R', ep_num);
		return NULL;
	}

	cppi_ch = (is_tx ? cppi->tx_cppi_ch : cppi->rx_cppi_ch) + ch_num;

	/* As of now, just return the corresponding CPPI 4.1 channel handle */
	if (is_tx) {
		/* Initialize the CPPI 4.1 Tx DMA channel */
		if (cppi41_tx_ch_init(&cppi_ch->dma_ch_obj,
				      usb_cppi41_info.dma_block,
				      usb_cppi41_info.ep_dma_ch[ch_num])) {
			DBG(1, "ERROR: cppi41_tx_ch_init failed for "
			    "channel %d\n", ch_num);
			return NULL;
		}
		/*
		 * Teardown descriptors will be pushed to the dedicated
		 * completion queue.
		 */
		cppi41_dma_ch_default_queue(&cppi_ch->dma_ch_obj,
					    0, cppi->teardownQNum);
	} else {
		struct cppi41_rx_ch_cfg rx_cfg;
		u8 q_mgr = usb_cppi41_info.q_mgr;
		int i;

		/* Initialize the CPPI 4.1 Rx DMA channel */
		if (cppi41_rx_ch_init(&cppi_ch->dma_ch_obj,
				      usb_cppi41_info.dma_block,
				      usb_cppi41_info.ep_dma_ch[ch_num])) {
			DBG(1, "ERROR: cppi41_rx_ch_init failed\n");
			return NULL;
		}

		if (cppi41_queue_alloc(CPPI41_FREE_DESC_BUF_QUEUE |
				       CPPI41_UNASSIGNED_QUEUE,
				       q_mgr, &cppi_ch->src_queue.q_num)) {
			DBG(1, "ERROR: cppi41_queue_alloc failed for "
			    "free descriptor/buffer queue\n");
			return NULL;
		}
		DBG(4, "Allocated free descriptor/buffer queue %d in "
		    "queue manager %d\n", cppi_ch->src_queue.q_num, q_mgr);

		rx_cfg.default_desc_type = cppi41_rx_host_desc;
		rx_cfg.sop_offset = 0;
		rx_cfg.retry_starved = 1;
		rx_cfg.rx_queue.q_mgr = cppi_ch->src_queue.q_mgr = q_mgr;
		rx_cfg.rx_queue.q_num = usb_cppi41_info.rx_comp_q[0];
		for (i = 0; i < 4; i++)
			rx_cfg.cfg.host_pkt.fdb_queue[i] = cppi_ch->src_queue;
		cppi41_rx_ch_configure(&cppi_ch->dma_ch_obj, &rx_cfg);
	}

	/* Initialize the CPPI 4.1 DMA source queue */
	if (cppi41_queue_init(&cppi_ch->queue_obj, cppi_ch->src_queue.q_mgr,
			       cppi_ch->src_queue.q_num)) {
		DBG(1, "ERROR: cppi41_queue_init failed for %s queue",
		    is_tx ? "Tx" : "Rx free descriptor/buffer");
		if (is_tx == 0 &&
		    cppi41_queue_free(cppi_ch->src_queue.q_mgr,
				      cppi_ch->src_queue.q_num))
			DBG(1, "ERROR: failed to free Rx descriptor/buffer "
			    "queue\n");
		 return NULL;
	}

	/* Enable the DMA channel */
	cppi41_dma_ch_enable(&cppi_ch->dma_ch_obj);

	if (cppi_ch->end_pt)
		DBG(1, "Re-allocating DMA %cx channel %d (%p)\n",
		    is_tx ? 'T' : 'R', ch_num, cppi_ch);

	cppi_ch->end_pt = ep;
	cppi_ch->ch_num = ch_num;
	cppi_ch->channel.status = MUSB_DMA_STATUS_FREE;

	DBG(4, "Allocated DMA %cx channel %d for EP%d\n", is_tx ? 'T' : 'R',
	    ch_num, ep_num);

	return &cppi_ch->channel;
}

/**
 * cppi41_channel_release - release a CPPI DMA channel
 * @channel: the channel
 */
static void cppi41_channel_release(struct dma_channel *channel)
{
	struct cppi41_channel *cppi_ch;

	/* REVISIT: for paranoia, check state and abort if needed... */
	cppi_ch = container_of(channel, struct cppi41_channel, channel);
	if (cppi_ch->end_pt == NULL)
		DBG(1, "Releasing idle DMA channel %p\n", cppi_ch);

	/* But for now, not its IRQ */
	cppi_ch->end_pt = NULL;
	channel->status = MUSB_DMA_STATUS_UNKNOWN;

	cppi41_dma_ch_disable(&cppi_ch->dma_ch_obj);

	/* De-allocate Rx free descriptior/buffer queue */
	if (cppi_ch->transmit == 0 &&
	    cppi41_queue_free(cppi_ch->src_queue.q_mgr,
			      cppi_ch->src_queue.q_num))
		DBG(1, "ERROR: failed to free Rx descriptor/buffer queue\n");
}

static void cppi41_mode_update(struct cppi41_channel *cppi_ch, u8 mode)
{
	if (mode != cppi_ch->dma_mode) {
		struct cppi41 *cppi = cppi_ch->channel.private_data;
		void *__iomem reg_base = cppi->musb->ctrl_base;
		u32 reg_val = musb_readl(reg_base, USB_MODE_REG);
		u8 ep_num = cppi_ch->ch_num + 1;

		if (cppi_ch->transmit) {
			reg_val &= ~USB_TX_MODE_MASK(ep_num);
			reg_val |= mode << USB_TX_MODE_SHIFT(ep_num);
		} else {
			reg_val &= ~USB_RX_MODE_MASK(ep_num);
			reg_val |= mode << USB_RX_MODE_SHIFT(ep_num);
		}
		musb_writel(reg_base, USB_MODE_REG, reg_val);
		cppi_ch->dma_mode = mode;
	}
}

/*
 * CPPI 4.1 Tx:
 * ============
 * Tx is a lot more reasonable than Rx: RNDIS mode seems to behave well except
 * how it handles the exactly-N-packets case. It appears that there's a hiccup
 * in that case (maybe the DMA completes before a ZLP gets written?) boiling
 * down to not being able to rely on the XFER DMA writing any terminating zero
 * length packet before the next transfer is started...
 *
 * The generic RNDIS mode does not have this misfeature, so we prefer using it
 * instead.  We then send the terminating ZLP *explictly* using DMA instead of
 * doing it by PIO after an IRQ.
 *
 */

/**
 * cppi41_next_tx_segment - DMA write for the next chunk of a buffer
 * @tx_ch:	Tx channel
 *
 * Context: controller IRQ-locked
 */
static unsigned cppi41_next_tx_segment(struct cppi41_channel *tx_ch)
{
	struct cppi41 *cppi = tx_ch->channel.private_data;
	struct usb_pkt_desc *curr_pd;
	u32 length = tx_ch->length - tx_ch->curr_offset;
	u32 pkt_size = tx_ch->pkt_size;
	unsigned num_pds, n;

	/*
	 * Tx can use the generic RNDIS mode where we can probably fit this
	 * transfer in one PD and one IRQ.  The only time we would NOT want
	 * to use it is when the hardware constraints prevent it...
	 */
	if ((pkt_size & 0x3f) == 0 && length > pkt_size) {
		num_pds  = 1;
		pkt_size = length;
		cppi41_mode_update(tx_ch, USB_GENERIC_RNDIS_MODE);
	} else {
		num_pds  = (length + pkt_size - 1) / pkt_size;
		cppi41_mode_update(tx_ch, USB_TRANSPARENT_MODE);
	}

	/*
	 * If length of transmit buffer is 0 or a multiple of the endpoint size,
	 * then send the zero length packet.
	 */
	if (!length || (tx_ch->transfer_mode && length % pkt_size == 0))
		num_pds++;

	DBG(4, "TX DMA%u, %s, maxpkt %u, %u PDs, addr %#x, len %u\n",
	    tx_ch->ch_num, tx_ch->dma_mode ? "accelerated" : "transparent",
	    pkt_size, num_pds, tx_ch->start_addr + tx_ch->curr_offset, length);

	for (n = 0; n < num_pds; n++) {
		struct cppi41_host_pkt_desc *hw_desc;

		/* Get Tx host packet descriptor from the free pool */
		curr_pd = usb_get_free_pd(cppi);
		if (curr_pd == NULL) {
			DBG(1, "No Tx PDs\n");
			break;
		}

		if (length < pkt_size)
			pkt_size = length;

		hw_desc = &curr_pd->hw_desc;
		hw_desc->desc_info = (CPPI41_DESC_TYPE_HOST <<
				      CPPI41_DESC_TYPE_SHIFT) | pkt_size;
		hw_desc->tag_info = tx_ch->tag_info;
		hw_desc->pkt_info = cppi->pkt_info;

		hw_desc->buf_ptr = tx_ch->start_addr + tx_ch->curr_offset;
		hw_desc->buf_len = pkt_size;
		hw_desc->next_desc_ptr = 0;

		curr_pd->ch_num = tx_ch->ch_num;
		curr_pd->ep_num = tx_ch->end_pt->epnum;

		tx_ch->curr_offset += pkt_size;
		length -= pkt_size;

		if (pkt_size == 0)
			tx_ch->zlp_queued = 1;

		DBG(5, "TX PD %p: buf %08x, len %08x, pkt info %08x\n", curr_pd,
		    hw_desc->buf_ptr, hw_desc->buf_len, hw_desc->pkt_info);

		cppi41_queue_push(&tx_ch->queue_obj, curr_pd->dma_addr,
				  USB_CPPI41_DESC_ALIGN, pkt_size);
	}

	return n;
}

static void cppi41_autoreq_update(struct cppi41_channel *rx_ch, u8 autoreq)
{
	struct cppi41 *cppi = rx_ch->channel.private_data;

	if (is_host_active(cppi->musb) &&
	    autoreq != rx_ch->autoreq) {
		void *__iomem reg_base = cppi->musb->ctrl_base;
		u32 reg_val = musb_readl(reg_base, USB_AUTOREQ_REG);
		u8 ep_num = rx_ch->ch_num + 1;

		reg_val &= ~USB_RX_AUTOREQ_MASK(ep_num);
		reg_val |= autoreq << USB_RX_AUTOREQ_SHIFT(ep_num);

		musb_writel(reg_base, USB_AUTOREQ_REG, reg_val);
		rx_ch->autoreq = autoreq;
	}
}

static void cppi41_set_ep_size(struct cppi41_channel *rx_ch, u32 pkt_size)
{
	struct cppi41 *cppi = rx_ch->channel.private_data;
	void *__iomem reg_base = cppi->musb->ctrl_base;
	u8 ep_num = rx_ch->ch_num + 1;

	musb_writel(reg_base, USB_GENERIC_RNDIS_EP_SIZE_REG(ep_num), pkt_size);
}

/*
 * CPPI 4.1 Rx:
 * ============
 * Consider a 1KB bulk Rx buffer in two scenarios: (a) it's fed two 300 byte
 * packets back-to-back, and (b) it's fed two 512 byte packets back-to-back.
 * (Full speed transfers have similar scenarios.)
 *
 * The correct behavior for Linux is that (a) fills the buffer with 300 bytes,
 * and the next packet goes into a buffer that's queued later; while (b) fills
 * the buffer with 1024 bytes.  How to do that with accelerated DMA modes?
 *
 * Rx queues in RNDIS mode (one single BD) handle (a) correctly but (b) loses
 * BADLY because nothing (!) happens when that second packet fills the buffer,
 * much less when a third one arrives -- which makes it not a "true" RNDIS mode.
 * In the RNDIS protocol short-packet termination is optional, and it's fine if
 * the peripherals (not hosts!) pad the messages out to end of buffer. Standard
 * PCI host controller DMA descriptors implement that mode by default... which
 * is no accident.
 *
 * Generic RNDIS mode is the only way to reliably make both cases work.  This
 * mode is identical to the "normal" RNDIS mode except for the case where the
 * last packet of the segment matches the max USB packet size -- in this case,
 * the packet will be closed when a value (0x10000 max) in the Generic RNDIS
 * EP Size register is reached.  This mode will work for the network drivers
 * (CDC/RNDIS) as well as for the mass storage drivers where there is no short
 * packet.
 *
 * BUT we can only use non-transparent modes when USB packet size is a multiple
 * of 64 bytes. Let's see what happens when  this is not the case...
 *
 * Rx queues (2 BDs with 512 bytes each) have converse problems to RNDIS mode:
 * (b) is handled right but (a) loses badly.  DMA doesn't stop after receiving
 * a short packet and processes both of those PDs; so both packets are loaded
 * into the buffer (with 212 byte gap between them), and the next buffer queued
 * will NOT get its 300 bytes of data.  Even in the case when there should be
 * no short packets (URB_SHORT_NOT_OK is set), queueing several packets in the
 * host mode doesn't win us anything since we have to manually "prod" the Rx
 * process after each packet is received by setting ReqPkt bit in endpoint's
 * RXCSR; in the peripheral mode without short packets, queueing could be used
 * BUT we'll have to *teardown* the channel if a short packet still arrives in
 * the peripheral mode, and to "collect" the left-over packet descriptors from
 * the free descriptor/buffer queue in both cases...
 *
 * One BD at a time is the only way to make make both cases work reliably, with
 * software handling both cases correctly, at the significant penalty of needing
 * an IRQ per packet.  (The lack of I/O overlap can be slightly ameliorated by
 * enabling double buffering.)
 *
 * There seems to be no way to identify for sure the cases where the CDC mode
 * is appropriate...
 *
 */

/**
 * cppi41_next_rx_segment - DMA read for the next chunk of a buffer
 * @rx_ch:	Rx channel
 *
 * Context: controller IRQ-locked
 *
 * NOTE: In the transparent mode, we have to queue one packet at a time since:
 *	 - we must avoid starting reception of another packet after receiving
 *	   a short packet;
 *	 - in host mode we have to set ReqPkt bit in the endpoint's RXCSR after
 *	   receiving each packet but the last one... ugly!
 */
static unsigned cppi41_next_rx_segment(struct cppi41_channel *rx_ch)
{
	struct cppi41 *cppi = rx_ch->channel.private_data;
	struct usb_pkt_desc *curr_pd;
	struct cppi41_host_pkt_desc *hw_desc;
	u32 length = rx_ch->length - rx_ch->curr_offset;
	u32 pkt_size = rx_ch->pkt_size;

	/*
	 * Rx can use the generic RNDIS mode where we can probably fit this
	 * transfer in one PD and one IRQ (or two with a short packet).
	 */
	if ((pkt_size & 0x3f) == 0 && length >= 2 * pkt_size) {
		cppi41_mode_update(rx_ch, USB_GENERIC_RNDIS_MODE);
		cppi41_autoreq_update(rx_ch, USB_AUTOREQ_ALL_BUT_EOP);

		if (likely(length < 0x10000))
			pkt_size = length - length % pkt_size;
		else
			pkt_size = 0x10000;
		cppi41_set_ep_size(rx_ch, pkt_size);
	} else {
		cppi41_mode_update(rx_ch, USB_TRANSPARENT_MODE);
		cppi41_autoreq_update(rx_ch, USB_NO_AUTOREQ);
	}

	DBG(4, "RX DMA%u, %s, maxpkt %u, addr %#x, rec'd %u/%u\n",
	    rx_ch->ch_num, rx_ch->dma_mode ? "accelerated" : "transparent",
	    pkt_size, rx_ch->start_addr + rx_ch->curr_offset,
	    rx_ch->curr_offset, rx_ch->length);

	/* Get Rx packet descriptor from the free pool */
	curr_pd = usb_get_free_pd(cppi);
	if (curr_pd == NULL) {
		/* Shouldn't ever happen! */
		DBG(4, "No Rx PDs\n");
		return 0;
	}

	/*
	 * HCD arranged ReqPkt for the first packet.
	 * We arrange it for all but the last one.
	 */
	if (is_host_active(cppi->musb) && rx_ch->channel.actual_len) {
		void __iomem *epio = rx_ch->end_pt->regs;
		u16 csr = musb_readw(epio, MUSB_RXCSR);

		csr |= MUSB_RXCSR_H_REQPKT | MUSB_RXCSR_H_WZC_BITS;
		musb_writew(epio, MUSB_RXCSR, csr);
	}

	if (length < pkt_size)
		pkt_size = length;

	hw_desc = &curr_pd->hw_desc;
	hw_desc->orig_buf_ptr = rx_ch->start_addr + rx_ch->curr_offset;
	hw_desc->orig_buf_len = pkt_size;

	curr_pd->ch_num = rx_ch->ch_num;
	curr_pd->ep_num = rx_ch->end_pt->epnum;

	rx_ch->curr_offset += pkt_size;

	/*
	 * Push the free Rx packet descriptor
	 * to the free descriptor/buffer queue.
	 */
	cppi41_queue_push(&rx_ch->queue_obj, curr_pd->dma_addr,
		USB_CPPI41_DESC_ALIGN, 0);

	return 1;
}

/**
 * cppi41_channel_program - program channel for data transfer
 * @channel:	the channel
 * @maxpacket:	max packet size
 * @mode:	for Rx, 1 unless the USB protocol driver promised to treat
 *		all short reads as errors and kick in high level fault recovery;
 *		for Tx, 0 unless the protocol driver _requires_ short-packet
 *		termination mode
 * @dma_addr:	DMA address of buffer
 * @length:	length of buffer
 *
 * Context: controller IRQ-locked
 */
static int cppi41_channel_program(struct dma_channel *channel,	u16 maxpacket,
				  u8 mode, dma_addr_t dma_addr, u32 length)
{
	struct cppi41_channel *cppi_ch;
	unsigned queued;

	cppi_ch = container_of(channel, struct cppi41_channel, channel);

	switch (channel->status) {
	case MUSB_DMA_STATUS_BUS_ABORT:
	case MUSB_DMA_STATUS_CORE_ABORT:
		/* Fault IRQ handler should have handled cleanup */
		WARNING("%cx DMA%d not cleaned up after abort!\n",
			cppi_ch->transmit ? 'T' : 'R', cppi_ch->ch_num);
		break;
	case MUSB_DMA_STATUS_BUSY:
		WARNING("Program active channel? %cx DMA%d\n",
			cppi_ch->transmit ? 'T' : 'R', cppi_ch->ch_num);
		break;
	case MUSB_DMA_STATUS_UNKNOWN:
		DBG(1, "%cx DMA%d not allocated!\n",
		    cppi_ch->transmit ? 'T' : 'R', cppi_ch->ch_num);
		return 0;
	case MUSB_DMA_STATUS_FREE:
		break;
	}

	channel->status = MUSB_DMA_STATUS_BUSY;

	/* Set the transfer parameters, then queue up the first segment */
	cppi_ch->start_addr = dma_addr;
	cppi_ch->curr_offset = 0;
	cppi_ch->pkt_size = maxpacket;
	cppi_ch->length = length;
	cppi_ch->transfer_mode = mode;
	cppi_ch->zlp_queued = 0;

	/* Tx or Rx channel? */
	if (cppi_ch->transmit)
		queued = cppi41_next_tx_segment(cppi_ch);
	else
		queued = cppi41_next_rx_segment(cppi_ch);

	return	queued > 0;
}

static struct usb_pkt_desc *usb_get_pd_ptr(struct cppi41 *cppi,
					   unsigned long pd_addr)
{
	if (pd_addr >= cppi->pd_mem_phys && pd_addr < cppi->pd_mem_phys +
	    USB_CPPI41_MAX_PD * USB_CPPI41_DESC_ALIGN)
		return pd_addr - cppi->pd_mem_phys + cppi->pd_mem;
	else
		return NULL;
}

static int usb_check_teardown(struct cppi41_channel *cppi_ch,
			      unsigned long pd_addr)
{
	u32 info;

	if (cppi41_get_teardown_info(pd_addr, &info)) {
		DBG(1, "ERROR: not a teardown descriptor\n");
		return 0;
	}

	if ((info & CPPI41_TEARDOWN_TX_RX_MASK) ==
	    (!cppi_ch->transmit << CPPI41_TEARDOWN_TX_RX_SHIFT) &&
	    (info & CPPI41_TEARDOWN_DMA_NUM_MASK) ==
	    (usb_cppi41_info.dma_block << CPPI41_TEARDOWN_DMA_NUM_SHIFT) &&
	    (info & CPPI41_TEARDOWN_CHAN_NUM_MASK) ==
	    (usb_cppi41_info.ep_dma_ch[cppi_ch->ch_num] <<
	     CPPI41_TEARDOWN_CHAN_NUM_SHIFT))
		return 1;

	DBG(1, "ERROR: unexpected values in teardown descriptor\n");
	return 0;
}

/*
 * We can't handle the channel teardown via the default completion queue in
 * context of the controller IRQ-locked, so we use the dedicated teardown
 * completion queue which we can just poll for a teardown descriptor, not
 * interfering with the Tx completion queue processing.
 */
static void usb_tx_ch_teardown(struct cppi41_channel *tx_ch)
{
	struct cppi41 *cppi = tx_ch->channel.private_data;
	unsigned long pd_addr;

	/* Initiate teardown for Tx DMA channel */
	cppi41_dma_ch_teardown(&tx_ch->dma_ch_obj);

	do {
		/* Wait for a descriptor to be queued and pop it... */
		do {
			pd_addr = cppi41_queue_pop(&cppi->queue_obj);
		} while (!pd_addr);

		dprintk("Descriptor (%08lx) popped from teardown completion "
			"queue\n", pd_addr);
	} while (!usb_check_teardown(tx_ch, pd_addr));
}

/*
 * For Rx DMA channels, the situation is more complex: there's only a single
 * completion queue for all our needs, so we have to temporarily redirect the
 * completed descriptors to our teardown completion queue, with a possibility
 * of a completed packet landing there as well...
 */
static void usb_rx_ch_teardown(struct cppi41_channel *rx_ch)
{
	struct cppi41 *cppi = rx_ch->channel.private_data;

	cppi41_dma_ch_default_queue(&rx_ch->dma_ch_obj, 0, cppi->teardownQNum);

	/* Initiate teardown for Rx DMA channel */
	cppi41_dma_ch_teardown(&rx_ch->dma_ch_obj);

	while (1) {
		struct usb_pkt_desc *curr_pd;
		unsigned long pd_addr;

		/* Wait for a descriptor to be queued and pop it... */
		do {
			pd_addr = cppi41_queue_pop(&cppi->queue_obj);
		} while (!pd_addr);

		dprintk("Descriptor (%08lx) popped from teardown completion "
			"queue\n", pd_addr);

		/*
		 * We might have popped a completed Rx PD, so check if the
		 * physical address is within the PD region first.  If it's
		 * not the case, it must be a teardown descriptor...
		 * */
		curr_pd = usb_get_pd_ptr(cppi, pd_addr);
		if (curr_pd == NULL) {
			if (usb_check_teardown(rx_ch, pd_addr))
				break;
			continue;
		}

		/* Paranoia: check if PD is from the right channel... */
		if (curr_pd->ch_num != rx_ch->ch_num) {
			ERR("Unexpected channel %d in Rx PD\n",
			    curr_pd->ch_num);
			continue;
		}

		/* Extract the buffer length from the completed PD */
		rx_ch->channel.actual_len += curr_pd->hw_desc.buf_len;

		/*
		 * Return Rx PDs to the software list --
		 * this is protected by critical section.
		 */
		usb_put_free_pd(cppi, curr_pd);
	}

	/* Now restore the default Rx completion queue... */
	cppi41_dma_ch_default_queue(&rx_ch->dma_ch_obj, usb_cppi41_info.q_mgr,
				    usb_cppi41_info.rx_comp_q[0]);
}

/*
 * cppi41_channel_abort
 *
 * Context: controller IRQ-locked, endpoint selected.
 */
static int cppi41_channel_abort(struct dma_channel *channel)
{
	struct cppi41 *cppi;
	struct cppi41_channel *cppi_ch;
	struct musb  *musb;
	void __iomem *reg_base, *epio;
	unsigned long pd_addr;
	u32 csr, td_reg;
	u8 ch_num, ep_num;

	cppi_ch = container_of(channel, struct cppi41_channel, channel);
	ch_num = cppi_ch->ch_num;

	switch (channel->status) {
	case MUSB_DMA_STATUS_BUS_ABORT:
	case MUSB_DMA_STATUS_CORE_ABORT:
		/* From Rx or Tx fault IRQ handler */
	case MUSB_DMA_STATUS_BUSY:
		/* The hardware needs shutting down... */
		dprintk("%s: DMA busy, status = %x\n",
			__func__, channel->status);
		break;
	case MUSB_DMA_STATUS_UNKNOWN:
		DBG(1, "%cx DMA%d not allocated\n",
		    cppi_ch->transmit ? 'T' : 'R', ch_num);
		/* FALLTHROUGH */
	case MUSB_DMA_STATUS_FREE:
		return 0;
	}

	cppi = cppi_ch->channel.private_data;
	musb = cppi->musb;
	reg_base = musb->ctrl_base;
	epio = cppi_ch->end_pt->regs;
	ep_num = ch_num + 1;

#ifdef DEBUG_CPPI_TD
	printk("Before teardown:");
	print_pd_list(cppi->pd_pool_head);
#endif

	if (cppi_ch->transmit) {
		dprintk("Tx channel teardown, cppi_ch = %p\n", cppi_ch);

		/* Tear down Tx DMA channel */
		usb_tx_ch_teardown(cppi_ch);

		/* Issue CPPI FIFO teardown for Tx channel */
		td_reg  = musb_readl(reg_base, USB_TEARDOWN_REG);
		td_reg |= USB_TX_TDOWN_MASK(ep_num);
		musb_writel(reg_base, USB_TEARDOWN_REG, td_reg);

		/* Flush FIFO of the endpoint */
		csr  = musb_readw(epio, MUSB_TXCSR);
		csr |= MUSB_TXCSR_FLUSHFIFO | MUSB_TXCSR_H_WZC_BITS;
		musb_writew(epio, MUSB_TXCSR, csr);
	} else { /* Rx */
		dprintk("Rx channel teardown, cppi_ch = %p\n", cppi_ch);

		/* Flush FIFO of the endpoint */
		csr  = musb_readw(epio, MUSB_RXCSR);
		csr |= MUSB_RXCSR_FLUSHFIFO | MUSB_RXCSR_H_WZC_BITS;
		musb_writew(epio, MUSB_RXCSR, csr);

		/* Issue CPPI FIFO teardown for Rx channel */
		td_reg  = musb_readl(reg_base, USB_TEARDOWN_REG);
		td_reg |= USB_RX_TDOWN_MASK(ep_num);
		musb_writel(reg_base, USB_TEARDOWN_REG, td_reg);

		/* Tear down Rx DMA channel */
		usb_rx_ch_teardown(cppi_ch);

		/*
		 * NOTE: docs don't guarantee any of this works...  we expect
		 * that if the USB core stops telling the CPPI core to pull
		 * more data from it, then it'll be safe to flush current Rx
		 * DMA state iff any pending FIFO transfer is done.
		 */

		/* For host, ensure ReqPkt is never set again */
		cppi41_autoreq_update(cppi_ch, USB_NO_AUTOREQ);

		/* For host, clear (just) ReqPkt at end of current packet(s) */
		if (is_host_active(cppi->musb))
			csr &= ~MUSB_RXCSR_H_REQPKT;
		csr |= MUSB_RXCSR_H_WZC_BITS;

		/* Clear DMA enable */
		csr &= ~MUSB_RXCSR_DMAENAB;
		musb_writew(epio, MUSB_RXCSR, csr);

		/* Flush the FIFO of endpoint once again */
		csr  = musb_readw(epio, MUSB_RXCSR);
		csr |= MUSB_RXCSR_FLUSHFIFO | MUSB_RXCSR_H_WZC_BITS;
		musb_writew(epio, MUSB_RXCSR, csr);

		udelay(50);
	}

	/*
	 * There might be PDs in the Rx/Tx source queue that were not consumed
	 * by the DMA controller -- they need to be recycled properly.
	 */
	while ((pd_addr = cppi41_queue_pop(&cppi_ch->queue_obj)) != 0) {
		struct usb_pkt_desc *curr_pd;

		curr_pd = usb_get_pd_ptr(cppi, pd_addr);
		if (curr_pd == NULL) {
			ERR("Invalid PD popped from source queue\n");
			continue;
		}

		/*
		 * Return Rx/Tx PDs to the software list --
		 * this is protected by critical section.
		 */
		dprintk("Returning PD %p to the free PD list\n", curr_pd);
		usb_put_free_pd(cppi, curr_pd);
	}

#ifdef DEBUG_CPPI_TD
	printk("After teardown:");
	print_pd_list(cppi->pd_pool_head);
#endif

	/* Re-enable the DMA channel */
	cppi41_dma_ch_enable(&cppi_ch->dma_ch_obj);

	channel->status = MUSB_DMA_STATUS_FREE;

	return 0;
}

/**
 * dma_controller_create - instantiate an object representing DMA controller.
 */
struct dma_controller * __init dma_controller_create(struct musb  *musb,
						     void __iomem *mregs)
{
	struct cppi41 *cppi;

	cppi = kzalloc(sizeof *cppi, GFP_KERNEL);
	if (!cppi)
		return NULL;

	/* Initialize the CPPI 4.1 DMA controller structure */
	cppi->musb  = musb;
	cppi->controller.start = cppi41_controller_start;
	cppi->controller.stop  = cppi41_controller_stop;
	cppi->controller.channel_alloc = cppi41_channel_alloc;
	cppi->controller.channel_release = cppi41_channel_release;
	cppi->controller.channel_program = cppi41_channel_program;
	cppi->controller.channel_abort = cppi41_channel_abort;

	return &cppi->controller;
}

/**
 * dma_controller_destroy - destroy a previously instantiated DMA controller
 * @controller: the controller
 */
void dma_controller_destroy(struct dma_controller *controller)
{
	struct cppi41 *cppi;

	cppi = container_of(controller, struct cppi41, controller);

	/* Free the CPPI object */
	kfree(cppi);
}

static void usb_process_tx_queue(struct cppi41 *cppi, unsigned index)
{
	struct cppi41_queue_obj tx_queue_obj;
	unsigned long pd_addr;

	if (cppi41_queue_init(&tx_queue_obj, usb_cppi41_info.q_mgr,
			      usb_cppi41_info.tx_comp_q[index])) {
		DBG(1, "ERROR: cppi41_queue_init failed for "
		    "Tx completion queue");
		return;
	}

	while ((pd_addr = cppi41_queue_pop(&tx_queue_obj)) != 0) {
		struct usb_pkt_desc *curr_pd;
		struct cppi41_channel *tx_ch;
		u8 ch_num, ep_num;
		u32 length;

		curr_pd = usb_get_pd_ptr(cppi, pd_addr);
		if (curr_pd == NULL) {
			ERR("Invalid PD popped from Tx completion queue\n");
			continue;
		}

		/* Extract the data from received packet descriptor */
		ch_num = curr_pd->ch_num;
		ep_num = curr_pd->ep_num;
		length = curr_pd->hw_desc.buf_len;

		tx_ch = &cppi->tx_cppi_ch[ch_num];
		tx_ch->channel.actual_len += length;

		/*
		 * Return Tx PD to the software list --
		 * this is protected by critical section
		 */
		usb_put_free_pd(cppi, curr_pd);

		if ((tx_ch->curr_offset < tx_ch->length) ||
		    (tx_ch->transfer_mode && !tx_ch->zlp_queued))
			cppi41_next_tx_segment(tx_ch);
		else if (tx_ch->channel.actual_len >= tx_ch->length) {
			tx_ch->channel.status = MUSB_DMA_STATUS_FREE;

			/* Tx completion routine callback */
			musb_dma_completion(cppi->musb, ep_num, 1);
		}
	}
}

static void usb_process_rx_queue(struct cppi41 *cppi, unsigned index)
{
	struct cppi41_queue_obj rx_queue_obj;
	unsigned long pd_addr;

	if (cppi41_queue_init(&rx_queue_obj, usb_cppi41_info.q_mgr,
			      usb_cppi41_info.rx_comp_q[index])) {
		DBG(1, "ERROR: cppi41_queue_init failed for Rx queue\n");
		return;
	}

	while ((pd_addr = cppi41_queue_pop(&rx_queue_obj)) != 0) {
		struct usb_pkt_desc *curr_pd;
		struct cppi41_channel *rx_ch;
		u8 ch_num, ep_num;
		u32 length;

		curr_pd = usb_get_pd_ptr(cppi, pd_addr);
		if (curr_pd == NULL) {
			ERR("Invalid PD popped from Rx completion queue\n");
			continue;
		}

		/* Extract the data from received packet descriptor */
		ch_num = curr_pd->ch_num;
		ep_num = curr_pd->ep_num;
		length = curr_pd->hw_desc.buf_len;

		rx_ch = &cppi->rx_cppi_ch[ch_num];
		rx_ch->channel.actual_len += length;

		/*
		 * Return Rx PD to the software list --
		 * this is protected by critical section
		 */
		usb_put_free_pd(cppi, curr_pd);

		if (unlikely(rx_ch->channel.actual_len >= rx_ch->length ||
			     length < curr_pd->hw_desc.orig_buf_len)) {
			rx_ch->channel.status = MUSB_DMA_STATUS_FREE;

			/* Rx completion routine callback */
			musb_dma_completion(cppi->musb, ep_num, 0);
		} else
			cppi41_next_rx_segment(rx_ch);
	}
}

/*
 * cppi41_completion - handle interrupts from the Tx/Rx completion queues
 *
 * NOTE: since we have to manually prod the Rx process in the transparent mode,
 *	 we certainly want to handle the Rx queues first.
 */
void cppi41_completion(struct musb *musb, u32 rx, u32 tx)
{
	struct cppi41 *cppi;
	unsigned index;

	cppi = container_of(musb->dma_controller, struct cppi41, controller);

	/* Process packet descriptors from the Rx queues */
	for (index = 0; rx != 0; rx >>= 1, index++)
		if (rx & 1)
			usb_process_rx_queue(cppi, index);

	/* Process packet descriptors from the Tx completion queues */
	for (index = 0; tx != 0; tx >>= 1, index++)
		if (tx & 1)
			usb_process_tx_queue(cppi, index);
}
