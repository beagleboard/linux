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
#include <linux/module.h>

#include "cppi41.h"

#include "musb_core.h"
#include "musb_dma.h"
#include "cppi41_dma.h"

/* Configuration */
#define USB_CPPI41_DESC_SIZE_SHIFT 6
#define USB_CPPI41_DESC_ALIGN	(1 << USB_CPPI41_DESC_SIZE_SHIFT)
#define USB_CPPI41_CH_NUM_PD	128	/* 4K bulk data at full speed */
#define USB_CPPI41_MAX_PD	(USB_CPPI41_CH_NUM_PD * (USB_CPPI41_NUM_CH+1))

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
	struct cppi41_host_pkt_desc hw_desc;	/* 40 bytes */
	/* Protocol specific data */
	dma_addr_t dma_addr;			/* offs:44 byte */
	struct usb_pkt_desc *next_pd_ptr;	/* offs:48 byte*/
	u8 ch_num;
	u8 ep_num;
	u8 eop;
	u8 res1;				/* offs:52 */
	u8 res2[12];				/* offs:64 */
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
	u8  inf_mode;
	u8  tx_complete;
	u8  hb_mult;
	u8  txf_complete;
	u8  txfifo_intr_enable;
	u8  count;
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
	struct work_struct      txdma_work;

	struct usb_pkt_desc *pd_pool_head; /* Free PD pool head */
	dma_addr_t pd_mem_phys;		/* PD memory physical address */
	void *pd_mem;			/* PD memory pointer */
	u8 pd_mem_rgn;			/* PD memory region number */

	u16 teardownQNum;		/* Teardown completion queue number */
	struct cppi41_queue_obj queue_obj; /* Teardown completion queue */
					/* object */
	u32 pkt_info;			/* Tx PD Packet Information field */
	struct usb_cppi41_info *cppi_info; /* cppi channel information */
	u8 en_bd_intr;			/* enable bd interrupt */
	u32 automode_reg_offs;		/* USB_AUTOREQ_REG offset */
	u32 teardown_reg_offs;		/* USB_TEARDOWN_REG offset */
	u32 bd_size;
	u8  inf_mode;
	u8  txfifo_intr_enable;		/* txfifo empty interrupt logic */
};

struct usb_cppi41_info usb_cppi41_info[2];
EXPORT_SYMBOL(usb_cppi41_info);

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
static int __devinit cppi41_controller_start(struct dma_controller *controller)
{
	struct cppi41 *cppi;
	struct cppi41_channel *cppi_ch;
	void __iomem *reg_base;
	struct usb_pkt_desc *curr_pd;
	unsigned long pd_addr;
	int i;
	struct usb_cppi41_info *cppi_info;
	struct musb *musb;

	cppi = container_of(controller, struct cppi41, controller);
	cppi_info = cppi->cppi_info;
	musb = cppi->musb;

	if (cpu_is_ti816x() || cpu_is_am33xx()) {
		cppi->automode_reg_offs = TI81XX_USB_AUTOREQ_REG;
		cppi->teardown_reg_offs = TI81XX_USB_TEARDOWN_REG;
	} else {
		cppi->automode_reg_offs = USB_AUTOREQ_REG;
		cppi->teardown_reg_offs = USB_TEARDOWN_REG;
	}

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
	cppi->bd_size = USB_CPPI41_MAX_PD * sizeof(struct usb_pkt_desc);
	cppi->pd_mem = dma_alloc_coherent(cppi->musb->controller,
					  cppi->bd_size,
					  &cppi->pd_mem_phys,
					  GFP_KERNEL | GFP_DMA);
	if (cppi->pd_mem == NULL) {
		dev_dbg(musb->controller, "ERROR: packet descriptor memory allocation failed\n");
		return 0;
	}

	if (cppi41_mem_rgn_alloc(cppi_info->q_mgr, cppi->pd_mem_phys,
				 USB_CPPI41_DESC_SIZE_SHIFT,
				 get_count_order(USB_CPPI41_MAX_PD),
				 &cppi->pd_mem_rgn)) {
		dev_dbg(musb->controller, "ERROR: queue manager memory region allocation "
		    "failed\n");
		goto free_pds;
	}

	/* Allocate the teardown completion queue */
	if (cppi41_queue_alloc(CPPI41_UNASSIGNED_QUEUE,
			       0, &cppi->teardownQNum)) {
		dev_dbg(musb->controller, "ERROR: teardown completion queue allocation failed\n");
		goto free_mem_rgn;
	}
	dev_dbg(musb->controller, "Allocated teardown completion queue %d in queue manager 0\n",
	    cppi->teardownQNum);

	if (cppi41_queue_init(&cppi->queue_obj, 0, cppi->teardownQNum)) {
		dev_dbg(musb->controller, "ERROR: teardown completion queue initialization "
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
		tx_info = cppi41_dma_block[cppi_info->dma_block].tx_ch_info
			  + cppi_info->ep_dma_ch[i];
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
			 (CPPI41_RETURN_LINKED << CPPI41_RETURN_POLICY_SHIFT);

	/* Do necessary configuartion in hardware to get started */
	reg_base = cppi->musb->ctrl_base;

	/* Disable auto request mode */
	musb_writel(reg_base, cppi->automode_reg_offs, 0);

	/* Disable the CDC/RNDIS modes */
	musb_writel(reg_base, USB_TX_MODE_REG, 0);
	musb_writel(reg_base, USB_RX_MODE_REG, 0);

	return 1;

 free_queue:
	if (cppi41_queue_free(0, cppi->teardownQNum))
		dev_dbg(musb->controller, "ERROR: failed to free teardown completion queue\n");

 free_mem_rgn:
	if (cppi41_mem_rgn_free(cppi_info->q_mgr, cppi->pd_mem_rgn))
		dev_dbg(musb->controller, "ERROR: failed to free queue manager memory region\n");

 free_pds:
	dma_free_coherent(cppi->musb->controller,
			  cppi->bd_size,
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
	struct usb_cppi41_info *cppi_info;
	struct musb *musb;

	cppi = container_of(controller, struct cppi41, controller);
	cppi_info = cppi->cppi_info;
	musb = cppi->musb;

	/* Free the teardown completion queue */
	if (cppi41_queue_free(cppi_info->q_mgr, cppi->teardownQNum))
		dev_dbg(musb->controller, "ERROR: failed to free teardown completion queue\n");

	/*
	 * Free the packet descriptor region allocated
	 * for all Tx/Rx channels.
	 */
	if (cppi41_mem_rgn_free(cppi_info->q_mgr, cppi->pd_mem_rgn))
		dev_dbg(musb->controller, "ERROR: failed to free queue manager memory region\n");

	dma_free_coherent(cppi->musb->controller, cppi->bd_size,
			  cppi->pd_mem, cppi->pd_mem_phys);

	cppi->pd_mem = 0;
	cppi->pd_mem_phys = 0;
	cppi->pd_pool_head = 0;
	cppi->bd_size = 0;

	reg_base = cppi->musb->ctrl_base;

	/* Disable auto request mode */
	musb_writel(reg_base, cppi->automode_reg_offs, 0);

	/* Disable the CDC/RNDIS modes */
	musb_writel(reg_base, USB_TX_MODE_REG, 0);
	musb_writel(reg_base, USB_RX_MODE_REG, 0);

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
	struct usb_cppi41_info *cppi_info;
	struct musb *musb;

	cppi = container_of(controller, struct cppi41, controller);
	cppi_info = cppi->cppi_info;
	musb = cppi->musb;

	/* Remember, ep_num: 1 .. Max_EP, and CPPI ch_num: 0 .. Max_EP - 1 */
	ch_num = ep_num - 1;

	if (ep_num > USB_CPPI41_NUM_CH) {
		dev_dbg(musb->controller, "No %cx DMA channel for EP%d\n",
		    is_tx ? 'T' : 'R', ep_num);
		return NULL;
	}

	cppi_ch = (is_tx ? cppi->tx_cppi_ch : cppi->rx_cppi_ch) + ch_num;

	/* As of now, just return the corresponding CPPI 4.1 channel handle */
	if (is_tx) {
		/* Initialize the CPPI 4.1 Tx DMA channel */
		if (cppi41_tx_ch_init(&cppi_ch->dma_ch_obj,
				      cppi_info->dma_block,
				      cppi_info->ep_dma_ch[ch_num])) {
			dev_dbg(musb->controller, "ERROR: cppi41_tx_ch_init failed for "
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
		u8 q_mgr = cppi_info->q_mgr;
		int i;

		/* Initialize the CPPI 4.1 Rx DMA channel */
		if (cppi41_rx_ch_init(&cppi_ch->dma_ch_obj,
				      cppi_info->dma_block,
				      cppi_info->ep_dma_ch[ch_num])) {
			dev_dbg(musb->controller, "ERROR: cppi41_rx_ch_init failed\n");
			return NULL;
		}

		if (cppi41_queue_alloc(CPPI41_FREE_DESC_BUF_QUEUE |
				       CPPI41_UNASSIGNED_QUEUE,
				       q_mgr, &cppi_ch->src_queue.q_num)) {
			dev_dbg(musb->controller, "ERROR: cppi41_queue_alloc failed for "
			    "free descriptor/buffer queue\n");
			return NULL;
		}
		dev_dbg(musb->controller, "Allocated free descriptor/buffer queue %d in "
		    "queue manager %d\n", cppi_ch->src_queue.q_num, q_mgr);

		rx_cfg.default_desc_type = cppi41_rx_host_desc;
		rx_cfg.sop_offset = 0;
		rx_cfg.retry_starved = 1;
		rx_cfg.rx_max_buf_cnt = 0;
		rx_cfg.rx_queue.q_mgr = cppi_ch->src_queue.q_mgr = q_mgr;
		rx_cfg.rx_queue.q_num = cppi_info->rx_comp_q[ch_num];
		for (i = 0; i < 4; i++)
			rx_cfg.cfg.host_pkt.fdb_queue[i] = cppi_ch->src_queue;
		cppi41_rx_ch_configure(&cppi_ch->dma_ch_obj, &rx_cfg);
	}

	/* Initialize the CPPI 4.1 DMA source queue */
	if (cppi41_queue_init(&cppi_ch->queue_obj, cppi_ch->src_queue.q_mgr,
			       cppi_ch->src_queue.q_num)) {
		dev_dbg(musb->controller, "ERROR: cppi41_queue_init failed for %s queue",
		    is_tx ? "Tx" : "Rx free descriptor/buffer");
		if (is_tx == 0 &&
		    cppi41_queue_free(cppi_ch->src_queue.q_mgr,
				      cppi_ch->src_queue.q_num))
			dev_dbg(musb->controller, "ERROR: failed to free Rx descriptor/buffer "
			    "queue\n");
		 return NULL;
	}

	/* Enable the DMA channel */
	cppi41_dma_ch_enable(&cppi_ch->dma_ch_obj);

	if (cppi_ch->end_pt)
		dev_dbg(musb->controller, "Re-allocating DMA %cx channel %d (%p)\n",
		    is_tx ? 'T' : 'R', ch_num, cppi_ch);

	cppi_ch->end_pt = ep;
	cppi_ch->ch_num = ch_num;
	cppi_ch->channel.status = MUSB_DMA_STATUS_FREE;
	cppi_ch->channel.max_len = is_tx ?
				CPPI41_TXDMA_MAXLEN : CPPI41_RXDMA_MAXLEN;

	dev_dbg(musb->controller, "Allocated DMA %cx channel %d for EP%d\n", is_tx ? 'T' : 'R',
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
		printk(KERN_INFO "Releasing idle DMA channel %p\n", cppi_ch);

	/* But for now, not its IRQ */
	cppi_ch->end_pt = NULL;
	channel->status = MUSB_DMA_STATUS_UNKNOWN;

	cppi41_dma_ch_disable(&cppi_ch->dma_ch_obj);

	/* De-allocate Rx free descriptior/buffer queue */
	if (cppi_ch->transmit == 0 &&
	    cppi41_queue_free(cppi_ch->src_queue.q_mgr,
			      cppi_ch->src_queue.q_num))
		printk(KERN_ERR "ERROR: failed to free Rx descriptor/buffer queue\n");
}

static void cppi41_mode_update(struct cppi41_channel *cppi_ch, u8 mode)
{
	if (mode != cppi_ch->dma_mode) {
		struct cppi41 *cppi = cppi_ch->channel.private_data;
		void *__iomem reg_base = cppi->musb->ctrl_base;
		u32 reg_val;
		u8 ep_num = cppi_ch->ch_num + 1;

		if (cppi_ch->transmit) {
			reg_val = musb_readl(reg_base, USB_TX_MODE_REG);
			reg_val &= ~USB_TX_MODE_MASK(ep_num);
			reg_val |= mode << USB_TX_MODE_SHIFT(ep_num);
			musb_writel(reg_base, USB_TX_MODE_REG, reg_val);
		} else {
			reg_val = musb_readl(reg_base, USB_RX_MODE_REG);
			reg_val &= ~USB_RX_MODE_MASK(ep_num);
			reg_val |= mode << USB_RX_MODE_SHIFT(ep_num);
			musb_writel(reg_base, USB_RX_MODE_REG, reg_val);
		}
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
	struct musb *musb = cppi->musb;
	struct usb_pkt_desc *curr_pd;
	u32 length = tx_ch->length - tx_ch->curr_offset;
	u32 pkt_size = tx_ch->pkt_size;
	unsigned num_pds, n;
	struct usb_cppi41_info *cppi_info = cppi->cppi_info;
	u16 q_mgr = cppi_info->q_mgr;
	u16 tx_comp_q = cppi_info->tx_comp_q[tx_ch->ch_num];
	u8 en_bd_intr = cppi->en_bd_intr;
	u8 is_isoc = 0;
	struct musb_hw_ep *hw_ep = cppi->musb->endpoints + tx_ch->end_pt->epnum;
	int xfer_type = hw_ep->xfer_type;

	/*
	 * Tx can use the generic RNDIS mode where we can probably fit this
	 * transfer in one PD and one IRQ.  The only time we would NOT want
	 * to use it is when the hardware constraints prevent it...
	 */
	if ((pkt_size & 0x3f) == 0) {
		num_pds  = length ? 1 : 0;
		cppi41_mode_update(tx_ch, USB_GENERIC_RNDIS_MODE);
	} else {
		num_pds  = (length + pkt_size - 1) / pkt_size;
		cppi41_mode_update(tx_ch, USB_TRANSPARENT_MODE);
	}

	pkt_size = length;
	/*
	 * If length of transmit buffer is 0 or a multiple of the endpoint size,
	 * then send the zero length packet.
	 */
	if (!length || (tx_ch->transfer_mode && length % pkt_size == 0))
		num_pds++;

	dev_dbg(musb->controller, "TX DMA%u, %s, maxpkt %u, %u PDs, addr %#x, len %u\n",
	    tx_ch->ch_num, tx_ch->dma_mode ? "accelerated" : "transparent",
	    pkt_size, num_pds, tx_ch->start_addr + tx_ch->curr_offset, length);

	if (xfer_type  == USB_ENDPOINT_XFER_ISOC)
		is_isoc = 1;

	if (is_isoc && cppi->txfifo_intr_enable && (length <= tx_ch->pkt_size))
		tx_ch->txfifo_intr_enable = 1;

	for (n = 0; n < num_pds; n++) {
		struct cppi41_host_pkt_desc *hw_desc;

		/* Get Tx host packet descriptor from the free pool */
		curr_pd = usb_get_free_pd(cppi);
		if (curr_pd == NULL) {
			dev_dbg(musb->controller, "No Tx PDs\n");
			break;
		}

		if (length < pkt_size)
			pkt_size = length;

		hw_desc = &curr_pd->hw_desc;
		hw_desc->desc_info = (CPPI41_DESC_TYPE_HOST <<
				      CPPI41_DESC_TYPE_SHIFT) | pkt_size;
		hw_desc->tag_info = tx_ch->tag_info;
		hw_desc->pkt_info = cppi->pkt_info;
		hw_desc->pkt_info |= ((q_mgr << CPPI41_RETURN_QMGR_SHIFT) |
				(tx_comp_q << CPPI41_RETURN_QNUM_SHIFT));

		hw_desc->buf_ptr = tx_ch->start_addr + tx_ch->curr_offset;
		hw_desc->buf_len = pkt_size;
		hw_desc->next_desc_ptr = 0;
		hw_desc->orig_buf_len = pkt_size;

		curr_pd->ch_num = tx_ch->ch_num;
		curr_pd->ep_num = tx_ch->end_pt->epnum;

		tx_ch->curr_offset += pkt_size;
		length -= pkt_size;

		if (pkt_size == 0)
			tx_ch->zlp_queued = 1;

		if (en_bd_intr)
			hw_desc->orig_buf_len |= CPPI41_PKT_INTR_FLAG;

		dev_dbg(musb->controller, "TX PD %p: buf %08x, len %08x, pkt info %08x\n", curr_pd,
		    hw_desc->buf_ptr, hw_desc->buf_len, hw_desc->pkt_info);

		/* enable tx fifo empty interrupt */
		if (tx_ch->txfifo_intr_enable) {
			dev_dbg(musb->controller, "txs(%p %d) enable txFintr\n",
				curr_pd, hw_desc->orig_buf_len &
					~CPPI41_PKT_INTR_FLAG);
			txfifoempty_int_enable(cppi->musb, curr_pd->ep_num);
		}

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
		u32 reg_val = musb_readl(reg_base, cppi->automode_reg_offs);
		u8 ep_num = rx_ch->ch_num + 1;

		reg_val &= ~USB_RX_AUTOREQ_MASK(ep_num);
		reg_val |= autoreq << USB_RX_AUTOREQ_SHIFT(ep_num);

		musb_writel(reg_base, cppi->automode_reg_offs, reg_val);
		rx_ch->autoreq = autoreq;
	}
}

static void cppi41_set_ep_size(struct cppi41_channel *rx_ch, u32 pkt_size)
{
	struct cppi41 *cppi = rx_ch->channel.private_data;
	void *__iomem reg_base = cppi->musb->ctrl_base;
	u8 ep_num = rx_ch->ch_num + 1;
	u32 res = pkt_size % 64;

	/* epsize register must be multiple of 64 */
	pkt_size += res ? (64 - res) : res;

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
	struct musb *musb = cppi->musb;
	struct usb_pkt_desc *curr_pd;
	struct cppi41_host_pkt_desc *hw_desc;
	u32 length = rx_ch->length - rx_ch->curr_offset;
	u32 pkt_size = rx_ch->pkt_size;
	u32 max_rx_transfer_size = 64 * 1024;
	u32 i, n_bd , pkt_len;
	struct usb_gadget_driver *gadget_driver;
	u8 en_bd_intr = cppi->en_bd_intr, mode;

	if (is_peripheral_active(cppi->musb)) {
		/* TODO: temporary fix for CDC/RNDIS which needs to be in
		 * GENERIC_RNDIS mode. Without this RNDIS gadget taking
		 * more then 2K ms for a 64 byte pings.
		 */
		gadget_driver = cppi->musb->gadget_driver;

		pkt_len = rx_ch->pkt_size;
		mode = USB_GENERIC_RNDIS_MODE;
		if (!strcmp(gadget_driver->driver.name, "g_file_storage") ||
			!strcmp(gadget_driver->driver.name, "g_mass_storage")) {
			if (cppi->inf_mode && length > pkt_len) {
				pkt_len = 0;
				length = length - rx_ch->pkt_size;
				cppi41_rx_ch_set_maxbufcnt(&rx_ch->dma_ch_obj,
					DMA_CH_RX_MAX_BUF_CNT_1);
				rx_ch->inf_mode = 1;
			} else {
				max_rx_transfer_size = rx_ch->pkt_size;
				mode = USB_TRANSPARENT_MODE;
			}
		} else
			if (rx_ch->length < max_rx_transfer_size)
				pkt_len = rx_ch->length;

		if (mode != USB_TRANSPARENT_MODE)
			cppi41_set_ep_size(rx_ch, pkt_len);
		cppi41_mode_update(rx_ch, mode);
	} else {
		/*
		 * Rx can use the generic RNDIS mode where we can
		 * probably fit this transfer in one PD and one IRQ
		 * (or two with a short packet).
		 */
		if (cppi->cppi_info->grndis_for_host_rx &&
					(pkt_size & 0x3f) == 0) {
			cppi41_mode_update(rx_ch, USB_GENERIC_RNDIS_MODE);
			cppi41_autoreq_update(rx_ch, USB_AUTOREQ_ALL_BUT_EOP);

			pkt_size = (length > 0x10000) ? 0x10000 : length;
			cppi41_set_ep_size(rx_ch, pkt_size);
			mode = USB_GENERIC_RNDIS_MODE;
		} else {
			cppi41_mode_update(rx_ch, USB_TRANSPARENT_MODE);
			cppi41_autoreq_update(rx_ch, USB_NO_AUTOREQ);
			max_rx_transfer_size = rx_ch->hb_mult * rx_ch->pkt_size;
			mode = USB_TRANSPARENT_MODE;
		}
	}

	dev_dbg(musb->controller, "RX DMA%u, %s, maxpkt %u, addr %#x, rec'd %u/%u\n",
	    rx_ch->ch_num, rx_ch->dma_mode ? "accelerated" : "transparent",
	    pkt_size, rx_ch->start_addr + rx_ch->curr_offset,
	    rx_ch->curr_offset, rx_ch->length);

	/* calculate number of bd required */
	if (is_host_active(cppi->musb) && (mode == USB_TRANSPARENT_MODE))
		n_bd = 1;
	else
		n_bd = (length + max_rx_transfer_size - 1)/max_rx_transfer_size;

	for (i = 0; i < n_bd ; ++i) {
		/* Get Rx packet descriptor from the free pool */
		curr_pd = usb_get_free_pd(cppi);
		if (curr_pd == NULL) {
			/* Shouldn't ever happen! */
			dev_dbg(musb->controller, "No Rx PDs\n");
			goto sched;
		}

		pkt_len =
		(length > max_rx_transfer_size) ? max_rx_transfer_size : length;

		hw_desc = &curr_pd->hw_desc;
		hw_desc->desc_info = (CPPI41_DESC_TYPE_HOST <<
				      CPPI41_DESC_TYPE_SHIFT);
		hw_desc->orig_buf_ptr = rx_ch->start_addr + rx_ch->curr_offset;
		hw_desc->orig_buf_len = pkt_len;

		/* buf_len field of buffer descriptor updated by dma
		 * after reception of data is completed
		 */
		hw_desc->buf_len = 0;

		curr_pd->ch_num = rx_ch->ch_num;
		curr_pd->ep_num = rx_ch->end_pt->epnum;

		curr_pd->eop = (length -= pkt_len) ? 0 : 1;
		rx_ch->curr_offset += pkt_len;

		if (en_bd_intr)
			hw_desc->orig_buf_len |= CPPI41_PKT_INTR_FLAG;
		/*
		 * Push the free Rx packet descriptor
		 * to the free descriptor/buffer queue.
		 */
		cppi41_queue_push(&rx_ch->queue_obj, curr_pd->dma_addr,
			USB_CPPI41_DESC_ALIGN, 0);
	}

sched:
	/*
	 * HCD arranged ReqPkt for the first packet.
	 * We arrange it for all but the last one.
	 */
	if (is_host_active(cppi->musb) && rx_ch->channel.actual_len) {
		void __iomem *epio = rx_ch->end_pt->regs;
		u16 csr = musb_readw(epio, MUSB_RXCSR);
		u8 curr_toggle = (csr & MUSB_RXCSR_H_DATATOGGLE) ? 1 : 0;

		/* check if data toggle bit got out of sync */
		if (curr_toggle == rx_ch->end_pt->prev_toggle) {
			dev_dbg(musb->controller,
				"Data toggle same as previous (=%d) on ep%d\n",
					curr_toggle, rx_ch->end_pt->epnum);

			csr |= MUSB_RXCSR_H_DATATOGGLE |
					MUSB_RXCSR_H_WR_DATATOGGLE;
			musb_writew(epio, MUSB_RXCSR, csr);

			rx_ch->end_pt->prev_toggle = !curr_toggle;
		} else {
			rx_ch->end_pt->prev_toggle = curr_toggle;
		}

		csr = musb_readw(epio, MUSB_RXCSR);
		csr |= MUSB_RXCSR_H_REQPKT | MUSB_RXCSR_H_WZC_BITS;
		musb_writew(epio, MUSB_RXCSR, csr);
	}

	/* enable schedular if not enabled */
	if (is_peripheral_active(cppi->musb) && (n_bd > 0))
		cppi41_schedtbl_add_dma_ch(0, 0, rx_ch->ch_num, 0);
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
		WARNING("%cx DMA%d not allocated!\n",
		    cppi_ch->transmit ? 'T' : 'R', cppi_ch->ch_num);
		return 0;
	case MUSB_DMA_STATUS_FREE:
		break;
	}

	channel->status = MUSB_DMA_STATUS_BUSY;

	/* Set the transfer parameters, then queue up the first segment */
	cppi_ch->start_addr = dma_addr;
	cppi_ch->curr_offset = 0;
	cppi_ch->hb_mult = (maxpacket >> 11) & 0x03;
	cppi_ch->pkt_size = maxpacket & 0x7ff;
	cppi_ch->length = length;
	cppi_ch->transfer_mode = mode;
	cppi_ch->zlp_queued = 0;
	cppi_ch->channel.actual_len = 0;

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
	struct cppi41 *cppi = cppi_ch->channel.private_data;
	struct usb_cppi41_info *cppi_info = cppi->cppi_info;
	struct musb *musb = cppi->musb;

	if (cppi41_get_teardown_info(pd_addr, &info)) {
		dev_dbg(musb->controller, "ERROR: not a teardown descriptor\n");
		return 0;
	}

	if ((info & CPPI41_TEARDOWN_TX_RX_MASK) ==
	    (!cppi_ch->transmit << CPPI41_TEARDOWN_TX_RX_SHIFT) &&
	    (info & CPPI41_TEARDOWN_DMA_NUM_MASK) ==
	    (cppi_info->dma_block << CPPI41_TEARDOWN_DMA_NUM_SHIFT) &&
	    (info & CPPI41_TEARDOWN_CHAN_NUM_MASK) ==
	    (cppi_info->ep_dma_ch[cppi_ch->ch_num] <<
	     CPPI41_TEARDOWN_CHAN_NUM_SHIFT))
		return 1;

	dev_dbg(musb->controller, "ERROR: unexpected values in teardown descriptor\n");
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
	struct musb *musb = cppi->musb;
	void __iomem *reg_base = musb->ctrl_base;
	u32 td_reg, timeout = 0xfffff;
	u8 ep_num = tx_ch->ch_num + 1;
	unsigned long pd_addr;
	struct cppi41_queue_obj tx_queue_obj;
	struct usb_cppi41_info *cppi_info;

	/* Initiate teardown for Tx DMA channel */
	cppi41_dma_ch_teardown(&tx_ch->dma_ch_obj);

	/* Wait for a descriptor to be queued and pop it... */
	do {
		td_reg  = musb_readl(reg_base, cppi->teardown_reg_offs);
		td_reg |= USB_TX_TDOWN_MASK(ep_num);
		musb_writel(reg_base, cppi->teardown_reg_offs, td_reg);

		pd_addr = cppi41_queue_pop(&cppi->queue_obj);
	} while (!pd_addr && timeout--);

	if (pd_addr) {

		dev_dbg(musb->controller, "Descriptor (%08lx) popped from teardown completion "
			"queue\n", pd_addr);

		if (usb_check_teardown(tx_ch, pd_addr)) {
			dev_dbg(musb->controller, "Teardown Desc (%lx) rcvd\n", pd_addr);
		} else
			ERR("Invalid PD(%08lx)popped from TearDn completion"
				"queue\n", pd_addr);
	} else {
		if (timeout <= 0)
			ERR("Teardown Desc not rcvd\n");
	}

	/* read the tx completion queue and remove
	 * completion bd if any
	 */
	cppi_info = cppi->cppi_info;
	if (cppi41_queue_init(&tx_queue_obj, cppi_info->q_mgr,
			      cppi_info->tx_comp_q[tx_ch->ch_num])) {
		ERR("ERROR: cppi41_queue_init failed for "
		    "Tx completion queue");
		return;
	}

	while ((pd_addr = cppi41_queue_pop(&tx_queue_obj)) != 0) {
		struct usb_pkt_desc *curr_pd;

		curr_pd = usb_get_pd_ptr(cppi, pd_addr);
		if (curr_pd == NULL) {
			ERR("Invalid PD popped from Tx completion queue\n");
			continue;
		}

		dev_dbg(musb->controller, "Tx-PD(%p) popped from completion queue\n", curr_pd);
		dev_dbg(musb->controller, "ch(%d)epnum(%d)len(%d)\n", curr_pd->ch_num,
			curr_pd->ep_num, curr_pd->hw_desc.buf_len);

		usb_put_free_pd(cppi, curr_pd);
	}
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
	struct musb *musb = cppi->musb;
	struct usb_cppi41_info *cppi_info = cppi->cppi_info;
	u32 timeout = 0xfffff, pd_addr;
	struct cppi41_queue_obj rx_queue_obj;

	cppi41_dma_ch_default_queue(&rx_ch->dma_ch_obj, 0, cppi->teardownQNum);

	/* Initiate teardown for Rx DMA channel */
	cppi41_dma_ch_teardown(&rx_ch->dma_ch_obj);

	do {
		struct usb_pkt_desc *curr_pd;
		unsigned long pd_addr;

		/* Wait for a descriptor to be queued and pop it... */
		do {
			pd_addr = cppi41_queue_pop(&cppi->queue_obj);
		} while (!pd_addr && timeout--);

		if (timeout <= 0 || !pd_addr) {
			ERR("teardown Desc not found\n");
			break;
		}

		dev_dbg(musb->controller, "Descriptor (%08lx) popped from teardown completion "
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
	} while (0);

	/* read the rx completion queue and remove
	 * completion bd if any
	 */
	if (cppi41_queue_init(&rx_queue_obj, cppi_info->q_mgr,
			      cppi_info->rx_comp_q[rx_ch->ch_num])) {
		ERR("ERROR: cppi41_queue_init failed for "
		    "Rx completion queue");
		return;
	}

	while ((pd_addr = cppi41_queue_pop(&rx_queue_obj)) != 0) {
		struct usb_pkt_desc *curr_pd;

		curr_pd = usb_get_pd_ptr(cppi, pd_addr);
		if (curr_pd == NULL) {
			ERR("Invalid PD popped from Rx completion queue\n");
			continue;
		}

		dev_dbg(musb->controller, "Rx-PD(%p) popped from completion queue\n", curr_pd);
		dev_dbg(musb->controller, "ch(%d)epnum(%d)len(%d)\n", curr_pd->ch_num,
			curr_pd->ep_num, curr_pd->hw_desc.buf_len);

		usb_put_free_pd(cppi, curr_pd);
	}

	/* Now restore the default Rx completion queue... */
	cppi41_dma_ch_default_queue(&rx_ch->dma_ch_obj, cppi_info->q_mgr,
				    cppi_info->rx_comp_q[rx_ch->ch_num]);
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
	cppi = cppi_ch->channel.private_data;
	musb = cppi->musb;

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
		dev_dbg(musb->controller, "%cx DMA%d not allocated\n",
		    cppi_ch->transmit ? 'T' : 'R', ch_num);
		/* FALLTHROUGH */
	case MUSB_DMA_STATUS_FREE:
		return 0;
	}

	reg_base = musb->ctrl_base;
	epio = cppi_ch->end_pt->regs;
	ep_num = ch_num + 1;

#ifdef DEBUG_CPPI_TD
	printk("Before teardown:");
	print_pd_list(cppi->pd_pool_head);
#endif

	if (cppi_ch->transmit) {
		dprintk("Tx channel teardown, cppi_ch = %p\n", cppi_ch);

		/* disable the DMAreq before teardown */
		csr  = musb_readw(epio, MUSB_TXCSR);
		csr &= ~MUSB_TXCSR_DMAENAB;
		musb_writew(epio, MUSB_TXCSR, csr);

		cppi_ch->tx_complete = 0;
		cppi_ch->txf_complete = 0;
		/* Tear down Tx DMA channel */
		usb_tx_ch_teardown(cppi_ch);

		/* Issue CPPI FIFO teardown for Tx channel */
		td_reg  = musb_readl(reg_base, cppi->teardown_reg_offs);
		td_reg |= USB_TX_TDOWN_MASK(ep_num);
		musb_writel(reg_base, cppi->teardown_reg_offs, td_reg);

		/* Flush FIFO of the endpoint */
		csr  = musb_readw(epio, MUSB_TXCSR);
		csr |= MUSB_TXCSR_FLUSHFIFO | MUSB_TXCSR_H_WZC_BITS;
		musb_writew(epio, MUSB_TXCSR, csr);
		musb_writew(epio, MUSB_TXCSR, csr);
	} else { /* Rx */
		dprintk("Rx channel teardown, cppi_ch = %p\n", cppi_ch);

		/* disable the DMAreq and remove reqpkt */
		csr  = musb_readw(epio, MUSB_RXCSR);
		dev_dbg(musb->controller,
			"before rx-teardown: rxcsr %x rxcount %x\n", csr,
			musb_readw(epio, MUSB_RXCOUNT));

		/* For host, clear (just) ReqPkt at end of current packet(s) */
		if (is_host_active(cppi->musb))
			csr &= ~MUSB_RXCSR_H_REQPKT;

		csr &= ~MUSB_RXCSR_DMAENAB;
		musb_writew(epio, MUSB_RXCSR, csr);


		/* Flush FIFO of the endpoint */
		csr  = musb_readw(epio, MUSB_RXCSR);

		if (csr & MUSB_RXCSR_RXPKTRDY)
			csr |= MUSB_RXCSR_FLUSHFIFO;

		csr |= MUSB_RXCSR_H_WZC_BITS;
		musb_writew(epio, MUSB_RXCSR, csr);
		musb_writew(epio, MUSB_RXCSR, csr);
		csr  = musb_readw(epio, MUSB_RXCSR);

		/* Issue CPPI FIFO teardown for Rx channel */
		td_reg  = musb_readl(reg_base, cppi->teardown_reg_offs);
		td_reg |= USB_RX_TDOWN_MASK(ep_num);
		musb_writel(reg_base, cppi->teardown_reg_offs, td_reg);

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

void txdma_completion_work(struct work_struct *data)
{
	struct cppi41 *cppi = container_of(data, struct cppi41, txdma_work);
	struct cppi41_channel *tx_ch;
	struct musb *musb = cppi->musb;
	unsigned index;
	u8 resched = 0;
	unsigned long flags;

	while (1) {
		for (index = 0; index < USB_CPPI41_NUM_CH; index++) {
			void __iomem *epio;
			u16 csr;

			tx_ch = &cppi->tx_cppi_ch[index];
			spin_lock_irqsave(&musb->lock, flags);
			if (tx_ch->tx_complete) {
				/* Sometimes a EP can unregister from a DMA
				 * channel while the data is still in the FIFO.
				 * Probable reason a proper abort was not
				 * called before taking such a step.
				 * Protect against such cases.
				 */
				if (!tx_ch->end_pt) {
					tx_ch->tx_complete = 0;
					tx_ch->count = 0;
					continue;
				}

				epio = tx_ch->end_pt->regs;
				csr = musb_readw(epio, MUSB_TXCSR);

				if (!tx_ch->txfifo_intr_enable &&
					(csr & (MUSB_TXCSR_TXPKTRDY |
					MUSB_TXCSR_FIFONOTEMPTY))) {
					resched = 1;
				} else {
					if (tx_ch->count > 0) {
						tx_ch->count--;
						resched = 1;
						continue;
					}
					tx_ch->channel.status =
						MUSB_DMA_STATUS_FREE;
					tx_ch->tx_complete = 0;
					musb_dma_completion(musb, index+1, 1);
				}
			}
			spin_unlock_irqrestore(&musb->lock, flags);

			if (!resched)
				cond_resched();
		}

		if (resched) {
			resched = 0;
			cond_resched();
		} else {
			return ;
		}
	}

}

void cppi41_handle_txfifo_intr(struct musb *musb, u16 usbintr)
{
	struct cppi41 *cppi;
	struct cppi41_channel *tx_ch;
	int index;

	cppi = container_of(musb->dma_controller, struct cppi41, controller);
	for (index = 0; (index < USB_CPPI41_NUM_CH) && usbintr; index++) {
		if (usbintr & 1) {
			tx_ch = &cppi->tx_cppi_ch[index];
			if (tx_ch->txf_complete) {
				/* disable txfifo empty interupt */
				txfifoempty_int_disable(musb, index+1);
				tx_ch->txf_complete = 0;
				if (!tx_ch->txfifo_intr_enable)
					dev_dbg(musb->controller,
					"Bug, wrong TxFintr ep%d\n", index+1);
				tx_ch->txfifo_intr_enable = 0;

				tx_ch->channel.status =
					MUSB_DMA_STATUS_FREE;

				dev_dbg(musb->controller,
					"txc: givback ep%d\n", index+1);
				musb_dma_completion(musb, index+1, 1);
			}
		}
		usbintr = usbintr >> 1;
	}
}
EXPORT_SYMBOL(cppi41_handle_txfifo_intr);

/**
 * cppi41_dma_controller_create -
 * instantiate an object representing DMA controller.
 */
struct dma_controller * __devinit
cppi41_dma_controller_create(struct musb  *musb, void __iomem *mregs)
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
	cppi->cppi_info = (struct usb_cppi41_info *)&usb_cppi41_info[musb->id];;
	cppi->en_bd_intr = cppi->cppi_info->bd_intr_ctrl;
	cppi->txfifo_intr_enable = musb->txfifo_intr_enable;
	INIT_WORK(&cppi->txdma_work, txdma_completion_work);

	/*
	 * Extra IN token has been seen when a file is transferred from one MSC
	 * device to other due to xDMA IP bug when multiple masters access
	 * mentor controller register space.
	 * As a software workaround use transparent mode and correct data toggle
	 * when they go wrong.
	 * This issue is expected to be fixed in RTL version post 0xD.
	 */
	if ((cppi->cppi_info->version & USBSS_RTL_VERSION_MASK) >
			USBSS_RTL_VERSION_D)
		cppi->cppi_info->grndis_for_host_rx = 1;
	else
		cppi->cppi_info->grndis_for_host_rx = 0;

	/* enable infinite mode only for ti81xx silicon rev2 */
	if (cpu_is_am33xx() || cpu_is_ti816x()) {
		dev_dbg(musb->controller, "cppi41dma supports infinite mode\n");
		cppi->inf_mode = 1;
	}

	return &cppi->controller;
}
EXPORT_SYMBOL(cppi41_dma_controller_create);

/**
 * cppi41_dma_controller_destroy -
 * destroy a previously instantiated DMA controller
 * @controller: the controller
 */
void cppi41_dma_controller_destroy(struct dma_controller *controller)
{
	struct cppi41 *cppi;

	cppi = container_of(controller, struct cppi41, controller);

	/* Free the CPPI object */
	kfree(cppi);
}
EXPORT_SYMBOL(cppi41_dma_controller_destroy);

static void usb_process_tx_queue(struct cppi41 *cppi, unsigned index)
{
	struct cppi41_queue_obj tx_queue_obj;
	unsigned long pd_addr;
	struct usb_cppi41_info *cppi_info = cppi->cppi_info;
	struct musb *musb = cppi->musb;

	if (cppi41_queue_init(&tx_queue_obj, cppi_info->q_mgr,
			      cppi_info->tx_comp_q[index])) {
		dev_dbg(musb->controller, "ERROR: cppi41_queue_init failed for "
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
			void __iomem *epio;
			u16 csr;

			/*
			 * We get Tx DMA completion interrupt even when
			 * data is still in FIFO and not moved out to
			 * USB bus. As we program the next request we
			 * flush out and old data in FIFO which affects
			 * USB functionality. So far, we have obsered
			 * failure with iperf.
			 */
			/* wait for tx fifo empty completion interrupt
			 * if enabled other wise use the workthread
			 * to poll fifo empty status
			 */
			epio = tx_ch->end_pt->regs;
			csr = musb_readw(epio, MUSB_TXCSR);

			if (tx_ch->txfifo_intr_enable) {
				tx_ch->txf_complete = 1;
				dev_dbg(musb->controller,
				"wait for TxF-EmptyIntr ep%d\n", ep_num);
			} else {
				int residue;

				residue = tx_ch->channel.actual_len %
						tx_ch->pkt_size;

				if (tx_ch->pkt_size > 128 && !residue) {
					tx_ch->channel.status =
						MUSB_DMA_STATUS_FREE;
					musb_dma_completion(cppi->musb,
						ep_num, 1);
				} else {
					tx_ch->tx_complete = 1;
					tx_ch->count = 1;
					schedule_work(&cppi->txdma_work);
				}
			}
		}
	}
}

static void usb_process_rx_queue(struct cppi41 *cppi, unsigned index)
{
	struct cppi41_queue_obj rx_queue_obj;
	unsigned long pd_addr;
	struct usb_cppi41_info *cppi_info = cppi->cppi_info;
	struct musb *musb = cppi->musb;
	u8 en_bd_intr = cppi->en_bd_intr;

	if (cppi41_queue_init(&rx_queue_obj, cppi_info->q_mgr,
			      cppi_info->rx_comp_q[index])) {
		dev_dbg(musb->controller, "ERROR: cppi41_queue_init failed for Rx queue\n");
		return;
	}

	while ((pd_addr = cppi41_queue_pop(&rx_queue_obj)) != 0) {
		struct usb_pkt_desc *curr_pd;
		struct cppi41_channel *rx_ch;
		u8 ch_num, ep_num;
		u32 length = 0, orig_buf_len, timeout = 50;

		curr_pd = usb_get_pd_ptr(cppi, pd_addr);
		if (curr_pd == NULL) {
			ERR("Invalid PD popped from Rx completion queue\n");
			continue;
		}

		/* This delay is required to overcome the dma race condition
		 * where software reads buffer descriptor before being updated
		 * by dma as buffer descriptor's writes by dma still pending in
		 * interconnect bridge.
		 */
		while (timeout--) {
			length = curr_pd->hw_desc.desc_info &
					CPPI41_PKT_LEN_MASK;
			if (length != 0)
				break;
			udelay(1);
		}

		if (length == 0)
			ERR("!Race condtion: rxBD read before updated by dma");

		/* Extract the data from received packet descriptor */
		ch_num = curr_pd->ch_num;
		ep_num = curr_pd->ep_num;

		dev_dbg(musb->controller, "Rx complete: dma channel(%d) ep%d len %d timeout %d\n",
			ch_num, ep_num, length, (50-timeout));

		rx_ch = &cppi->rx_cppi_ch[ch_num];
		rx_ch->channel.actual_len += length;

		if (curr_pd->eop) {
			curr_pd->eop = 0;
			/* disable the rx dma schedular */
			if (is_peripheral_active(cppi->musb) && !cppi->inf_mode)
				cppi41_schedtbl_remove_dma_ch(0, 0, ch_num, 0);
		}

		/*
		 * Return Rx PD to the software list --
		 * this is protected by critical section
		 */
		usb_put_free_pd(cppi, curr_pd);

		orig_buf_len = curr_pd->hw_desc.orig_buf_len;
		if (en_bd_intr)
			orig_buf_len &= ~CPPI41_PKT_INTR_FLAG;

		dev_dbg(musb->controller,
			"curr_pd=%p, len=%d, origlen=%d,rxch(alen/len)=%d/%d\n",
				curr_pd, length, orig_buf_len,
				rx_ch->channel.actual_len, rx_ch->length);

		if (unlikely(rx_ch->channel.actual_len >= rx_ch->length ||
			     length < orig_buf_len)) {

#if defined(CONFIG_SOC_OMAPTI81XX) || defined(CONFIG_SOC_OMAPAM33XX)
			struct musb_hw_ep *ep;
			u8 isoc, next_seg = 0;

			/* Workaround for early rx completion of
			 * cppi41 dma in Generic RNDIS mode for ti81xx
			 */
			if (is_host_enabled(cppi->musb)) {
				u32 pkt_size = rx_ch->pkt_size;
				ep = cppi->musb->endpoints + ep_num;
				isoc = musb_readb(ep->regs, MUSB_RXTYPE);
				isoc = (isoc >> 4) & 0x1;

				if (!isoc
				&& (rx_ch->dma_mode == USB_GENERIC_RNDIS_MODE)
				&& (rx_ch->channel.actual_len < rx_ch->length)
				&& !(rx_ch->channel.actual_len % pkt_size))
					next_seg = 1;
			}
			if (next_seg) {
				rx_ch->curr_offset = rx_ch->channel.actual_len;
				cppi41_next_rx_segment(rx_ch);
			} else
#endif
			{
				rx_ch->channel.status = MUSB_DMA_STATUS_FREE;

				if (rx_ch->inf_mode) {
					cppi41_rx_ch_set_maxbufcnt(
					&rx_ch->dma_ch_obj, 0);
					rx_ch->inf_mode = 0;
				}
				/* Rx completion routine callback */
				musb_dma_completion(cppi->musb, ep_num, 0);
			}
		} else {
			if ((is_peripheral_active(cppi->musb) ||
				!cppi->cppi_info->grndis_for_host_rx) &&
				(rx_ch->length - rx_ch->curr_offset) > 0)
				cppi41_next_rx_segment(rx_ch);
		}
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
EXPORT_SYMBOL(cppi41_completion);

MODULE_DESCRIPTION("CPPI4.1 dma controller driver for musb");
MODULE_LICENSE("GPL v2");

static int __init cppi41_dma_init(void)
{
	return 0;
}
module_init(cppi41_dma_init);

static void __exit cppi41_dma__exit(void)
{
}
module_exit(cppi41_dma__exit);
