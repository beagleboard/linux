// SPDX-License-Identifier: GPL-2.0
/*
 *  Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com
 *  Author: Peter Ujfalusi <peter.ujfalusi@ti.com>
 */

#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/of.h>
#include <linux/of_dma.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/workqueue.h>
#include <linux/completion.h>
#include <linux/soc/ti/k3-navss-psilcfg.h>
#include <dt-bindings/dma/k3-udma.h>
#include <linux/soc/ti/k3-navss-ringacc.h>
#include <linux/soc/ti/ti_sci_protocol.h>
#include <linux/irqchip/irq-ti-sci-inta.h>
#include <linux/soc/ti/cppi5.h>

#include "../virt-dma.h"
#include "k3-udma.h"

/* element sizes */
#define UDMA_ELSIZE_8				0
#define UDMA_ELSIZE_16				1
#define UDMA_ELSIZE_24				2
#define UDMA_ELSIZE_32				3
#define UDMA_ELSIZE_64				4

static const u8 elsize_bytes[] = {
	[UDMA_ELSIZE_8] = 1,
	[UDMA_ELSIZE_16] = 2,
	[UDMA_ELSIZE_24] = 3,
	[UDMA_ELSIZE_32] = 4,
	[UDMA_ELSIZE_64] = 8,
};

#define CPPI50_TR_FLAGS_TYPE(x)			(x << 0)

#define CPPI50_TR_FLAGS_STATIC			(1 << 4)

#define CPPI50_TR_FLAGS_EVENT_COMPLETED		(0 << 6)
#define CPPI50_TR_FLAGS_EVENT_ICNT1		(1 << 6)
#define CPPI50_TR_FLAGS_EVENT_ICNT2		(2 << 6)
#define CPPI50_TR_FLAGS_EVENT_ICNT3		(3 << 6)

/* Transfer Request Type 0: One Dimensional Transfer */
struct cppi50_tr_req_type0 {
	u32 flags;
	u16 icnt0;
	u16 unused;
	u64 addr;
} __packed;

/* Transfer Request Type 1: Two Dimensional Transfer */
struct cppi50_tr_req_type1 {
	u32 flags;
	u16 icnt0;
	u16 icnt1;
	u64 addr;
	s32 dim1;
	u32 padd[3]; /* full size is 32 bytes */
} __packed;

/* Transfer Request Type 2: Three Dimensional Transfer */
struct cppi50_tr_req_type2 {
	u32 flags;
	u16 icnt0;
	u16 icnt1;
	u64 addr;
	s32 dim1;
	u16 icnt2;
	u16 unused;
	s32 dim2;
	u32 padd; /* full size is 32 bytes */
} __packed;

/* Transfer Request Type 9: Four Dimensional Block Copy with Repacking */
struct cppi50_tr_req_type9 {
	u32 flags;
	u16 icnt0;
	u16 icnt1;
	u64 addr;
	s32 dim1;
	u16 icnt2;
	u16 icnt3;
	s32 dim2;
	s32 dim3;
	u32 fmtflags;
	s32 ddim1;
	u64 daddr;
	s32 ddim2;
	s32 ddim3;
	u16 dicnt0;
	u16 dicnt1;
	u16 dicnt2;
	u16 dicnt3;
} __packed;

/* Transfer Request Type 10: Two Dimensional BLock Copy */
struct cppi50_tr_req_type10 {
	u32 flags;
	u16 icnt0;
	u16 icnt1;
	u64 addr;
	s32 dim1;
	u32 unused[3];
	u32 fmtflags;
	s32 ddim1;
	u64 daddr;
	u32 padd[4]; /* full size is 64 bytes */
} __packed;

/*
 * Transfer Request Type 15: Four Dimensional Block Copy with Repacking
 *			     and Indirection Support
 */
struct cppi50_tr_req_type15 {
	u32 flags;
	u16 icnt0;
	u16 icnt1;
	u64 addr;
	s32 dim1;
	u16 icnt2;
	u16 icnt3;
	s32 dim2;
	s32 dim3;
	u32 unused;
	s32 ddim1;
	u64 daddr;
	s32 ddim2;
	s32 ddim3;
	u16 dicnt0;
	u16 dicnt1;
	u16 dicnt2;
	u16 dicnt3;
} __packed;

struct cppi50_tr_resp {
	u8 status;
	u8 reserved;
	u8 cmd_id;
	u8 flags;
} __packed;

#define CPPI50_TRDESC_W0_LAST_ENTRY(x)		(((x) & 0x3fff) << 0)
#define CPPI50_TRDESC_W0_RELOAD_IDX(x)		(((x) & 0x3fff) << 14)
#define CPPI50_TRDESC_W0_RELOAD_EN		(0x1 << 28)
#define CPPI50_TRDESC_W0_TYPE			(0x3 << 30)
#define CPPI50_HPDESC_W0_TYPE			(0x2 << 30)

#define CPPI50_TRDESC_W1_FLOWID(x)		(((x) & 0x3fff) << 0)
#define CPPI50_TRDESC_W1_PACKETID(x)		(((x) & 0x3ff) << 14)
#define CPPI50_TRDESC_W1_TR_SIZE_16		(0 << 24)
#define CPPI50_TRDESC_W1_TR_SIZE_32		(1 << 24)
#define CPPI50_TRDESC_W1_TR_SIZE_64		(2 << 24)
#define CPPI50_TRDESC_W1_TR_SIZE_128		(3 << 24)

/* CPPI 5.0 Transfer Request Descriptor */
struct cppi50_tr_req_desc {
	u32 packet_info[4];
};

struct udma_static_tr {
	u8 elsize; /* RPSTR0 */
	u16 elcnt; /* RPSTR0 */
	u16 bstcnt; /* RPSTR1 */
};

/* CPPI 5.0 Host Packet Descriptor */
struct cppi50_host_packet_desc {
	u32 packet_info[4];
	u32 linking_info[2];
	u32 buffer_info[3];
	u32 original_buffer_info[3];
	u32 epib[4];
};

/* CPPI 5.0 Host Buffer Descriptor */
struct cppi50_host_buffer_desc {
	u32 reserved[2];
	u32 buffer_reclamation_info[2];

	u32 linking_info[2];
	u32 buffer_info[3];
	u32 original_buffer_info[3];
};

#define K3_UDMA_MAX_RFLOWS 1024

struct udma_chan;

enum udma_mmr {
	MMR_GCFG = 0,
	MMR_RCHANRT,
	MMR_TCHANRT,
	MMR_LAST,
};

static const char * const mmr_names[] = { "gcfg", "rchanrt", "tchanrt" };

struct udma_tchan {
	void __iomem *reg_rt;

	int id;
	struct k3_nav_ring *t_ring; /* Transmit ring */
	struct k3_nav_ring *tc_ring; /* Transmit Completion ring */
};

struct udma_rchan {
	void __iomem *reg_rt;

	int id;
	struct k3_nav_ring *fd_ring; /* Free Descriptor ring */
	struct k3_nav_ring *r_ring; /* Receive ring*/
};

struct udma_rflow {
	void __iomem *reg_rflow;

	int id;

	/* Do we needthe  rings allocated per flows, not per rchan ? */
};

struct udma_match_data {
	u8 tpl_levels;
	u32 level_start_idx[];
};

struct udma_dev {
	struct dma_device ddev;
	struct device *dev;
	void __iomem *mmrs[MMR_LAST];
	const struct udma_match_data *match_data;

	struct udma_tisci_rm tisci_rm;

	struct device_node *psil_node;
	struct k3_nav_ringacc *ringacc;

	struct irq_domain *irq_domain;

	struct work_struct purge_work;
	struct list_head desc_to_purge;
	spinlock_t lock;

	int tchan_cnt;
	int echan_cnt;
	int rchan_cnt;
	int rflow_cnt;
	unsigned long *tchan_map;
	unsigned long *rchan_map;
	unsigned long *rflow_map;
	unsigned long *rflow_map_reserved;

	struct udma_tchan *tchans;
	struct udma_rchan *rchans;
	struct udma_rflow *rflows;

	struct udma_chan *channels;
	u32 psil_base;
};

/*
 * Slave RX scatter gather workaround:
 * We need to use single continuous buffer if the original buffer is scattered
 */
struct udma_rx_sg_workaround {
	bool in_use;

	struct scatterlist *sgl;
	unsigned int sglen;
	size_t total_len;

	struct scatterlist single_sg;
};

struct udma_desc {
	struct virt_dma_desc vd;

	bool terminated;

	enum dma_transfer_direction dir;

	struct udma_static_tr static_tr;
	u32 residue;

	unsigned int sglen;
	unsigned int desc_idx;
	unsigned int tr_idx;

	/* for slave_sg RX workaround */
	struct udma_rx_sg_workaround rx_sg_wa;

	/* Size of the descriptor area, in bytes  */
	size_t cppi5_desc_area_size;
	/* Size of one descriptor within the descriptor area, in bytes */
	size_t cppi5_desc_size;
	dma_addr_t cppi5_desc_paddr;
	void *cppi5_desc_vaddr;

	/* TR descriptor internal pointers */
	void *tr_req_base;
	struct cppi50_tr_resp *tr_resp_base;

	u32 metadata_size;
	void *metadata; /* pointer to provided metadata buffer (EPIP, PSdata) */
};

enum udma_chan_state {
	UDMA_CHAN_IS_IDLE = 0, /* not active, no teardown is in progress */
	UDMA_CHAN_IS_ACTIVE, /* Normal operation */
	UDMA_CHAN_IS_ACTIVE_FLUSH, /* Flushing for delayed tx */
	UDMA_CHAN_IS_TERMINATING, /* channel is being terminated */
};

struct udma_chan {
	struct virt_dma_chan vc;
	struct dma_slave_config	cfg;
	struct udma_dev *ud;
	struct udma_desc *desc;
	struct udma_desc *terminated_desc;
	struct udma_static_tr static_tr;
	char *name;

	struct udma_tchan *tchan;
	struct udma_rchan *rchan;
	struct udma_rflow *rflow;

	struct k3_nav_psil_entry *psi_link;

	u32 irq_ra_tisci;
	u32 irq_ra_idx;
	u32 irq_udma_idx;

	int irq_num_ring;
	int irq_num_udma;

	bool cyclic;
	bool paused;

	enum udma_chan_state state;
	struct completion teardown_completed;

	u32 bcnt; /* number of bytes completed since the start of the channel */
	u32 in_ring_cnt; /* number of descriptors in flight */

	bool pkt_mode; /* TR or packet */
	bool needs_epib; /* EPIB is needed for the communication or not */
	u32 psd_size; /* size of Protocol Specific Data */
	u32 metadata_size; /* (needs_epib ? 16:0) + psd_size */
	int slave_thread_id;
	u32 src_thread;
	u32 dst_thread;
	u32 static_tr_type;
	enum udma_tp_level channel_tpl; /* Channel Throughput Level */

	u32 id;
	enum dma_transfer_direction dir;
};

static inline struct udma_dev *to_udma_dev(struct dma_device *d)
{
	return container_of(d, struct udma_dev, ddev);
}

static inline struct udma_chan *to_udma_chan(struct dma_chan *c)
{
	return container_of(c, struct udma_chan, vc.chan);
}

static inline struct udma_desc *to_udma_desc(struct dma_async_tx_descriptor *t)
{
	return container_of(t, struct udma_desc, vd.tx);
}

#define UDMA_CH_1000(ch)		(ch * 0x1000)
#define UDMA_CH_100(ch)			(ch * 0x100)
#define UDMA_CH_40(ch)			(ch * 0x40)

/* Generic register access functions */
static inline u32 udma_read(void __iomem *base, int reg)
{
	return __raw_readl(base + reg);
}

static inline void udma_write(void __iomem *base, int reg, u32 val)
{
	__raw_writel(val, base + reg);
}

static inline void udma_update_bits(void __iomem *base, int reg,
				    u32 mask, u32 val)
{
	u32 tmp, orig;

	orig = __raw_readl(base + reg);
	tmp = orig & ~mask;
	tmp |= (val & mask);

	if (tmp != orig)
		__raw_writel(tmp, base + reg);
}

/* TCHANRT */
static inline u32 udma_tchanrt_read(struct udma_tchan *tchan, int reg)
{
	if (!tchan)
		return 0;
	return udma_read(tchan->reg_rt, reg);
}

static inline void udma_tchanrt_write(struct udma_tchan *tchan, int reg,
				      u32 val)
{
	if (!tchan)
		return;
	udma_write(tchan->reg_rt, reg, val);
}

static inline void udma_tchanrt_update_bits(struct udma_tchan *tchan, int reg,
					    u32 mask, u32 val)
{
	if (!tchan)
		return;
	udma_update_bits(tchan->reg_rt, reg, mask, val);
}

/* RCHANRT */
static inline u32 udma_rchanrt_read(struct udma_rchan *rchan, int reg)
{
	if (!rchan)
		return 0;
	return udma_read(rchan->reg_rt, reg);
}

static inline void udma_rchanrt_write(struct udma_rchan *rchan, int reg,
				      u32 val)
{
	if (!rchan)
		return;
	udma_write(rchan->reg_rt, reg, val);
}

static inline void udma_rchanrt_update_bits(struct udma_rchan *rchan, int reg,
					    u32 mask, u32 val)
{
	if (!rchan)
		return;
	udma_update_bits(rchan->reg_rt, reg, mask, val);
}

static inline char *udma_get_dir_text(enum dma_transfer_direction dir)
{
	switch (dir) {
	case DMA_DEV_TO_MEM:
		return "DEV_TO_MEM";
	case DMA_MEM_TO_DEV:
		return "MEM_TO_DEV";
	case DMA_MEM_TO_MEM:
		return "MEM_TO_MEM";
	case DMA_DEV_TO_DEV:
		return "DEV_TO_DEV";
	default:
		break;
	}

	return "invalid";
}

static inline void udma_dump_chan_stdata(struct udma_chan *uc)
{
	struct device *dev = uc->ud->dev;
	u32 offset;
	int i;

	if (uc->dir == DMA_MEM_TO_DEV || uc->dir == DMA_MEM_TO_MEM) {
		dev_dbg(dev, "TCHAN State data:\n");
		for (i = 0; i < 32; i++) {
			offset = UDMA_TCHAN_RT_STDATA_REG + i * 4;
			dev_dbg(dev, "TRT_STDATA[%02d]: 0x%08x\n", i,
				udma_tchanrt_read(uc->tchan, offset));
		}
	}

	if (uc->dir == DMA_DEV_TO_MEM || uc->dir == DMA_MEM_TO_MEM) {
		dev_dbg(dev, "RCHAN State data:\n");
		for (i = 0; i < 32; i++) {
			offset = UDMA_RCHAN_RT_STDATA_REG + i * 4;
			dev_dbg(dev, "RRT_STDATA[%02d]: 0x%08x\n", i,
				udma_rchanrt_read(uc->rchan, offset));
		}
	}
}

static inline dma_addr_t udma_curr_cppi5_desc_paddr(struct udma_desc *d,
						    int idx)
{
	return d->cppi5_desc_paddr + idx * d->cppi5_desc_size;
}

static inline void *udma_curr_cppi5_desc_vaddr(struct udma_desc *d, int idx)
{
	return d->cppi5_desc_vaddr + idx * d->cppi5_desc_size;
}

static inline void *udma_cppi5_paddr_to_vaddr(struct udma_desc *d,
					      dma_addr_t paddr)
{
	if (paddr < d->cppi5_desc_paddr ||
	    paddr >= (d->cppi5_desc_paddr + d->cppi5_desc_area_size))
		return NULL;

	return d->cppi5_desc_vaddr + (paddr - d->cppi5_desc_paddr);
}

static inline struct udma_desc *udma_udma_desc_from_paddr(struct udma_chan *uc,
							  dma_addr_t paddr)
{
	struct udma_desc *d = uc->terminated_desc;

	if (d) {
		dma_addr_t desc_paddr = udma_curr_cppi5_desc_paddr(d,
								   d->desc_idx);

		if (desc_paddr != paddr)
			d = NULL;
	}

	if (!d) {
		d = uc->desc;
		if (d) {
			dma_addr_t desc_paddr = udma_curr_cppi5_desc_paddr(d,
								d->desc_idx);

			if (desc_paddr != paddr)
				d = NULL;
		}
	}

	return d;
}

static void udma_purge_desc_work(struct work_struct *work)
{
	struct udma_dev *ud = container_of(work, typeof(*ud), purge_work);
	struct virt_dma_desc *vd, *_vd;
	unsigned long flags;
	LIST_HEAD(head);

	spin_lock_irqsave(&ud->lock, flags);
	list_splice_tail_init(&ud->desc_to_purge, &head);
	spin_unlock_irqrestore(&ud->lock, flags);

	list_for_each_entry_safe(vd, _vd, &head, node) {
		struct udma_desc *d;

		d = to_udma_desc(&vd->tx);

		if (d->cppi5_desc_vaddr) {
			dma_free_coherent(ud->dev, d->cppi5_desc_area_size,
					  d->cppi5_desc_vaddr,
					  d->cppi5_desc_paddr);
		}

		if (d->rx_sg_wa.in_use) {
			dma_unmap_sg(ud->dev, &d->rx_sg_wa.single_sg, 1,
				     DMA_FROM_DEVICE);
			kfree(sg_virt(&d->rx_sg_wa.single_sg));
		}

		list_del(&vd->node);
		kfree(d);
	}

	/* If more to purge, schedule the work again */
	if (!list_empty(&ud->desc_to_purge))
		schedule_work(&ud->purge_work);
}

static void udma_desc_free(struct virt_dma_desc *vd)
{
	struct udma_dev *ud = to_udma_dev(vd->tx.chan->device);
	struct udma_chan *uc = to_udma_chan(vd->tx.chan);
	struct udma_desc *d = to_udma_desc(&vd->tx);
	unsigned long flags;

	if (uc->terminated_desc == d)
		uc->terminated_desc = NULL;

	spin_lock_irqsave(&ud->lock, flags);
	list_add_tail(&vd->node, &ud->desc_to_purge);
	spin_unlock_irqrestore(&ud->lock, flags);

	schedule_work(&ud->purge_work);
}

static inline bool udma_is_chan_running(struct udma_chan *uc)
{
	u32 trt_ctl = 0;
	u32 rrt_ctl = 0;

	switch (uc->dir) {
	case DMA_DEV_TO_MEM:
		rrt_ctl = udma_rchanrt_read(uc->rchan, UDMA_RCHAN_RT_CTL_REG);
		break;
	case DMA_MEM_TO_DEV:
		trt_ctl = udma_tchanrt_read(uc->tchan, UDMA_TCHAN_RT_CTL_REG);
		break;
	case DMA_MEM_TO_MEM:
		trt_ctl = udma_tchanrt_read(uc->tchan, UDMA_TCHAN_RT_CTL_REG);
		rrt_ctl = udma_rchanrt_read(uc->rchan, UDMA_RCHAN_RT_CTL_REG);
		break;
	default:
		break;
	}

	if (trt_ctl & UDMA_CHAN_RT_CTL_EN || rrt_ctl & UDMA_CHAN_RT_CTL_EN)
		return true;

	return false;
}

static int udma_push_to_ring(struct udma_chan *uc, struct udma_desc *d, int idx)
{
	struct k3_nav_ring *ring = NULL;
	int ret = -EINVAL;

	switch (uc->dir) {
	case DMA_DEV_TO_MEM:
		ring = uc->rchan->fd_ring;
		break;
	case DMA_MEM_TO_DEV:
		ring = uc->tchan->t_ring;
		break;
	case DMA_MEM_TO_MEM:
		ring = uc->tchan->t_ring;
		break;
	default:
		break;
	}

	if (ring) {
		dma_addr_t desc_addr = udma_curr_cppi5_desc_paddr(d, idx);

		wmb(); /* Ensure that writes are not moved over this point */
		dma_sync_single_for_device(uc->ud->dev, desc_addr,
					   d->cppi5_desc_size, DMA_TO_DEVICE);
		ret = k3_nav_ringacc_ring_push(ring, &desc_addr);
		uc->in_ring_cnt++;
	}

	return ret;
}

static int udma_pop_from_ring(struct udma_chan *uc, dma_addr_t *addr)
{
	struct k3_nav_ring *ring = NULL;
	int ret = -ENOENT;

	switch (uc->dir) {
	case DMA_DEV_TO_MEM:
		ring = uc->rchan->r_ring;
		break;
	case DMA_MEM_TO_DEV:
		ring = uc->tchan->tc_ring;
		break;
	case DMA_MEM_TO_MEM:
		ring = uc->tchan->tc_ring;
		break;
	default:
		break;
	}

	if (ring && k3_nav_ringacc_ring_get_occ(ring)) {
		struct udma_desc *d = NULL;

		ret = k3_nav_ringacc_ring_pop(ring, addr);
		if (ret)
			return ret;

		/* Teardown completion */
		if (*addr & 0x1)
			return ret;

		d = udma_udma_desc_from_paddr(uc, *addr);

		if (d)
			dma_sync_single_for_cpu(uc->ud->dev, *addr,
						d->cppi5_desc_size,
						DMA_FROM_DEVICE);
		rmb(); /* Ensure that reads are not moved before this point */

		if (!ret)
			uc->in_ring_cnt--;
	}

	return ret;
}

static void udma_reset_rings(struct udma_chan *uc)
{
	struct k3_nav_ring *ring1 = NULL;
	struct k3_nav_ring *ring2 = NULL;

	switch (uc->dir) {
	case DMA_DEV_TO_MEM:
		if (uc->rchan) {
			ring1 = uc->rchan->fd_ring;
			ring2 = uc->rchan->r_ring;
		}
		break;
	case DMA_MEM_TO_DEV:
		if (uc->tchan) {
			ring1 = uc->tchan->t_ring;
			ring2 = uc->tchan->tc_ring;
		}
		break;
	case DMA_MEM_TO_MEM:
		if (uc->tchan) {
			ring1 = uc->tchan->t_ring;
			ring2 = uc->tchan->tc_ring;
		}
		break;
	default:
		break;
	}

	if (ring1)
		k3_nav_ringacc_ring_reset_dma(ring1, 0);
	if (ring2)
		k3_nav_ringacc_ring_reset(ring2);

	/* make sure we are not leaking memory by stalled descriptor */
	if (uc->terminated_desc) {
		udma_desc_free(&uc->terminated_desc->vd);
		uc->terminated_desc = NULL;
	}

	uc->in_ring_cnt = 0;
}

static inline void udma_reset_counters(struct udma_chan *uc)
{
	u32 val;

	if (uc->tchan) {
		val = udma_tchanrt_read(uc->tchan, UDMA_TCHAN_RT_BCNT_REG);
		udma_tchanrt_write(uc->tchan, UDMA_TCHAN_RT_BCNT_REG, val);

		val = udma_tchanrt_read(uc->tchan, UDMA_TCHAN_RT_SBCNT_REG);
		udma_tchanrt_write(uc->tchan, UDMA_TCHAN_RT_SBCNT_REG, val);

		val = udma_tchanrt_read(uc->tchan, UDMA_TCHAN_RT_PCNT_REG);
		udma_tchanrt_write(uc->tchan, UDMA_TCHAN_RT_PCNT_REG, val);

		val = udma_tchanrt_read(uc->tchan, UDMA_TCHAN_RT_PEER_BCNT_REG);
		udma_tchanrt_write(uc->tchan, UDMA_TCHAN_RT_PEER_BCNT_REG, val);
	}

	if (uc->rchan) {
		val = udma_rchanrt_read(uc->rchan, UDMA_RCHAN_RT_BCNT_REG);
		udma_rchanrt_write(uc->rchan, UDMA_RCHAN_RT_BCNT_REG, val);

		val = udma_rchanrt_read(uc->rchan, UDMA_RCHAN_RT_SBCNT_REG);
		udma_rchanrt_write(uc->rchan, UDMA_RCHAN_RT_SBCNT_REG, val);

		val = udma_rchanrt_read(uc->rchan, UDMA_RCHAN_RT_PCNT_REG);
		udma_rchanrt_write(uc->rchan, UDMA_RCHAN_RT_PCNT_REG, val);

		val = udma_rchanrt_read(uc->rchan, UDMA_RCHAN_RT_PEER_BCNT_REG);
		udma_rchanrt_write(uc->rchan, UDMA_RCHAN_RT_PEER_BCNT_REG, val);
	}

	uc->bcnt = 0;
}

static inline int udma_reset_chan(struct udma_chan *uc, bool hard)
{
	switch (uc->dir) {
	case DMA_DEV_TO_MEM:
		udma_rchanrt_write(uc->rchan, UDMA_RCHAN_RT_PEER_RT_EN_REG, 0);
		udma_rchanrt_write(uc->rchan, UDMA_RCHAN_RT_CTL_REG, 0);
		break;
	case DMA_MEM_TO_DEV:
		udma_tchanrt_write(uc->tchan, UDMA_TCHAN_RT_CTL_REG, 0);
		udma_tchanrt_write(uc->tchan, UDMA_TCHAN_RT_PEER_RT_EN_REG, 0);
		break;
	case DMA_MEM_TO_MEM:
		udma_rchanrt_write(uc->rchan, UDMA_RCHAN_RT_CTL_REG, 0);
		udma_tchanrt_write(uc->tchan, UDMA_TCHAN_RT_CTL_REG, 0);
		break;
	default:
		return -EINVAL;
	}

	/* Reset all counters */
	udma_reset_counters(uc);

	if (hard) {
		k3_nav_psil_release_link(uc->psi_link);
		uc->psi_link = k3_nav_psil_request_link(uc->ud->psil_node,
							uc->src_thread,
							uc->dst_thread);
		if (IS_ERR(uc->psi_link)) {
			dev_err(uc->ud->dev,
				"Hard reset failed, chan%d can not be used.\n",
				uc->id);
			uc->psi_link = NULL;
		}
	}
	uc->state = UDMA_CHAN_IS_IDLE;

	return 0;
}

static inline void udma_start_desc(struct udma_chan *uc)
{
	if (uc->cyclic && uc->pkt_mode) {
		int i;

		/* Push all descriptors to ring for cyclic packet mode */
		for (i = 0; i < uc->desc->sglen; i++)
			udma_push_to_ring(uc, uc->desc, i);
	} else {
		udma_push_to_ring(uc, uc->desc, 0);
	}
}

static inline bool udma_chan_needs_reconfiguration(struct udma_chan *uc)
{
	/* Only PDMAs have staticTR */
	if (!uc->static_tr_type)
		return false;

	/* RX channels always need to be reset, reconfigured */
	if (uc->dir == DMA_DEV_TO_MEM)
		return true;

	/* Check if the staticTR configuration has changed for TX */
	if (memcmp(&uc->static_tr, &uc->desc->static_tr, sizeof(uc->static_tr)))
		return true;

	return false;
}

static int udma_start(struct udma_chan *uc)
{
	struct virt_dma_desc *vd = vchan_next_desc(&uc->vc);

	if (!vd) {
		uc->desc = NULL;
		return -ENOENT;
	}

	list_del(&vd->node);

	uc->desc = to_udma_desc(&vd->tx);

	/* Channel is already running and does not need reconfiguration */
	if (udma_is_chan_running(uc) && !udma_chan_needs_reconfiguration(uc)) {
		udma_start_desc(uc);
		goto out;
	}

	/* Make sure that we clear the teardown bit, if it is set */
	udma_reset_chan(uc, false);

	/* Push descriptors before we start the channel */
	udma_start_desc(uc);

	switch (uc->desc->dir) {
	case DMA_DEV_TO_MEM:
		/* Config remote TR */
		if (uc->static_tr_type) {
			udma_rchanrt_write(uc->rchan,
				UDMA_RCHAN_RT_PEER_STATIC_TR_XY_REG,
				PDMA_STATIC_TR_Y(uc->desc->static_tr.elcnt) |
				PDMA_STATIC_TR_X(uc->desc->static_tr.elsize));
			udma_rchanrt_write(uc->rchan,
				UDMA_RCHAN_RT_PEER_STATIC_TR_Z_REG,
				PDMA_STATIC_TR_Z(uc->desc->static_tr.bstcnt));

			/* save the current staticTR configuration */
			memcpy(&uc->static_tr, &uc->desc->static_tr,
			       sizeof(uc->static_tr));
		}

		udma_rchanrt_write(uc->rchan, UDMA_RCHAN_RT_CTL_REG,
				   UDMA_CHAN_RT_CTL_EN);

		/* Enable remote */
		udma_rchanrt_write(uc->rchan, UDMA_RCHAN_RT_PEER_RT_EN_REG,
				   UDMA_PEER_RT_EN_ENABLE);

		break;
	case DMA_MEM_TO_DEV:
		/* Config remote TR */
		if (uc->static_tr_type) {
			udma_tchanrt_write(uc->tchan,
				UDMA_TCHAN_RT_PEER_STATIC_TR_XY_REG,
				PDMA_STATIC_TR_Y(uc->desc->static_tr.elcnt) |
				PDMA_STATIC_TR_X(uc->desc->static_tr.elsize));

			/* save the current staticTR configuration */
			memcpy(&uc->static_tr, &uc->desc->static_tr,
			       sizeof(uc->static_tr));
		}

		/* Enable remote */
		udma_tchanrt_write(uc->tchan, UDMA_TCHAN_RT_PEER_RT_EN_REG,
				   UDMA_PEER_RT_EN_ENABLE);

		udma_tchanrt_write(uc->tchan, UDMA_TCHAN_RT_CTL_REG,
				   UDMA_CHAN_RT_CTL_EN);

		break;
	case DMA_MEM_TO_MEM:
		udma_rchanrt_write(uc->rchan, UDMA_RCHAN_RT_CTL_REG,
				   UDMA_CHAN_RT_CTL_EN);
		udma_tchanrt_write(uc->tchan, UDMA_TCHAN_RT_CTL_REG,
				   UDMA_CHAN_RT_CTL_EN);

		break;
	default:
		return -EINVAL;
	}

	uc->state = UDMA_CHAN_IS_ACTIVE;
out:

	return 0;
}

static inline int udma_stop(struct udma_chan *uc)
{
	enum udma_chan_state old_state = uc->state;

	uc->state = UDMA_CHAN_IS_TERMINATING;
	reinit_completion(&uc->teardown_completed);

	switch (uc->dir) {
	case DMA_DEV_TO_MEM:
		udma_rchanrt_write(uc->rchan, UDMA_RCHAN_RT_PEER_RT_EN_REG,
				   UDMA_PEER_RT_EN_ENABLE |
				   UDMA_PEER_RT_EN_TEARDOWN);
		break;
	case DMA_MEM_TO_DEV:
		udma_tchanrt_write(uc->tchan, UDMA_TCHAN_RT_PEER_RT_EN_REG,
				   UDMA_PEER_RT_EN_ENABLE |
				   UDMA_PEER_RT_EN_FLUSH);
		udma_tchanrt_write(uc->tchan, UDMA_TCHAN_RT_CTL_REG,
				   UDMA_CHAN_RT_CTL_EN |
				   UDMA_CHAN_RT_CTL_TDOWN);
		break;
	case DMA_MEM_TO_MEM:
		udma_tchanrt_write(uc->tchan, UDMA_TCHAN_RT_CTL_REG,
				   UDMA_CHAN_RT_CTL_EN |
				   UDMA_CHAN_RT_CTL_TDOWN);
		break;
	default:
		uc->state = old_state;
		complete_all(&uc->teardown_completed);
		return -EINVAL;
	}

	return 0;
}

static void udma_cyclic_packet_elapsed(struct udma_chan *uc,
				       struct udma_desc *d)
{
	unsigned int hdesc_size = d->cppi5_desc_size;
	struct knav_udmap_host_desc_t *h_desc;

	h_desc = d->cppi5_desc_vaddr + hdesc_size * d->desc_idx;
	knav_udmap_hdesc_reset_to_original(h_desc);
	udma_push_to_ring(uc, d, d->desc_idx);
	d->desc_idx = (d->desc_idx + 1) % d->sglen;
}

static inline void udma_fetch_epib(struct udma_chan *uc, struct udma_desc *d)
{
	struct knav_udmap_host_desc_t *h_desc = d->cppi5_desc_vaddr;

	memcpy(d->metadata, h_desc->epib, d->metadata_size);
}

static inline bool udma_is_desc_really_done(struct udma_chan *uc,
					    struct udma_desc *d)
{
	u32 peer_bcnt, bcnt;

	/* Only TX towards PDMA is affected */
	if (!uc->static_tr_type || uc->dir != DMA_MEM_TO_DEV)
		return true;

	peer_bcnt = udma_tchanrt_read(uc->tchan, UDMA_TCHAN_RT_PEER_BCNT_REG);
	bcnt = udma_tchanrt_read(uc->tchan, UDMA_TCHAN_RT_BCNT_REG);

	if (peer_bcnt < bcnt)
		return false;

	return true;
}

static void udma_flush_tx(struct udma_chan *uc)
{
	if (uc->dir != DMA_MEM_TO_DEV)
		return;

	uc->state = UDMA_CHAN_IS_ACTIVE_FLUSH;

	udma_tchanrt_write(uc->tchan, UDMA_TCHAN_RT_CTL_REG,
			   UDMA_CHAN_RT_CTL_EN |
			   UDMA_CHAN_RT_CTL_TDOWN);
}

static void udma_ring_callback(struct udma_chan *uc, dma_addr_t paddr)
{
	struct udma_desc *d;
	unsigned long flags;

	if (!paddr)
		return;

	spin_lock_irqsave(&uc->vc.lock, flags);

	/* Teardown completion message */
	if (paddr & 0x1) {
		/* Compensate our internal pop/push counter */
		uc->in_ring_cnt++;

		complete_all(&uc->teardown_completed);

		if (uc->terminated_desc) {
			udma_desc_free(&uc->terminated_desc->vd);
			uc->terminated_desc = NULL;
		}

		if (!uc->desc)
			udma_start(uc);

		if (uc->state != UDMA_CHAN_IS_ACTIVE_FLUSH)
			goto out;
		else if (uc->desc)
			paddr = udma_curr_cppi5_desc_paddr(uc->desc,
							   uc->desc->desc_idx);
	}

	d = udma_udma_desc_from_paddr(uc, paddr);

	if (d) {
		dma_addr_t desc_paddr = udma_curr_cppi5_desc_paddr(d,
								   d->desc_idx);
		if (desc_paddr != paddr) {
			dev_err(uc->ud->dev, "not matching descriptors!\n");
			goto out;
		}

		if (uc->cyclic) {
			/* push the descriptor back to the ring */
			if (d == uc->desc) {
				udma_cyclic_packet_elapsed(uc, d);
				vchan_cyclic_callback(&d->vd);
			}
		} else {
			bool desc_done = true;

			if (d == uc->desc) {
				desc_done = udma_is_desc_really_done(uc, d);

				if (desc_done) {
					uc->bcnt += d->residue;
					udma_start(uc);
				} else {
					udma_flush_tx(uc);
				}
			} else if (d == uc->terminated_desc) {
				uc->terminated_desc = NULL;
			}

			if (desc_done)
				vchan_cookie_complete(&d->vd);
		}
	}
out:
	spin_unlock_irqrestore(&uc->vc.lock, flags);
}

static void udma_tr_event_callback(struct udma_chan *uc)
{
	struct udma_desc *d;
	unsigned long flags;

	spin_lock_irqsave(&uc->vc.lock, flags);
	d = uc->desc;
	if (d) {
		d->tr_idx = (d->tr_idx + 1) % d->sglen;

		if (uc->cyclic) {
			vchan_cyclic_callback(&d->vd);
		} else {
			/* TODO: figure out the real amount of data */
			uc->bcnt += d->residue;
			udma_start(uc);
			vchan_cookie_complete(&d->vd);
		}
	}

	spin_unlock_irqrestore(&uc->vc.lock, flags);
}

static irqreturn_t udma_ring_irq_handler(int irq, void *data)
{
	struct udma_chan *uc = data;
	dma_addr_t paddr = 0;

	if (!udma_pop_from_ring(uc, &paddr))
		udma_ring_callback(uc, paddr);

	return IRQ_HANDLED;
}

static irqreturn_t udma_udma_irq_handler(int irq, void *data)
{
	struct udma_chan *uc = data;
	struct udma_tisci_rm *tisci_rm = &uc->ud->tisci_rm;

	ti_sci_inta_ack_event(uc->ud->irq_domain, tisci_rm->tisci_dev_id,
			      uc->irq_udma_idx, uc->irq_num_udma);

	udma_tr_event_callback(uc);

	return IRQ_HANDLED;
}

/**
 * __udma_reserve_rflow_range - reserve range of flow ids
 * @ud: UDMA device
 * @from: Start the search from this flow id number
 * @cnt: Number of consecutive flow ids to allocate
 *
 * Reserve range of flow ids for future use, those flows can be allocated
 * only using explicit flow id number. if @from is set to -1 it will try to find
 * first free range. if @from is positive value it will force allocation only
 * of the specified range of flows.
 *
 * Returns -ENOMEM if can't find free range.
 * -EEXIST if requested range is busy.
 * -EINVAL if wrong input values passed.
 * Returns flow id on success.
 */
static int __udma_reserve_rflow_range(struct udma_dev *ud, int from, int cnt)
{
	int start, tmp_from;
	DECLARE_BITMAP(tmp, K3_UDMA_MAX_RFLOWS);

	tmp_from = from;
	if (tmp_from < 0)
		tmp_from = ud->rchan_cnt;
	/* default flows can't be reserved and accessible only by id */
	if (tmp_from < ud->rchan_cnt)
		return -EINVAL;

	if (tmp_from + cnt > ud->rflow_cnt)
		return -EINVAL;

	bitmap_or(tmp, ud->rflow_map, ud->rflow_map_reserved,
		  ud->rflow_cnt);

	start = bitmap_find_next_zero_area(tmp,
					   ud->rflow_cnt,
					   tmp_from, cnt, 0);
	if (start >= ud->rflow_cnt)
		return -ENOMEM;

	if (from >= 0 && start != from)
		return -EEXIST;

	bitmap_set(ud->rflow_map_reserved, start, cnt);
	return start;
}

static int __udma_free_rflow_range(struct udma_dev *ud, int from, int cnt)
{
	if (from < ud->rchan_cnt)
		return -EINVAL;
	if (from + cnt > ud->rflow_cnt)
		return -EINVAL;

	bitmap_clear(ud->rflow_map_reserved, from, cnt);
	return 0;
}

static struct udma_rflow *__udma_reserve_rflow(struct udma_dev *ud,
					       enum udma_tp_level tpl, int id)
{
	DECLARE_BITMAP(tmp, K3_UDMA_MAX_RFLOWS);

	if (id >= 0) {
		if (test_bit(id, ud->rflow_map)) {
			dev_err(ud->dev, "rflow%d is in use\n", id);
			return ERR_PTR(-ENOENT);
		}
	} else {
		bitmap_or(tmp, ud->rflow_map, ud->rflow_map_reserved,
			  ud->rflow_cnt);

		id = find_next_zero_bit(tmp, ud->rflow_cnt, ud->rchan_cnt);
		if (id >= ud->rflow_cnt)
			return ERR_PTR(-ENOENT);
	}

	set_bit(id, ud->rflow_map);
	return &ud->rflows[id];
}

#define UDMA_RESERVE_RESOURCE(res)					\
static struct udma_##res *__udma_reserve_##res(struct udma_dev *ud,	\
					       enum udma_tp_level tpl,	\
					       int id)			\
{									\
	if (id >= 0) {							\
		if (test_bit(id, ud->res##_map)) {			\
			dev_err(ud->dev, "res##%d is in use\n", id);	\
			return ERR_PTR(-ENOENT);			\
		}							\
	} else {							\
		int start;						\
									\
		if (tpl >= ud->match_data->tpl_levels)			\
			tpl = ud->match_data->tpl_levels - 1;		\
									\
		start = ud->match_data->level_start_idx[tpl];		\
									\
		id = find_next_zero_bit(ud->res##_map, ud->res##_cnt,	\
					start);				\
		if (id == ud->res##_cnt) {				\
			return ERR_PTR(-ENOENT);			\
		}							\
	}								\
									\
	set_bit(id, ud->res##_map);					\
	return &ud->res##s[id];						\
}

UDMA_RESERVE_RESOURCE(tchan);
UDMA_RESERVE_RESOURCE(rchan);

static int udma_get_tchan(struct udma_chan *uc)
{
	struct udma_dev *ud = uc->ud;

	if (uc->tchan) {
		dev_dbg(ud->dev, "chan%d: already have tchan%d allocated\n",
			uc->id, uc->tchan->id);
		return 0;
	}

	uc->tchan = __udma_reserve_tchan(ud, uc->channel_tpl, -1);
	if (IS_ERR(uc->tchan))
		return PTR_ERR(uc->tchan);

	if (udma_is_chan_running(uc)) {
		dev_warn(ud->dev, "chan%d: tchan%d is running!\n", uc->id,
			 uc->tchan->id);
		udma_stop(uc);
		if (udma_is_chan_running(uc))
			dev_err(ud->dev, "chan%d: won't stop!\n", uc->id);
	}

	return 0;
}

static int udma_get_rchan(struct udma_chan *uc)
{
	struct udma_dev *ud = uc->ud;

	if (uc->rchan) {
		dev_dbg(ud->dev, "chan%d: already have rchan%d allocated\n",
			uc->id, uc->rchan->id);
		return 0;
	}

	uc->rchan = __udma_reserve_rchan(ud, uc->channel_tpl, -1);
	if (IS_ERR(uc->rchan))
		return PTR_ERR(uc->rchan);

	if (udma_is_chan_running(uc)) {
		dev_warn(ud->dev, "chan%d: rchan%d is running!\n", uc->id,
			 uc->rchan->id);
		udma_stop(uc);
		if (udma_is_chan_running(uc))
			dev_err(ud->dev, "chan%d: won't stop!\n", uc->id);
	}

	return 0;
}

static int udma_get_chan_pair(struct udma_chan *uc)
{
	struct udma_dev *ud = uc->ud;
	int chan_id, end;

	if ((uc->tchan && uc->rchan) && uc->tchan->id == uc->rchan->id) {
		dev_info(ud->dev, "chan%d: already have %d pair allocated\n",
			 uc->id, uc->tchan->id);
		return 0;
	}

	if (uc->tchan) {
		dev_err(ud->dev, "chan%d: already have tchan%d allocated\n",
			uc->id, uc->tchan->id);
		return -EBUSY;
	} else if (uc->rchan) {
		dev_err(ud->dev, "chan%d: already have rchan%d allocated\n",
			uc->id, uc->rchan->id);
		return -EBUSY;
	}

	/* Can be optimized, but let's have it like this for now */
	end = min(ud->tchan_cnt, ud->rchan_cnt);
	for (chan_id = ud->match_data->level_start_idx[UDMA_TP_NORMAL];
	     chan_id < end; chan_id++) {
		if (!test_bit(chan_id, ud->tchan_map) &&
		    !test_bit(chan_id, ud->rchan_map))
			break;
	}

	if (chan_id == end)
		return -ENOENT;

	set_bit(chan_id, ud->tchan_map);
	set_bit(chan_id, ud->rchan_map);
	uc->tchan = &ud->tchans[chan_id];
	uc->rchan = &ud->rchans[chan_id];

	if (udma_is_chan_running(uc)) {
		dev_warn(ud->dev, "chan%d: t/rchan%d pair is running!\n",
			 uc->id, chan_id);
		udma_stop(uc);
		if (udma_is_chan_running(uc))
			dev_err(ud->dev, "chan%d: won't stop!\n", uc->id);
	}

	return 0;
}

static int udma_get_rflow(struct udma_chan *uc, int flow_id)
{
	struct udma_dev *ud = uc->ud;

	if (uc->rflow) {
		dev_dbg(ud->dev, "chan%d: already have rflow%d allocated\n",
			uc->id, uc->rflow->id);
		return 0;
	}

	if (!uc->rchan)
		dev_warn(ud->dev, "chan%d: does not have rchan??\n", uc->id);

	uc->rflow = __udma_reserve_rflow(ud, uc->channel_tpl, flow_id);
	if (IS_ERR(uc->rflow))
		return PTR_ERR(uc->rflow);

	return 0;
}

static void udma_put_rchan(struct udma_chan *uc)
{
	struct udma_dev *ud = uc->ud;

	if (uc->rchan) {
		dev_dbg(ud->dev, "chan%d: put rchan%d\n", uc->id,
			uc->rchan->id);
		clear_bit(uc->rchan->id, ud->rchan_map);
		uc->rchan = NULL;
	}
}

static void udma_put_tchan(struct udma_chan *uc)
{
	struct udma_dev *ud = uc->ud;

	if (uc->tchan) {
		dev_dbg(ud->dev, "chan%d: put tchan%d\n", uc->id,
			uc->tchan->id);
		clear_bit(uc->tchan->id, ud->tchan_map);
		uc->tchan = NULL;
	}
}

static void udma_put_rflow(struct udma_chan *uc)
{
	struct udma_dev *ud = uc->ud;

	if (uc->rflow) {
		dev_dbg(ud->dev, "chan%d: put rflow%d\n", uc->id,
			uc->rflow->id);
		clear_bit(uc->rflow->id, ud->rflow_map);
		uc->rflow = NULL;
	}
}

static void udma_free_tx_resources(struct udma_chan *uc)
{
	if (!uc->tchan)
		return;

	k3_nav_ringacc_ring_free(uc->tchan->t_ring);
	k3_nav_ringacc_ring_free(uc->tchan->tc_ring);
	uc->tchan->t_ring = NULL;
	uc->tchan->tc_ring = NULL;

	udma_put_tchan(uc);
}

static int udma_alloc_tx_resources(struct udma_chan *uc)
{
	struct k3_nav_ring_cfg ring_cfg;
	struct udma_dev *ud = uc->ud;
	int ret;

	ret = udma_get_tchan(uc);
	if (ret)
		return ret;

	uc->tchan->t_ring = k3_nav_ringacc_request_ring(ud->ringacc,
							uc->tchan->id, 0);
	if (!uc->tchan->t_ring) {
		ret = -EBUSY;
		goto err_tx_ring;
	}

	uc->tchan->tc_ring = k3_nav_ringacc_request_ring(ud->ringacc, -1, 0);
	if (!uc->tchan->tc_ring) {
		ret = -EBUSY;
		goto err_txc_ring;
	}

	memset(&ring_cfg, 0, sizeof(ring_cfg));
	ring_cfg.size = 16;
	ring_cfg.elm_size = K3_NAV_RINGACC_RING_ELSIZE_8;
	ring_cfg.mode = K3_NAV_RINGACC_RING_MODE_MESSAGE;

	ret = k3_nav_ringacc_ring_cfg(uc->tchan->t_ring, &ring_cfg);
	ret |= k3_nav_ringacc_ring_cfg(uc->tchan->tc_ring, &ring_cfg);

	if (ret)
		goto err_ringcfg;

	return 0;

err_ringcfg:
	k3_nav_ringacc_ring_free(uc->tchan->tc_ring);
	uc->tchan->tc_ring = NULL;
err_txc_ring:
	k3_nav_ringacc_ring_free(uc->tchan->t_ring);
	uc->tchan->t_ring = NULL;
err_tx_ring:
	udma_put_tchan(uc);

	return ret;
}

static void udma_free_rx_resources(struct udma_chan *uc)
{
	if (!uc->rchan)
		return;

	if (uc->dir != DMA_MEM_TO_MEM) {
		k3_nav_ringacc_ring_free(uc->rchan->fd_ring);
		k3_nav_ringacc_ring_free(uc->rchan->r_ring);
		uc->rchan->fd_ring = NULL;
		uc->rchan->r_ring = NULL;

		udma_put_rflow(uc);
	}

	udma_put_rchan(uc);
}

static int udma_alloc_rx_resources(struct udma_chan *uc)
{
	struct k3_nav_ring_cfg ring_cfg;
	struct udma_dev *ud = uc->ud;
	int fd_ring_id;
	int ret;

	ret = udma_get_rchan(uc);
	if (ret)
		return ret;

	/* For MEM_TO_MEM we don't need rflow or rings */
	if (uc->dir == DMA_MEM_TO_MEM)
		return 0;

	ret = udma_get_rflow(uc, uc->rchan->id);
	if (ret) {
		ret = -EBUSY;
		goto err_rflow;
	}

	fd_ring_id = ud->tchan_cnt + ud->echan_cnt + uc->rchan->id;
	uc->rchan->fd_ring = k3_nav_ringacc_request_ring(ud->ringacc,
							 fd_ring_id, 0);
	if (!uc->rchan->fd_ring) {
		ret = -EBUSY;
		goto err_rx_ring;
	}

	uc->rchan->r_ring = k3_nav_ringacc_request_ring(ud->ringacc, -1, 0);
	if (!uc->rchan->r_ring) {
		ret = -EBUSY;
		goto err_rxc_ring;
	}

	memset(&ring_cfg, 0, sizeof(ring_cfg));
	ring_cfg.size = 16;
	ring_cfg.elm_size = K3_NAV_RINGACC_RING_ELSIZE_8;
	ring_cfg.mode = K3_NAV_RINGACC_RING_MODE_MESSAGE;

	ret = k3_nav_ringacc_ring_cfg(uc->rchan->fd_ring, &ring_cfg);
	ret |= k3_nav_ringacc_ring_cfg(uc->rchan->r_ring, &ring_cfg);

	if (ret)
		goto err_ringcfg;

	return 0;

err_ringcfg:
	k3_nav_ringacc_ring_free(uc->rchan->r_ring);
	uc->rchan->r_ring = NULL;
err_rxc_ring:
	k3_nav_ringacc_ring_free(uc->rchan->fd_ring);
	uc->rchan->fd_ring = NULL;
err_rx_ring:
	udma_put_rflow(uc);
err_rflow:
	udma_put_rchan(uc);

	return ret;
}

static int udma_alloc_chan_resources(struct dma_chan *chan)
{
	struct udma_chan *uc = to_udma_chan(chan);
	struct udma_dev *ud = to_udma_dev(chan->device);
	struct udma_tisci_rm *tisci_rm = &ud->tisci_rm;
	const struct ti_sci_rm_udmap_ops *tisci_ops = tisci_rm->tisci_udmap_ops;
	struct udma_tchan *tchan;
	struct udma_rchan *rchan;
	int ret;

	pm_runtime_get_sync(ud->ddev.dev);

	/*
	 * Make sure that the completion is in a known state:
	 * No teardown, the channel is idle
	 */
	reinit_completion(&uc->teardown_completed);
	complete_all(&uc->teardown_completed);
	uc->state = UDMA_CHAN_IS_IDLE;

	switch (uc->dir) {
	case DMA_MEM_TO_MEM:
		/* Non synchronized - mem to mem type of transfer */
		dev_dbg(uc->ud->dev, "%s: chan%d as MEM-to-MEM\n", __func__,
			uc->id);

		ret = udma_get_chan_pair(uc);
		if (ret)
			return ret;

		ret = udma_alloc_tx_resources(uc);
		if (ret)
			return ret;

		ret = udma_alloc_rx_resources(uc);
		if (ret) {
			udma_free_tx_resources(uc);
			return ret;
		}

		uc->src_thread = ud->psil_base + uc->tchan->id;
		uc->dst_thread = (ud->psil_base + uc->rchan->id) | 0x8000;

		break;
	case DMA_MEM_TO_DEV:
		/* Slave transfer synchronized - mem to dev (TX) trasnfer */
		dev_dbg(uc->ud->dev, "%s: chan%d as MEM-to-DEV\n", __func__,
			uc->id);

		ret = udma_alloc_tx_resources(uc);
		if (ret) {
			uc->slave_thread_id = -1;
			return ret;
		}

		uc->src_thread = ud->psil_base + uc->tchan->id;
		uc->dst_thread = uc->slave_thread_id;
		if (!(uc->dst_thread & 0x8000))
			uc->dst_thread |= 0x8000;

		break;
	case DMA_DEV_TO_MEM:
		/* Slave transfer synchronized - dev to mem (RX) trasnfer */
		dev_dbg(uc->ud->dev, "%s: chan%d as DEV-to-MEM\n", __func__,
			uc->id);

		ret = udma_alloc_rx_resources(uc);
		if (ret) {
			uc->slave_thread_id = -1;
			return ret;
		}

		uc->src_thread = uc->slave_thread_id;
		uc->dst_thread = (ud->psil_base + uc->rchan->id) | 0x8000;

		break;
	default:
		/* Can not happen */
		dev_err(uc->ud->dev, "%s: chan%d invalid direction (%u)\n",
			__func__, uc->id, uc->dir);
		return -EINVAL;
	}

	tchan = uc->tchan;
	rchan = uc->rchan;

	/*
	 * Configure Tx and Rx channel type to:
	 * Third Party DMA control transfers using pass by reference rings
	 */
	if (uc->dir == DMA_MEM_TO_MEM) {
		/* Non synchronized - mem to mem type of transfer */
		int tc_ring = k3_nav_ringacc_get_ring_id(tchan->tc_ring);
		struct ti_sci_msg_rm_udmap_tx_ch_cfg req_tx = { 0 };
		struct ti_sci_msg_rm_udmap_rx_ch_cfg req_rx = { 0 };

		req_tx.valid_params =
			TI_SCI_MSG_VALUE_RM_UDMAP_CH_PAUSE_ON_ERR_VALID |
			TI_SCI_MSG_VALUE_RM_UDMAP_CH_TX_FILT_EINFO_VALID |
			TI_SCI_MSG_VALUE_RM_UDMAP_CH_TX_FILT_PSWORDS_VALID |
			TI_SCI_MSG_VALUE_RM_UDMAP_CH_CHAN_TYPE_VALID |
			TI_SCI_MSG_VALUE_RM_UDMAP_CH_TX_SUPR_TDPKT_VALID |
			TI_SCI_MSG_VALUE_RM_UDMAP_CH_FETCH_SIZE_VALID |
			TI_SCI_MSG_VALUE_RM_UDMAP_CH_CQ_QNUM_VALID;

		req_tx.nav_id = tisci_rm->tisci_dev_id;
		req_tx.index = tchan->id;
		req_tx.tx_pause_on_err = 0;
		req_tx.tx_filt_einfo = 0;
		req_tx.tx_filt_pswords = 0;
		req_tx.tx_chan_type = TI_SCI_RM_UDMAP_CHAN_TYPE_3RDP_BCOPY_PBRR;
		req_tx.tx_supr_tdpkt = 0;
		req_tx.tx_fetch_size = sizeof(struct cppi50_tr_req_desc) >> 2;
		req_tx.txcq_qnum = tc_ring;

		ret = tisci_ops->tx_ch_cfg(tisci_rm->tisci, &req_tx);
		if (ret) {
			dev_err(ud->dev, "tchan%d cfg failed %d\n",
				tchan->id, ret);
			goto err_res_free;
		}

		req_rx.valid_params =
			TI_SCI_MSG_VALUE_RM_UDMAP_CH_PAUSE_ON_ERR_VALID |
			TI_SCI_MSG_VALUE_RM_UDMAP_CH_FETCH_SIZE_VALID |
			TI_SCI_MSG_VALUE_RM_UDMAP_CH_CQ_QNUM_VALID |
			TI_SCI_MSG_VALUE_RM_UDMAP_CH_CHAN_TYPE_VALID |
			TI_SCI_MSG_VALUE_RM_UDMAP_CH_RX_IGNORE_SHORT_VALID |
			TI_SCI_MSG_VALUE_RM_UDMAP_CH_RX_IGNORE_LONG_VALID;

		req_rx.nav_id = tisci_rm->tisci_dev_id;
		req_rx.index = rchan->id;
		req_rx.rx_fetch_size = sizeof(struct cppi50_tr_req_desc) >> 2;
		req_rx.rxcq_qnum = tc_ring;
		req_rx.rx_pause_on_err = 0;
		req_rx.rx_chan_type = TI_SCI_RM_UDMAP_CHAN_TYPE_3RDP_BCOPY_PBRR;
		req_rx.rx_ignore_short = 0;
		req_rx.rx_ignore_long = 0;

		ret = tisci_ops->rx_ch_cfg(tisci_rm->tisci, &req_rx);
		if (ret) {
			dev_err(ud->dev, "rchan%d alloc failed %d\n",
				rchan->id, ret);
			goto err_res_free;
		}

		uc->irq_ra_tisci = k3_nav_ringacc_get_tisci_dev_id(
								tchan->tc_ring);
		uc->irq_ra_idx = tc_ring;
		uc->irq_udma_idx = tchan->id;
	} else {
		/* Slave transfer */
		u32 mode, fetch_size;

		if (uc->pkt_mode) {
			mode = TI_SCI_RM_UDMAP_CHAN_TYPE_PKT_PBRR;
			fetch_size = knav_udmap_hdesc_calc_size(uc->needs_epib,
								uc->psd_size,
								0);
		} else {
			mode = TI_SCI_RM_UDMAP_CHAN_TYPE_3RDP_PBRR;
			fetch_size = sizeof(struct cppi50_tr_req_desc);
		}

		if (uc->dir == DMA_MEM_TO_DEV) {
			/* TX */
			int tc_ring = k3_nav_ringacc_get_ring_id(
								tchan->tc_ring);
			struct ti_sci_msg_rm_udmap_tx_ch_cfg req_tx = { 0 };

			req_tx.valid_params =
			TI_SCI_MSG_VALUE_RM_UDMAP_CH_PAUSE_ON_ERR_VALID |
			TI_SCI_MSG_VALUE_RM_UDMAP_CH_TX_FILT_EINFO_VALID |
			TI_SCI_MSG_VALUE_RM_UDMAP_CH_TX_FILT_PSWORDS_VALID |
			TI_SCI_MSG_VALUE_RM_UDMAP_CH_CHAN_TYPE_VALID |
			TI_SCI_MSG_VALUE_RM_UDMAP_CH_TX_SUPR_TDPKT_VALID |
			TI_SCI_MSG_VALUE_RM_UDMAP_CH_FETCH_SIZE_VALID |
			TI_SCI_MSG_VALUE_RM_UDMAP_CH_CQ_QNUM_VALID;

			req_tx.nav_id = tisci_rm->tisci_dev_id;
			req_tx.index = tchan->id;
			req_tx.tx_pause_on_err = 0;
			req_tx.tx_filt_einfo = 0;
			req_tx.tx_filt_pswords = 0;
			req_tx.tx_chan_type = mode;
			req_tx.tx_supr_tdpkt = 0;
			req_tx.tx_fetch_size = fetch_size >> 2;
			req_tx.txcq_qnum = tc_ring;

			ret = tisci_ops->tx_ch_cfg(tisci_rm->tisci, &req_tx);
			if (ret) {
				dev_err(ud->dev, "tchan%d cfg failed %d\n",
					tchan->id, ret);
				goto err_res_free;
			}

			uc->irq_ra_tisci = k3_nav_ringacc_get_tisci_dev_id(
								tchan->tc_ring);
			uc->irq_ra_idx = tc_ring;
			uc->irq_udma_idx = tchan->id;
		} else {
			/* RX */
			int fd_ring = k3_nav_ringacc_get_ring_id(
								rchan->fd_ring);
			int rx_ring = k3_nav_ringacc_get_ring_id(rchan->r_ring);
			struct ti_sci_msg_rm_udmap_rx_ch_cfg req_rx = { 0 };
			struct ti_sci_msg_rm_udmap_flow_cfg flow_req = { 0 };

			req_rx.valid_params =
			TI_SCI_MSG_VALUE_RM_UDMAP_CH_PAUSE_ON_ERR_VALID |
			TI_SCI_MSG_VALUE_RM_UDMAP_CH_FETCH_SIZE_VALID |
			TI_SCI_MSG_VALUE_RM_UDMAP_CH_CQ_QNUM_VALID |
			TI_SCI_MSG_VALUE_RM_UDMAP_CH_CHAN_TYPE_VALID |
			TI_SCI_MSG_VALUE_RM_UDMAP_CH_RX_IGNORE_SHORT_VALID |
			TI_SCI_MSG_VALUE_RM_UDMAP_CH_RX_IGNORE_LONG_VALID;

			req_rx.nav_id = tisci_rm->tisci_dev_id;
			req_rx.index = rchan->id;
			req_rx.rx_fetch_size =  fetch_size >> 2;
			req_rx.rxcq_qnum = rx_ring;
			req_rx.rx_pause_on_err = 0;
			req_rx.rx_chan_type = mode;
			req_rx.rx_ignore_short = 0;
			req_rx.rx_ignore_long = 0;

			ret = tisci_ops->rx_ch_cfg(tisci_rm->tisci, &req_rx);
			if (ret) {
				dev_err(ud->dev, "rchan%d cfg failed %d\n",
					rchan->id, ret);
				goto err_res_free;
			}

			flow_req.valid_params =
			TI_SCI_MSG_VALUE_RM_UDMAP_FLOW_EINFO_PRESENT_VALID |
			TI_SCI_MSG_VALUE_RM_UDMAP_FLOW_PSINFO_PRESENT_VALID |
			TI_SCI_MSG_VALUE_RM_UDMAP_FLOW_ERROR_HANDLING_VALID |
			TI_SCI_MSG_VALUE_RM_UDMAP_FLOW_DESC_TYPE_VALID |
			TI_SCI_MSG_VALUE_RM_UDMAP_FLOW_DEST_QNUM_VALID |
			TI_SCI_MSG_VALUE_RM_UDMAP_FLOW_SRC_TAG_HI_SEL_VALID |
			TI_SCI_MSG_VALUE_RM_UDMAP_FLOW_SRC_TAG_LO_SEL_VALID |
			TI_SCI_MSG_VALUE_RM_UDMAP_FLOW_DEST_TAG_HI_SEL_VALID |
			TI_SCI_MSG_VALUE_RM_UDMAP_FLOW_DEST_TAG_LO_SEL_VALID |
			TI_SCI_MSG_VALUE_RM_UDMAP_FLOW_FDQ0_SZ0_QNUM_VALID |
			TI_SCI_MSG_VALUE_RM_UDMAP_FLOW_FDQ1_QNUM_VALID |
			TI_SCI_MSG_VALUE_RM_UDMAP_FLOW_FDQ2_QNUM_VALID |
			TI_SCI_MSG_VALUE_RM_UDMAP_FLOW_FDQ3_QNUM_VALID;

			flow_req.nav_id = tisci_rm->tisci_dev_id;
			flow_req.flow_index = rchan->id;

			if (uc->needs_epib)
				flow_req.rx_einfo_present = 1;
			else
				flow_req.rx_einfo_present = 0;
			if (uc->psd_size)
				flow_req.rx_psinfo_present = 1;
			else
				flow_req.rx_psinfo_present = 0;
			flow_req.rx_error_handling = 1;
			flow_req.rx_desc_type = 0;
			flow_req.rx_dest_qnum = rx_ring;
			flow_req.rx_src_tag_hi_sel = 2;
			flow_req.rx_src_tag_lo_sel = 4;
			flow_req.rx_dest_tag_hi_sel = 5;
			flow_req.rx_dest_tag_lo_sel = 4;
			flow_req.rx_fdq0_sz0_qnum = fd_ring;
			flow_req.rx_fdq1_qnum = fd_ring;
			flow_req.rx_fdq2_qnum = fd_ring;
			flow_req.rx_fdq3_qnum = fd_ring;

			ret = tisci_ops->rx_flow_cfg(tisci_rm->tisci,
						      &flow_req);

			if (ret) {
				dev_err(ud->dev, "flow%d config failed: %d\n",
					rchan->id, ret);
				goto err_chan_free;
			}

			uc->irq_ra_tisci = k3_nav_ringacc_get_tisci_dev_id(
								rchan->r_ring);
			uc->irq_ra_idx = rx_ring;
			uc->irq_udma_idx = 0x2000 + rchan->id;
		}
	}

	/* PSI-L pairing */
	uc->psi_link = k3_nav_psil_request_link(ud->psil_node, uc->src_thread,
						uc->dst_thread);
	if (IS_ERR(uc->psi_link)) {
		ret = PTR_ERR(uc->psi_link);
		goto err_chan_free;
	}

	/* Get the interrupts... */
	uc->irq_num_ring = ti_sci_inta_register_event(ud->dev, uc->irq_ra_tisci,
						      uc->irq_ra_idx, 0,
						      IRQF_TRIGGER_HIGH);
	if (uc->irq_num_ring <= 0) {
		dev_err(ud->dev, "Failed to get ring irq (index: %u) %d\n",
			uc->irq_ra_idx, uc->irq_num_ring);
		ret = -EINVAL;
		goto err_psi_free;
	}

	uc->irq_num_udma = ti_sci_inta_register_event(ud->dev,
						      tisci_rm->tisci_dev_id,
						      uc->irq_udma_idx, 0,
						      IRQF_TRIGGER_HIGH);
	if (uc->irq_num_udma <= 0) {
		dev_err(ud->dev, "Failed to get udma irq (index: %u) %d\n",
			uc->irq_udma_idx, uc->irq_num_udma);

		ti_sci_inta_unregister_event(ud->dev, uc->irq_ra_tisci,
					     uc->irq_ra_idx, uc->irq_num_ring);

		ret = -EINVAL;
		goto err_psi_free;
	}

	ret = request_irq(uc->irq_num_ring, udma_ring_irq_handler, 0, uc->name,
			  uc);
	if (ret) {
		dev_err(ud->dev, "%s: chan%d: Failed to request ring irq\n",
			__func__, uc->id);
		goto err_irq_free;
	}

	ret = request_irq(uc->irq_num_udma, udma_udma_irq_handler, 0, uc->name,
			  uc);
	if (ret) {
		dev_err(ud->dev, "%s: chan%d: Failed to request UDMA irq\n",
			__func__, uc->id);
		free_irq(uc->irq_num_ring, uc);
		goto err_irq_free;
	}

	udma_reset_rings(uc);

	return 0;

err_irq_free:
	ti_sci_inta_unregister_event(ud->dev, uc->irq_ra_tisci, uc->irq_ra_idx,
				     uc->irq_num_ring);
	uc->irq_num_ring = 0;

	ti_sci_inta_unregister_event(ud->dev, tisci_rm->tisci_dev_id,
				     uc->irq_udma_idx, uc->irq_num_udma);
	uc->irq_num_udma = 0;
err_psi_free:
	k3_nav_psil_release_link(uc->psi_link);
	uc->psi_link = NULL;
err_chan_free:
err_res_free:
	udma_free_tx_resources(uc);
	udma_free_rx_resources(uc);
	uc->slave_thread_id = -1;

	return ret;
}

static int udma_slave_config(struct dma_chan *chan,
			     struct dma_slave_config *cfg)
{
	struct udma_chan *uc = to_udma_chan(chan);

	memcpy(&uc->cfg, cfg, sizeof(uc->cfg));

	return 0;
}

static struct udma_desc *udma_alloc_tr_desc(struct udma_chan *uc,
					    size_t tr_size, int tr_count,
					    enum dma_transfer_direction dir)
{
	struct cppi50_tr_req_desc *tr_desc;
	struct udma_desc *d;
	u32 tr_nominal_size;

	switch (tr_size) {
	case 16:
		tr_nominal_size = CPPI50_TRDESC_W1_TR_SIZE_16;
		break;
	case 32:
		tr_nominal_size = CPPI50_TRDESC_W1_TR_SIZE_32;
		break;
	case 64:
		tr_nominal_size = CPPI50_TRDESC_W1_TR_SIZE_64;
		break;
	case 128:
		tr_nominal_size = CPPI50_TRDESC_W1_TR_SIZE_128;
		break;
	default:
		dev_err(uc->ud->dev, "Unsupported TR size of %zu\n", tr_size);
		return NULL;
	}

	d = kzalloc(sizeof(*d), GFP_ATOMIC);
	if (!d)
		return NULL;

	d->sglen = tr_count;
	d->cppi5_desc_area_size = sizeof(struct cppi50_tr_req_desc);
	d->cppi5_desc_area_size += tr_size * (tr_count + 1);
	d->cppi5_desc_area_size += tr_count * sizeof(struct cppi50_tr_resp);
	/* We have one descriptor with multiple TRs */
	d->cppi5_desc_size = d->cppi5_desc_area_size;

	/* Allocate memory for DMA ring descriptor */
	d->cppi5_desc_vaddr = dma_zalloc_coherent(uc->ud->dev,
						  d->cppi5_desc_area_size,
						  &d->cppi5_desc_paddr,
						  GFP_ATOMIC);
	if (!d->cppi5_desc_vaddr) {
		kfree(d);
		return NULL;
	}

	tr_desc = d->cppi5_desc_vaddr;

	/* Start of the TR req records */
	d->tr_req_base = d->cppi5_desc_vaddr + tr_size;
	/* Start address of the TR response array */
	d->tr_resp_base = d->tr_req_base + tr_size * tr_count;

	tr_desc->packet_info[0] = CPPI50_TRDESC_W0_LAST_ENTRY(tr_count - 1) |
				     CPPI50_TRDESC_W0_TYPE;
	if (uc->cyclic)
		tr_desc->packet_info[0] |= (0x1ff << 20);

	/* Flow and Packed ID ??? */
	tr_desc->packet_info[1] = tr_nominal_size |
				     CPPI50_TRDESC_W1_PACKETID(uc->id) |
				     CPPI50_TRDESC_W1_FLOWID(0x3fff); // ??

	/* TODO: re-check this... */
	if (dir == DMA_DEV_TO_MEM)
		tr_desc->packet_info[2] = k3_nav_ringacc_get_ring_id(
							uc->rchan->r_ring);
	else
		tr_desc->packet_info[2] = k3_nav_ringacc_get_ring_id(
							uc->tchan->tc_ring);

	return d;
}

static struct udma_desc *udma_prep_slave_sg_tr(
	struct udma_chan *uc, struct scatterlist *sgl, unsigned int sglen,
	enum dma_transfer_direction dir, unsigned long tx_flags, void *context)
{
	enum dma_slave_buswidth dev_width;
	struct scatterlist *sgent;
	struct udma_desc *d;
	size_t tr_size;
	struct cppi50_tr_req_type1 *tr_req = NULL;
	unsigned int i;
	u8 elsize;
	u32 burst;

	if (dir == DMA_DEV_TO_MEM) {
		dev_width = uc->cfg.src_addr_width;
		burst = uc->cfg.src_maxburst;
	} else if (dir == DMA_MEM_TO_DEV) {
		dev_width = uc->cfg.dst_addr_width;
		burst = uc->cfg.dst_maxburst;
	} else {
		dev_err(uc->ud->dev, "%s: bad direction?\n", __func__);
		return NULL;
	}

	/* Bus width translates to the element size (ES) */
	switch (dev_width) {
	case DMA_SLAVE_BUSWIDTH_1_BYTE:
		elsize = UDMA_ELSIZE_8;
		break;
	case DMA_SLAVE_BUSWIDTH_2_BYTES:
		elsize = UDMA_ELSIZE_16;
		break;
	case DMA_SLAVE_BUSWIDTH_3_BYTES:
		elsize = UDMA_ELSIZE_24;
		break;
	case DMA_SLAVE_BUSWIDTH_4_BYTES:
		elsize = UDMA_ELSIZE_32;
		break;
	case DMA_SLAVE_BUSWIDTH_8_BYTES:
		elsize = UDMA_ELSIZE_64;
		break;
	default: /* not reached */
		return NULL;
	}

	if (!burst)
		burst = 1;

	/* Now allocate and setup the descriptor. */
	tr_size = sizeof(struct cppi50_tr_req_type1);
	d = udma_alloc_tr_desc(uc, tr_size, sglen, dir);
	if (!d)
		return NULL;

	d->sglen = sglen;

	tr_req = (struct cppi50_tr_req_type1 *)d->tr_req_base;
	for_each_sg(sgl, sgent, sglen, i) {
		d->residue += sg_dma_len(sgent);
		tr_req[i].flags = CPPI50_TR_FLAGS_TYPE(1);
		tr_req[i].addr = sg_dma_address(sgent);
		tr_req[i].icnt0 = burst * elsize_bytes[elsize];
		tr_req[i].dim1 = burst * elsize_bytes[elsize];
		tr_req[i].icnt1 = sg_dma_len(sgent) / tr_req[i].icnt0;

		/* trigger types */
		tr_req[i].flags |= (3 << 10); /* TRIGGER0_TYPE */
		tr_req[i].flags |= (3 << 14); /* TRIGGER1_TYPE */

		tr_req[i].flags |= (1 << 26); /* suppress event */
	}

	tr_req[i - 1].flags |= (1 << 31); /* EOP */

	return d;
}

static inline int udma_configure_statictr(struct udma_chan *uc,
					  struct udma_desc *d, u8 elsize,
					  u16 elcnt)
{
	if (!uc->static_tr_type)
		return 0;

	d->static_tr.elsize = elsize;
	d->static_tr.elcnt = elcnt;
	if (uc->pkt_mode) {
		unsigned int div = elsize_bytes[elsize] * elcnt;

		if (uc->cyclic)
			d->static_tr.bstcnt = d->residue / d->sglen / div;
		else
			d->static_tr.bstcnt = d->residue / div;

		if (uc->dir == DMA_DEV_TO_MEM && d->static_tr.bstcnt > 0xfff)
			return -EINVAL;
	} else {
		d->static_tr.bstcnt = 0;
	}

	return 0;
}

static struct udma_desc *udma_prep_slave_sg_pkt(
	struct udma_chan *uc, struct scatterlist *sgl, unsigned int sglen,
	enum dma_transfer_direction dir, unsigned long tx_flags, void *context)
{
	struct scatterlist *sgent;
	struct knav_udmap_host_desc_t *h_desc;
	unsigned int hdesc_size = sizeof(struct knav_udmap_host_desc_t);
	struct udma_desc *d;
	u32 ring_id;
	unsigned int i;

	d = kzalloc(sizeof(*d), GFP_ATOMIC);
	if (!d)
		return NULL;

	if (dir == DMA_DEV_TO_MEM && sglen > 1) {
		void *buffer;
		int ret;
		size_t total_len = 0;

		/* Count the total length of the receive SG buffer */
		for_each_sg(sgl, sgent, sglen, i)
			total_len += sg_dma_len(sgent);

		buffer = kzalloc(total_len, GFP_ATOMIC);
		if (!buffer) {
			kfree(d);
			return NULL;
		}

		sg_init_table(&d->rx_sg_wa.single_sg, 1);
		sg_set_buf(&d->rx_sg_wa.single_sg, buffer, total_len);
		ret = dma_map_sg(uc->ud->dev, &d->rx_sg_wa.single_sg, 1,
				 DMA_FROM_DEVICE);
		if (ret != 1) {
			dev_err(uc->ud->dev,
				"mapping of temp buffer error (%d)\n", ret);
			kfree(buffer);
			kfree(d);
			return NULL;
		}

		d->rx_sg_wa.in_use = true;

		d->rx_sg_wa.sgl = sgl;
		d->rx_sg_wa.sglen = sglen;
		d->rx_sg_wa.total_len = total_len;

		sgl = &d->rx_sg_wa.single_sg;
		sglen = 1;
	}

	/* We need to allocate space for EPIB/PSD */
	hdesc_size += uc->metadata_size;
	hdesc_size = ALIGN(hdesc_size, 0x10);

	d->cppi5_desc_size = hdesc_size;
	d->cppi5_desc_area_size = hdesc_size * sglen;
	d->sglen = sglen;

	/* Allocate memory for DMA ring descriptor */
	d->cppi5_desc_vaddr = dma_zalloc_coherent(uc->ud->dev,
						  d->cppi5_desc_area_size,
						  &d->cppi5_desc_paddr,
						  GFP_ATOMIC);
	if (!d->cppi5_desc_vaddr) {
		if (d->rx_sg_wa.in_use) {
			dma_unmap_sg(uc->ud->dev, &d->rx_sg_wa.single_sg, 1,
				     DMA_FROM_DEVICE);
			kfree(sg_virt(&d->rx_sg_wa.single_sg));
		}
		kfree(d);
		return NULL;
	}

	if (dir == DMA_DEV_TO_MEM)
		ring_id = k3_nav_ringacc_get_ring_id(uc->rchan->r_ring);
	else
		ring_id = k3_nav_ringacc_get_ring_id(uc->tchan->tc_ring);

	for_each_sg(sgl, sgent, sglen, i) {
		dma_addr_t sg_addr = sg_dma_address(sgent);
		dma_addr_t bdesc_paddr = d->cppi5_desc_paddr + hdesc_size * i;
		struct knav_udmap_host_desc_t *desc =
					d->cppi5_desc_vaddr + hdesc_size * i;
		size_t sg_len = sg_dma_len(sgent);

		d->residue += sg_len;

		if (i == 0) {
			knav_udmap_hdesc_init(desc, 0, 0);
			/* Flow and Packed ID ??? */
			knav_udmap_hdesc_set_pktids(&desc->hdr, uc->id, 0x3fff);
			knav_udmap_desc_set_retpolicy(&desc->hdr, 0, ring_id);
		} else {
			knav_udmap_hdesc_reset_hbdesc(desc);
			knav_udmap_desc_set_retpolicy(&desc->hdr, 0, 0xffff);
		}

		/* attach the sg buffer to the descriptor */
		knav_udmap_hdesc_attach_buf(desc,
					    sg_addr, sg_len,
					    sg_addr, sg_len);

		/* Attach link as host buffer descriptor */
		if (i)
			knav_udmap_hdesc_link_hbdesc(h_desc, bdesc_paddr);

		h_desc = desc;
	}

	if (d->residue > 0x3FFFFF)
		dev_err(uc->ud->dev, "%s: transfer size is too big: %u...\n",
			__func__, d->residue);

	h_desc = d->cppi5_desc_vaddr;
	knav_udmap_hdesc_set_pktlen(h_desc, d->residue);

	return d;
}

static int udma_attach_metadata(struct dma_async_tx_descriptor *desc,
				void *data, size_t len)
{
	struct udma_desc *d = to_udma_desc(desc);
	struct udma_chan *uc = to_udma_chan(desc->chan);
	struct knav_udmap_host_desc_t *h_desc;
	u32 psd_size = len;
	u32 flags = 0;

	if (!uc->pkt_mode || !uc->metadata_size)
		return -ENOTSUPP;

	if (!data || len > uc->metadata_size)
		return -EINVAL;

	if (uc->needs_epib && len < 16)
		return -EINVAL;

	h_desc = d->cppi5_desc_vaddr;
	if (d->dir == DMA_MEM_TO_DEV)
		memcpy(h_desc->epib, data, len);

	if (uc->needs_epib)
		psd_size -= 16;

	d->metadata = data;
	d->metadata_size = len;
	if (uc->needs_epib)
		flags |= KNAV_UDMAP_INFO0_HDESC_EPIB_PRESENT;

	knav_udmap_hdesc_update_flags(h_desc, flags);
	knav_udmap_hdesc_update_psdata_size(h_desc, psd_size);

	return 0;
}

static void *udma_get_metadata_ptr(struct dma_async_tx_descriptor *desc,
				   size_t *payload_len, size_t *max_len)
{
	struct udma_desc *d = to_udma_desc(desc);
	struct udma_chan *uc = to_udma_chan(desc->chan);
	struct knav_udmap_host_desc_t *h_desc;

	if (!uc->pkt_mode || !uc->metadata_size)
		return ERR_PTR(-ENOTSUPP);

	h_desc = d->cppi5_desc_vaddr;

	*max_len = uc->metadata_size;

	*payload_len = knav_udmap_desc_is_epib_present(&h_desc->hdr) ? 16 : 0;
	*payload_len += knav_udmap_hdesc_get_psdata_size(h_desc);

	return h_desc->epib;
}

static int udma_set_metadata_len(struct dma_async_tx_descriptor *desc,
				 size_t payload_len)
{
	struct udma_desc *d = to_udma_desc(desc);
	struct udma_chan *uc = to_udma_chan(desc->chan);
	struct knav_udmap_host_desc_t *h_desc;
	u32 psd_size = payload_len;
	u32 flags = 0;

	if (!uc->pkt_mode || !uc->metadata_size)
		return -ENOTSUPP;

	if (payload_len > uc->metadata_size)
		return -EINVAL;

	if (uc->needs_epib && payload_len < 16)
		return -EINVAL;

	h_desc = d->cppi5_desc_vaddr;

	if (uc->needs_epib) {
		psd_size -= 16;
		flags |= KNAV_UDMAP_INFO0_HDESC_EPIB_PRESENT;
	}

	knav_udmap_hdesc_update_flags(h_desc, flags);
	knav_udmap_hdesc_update_psdata_size(h_desc, psd_size);

	return 0;
}

static struct dma_descriptor_metadata_ops metadata_ops = {
	.attach = udma_attach_metadata,
	.get_ptr = udma_get_metadata_ptr,
	.set_len = udma_set_metadata_len,
};

static struct dma_async_tx_descriptor *udma_prep_slave_sg(
	struct dma_chan *chan, struct scatterlist *sgl, unsigned int sglen,
	enum dma_transfer_direction dir, unsigned long tx_flags, void *context)
{
	struct udma_chan *uc = to_udma_chan(chan);
	enum dma_slave_buswidth dev_width;
	struct udma_desc *d;
	u8 elsize;
	u32 burst;

	if (dir != uc->dir) {
		dev_err(chan->device->dev,
			"%s: chan%d is for %s, not supporting %s\n",
			__func__, uc->id, udma_get_dir_text(uc->dir),
			udma_get_dir_text(dir));
		return NULL;
	}

	if (dir == DMA_DEV_TO_MEM) {
		dev_width = uc->cfg.src_addr_width;
		burst = uc->cfg.src_maxburst;
	} else if (dir == DMA_MEM_TO_DEV) {
		dev_width = uc->cfg.dst_addr_width;
		burst = uc->cfg.dst_maxburst;
	} else {
		dev_err(chan->device->dev, "%s: bad direction?\n", __func__);
		return NULL;
	}

	/* Bus width translates to the element size (ES) */
	switch (dev_width) {
	case DMA_SLAVE_BUSWIDTH_1_BYTE:
		elsize = UDMA_ELSIZE_8;
		break;
	case DMA_SLAVE_BUSWIDTH_2_BYTES:
		elsize = UDMA_ELSIZE_16;
		break;
	case DMA_SLAVE_BUSWIDTH_3_BYTES:
		elsize = UDMA_ELSIZE_24;
		break;
	case DMA_SLAVE_BUSWIDTH_4_BYTES:
		elsize = UDMA_ELSIZE_32;
		break;
	case DMA_SLAVE_BUSWIDTH_8_BYTES:
		elsize = UDMA_ELSIZE_64;
		break;
	default: /* not reached */
		return NULL;
	}

	if (!burst)
		burst = 1;

	if (uc->pkt_mode)
		d = udma_prep_slave_sg_pkt(uc, sgl, sglen, dir, tx_flags,
					   context);
	else
		d = udma_prep_slave_sg_tr(uc, sgl, sglen, dir, tx_flags,
					  context);

	if (!d)
		return NULL;

	d->dir = dir;
	d->desc_idx = 0;
	d->tr_idx = 0;

	/* static TR for remote PDMA */
	if (udma_configure_statictr(uc, d, elsize, burst)) {
		dev_err(uc->ud->dev,
			"%s: StaticTR Z is limted to maximum 4095 (%u)\n",
			__func__, d->static_tr.bstcnt);

		dma_free_coherent(uc->ud->dev, d->cppi5_desc_area_size,
				  d->cppi5_desc_vaddr, d->cppi5_desc_paddr);

		if (d->rx_sg_wa.in_use) {
			dma_unmap_sg(uc->ud->dev, &d->rx_sg_wa.single_sg, 1,
				     DMA_FROM_DEVICE);
			kfree(sg_virt(&d->rx_sg_wa.single_sg));
		}

		kfree(d);
		return NULL;
	}

	if (uc->metadata_size)
		d->vd.tx.metadata_ops = &metadata_ops;

	return vchan_tx_prep(&uc->vc, &d->vd, tx_flags);
}

static struct udma_desc *udma_prep_dma_cyclic_tr(
	struct udma_chan *uc, dma_addr_t buf_addr, size_t buf_len,
	size_t period_len, enum dma_transfer_direction dir, unsigned long flags)
{
	enum dma_slave_buswidth dev_width;
	struct udma_desc *d;
	size_t tr_size;
	struct cppi50_tr_req_type1 *tr_req;
	unsigned int i;
	unsigned int periods = buf_len / period_len;
	u32 burst;

	if (dir == DMA_DEV_TO_MEM) {
		dev_width = uc->cfg.src_addr_width;
		burst = uc->cfg.src_maxburst;
	} else if (dir == DMA_MEM_TO_DEV) {
		dev_width = uc->cfg.dst_addr_width;
		burst = uc->cfg.dst_maxburst;
	} else {
		dev_err(uc->ud->dev, "%s: bad direction?\n", __func__);
		return NULL;
	}

	if (!burst)
		burst = 1;

	/* Now allocate and setup the descriptor. */
	tr_size = sizeof(struct cppi50_tr_req_type1);
	d = udma_alloc_tr_desc(uc, tr_size, periods, dir);
	if (!d)
		return NULL;

	tr_req = (struct cppi50_tr_req_type1 *)d->tr_req_base;
	for (i = 0; i < periods; i++) {
		tr_req[i].flags = CPPI50_TR_FLAGS_TYPE(1);
		tr_req[i].addr = buf_addr + period_len * i;
		tr_req[i].icnt0 = dev_width;
		tr_req[i].icnt1 = period_len / dev_width;
		tr_req[i].dim1 = dev_width;

		tr_req[i].flags |= CPPI50_TR_FLAGS_EVENT_COMPLETED;
		tr_req[i].flags |= (3 << 10); /* TRIGGER0_TYPE */
		tr_req[i].flags |= (3 << 14); /* TRIGGER1_TYPE */

		if (!(flags & DMA_PREP_INTERRUPT))
			tr_req[i].flags |= (1 << 26); /* suppress event output */
	}

	return d;
}

static struct udma_desc *udma_prep_dma_cyclic_pkt(
	struct udma_chan *uc, dma_addr_t buf_addr, size_t buf_len,
	size_t period_len, enum dma_transfer_direction dir, unsigned long flags)
{
	unsigned int hdesc_size = sizeof(struct knav_udmap_host_desc_t);
	struct udma_desc *d;
	u32 ring_id;
	int i;
	int periods = buf_len / period_len;

	if (periods > 15)
		return NULL;

	if (period_len > 0x3FFFFF)
		return NULL;

	d = kzalloc(sizeof(*d), GFP_ATOMIC);
	if (!d)
		return NULL;

	/* We need to allocate space for EPIB/PSD */
	hdesc_size += uc->metadata_size;
	hdesc_size = ALIGN(hdesc_size, 0x10);

	d->cppi5_desc_size = hdesc_size;
	d->cppi5_desc_area_size = hdesc_size * periods;

	/* Allocate memory for DMA ring descriptor */
	d->cppi5_desc_vaddr = dma_zalloc_coherent(uc->ud->dev,
						  d->cppi5_desc_area_size,
						  &d->cppi5_desc_paddr,
						  GFP_ATOMIC);
	if (!d->cppi5_desc_vaddr) {
		kfree(d);
		return NULL;
	}

	/* TODO: re-check this... */
	if (dir == DMA_DEV_TO_MEM)
		ring_id = k3_nav_ringacc_get_ring_id(uc->rchan->r_ring);
	else
		ring_id = k3_nav_ringacc_get_ring_id(uc->tchan->tc_ring);

	for (i = 0; i < periods; i++) {
		dma_addr_t period_addr = buf_addr + (period_len * i);
		struct knav_udmap_host_desc_t *h_desc =
					d->cppi5_desc_vaddr + hdesc_size * i;

		knav_udmap_hdesc_init(h_desc, 0, 0);
		knav_udmap_hdesc_set_pktlen(h_desc, period_len);

		/* Flow and Packed ID ??? */
		knav_udmap_hdesc_set_pktids(&h_desc->hdr, uc->id, 0x3fff);
		knav_udmap_desc_set_retpolicy(&h_desc->hdr, 0, ring_id);

		/* attach each period to a new descriptor */
		knav_udmap_hdesc_attach_buf(h_desc,
					    period_addr, period_len,
					    period_addr, period_len);
	}

	return d;
}

static struct dma_async_tx_descriptor *udma_prep_dma_cyclic(
	struct dma_chan *chan, dma_addr_t buf_addr, size_t buf_len,
	size_t period_len, enum dma_transfer_direction dir, unsigned long flags)
{
	struct udma_chan *uc = to_udma_chan(chan);
	enum dma_slave_buswidth dev_width;
	struct udma_desc *d;
	u8 elsize;
	u32 burst;

	if (dir != uc->dir) {
		dev_err(chan->device->dev,
			"%s: chan%d is for %s, not supporting %s\n",
			__func__, uc->id, udma_get_dir_text(uc->dir),
			udma_get_dir_text(dir));
		return NULL;
	}

	uc->cyclic = true;

	if (dir == DMA_DEV_TO_MEM) {
		dev_width = uc->cfg.src_addr_width;
		burst = uc->cfg.src_maxburst;
	} else if (dir == DMA_MEM_TO_DEV) {
		dev_width = uc->cfg.dst_addr_width;
		burst = uc->cfg.dst_maxburst;
	} else {
		dev_err(uc->ud->dev, "%s: bad direction?\n", __func__);
		return NULL;
	}

	if (!burst)
		burst = 1;

	/* Bus width translates to the element size (ES) */
	switch (dev_width) {
	case DMA_SLAVE_BUSWIDTH_1_BYTE:
		elsize = UDMA_ELSIZE_8;
		break;
	case DMA_SLAVE_BUSWIDTH_2_BYTES:
		elsize = UDMA_ELSIZE_16;
		break;
	case DMA_SLAVE_BUSWIDTH_3_BYTES:
		elsize = UDMA_ELSIZE_24;
		break;
	case DMA_SLAVE_BUSWIDTH_4_BYTES:
		elsize = UDMA_ELSIZE_32;
		break;
	case DMA_SLAVE_BUSWIDTH_8_BYTES:
		elsize = UDMA_ELSIZE_64;
		break;
	default: /* not reached */
		return NULL;
	}

	if (uc->pkt_mode)
		d = udma_prep_dma_cyclic_pkt(uc, buf_addr, buf_len, period_len,
					     dir, flags);
	else
		d = udma_prep_dma_cyclic_tr(uc, buf_addr, buf_len, period_len,
					    dir, flags);

	if (!d)
		return NULL;

	d->sglen = buf_len / period_len;

	d->dir = dir;
	d->residue = buf_len;

	/* static TR for remote PDMA */
	if (udma_configure_statictr(uc, d, elsize, burst)) {
		dev_err(uc->ud->dev,
			"%s: StaticTR Z is limted to maximum 4095 (%u)\n",
			__func__, d->static_tr.bstcnt);

		dma_free_coherent(uc->ud->dev, d->cppi5_desc_area_size,
				  d->cppi5_desc_vaddr, d->cppi5_desc_paddr);
		kfree(d);
		return NULL;
	}

	if (uc->metadata_size)
		d->vd.tx.metadata_ops = &metadata_ops;

	return vchan_tx_prep(&uc->vc, &d->vd, flags);
}

static struct dma_async_tx_descriptor *udma_prep_dma_memcpy(
	struct dma_chan *chan, dma_addr_t dest, dma_addr_t src,
	size_t len, unsigned long tx_flags)
{
	struct udma_chan *uc = to_udma_chan(chan);
	struct udma_desc *d;
	struct cppi50_tr_req_type15 *tr_req;
	int num_tr;
	size_t tr_size = sizeof(struct cppi50_tr_req_type15);
	u16 tr0_cnt0, tr0_cnt1, tr1_cnt0;

	if (uc->dir != DMA_MEM_TO_MEM) {
		dev_err(chan->device->dev,
			"%s: chan%d is for %s, not supporting %s\n",
			__func__, uc->id, udma_get_dir_text(uc->dir),
			udma_get_dir_text(DMA_MEM_TO_MEM));
		return NULL;
	}

	if (len < SZ_64K) {
		num_tr = 1;
		tr0_cnt0 = len;
		tr0_cnt1 = 1;
	} else {
		unsigned long align_to = __ffs(src | dest);

		if (align_to > 3)
			align_to = 3;
		/*
		 * Keep simple: tr0: SZ_64K-alignment blocks,
		 *		tr1: the remaining
		 */
		num_tr = 2;
		tr0_cnt0 = (SZ_64K - BIT(align_to));
		if (len / tr0_cnt0 >= SZ_64K) {
			dev_err(uc->ud->dev, "size %zu is not supported\n",
				len);
			return NULL;
		}

		tr0_cnt1 = len / tr0_cnt0;
		tr1_cnt0 = len % tr0_cnt0;
	}

	d = udma_alloc_tr_desc(uc, tr_size, num_tr, DMA_MEM_TO_MEM);
	if (!d)
		return NULL;

	d->dir = DMA_MEM_TO_MEM;
	d->desc_idx = 0;
	d->tr_idx = 0;
	d->residue = len;

	tr_req = (struct cppi50_tr_req_type15 *)d->tr_req_base;

	tr_req[0].flags = CPPI50_TR_FLAGS_TYPE(15);
	tr_req[0].addr = src;
	tr_req[0].icnt0 = tr0_cnt0;
	tr_req[0].icnt1 = tr0_cnt1;
	tr_req[0].icnt2 = 1;
	tr_req[0].icnt3 = 1;
	tr_req[0].dim1 = tr0_cnt0;

	tr_req[0].daddr = dest;
	tr_req[0].dicnt0 = tr0_cnt0;
	tr_req[0].dicnt1 = tr0_cnt1;
	tr_req[0].dicnt2 = 1;
	tr_req[0].dicnt3 = 1;
	tr_req[0].ddim1 = tr0_cnt0;

	tr_req[0].flags |= (1 << 5); /* WAIT */

	/* trigger types */
	tr_req[0].flags |= (3 << 10); /* TRIGGER0_TYPE */
	tr_req[0].flags |= (3 << 14); /* TRIGGER1_TYPE */

	tr_req[0].flags |= (1 << 26); /* suppress event */

	tr_req[0].flags |= (0x25 << 16); /* CMD_ID */

	if (num_tr == 2) {
		tr_req[1].flags = CPPI50_TR_FLAGS_TYPE(15);
		tr_req[1].addr = src + tr0_cnt1 * tr0_cnt0;
		tr_req[1].icnt0 = tr1_cnt0;
		tr_req[1].icnt1 = 1;
		tr_req[1].icnt2 = 1;
		tr_req[1].icnt3 = 1;

		tr_req[1].daddr = dest + tr0_cnt1 * tr0_cnt0;
		tr_req[1].dicnt0 = tr1_cnt0;
		tr_req[1].dicnt1 = 1;
		tr_req[1].dicnt2 = 1;
		tr_req[1].dicnt3 = 1;

		tr_req[1].flags |= (1 << 5); /* WAIT */

		/* trigger types */
		tr_req[1].flags |= (3 << 10); /* TRIGGER0_TYPE */
		tr_req[1].flags |= (3 << 14); /* TRIGGER1_TYPE */

		tr_req[1].flags |= (0x25 << 16); /* CMD_ID */
		tr_req[1].flags |= (1 << 26); /* suppress event */
	}

	tr_req[num_tr - 1].flags |= (1 << 31); /* EOP */

	if (uc->metadata_size)
		d->vd.tx.metadata_ops = &metadata_ops;

	return vchan_tx_prep(&uc->vc, &d->vd, tx_flags);
}

static void udma_issue_pending(struct dma_chan *chan)
{
	struct udma_chan *uc = to_udma_chan(chan);
	unsigned long flags;

	spin_lock_irqsave(&uc->vc.lock, flags);

	/* If we have something pending and no active descriptor, then */
	if (vchan_issue_pending(&uc->vc) && !uc->desc) {
		/*
		 * start a descriptor if the channel is NOT [marked as
		 * terminating _and_ it is still running (teardown has not
		 * completed yet)].
		 */
		if (!(uc->state == UDMA_CHAN_IS_TERMINATING &&
		      udma_is_chan_running(uc)))
			udma_start(uc);
	}

	spin_unlock_irqrestore(&uc->vc.lock, flags);
}

/* Not much yet */
static enum dma_status udma_tx_status(struct dma_chan *chan,
				      dma_cookie_t cookie,
				      struct dma_tx_state *txstate)
{
	struct udma_chan *uc = to_udma_chan(chan);
	enum dma_status ret;

	ret = dma_cookie_status(chan, cookie, txstate);

	if (!udma_is_chan_running(uc))
		ret = DMA_COMPLETE;

	if (ret == DMA_COMPLETE || !txstate)
		return ret;

	if (uc->desc && uc->desc->vd.tx.cookie == cookie) {
		u32 pdma_bcnt = 0;
		u32 bcnt = 0;
		u32 pcnt = 0;
		u32 residue = uc->desc->residue;
		u32 delay = 0;

		if (uc->desc->dir == DMA_MEM_TO_DEV) {
			bcnt = udma_tchanrt_read(uc->tchan,
						 UDMA_TCHAN_RT_BCNT_REG);
			pdma_bcnt = udma_tchanrt_read(uc->tchan,
						UDMA_TCHAN_RT_PEER_BCNT_REG);
			pcnt = udma_tchanrt_read(uc->tchan,
						 UDMA_TCHAN_RT_PCNT_REG);

			if (bcnt > pdma_bcnt)
				delay = bcnt - pdma_bcnt;
		} else if (uc->desc->dir == DMA_DEV_TO_MEM) {
			bcnt = udma_rchanrt_read(uc->rchan,
						 UDMA_RCHAN_RT_SBCNT_REG);
			pdma_bcnt = udma_rchanrt_read(uc->rchan,
						UDMA_RCHAN_RT_PEER_BCNT_REG);
			pcnt = udma_rchanrt_read(uc->rchan,
						 UDMA_RCHAN_RT_PCNT_REG);

			if (pdma_bcnt > bcnt)
				delay = pdma_bcnt - bcnt;
		} else {
			u32 sbcnt;

			sbcnt = udma_tchanrt_read(uc->tchan,
						  UDMA_TCHAN_RT_SBCNT_REG);
			bcnt = udma_tchanrt_read(uc->tchan,
						 UDMA_TCHAN_RT_PEER_BCNT_REG);
			pcnt = udma_tchanrt_read(uc->tchan,
						 UDMA_TCHAN_RT_PCNT_REG);

			if (sbcnt > bcnt)
				delay = sbcnt - bcnt;
		}

		residue -= ((bcnt - uc->bcnt) % uc->desc->residue);
		dma_set_residue(txstate, residue);
		dma_set_cached(txstate, delay);

	} else {
		ret = DMA_COMPLETE;
	}

	return ret;
}

static int udma_pause(struct dma_chan *chan)
{
	struct udma_chan *uc = to_udma_chan(chan);

	if (!uc->desc)
		return -EINVAL;

	/* pause the channel */
	switch (uc->desc->dir) {
	case DMA_DEV_TO_MEM:
		udma_rchanrt_update_bits(uc->rchan,
					 UDMA_RCHAN_RT_PEER_RT_EN_REG,
					 UDMA_PEER_RT_EN_PAUSE,
					 UDMA_PEER_RT_EN_PAUSE);
		break;
	case DMA_MEM_TO_DEV:
		udma_tchanrt_update_bits(uc->tchan,
					 UDMA_TCHAN_RT_PEER_RT_EN_REG,
					 UDMA_PEER_RT_EN_PAUSE,
					 UDMA_PEER_RT_EN_PAUSE);
		break;
	case DMA_MEM_TO_MEM:
		udma_tchanrt_update_bits(uc->tchan, UDMA_TCHAN_RT_CTL_REG,
					 UDMA_CHAN_RT_CTL_PAUSE,
					 UDMA_CHAN_RT_CTL_PAUSE);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int udma_resume(struct dma_chan *chan)
{
	struct udma_chan *uc = to_udma_chan(chan);

	if (!uc->desc)
		return -EINVAL;

	/* resume the channel */
	switch (uc->desc->dir) {
	case DMA_DEV_TO_MEM:
		udma_rchanrt_update_bits(uc->rchan,
					 UDMA_RCHAN_RT_PEER_RT_EN_REG,
					 UDMA_PEER_RT_EN_PAUSE, 0);

		break;
	case DMA_MEM_TO_DEV:
		udma_tchanrt_update_bits(uc->tchan,
					 UDMA_TCHAN_RT_PEER_RT_EN_REG,
					 UDMA_PEER_RT_EN_PAUSE, 0);
		break;
	case DMA_MEM_TO_MEM:
		udma_tchanrt_update_bits(uc->tchan, UDMA_TCHAN_RT_CTL_REG,
					 UDMA_CHAN_RT_CTL_PAUSE, 0);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int udma_terminate_all(struct dma_chan *chan)
{
	struct udma_chan *uc = to_udma_chan(chan);
	unsigned long flags;
	LIST_HEAD(head);

	spin_lock_irqsave(&uc->vc.lock, flags);

	if (udma_is_chan_running(uc))
		udma_stop(uc);

	if (uc->desc) {
		uc->bcnt += uc->desc->residue;
		uc->terminated_desc = uc->desc;
		uc->desc = NULL;
		uc->terminated_desc->terminated = true;
	}

	uc->paused = false;

	vchan_get_all_descriptors(&uc->vc, &head);
	spin_unlock_irqrestore(&uc->vc.lock, flags);
	vchan_dma_desc_free_list(&uc->vc, &head);

	return 0;
}

static void udma_synchronize(struct dma_chan *chan)
{
	struct udma_chan *uc = to_udma_chan(chan);
	unsigned long timeout = msecs_to_jiffies(1000);

	vchan_synchronize(&uc->vc);

	if (uc->state == UDMA_CHAN_IS_TERMINATING) {
		timeout = wait_for_completion_timeout(&uc->teardown_completed,
						      timeout);
		if (!timeout) {
			dev_warn(uc->ud->dev, "chan%d teardown timeout!\n",
				 uc->id);
			udma_dump_chan_stdata(uc);
			udma_reset_chan(uc, true);
		}
	}

	udma_reset_chan(uc, false);
	if (udma_is_chan_running(uc))
		dev_warn(uc->ud->dev, "chan%d refused to stop!\n", uc->id);

	udma_reset_rings(uc);
}

static void udma_desc_pre_callback(struct virt_dma_chan *vc,
				   struct virt_dma_desc *vd,
				   struct dmaengine_result *result)
{
	struct udma_chan *uc = to_udma_chan(&vc->chan);
	struct udma_desc *d;

	if (!vd)
		return;

	d = to_udma_desc(&vd->tx);

	if (d->metadata_size)
		udma_fetch_epib(uc, d);

	/* TODO: peek into the desc to know the real length */
	if (d->rx_sg_wa.in_use) {
		void *src = sg_virt(&d->rx_sg_wa.single_sg);

		dma_sync_sg_for_cpu(uc->ud->dev, &d->rx_sg_wa.single_sg, 1,
				    DMA_FROM_DEVICE);
		/* Ensure that reads are not moved before this point */
		rmb();

		sg_copy_from_buffer(d->rx_sg_wa.sgl, d->rx_sg_wa.sglen, src,
				    d->rx_sg_wa.total_len);
	}

	/* Provide residue information for the client */
	if (result) {
		void *desc_vaddr = udma_curr_cppi5_desc_vaddr(d, d->desc_idx);

		if (knav_udmap_desc_get_type(desc_vaddr) ==
			KNAV_UDMAP_INFO0_DESC_TYPE_VAL_HOST) {
			result->residue = knav_udmap_hdesc_get_pktlen(desc_vaddr);
			if (result->residue == d->residue)
				result->result = DMA_TRANS_NOERROR;
			else
				result->result = DMA_TRANS_ABORTED;
		} else {
			result->residue = d->residue;
			result->result = DMA_TRANS_NOERROR;
		}
	}
}

/*
 * This tasklet handles the completion of a DMA descriptor by
 * calling its callback and freeing it.
 */
static void udma_vchan_complete(unsigned long arg)
{
	struct virt_dma_chan *vc = (struct virt_dma_chan *)arg;
	struct virt_dma_desc *vd, *_vd;
	struct dmaengine_desc_callback cb;
	LIST_HEAD(head);

	spin_lock_irq(&vc->lock);
	list_splice_tail_init(&vc->desc_completed, &head);
	vd = vc->cyclic;
	if (vd) {
		vc->cyclic = NULL;
		dmaengine_desc_get_callback(&vd->tx, &cb);
	} else {
		memset(&cb, 0, sizeof(cb));
	}
	spin_unlock_irq(&vc->lock);

	udma_desc_pre_callback(vc, vd, NULL);
	dmaengine_desc_callback_invoke(&cb, NULL);

	list_for_each_entry_safe(vd, _vd, &head, node) {
		struct dmaengine_result result;

		dmaengine_desc_get_callback(&vd->tx, &cb);

		list_del(&vd->node);

		udma_desc_pre_callback(vc, vd, &result);
		dmaengine_desc_callback_invoke(&cb, &result);

		vchan_vdesc_fini(vd);
	}
}

static void udma_free_chan_resources(struct dma_chan *chan)
{
	struct udma_chan *uc = to_udma_chan(chan);
	struct udma_dev *ud = to_udma_dev(chan->device);
	struct udma_tisci_rm *tisci_rm = &ud->tisci_rm;

	udma_terminate_all(chan);

	if (uc->irq_num_ring > 0) {
		free_irq(uc->irq_num_ring, uc);

		ti_sci_inta_unregister_event(ud->dev, uc->irq_ra_tisci,
					     uc->irq_ra_idx, uc->irq_num_ring);
		uc->irq_num_ring = 0;
	}
	if (uc->irq_num_udma > 0) {
		free_irq(uc->irq_num_udma, uc);

		ti_sci_inta_unregister_event(ud->dev, tisci_rm->tisci_dev_id,
					     uc->irq_udma_idx,
					     uc->irq_num_udma);
		uc->irq_num_udma = 0;
	}

	/* Release PSI-L pairing */
	if (uc->psi_link)
		k3_nav_psil_release_link(uc->psi_link);

	vchan_free_chan_resources(&uc->vc);
	tasklet_kill(&uc->vc.task);

	pm_runtime_put(ud->ddev.dev);

	udma_free_tx_resources(uc);
	udma_free_rx_resources(uc);

	uc->slave_thread_id = -1;
	uc->dir = DMA_MEM_TO_MEM;
}

static struct platform_driver udma_driver;

static bool udma_dma_filter_fn(struct dma_chan *chan, void *param)
{
	u32 *args;
	struct udma_chan *uc;
	struct udma_dev *ud;
	struct device_node *chconf_node, *slave_node;
	char prop[50];
	u32 val;

	if (chan->device->dev->driver != &udma_driver.driver)
		return false;

	uc = to_udma_chan(chan);
	ud = uc->ud;
	args = param;

	if (args[2] == UDMA_DIR_TX) {
		uc->dir = DMA_MEM_TO_DEV;
	} else if (args[2] == UDMA_DIR_RX) {
		uc->dir = DMA_DEV_TO_MEM;
	} else {
		dev_err(ud->dev, "Invalid direction (%u)\n", args[2]);
		return false;
	}

	slave_node = of_find_node_by_phandle(args[0]);
	if (!slave_node) {
		dev_err(ud->dev, "Slave node is missing\n");
		return false;
	}

	snprintf(prop, sizeof(prop), "ti,psil-config%u", args[1]);
	chconf_node = of_find_node_by_name(slave_node, prop);
	if (!chconf_node) {
		dev_err(ud->dev, "Channel configuration node is missing\n");
		of_node_put(slave_node);
		return false;
	}

	if (!of_property_read_u32(chconf_node, "linux,udma-mode", &val)) {
		if (val == UDMA_PKT_MODE)
			uc->pkt_mode = true;
	}

	if (!of_property_read_u32(chconf_node, "statictr-type", &val))
		uc->static_tr_type = val;

	if (!of_property_read_u32(chconf_node, "ti,channel-tpl", &val))
		uc->channel_tpl = val;

	uc->needs_epib = of_property_read_bool(chconf_node, "ti,needs-epib");
	if (!of_property_read_u32(chconf_node, "ti,psd-size", &val))
		uc->psd_size = val;
	uc->metadata_size = (uc->needs_epib ? 16 : 0) + uc->psd_size;

	of_node_put(chconf_node);

	if (of_property_read_u32(slave_node, "ti,psil-base", &val)) {
		dev_err(ud->dev, "ti,psil-base is missing\n");
		of_node_put(slave_node);
		return false;
	}

	uc->slave_thread_id = val + args[1];

	of_node_put(slave_node);

	dev_dbg(ud->dev, "%s: Slave %s thread%d will be handled by vchan %d\n",
		__func__, udma_get_dir_text(uc->dir), uc->slave_thread_id,
		uc->id);

	return true;
}

static struct dma_chan *udma_of_xlate(struct of_phandle_args *dma_spec,
				      struct of_dma *ofdma)
{
	struct udma_dev *ud = ofdma->of_dma_data;
	struct dma_chan *chan;

	if (dma_spec->args_count != 3)
		return NULL;

	chan = dmadev_get_slave_channel(&ud->ddev, udma_dma_filter_fn,
					&dma_spec->args[0]);
	if (!chan) {
		dev_err(ud->dev, "get channel fail in %s.\n", __func__);
		return ERR_PTR(-EINVAL);
	}

	return chan;
}

struct udma_match_data am654_main_data = {
	.tpl_levels = 2,
	.level_start_idx = {
		[0] = 8, /* Normal channels */
		[1] = 0, /* High Throughput channels */
	},
};

struct udma_match_data am654_mcu_data = {
	.tpl_levels = 2,
	.level_start_idx = {
		[0] = 2, /* Normal channels */
		[1] = 0, /* High Throughput channels */
	},
};

static const struct of_device_id udma_of_match[] = {
	{ .compatible = "ti,am654-navss-main-udmap", .data = &am654_main_data, },
	{ .compatible = "ti,am654-navss-mcu-udmap", .data = &am654_mcu_data, },
	{},
};
MODULE_DEVICE_TABLE(of, udma_of_match);

static int udma_get_mmrs(struct platform_device *pdev, struct udma_dev *ud)
{
	struct resource *res;
	int i;

	for (i = 0; i < MMR_LAST; i++) {
		res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						   mmr_names[i]);
		ud->mmrs[i] = devm_ioremap_resource(&pdev->dev, res);
		if (IS_ERR(ud->mmrs[i]))
			return PTR_ERR(ud->mmrs[i]);
	}

	return 0;
}

static int udma_setup_resources(struct udma_dev *ud)
{
	struct device *dev = ud->dev;
	int ch_count, i;
	u32 cap2, cap3;
	struct ti_sci_resource_desc *rm_desc;
	struct ti_sci_resource *rm_res;
	struct udma_tisci_rm *tisci_rm = &ud->tisci_rm;
	char *range_names[] = { "ti,sci-rm-range-tchan",
				"ti,sci-rm-range-rchan",
				"ti,sci-rm-range-rflow" };

	cap2 = udma_read(ud->mmrs[MMR_GCFG], 0x28);
	cap3 = udma_read(ud->mmrs[MMR_GCFG], 0x2c);

	ud->rflow_cnt = cap3 & 0x3fff;
	ud->tchan_cnt = cap2 & 0x1ff;
	ud->echan_cnt = (cap2 >> 9) & 0x1ff;
	ud->rchan_cnt = (cap2 >> 18) & 0x1ff;
	ch_count  = ud->tchan_cnt + ud->rchan_cnt;

	ud->tchan_map = devm_kmalloc_array(dev, BITS_TO_LONGS(ud->tchan_cnt),
					   sizeof(unsigned long), GFP_KERNEL);
	ud->tchans = devm_kcalloc(dev, ud->tchan_cnt, sizeof(*ud->tchans),
				  GFP_KERNEL);
	ud->rchan_map = devm_kmalloc_array(dev, BITS_TO_LONGS(ud->rchan_cnt),
					   sizeof(unsigned long), GFP_KERNEL);
	ud->rchans = devm_kcalloc(dev, ud->rchan_cnt, sizeof(*ud->rchans),
				  GFP_KERNEL);
	ud->rflow_map = devm_kmalloc_array(dev, BITS_TO_LONGS(ud->rflow_cnt),
					   sizeof(unsigned long), GFP_KERNEL);
	ud->rflow_map_reserved = devm_kcalloc(dev, BITS_TO_LONGS(ud->rflow_cnt),
					      sizeof(unsigned long),
					      GFP_KERNEL);
	ud->rflows = devm_kcalloc(dev, ud->rflow_cnt, sizeof(*ud->rflows),
				  GFP_KERNEL);

	if (!ud->tchan_map || !ud->rchan_map || !ud->rflow_map ||
	    !ud->rflow_map_reserved || !ud->tchans || !ud->rchans ||
	    !ud->rflows)
		return -ENOMEM;

	/*
	 * RX flows with the same Ids as RX channels are reserved to be used
	 * as default flows if remote HW can't generate flow_ids. Those
	 * RX flows can be requested only explicitly by id.
	 */
	bitmap_set(ud->rflow_map_reserved, 0, ud->rchan_cnt);

	/* Get resource ranges from tisci */
	for (i = 0; i < RM_RANGE_LAST; i++)
		tisci_rm->rm_ranges[i] = devm_ti_sci_get_of_resource(
							tisci_rm->tisci, dev,
							range_names[i]);

	/* tchan ranges */
	rm_res = tisci_rm->rm_ranges[RM_RANGE_TCHAN];
	if (IS_ERR(rm_res)) {
		bitmap_zero(ud->tchan_map, ud->tchan_cnt);
	} else {
		bitmap_fill(ud->tchan_map, ud->tchan_cnt);
		for (i = 0; i < rm_res->sets; i++) {
			rm_desc = &rm_res->desc[i];
			bitmap_clear(ud->tchan_map, rm_desc->start,
				     rm_desc->num);
		}
	}

	/* rchan and matching default flow ranges */
	rm_res = tisci_rm->rm_ranges[RM_RANGE_RCHAN];
	if (IS_ERR(rm_res)) {
		bitmap_zero(ud->rchan_map, ud->rchan_cnt);
		bitmap_zero(ud->rflow_map, ud->rchan_cnt);
	} else {
		bitmap_fill(ud->rchan_map, ud->rchan_cnt);
		bitmap_fill(ud->rflow_map, ud->rchan_cnt);
		for (i = 0; i < rm_res->sets; i++) {
			rm_desc = &rm_res->desc[i];
			bitmap_clear(ud->rchan_map, rm_desc->start,
				     rm_desc->num);
			bitmap_clear(ud->rflow_map, rm_desc->start,
				     rm_desc->num);
		}
	}

	/* GP rflow ranges */
	rm_res = tisci_rm->rm_ranges[RM_RANGE_RFLOW];
	if (IS_ERR(rm_res)) {
		bitmap_clear(ud->rflow_map, ud->rchan_cnt,
			     ud->rflow_cnt - ud->rchan_cnt);
	} else {
		bitmap_set(ud->rflow_map, ud->rchan_cnt,
			   ud->rflow_cnt - ud->rchan_cnt);
		for (i = 0; i < rm_res->sets; i++) {
			rm_desc = &rm_res->desc[i];
			bitmap_clear(ud->rflow_map, rm_desc->start,
				     rm_desc->num);
		}
	}

	/*
	 * HACK: tchan0, rchan0,1 and rflow0,1 on main_navss is dedicated to
	 * sysfw.
	 * Only UDMAP on main_navss have echan, use it as a hint for now.
	 */
	if (ud->echan_cnt) {
		set_bit(0, ud->tchan_map);
		set_bit(0, ud->rchan_map);
		set_bit(1, ud->rchan_map);
		set_bit(0, ud->rflow_map);
		set_bit(1, ud->rflow_map);
	}

	ch_count -= bitmap_weight(ud->tchan_map, ud->tchan_cnt);
	ch_count -= bitmap_weight(ud->rchan_map, ud->rchan_cnt);
	if (!ch_count)
		return -ENODEV;

	ud->channels = devm_kcalloc(dev, ch_count, sizeof(*ud->channels),
				    GFP_KERNEL);
	if (!ud->channels)
		return -ENOMEM;

	dev_info(dev,
		 "Channels: %d (tchan: %u, echan: %u, rchan: %u, rflow: %u)\n",
		 ch_count, ud->tchan_cnt, ud->echan_cnt, ud->rchan_cnt,
		 ud->rflow_cnt);

	return ch_count;
}

#define TI_UDMAC_BUSWIDTHS	(BIT(DMA_SLAVE_BUSWIDTH_1_BYTE) | \
				 BIT(DMA_SLAVE_BUSWIDTH_2_BYTES) | \
				 BIT(DMA_SLAVE_BUSWIDTH_3_BYTES) | \
				 BIT(DMA_SLAVE_BUSWIDTH_4_BYTES) | \
				 BIT(DMA_SLAVE_BUSWIDTH_8_BYTES))

static int udma_probe(struct platform_device *pdev)
{
	struct device_node *parent_irq_node;
	struct device *dev = &pdev->dev;
	struct udma_dev *ud;
	const struct of_device_id *match;
	int i, ret;
	int ch_count;

	ret = dma_coerce_mask_and_coherent(dev, DMA_BIT_MASK(48));
	if (ret)
		dev_err(dev, "failed to set dma mask stuff\n");

	ud = devm_kzalloc(dev, sizeof(*ud), GFP_KERNEL);
	if (!ud)
		return -ENOMEM;

	ret = udma_get_mmrs(pdev, ud);
	if (ret)
		return ret;

	ud->tisci_rm.tisci = ti_sci_get_by_phandle(dev->of_node, "ti,sci");
	if (IS_ERR(ud->tisci_rm.tisci))
		return PTR_ERR(ud->tisci_rm.tisci);

	ret = of_property_read_u32(dev->of_node, "ti,sci-dev-id",
				   &ud->tisci_rm.tisci_dev_id);
	if (ret) {
		dev_err(dev, "ti,sci-dev-id read failure %d\n", ret);
		return ret;
	}

	ud->tisci_rm.tisci_udmap_ops = &ud->tisci_rm.tisci->ops.rm_udmap_ops;

	ud->psil_node = of_k3_nav_psil_get_by_phandle(dev->of_node,
						      "ti,psi-proxy");
	if (IS_ERR(ud->psil_node))
		return PTR_ERR(ud->psil_node);

	ud->ringacc = of_k3_nav_ringacc_get_by_phandle(dev->of_node,
						       "ti,ringacc");
	if (IS_ERR(ud->ringacc))
		return PTR_ERR(ud->ringacc);

	parent_irq_node = of_irq_find_parent(dev->of_node);
	if (!parent_irq_node) {
		dev_err(dev, "Failed to get IRQ parent node\n");
		return -ENODEV;
	}

	ud->irq_domain = irq_find_host(parent_irq_node);
	if (!ud->irq_domain)
		return -EPROBE_DEFER;

	match = of_match_node(udma_of_match, dev->of_node);
	if (!match) {
		dev_err(dev, "No compatible match found\n");
		return -ENODEV;
	}
	ud->match_data = match->data;

	dma_cap_set(DMA_SLAVE, ud->ddev.cap_mask);
	dma_cap_set(DMA_CYCLIC, ud->ddev.cap_mask);
	dma_cap_set(DMA_MEMCPY, ud->ddev.cap_mask);

	ud->ddev.device_alloc_chan_resources = udma_alloc_chan_resources;
	ud->ddev.device_config = udma_slave_config;
	ud->ddev.device_prep_slave_sg = udma_prep_slave_sg;
	ud->ddev.device_prep_dma_cyclic = udma_prep_dma_cyclic;
	ud->ddev.device_prep_dma_memcpy = udma_prep_dma_memcpy;
	ud->ddev.device_attach_metadata = udma_attach_metadata;
	ud->ddev.device_issue_pending = udma_issue_pending;
	ud->ddev.device_tx_status = udma_tx_status;
	ud->ddev.device_pause = udma_pause;
	ud->ddev.device_resume = udma_resume;
	ud->ddev.device_terminate_all = udma_terminate_all;
	ud->ddev.device_synchronize = udma_synchronize;

	ud->ddev.device_free_chan_resources = udma_free_chan_resources;
	ud->ddev.src_addr_widths = TI_UDMAC_BUSWIDTHS;
	ud->ddev.dst_addr_widths = TI_UDMAC_BUSWIDTHS;
	ud->ddev.directions = BIT(DMA_DEV_TO_MEM) | BIT(DMA_MEM_TO_DEV) |
			      BIT(DMA_MEM_TO_MEM);
	ud->ddev.residue_granularity = DMA_RESIDUE_GRANULARITY_BURST;
	ud->ddev.copy_align = DMAENGINE_ALIGN_8_BYTES;
	ud->ddev.dev = dev;
	ud->dev = dev;

	INIT_LIST_HEAD(&ud->ddev.channels);
	INIT_LIST_HEAD(&ud->desc_to_purge);

	ret = of_property_read_u32(dev->of_node, "ti,psil-base",
				   &ud->psil_base);
	if (ret) {
		dev_info(dev, "Missing ti,psil-base property, using %d.\n",
			 ret);
		return ret;
	}

	ch_count = udma_setup_resources(ud);
	if (ch_count <= 0)
		return ch_count;

	spin_lock_init(&ud->lock);
	INIT_WORK(&ud->purge_work, udma_purge_desc_work);

	for (i = 0; i < ud->tchan_cnt; i++) {
		struct udma_tchan *tchan = &ud->tchans[i];

		tchan->id = i;
		tchan->reg_rt = ud->mmrs[MMR_TCHANRT] + UDMA_CH_1000(i);
	}

	for (i = 0; i < ud->rchan_cnt; i++) {
		struct udma_rchan *rchan = &ud->rchans[i];

		rchan->id = i;
		rchan->reg_rt = ud->mmrs[MMR_RCHANRT] + UDMA_CH_1000(i);
	}

	for (i = 0; i < ud->rflow_cnt; i++) {
		struct udma_rflow *rflow = &ud->rflows[i];

		rflow->id = i;
	}

	for (i = 0; i < ch_count; i++) {
		struct udma_chan *uc = &ud->channels[i];

		uc->ud = ud;
		uc->vc.desc_free = udma_desc_free;
		uc->id = i;
		uc->slave_thread_id = -1;
		uc->tchan = NULL;
		uc->rchan = NULL;
		uc->dir = DMA_MEM_TO_MEM;
		uc->name = devm_kasprintf(dev, GFP_KERNEL, "UDMA chan%d", i);

		vchan_init(&uc->vc, &ud->ddev);
		/* Use custom vchan completion handling */
		tasklet_init(&uc->vc.task, udma_vchan_complete,
			     (unsigned long)&uc->vc);
		init_completion(&uc->teardown_completed);
	}

	ret = dma_async_device_register(&ud->ddev);
	if (ret) {
		dev_err(dev, "failed to register slave DMA engine: %d\n", ret);
		return ret;
	}

	platform_set_drvdata(pdev, ud);

	ret = of_dma_controller_register(dev->of_node, udma_of_xlate, ud);
	if (ret) {
		dev_err(dev, "failed to register of_dma controller\n");
		dma_async_device_unregister(&ud->ddev);
	}

	return ret;
}

static int udma_remove(struct platform_device *pdev)
{
	struct udma_dev *ud = platform_get_drvdata(pdev);

	of_dma_controller_free(pdev->dev.of_node);
	dma_async_device_unregister(&ud->ddev);

	/* Make sure that we did proper cleanup */
	cancel_work_sync(&ud->purge_work);
	udma_purge_desc_work(&ud->purge_work);

	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

	return 0;
}

static struct platform_driver udma_driver = {
	.driver = {
		.name	= "ti-udma",
		.of_match_table = udma_of_match,
	},
	.probe		= udma_probe,
	.remove		= udma_remove,
};

module_platform_driver(udma_driver);

/* Private interfaces to UDMA */
#include "k3-udma-private.c"

MODULE_ALIAS("platform:ti-udma");
MODULE_DESCRIPTION("TI K3 DMA driver for CPPI 5.0 compliant devices");
MODULE_AUTHOR("Peter Ujfalusi <peter.ujfalusi@ti.com>");
MODULE_LICENSE("GPL v2");
