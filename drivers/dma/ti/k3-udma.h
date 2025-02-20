/* SPDX-License-Identifier: GPL-2.0 */
/*
 *  Copyright (C) 2019 Texas Instruments Incorporated - http://www.ti.com
 */

#ifndef K3_UDMA_H_
#define K3_UDMA_H_

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/sys_soc.h>
#include <linux/of.h>
#include <linux/of_dma.h>
#include <linux/of_irq.h>
#include <linux/workqueue.h>
#include <linux/completion.h>
#include <linux/soc/ti/k3-ringacc.h>
#include <linux/soc/ti/ti_sci_protocol.h>
#include <linux/soc/ti/ti_sci_inta_msi.h>
#include <linux/dma/k3-event-router.h>
#include <linux/dma/ti-cppi5.h>

#include "../virt-dma.h"
#include "k3-psil-priv.h"

/* Global registers */
#define UDMA_REV_REG			0x0
#define UDMA_PERF_CTL_REG		0x4
#define UDMA_EMU_CTL_REG		0x8
#define UDMA_PSIL_TO_REG		0x10
#define UDMA_UTC_CTL_REG		0x1c
#define UDMA_CAP_REG(i)			(0x20 + ((i) * 4))
#define UDMA_RX_FLOW_ID_FW_OES_REG	0x80
#define UDMA_RX_FLOW_ID_FW_STATUS_REG	0x88

/* BCHANRT/TCHANRT/RCHANRT registers */
#define UDMA_CHAN_RT_CTL_REG		0x0
#define UDMA_CHAN_RT_SWTRIG_REG		0x8
#define UDMA_CHAN_RT_STDATA_REG		0x80

#define UDMA_CHAN_RT_PDMA_STATE_REG		0x80c

#define UDMA_CHAN_RT_PEER_REG(i)	(0x200 + ((i) * 0x4))
#define UDMA_CHAN_RT_PEER_STATIC_TR_XY_REG	\
	UDMA_CHAN_RT_PEER_REG(0)	/* PSI-L: 0x400 */
#define UDMA_CHAN_RT_PEER_STATIC_TR_Z_REG	\
	UDMA_CHAN_RT_PEER_REG(1)	/* PSI-L: 0x401 */
#define UDMA_CHAN_RT_PEER_BCNT_REG		\
	UDMA_CHAN_RT_PEER_REG(4)	/* PSI-L: 0x404 */
#define UDMA_CHAN_RT_PEER_RT_EN_REG		\
	UDMA_CHAN_RT_PEER_REG(8)	/* PSI-L: 0x408 */

#define UDMA_CHAN_RT_PCNT_REG		0x400
#define UDMA_CHAN_RT_BCNT_REG		0x408
#define UDMA_CHAN_RT_SBCNT_REG		0x410

/* UDMA_CAP Registers */
#define UDMA_CAP2_TCHAN_CNT(val)	((val) & 0x1ff)
#define UDMA_CAP2_ECHAN_CNT(val)	(((val) >> 9) & 0x1ff)
#define UDMA_CAP2_RCHAN_CNT(val)	(((val) >> 18) & 0x1ff)
#define UDMA_CAP3_RFLOW_CNT(val)	((val) & 0x3fff)
#define UDMA_CAP3_HCHAN_CNT(val)	(((val) >> 14) & 0x1ff)
#define UDMA_CAP3_UCHAN_CNT(val)	(((val) >> 23) & 0x1ff)

#define BCDMA_CAP2_BCHAN_CNT(val)	((val) & 0x1ff)
#define BCDMA_CAP2_TCHAN_CNT(val)	(((val) >> 9) & 0x1ff)
#define BCDMA_CAP2_RCHAN_CNT(val)	(((val) >> 18) & 0x1ff)
#define BCDMA_CAP3_HBCHAN_CNT(val)	(((val) >> 14) & 0x1ff)
#define BCDMA_CAP3_UBCHAN_CNT(val)	(((val) >> 23) & 0x1ff)
#define BCDMA_CAP4_HRCHAN_CNT(val)	((val) & 0xff)
#define BCDMA_CAP4_URCHAN_CNT(val)	(((val) >> 8) & 0xff)
#define BCDMA_CAP4_HTCHAN_CNT(val)	(((val) >> 16) & 0xff)
#define BCDMA_CAP4_UTCHAN_CNT(val)	(((val) >> 24) & 0xff)

#define PKTDMA_CAP4_TFLOW_CNT(val)	((val) & 0x3fff)

/* UDMA_CHAN_RT_CTL_REG */
#define UDMA_CHAN_RT_CTL_EN		BIT(31)
#define UDMA_CHAN_RT_CTL_TDOWN		BIT(30)
#define UDMA_CHAN_RT_CTL_PAUSE		BIT(29)
#define UDMA_CHAN_RT_CTL_FTDOWN		BIT(28)
#define UDMA_CHAN_RT_CTL_AUTOPAIR      BIT(23)
#define UDMA_CHAN_RT_CTL_PAIR_TIMEOUT  BIT(17)
#define UDMA_CHAN_RT_CTL_PAIR_COMPLETE BIT(16)
#define UDMA_CHAN_RT_CTL_ERROR		BIT(0)

/* UDMA_CHAN_RT_PDMA_STATE_REG */
#define UDMA_CHAN_RT_PDMA_STATE_IN_EVT		BIT(31)
#define UDMA_CHAN_RT_PDMA_STATE_TDOWN		BIT(30)
#define UDMA_CHAN_RT_PDMA_STATE_PAUSE		BIT(29)

/* UDMA_CHAN_RT_PEER_RT_EN_REG */
#define UDMA_PEER_RT_EN_ENABLE		BIT(31)
#define UDMA_PEER_RT_EN_TEARDOWN	BIT(30)
#define UDMA_PEER_RT_EN_PAUSE		BIT(29)
#define UDMA_PEER_RT_EN_FLUSH		BIT(28)
#define UDMA_PEER_RT_EN_IDLE		BIT(1)

/*
 * UDMA_TCHAN_RT_PEER_STATIC_TR_XY_REG /
 * UDMA_RCHAN_RT_PEER_STATIC_TR_XY_REG
 */
#define PDMA_STATIC_TR_X_MASK		GENMASK(26, 24)
#define PDMA_STATIC_TR_X_SHIFT		(24)
#define PDMA_STATIC_TR_Y_MASK		GENMASK(11, 0)
#define PDMA_STATIC_TR_Y_SHIFT		(0)

#define PDMA_STATIC_TR_Y(x)	\
	(((x) << PDMA_STATIC_TR_Y_SHIFT) & PDMA_STATIC_TR_Y_MASK)
#define PDMA_STATIC_TR_X(x)	\
	(((x) << PDMA_STATIC_TR_X_SHIFT) & PDMA_STATIC_TR_X_MASK)

#define PDMA_STATIC_TR_XY_ACC32		BIT(30)
#define PDMA_STATIC_TR_XY_BURST		BIT(31)

/*
 * UDMA_TCHAN_RT_PEER_STATIC_TR_Z_REG /
 * UDMA_RCHAN_RT_PEER_STATIC_TR_Z_REG
 */
#define PDMA_STATIC_TR_Z(x, mask)	((x) & (mask))

/* Address Space Select */
#define K3_ADDRESS_ASEL_SHIFT		48

#define K3_UDMA_MAX_RFLOWS		1024
#define K3_UDMA_DEFAULT_RING_SIZE	16

/* How SRC/DST tag should be updated by UDMA in the descriptor's Word 3 */
#define UDMA_RFLOW_SRCTAG_NONE		0
#define UDMA_RFLOW_SRCTAG_CFG_TAG	1
#define UDMA_RFLOW_SRCTAG_FLOW_ID	2
#define UDMA_RFLOW_SRCTAG_SRC_TAG	4

#define UDMA_RFLOW_DSTTAG_NONE		0
#define UDMA_RFLOW_DSTTAG_CFG_TAG	1
#define UDMA_RFLOW_DSTTAG_FLOW_ID	2
#define UDMA_RFLOW_DSTTAG_DST_TAG_LO	4
#define UDMA_RFLOW_DSTTAG_DST_TAG_HI	5

/* Device capability flags */
#define UDMA_FLAG_PDMA_ACC32		BIT(0)
#define UDMA_FLAG_PDMA_BURST		BIT(1)
#define UDMA_FLAG_TDTYPE		BIT(2)
#define UDMA_FLAG_BURST_SIZE		BIT(3)
#define UDMA_FLAGS_J7_CLASS		(UDMA_FLAG_PDMA_ACC32 | \
					 UDMA_FLAG_PDMA_BURST | \
					 UDMA_FLAG_TDTYPE | \
					 UDMA_FLAG_BURST_SIZE)

#define TI_UDMAC_BUSWIDTHS	(BIT(DMA_SLAVE_BUSWIDTH_1_BYTE) | \
				 BIT(DMA_SLAVE_BUSWIDTH_2_BYTES) | \
				 BIT(DMA_SLAVE_BUSWIDTH_3_BYTES) | \
				 BIT(DMA_SLAVE_BUSWIDTH_4_BYTES) | \
				 BIT(DMA_SLAVE_BUSWIDTH_8_BYTES))

struct udma_chan;
struct udma_dev;

enum k3_dma_type {
	DMA_TYPE_UDMA = 0,
	DMA_TYPE_BCDMA,
	DMA_TYPE_PKTDMA,
	DMA_TYPE_BCDMA_V2,
	DMA_TYPE_PKTDMA_V2,
};

enum udma_mmr {
	MMR_GCFG = 0,
	MMR_BCHANRT,
	MMR_RCHANRT,
	MMR_TCHANRT,
	MMR_LAST,
};

enum am62l_udma_mmr {
	AM62L_MMR_GCFG = 0,
	AM62L_MMR_BCHANRT,
	AM62L_MMR_CHANRT,
	AM62L_MMR_LAST,
};

enum udma_rm_range {
	RM_RANGE_BCHAN = 0,
	RM_RANGE_TCHAN,
	RM_RANGE_RCHAN,
	RM_RANGE_RFLOW,
	RM_RANGE_TFLOW,
	RM_RANGE_LAST,
};

enum udma_chan_state {
	UDMA_CHAN_IS_IDLE = 0, /* not active, no teardown is in progress */
	UDMA_CHAN_IS_ACTIVE, /* Normal operation */
	UDMA_CHAN_IS_TERMINATING, /* channel is being terminated */
};

struct udma_filter_param {
	int remote_thread_id;
	u32 atype;
	u32 asel;
	u32 tr_trigger_type;
};

struct am62l_udma_filter_param {
	u32 tr_trigger_type;
	u32 trigger_param;
	int remote_thread_id;
	u32 asel;
};

struct udma_static_tr {
	u8 elsize; /* RPSTR0 */
	u16 elcnt; /* RPSTR0 */
	u16 bstcnt; /* RPSTR1 */
};

struct udma_tchan {
	void __iomem *reg_rt;

	int id;
	struct k3_ring *t_ring; /* Transmit ring */
	struct k3_ring *tc_ring; /* Transmit Completion ring */
	int tflow_id; /* applicable only for PKTDMA */
};

#define udma_bchan udma_tchan
#define udma_rchan udma_tchan

struct udma_rflow {
	int id;
	struct k3_ring *fd_ring; /* Free Descriptor ring */
	struct k3_ring *r_ring; /* Receive ring */
	void __iomem *reg_rt;
};

struct udma_oes_offsets {
	/* K3 UDMA Output Event Offset */
	u32 udma_rchan;

	/* BCDMA Output Event Offsets */
	u32 bcdma_bchan_data;
	u32 bcdma_bchan_ring;
	u32 bcdma_tchan_data;
	u32 bcdma_tchan_ring;
	u32 bcdma_rchan_data;
	u32 bcdma_rchan_ring;

	/* PKTDMA Output Event Offsets */
	u32 pktdma_tchan_flow;
	u32 pktdma_rchan_flow;
};


static const char * const mmr_names[] = {
	[MMR_GCFG] = "gcfg",
	[MMR_BCHANRT] = "bchanrt",
	[MMR_RCHANRT] = "rchanrt",
	[MMR_TCHANRT] = "tchanrt",
};

static const char * const am62l_mmr_names[] = {
	[AM62L_MMR_GCFG] = "gcfg",
	[AM62L_MMR_BCHANRT] = "bchanrt",
	[AM62L_MMR_CHANRT] = "chanrt",
};

struct udma_tisci_rm {
	const struct ti_sci_handle *tisci;
	const struct ti_sci_rm_udmap_ops *tisci_udmap_ops;
	u32  tisci_dev_id;

	/* tisci information for PSI-L thread pairing/unpairing */
	const struct ti_sci_rm_psil_ops *tisci_psil_ops;
	u32  tisci_navss_dev_id;

	struct ti_sci_resource *rm_ranges[RM_RANGE_LAST];
};


struct udma_match_data {
	enum k3_dma_type type;
	u32 psil_base;
	bool enable_memcpy_support;
	u32 flags;
	u32 statictr_z_mask;
	u8 burst_size[3];
	struct udma_soc_data *soc_data;
	u32 bchan_cnt;
	u32 chan_cnt;
	u32 tchan_cnt;
	u32 rchan_cnt;
	u32 tflow_cnt;
	u32 rflow_cnt;
};

struct udma_soc_data {
	struct udma_oes_offsets oes;
	u32 bcdma_trigger_event_offset;
};

struct udma_hwdesc {
	size_t cppi5_desc_size;
	void *cppi5_desc_vaddr;
	dma_addr_t cppi5_desc_paddr;

	/* TR descriptor internal pointers */
	void *tr_req_base;
	struct cppi5_tr_resp_t *tr_resp_base;
};

struct udma_rx_flush {
	struct udma_hwdesc hwdescs[2];

	size_t buffer_size;
	void *buffer_vaddr;
	dma_addr_t buffer_paddr;
};

struct udma_tpl {
	u8 levels;
	u32 start_idx[3];
};

struct udma_desc {
	struct virt_dma_desc vd;

	bool terminated;

	enum dma_transfer_direction dir;

	struct udma_static_tr static_tr;
	u32 residue;

	unsigned int sglen;
	unsigned int desc_idx; /* Only used for cyclic in packet mode */
	unsigned int tr_idx;

	u32 metadata_size;
	void *metadata; /* pointer to provided metadata buffer (EPIP, PSdata) */

	unsigned int hwdesc_count;
	struct udma_hwdesc hwdesc[];
};

struct udma_tx_drain {
	struct delayed_work work;
	ktime_t tstamp;
	u32 residue;
};

struct udma_chan_config {
	bool pkt_mode; /* TR or packet */
	bool needs_epib; /* EPIB is needed for the communication or not */
	u32 psd_size; /* size of Protocol Specific Data */
	u32 metadata_size; /* (needs_epib ? 16:0) + psd_size */
	u32 hdesc_size; /* Size of a packet descriptor in packet mode */
	bool notdpkt; /* Suppress sending TDC packet */
	int remote_thread_id;
	u32 atype;
	u32 asel;
	u32 src_thread;
	u32 dst_thread;
	enum psil_endpoint_type ep_type;
	bool enable_acc32;
	bool enable_burst;
	enum udma_tp_level channel_tpl; /* Channel Throughput Level */

	u32 tr_trigger_type;
	unsigned long tx_flags;

	/* PKDMA mapped channel */
	int mapped_channel_id;
	/* PKTDMA default tflow or rflow for mapped channel */
	int default_flow_id;

	enum dma_transfer_direction dir;
};

struct udma_dev {
	struct dma_device ddev;
	struct device *dev;
	void __iomem *mmrs[MMR_LAST];
	void __iomem *rflow_rt;
	const struct udma_match_data *match_data;
	const struct udma_soc_data *soc_data;

	struct udma_tpl bchan_tpl;
	struct udma_tpl tchan_tpl;
	struct udma_tpl rchan_tpl;

	size_t desc_align; /* alignment to use for descriptors */

	struct udma_tisci_rm tisci_rm;

	struct k3_ringacc *ringacc;

	struct work_struct purge_work;
	struct list_head desc_to_purge;
	spinlock_t lock;

	struct udma_rx_flush rx_flush;

	int bchan_cnt;
	int chan_cnt;
	int tchan_cnt;
	int echan_cnt;
	int rchan_cnt;
	int rflow_cnt;
	int tflow_cnt;
	unsigned long *bchan_map;
	unsigned long *chan_map;
	unsigned long *tchan_map;
	unsigned long *rchan_map;
	unsigned long *rflow_gp_map;
	unsigned long *rflow_gp_map_allocated;
	unsigned long *rflow_in_use;
	unsigned long *tflow_map;

	struct udma_bchan *bchans;
	struct udma_tchan *chans;
	struct udma_tchan *tchans;
	struct udma_rchan *rchans;
	struct udma_rflow *rflows;

	struct udma_chan *channels;
	u32 psil_base;
	u32 atype;
	u32 asel;

	int (*udma_start)(struct udma_chan *uc);
	int (*udma_stop)(struct udma_chan *uc);
	int (*udma_reset_chan)(struct udma_chan *uc, bool hard);
	void (*udma_reset_rings)(struct udma_chan *uc);
	bool (*udma_is_desc_really_done)(struct udma_chan *uc, struct udma_desc *d);
	void (*udma_decrement_byte_counters)(struct udma_chan *uc, u32 val);
};

struct udma_chan {
	struct virt_dma_chan vc;
	struct dma_slave_config	cfg;
	struct udma_dev *ud;
	struct device *dma_dev;
	struct udma_desc *desc;
	struct udma_desc *terminated_desc;
	struct udma_static_tr static_tr;
	char *name;

	struct udma_bchan *bchan;
	struct udma_tchan *chan;
	struct udma_tchan *tchan;
	struct udma_rchan *rchan;
	struct udma_rflow *rflow;

	bool psil_paired;

	int irq_num_ring;
	int irq_num_udma;

	bool cyclic;
	bool paused;

	enum udma_chan_state state;
	struct completion teardown_completed;

	struct udma_tx_drain tx_drain;

	/* Channel configuration parameters */
	struct udma_chan_config config;
	/* Channel configuration parameters (backup) */
	struct udma_chan_config backup_config;

	/* dmapool for packet mode descriptors */
	bool use_dma_pool;
	struct dma_pool *hdesc_pool;

	u32 id;
};

/* K3 UDMA helper functions */
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

/* Generic register access functions */
static inline u32 udma_read(void __iomem *base, int reg)
{
	return readl(base + reg);
}

static inline void udma_write(void __iomem *base, int reg, u32 val)
{
	writel(val, base + reg);
}

static inline void udma_update_bits(void __iomem *base, int reg,
				    u32 mask, u32 val)
{
	u32 tmp, orig;

	orig = readl(base + reg);
	tmp = orig & ~mask;
	tmp |= (val & mask);

	if (tmp != orig)
		writel(tmp, base + reg);
}

#define _UDMA_REG_ACCESS(channel)					\
static inline u32 udma_##channel##rt_read(struct udma_chan *uc, int reg) \
{ \
	if (!uc->channel) \
		return 0; \
	return udma_read(uc->channel->reg_rt, reg); \
} \
\
static inline void udma_##channel##rt_write(struct udma_chan *uc, int reg, u32 val) \
{ \
	if (!uc->channel) \
		return; \
	udma_write(uc->channel->reg_rt, reg, val); \
} \
\
static inline void udma_##channel##rt_update_bits(struct udma_chan *uc, int reg, \
						u32 mask, u32 val) \
{ \
	if (!uc->channel) \
		return; \
	udma_update_bits(uc->channel->reg_rt, reg, mask, val); \
}

_UDMA_REG_ACCESS(chan);
_UDMA_REG_ACCESS(bchan);
_UDMA_REG_ACCESS(tchan);
_UDMA_REG_ACCESS(rchan);

static inline dma_addr_t udma_curr_cppi5_desc_paddr(struct udma_desc *d,
						    int idx)
{
	return d->hwdesc[idx].cppi5_desc_paddr;
}

static inline void *udma_curr_cppi5_desc_vaddr(struct udma_desc *d, int idx)
{
	return d->hwdesc[idx].cppi5_desc_vaddr;
}

static inline dma_addr_t udma_get_rx_flush_hwdesc_paddr(struct udma_chan *uc)
{
	return uc->ud->rx_flush.hwdescs[uc->config.pkt_mode].cppi5_desc_paddr;
}

static inline void udma_fetch_epib(struct udma_chan *uc, struct udma_desc *d)
{
	struct cppi5_host_desc_t *h_desc = d->hwdesc[0].cppi5_desc_vaddr;

	memcpy(d->metadata, h_desc->epib, d->metadata_size);
}

void udma_start_desc(struct udma_chan *uc);
bool udma_chan_needs_reconfiguration(struct udma_chan *uc);
void udma_cyclic_packet_elapsed(struct udma_chan *uc);
void udma_check_tx_completion(struct work_struct *work);
void udma_issue_pending(struct dma_chan *chan);
void udma_free_chan_resources(struct dma_chan *chan);
int setup_resources(struct udma_dev *ud);
void udma_mark_resource_ranges(struct udma_dev *ud, unsigned long *map,
			    struct ti_sci_resource_desc *rm_desc,
			    char *name);
int udma_setup_resources(struct udma_dev *ud);
int bcdma_setup_resources(struct udma_dev *ud);
int pktdma_setup_resources(struct udma_dev *ud);

void k3_configure_chan_coherency(struct dma_chan *chan, u32 asel);
u8 udma_get_chan_tpl_index(struct udma_tpl *tpl_map, int chan_id);
void udma_reset_uchan(struct udma_chan *uc);
void udma_dump_chan_stdata(struct udma_chan *uc);
struct udma_desc *udma_udma_desc_from_paddr(struct udma_chan *uc,
				   dma_addr_t paddr);
void udma_free_hwdesc(struct udma_chan *uc, struct udma_desc *d);
void udma_purge_desc_work(struct work_struct *work);
void udma_desc_free(struct virt_dma_desc *vd);
bool udma_is_chan_running(struct udma_chan *uc);
int udma_push_to_ring(struct udma_chan *uc, int idx);
bool udma_desc_is_rx_flush(struct udma_chan *uc, dma_addr_t addr);
int udma_pop_from_ring(struct udma_chan *uc, dma_addr_t *addr);

int __udma_alloc_gp_rflow_range(struct udma_dev *ud, int from, int cnt);
int __udma_free_gp_rflow_range(struct udma_dev *ud, int from, int cnt);
struct udma_rflow *__udma_get_rflow(struct udma_dev *ud, int id);
void __udma_put_rflow(struct udma_dev *ud, struct udma_rflow *rflow);

struct udma_bchan *__udma_reserve_bchan(struct udma_dev *ud, enum udma_tp_level tpl, int id);
struct udma_tchan *__udma_reserve_tchan(struct udma_dev *ud, enum udma_tp_level tpl, int id);
struct udma_rchan *__udma_reserve_rchan(struct udma_dev *ud, enum udma_tp_level tpl, int id);

int udma_get_tchan(struct udma_chan *uc);
int udma_get_rchan(struct udma_chan *uc);
int udma_get_chan_pair(struct udma_chan *uc);
int udma_get_rflow(struct udma_chan *uc, int flow_id);
void bcdma_put_bchan(struct udma_chan *uc);
void udma_put_rchan(struct udma_chan *uc);
void udma_put_tchan(struct udma_chan *uc);
void udma_put_rflow(struct udma_chan *uc);
void bcdma_free_bchan_resources(struct udma_chan *uc);
void udma_free_tx_resources(struct udma_chan *uc);
void udma_free_rx_resources(struct udma_chan *uc);
int udma_slave_config(struct dma_chan *chan,
	     struct dma_slave_config *cfg);
struct udma_desc *udma_alloc_tr_desc(struct udma_chan *uc,
			    size_t tr_size, int tr_count,
			    enum dma_transfer_direction dir);
int udma_get_tr_counters(size_t len, unsigned long align_to,
		u16 *tr0_cnt0, u16 *tr0_cnt1, u16 *tr1_cnt0);
struct udma_desc *
udma_prep_slave_sg_tr(struct udma_chan *uc, struct scatterlist *sgl,
		      unsigned int sglen, enum dma_transfer_direction dir,
		      unsigned long tx_flags, void *context);
struct udma_desc *
udma_prep_slave_sg_triggered_tr(struct udma_chan *uc, struct scatterlist *sgl,
				unsigned int sglen,
				enum dma_transfer_direction dir,
				unsigned long tx_flags, void *context);
int udma_configure_statictr(struct udma_chan *uc, struct udma_desc *d,
				   enum dma_slave_buswidth dev_width,
				   u16 elcnt);
struct udma_desc *
udma_prep_slave_sg_pkt(struct udma_chan *uc, struct scatterlist *sgl,
		       unsigned int sglen, enum dma_transfer_direction dir,
		       unsigned long tx_flags, void *context);
int udma_attach_metadata(struct dma_async_tx_descriptor *desc,
				void *data, size_t len);
void *udma_get_metadata_ptr(struct dma_async_tx_descriptor *desc,
				   size_t *payload_len, size_t *max_len);
int udma_set_metadata_len(struct dma_async_tx_descriptor *desc,
				 size_t payload_len);
struct dma_async_tx_descriptor *
udma_prep_slave_sg(struct dma_chan *chan, struct scatterlist *sgl,
		   unsigned int sglen, enum dma_transfer_direction dir,
		   unsigned long tx_flags, void *context);
struct udma_desc *
udma_prep_dma_cyclic_tr(struct udma_chan *uc, dma_addr_t buf_addr,
			size_t buf_len, size_t period_len,
			enum dma_transfer_direction dir, unsigned long flags);
struct udma_desc *
udma_prep_dma_cyclic_pkt(struct udma_chan *uc, dma_addr_t buf_addr,
			 size_t buf_len, size_t period_len,
			 enum dma_transfer_direction dir, unsigned long flags);
struct dma_async_tx_descriptor *
udma_prep_dma_cyclic(struct dma_chan *chan, dma_addr_t buf_addr, size_t buf_len,
		     size_t period_len, enum dma_transfer_direction dir,
		     unsigned long flags);
struct dma_async_tx_descriptor *
udma_prep_dma_memcpy(struct dma_chan *chan, dma_addr_t dest, dma_addr_t src,
		     size_t len, unsigned long tx_flags);
int udma_terminate_all(struct dma_chan *chan);
void udma_synchronize(struct dma_chan *chan);
void udma_desc_pre_callback(struct virt_dma_chan *vc,
		   struct virt_dma_desc *vd,
		   struct dmaengine_result *result);

void udma_vchan_complete(struct tasklet_struct *t);
int udma_setup_rx_flush(struct udma_dev *ud);

#ifdef CONFIG_DEBUG_FS
void udma_dbg_summary_show_chan(struct seq_file *s,
				       struct dma_chan *chan);
void udma_dbg_summary_show(struct seq_file *s,
				  struct dma_device *dma_dev);
#endif /* CONFIG_DEBUG_FS */

enum dmaengine_alignment udma_get_copy_align(struct udma_dev *ud);
int navss_psil_pair(struct udma_dev *ud, u32 src_thread, u32 dst_thread);
int navss_psil_unpair(struct udma_dev *ud, u32 src_thread,
	     u32 dst_thread);

/* Direct access to UDMA low lever resources for the glue layer */
int xudma_navss_psil_pair(struct udma_dev *ud, u32 src_thread, u32 dst_thread);
int xudma_navss_psil_unpair(struct udma_dev *ud, u32 src_thread,
			    u32 dst_thread);

struct udma_dev *of_xudma_dev_get(struct device_node *np, const char *property);
struct device *xudma_get_device(struct udma_dev *ud);
struct k3_ringacc *xudma_get_ringacc(struct udma_dev *ud);
u32 xudma_dev_get_psil_base(struct udma_dev *ud);
struct udma_tisci_rm *xudma_dev_get_tisci_rm(struct udma_dev *ud);

int xudma_alloc_gp_rflow_range(struct udma_dev *ud, int from, int cnt);
int xudma_free_gp_rflow_range(struct udma_dev *ud, int from, int cnt);

struct udma_tchan *xudma_tchan_get(struct udma_dev *ud, int id);
struct udma_rchan *xudma_rchan_get(struct udma_dev *ud, int id);
struct udma_rflow *xudma_rflow_get(struct udma_dev *ud, int id);

void xudma_tchan_put(struct udma_dev *ud, struct udma_tchan *p);
void xudma_rchan_put(struct udma_dev *ud, struct udma_rchan *p);
void xudma_rflow_put(struct udma_dev *ud, struct udma_rflow *p);

int xudma_tchan_get_id(struct udma_tchan *p);
int xudma_rchan_get_id(struct udma_rchan *p);
int xudma_rflow_get_id(struct udma_rflow *p);

u32 xudma_tchanrt_read(struct udma_tchan *tchan, int reg);
void xudma_tchanrt_write(struct udma_tchan *tchan, int reg, u32 val);
u32 xudma_rchanrt_read(struct udma_rchan *rchan, int reg);
void xudma_rchanrt_write(struct udma_rchan *rchan, int reg, u32 val);
bool xudma_rflow_is_gp(struct udma_dev *ud, int id);
int xudma_get_rflow_ring_offset(struct udma_dev *ud);

int xudma_is_pktdma(struct udma_dev *ud);

int xudma_pktdma_tflow_get_irq(struct udma_dev *ud, int udma_tflow_id);
int xudma_pktdma_rflow_get_irq(struct udma_dev *ud, int udma_rflow_id);
#endif /* K3_UDMA_H_ */
