/*
 * CPPI 4.1 definitions
 *
 * Copyright (c) 2008-2009, MontaVista Software, Inc. <source@mvista.com>
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

#include <linux/types.h>

/*
 * Queue Manager - Control Registers Region
 */
#define QMGR_REVISION_REG		0x00	/* Major and minor versions */
						/* of the module */
#define QMGR_QUEUE_DIVERSION_REG	0x08	/* Queue Diversion register */
#define QMGR_FREE_DESC_BUF_STARVED_REG(n) (0x20 + ((n) << 2)) /* Free Desc./ */
						/* Buffer Starvation Count */
#define QMGR_FREE_DESC_STARVED_REG(n)	(0x30 + ((n) << 2)) /* Free Desc. */
						/* Starvation Count */
#define QMGR_LINKING_RAM_RGN0_BASE_REG	0x80	/* Linking RAM Region 0 Base */
						/* Address */
#define QMGR_LINKING_RAM_RGN0_SIZE_REG	0x84	/* Linking RAM Region 0 Size */
#define QMGR_LINKING_RAM_RGN1_BASE_REG	0x88	/* Linking RAM Region 1 Base */
						/* Address */
#define QMGR_QUEUE_PENDING_REG(n)	(0x90 + ((n) << 2)) /* Pending status */
						/* for all queues */

/*
 * Queue Manager - Memory Region Registers
 */
#define QMGR_MEM_RGN_BASE_REG(r)	(0x00 + ((r) << 4))
#define QMGR_MEM_RGN_CTRL_REG(r)	(0x04 + ((r) << 4))

/* Memory Region R Control Register bits */
#define QMGR_MEM_RGN_INDEX_SHIFT	16
#define QMGR_MEM_RGN_INDEX_MASK		(0x3fff << QMGR_MEM_RGN_INDEX_SHIFT)
#define QMGR_MEM_RGN_DESC_SIZE_SHIFT	8
#define QMGR_MEM_RGN_DESC_SIZE_MASK	(0xf << QMGR_MEM_RGN_DESC_SIZE_SHIFT)
#define QMGR_MEM_RGN_SIZE_SHIFT		0
#define QMGR_MEM_RGN_SIZE_MASK		(7 << QMGR_MEM_RGN_SIZE_SHIFT)

/*
 * Queue Manager - Queues Region
 */
#define QMGR_QUEUE_REG_A(n)		(0x00 + ((n) << 4))
#define QMGR_QUEUE_REG_B(n)		(0x04 + ((n) << 4))
#define QMGR_QUEUE_REG_C(n)		(0x08 + ((n) << 4))
#define QMGR_QUEUE_REG_D(n)		(0x0C + ((n) << 4))

/* Queue N Register C bits */
#define QMGR_QUEUE_HEAD_TAIL_SHIFT	31
#define QMGR_QUEUE_HEAD_TAIL_MASK	(1 << QMGR_QUEUE_HEAD_TAIL_SHIFT)
#define QMGR_QUEUE_PKT_SIZE_SHIFT	0
#define QMGR_QUEUE_PKT_SIZE_MASK	(0x3fff << QMGR_QUEUE_PKT_SIZE_SHIFT)
/* Queue N Register D bits */
#define QMGR_QUEUE_DESC_PTR_SHIFT	5
#define QMGR_QUEUE_DESC_PTR_MASK	(0x7ffffff << QMGR_QUEUE_DESC_PTR_SHIFT)
#define QMGR_QUEUE_DESC_SIZE_SHIFT	0
#define QMGR_QUEUE_DESC_SIZE_MASK	(0x1f << QMGR_QUEUE_DESC_SIZE_SHIFT)

/*
 * Queue Manager - Queue Status Region
 */
#define QMGR_QUEUE_STATUS_REG_A(n)	(0x00 + ((n) << 4))
#define QMGR_QUEUE_STATUS_REG_B(n)	(0x04 + ((n) << 4))
#define QMGR_QUEUE_STATUS_REG_C(n)	(0x08 + ((n) << 4))

/*
 * DMA Controller - Global Control Registers Region
 */
#define DMA_REVISION_REG		0x00	/* Major and minor versions */
						/* of the module */
#define DMA_TEARDOWN_FREE_DESC_CTRL_REG 0x04	/* Queue  manager and queue */
						/* number for Teardown free */
						/* descriptor queue */
#define DMA_EMULATION_CTRL_REG		0x08	/* Emulation control register */

/* Teardown Free Descriptor Queue Control Register bits */
#define DMA_TD_DESC_QMGR_SHIFT		12
#define DMA_TD_DESC_QMGR_MASK		(3 << DMA_TD_DESC_QMGR_SHIFT)
#define DMA_TD_DESC_QNUM_SHIFT		0
#define DMA_TD_DESC_QNUM_MASK		(0xfff << DMA_TD_DESC_QNUM_SHIFT)

/*
 * DMA Controller - Channel Control / Status Registers Region
 */
#define DMA_CH_TX_GLOBAL_CFG_REG(n)	 (0x00 + ((n) << 5))
#define DMA_CH_RX_GLOBAL_CFG_REG(n)	 (0x08 + ((n) << 5))
#define DMA_CH_RX_HOST_PKT_CFG_REG_A(n)  (0x0C + ((n) << 5))
#define DMA_CH_RX_HOST_PKT_CFG_REG_B(n)  (0x10 + ((n) << 5))
#define DMA_CH_RX_EMBED_PKT_CFG_REG_A(n) (0x14 + ((n) << 5))
#define DMA_CH_RX_EMBED_PKT_CFG_REG_B(n) (0x18 + ((n) << 5))
#define DMA_CH_RX_MONO_PKT_CFG_REG(n)	 (0x1C + ((n) << 5))

/* Tx Channel N Global Configuration Register bits */
#define DMA_CH_TX_ENABLE_SHIFT		31
#define DMA_CH_TX_ENABLE_MASK		(1 << DMA_CH_TX_ENABLE_SHIFT)
#define DMA_CH_TX_TEARDOWN_SHIFT	30
#define DMA_CH_TX_TEARDOWN_MASK		(1 << DMA_CH_TX_TEARDOWN_SHIFT)
#define DMA_CH_TX_DEFAULT_QMGR_SHIFT	12
#define DMA_CH_TX_DEFAULT_QMGR_MASK	(3 << DMA_CH_TX_DEFAULT_QMGR_SHIFT)
#define DMA_CH_TX_DEFAULT_QNUM_SHIFT	0
#define DMA_CH_TX_DEFAULT_QNUM_MASK	(0xfff << DMA_CH_TX_DEFAULT_QNUM_SHIFT)

/* Rx Channel N Global Configuration Register bits */
#define DMA_CH_RX_ENABLE_SHIFT		31
#define DMA_CH_RX_ENABLE_MASK		(1 << DMA_CH_RX_ENABLE_SHIFT)
#define DMA_CH_RX_TEARDOWN_SHIFT	30
#define DMA_CH_RX_TEARDOWN_MASK		(1 << DMA_CH_RX_TEARDOWN_SHIFT)
#define DMA_CH_RX_ERROR_HANDLING_SHIFT	24
#define DMA_CH_RX_ERROR_HANDLING_MASK	(1 << DMA_CH_RX_ERROR_HANDLING_SHIFT)
#define DMA_CH_RX_SOP_OFFSET_SHIFT	16
#define DMA_CH_RX_SOP_OFFSET_MASK	(0xff << DMA_CH_RX_SOP_OFFSET_SHIFT)
#define DMA_CH_RX_DEFAULT_DESC_TYPE_SHIFT 14
#define DMA_CH_RX_DEFAULT_DESC_TYPE_MASK  (3 << \
					   DMA_CH_RX_DEFAULT_DESC_TYPE_SHIFT)
#define DMA_CH_RX_DEFAULT_DESC_EMBED	0
#define DMA_CH_RX_DEFAULT_DESC_HOST	1
#define DMA_CH_RX_DEFAULT_DESC_MONO	2
#define DMA_CH_RX_DEFAULT_RQ_QMGR_SHIFT 12
#define DMA_CH_RX_DEFAULT_RQ_QMGR_MASK	(3 << DMA_CH_RX_DEFAULT_RQ_QMGR_SHIFT)
#define DMA_CH_RX_DEFAULT_RQ_QNUM_SHIFT 0
#define DMA_CH_RX_DEFAULT_RQ_QNUM_MASK	(0xfff << \
					 DMA_CH_RX_DEFAULT_RQ_QNUM_SHIFT)
#define DMA_CH_RX_MAX_BUF_CNT_SHIFT	26
#define	DMA_CH_RX_MAX_BUF_CNT_0		0
#define	DMA_CH_RX_MAX_BUF_CNT_1		1
#define	DMA_CH_RX_MAX_BUF_CNT_2		2
#define	DMA_CH_RX_MAX_BUF_CNT_3		3

/* Rx Channel N Host Packet Configuration Register A/B bits */
#define DMA_CH_RX_HOST_FDQ_QMGR_SHIFT(n) (12 + 16 * ((n) & 1))
#define DMA_CH_RX_HOST_FDQ_QMGR_MASK(n)  (3 << DMA_CH_RX_HOST_FDQ_QMGR_SHIFT(n))
#define DMA_CH_RX_HOST_FDQ_QNUM_SHIFT(n) (0 + 16 * ((n) & 1))
#define DMA_CH_RX_HOST_FDQ_QNUM_MASK(n)  (0xfff << \
					  DMA_CH_RX_HOST_FDQ_QNUM_SHIFT(n))

/* Rx Channel N Embedded Packet Configuration Register A bits */
#define DMA_CH_RX_EMBED_FBP_BMGR_SHIFT(n) (6 + 8 * (n))
#define DMA_CH_RX_EMBED_FBP_BMGR_MASK(n)  (3 << \
					   DMA_CH_RX_EMBED_FBP_BMGR_SHIFT(n))
#define DMA_CH_RX_EMBED_FBP_PNUM_SHIFT(n) (0 + 8 * (n))
#define DMA_CH_RX_EMBED_FBP_PNUM_MASK(n)  (0x1f << \
					   DMA_CH_RX_EMBED_FBP_PNUM_SHIFT(n))

/* Rx Channel N Embedded Packet Configuration Register B bits */
#define DMA_CH_RX_EMBED_NUM_SLOT_SHIFT	24
#define DMA_CH_RX_EMBED_NUM_SLOT_MASK	(7 << DMA_CH_RX_EMBED_NUM_SLOT_SHIFT)
#define DMA_CH_RX_EMBED_SOP_SLOT_SHIFT	16
#define DMA_CH_RX_EMBED_SOP_SLOT_MASK	(7 << DMA_CH_RX_EMBED_SOP_SLOT_SHIFT)
#define DMA_CH_RX_EMBED_FDQ_QMGR_SHIFT	12
#define DMA_CH_RX_EMBED_FDQ_QMGR_MASK	(3 << DMA_CH_RX_EMBED_FDQ_QMGR_SHIFT)
#define DMA_CH_RX_EMBED_FDQ_QNUM_SHIFT	0
#define DMA_CH_RX_EMBED_FDQ_QNUM_MASK	(0xfff << \
					 DMA_CH_RX_EMBED_FDQ_QNUM_SHIFT)

/* Rx Channel N Monolithic Packet Configuration Register bits */
#define DMA_CH_RX_MONO_SOP_OFFSET_SHIFT 16
#define DMA_CH_RX_MONO_SOP_OFFSET_MASK	(0xff << \
					 DMA_CH_RX_MONO_SOP_OFFSET_SHIFT)
#define DMA_CH_RX_MONO_FDQ_QMGR_SHIFT	12
#define DMA_CH_RX_MONO_FDQ_QMGR_MASK	(3 << DMA_CH_RX_MONO_FDQ_QMGR_SHIFT)
#define DMA_CH_RX_MONO_FDQ_QNUM_SHIFT	0
#define DMA_CH_RX_MONO_FDQ_QNUM_MASK	(0xfff << DMA_CH_RX_MONO_FDQ_QNUM_SHIFT)

/*
 * DMA Scheduler - Control Region
 */
#define DMA_SCHED_CTRL_REG		0x00

/* DMA Scheduler Control Register bits */
#define DMA_SCHED_ENABLE_SHIFT		31
#define DMA_SCHED_ENABLE_MASK		(1 << DMA_SCHED_ENABLE_SHIFT)
#define DMA_SCHED_LAST_ENTRY_SHIFT	0
#define DMA_SCHED_LAST_ENTRY_MASK	(0xff << DMA_SCHED_LAST_ENTRY_SHIFT)

#define CPPI41_TXDMA_MAXLEN		(4 * 1024 * 1024 - 1)
#define CPPI41_RXDMA_MAXLEN		(64 * 1024)

/*
 * Queue Status register
 */
#define CPPI41_QSTATUS_REG0	0x90
#define CPPI41_QSTATUS_REG1	0x94
#define CPPI41_QSTATUS_REG2	0x98
#define CPPI41_QSTATUS_REG3	0x9c
#define CPPI41_QSTATUS_REG4	0xa0

/*
 * DMA Scheduler - Table Region
 */
#define DMA_SCHED_TABLE_WORD_REG(n)	((n) << 2)
#define MAX_SCHED_TBL_ENTRY     8

/*
 * CPPI 4.1 Host Packet Descriptor
 */
struct cppi41_host_pkt_desc {
	u32 desc_info;		/* Descriptor type, protocol specific word */
				/* count, packet length */
	u32 tag_info;		/* Source tag (31:16), destination tag (15:0) */
	u32 pkt_info;		/* Packet error state, type, protocol flags, */
				/* return info, descriptor location */
	u32 buf_len;		/* Number of valid data bytes in the buffer */
	u32 buf_ptr;		/* Pointer to the buffer associated with */
				/* this descriptor */
	u32 next_desc_ptr;	/* Pointer to the next buffer descriptor */
	u32 orig_buf_len;	/* Original buffer length */
	u32 orig_buf_ptr;	/* Original buffer pointer */
	u32 stk_comms_info[2];	/* Network stack private communications info */
};

/*
 * CPPI 4.1 Host Buffer Descriptor
 */
struct cppi41_host_buf_desc {
	u32 reserved[2];
	u32 buf_recl_info;	/* Return info, descriptor location */
	u32 buf_len;		/* Number of valid data bytes in the buffer */
	u32 buf_ptr;		/* Pointer to the buffer associated with */
				/* this descriptor */
	u32 next_desc_ptr;	/* Pointer to the next buffer descriptor */
	u32 orig_buf_len;	/* Original buffer length */
	u32 orig_buf_ptr;	/* Original buffer pointer */
};

#define CPPI41_DESC_TYPE_SHIFT		27
#define CPPI41_DESC_TYPE_MASK		(0x1f << CPPI41_DESC_TYPE_SHIFT)
#define CPPI41_DESC_TYPE_HOST		16
#define CPPI41_DESC_TYPE_MONOLITHIC	18
#define CPPI41_DESC_TYPE_TEARDOWN	19
#define CPPI41_PROT_VALID_WORD_CNT_SHIFT 22
#define CPPI41_PROT_VALID_WORD_CNT_MASK	(0x1f << CPPI41_PROT_WORD_CNT_SHIFT)
#define CPPI41_PKT_LEN_SHIFT		0
#define CPPI41_PKT_LEN_MASK		(0x1fffff << CPPI41_PKT_LEN_SHIFT)

#define CPPI41_PKT_ERROR_SHIFT		31
#define CPPI41_PKT_ERROR_MASK		(1 << CPPI41_PKT_ERROR_SHIFT)
#define CPPI41_PKT_TYPE_SHIFT		26
#define CPPI41_PKT_TYPE_MASK		(0x1f << CPPI41_PKT_TYPE_SHIFT)
#define CPPI41_PKT_TYPE_ATM_AAL5	0
#define CPPI41_PKT_TYPE_ATM_NULL_AAL	1
#define CPPI41_PKT_TYPE_ATM_OAM		2
#define CPPI41_PKT_TYPE_ATM_TRANSPARENT	3
#define CPPI41_PKT_TYPE_EFM		4
#define CPPI41_PKT_TYPE_USB		5
#define CPPI41_PKT_TYPE_GENERIC		6
#define CPPI41_PKT_TYPE_ETHERNET	7
#define CPPI41_RETURN_POLICY_SHIFT	15
#define CPPI41_RETURN_POLICY_MASK	(1 << CPPI41_RETURN_POLICY_SHIFT)
#define CPPI41_RETURN_LINKED		0
#define CPPI41_RETURN_UNLINKED		1
#define CPPI41_ONCHIP_SHIFT		14
#define CPPI41_ONCHIP_MASK		(1 << CPPI41_ONCHIP_SHIFT)
#define CPPI41_RETURN_QMGR_SHIFT	12
#define CPPI41_RETURN_QMGR_MASK		(3 << CPPI41_RETURN_QMGR_SHIFT)
#define CPPI41_RETURN_QNUM_SHIFT	0
#define CPPI41_RETURN_QNUM_MASK		(0xfff << CPPI41_RETURN_QNUM_SHIFT)

#define CPPI41_SRC_TAG_PORT_NUM_SHIFT	27
#define CPPI41_SRC_TAG_PORT_NUM_MASK	(0x1f << CPPI41_SRC_TAG_PORT_NUM_SHIFT)
#define CPPI41_SRC_TAG_CH_NUM_SHIFT	21
#define CPPI41_SRC_TAG_CH_NUM_MASK	(0x3f << CPPI41_SRC_TAG_CH_NUM_SHIFT)
#define CPPI41_SRC_TAG_SUB_CH_NUM_SHIFT 16
#define CPPI41_SRC_TAG_SUB_CH_NUM_MASK	(0x1f << \
					CPPI41_SRC_TAG_SUB_CH_NUM_SHIFT)
#define CPPI41_DEST_TAG_SHIFT		0
#define CPPI41_DEST_TAG_MASK		(0xffff << CPPI41_DEST_TAG_SHIFT)
#define CPPI41_PKT_INTR_FLAG		(1 << 31)

/*
 * CPPI 4.1 Teardown Descriptor
 */
struct cppi41_teardown_desc {
	u32 teardown_info;	/* Teardown information */
	u32 reserved[7];	/* 28 byte padding */
};

#define CPPI41_TEARDOWN_TX_RX_SHIFT	16
#define CPPI41_TEARDOWN_TX_RX_MASK	(1 << CPPI41_TEARDOWN_TX_RX_SHIFT)
#define CPPI41_TEARDOWN_DMA_NUM_SHIFT	10
#define CPPI41_TEARDOWN_DMA_NUM_MASK	(0x3f << CPPI41_TEARDOWN_DMA_NUM_SHIFT)
#define CPPI41_TEARDOWN_CHAN_NUM_SHIFT	0
#define CPPI41_TEARDOWN_CHAN_NUM_MASK	(0x3f << CPPI41_TEARDOWN_CHAN_NUM_SHIFT)

#define CPPI41_MAX_MEM_RGN		16

/* CPPI 4.1 configuration for AM3517 */
#define CPPI41_NUM_QUEUE_MGR		1	/* 4  max */
#define CPPI41_NUM_DMA_BLOCK		1	/* 64 max */
#define cppi41_num_queue_mgr	CPPI41_NUM_QUEUE_MGR
#define cppi41_num_dma_block	CPPI41_NUM_DMA_BLOCK

/**
 * struct cppi41_queue_manager - CPPI 4.1 DMA queue manager registers for
 * context save and restore.
 */
struct cppi41_queue_manager {
	u32	link_ram_rgn0_base;
	u32	link_ram_rgn0_size;
	u32	link_ram_rgn1_base;

	u32	memr_base[8];
	u32	memr_ctrl[8];
};

/**
 * struct cppi41_dma_regs - CPPI 4.1 DMA registers for
 * context save and restore.
 */
struct cppi41_dma_regs {
	u32	teardn_fdq_ctrl;
	u32	emulation_ctrl;

	/* CPPI DMA scheduler registers */
	u32	sched_ctrl;

	/* Queue manager registers */
	struct cppi41_queue_manager qmgr;
};

/**
 * struct cppi41_queue - Queue Tuple
 *
 * The basic queue tuple in CPPI 4.1 used across all data structures
 * where a definition of a queue is required.
 */
struct cppi41_queue {
	u8  q_mgr;		/* The queue manager number */
	u16 q_num;		/* The queue number */
};

/**
 * struct cppi41_buf_pool - Buffer Pool Tuple
 *
 * The basic buffer pool tuple in CPPI 4.1 used across all data structures
 * where a definition of a buffer pool is required.
 */
struct cppi41_buf_pool {
	u8  b_mgr;		/* The buffer manager number */
	u16 b_pool;		/* The buffer pool number */
};

/**
 * struct cppi41_queue_mgr - Queue Manager information
 *
 * Contains the information about the queue manager which should be copied from
 * the hardware spec as is.
 */
struct cppi41_queue_mgr {
	void __iomem *q_mgr_rgn_base; /* Base address of the Control region. */
	void __iomem *desc_mem_rgn_base; /* Base address of the descriptor */
				/* memory region. */
	void __iomem *q_mgmt_rgn_base; /* Base address of the queues region. */
	void __iomem *q_stat_rgn_base; /* Base address of the queue status */
				/* region. */
	u16 num_queue;		/* Number of the queues supported. */
	u8 queue_types;		/* Bitmask of the supported queue types. */
	u16 base_fdq_num;	/* The base free descriptor queue number. */
				/* If present, there's always 16 such queues. */
	u16 base_fdbq_num;	/* The base free descriptor/buffer queue */
				/* number.  If present, there's always 16 */
				/* such queues. */
	const u32 *assigned;	/* Pointer to the bitmask of the pre-assigned */
				/* queues. */
};

/* Queue type flags */
#define CPPI41_FREE_DESC_QUEUE		0x01
#define CPPI41_FREE_DESC_BUF_QUEUE	0x02
#define CPPI41_UNASSIGNED_QUEUE		0x04

/**
 * struct cppi41_embed_pkt_cfg - Rx Channel Embedded packet configuration
 *
 * An instance of this structure forms part of the Rx channel information
 * structure.
 */
struct cppi41_embed_pkt_cfg {
	struct cppi41_queue fd_queue; /* Free Descriptor queue.*/
	u8 num_buf_slot;	/* Number of buffer slots in the descriptor */
	u8 sop_slot_num;	/* SOP buffer slot number. */
	struct cppi41_buf_pool free_buf_pool[4]; /* Free Buffer pool. Element */
				/* 0 used for the 1st Rx buffer, etc. */
};

/**
 * struct cppi41_host_pkt_cfg - Rx Channel Host Packet Configuration
 *
 * An instance of this structure forms part of the Rx channel information
 * structure.
 */
struct cppi41_host_pkt_cfg {
	struct cppi41_queue fdb_queue[4]; /* Free Desc/Buffer queue. Element */
				/* 0 used for 1st Rx buffer, etc. */
};

/**
 * struct cppi41_mono_pkt_cfg - Rx Channel Monolithic Packet Configuration
 *
 * An instance of this structure forms part of the Rx channel information
 * structure.
 */
struct cppi41_mono_pkt_cfg {
	struct cppi41_queue fd_queue; /* Free descriptor queue */
	u8 sop_offset;		/* Number of bytes to skip before writing */
				/* payload */
};

enum cppi41_rx_desc_type {
	cppi41_rx_embed_desc,
	cppi41_rx_host_desc,
	cppi41_rx_mono_desc,
};

/**
 * struct cppi41_rx_ch_cfg - Rx Channel Configuration
 *
 * Must be allocated and filled by the caller of cppi41_rx_ch_configure().
 *
 * The same channel can be configured to receive different descripor type
 * packets (not simaltaneously). When the Rx packets on a port need to be sent
 * to the SR, the channels default descriptor type is set to Embedded and the
 * Rx completion queue is set to the queue which CPU polls for input packets.
 * When in SR bypass mode, the same channel's default descriptor type will be
 * set to Host and the Rx completion queue set to one of the queues which host
 * can get interrupted on (via the Queuing proxy/accumulator). In this example,
 * the embedded mode configuration fetches free descriptor from the Free
 * descriptor queue (as defined by struct cppi41_embed_pkt_cfg) and host
 * mode configuration fetches free descriptors/buffers from the free descriptor/
 * buffer queue (as defined by struct cppi41_host_pkt_cfg).
 *
 * NOTE: There seems to be no separate configuration for teardown completion
 * descriptor. The assumption is rxQueue tuple is used for this purpose as well.
 */
struct cppi41_rx_ch_cfg {
	enum cppi41_rx_desc_type default_desc_type; /* Describes which queue */
				/* configuration is used for the free */
				/* descriptors and/or buffers */
	u8 sop_offset;		/* Number of bytes to skip in SOP buffer */
				/* before writing payload */
	u8 retry_starved;	/* 0 = Drop packet on descriptor/buffer */
				/* starvartion, 1 = DMA retries FIFO block */
				/* transfer at a later time */
	u8 rx_max_buf_cnt;	/* The DMA ignores the SOP bit and closes up
				 * a packet after a max_buf_cnt buffer has been
				 * filled OR if the EOP field is set in the
				 * info word 0
				 */
	struct cppi41_queue rx_queue; /* Rx complete packets queue */
	union {
		struct cppi41_host_pkt_cfg host_pkt; /* Host packet */
				/* configuration. This defines where channel */
				/* picks free descriptors from. */
		struct cppi41_embed_pkt_cfg embed_pkt; /* Embedded packet */
				/* configuration. This defines where channel */
				/* picks free descriptors/buffers from. */
				/* from. */
		struct cppi41_mono_pkt_cfg mono_pkt; /* Monolithic packet */
				/* configuration. This defines where channel */
				/* picks free descriptors from. */
	} cfg;			/* Union of packet configuration structures */
				/* to be filled in depending on the */
				/* defDescType field. */
};

/**
 * struct cppi41_tx_ch - Tx channel information
 *
 * NOTE: The queues that feed into the Tx channel are fixed at SoC design time.
 */
struct cppi41_tx_ch {
	u8 port_num;		/* Port number. */
	u8 ch_num;		/* Channel number within port. */
	u8 sub_ch_num;		/* Sub-channel number within channel. */
	u8 num_tx_queue;	/* Number of queues from which the channel */
				/* can feed. */
	struct cppi41_queue tx_queue[4]; /* List of queues from which the */
				/* channel can feed. */
};

/**
 * struct cppi41_dma_block - CPPI 4.1 DMA configuration
 *
 * Configuration information for CPPI DMA functionality. Includes the Global
 * configuration, Channel configuration, and the Scheduler configuration.
 */
struct cppi41_dma_block {
	void __iomem *global_ctrl_base; /* Base address of the Global Control */
				/* registers. */
	void __iomem *ch_ctrl_stat_base; /* Base address of the Channel */
				/* Control/Status registers. */
	void __iomem *sched_ctrl_base; /* Base address of the Scheduler */
				/* Control register. */
	void __iomem *sched_table_base; /* Base address of the Scheduler */
				/* Table registers. */
	u8 num_tx_ch;		/* Number of the Tx channels. */
	u8 num_rx_ch;		/* Number of the Rx channels. */
	u8 num_max_ch;		/* maximum dma channels */
	const struct cppi41_tx_ch *tx_ch_info;
	struct cppi41_dma_regs cppi41_regs; /* registers to save and restore */
};

extern struct cppi41_queue_mgr cppi41_queue_mgr[];
extern struct cppi41_dma_block cppi41_dma_block[];

/**
 * struct cppi41_dma_ch_obj - CPPI 4.1 DMA Channel object
 */
struct cppi41_dma_ch_obj {
	void __iomem *base_addr; /* The address of the channel global */
				/* configuration register */
	u32 global_cfg;		/* Tx/Rx global configuration backed-up value */
};

/**
 * struct cppi41_queue_obj - CPPI 4.1 queue object
 */
struct cppi41_queue_obj {
	void __iomem *base_addr; /* The base address of the queue management */
				/* registers */
};

static inline u32 cppi_readl(const void __iomem *addr)
	{ return readl(addr); }
static inline void cppi_writel(u32 data, void __iomem *addr)
	{ writel(data, addr); }
/**
 * cppi41_queue_mgr_init - CPPI 4.1 queue manager initialization.
 * @q_mgr:	the queue manager to initialize
 * @rgn0_base:	linking RAM region 0 physical address
 * @rgn0_size:	linking RAM region 0 size in 32-bit words (0 to 0x3fff)
 *
 * Returns 0 on success, error otherwise.
 */
int cppi41_queue_mgr_init(u8 q_mgr, dma_addr_t rgn0_base, u16 rgn0_size);

/**
 * cppi41_queue_mgr_init - CPPI 4.1 queue manager un-initialization.
 * @q_mgr:	the queue manager to un-initialize
 * Returns 0 on success, error otherwise.
 */
int cppi41_queue_mgr_uninit(u8 q_mgr);

/*
 * CPPI 4.1 Queue Manager Memory Region Allocation and De-allocation APIs.
 */

/**
 * cppi41_mem_rgn_alloc - CPPI 4.1 queue manager memory region allocation.
 * @q_mgr:	the queue manager whose memory region to allocate
 * @rgn_addr:	physical address of the memory region
 * @size_order:	descriptor size as a power of two (between 5 and 13)
 * @num_order:	number of descriptors as a power of two (between 5 and 12)
 * @mem_rgn:	pointer to the index of the memory region allocated
 *
 * This function allocates a memory region within the queue manager
 * consisiting of the descriptors of paricular size and number.
 *
 * Returns 0 on success, error otherwise.
 */
int cppi41_mem_rgn_alloc(u8 q_mgr, dma_addr_t rgn_addr, u8 size_order,
			 u8 num_order, u8 *mem_rgn);

/**
 * cppi41_mem_rgn_free - CPPI 4.1 queue manager memory region de-allocation.
 * @q_mgr:	the queue manager whose memory region was allocated
 * @mem_rgn:	index of the memory region
 *
 * This function frees the memory region allocated by cppi41_mem_rgn_alloc().
 *
 * Returns 0 on success, -EINVAL otherwise.
 */
int cppi41_mem_rgn_free(u8 q_mgr, u8 mem_rgn);

/**
 * cppi41_dma_block_init - CPPI 4.1 DMA block initialization.
 * @dma_num:	number of the DMA block
 * @q_mgr:	the queue manager in which to allocate the free teardown
 *		descriptor queue
 * @num_order:	number of teardown descriptors as a power of two (at least 5)
 * @sched_tbl:	the DMA scheduler table
 * @tbl_size:	number of entries in the DMA scheduler table
 *
 * This function frees the memory region allocated by cppi41_mem_rgn_alloc().
 *
 * Returns 0 on success, error otherwise.
 */
int cppi41_dma_block_init(u8 dma_num, u8 q_mgr, u8 num_order,
				 u32 *sched_tbl, u8 tbl_size);

/**
 * cppi41_dma_block_init - CPPI 4.1 DMA block un-initialization.
 * @dma_num:	number of the DMA block
 * @q_mgr:	the queue manager in which to allocate the free teardown
 *		descriptor queue
 * @num_order:	number of teardown descriptors as a power of two (at least 5)
 * @sched_tbl:	the DMA scheduler table
 * @tbl_size:	number of entries in the DMA scheduler table
 *
 * Returns 0 on success, error otherwise.
 */
int cppi41_dma_block_uninit(u8 dma_num, u8 q_mgr, u8 num_order,
				 u32 *sched_tbl, u8 tbl_size);

/*
 * CPPI 4.1 DMA Channel Management APIs
 */

/**
 * cppi41_tx_ch_init - initialize CPPI 4.1 transmit channel object
 * @tx_ch_obj:	pointer to Tx channel object
 * @dma_num:	DMA block to which this channel belongs
 * @ch_num:	DMA channel number
 *
 * Returns 0 if valid Tx channel, -EINVAL otherwise.
 */
int cppi41_tx_ch_init(struct cppi41_dma_ch_obj *tx_ch_obj,
		      u8 dma_num, u8 ch_num);

/**
 * cppi41_rx_ch_init - initialize CPPI 4.1 receive channel object
 * @rx_ch_obj:	pointer to Rx channel object
 * @dma_num:	DMA block to which this channel belongs
 * @ch_num:	DMA channel number
 *
 * Returns 0 if valid Rx channel, -EINVAL otherwise.
 */
int cppi41_rx_ch_init(struct cppi41_dma_ch_obj *rx_ch_obj,
		      u8 dma_num, u8 ch_num);

/**
 * cppi41_dma_ch_default_queue - set CPPI 4.1 channel default completion queue
 * @dma_ch_obj: pointer to DMA channel object
 * @q_mgr:	default queue manager
 * @q_num:	default queue number
 *
 * This function configures the specified channel.  The caller is required to
 * provide the default queue onto which the teardown descriptors will be queued.
 */
void cppi41_dma_ch_default_queue(struct cppi41_dma_ch_obj *dma_ch_obj,
				 u8 q_mgr, u16 q_num);

/**
 * cppi41_rx_ch_configure - configure CPPI 4.1 receive channel
 * @rx_ch_obj:	pointer to Rx channel object
 * @cfg:	pointer to Rx channel configuration
 *
 * This function configures and opens the specified Rx channel.  The caller
 * is required to provide channel configuration information by initializing
 * a struct cppi41_rx_ch_cfg.
 */
void cppi41_rx_ch_configure(struct cppi41_dma_ch_obj *rx_ch_obj,
			    struct cppi41_rx_ch_cfg  *cfg);

/**
 * cppi41_rx_ch_set_maxbufcnt - configure max rx buffer count
 * @rx_ch_obj:	pointer to Rx channel object
 * rx_max_buf_cnt: maximum rx buffer count
 *
 * This function configures the maximum rx buffer count in rx dma
 * global configuration register. The valid rx_max_buf_cnt value
 * must be 0 to 4.
 */
void cppi41_rx_ch_set_maxbufcnt(struct cppi41_dma_ch_obj *rx_ch_obj,
			    u8 rx_max_buf_cnt);
/**
 * cppi41_dma_ch_enable - enable CPPI 4.1 Tx/Rx DMA channel
 * @dma_ch_obj:	pointer to DMA channel object
 *
 * This function enables  a specified Tx channel.  The caller is required to
 * provide a reference to a channel object initialized by an earlier call of
 * the cppi41_dma_ch_init() function.  After the successful completion of this
 * function, the Tx DMA channel will be active and ready for data transmission.
 */
void cppi41_dma_ch_enable(struct cppi41_dma_ch_obj *dma_ch_obj);

/**
 * cppi41_dma_ch_disable - disable CPPI 4.1 Tx/Rx DMA channel
 * @dma_ch_obj:	pointer to DMA channel object
 *
 * This function disables a specific Tx channel.  The caller is required to
 * provide a reference to a channel object initialized by an earlier call of
 * the cppi41_dma_ch_init() function.  After the successful completion of this
 * function, the Tx DMA channel will be deactived.
 */
void cppi41_dma_ch_disable(struct cppi41_dma_ch_obj *dma_ch_obj);

/**
 * cppi41_dma_ch_teardown - tear down CPPI 4.1 transmit channel
 * @dma_ch_obj:	pointer DMA channel object
 *
 * This function triggers the teardown of the given DMA channel.
 *
 * ATTENTION: Channel disable should not be called before the teardown is
 * completed as a disable will stop the DMA scheduling on the channel resulting
 * in the teardown complete event not being registered at all.
 *
 * NOTE: A successful channel teardown event is reported via queueing of a
 * teardown descriptor.
 *
 * This function just sets up for the teardown of the channel and returns. The
 * caller must detect the channel teardown event to assume that the channel is
 * disabled.
 *
 * See cppi41_get_teardown_info() for the teardown completion processing.
 */
void cppi41_dma_ch_teardown(struct cppi41_dma_ch_obj *dma_ch_obj);

/*
 * CPPI 4.1 Queue Allocation and De-allocation APIs.
 */

/**
 * cppi41_queue_alloc - allocate CPPI 4.1 queue
 * @type:	queue type bitmask
 * @q_mgr:	queue manager
 * @q_num:	pointer to the queue number
 *
 * Returns 0 if queue allocated, error otherwise.
 */
int cppi41_queue_alloc(u8 type, u8 q_mgr, u16 *q_num);

/**
 * cppi41_queue_free - de-allocate CPPI 4.1 queue
 * @q_mgr:	queue manager
 * @q_num:	queue number
 *
 * Returns 0 on success, -EINVAL otherwise.
 */
int cppi41_queue_free(u8 q_mgr, u16 q_num);

/*
 *  CPPI 4.1 Queue Management APIs
 */

/**
 * cppi41_queue_init - initialize CPPI 4.1 queue object
 * @queue_obj:	pointer to the queue object
 * @q_mgr:	queue manager
 * @q_num:	queue number
 *
 * Returns 0 if valid queue, -EINVAL otherwise.
 */
int cppi41_queue_init(struct cppi41_queue_obj *queue_obj, u8 q_mgr, u16 q_num);

/**
 * cppi41_queue_push - push to CPPI 4.1 queue
 * @queue_obj:	pointer to the queue object
 * @desc_addr:	descriptor physical address
 * @desc_size:	descriptor size
 * @pkt_size:	packet size
 *
 * This function is called to queue a descriptor onto a queue.
 * NOTE: pSize parameter is optional. Pass 0 in case not required.
 */
void cppi41_queue_push(const struct cppi41_queue_obj *queue_obj, u32 desc_addr,
		       u32 desc_size, u32 pkt_size);

/**
 * cppi41_queue_pop - pop from CPPI 4.1 queue
 * @queue_obj:	pointer to the queue object
 *
 * This function is called to pop a single descriptor from the queue.
 *
 * Returns a packet descriptor's physical address.
 */
unsigned long cppi41_queue_pop(const struct cppi41_queue_obj *queue_obj);

/*
 * CPPI 4.1 Miscellaneous APIs
 */

/**
 * cppi41_get_teardown_info - CPPI 4.1 teardown completion processing function
 *
 * @addr:	physical address of teardown descriptor
 * @info:	pointer to the teardown information word
 *
 * This function is called to complete the teardown processing on a channel
 * and provides teardown information from the teardown descriptor passed to it.
 * It also recycles the teardown descriptor back to the teardown descriptor
 * queue.
 *
 * Returns 0 if valid descriptor, -EINVAL otherwise.
 */
int cppi41_get_teardown_info(unsigned long addr, u32 *info);

/**
 * cppi41_dma_sched_tbl_init
 */
int cppi41_dma_sched_tbl_init(u8 dma_num, u8 q_mgr,
				u32 *sched_tbl, u8 tbl_size);

/**
 * cppi41_schedtbl_add_dma_ch - add a dma channel to schedular table
 *
 * @dmanum      Number of DMa block
 * @qmgr        Queue Manager Number
 * @dma_ch      dma channel number
 * @is_tx       transmit (is_tx=1) or recieve(is_tx=0)
 *
 * returns      number of channel in schedular table
 */
int cppi41_schedtbl_add_dma_ch(u8 dmanum, u8 qmgr, u8 dma_ch, u8 is_tx);

/**
 * cppi41_schedtbl_remove_dma_ch - remove a dma channel from schedular table
 *
 * @dmanum      Number of DMa block
 * @qmgr        Queue Manager Number
 * @dma_ch      dma channel number
 * @is_tx       transmit (is_tx=1) or recieve(is_tx=0)
 *
 * returns      number of channel in schedular table
 */
int cppi41_schedtbl_remove_dma_ch(u8 dmanum, u8 qmgr, u8 dma_ch, u8 is_tx);

/**
 * cppi41_init_teardown_queue
 */
void cppi41_init_teardown_queue(int dma_num);

/**
 * cppi41_free_teardown_queue
 */
void cppi41_free_teardown_queue(int dma_num);

/**
 * cppi41_save_context
 */
void cppi41_save_context(u8 dma_num);

/**
 * cppi41_restore_context
 */
void cppi41_restore_context(u8 dma_num, u32 *sched_tbl);
