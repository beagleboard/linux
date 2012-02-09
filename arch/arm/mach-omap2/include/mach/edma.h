/*
 * TI EDMA3 definitions
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

/*
 * This EDMA3 programming framework exposes two basic kinds of resource:
 *
 *  Channel	Triggers transfers, usually from a hardware event but
 *		also manually or by "chaining" from DMA completions.
 *		Each channel is coupled to a Parameter RAM (PaRAM) slot.
 *
 *  Slot	Each PaRAM slot holds a DMA transfer descriptor (PaRAM
 *		"set"), source and destination addresses, a link to a
 *		next PaRAM slot (if any), options for the transfer, and
 *		instructions for updating those addresses.  There are
 *		more than twice as many slots as event channels.
 *
 * Each PaRAM set describes a sequence of transfers, either for one large
 * buffer or for several discontiguous smaller buffers.  An EDMA transfer
 * is driven only from a channel, which performs the transfers specified
 * in its PaRAM slot until there are no more transfers.  When that last
 * transfer completes, the "link" field may be used to reload the channel's
 * PaRAM slot with a new transfer descriptor.
 *
 * The EDMA Channel Controller (CC) maps requests from channels into physical
 * Transfer Controller (TC) requests when the channel triggers (by hardware
 * or software events, or by chaining).  The two physical DMA channels provided
 * by the TCs are thus shared by many logical channels.
 *
 * EDMA hardware also has a "QDMA" mechanism which is not currently
 * supported through this interface.  (DSP firmware uses it though.)
 */

#ifndef EDMA_H_
#define EDMA_H_

/* PaRAM slots are laid out like this */
struct edmacc_param {
	unsigned int opt;
	unsigned int src;
	unsigned int a_b_cnt;
	unsigned int dst;
	unsigned int src_dst_bidx;
	unsigned int link_bcntrld;
	unsigned int src_dst_cidx;
	unsigned int ccnt;
};

/* fields in edmacc_param.opt */
#define SAM		BIT(0)
#define DAM		BIT(1)
#define SYNCDIM		BIT(2)
#define STATIC		BIT(3)
#define EDMA_FWID	(0x07 << 8)
#define TCCMODE		BIT(11)
#define EDMA_TCC(t)	((t) << 12)
#define TCINTEN		BIT(20)
#define ITCINTEN	BIT(21)
#define TCCHEN		BIT(22)
#define ITCCHEN		BIT(23)

#define TRWORD (0x7<<2)
#define PAENTRY (0x1ff<<5)

/*ch_status paramater of callback function possible values*/
#define DMA_COMPLETE 1
#define DMA_CC_ERROR 2
#define DMA_TC0_ERROR 3
#define DMA_TC1_ERROR 4
#define DMA_TC2_ERROR 5
#define DMA_TC3_ERROR 6

enum address_mode {
	INCR = 0,
	FIFO = 1
};

enum fifo_width {
	W8BIT = 0,
	W16BIT = 1,
	W32BIT = 2,
	W64BIT = 3,
	W128BIT = 4,
	W256BIT = 5
};

enum dma_event_q {
	EVENTQ_0 = 0,
	EVENTQ_1 = 1,
	EVENTQ_2 = 2,
	EVENTQ_3 = 3,
	EVENTQ_DEFAULT = -1
};

enum sync_dimension {
	ASYNC = 0,
	ABSYNC = 1
};

#define EDMA_CTLR_CHAN(ctlr, chan)	(((ctlr) << 16) | (chan))
#define EDMA_CTLR(i)			((i) >> 16)
#define EDMA_CHAN_SLOT(i)		((i) & 0xffff)

#define EDMA_CHANNEL_ANY		-1	/* for edma_alloc_channel() */
#define EDMA_SLOT_ANY			-1	/* for edma_alloc_slot() */
#define EDMA_CONT_PARAMS_ANY		 1001
#define EDMA_CONT_PARAMS_FIXED_EXACT	 1002
#define EDMA_CONT_PARAMS_FIXED_NOT_EXACT 1003

#define EDMA_MAX_DMACH           64
#define EDMA_MAX_PARAMENTRY     512
#define EDMA_MAX_CC               2
#define EDMA_MAX_REGION           4


/* Mapping of crossbar event numbers to actual DMA channels*/
struct event_to_channel_map {
	unsigned        xbar_event_no;
	int		channel_no;
};

/* actual number of DMA channels and slots on this silicon */
struct edma {
	/* how many dma resources of each type */
	unsigned	num_channels;
	unsigned	num_region;
	unsigned	num_slots;
	unsigned	num_tc;
	unsigned	num_cc;
	enum dma_event_q	default_queue;

	/* list of channels with no even trigger; terminated by "-1" */
	const s8	*noevent;

	/* The edma_inuse bit for each PaRAM slot is clear unless the
	 * channel is in use ... by ARM or DSP, for QDMA, or whatever.
	 */
	DECLARE_BITMAP(edma_inuse, EDMA_MAX_PARAMENTRY);

	/* The edma_unused bit for each channel is clear unless
	 * it is not being used on this platform. It uses a bit
	 * of SOC-specific initialization code.
	 */
	DECLARE_BITMAP(edma_unused, EDMA_MAX_DMACH);

	unsigned	irq_res_start;
	unsigned	irq_res_end;

	struct dma_interrupt_data {
		void (*callback)(unsigned channel, unsigned short ch_status,
				void *data);
		void *data;
	} intr_data[EDMA_MAX_DMACH];

	unsigned	is_xbar;
	unsigned	num_events;
	struct event_to_channel_map	*xbar_event_mapping;

	/* suspend/resume backup parameters */
	struct edmacc_param *bkp_prm_set;
	unsigned int *bkp_ch_map;		/* 64 registers */
	unsigned int *bkp_que_num;		/* 8 registers */
	unsigned int *bkp_drae;
	unsigned int *bkp_draeh;
	unsigned int *bkp_qrae;
	unsigned int bkp_sh_esr;
	unsigned int bkp_sh_esrh;
	unsigned int bkp_sh_eesr;
	unsigned int bkp_sh_eesrh;
	unsigned int bkp_sh_iesr;
	unsigned int bkp_sh_iesrh;
	unsigned int bkp_que_tc_map;
	unsigned int bkp_que_pri;
};

extern struct edma *edma_cc[EDMA_MAX_CC];

/* alloc/free DMA channels and their dedicated parameter RAM slots */
int edma_alloc_channel(int channel,
	void (*callback)(unsigned channel, u16 ch_status, void *data),
	void *data, enum dma_event_q);
void edma_free_channel(unsigned channel);

/* alloc/free parameter RAM slots */
int edma_alloc_slot(unsigned ctlr, int slot);
void edma_free_slot(unsigned slot);

/* alloc/free a set of contiguous parameter RAM slots */
int edma_alloc_cont_slots(unsigned ctlr, unsigned int id, int slot, int count);
int edma_free_cont_slots(unsigned slot, int count);

/* calls that operate on part of a parameter RAM slot */
void edma_set_src(unsigned slot, dma_addr_t src_port,
				enum address_mode mode, enum fifo_width);
void edma_set_dest(unsigned slot, dma_addr_t dest_port,
				 enum address_mode mode, enum fifo_width);
void edma_get_position(unsigned slot, dma_addr_t *src, dma_addr_t *dst);
void edma_set_src_index(unsigned slot, s16 src_bidx, s16 src_cidx);
void edma_set_dest_index(unsigned slot, s16 dest_bidx, s16 dest_cidx);
void edma_set_transfer_params(unsigned slot, u16 acnt, u16 bcnt, u16 ccnt,
		u16 bcnt_rld, enum sync_dimension sync_mode);
void edma_link(unsigned from, unsigned to);
void edma_unlink(unsigned from);

/* calls that operate on an entire parameter RAM slot */
void edma_write_slot(unsigned slot, const struct edmacc_param *params);
void edma_read_slot(unsigned slot, struct edmacc_param *params);

/* channel control operations */
int edma_start(unsigned channel);
void edma_stop(unsigned channel);
void edma_clean_channel(unsigned channel);
void edma_clear_event(unsigned channel);
void edma_pause(unsigned channel);
void edma_resume(unsigned channel);

/* platform_data for EDMA driver */
struct edma_soc_info {

	/* how many dma resources of each type */
	unsigned	n_channel;
	unsigned	n_region;
	unsigned	n_slot;
	unsigned	n_tc;
	unsigned	n_cc;
	enum dma_event_q	default_queue;

	const s16	(*rsv_chans)[2];
	const s16	(*rsv_slots)[2];
	const s8	(*queue_tc_mapping)[2];
	const s8	(*queue_priority_mapping)[2];
	unsigned	is_xbar;
	unsigned	n_events;
	struct event_to_channel_map    *xbar_event_mapping;
	int		(*map_xbar_channel)(unsigned event, unsigned *channel,
				struct event_to_channel_map *xbar_event_map);
};

#endif
