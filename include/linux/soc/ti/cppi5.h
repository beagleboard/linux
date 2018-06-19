/* SPDX-License-Identifier: GPL-2.0 */
/*
 * K3 DMA descriptors interface
 *
 * Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com
 */

#ifndef __CPPI5_H__
#define __CPPI5_H__

#include <linux/bitops.h>

/**
 * Descriptor header, present in all types of descriptors
 */
struct knav_udmap_desc_hdr_t {
	u32 pkt_info0;	/* Packet info word 0 (n/a in Buffer desc) */
	u32 pkt_info1;	/* Packet info word 1 (n/a in Buffer desc) */
	u32 pkt_info2;	/* Packet info word 2 Buffer reclamation info */
	u32 src_dst_tag; /* Packet info word 3 (n/a in Buffer desc) */
} __packed;

/**
 * Host-mode packet and buffer descriptor definition
 */
struct knav_udmap_host_desc_t {
	struct knav_udmap_desc_hdr_t hdr;
	u64 next_desc;	/* w4/5: Linking word */
	u64 buf_ptr;	/* w6/7: Buffer pointer */
	u32 buf_info1;	/* w8: Buffer valid data length */
	u32 org_buf_len; /* w9: Original buffer length */
	u64 org_buf_ptr; /* w10/11: Original buffer pointer */
	u32 epib[0];	/* Extended Packet Info Data (optional, 4 words) */
	/*
	 * Protocol Specific Data (optional, 0-128 bytes in multiples of 4),
	 * and/or Other Software Data (0-N bytes, optional)
	 */
} __packed;

#define KNAV_UDMAP_DESC_MIN_ALIGN		(16U)

#define KNAV_UDMAP_INFO0_HDESC_EPIB_SIZE	(16U)
#define KNAV_UDMAP_INFO0_HDESC_PSDATA_MAX_SIZE	(128U)

#define KNAV_UDMAP_INFO0_HDESC_TYPE_SHIFT	(30U)
#define KNAV_UDMAP_INFO0_HDESC_TYPE_MASK	GENMASK(31, 30)
#define   KNAV_UDMAP_INFO0_DESC_TYPE_VAL_HOST	(1U)
#define   KNAV_UDMAP_INFO0_DESC_TYPE_VAL_MONO	(2U)
#define   KNAV_UDMAP_INFO0_DESC_TYPE_VAL_TR	(3U)
#define KNAV_UDMAP_INFO0_HDESC_EPIB_PRESENT	BIT(29)
/*
 * Protocol Specific Words location:
 * 0 - located in the descriptor,
 * 1 = located in the SOP Buffer immediately prior to the data.
 */
#define KNAV_UDMAP_INFO0_HDESC_PSINFO_LOCATION		BIT(28)
#define KNAV_UDMAP_INFO0_HDESC_PSINFO_SIZE_SHIFT	(22U)
#define KNAV_UDMAP_INFO0_HDESC_PSINFO_SIZE_MASK		GENMASK(27, 22)
#define KNAV_UDMAP_INFO0_HDESC_PKTLEN_SHIFT		(0)
#define KNAV_UDMAP_INFO0_HDESC_PKTLEN_MASK		GENMASK(21, 0)

#define KNAV_UDMAP_INFO1_DESC_PKTERROR_SHIFT	(28U)
#define KNAV_UDMAP_INFO1_DESC_PKTERROR_MASK	GENMASK(31, 28)
#define KNAV_UDMAP_INFO1_HDESC_PSFLGS_SHIFT	(24U)
#define KNAV_UDMAP_INFO1_HDESC_PSFLGS_MASK	GENMASK(27, 24)
#define KNAV_UDMAP_INFO1_DESC_PKTID_SHIFT	(14U)
#define KNAV_UDMAP_INFO1_DESC_PKTID_MASK	GENMASK(23, 14)
#define KNAV_UDMAP_INFO1_DESC_FLOWID_SHIFT	(0)
#define KNAV_UDMAP_INFO1_DESC_FLOWID_MASK	GENMASK(13, 0)

#define KNAV_UDMAP_INFO2_HDESC_PKTTYPE_SHIFT	(27U)
#define KNAV_UDMAP_INFO2_HDESC_PKTTYPE_MASK	GENMASK(31, 27)
/* Return Policy: 0 - Entire packet 1 - Each buffer */
#define KNAV_UDMAP_INFO2_HDESC_RETPOLICY	BIT(18)
/*
 * Early Return:
 * 0 = desc pointers should be returned after all reads have been completed
 * 1 = desc pointers should be returned immediately upon fetching
 * the descriptor and beginning to transfer data.
 */
#define KNAV_UDMAP_INFO2_HDESC_EARLYRET		BIT(17)
/*
 * Return Push Policy:
 * 0 = Descriptor must be returned to tail of queue
 * 1 = Descriptor must be returned to head of queue
 */
#define KNAV_UDMAP_INFO2_DESC_RETPUSHPOLICY	BIT(16)
#define KNAV_UDMAP_INFO2_DESC_RETQ_SHIFT	(0)
#define KNAV_UDMAP_INFO2_DESC_RETQ_MASK		GENMASK(15, 0)

#define KNAV_UDMAP_INFO3_DESC_SRCTAG_SHIFT	(16U)
#define KNAV_UDMAP_INFO3_DESC_SRCTAG_MASK	GENMASK(31, 16)
#define KNAV_UDMAP_INFO3_DESC_DSTTAG_SHIFT	(0)
#define KNAV_UDMAP_INFO3_DESC_DSTTAG_MASK	GENMASK(15, 0)

#define KNAV_UDMAP_BUFINFO1_HDESC_DATA_LEN_SHIFT	(0)
#define KNAV_UDMAP_BUFINFO1_HDESC_DATA_LEN_MASK		GENMASK(27, 0)

#define KNAV_UDMAP_OBUFINFO0_HDESC_BUF_LEN_SHIFT	(0)
#define KNAV_UDMAP_OBUFINFO0_HDESC_BUF_LEN_MASK		GENMASK(27, 0)

/*
 * Host Packet Descriptor Extended Packet Info Block
 */
struct knav_udmap_desc_epib_t {
	u32 timestamp;	/* w0: application specific timestamp */
	u32 sw_info0;	/* w1: Software Info 0 */
	u32 sw_info1;	/* w2: Software Info 1 */
	u32 sw_info2;	/* w3: Software Info 2 */
};

/**
 * Monolithic-mode packet descriptor
 */
struct knav_udmap_monolithic_desc_t {
	struct knav_udmap_desc_hdr_t hdr;
	u32 epib[0];	/* Extended Packet Info Data (optional, 4 words) */
	/*
	 * Protocol Specific Data (optional, 0-128 bytes in multiples of 4),
	 *  and/or Other Software Data (0-N bytes, optional)
	 */
};

#define KNAV_UDMAP_INFO2_MDESC_DATA_OFFSET_SHIFT	(18U)
#define KNAV_UDMAP_INFO2_MDESC_DATA_OFFSET_MASK		GENMASK(26, 18)

/**
 * Transfer Request Descriptor (TR)
 */
struct knav_udmap_tr_desc_t {
	struct knav_udmap_desc_hdr_t hdr;
	u8	tr_records[0];
};

/*
 * Reload Enable:
 * 0 = Finish the packet and place the descriptor back on the return queue
 * 1 = Vector to the Reload Index and resume processing
 */
#define KNAV_UDMAP_INFO0_TRDESC_RELOAD		BIT(28)
#define KNAV_UDMAP_INFO0_TRDESC_RLDIDX_SHIFT	(14U)
#define KNAV_UDMAP_INFO0_TRDESC_RLDIDX_MASK	GENMASK(27, 14)
#define KNAV_UDMAP_INFO0_TRDESC_LASTIDX_SHIFT	(0)
#define KNAV_UDMAP_INFO0_TRDESC_LASTIDX_MASK	GENMASK(13, 0)

#define KNAV_UDMAP_INFO1_TRDESC_RECSIZE_SHIFT	(24U)
#define KNAV_UDMAP_INFO1_TRDESC_RECSIZE_MASK	GENMASK(26, 24)
#define   KNAV_UDMAP_INFO1_TRDESC_RECSIZE_VAL_16B	(0)
#define   KNAV_UDMAP_INFO1_TRDESC_RECSIZE_VAL_32B	(1U)
#define   KNAV_UDMAP_INFO1_TRDESC_RECSIZE_VAL_64B	(2U)
#define   KNAV_UDMAP_INFO1_TRDESC_RECSIZE_VAL_128B	(3U)

static inline void knav_udmap_desc_dump(void *desc, u32 size)
{
	print_hex_dump(KERN_ERR, "dump udmap_desc: ", DUMP_PREFIX_NONE,
		       32, 4, desc, size, false);
}

/**
 * knav_udmap_hdesc_calc_size - Calculate Host Packet Descriptor size
 * @epib: is EPIB present
 * @psdata_size: PSDATA size
 * @sw_data_size: SWDATA size
 *
 * Returns required Host Packet Descriptor size
 * 0 - if PSDATA > KNAV_UDMAP_INFO0_HDESC_PSDATA_MAX_SIZE
 */
static inline u32 knav_udmap_hdesc_calc_size(bool epib, u32 psdata_size,
					     u32 sw_data_size)
{
	u32 desc_size;

	if (psdata_size > KNAV_UDMAP_INFO0_HDESC_PSDATA_MAX_SIZE)
		return 0;
	//TODO_GS: align
	desc_size = sizeof(struct knav_udmap_host_desc_t) + psdata_size +
		    sw_data_size;

	if (epib)
		desc_size += KNAV_UDMAP_INFO0_HDESC_EPIB_SIZE;

	return ALIGN(desc_size, KNAV_UDMAP_DESC_MIN_ALIGN);
}

/**
 * knav_udmap_hdesc_init - Init Host Packet Descriptor size
 * @desc: Host packet descriptor
 * @flags: supported values
 *	KNAV_UDMAP_INFO0_HDESC_EPIB_PRESENT
 *	KNAV_UDMAP_INFO0_HDESC_PSINFO_LOCATION
 * @psdata_size: PSDATA size
 *
 * Returns required Host Packet Descriptor size
 * 0 - if PSDATA > KNAV_UDMAP_INFO0_HDESC_PSDATA_MAX_SIZE
 */
static inline void knav_udmap_hdesc_init(struct knav_udmap_host_desc_t *desc,
					 u32 flags, u32 psdata_size)
{
	WARN_ON(!desc);
	WARN_ON(psdata_size > KNAV_UDMAP_INFO0_HDESC_PSDATA_MAX_SIZE);
	WARN_ON(flags & ~(KNAV_UDMAP_INFO0_HDESC_EPIB_PRESENT |
			 KNAV_UDMAP_INFO0_HDESC_PSINFO_LOCATION));

	desc->hdr.pkt_info0 = (KNAV_UDMAP_INFO0_DESC_TYPE_VAL_HOST <<
			       KNAV_UDMAP_INFO0_HDESC_TYPE_SHIFT) | (flags);
	desc->hdr.pkt_info0 |=
			((psdata_size >> 2) <<
			  KNAV_UDMAP_INFO0_HDESC_PSINFO_SIZE_SHIFT) &
			  KNAV_UDMAP_INFO0_HDESC_PSINFO_SIZE_MASK;
	desc->next_desc = 0;
}

/**
 * knav_udmap_hdesc_update_flags - Replace descriptor flags
 * @desc: Host packet descriptor
 * @flags: supported values
 *	KNAV_UDMAP_INFO0_HDESC_EPIB_PRESENT
 *	KNAV_UDMAP_INFO0_HDESC_PSINFO_LOCATION
 */
static inline void knav_udmap_hdesc_update_flags(
	struct knav_udmap_host_desc_t *desc, u32 flags)
{
	WARN_ON(!desc);
	WARN_ON(flags & ~(KNAV_UDMAP_INFO0_HDESC_EPIB_PRESENT |
			 KNAV_UDMAP_INFO0_HDESC_PSINFO_LOCATION));

	desc->hdr.pkt_info0 &= ~(KNAV_UDMAP_INFO0_HDESC_EPIB_PRESENT |
				 KNAV_UDMAP_INFO0_HDESC_PSINFO_LOCATION);
	desc->hdr.pkt_info0 |= flags;
}

/**
 * knav_udmap_hdesc_update_psdata_size - Replace PSdata size
 * @desc: Host packet descriptor
 * @psdata_size: PSDATA size
 */
static inline void knav_udmap_hdesc_update_psdata_size(
	struct knav_udmap_host_desc_t *desc, u32 psdata_size)
{
	WARN_ON(!desc);
	WARN_ON(psdata_size > KNAV_UDMAP_INFO0_HDESC_PSDATA_MAX_SIZE);

	desc->hdr.pkt_info0 &= ~KNAV_UDMAP_INFO0_HDESC_PSINFO_SIZE_MASK;
	desc->hdr.pkt_info0 |= ((psdata_size >> 2) <<
				KNAV_UDMAP_INFO0_HDESC_PSINFO_SIZE_SHIFT) &
				KNAV_UDMAP_INFO0_HDESC_PSINFO_SIZE_MASK;
}

/**
 * knav_udmap_hdesc_get_psdata_size - get PSdata size in bytes
 * @desc: Host packet descriptor
 */
static inline u32 knav_udmap_hdesc_get_psdata_size(
	struct knav_udmap_host_desc_t *desc)
{
	u32 psdata_size = 0;

	WARN_ON(!desc);

	if (!(desc->hdr.pkt_info0 & KNAV_UDMAP_INFO0_HDESC_PSINFO_LOCATION))
		psdata_size = (desc->hdr.pkt_info0 &
			       KNAV_UDMAP_INFO0_HDESC_PSINFO_SIZE_MASK) >>
			       KNAV_UDMAP_INFO0_HDESC_PSINFO_SIZE_SHIFT;

	return (psdata_size << 2);
}

/**
 * knav_udmap_desc_get_type - get descriptor type
 * @desc_hdr: packet descriptor/TR header
 *
 * Returns descriptor type:
 * KNAV_UDMAP_INFO0_DESC_TYPE_VAL_HOST
 * KNAV_UDMAP_INFO0_DESC_TYPE_VAL_MONO
 * KNAV_UDMAP_INFO0_DESC_TYPE_VAL_TR
 */
static inline u32
knav_udmap_desc_get_type(struct knav_udmap_desc_hdr_t *desc_hdr)
{
	WARN_ON(!desc_hdr);

	return (desc_hdr->pkt_info0 & KNAV_UDMAP_INFO0_HDESC_TYPE_MASK) >>
		KNAV_UDMAP_INFO0_HDESC_TYPE_SHIFT;
}

/**
 * knav_udmap_hdesc_get_pktlen - get Packet Length from HDesc
 * @desc: Host packet descriptor
 *
 * Returns Packet Length from Host Packet Descriptor
 */
static inline u32
knav_udmap_hdesc_get_pktlen(struct knav_udmap_host_desc_t *desc)
{
	WARN_ON(!desc);

	return (desc->hdr.pkt_info0 & KNAV_UDMAP_INFO0_HDESC_PKTLEN_MASK);
}

/**
 * knav_udmap_hdesc_set_pktlen - set Packet Length in HDesc
 * @desc: Host packet descriptor
 */
static inline void
knav_udmap_hdesc_set_pktlen(struct knav_udmap_host_desc_t *desc, u32 pkt_len)
{
	WARN_ON(!desc);

	desc->hdr.pkt_info0 |= (pkt_len & KNAV_UDMAP_INFO0_HDESC_PKTLEN_MASK);
}

/**
 * knav_udmap_desc_get_errflags - get Error Flags from Desc
 * @desc_hdr: packet/TR descriptor header
 *
 * Returns Error Flags from Packet/TR Descriptor
 */
static inline u32
knav_udmap_desc_get_errflags(struct knav_udmap_desc_hdr_t *desc_hdr)
{
	WARN_ON(!desc_hdr);

	return (desc_hdr->pkt_info1 & KNAV_UDMAP_INFO1_DESC_PKTERROR_MASK) >>
		KNAV_UDMAP_INFO1_DESC_PKTERROR_SHIFT;
}

/**
 * knav_udmap_hdesc_get_psflags - get Protocol Specific Flags from HDesc
 * @desc: Host packet descriptor
 *
 * Returns Protocol Specific Flags from Host Packet Descriptor
 */
static inline u32
knav_udmap_hdesc_get_psflags(struct knav_udmap_host_desc_t *desc)
{
	WARN_ON(!desc);

	return (desc->hdr.pkt_info1 & KNAV_UDMAP_INFO1_HDESC_PSFLGS_MASK) >>
		KNAV_UDMAP_INFO1_HDESC_PSFLGS_SHIFT;
}

/**
 * knav_udmap_hdesc_set_psflags - set Protocol Specific Flags in HDesc
 * @desc: Host packet descriptor
 */
static inline void
knav_udmap_hdesc_set_psflags(struct knav_udmap_host_desc_t *desc, u32 ps_flags)
{
	WARN_ON(!desc);

	desc->hdr.pkt_info1 |= (ps_flags <<
				KNAV_UDMAP_INFO1_HDESC_PSFLGS_SHIFT) &
				KNAV_UDMAP_INFO1_HDESC_PSFLGS_MASK;
}

/**
 * knav_udmap_hdesc_get_pktids - get Packet and Flow ids from Desc
 * @desc_hdr: packet/TR descriptor header
 * @pkt_id: Packet ID
 * @flow_id: Flow ID
 *
 * Returns Packet and Flow ids from packet/TR descriptor
 */
static inline void
knav_udmap_hdesc_get_pktids(struct knav_udmap_desc_hdr_t *desc_hdr,
			    u32 *pkt_id, u32 *flow_id)
{
	WARN_ON(!desc_hdr);

	*pkt_id = (desc_hdr->pkt_info1 & KNAV_UDMAP_INFO1_DESC_PKTID_MASK) >>
		   KNAV_UDMAP_INFO1_DESC_PKTID_SHIFT;
	*flow_id = (desc_hdr->pkt_info1 & KNAV_UDMAP_INFO1_DESC_FLOWID_MASK) >>
		    KNAV_UDMAP_INFO1_DESC_FLOWID_SHIFT;
}

/**
 * knav_udmap_hdesc_get_errflags - set Packet and Flow ids in Desc
 * @desc_hdr: packet/TR descriptor header
 * @pkt_id: Packet ID
 * @flow_id: Flow ID
 */
static inline void
knav_udmap_hdesc_set_pktids(struct knav_udmap_desc_hdr_t *desc_hdr,
			    u32 pkt_id, u32 flow_id)
{
	WARN_ON(!desc_hdr);

	desc_hdr->pkt_info1 |= (pkt_id <<
				KNAV_UDMAP_INFO1_DESC_PKTID_SHIFT) &
				KNAV_UDMAP_INFO1_DESC_PKTID_MASK;
	desc_hdr->pkt_info1 |= (flow_id <<
				KNAV_UDMAP_INFO1_DESC_FLOWID_SHIFT) &
				KNAV_UDMAP_INFO1_DESC_FLOWID_MASK;
}

/**
 * knav_udmap_hdesc_get_errflags - set Packet Return Policy in Desc
 * @desc_hdr: packet/TR descriptor header
 * @flags: fags, supported values
 *  KNAV_UDMAP_INFO2_HDESC_RETPOLICY
 *  KNAV_UDMAP_INFO2_HDESC_EARLYRET
 *  KNAV_UDMAP_INFO2_DESC_RETPUSHPOLICY
 * @return_ring_id: Packet Return Queue/Ring id, value 0xFFFF reserved
 */
static inline void knav_udmap_desc_set_retpolicy(
		struct knav_udmap_desc_hdr_t *desc_hdr,
		u32 flags, u32 return_ring_id)
{
	WARN_ON(!desc_hdr);

	desc_hdr->pkt_info2 |= flags;
	desc_hdr->pkt_info2 |= return_ring_id &
			       KNAV_UDMAP_INFO2_DESC_RETQ_MASK;
}

/**
 * knav_udmap_hdesc_get_errflags - get Packet Type from HDesc
 * @desc: Host packet descriptor
 */
static inline u32
knav_udmap_hdesc_get_pkttype(struct knav_udmap_host_desc_t *desc)
{
	WARN_ON(!desc);

	return (desc->hdr.pkt_info2 & KNAV_UDMAP_INFO2_HDESC_PKTTYPE_MASK) >>
		KNAV_UDMAP_INFO2_HDESC_PKTTYPE_SHIFT;
}

/**
 * knav_udmap_hdesc_get_errflags - set Packet Type in HDesc
 * @desc: Host packet descriptor
 * @pkt_type: Packet Type
 */
static inline void
knav_udmap_hdesc_set_pkttype(struct knav_udmap_host_desc_t *desc, u32 pkt_type)
{
	WARN_ON(!desc);
	desc->hdr.pkt_info2 |=
			(pkt_type << KNAV_UDMAP_INFO2_HDESC_PKTTYPE_SHIFT) &
			 KNAV_UDMAP_INFO2_HDESC_PKTTYPE_MASK;
}

/**
 * knav_udmap_hdesc_get_pktids - get Packet Src/Dst Tags from Desc
 * @desc_hdr: packet/TR descriptor header
 * @src_tag_id: Source Tag
 * @dst_tag_id: Dest Tag
 *
 * Returns Packet Src/Dst Tags from packet/TR descriptor
 */
static inline void
knav_udmap_desc_get_tags_ids(struct knav_udmap_desc_hdr_t *desc_hdr,
			     u32 *src_tag_id, u32 *dst_tag_id)
{
	WARN_ON(!desc_hdr);

	if (src_tag_id)
		*src_tag_id = (desc_hdr->src_dst_tag &
			      KNAV_UDMAP_INFO3_DESC_SRCTAG_MASK) >>
			      KNAV_UDMAP_INFO3_DESC_SRCTAG_SHIFT;
	if (dst_tag_id)
		*dst_tag_id = desc_hdr->src_dst_tag &
			      KNAV_UDMAP_INFO3_DESC_DSTTAG_MASK;
}

/**
 * knav_udmap_hdesc_set_pktids - set Packet Src/Dst Tags in HDesc
 * @desc_hdr: packet/TR descriptor header
 * @src_tag_id: Source Tag
 * @dst_tag_id: Dest Tag
 *
 * Returns Packet Src/Dst Tags from packet/TR descriptor
 */
static inline void
knav_udmap_desc_set_tags_ids(struct knav_udmap_desc_hdr_t *desc_hdr,
			     u32 src_tag_id, u32 dst_tag_id)
{
	WARN_ON(!desc_hdr);

	desc_hdr->src_dst_tag =
			(src_tag_id << KNAV_UDMAP_INFO3_DESC_SRCTAG_SHIFT) &
			KNAV_UDMAP_INFO3_DESC_SRCTAG_MASK;
	desc_hdr->src_dst_tag |= dst_tag_id & KNAV_UDMAP_INFO3_DESC_DSTTAG_MASK;
}

/**
 * knav_udmap_hdesc_attach_buf - attach buffer to HDesc
 * @desc: Host packet descriptor
 * @buf: Buffer physical address
 * @buf_data_len: Buffer length
 * @obuf: Original Buffer physical address
 * @obuf_len: Original Buffer length
 *
 * Attaches buffer to Host Packet Descriptor
 */
static inline void
knav_udmap_hdesc_attach_buf(struct knav_udmap_host_desc_t *desc,
			    dma_addr_t buf, u32 buf_data_len,
			    dma_addr_t obuf, u32 obuf_len)
{
	WARN_ON(!desc);
	WARN_ON(!buf && !obuf);

	desc->buf_ptr = buf;
	desc->buf_info1 = buf_data_len &
			  KNAV_UDMAP_BUFINFO1_HDESC_DATA_LEN_MASK;
	desc->org_buf_ptr = obuf;
	desc->org_buf_len = obuf_len & KNAV_UDMAP_OBUFINFO0_HDESC_BUF_LEN_MASK;
}

static inline void
knav_udmap_hdesc_get_obuf(struct knav_udmap_host_desc_t *desc,
			  dma_addr_t *obuf, u32 *obuf_len)
{
	WARN_ON(!desc);
	WARN_ON(!obuf);
	WARN_ON(!obuf_len);

	*obuf = desc->org_buf_ptr;
	*obuf_len = desc->org_buf_len & KNAV_UDMAP_OBUFINFO0_HDESC_BUF_LEN_MASK;
}

static inline void
knav_udmap_hdesc_reset_to_original(struct knav_udmap_host_desc_t *desc)
{
	WARN_ON(!desc);

	desc->buf_ptr = desc->org_buf_ptr;
	desc->buf_info1 = desc->org_buf_len;
}

/**
 * knav_udmap_hdesc_link_hbdesc - link Host Buffer Descriptor to HDesc
 * @desc: Host Packet Descriptor
 * @buf_desc: Host Buffer Descriptor physical address
 *
 * add and link Host Buffer Descriptor to HDesc
 */
static inline void
knav_udmap_hdesc_link_hbdesc(struct knav_udmap_host_desc_t *desc,
			     dma_addr_t hbuf_desc)
{
	WARN_ON(!desc);
	WARN_ON(!hbuf_desc);

	desc->next_desc = hbuf_desc;
}

static inline dma_addr_t
knav_udmap_hdesc_get_next_hbdesc(struct knav_udmap_host_desc_t *desc)
{
	WARN_ON(!desc);

	return (dma_addr_t)desc->next_desc;
}

static inline void
knav_udmap_hdesc_reset_hbdesc(struct knav_udmap_host_desc_t *desc)
{
	WARN_ON(!desc);

	desc->hdr = (struct knav_udmap_desc_hdr_t) { 0 };
	desc->next_desc = 0;
}

/**
 * knav_udmap_desc_is_epib_present -  check if EPIB present
 * @desc_hdr: packet descriptor/TR header
 *
 * Returns true if EPIB present in the packet
 */
static inline bool
knav_udmap_desc_is_epib_present(struct knav_udmap_desc_hdr_t *desc_hdr)
{
	WARN_ON(!desc_hdr);
	return !!(desc_hdr->pkt_info0 & KNAV_UDMAP_INFO0_HDESC_EPIB_PRESENT);
}

/**
 * knav_udmap_hdesc_get_psdata -  Get pointer on PSDATA
 * @desc: Host packet descriptor
 *
 * Returns pointer on PSDATA in HDesc.
 * NULL - if ps_data placed at the start of data buffer.
 */
static inline void *knav_udmap_hdesc_get_psdata(
		struct knav_udmap_host_desc_t *desc)
{
	u32 psdata_size;
	void *psdata;

	WARN_ON(!desc);

	if (desc->hdr.pkt_info0 & KNAV_UDMAP_INFO0_HDESC_PSINFO_LOCATION)
		return NULL;

	psdata_size = (desc->hdr.pkt_info0 &
		       KNAV_UDMAP_INFO0_HDESC_PSINFO_SIZE_MASK) >>
		       KNAV_UDMAP_INFO0_HDESC_PSINFO_SIZE_SHIFT;

	if (!psdata_size)
		return NULL;

	psdata = &desc->epib;

	if (knav_udmap_desc_is_epib_present(&desc->hdr))
		psdata += KNAV_UDMAP_INFO0_HDESC_EPIB_SIZE;

	return psdata;
}

static inline u32 *knav_udmap_hdesc_get_psdata32(
		struct knav_udmap_host_desc_t *desc)
{
	return (u32 *)knav_udmap_hdesc_get_psdata(desc);
}

/**
 * knav_udmap_hdesc_get_swdata -  Get pointer on swdata
 * @desc: Host packet descriptor
 *
 * Returns pointer on SWDATA in HDesc.
 * NOTE. It's caller responsibility to be sure hdesc actually has swdata.
 */
static inline void *knav_udmap_hdesc_get_swdata(
		struct knav_udmap_host_desc_t *desc)
{
	u32 psdata_size = 0;
	void *swdata;

	WARN_ON(!desc);

	if (!(desc->hdr.pkt_info0 & KNAV_UDMAP_INFO0_HDESC_PSINFO_LOCATION))
		psdata_size = (desc->hdr.pkt_info0 &
			       KNAV_UDMAP_INFO0_HDESC_PSINFO_SIZE_MASK) >>
			       KNAV_UDMAP_INFO0_HDESC_PSINFO_SIZE_SHIFT;

	swdata = &desc->epib;

	if (knav_udmap_desc_is_epib_present(&desc->hdr))
		swdata += KNAV_UDMAP_INFO0_HDESC_EPIB_SIZE;

	swdata += (psdata_size << 2);

	return swdata;
}

/* ================================== TR ================================== */

/**
 * knav_udmap_tr_calc_size - Calculate TR Descriptor size
 * @tr_num: number of TR records
 * @tr_size: Nominal size of TR record (max) [16, 32, 64, 128]
 *
 * Returns required TR Descriptor size
 */
static inline u32 knav_udmap_tr_calc_size(u32 tr_num, u32 tr_size)
{
	return sizeof(struct knav_udmap_tr_desc_t) + tr_num * tr_size;
}

/**
 * knav_udmap_tr_desc_init - Init TR Descriptor
 * @desc: TR Descriptor
 * @flags: supported values
 *	KNAV_UDMAP_INFO0_TRDESC_RELOAD
 * @tr_num: number of TR records
 * @tr_size: Nominal size of TR record (max) [16, 32, 64, 128]
 * @reload_idx: Reload Index
 *
 * Init TR Descriptor
 */
static inline void knav_udmap_tr_desc_init(
		struct knav_udmap_tr_desc_t *desc,
		u32 flags, u32 tr_num, u32 tr_size, u32 reload_idx)
{
	WARN_ON(!desc);
	WARN_ON(flags & ~KNAV_UDMAP_INFO0_TRDESC_RELOAD);
	WARN_ON(tr_num & ~KNAV_UDMAP_INFO0_TRDESC_LASTIDX_MASK);

	desc->hdr.pkt_info0 = (KNAV_UDMAP_INFO0_DESC_TYPE_VAL_TR <<
			       KNAV_UDMAP_INFO0_HDESC_TYPE_SHIFT) | (flags);
	desc->hdr.pkt_info0 |=
			(reload_idx << KNAV_UDMAP_INFO0_TRDESC_RLDIDX_SHIFT) &
			KNAV_UDMAP_INFO0_TRDESC_RLDIDX_MASK;
	desc->hdr.pkt_info0 |= tr_num & KNAV_UDMAP_INFO0_TRDESC_LASTIDX_MASK;

	desc->hdr.pkt_info1 |= ((ffs(tr_size >> 4) - 1) <<
				KNAV_UDMAP_INFO1_TRDESC_RECSIZE_SHIFT) &
				KNAV_UDMAP_INFO1_TRDESC_RECSIZE_MASK;
}

/* ================================== TRs definitions====================== */

#define KNAV_UDMAP_TR_FLAGS_TYPE_SHIFT			(0U)
#define KNAV_UDMAP_TR_FLAGS_TYPE_MASK			GENMASK(3, 0)
#define KNAV_UDMAP_TR_FLAGS_STATIC			BIT(4)
#define KNAV_UDMAP_TR_FLAGS_EOL				BIT(5)
#define KNAV_UDMAP_TR_FLAGS_EVENT_SIZE_SHIFT		(6U)
#define KNAV_UDMAP_TR_FLAGS_EVENT_SIZE_MASK		GENMASK(7, 6)
#define KNAV_UDMAP_TR_FLAGS_TRIGGER0_SHIFT		(8U)
#define KNAV_UDMAP_TR_FLAGS_TRIGGER0_MASK		GENMASK(9, 8)
#define KNAV_UDMAP_TR_FLAGS_TRIGGER0_TYPE_SHIFT		(10U)
#define KNAV_UDMAP_TR_FLAGS_TRIGGER0_TYPE_MASK		GENMASK(11, 10)
#define KNAV_UDMAP_TR_FLAGS_TRIGGER1_SHIFT		(12U)
#define KNAV_UDMAP_TR_FLAGS_TRIGGER1_MASK		GENMASK(13, 12)
#define KNAV_UDMAP_TR_FLAGS_TRIGGER1_TYPE_SHIFT		(14U)
#define KNAV_UDMAP_TR_FLAGS_TRIGGER1_TYPE_MASK		GENMASK(15, 14)
#define KNAV_UDMAP_TR_FLAGS_CMD_ID_SHIFT		(16U)
#define KNAV_UDMAP_TR_FLAGS_CMD_ID_MASK			GENMASK(23, 16)
#define KNAV_UDMAP_TR_FLAGS_CFG_FLAGS_SHIFT		(24U)
#define KNAV_UDMAP_TR_FLAGS_CFG_FLAGS_MASK		GENMASK(31, 24)
#define   KNAV_UDMAP_TR_FLAGS_CFG_SA_INDIRECT_SHIFT	BIT(0)
#define   KNAV_UDMAP_TR_FLAGS_CFG_DA_INDIRECT_SHIFT	BIT(1)
#define   KNAV_UDMAP_TR_FLAGS_CFG_EOL_ADV_SHIFT		(4U)
#define   KNAV_UDMAP_TR_FLAGS_CFG_EOL_ADV_MASK		GENMASK(6, 4)
#define   KNAV_UDMAP_TR_FLAGS_CFG_EOP_SHIFT		BIT(7)

/* Udmap TR flags Type field specifies the type of TR. */
enum knav_udmap_tr_types {
	KNAV_UDMAP_TR_FLAGS_TYPE_1D_DATA_MOVE = 0,
	KNAV_UDMAP_TR_FLAGS_TYPE_2D_DATA_MOVE,
	KNAV_UDMAP_TR_FLAGS_TYPE_3D_DATA_MOVE,
	KNAV_UDMAP_TR_FLAGS_TYPE_4D_DATA_MOVE,
	KNAV_UDMAP_TR_FLAGS_TYPE_4D_DATA_MOVE_FORMATTING,
	KNAV_UDMAP_TR_FLAGS_TYPE_4D_CACHE_WARM,
	KNAV_UDMAP_TR_FLAGS_TYPE_4D_BLK_MOVE = 8,
	KNAV_UDMAP_TR_FLAGS_TYPE_4D_BLK_MOVE_REPACKING,
	KNAV_UDMAP_TR_FLAGS_TYPE_2D_BLK_MOVE,
	KNAV_UDMAP_TR_FLAGS_TYPE_2D_BLK_MOVE_REPACKING,
	KNAV_UDMAP_TR_FLAGS_TYPE_4D_BLK_MOVE_REPACKING_IA = 15,
	KNAV_UDMAP_TR_FLAGS_TYPE_MAX
};

/*
 * Udmap TR Flags EVENT_SIZE field specifies when an event is generated
 * for each TR.
 */
enum knav_udmap_tr_event_size {
	/* When TR is complete and all status for the TR has been received */
	KNAV_UDMAP_TR_FLAGS_EVENT_SIZE_COMPLETION,
	/*
	 * Type 0: when the last data transaction is sent for the TR;
	 * Type 1-11: when ICNT1 is decremented
	 */
	KNAV_UDMAP_TR_FLAGS_EVENT_SIZE_ICNT1_DEC,
	/*
	 * Type 0-1,10-11: when the last data transaction is sent for the TR;
	 * All other types: when ICNT2 is decremented
	 */
	KNAV_UDMAP_TR_FLAGS_EVENT_SIZE_ICNT2_DEC,
	/*
	 * Type 0-2,10-11: when the last data transaction is sent for the TR;
	 * All other types: when ICNT3 is decremented
	 */
	KNAV_UDMAP_TR_FLAGS_EVENT_SIZE_ICNT3_DEC,
	KNAV_UDMAP_TR_FLAGS_EVENT_SIZE_MAX
};

/*
 * Udmap TR Flags TRIGGERx field specifies the type of trigger used to
 * enable the TR to transfer data as specified by TRIGGERx_TYPE field.
 */
enum knav_udmap_tr_trigger {
	KNAV_UDMAP_TR_FLAGS_TRIGGER_NONE,		/* No Trigger */
	KNAV_UDMAP_TR_FLAGS_TRIGGER_GLOBAL0,		/* Global Trigger 0 */
	KNAV_UDMAP_TR_FLAGS_TRIGGER_GLOBAL1,		/* Global Trigger 1 */
	KNAV_UDMAP_TR_FLAGS_TRIGGER_LOCAL_EVENT,	/* Local Event */
	KNAV_UDMAP_TR_FLAGS_TRIGGER_MAX
};

/*
 * Udmap TR Flags TRIGGERx_TYPE field specifies the type of data transfer
 * that will be enabled by receiving a trigger as specified by TRIGGERx.
 */
enum knav_udmap_tr_trigger_type {
	/* The second inner most loop (ICNT1) will be decremented by 1 */
	KNAV_UDMAP_TR_FLAGS_TRIGGER_TYPE_ICNT1_DEC,
	/* The third inner most loop (ICNT2) will be decremented by 1 */
	KNAV_UDMAP_TR_FLAGS_TRIGGER_TYPE_ICNT2_DEC,
	/* The outer most loop (ICNT3) will be decremented by 1 */
	KNAV_UDMAP_TR_FLAGS_TRIGGER_TYPE_ICNT3_DEC,
	/* The entire TR will be allowed to complete */
	KNAV_UDMAP_TR_FLAGS_TRIGGER_TYPE_ALL,
	KNAV_UDMAP_TR_FLAGS_TRIGGER_TYPE_MAX
};

struct knav_udmap_tr_hdr_t {
	u32 flags;
} __packed;

/*
 * Type 0 (One dimensional data move) TR (16 byte).
 */
struct knav_udmap_tr0_t {
	struct knav_udmap_tr_hdr_t hdr;
	u32 icnt0;
	u64 addr;
} __aligned(16) __packed;

/*
 * Type 1 (Two dimensional data move) TR (32 byte).
 */
struct knav_udmap_tr1_t {
	struct knav_udmap_tr_hdr_t hdr;
	u16 icnt0;
	u16 icnt1;
	u64 addr;
	s32  dim1;
} __aligned(32) __packed;

/*
 * Type 2 (Three dimensional data move) TR (32 byte).
 */
struct knav_udmap_tr2_t {
	struct knav_udmap_tr_hdr_t hdr;
	u16 icnt0;
	u16 icnt1;
	u64 addr;
	s32  dim1;
	u32 icnt2;
	s32  dim2;
} __aligned(32) __packed;

/*
 * Type 3 (Four dimensional data move) TR (32 byte).
 */
struct knav_udmap_tr3_t {
	struct knav_udmap_tr_hdr_t hdr;
	u16 icnt0;
	u16 icnt1;
	u64 addr;
	s32  dim1;
	u16 icnt2;
	u16 icnt3;
	s32  dim2;
	s32  dim3;
} __aligned(32) __packed;

/*
 * Type 4 (Four dimensional data move with data formatting) TR (64 byte).
 */
struct knav_udmap_tr4_t {
	struct knav_udmap_tr_hdr_t hdr;
	u16 icnt0;
	u16 icnt1;
	u64 addr;
	s32  dim1;
	u16 icnt2;
	u16 icnt3;
	s32  dim2;
	s32  dim3;
	u32 fmtflags;
} __aligned(64) __packed;

/*
 * Type 5 (Four dimensional cache warm) TR (64 byte).
 */
struct knav_udmap_tr5_t {
	struct knav_udmap_tr_hdr_t hdr;
	u16 icnt0;
	u16 icnt1;
	u64 addr;
	s32  dim1;
	u16 icnt2;
	u16 icnt3;
	s32  dim2;
	s32  dim3;
	u32 cacheflags;
} __aligned(64) __packed;

/*
 * Type 8 (Four Dimensional Block Copy) TR (64 byte).
 */
struct knav_udmap_tr8_t {
	struct knav_udmap_tr_hdr_t hdr;
	u16 icnt0;
	u16 icnt1;
	u64 addr;
	s32  dim1;
	u16 icnt2;
	u16 icnt3;
	s32  dim2;
	s32  dim3;
	u32 fmtflags;
	s32  ddim1;
	u64 daddr;
	s32  ddim2;
	s32  ddim3;
} __aligned(64) __packed;

/*
 * Type 9 (Four Dimensional Block Copy with Repacking) TR (64 byte).
 */
struct knav_udmap_tr9_t {
	struct knav_udmap_tr_hdr_t hdr;
	u16 icnt0;
	u16 icnt1;
	u64 addr;
	s32  dim1;
	u16 icnt2;
	u16 icnt3;
	s32  dim2;
	s32  dim3;
	u32 fmtflags;
	s32  ddim1;
	u64 daddr;
	s32  ddim2;
	s32  ddim3;
	u16 dicnt0;
	u16 dicnt1;
	u16 dicnt2;
	u16 dicnt3;
} __aligned(64) __packed;

/*
 * Type 10 (Two Dimensional Block Copy) TR (64 byte).
 */
struct knav_udmap_tr10_t {
	struct knav_udmap_tr_hdr_t hdr;
	u16 icnt0;
	u16 icnt1;
	u64 addr;
	s32  dim1;
	u32 rsvd[3];
	u32 fmtflags;
	s32  ddim1;
	u64 daddr;
} __aligned(64) __packed;

/*
 * Type 11 (Two Dimensional Block Copy with Repacking) TR (64 byte).
 */
struct knav_udmap_tr11_t {
	struct knav_udmap_tr_hdr_t hdr;
	u16 icnt0;
	u16 icnt1;
	u64 addr;
	s32  dim1;
	u32 rsvd0[3];
	u32 fmtflags;
	s32  ddim1;
	u64 daddr;
	u32 rsvd1[2];
	u16 dicnt0;
	u16 dicnt1;
} __aligned(64) __packed;

/*
 * Type 15 (Four Dimensional Block Copy with Repacking and
 * Indirection Support) TR (64 byte).
 */
struct knav_udmap_tr15_t {
	struct knav_udmap_tr_hdr_t hdr;
	u16 icnt0;
	u16 icnt1;
	u64 addr;
	s32  dim1;
	u16 icnt2;
	u16 icnt3;
	s32  dim2;
	s32  dim3;
	u32 fmtflags;
	s32  ddim1;
	u64 daddr;
	s32  ddim2;
	s32  ddim3;
	u16 dicnt0;
	u16 dicnt1;
	u16 dicnt2;
	u16 dicnt3;
} __aligned(64) __packed;

struct knav_udmap_tr_resp {
	u8 status;
	u8 reserved;
	u8 cmd_id;
	u8 flags;
} __packed;

#define KNAV_UDMAP_TR_RESPONSE_STATUS_TYPE_SHIFT	(0U)
#define KNAV_UDMAP_TR_RESPONSE_STATUS_TYPE_MASK		GENMASK(3, 0)
#define KNAV_UDMAP_TR_RESPONSE_STATUS_INFO_SHIFT	(4U)
#define KNAV_UDMAP_TR_RESPONSE_STATUS_INFO_MASK		GENMASK(7, 4)
#define KNAV_UDMAP_TR_RESPONSE_CMDID_SHIFT		(16U)
#define KNAV_UDMAP_TR_RESPONSE_CMDID_MASK		GENMASK(23, 16)
#define KNAV_UDMAP_TR_RESPONSE_CFG_SPECIFIC_SHIFT	(24U)
#define KNAV_UDMAP_TR_RESPONSE_CFG_SPECIFIC_MASK	GENMASK(31, 24)

/*
 * Udmap TR Response Status Type field is used to determine
 * what type of status is being returned.
 */
enum knav_udmap_tr_resp_status_type {
	KNAV_UDMAP_TR_RESPONSE_STATUS_COMPLETE,		/* None */
	KNAV_UDMAP_TR_RESPONSE_STATUS_TRANSFER_ERR,	/* Transfer Error */
	KNAV_UDMAP_TR_RESPONSE_STATUS_ABORTED_ERR,	/* Aborted Error */
	KNAV_UDMAP_TR_RESPONSE_STATUS_SUBMISSION_ERR,	/* Submission Error */
	KNAV_UDMAP_TR_RESPONSE_STATUS_UNSUPPORTED_ERR,	/* Unsup. Feature */
	KNAV_UDMAP_TR_RESPONSE_STATUS_MAX
};

/*
 * Udmap TR Response Status field values which corresponds
 * KNAV_UDMAP_TR_RESPONSE_STATUS_SUBMISSION_ERR
 */
enum knav_udmap_tr_resp_status_submission {
	/* ICNT0 was 0 */
	KNAV_UDMAP_TR_RESPONSE_STATUS_SUBMISSION_ICNT0,
	/* Channel FIFO was full when TR received */
	KNAV_UDMAP_TR_RESPONSE_STATUS_SUBMISSION_FIFO_FULL,
	/* Channel is not owned by the submitter */
	KNAV_UDMAP_TR_RESPONSE_STATUS_SUBMISSION_OWN,
	KNAV_UDMAP_TR_RESPONSE_STATUS_SUBMISSION_MAX
};

/*
 * Udmap TR Response Status field values which corresponds
 * KNAV_UDMAP_TR_RESPONSE_STATUS_UNSUPPORTED_ERR
 */
enum knav_udmap_tr_resp_status_unsupported {
	/* TR Type not supported */
	KNAV_UDMAP_TR_RESPONSE_STATUS_UNSUPPORTED_TR_TYPE,
	/* STATIC not supported */
	KNAV_UDMAP_TR_RESPONSE_STATUS_UNSUPPORTED_STATIC,
	/* EOL not supported */
	KNAV_UDMAP_TR_RESPONSE_STATUS_UNSUPPORTED_EOL,
	/* CONFIGURATION SPECIFIC not supported */
	KNAV_UDMAP_TR_RESPONSE_STATUS_UNSUPPORTED_CFG_SPECIFIC,
	/* AMODE not supported */
	KNAV_UDMAP_TR_RESPONSE_STATUS_UNSUPPORTED_AMODE,
	/* ELTYPE not supported */
	KNAV_UDMAP_TR_RESPONSE_STATUS_UNSUPPORTED_ELTYPE,
	/* DFMT not supported */
	KNAV_UDMAP_TR_RESPONSE_STATUS_UNSUPPORTED_DFMT,
	/* SECTR not supported */
	KNAV_UDMAP_TR_RESPONSE_STATUS_UNSUPPORTED_SECTR,
	/* AMODE SPECIFIC field not supported */
	KNAV_UDMAP_TR_RESPONSE_STATUS_UNSUPPORTED_AMODE_SPECIFIC,
	KNAV_UDMAP_TR_RESPONSE_STATUS_UNSUPPORTED_MAX
};

/**
 * knav_udmap_tr_init - Init TR record
 * @hdr: TR header (flags)
 * @type: TR type
 * @event: output event generation cfg
 * @cmd_id: TR identifier (application specifics)
 * @static_tr: TR is static
 * @cfg_flags: Configuration Specific Flags
 *
 * Init TR record
 */
static inline void knav_udmap_tr_init(struct knav_udmap_tr_hdr_t *hdr,
				      enum knav_udmap_tr_types type,
				      enum knav_udmap_tr_event_size event,
				      u32 cmd_id, bool static_tr, u32 cfg_flags)
{
	WARN_ON(!hdr);

	hdr->flags = type;
	hdr->flags |= (event << KNAV_UDMAP_TR_FLAGS_EVENT_SIZE_SHIFT) &
		      KNAV_UDMAP_TR_FLAGS_EVENT_SIZE_MASK;

	hdr->flags |= (cmd_id << KNAV_UDMAP_TR_FLAGS_CMD_ID_SHIFT) &
		      KNAV_UDMAP_TR_FLAGS_CMD_ID_MASK;

	hdr->flags |= (cfg_flags << KNAV_UDMAP_TR_FLAGS_CFG_FLAGS_SHIFT) &
			KNAV_UDMAP_TR_FLAGS_CFG_FLAGS_MASK;

	if (static_tr)
		hdr->flags |= KNAV_UDMAP_TR_FLAGS_STATIC;
}

/* ================================== Descs pool ====================== */
struct k3_knav_desc_pool;

void k3_knav_pool_destroy(struct k3_knav_desc_pool *pool);
struct k3_knav_desc_pool *k3_knav_pool_create_name(struct device *dev,
						   size_t size,
						   size_t desc_size,
						   const char *name);
#define k3_knav_pool_create(dev, size, desc_size) \
		k3_knav_pool_create_name(dev, size, desc_size, NULL)
dma_addr_t k3_knav_pool_virt2dma(struct k3_knav_desc_pool *pool, void *addr);
void *k3_knav_pool_dma2virt(struct k3_knav_desc_pool *pool, dma_addr_t dma);
void *k3_knav_pool_alloc(struct k3_knav_desc_pool *pool);
void k3_knav_pool_free(struct k3_knav_desc_pool *pool, void *addr);
size_t k3_knav_pool_avail(struct k3_knav_desc_pool *pool);

#endif /* __CPPI5_H__ */
