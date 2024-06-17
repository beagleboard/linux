/* SPDX-License-Identifier: GPL-2.0 */
/*
 * VXD Decoder Component header
 *
 * Copyright (c) Imagination Technologies Ltd.
 * Copyright (c) 2021 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Authors:
 *	Sunita Nadampalli <sunitan@ti.com>
 *
 * Re-written for upstream
 *      Sidraya Jayagond <sidraya.bj@pathpartnertech.com>
 *      Prashanth Kumar Amai <prashanth.ka@pathpartnertech.com>
 */

#ifndef __DECODER_H__
#define __DECODER_H__

#include "bspp.h"
#include "dq.h"
#ifdef HAS_JPEG
#include "jpegfw_data.h"
#endif
#include "lst.h"
#include "vdecdd_defs.h"
#include "vdec_defs.h"
#include "vid_buf.h"
#include "vxd_ext.h"
#include "vxd_props.h"
#include "hevcfw_data.h"

#define MAX_CONCURRENT_STREAMS 16
#define CORE_NUM_DECODE_SLOTS 2

enum dec_pict_states {
	DECODER_PICTURE_STATE_TO_DECODE = 0,
	DECODER_PICTURE_STATE_DECODED,
	DECODER_PICTURE_STATE_TO_DISCARD,
	DECODER_PICTURE_STATE_MAX,
	DECODER_PICTURE_FORCE32BITS     = 0x7FFFFFFFU
};

enum dec_res_type {
	DECODER_RESTYPE_TRANSACTION = 0,
	DECODER_RESTYPE_HDR,
	DECODER_RESTYPE_BATCH_MSG,
#ifdef HAS_HEVC
	DECODER_RESTYPE_PVDEC_BUF,
#endif
	DECODER_RESTYPE_MAX,
	DECODER_RESTYPE_FORCE32BITS = 0x7FFFFFFFU
};

enum dec_core_query_type {
	DECODER_CORE_GET_RES_LIMIT = 0,
	DECODER_CORE_FORCE32BITS   = 0x7FFFFFFFU
};

/*
 * @Function              pfnRefPicGetMaxNum
 * @Description
 * This is the prototype for functions calculating the maximum number
 * of reference pictures required per video standard.
 *
 * @Input    psComSequHdrInfo  : A pointer to the common VSH information
 * structure.
 *
 * @Output   pui32MaxRefPicNum :  A pointer used to return the maximum number
 *                               of reference frames required.
 *
 * @Return   IMG_RESULT : This function returns either IMG_SUCCESS or
 * an error code.
 */
typedef int (*ref_pic_get_maximum)(const struct vdec_comsequ_hdrinfo *comseq_hdr_info,
				   unsigned int *max_ref_pict_num);

typedef int (*strunit_processed_cb)(void *handle, int cb_type, void *item);

typedef int (*core_gen_cb)(void *handle, int query, void *item);

struct dec_ctx;

/*
 * This structure contains the core context.
 * @brief  Decoder Core Context
 */
struct dec_core_ctx {
	void **link;            /* to be part of single linked list */
	struct dec_ctx *dec_ctx;
	unsigned char enumerated;
	unsigned char master;
	unsigned char configured;
	unsigned int core_features;
	unsigned int pipe_features[VDEC_MAX_PIXEL_PIPES];
	struct vxd_coreprops core_props;
	void *resources;
	void *hw_ctx;
	unsigned int cum_pics;
	unsigned char busy;
};

struct dec_ctx {
	unsigned char inited;
	void *user_data;
	const struct vdecdd_dd_devconfig *dev_cfg;
	unsigned int num_pipes;
	struct dec_core_ctx *dec_core_ctx;
	struct lst_t str_list;
	void *mmu_dev_handle;
	void *dev_handle;
	struct vidio_ddbufinfo ptd_buf_info;
	unsigned char sup_stds[VDEC_STD_MAX];
	unsigned int internal_heap_id;
	unsigned int str_cnt;
};

/*
 * This structure contains the device decode resource (used for decoding and
 * held for subsequent decoding).
 * @brief  Decoder Device Resource
 */
struct dec_pictdec_res {
	void **link;    /* to be part of single linked list */
	unsigned int transaction_id;
	struct vidio_ddbufinfo fw_ctx_buf;
	struct vidio_ddbufinfo h264_sgm_buf;
	unsigned int ref_cnt;
};

struct dec_decpict;

/*
 *
 * This structure contains the stream context.
 * @brief  Decoder Stream Context
 */
struct dec_str_ctx {
	void **link;    /* to be part of single linked list */
	int km_str_id;
	struct vdec_str_configdata config;
	struct dec_ctx *decctx;
	void *vxd_dec_ctx;
	void *usr_int_data;
	void *mmu_str_handle;
	void *pict_idgen;
	struct lst_t pend_strunit_list;
	struct dq_linkage_t str_decd_pict_list;
	unsigned int num_ref_res;
	struct lst_t ref_res_lst;
	unsigned int num_dec_res;
	struct lst_t dec_res_lst;
	unsigned int avail_pipes;
	unsigned int avail_slots;
	struct vdecdd_decstr_status dec_str_st;
	struct vidio_ddbufinfo pvdec_fw_ctx_buf;
	unsigned int last_fe_transaction_id;
	unsigned int next_dec_pict_id;
	unsigned int next_pict_id_expected;
	struct dec_pictdec_res *cur_fe_pict_dec_res;
	struct dec_pictdec_res *prev_fe_pict_dec_res;
	struct dec_pictdec_res *last_be_pict_dec_res;
	struct dec_decpict *cur_pict;
	void *resources;
	strunit_processed_cb str_processed_cb;
	core_gen_cb core_query_cb;
};

/*
 * Resource Structure for DECODER_sDdResourceInfo to be used with pools
 */
struct res_resinfo {
	void **link;                    /* to be part of single linked list */
	void *res;
	struct vidio_ddbufinfo *ddbuf_info;
};

struct vdecdd_ddstr_ctx;

/*
 * This structure contains the Decoded attributes
 * @brief Decoded attributes
 */
struct dec_pict_attrs {
	unsigned char first_fld_rcvd;
	unsigned int fe_err;
	unsigned int no_be_wdt;
	unsigned int mbs_dropped;
	unsigned int mbs_recovered;
	struct vxd_pict_attrs pict_attrs;
};

/*
 * This union contains firmware contexts. Used to allocate buffers for firmware
 * context.
 */
union dec_fw_contexts {
	struct h264fw_context_data h264_context;
#ifdef HAS_JPEG
	struct jpegfw_context_data jpeg_context;
#endif
#ifdef HAS_HEVC
	struct hevcfw_ctx_data hevc_context;
#endif
};

/*
 * for debug
 */
struct dec_fwmsg {
	void **link;
	struct dec_pict_attrs pict_attrs;
	struct vdec_pict_hwcrc pict_hwcrc;
};

/*
 * This structure contains the stream decode resource (persistent for
 * longer than decoding).
 * @brief  Decoder Stream Resource
 */
struct dec_pictref_res {
	void **link;                    /* to be part of single linked list */
	struct vidio_ddbufinfo fw_ctrlbuf;
	unsigned int ref_cnt;
};

/*
 * This structure defines the decode picture.
 * @brief  Decoder Picture
 */
struct dec_decpict {
	void **link;
	unsigned int transaction_id;
	void *dec_str_ctx;
	unsigned char twopass;
	unsigned char first_fld_rcvd;
	struct res_resinfo *transaction_info;
	struct res_resinfo *hdr_info;
#ifdef HAS_HEVC
	struct res_resinfo *pvdec_info;
	unsigned int temporal_out_addr;
#endif
	struct vdecdd_ddpict_buf *recon_pict;
	struct vdecdd_ddpict_buf *alt_pict;
	struct res_resinfo *batch_msginfo;
	struct vidio_ddbufinfo *intra_bufinfo;
	struct vidio_ddbufinfo *auxline_bufinfo;
	struct vidio_ddbufinfo *vlc_tables_bufinfo;
	struct vidio_ddbufinfo *vlc_idx_tables_bufinfo;
	struct vidio_ddbufinfo *start_code_bufinfo;
	struct dec_fwmsg *first_fld_fwmsg;
	struct dec_fwmsg *second_fld_fwmsg;
	struct bspp_pict_hdr_info *pict_hdr_info;
	struct dec_pictdec_res *cur_pict_dec_res;
	struct dec_pictdec_res *prev_pict_dec_res;
	struct dec_pictref_res *pict_ref_res;
	struct lst_t dec_pict_seg_list;
	struct lst_t fragment_list;
	unsigned char eop_found;
	unsigned int operating_op;
	unsigned short genc_id;
	struct vdecdd_ddbuf_mapinfo **genc_bufs;
	struct vdecdd_ddbuf_mapinfo *genc_fragment_buf;
	unsigned int ctrl_alloc_bytes;
	unsigned int ctrl_alloc_offset;
	enum dec_pict_states state;
	struct vidio_ddbufinfo *str_pvdec_fw_ctxbuf;
};

/*
 *
 * This structure defines the decode picture reference.
 * @brief  Decoder Picture Reference
 */
struct dec_str_unit {
	void **link;                    /* to be part of single linked list */
	struct dec_decpict *dec_pict;
	struct vdecdd_str_unit *str_unit;
};

/*
 * This structure defines the decoded picture.
 * @brief  Decoded Picture
 */
struct dec_decoded_pict {
	struct dq_linkage_t link;     /* to be part of double linked list */
	unsigned int transaction_id;
	unsigned char processed;
	unsigned char process_failed;
	unsigned char force_display;
	unsigned char displayed;
	unsigned char merged;
	unsigned int disp_idx;
	unsigned int rel_idx;
	struct vdecdd_picture *pict;
	struct dec_fwmsg *first_fld_fwmsg;
	struct dec_fwmsg *second_fld_fwmsg;
	struct dec_pictref_res *pict_ref_res;
};

struct dec_pict_fragment {
	void **link;    /* to be part of single linked list */
	/* Control allocation size in bytes */
	unsigned int ctrl_alloc_bytes;
	/* Control allocation offset in bytes */
	unsigned int ctrl_alloc_offset;
};

/*
 * This structure contains the pointer to the picture segment.
 * All the segments could be added to the list in struct dec_decpict,
 * but because list items cannot belong to more than one list this wrapper
 * is used which is added in the list sDecPictSegList inside struct dec_decpict
 * @brief  Decoder Picture Segment
 */
struct dec_decpict_seg {
	void **link;                    /* to be part of single linked list */
	struct bspp_bitstr_seg *bstr_seg;
	unsigned char internal_seg;
};

struct decoder_regsoffsets {
	unsigned int vdmc_cmd_offset;
	unsigned int vec_offset;
	unsigned int entropy_offset;
	unsigned int vec_be_regs_offset;
	unsigned int vdec_be_codec_regs_offset;
};

int decoder_initialise(void *init_usr_data, unsigned int internal_heap_id,
		       struct vdecdd_dd_devconfig *dd_devcfg, unsigned int *num_pipes,
		       void **dec_ctx);

int decoder_deinitialise(void *dec_ctx);

int decoder_supported_features(void *dec_ctx, struct vdec_features *features);

int decoder_stream_destroy(void *dec_str_ctx, unsigned char abort);

int decoder_stream_create(void *dec_ctx, struct vdec_str_configdata str_cfg,
			  unsigned int kmstr_id, void **mmu_str_handle,
			  void *vxd_dec_ctx, void *str_usr_int_data,
			  void **dec_str_ctx, void *decoder_cb, void *query_cb);

int decoder_stream_prepare_ctx(void *dec_str_ctx, unsigned char flush_dpb);

int decoder_stream_process_unit(void *dec_str_ctx,
				struct vdecdd_str_unit *str_unit);

int decoder_get_load(void *dec_str_ctx, unsigned int *avail_slots);

int
decoder_check_support(void *dec_ctx,
		      const struct vdec_str_configdata *str_cfg,
		      const struct vdec_str_opconfig *op_cfg,
		      const struct vdecdd_ddpict_buf *disp_pictbuf,
		      const struct vdec_pict_rendinfo *req_pict_rendinfo,
		      const struct vdec_comsequ_hdrinfo *comseq_hdrinfo,
		      const struct bspp_pict_hdr_info *pict_hdrinfo,
		      const struct vdec_comsequ_hdrinfo *prev_comseq_hdrinfo,
		      const struct bspp_pict_hdr_info *prev_pict_hdrinfo,
		      unsigned char non_cfg_req, struct vdec_unsupp_flags *unsupported,
		      unsigned int *features);

unsigned char decoder_is_stream_idle(void *dec_str_ctx);

int decoder_stream_flush(void *dec_str_ctx, unsigned char discard_refs);

int decoder_stream_release_buffers(void *dec_str_ctx);

int decoder_stream_get_status(void *dec_str_ctx,
			      struct vdecdd_decstr_status *dec_str_st);

int decoder_service_firmware_response(void *dec_str_ctx_arg, unsigned int *msg,
				      unsigned int msg_size, unsigned int msg_flags);

#endif
