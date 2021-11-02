/* SPDX-License-Identifier: GPL-2.0 */
/*
 * encoder header generation interface header
 *
 * Copyright (c) Imagination Technologies Ltd.
 * Copyright (c) 2021 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Authors:
 *	Sunita Nadampalli <sunitan@ti.com>
 *
 * Re-written for upstreming
 *	Sidraya Jayagond <sidraya.bj@pathpartnertech.com>
 */

#include "fw_headers/topazscfwif.h"
#include <linux/types.h>
#include "topaz_api.h"
#include "vid_buf.h"

/*
 * enum describing slice/frame type (H264)
 */
enum slhp_sliceframe_type {
	SLHP_P_SLICEFRAME_TYPE,
	SLHP_B_SLICEFRAME_TYPE,
	SLHP_I_SLICEFRAME_TYPE,
	SLHP_SP_SLICEFRAME_TYPE,
	SLHP_SI_SLICEFRAME_TYPE,
	SLHP_IDR_SLICEFRAME_TYPE,
	SLHP_SLICE_FRAME_TYPE_FORCE32BITS = 0x7FFFFFFFU
};

/*
 * struct describing scaling lists (H264)
 */
struct h264_scaling_matrix_params {
	unsigned char scaling_lists4x4[6][16];
	unsigned char scaling_lists8x8[2][64];
	unsigned int list_mask;
};

/*
 * struct describing picture parameter set (H264)
 */
struct h264_picture_header_params {
	unsigned char pic_parameter_set_id;
	unsigned char seq_parameter_set_id;
	unsigned char entropy_coding_mode_flag;
	unsigned char weighted_pred_flag;
	unsigned char weighted_bipred_idc;
	signed char chroma_qp_index_offset;
	unsigned char constrained_intra_pred_flag;
	unsigned char transform_8x8_mode_flag;
	unsigned char pic_scaling_matrix_present_flag;
	unsigned char use_default_scaling_list;
	signed char second_chroma_qp_index_offset;
};

/*
 * struct describing slice header (H264)
 */
struct h264_slice_header_params {
	unsigned char startcode_prefix_size_bytes;
	enum slhp_sliceframe_type slice_frame_type;
	unsigned int first_mb_address;
	unsigned char log2_max_pic_order_cnt;
	unsigned char disable_deblocking_filter_idc;
	unsigned char pic_interlace;
	unsigned char reference_picture;
	signed char deb_alpha_offset_div2;
	signed char deb_beta_offset_div2;
	unsigned short mvc_view_idx;
	unsigned char is_longterm_ref;
	unsigned char longterm_ref_num;
	/* Long term reference info for reference frames */
	unsigned char ref_is_longterm_ref[2];
	unsigned char ref_longterm_ref_num[2];
};

void generate_slice_params_template(struct img_enc_context *enc,
				    struct vidio_ddbufinfo *mem_info,
				    enum img_frame_template_type slice_type,
				    unsigned char is_interlaced, int fine_y_search_size);

void h264_prepare_sequence_header(struct mtx_header_params *mtx_header,
				  unsigned int pic_width_in_mbs,
				  unsigned int pic_height_in_mbs, unsigned char vui_params_present,
				  struct h264_vui_params *params,
				  struct h264_crop_params *crop,
				  struct h264_sequence_header_params *sh_params,
				  unsigned char aso);

void h264_prepare_mvc_sequence_header(struct mtx_header_params *mtx_header,
				      unsigned int pic_width_in_mbs, unsigned int pic_height_in_mbs,
				      unsigned char vui_params_present,
				      struct h264_vui_params *params,
				      struct h264_crop_params *crop,
				      struct h264_sequence_header_params *sh_params);

void h264_prepare_aud_header(struct mtx_header_params *mtx_header);

void h264_prepare_picture_header(struct mtx_header_params *mtx_header,
				 unsigned char cabac_enabled,
				 unsigned char transform_8x8,
				 unsigned char intra_constrained,
				 signed char cqp_offset,
				 unsigned char weighted_prediction,
				 unsigned char weighted_bi_pred,
				 unsigned char mvc_pps,
				 unsigned char scaling_matrix,
				 unsigned char scaling_lists);
