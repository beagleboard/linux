/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Public data structures for the hevc parser firmware module
 *
 * Copyright (c) Imagination Technologies Ltd.
 * Copyright (c) 2021 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Authors:
 *	Angela Stegmaier <angelabaker@ti.com>
 *
 * Re-written for upstreming
 *	Sidraya Jayagond <sidraya.bj@pathpartnertech.com>
 */
#ifdef USE_SHARING
#endif

#ifndef _HEVCFW_DATA_H_
#define _HEVCFW_DATA_H_

#include "vdecfw_share.h"
#include "vdecfw_shared.h"

#define HEVC_MAX_VPS_COUNT 16
#define HEVC_MAX_SPS_COUNT 16
#define HEVC_MAX_PPS_COUNT 64

#define HEVCFW_MAX_NUM_PROFILE_IDC 32
#define HEVCFW_MAX_VPS_OP_SETS_PLUS1 1024
#define HEVCFW_MAX_VPS_NUH_RESERVED_ZERO_LAYER_ID_PLUS1 1

#define HEVCFW_MAX_NUM_REF_PICS 16
#define HEVCFW_MAX_NUM_ST_REF_PIC_SETS 65
#define HEVCFW_MAX_NUM_LT_REF_PICS 32
#define HEVCFW_MAX_NUM_SUBLAYERS 7
#define HEVCFW_SCALING_LISTS_BUFSIZE 256
#define HEVCFW_MAX_TILE_COLS 20
#define HEVCFW_MAX_TILE_ROWS 22

#define HEVCFW_MAX_CHROMA_QP 6

#define HEVCFW_MAX_DPB_SIZE HEVCFW_MAX_NUM_REF_PICS
#define HEVCFW_REF_PIC_LIST0 0
#define HEVCFW_REF_PIC_LIST1 1
#define HEVCFW_NUM_REF_PIC_LISTS 2
#define HEVCFW_NUM_DPB_DIFF_REGS 4

/* non-critical errors*/
#define HEVC_ERR_INVALID_VALUE (20)
#define HEVC_ERR_CORRECTION_VALIDVALUE (21)

#define HEVC_IS_ERR_CRITICAL(err) \
	((err) > HEVC_ERR_CORRECTION_VALIDVALUE ? 1 : 0)

/* critical errors*/
#define HEVC_ERR_INV_VIDEO_DIMENSION (22)
#define HEVC_ERR_NO_SEQUENCE_HDR (23)
#define HEVC_ERR_SPS_EXT_UNSUPP (24 | VDECFW_UNSUPPORTED_CODE_BASE)
#define HEVC_ERR_PPS_EXT_UNSUPP (25 | VDECFW_UNSUPPORTED_CODE_BASE)

#define HEVC_ERR_FAILED_TO_STORE_VPS (100)
#define HEVC_ERR_FAILED_TO_STORE_SPS (101)
#define HEVC_ERR_FAILED_TO_STORE_PPS (102)

#define HEVC_ERR_FAILED_TO_FETCH_VPS (103)
#define HEVC_ERR_FAILED_TO_FETCH_SPS (104)
#define HEVC_ERR_FAILED_TO_FETCH_PPS (105)
/* HEVC Scaling Lists (all values are maximum possible ones) */
#define HEVCFW_SCALING_LIST_NUM_SIZES 4
#define HEVCFW_SCALING_LIST_NUM_MATRICES 6
#define HEVCFW_SCALING_LIST_MATRIX_SIZE 64

struct hevcfw_scaling_listdata {
	unsigned char dc_coeffs
	[HEVCFW_SCALING_LIST_NUM_SIZES - 2]
	[HEVCFW_SCALING_LIST_NUM_MATRICES];

	unsigned char lists
	[HEVCFW_SCALING_LIST_NUM_SIZES]
	[HEVCFW_SCALING_LIST_NUM_MATRICES]
	[HEVCFW_SCALING_LIST_MATRIX_SIZE];
};

/* HEVC Video Profile_Tier_Level */
struct hevcfw_profile_tier_level {
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, general_profile_space);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, general_tier_flag);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, general_profile_idc);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char,
			general_profile_compatibility_flag
			[HEVCFW_MAX_NUM_PROFILE_IDC]);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, general_progressive_source_flag);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, general_interlaced_source_flag);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, general_non_packed_constraint_flag);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, general_frame_only_constraint_flag);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, general_max_12bit_constraint_flag);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, general_max_10bit_constraint_flag);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, general_max_8bit_constraint_flag);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, general_max_422chroma_constraint_flag);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, general_max_420chroma_constraint_flag);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, general_max_monochrome_constraint_flag);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, general_intra_constraint_flag);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char,
			general_one_picture_only_constraint_flag);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, general_lower_bit_rate_constraint_flag);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, general_level_idc);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char,
			sub_layer_profile_present_flag[HEVCFW_MAX_NUM_SUBLAYERS - 1]);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char,
			sub_layer_level_present_flag[HEVCFW_MAX_NUM_SUBLAYERS - 1]);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char,
			sub_layer_profile_space[HEVCFW_MAX_NUM_SUBLAYERS - 1]);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char,
			sub_layer_tier_flag[HEVCFW_MAX_NUM_SUBLAYERS - 1]);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char,
			sub_layer_profile_idc[HEVCFW_MAX_NUM_SUBLAYERS - 1]);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char,
			sub_layer_profile_compatibility_flag
			[HEVCFW_MAX_NUM_SUBLAYERS - 1][HEVCFW_MAX_NUM_PROFILE_IDC]);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char,
			sub_layer_progressive_source_flag[HEVCFW_MAX_NUM_SUBLAYERS - 1]);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char,
			sub_layer_interlaced_source_flag[HEVCFW_MAX_NUM_SUBLAYERS - 1]);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char,
			sub_layer_non_packed_constraint_flag[HEVCFW_MAX_NUM_SUBLAYERS - 1]);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char,
			sub_layer_frame_only_constraint_flag[HEVCFW_MAX_NUM_SUBLAYERS - 1]);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char,
			sub_layer_max_12bit_constraint_flag[HEVCFW_MAX_NUM_SUBLAYERS - 1]);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char,
			sub_layer_max_10bit_constraint_flag[HEVCFW_MAX_NUM_SUBLAYERS - 1]);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char,
			sub_layer_max_8bit_constraint_flag[HEVCFW_MAX_NUM_SUBLAYERS - 1]);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char,
			sub_layer_max_422chroma_constraint_flag[HEVCFW_MAX_NUM_SUBLAYERS - 1]);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char,
			sub_layer_max_420chroma_constraint_flag[HEVCFW_MAX_NUM_SUBLAYERS - 1]);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char,
			sub_layer_max_monochrome_constraint_flag[HEVCFW_MAX_NUM_SUBLAYERS - 1]);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char,
			sub_layer_intra_constraint_flag[HEVCFW_MAX_NUM_SUBLAYERS - 1]);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char,
			sub_layer_one_picture_only_constraint_flag[HEVCFW_MAX_NUM_SUBLAYERS - 1]);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char,
			sub_layer_lower_bit_rate_constraint_flag[HEVCFW_MAX_NUM_SUBLAYERS - 1]);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char,
			sub_layer_level_idc[HEVCFW_MAX_NUM_SUBLAYERS - 1]);
};

struct hevcfw_video_ps {
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			int, is_different);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			int, is_sent);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			int, is_available);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, vps_video_parameter_set_id);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, vps_reserved_three_2bits);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, vps_max_layers_minus1);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, vps_max_sub_layers_minus1);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, vps_temporal_id_nesting_flag);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned short, vps_reserved_0xffff_16bits);
	struct hevcfw_profile_tier_level profile_tier_level;
};

/* HEVC Video Usability Information */
struct hevcfw_vui_params {
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, aspect_ratio_info_present_flag);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, aspect_ratio_idc);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned short, sar_width);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned short, sar_height);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, overscan_info_present_flag);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, overscan_appropriate_flag);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, video_signal_type_present_flag);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, video_format);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, video_full_range_flag);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, colour_description_present_flag);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, colour_primaries);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, transfer_characteristics);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, matrix_coeffs);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, chroma_loc_info_present_flag);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, chroma_sample_loc_type_top_field);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, chroma_sample_loc_type_bottom_field);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, neutral_chroma_indication_flag);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, field_seq_flag);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, frame_field_info_present_flag);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, default_display_window_flag);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned short, def_disp_win_left_offset);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned short, def_disp_win_right_offset);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned short, def_disp_win_top_offset);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned short, def_disp_win_bottom_offset);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, vui_timing_info_present_flag);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned int, vui_num_units_in_tick);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned int, vui_time_scale);
};

/* HEVC Short Term Reference Picture Set */
struct hevcfw_short_term_ref_picset {
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, num_negative_pics);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, num_positive_pics);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			short, delta_poc_s0[HEVCFW_MAX_NUM_REF_PICS]);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			short, delta_poc_s1[HEVCFW_MAX_NUM_REF_PICS]);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, used_bycurr_pic_s0[HEVCFW_MAX_NUM_REF_PICS]);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, used_bycurr_pic_s1[HEVCFW_MAX_NUM_REF_PICS]);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, num_delta_pocs);
};

/*
 * This describes the SPS header data required by the HEVC firmware that should
 * be supplied by the Host.
 */
struct hevcfw_sequence_ps {
	/* syntax elements from SPS */
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned short, pic_width_in_luma_samples);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned short, pic_height_in_luma_samples);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, num_short_term_ref_pic_sets);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, num_long_term_ref_pics_sps);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned short,
			lt_ref_pic_poc_lsb_sps[HEVCFW_MAX_NUM_LT_REF_PICS]);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char,
			used_by_curr_pic_lt_sps_flag[HEVCFW_MAX_NUM_LT_REF_PICS]);
	struct hevcfw_short_term_ref_picset st_rps_list[HEVCFW_MAX_NUM_ST_REF_PIC_SETS];

	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, sps_max_sub_layers_minus1);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char,
			sps_max_dec_pic_buffering_minus1[HEVCFW_MAX_NUM_SUBLAYERS]);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char,
			sps_max_num_reorder_pics[HEVCFW_MAX_NUM_SUBLAYERS]);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned int,
			sps_max_latency_increase_plus1[HEVCFW_MAX_NUM_SUBLAYERS]);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, max_transform_hierarchy_depth_inter);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, max_transform_hierarchy_depth_intra);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, log2_diff_max_min_transform_block_size);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, log2_min_transform_block_size_minus2);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char,
			log2_diff_max_min_luma_coding_block_size);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, log2_min_luma_coding_block_size_minus3);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, chroma_format_idc);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, separate_colour_plane_flag);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, num_extra_slice_header_bits);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, log2_max_pic_order_cnt_lsb_minus4);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, long_term_ref_pics_present_flag);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, sample_adaptive_offset_enabled_flag);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, sps_temporal_mvp_enabled_flag);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, bit_depth_luma_minus8);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, bit_depth_chroma_minus8);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, pcm_sample_bit_depth_luma_minus1);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, pcm_sample_bit_depth_chroma_minus1);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char,
			log2_min_pcm_luma_coding_block_size_minus3);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char,
			log2_diff_max_min_pcm_luma_coding_block_size);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, pcm_loop_filter_disabled_flag);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, amp_enabled_flag);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, pcm_enabled_flag);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, strong_intra_smoothing_enabled_flag);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, scaling_list_enabled_flag);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, transform_skip_rotation_enabled_flag);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, transform_skip_context_enabled_flag);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, implicit_rdpcm_enabled_flag);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, explicit_rdpcm_enabled_flag);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, extended_precision_processing_flag);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, intra_smoothing_disabled_flag);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, high_precision_offsets_enabled_flag);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, persistent_rice_adaptation_enabled_flag);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, cabac_bypass_alignment_enabled_flag);
	/* derived elements */
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned int, pic_size_in_ctbs_y);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned short, pic_height_in_ctbs_y);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned short, pic_width_in_ctbs_y);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, ctb_size_y);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, ctb_log2size_y);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			int, max_pic_order_cnt_lsb);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned int,
			sps_max_latency_pictures[HEVCFW_MAX_NUM_SUBLAYERS]);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, pps_seq_parameter_set_id);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, sps_video_parameter_set_id);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, sps_temporal_id_nesting_flag);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, sps_seq_parameter_set_id);

	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, conformance_window_flag);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned short, conf_win_left_offset);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned short, conf_win_right_offset);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned short, conf_win_top_offset);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned short, conf_win_bottom_offset);

	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, sps_sub_layer_ordering_info_present_flag);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, sps_scaling_list_data_present_flag);

	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, vui_parameters_present_flag);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, sps_extension_present_flag);

	struct hevcfw_vui_params vui_params;
	/* derived elements */
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, sub_width_c);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, sub_height_c);

	struct hevcfw_profile_tier_level profile_tier_level;
	struct hevcfw_scaling_listdata scaling_listdata;
};

/*
 * This describes the HEVC parser component "Header data", shown in the
 * Firmware Memory Layout diagram. This data is required by the HEVC firmware
 * and should be supplied by the Host.
 */
struct hevcfw_headerdata {
	/* Decode buffers and output control for the current picture */
	/* Primary decode buffer base addresses */
	struct vdecfw_image_buffer primary;
	/* buffer base addresses for alternate output */
	struct vdecfw_image_buffer alternate;
	/* address of buffer for temporal mv params */
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned int, temporal_outaddr);
};

/*
 * This describes the PPS header data required by the HEVC firmware that should
 * be supplied by the Host.
 */
struct hevcfw_picture_ps {
	/* syntax elements from the PPS */
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, pps_pic_parameter_set_id);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, num_tile_columns_minus1);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, num_tile_rows_minus1);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, diff_cu_qp_delta_depth);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, init_qp_minus26);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, pps_beta_offset_div2);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, pps_tc_offset_div2);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, pps_cb_qp_offset);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, pps_cr_qp_offset);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, log2_parallel_merge_level_minus2);

	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, dependent_slice_segments_enabled_flag);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, output_flag_present_flag);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, num_extra_slice_header_bits);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, lists_modification_present_flag);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, cabac_init_present_flag);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, weighted_pred_flag);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, weighted_bipred_flag);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char,
			pps_slice_chroma_qp_offsets_present_flag);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char,
			deblocking_filter_override_enabled_flag);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, tiles_enabled_flag);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, entropy_coding_sync_enabled_flag);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char,
			slice_segment_header_extension_present_flag);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, transquant_bypass_enabled_flag);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, cu_qp_delta_enabled_flag);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, transform_skip_enabled_flag);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, sign_data_hiding_enabled_flag);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, num_ref_idx_l0_default_active_minus1);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, num_ref_idx_l1_default_active_minus1);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, constrained_intra_pred_flag);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, pps_deblocking_filter_disabled_flag);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char,
			pps_loop_filter_across_slices_enabled_flag);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, loop_filter_across_tiles_enabled_flag);

	/* rewritten from SPS, maybe at some point we could get rid of this */
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, scaling_list_enabled_flag);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char,
			log2_max_transform_skip_block_size_minus2);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char,
			cross_component_prediction_enabled_flag);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, chroma_qp_offset_list_enabled_flag);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, diff_cu_chroma_qp_offset_depth);
	/*
	 * PVDEC derived elements. HEVCFW_SCALING_LISTS_BUFSIZE is
	 * multiplied by 2 to ensure that there will be space for address of
	 * each element. These addresses are completed in lower layer.
	 */
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned int,
			scaling_lists[HEVCFW_SCALING_LISTS_BUFSIZE * 2]);
	/* derived elements */
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned short, col_bd[HEVCFW_MAX_TILE_COLS + 1]);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned short, row_bd[HEVCFW_MAX_TILE_ROWS + 1]);

	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, chroma_qp_offset_list_len_minus1);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, cb_qp_offset_list[HEVCFW_MAX_CHROMA_QP]);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, cr_qp_offset_list[HEVCFW_MAX_CHROMA_QP]);

	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, uniform_spacing_flag);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char,
			column_width_minus1[HEVCFW_MAX_TILE_COLS]);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char,
			row_height_minus1[HEVCFW_MAX_TILE_ROWS]);

	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, pps_seq_parameter_set_id);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, deblocking_filter_control_present_flag);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, pps_scaling_list_data_present_flag);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, pps_extension_present_flag);

	struct hevcfw_scaling_listdata scaling_list;
};

/* This enum determines reference picture status */
enum hevcfw_reference_type {
	HEVCFW_REF_UNUSED      = 0,
	HEVCFW_REF_SHORTTERM,
	HEVCFW_REF_LONGTERM,
	HEVCFW_REF_FORCE32BITS = 0x7FFFFFFFU
};

/* This describes an HEVC picture. It is part of the Context data */
struct hevcfw_picture {
	/* Primary (reconstructed) picture buffers */
	struct vdecfw_image_buffer primary;
	/* Secondary (alternative) picture buffers */
	struct vdecfw_image_buffer alternate;
	/* Unique ID for this picture */
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned int, transaction_id);
	/* nut of first ssh of picture, determines picture type */
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, nalunit_type);
	/* Picture Order Count (frame number) */
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			int, pic_order_cnt_val);
	/* Slice Picture Order Count Lsb */
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			int, slice_pic_ordercnt_lsb);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, pic_output_flag);
	/* information about long-term pictures */
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned short, dpb_longterm_flags);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned int,
			dpb_pic_order_diff[HEVCFW_NUM_DPB_DIFF_REGS]);
	/* address of buffer for temporal mv params */
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned int, temporal_outaddr);
	/* worst case Dpb diff for the current pic */
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned int, dpb_diff);
};

/*
 * This is a wrapper for a picture to hold it in a Decoded Picture Buffer
 * for further reference
 */
struct hevcfw_picture_in_dpb {
	/* DPB data about the picture */
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			enum hevcfw_reference_type, ref_type);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, valid);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, needed_for_output);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, pic_latency_count);
	/* Picture itself */
	struct hevcfw_picture picture;
};

/*
 * This describes an HEVC's Decoded Picture Buffer (DPB).
 * It is part of the Context data
 */

#define HEVCFW_DPB_IDX_INVALID -1

struct hevcfw_decoded_picture_buffer {
	/* reference pictures */
	struct hevcfw_picture_in_dpb pictures[HEVCFW_MAX_DPB_SIZE];
	/* organizational data of DPB */
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT, unsigned int, fullness);
};

/*
 * This describes an HEVC's Reference Picture Set (RPS).
 * It is part of the Context data
 */
struct hevcfw_reference_picture_set {
	/* sizes of poc lists */
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, num_pocst_curr_before);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, num_pocst_curr_after);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, num_pocst_foll);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, num_poclt_curr);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, num_poclt_foll);
	/* poc lists */
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			int, pocst_curr_before[HEVCFW_MAX_NUM_REF_PICS]);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			int, pocst_curr_after[HEVCFW_MAX_NUM_REF_PICS]);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			int, pocst_foll[HEVCFW_MAX_NUM_REF_PICS]);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			int, poclt_curr[HEVCFW_MAX_NUM_REF_PICS]);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			int, poclt_foll[HEVCFW_MAX_NUM_REF_PICS]);
	/* derived elements */
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char,
			curr_delta_pocmsb_presentflag[HEVCFW_MAX_NUM_REF_PICS]);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char,
			foll_delta_pocmsb_presentflag[HEVCFW_MAX_NUM_REF_PICS]);
	/* reference picture sets: indices in DPB */
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, ref_picsetlt_curr[HEVCFW_MAX_NUM_REF_PICS]);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, ref_picsetlt_foll[HEVCFW_MAX_NUM_REF_PICS]);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char,
			ref_picsetst_curr_before[HEVCFW_MAX_NUM_REF_PICS]);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char,
			ref_picsetst_curr_after[HEVCFW_MAX_NUM_REF_PICS]);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, ref_picsetst_foll[HEVCFW_MAX_NUM_REF_PICS]);
};

/*
 * This describes the HEVC parser component "Context data", shown in the
 * Firmware Memory Layout diagram. This data is the state preserved across
 * pictures. It is loaded and saved by the Firmware, but requires the host to
 * provide buffer(s) for this.
 */
struct hevcfw_ctx_data {
	struct hevcfw_sequence_ps sps;
	struct hevcfw_picture_ps pps;
	/*
	 * data from last picture with TemporalId = 0 that is not a RASL, RADL
	 * or sub-layer non-reference picture
	 */
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			int, prev_pic_order_cnt_lsb);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			int, prev_pic_order_cnt_msb);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char, last_irapnorasl_output_flag);
	/*
	 * Decoded Pictures Buffer holds information about decoded pictures
	 * needed for further INTER decoding
	 */
	struct hevcfw_decoded_picture_buffer dpb;
	/* Reference Picture Set is determined on per-picture basis */
	struct hevcfw_reference_picture_set rps;
	/*
	 * Reference Picture List is determined using data from Reference
	 * Picture Set and from Slice (Segment) Header on per-slice basis
	 */
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT, unsigned char,
			ref_pic_list[HEVCFW_NUM_REF_PIC_LISTS][HEVCFW_MAX_NUM_REF_PICS]);
	/*
	 * Reference Picture List used to send reflist to the host, the only
	 * difference is that missing references are marked
	 * with HEVCFW_DPB_IDX_INVALID
	 */
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned char,
			ref_pic_listhlp[HEVCFW_NUM_REF_PIC_LISTS][HEVCFW_MAX_NUM_REF_PICS]);

	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned int, pic_count);
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			unsigned int, slice_segment_count);
	/* There was EOS NAL detected and no new picture yet */
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			int, eos_detected);
	/* This is first picture after EOS NAL */
	IMG_ALIGN_FIELD(VDECFW_SHARE_DEFAULT_ALIGNMENT,
			int, first_after_eos);
};

#endif /* _HEVCFW_DATA_H_ */
