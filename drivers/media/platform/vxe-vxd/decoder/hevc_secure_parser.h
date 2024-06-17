/* SPDX-License-Identifier: GPL-2.0 */
/*
 * h.264 secure data unit parsing API.
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
#ifndef __HEVCSECUREPARSER_H__
#define __HEVCSECUREPARSER_H__

#include "bspp_int.h"

#define HEVC_MAX_NUM_PROFILE_IDC        (32)
#define HEVC_MAX_NUM_SUBLAYERS          (7)
#define HEVC_MAX_VPS_OP_SETS_PLUS1      (1024)
#define HEVC_MAX_VPS_NUH_RESERVED_ZERO_LAYER_ID_PLUS1   (1)
#define HEVC_MAX_NUM_REF_PICS           (16)
#define HEVC_MAX_NUM_ST_REF_PIC_SETS    (65)
#define HEVC_MAX_NUM_LT_REF_PICS        (32)
#define HEVC_MAX_NUM_REF_IDX_ACTIVE     (15)
#define HEVC_LEVEL_IDC_MIN              (30)
#define HEVC_LEVEL_IDC_MAX              (186)
#define HEVC_1_0_PROFILE_IDC_MAX        (3)
#define HEVC_MAX_CPB_COUNT              (32)
#define HEVC_MIN_CODED_UNIT_SIZE        (8)

/* hevc scaling lists (all values are maximum possible ones) */
#define HEVC_SCALING_LIST_NUM_SIZES     (4)
#define HEVC_SCALING_LIST_NUM_MATRICES  (6)
#define HEVC_SCALING_LIST_MATRIX_SIZE   (64)

#define HEVC_MAX_TILE_COLS              (20)
#define HEVC_MAX_TILE_ROWS              (22)

#define HEVC_EXTENDED_SAR               (255)

#define HEVC_MAX_CHROMA_QP              (6)

enum hevc_nalunittype {
	HEVC_NALTYPE_TRAIL_N        = 0,
	HEVC_NALTYPE_TRAIL_R        = 1,
	HEVC_NALTYPE_TSA_N          = 2,
	HEVC_NALTYPE_TSA_R          = 3,
	HEVC_NALTYPE_STSA_N         = 4,
	HEVC_NALTYPE_STSA_R         = 5,
	HEVC_NALTYPE_RADL_N         = 6,
	HEVC_NALTYPE_RADL_R         = 7,
	HEVC_NALTYPE_RASL_N         = 8,
	HEVC_NALTYPE_RASL_R         = 9,
	HEVC_NALTYPE_RSV_VCL_N10    = 10,
	HEVC_NALTYPE_RSV_VCL_R11    = 11,
	HEVC_NALTYPE_RSV_VCL_N12    = 12,
	HEVC_NALTYPE_RSV_VCL_R13    = 13,
	HEVC_NALTYPE_RSV_VCL_N14    = 14,
	HEVC_NALTYPE_RSV_VCL_R15    = 15,
	HEVC_NALTYPE_BLA_W_LP       = 16,
	HEVC_NALTYPE_BLA_W_RADL     = 17,
	HEVC_NALTYPE_BLA_N_LP       = 18,
	HEVC_NALTYPE_IDR_W_RADL     = 19,
	HEVC_NALTYPE_IDR_N_LP       = 20,
	HEVC_NALTYPE_CRA            = 21,
	HEVC_NALTYPE_RSV_IRAP_VCL22 = 22,
	HEVC_NALTYPE_RSV_IRAP_VCL23 = 23,
	HEVC_NALTYPE_VPS            = 32,
	HEVC_NALTYPE_SPS            = 33,
	HEVC_NALTYPE_PPS            = 34,
	HEVC_NALTYPE_AUD            = 35,
	HEVC_NALTYPE_EOS            = 36,
	HEVC_NALTYPE_EOB            = 37,
	HEVC_NALTYPE_FD             = 38,
	HEVC_NALTYPE_PREFIX_SEI     = 39,
	HEVC_NALTYPE_SUFFIX_SEI     = 40,
	HEVC_NALTYPE_FORCE32BITS    = 0x7FFFFFFFU
};

enum bspp_hevcslicetype {
	HEVC_SLICE_B           = 0,
	HEVC_SLICE_P           = 1,
	HEVC_SLICE_I           = 2,
	HEVC_SLICE_FORCE32BITS = 0x7FFFFFFFU
};

/* HEVC NAL unit header */
struct bspp_hevcnalheader {
	unsigned char nal_unit_type;
	unsigned char nuh_layer_id;
	unsigned char nuh_temporal_id_plus1;
};

/* HEVC video profile_tier_level */
struct bspp_hevc_profile_tierlevel {
	unsigned char general_profile_space;
	unsigned char general_tier_flag;
	unsigned char general_profile_idc;
	unsigned char general_profile_compatibility_flag[HEVC_MAX_NUM_PROFILE_IDC];
	unsigned char general_progressive_source_flag;
	unsigned char general_interlaced_source_flag;
	unsigned char general_non_packed_constraint_flag;
	unsigned char general_frame_only_constraint_flag;
	unsigned char general_max_12bit_constraint_flag;
	unsigned char general_max_10bit_constraint_flag;
	unsigned char general_max_8bit_constraint_flag;
	unsigned char general_max_422chroma_constraint_flag;
	unsigned char general_max_420chroma_constraint_flag;
	unsigned char general_max_monochrome_constraint_flag;
	unsigned char general_intra_constraint_flag;
	unsigned char general_one_picture_only_constraint_flag;
	unsigned char general_lower_bit_rate_constraint_flag;
	unsigned char general_level_idc;
	unsigned char sub_layer_profile_present_flag[HEVC_MAX_NUM_SUBLAYERS - 1];
	unsigned char sub_layer_level_present_flag[HEVC_MAX_NUM_SUBLAYERS - 1];
	unsigned char sub_layer_profile_space[HEVC_MAX_NUM_SUBLAYERS - 1];
	unsigned char sub_layer_tier_flag[HEVC_MAX_NUM_SUBLAYERS - 1];
	unsigned char sub_layer_profile_idc[HEVC_MAX_NUM_SUBLAYERS - 1];
	unsigned char sub_layer_profile_compatibility_flag[HEVC_MAX_NUM_SUBLAYERS -
						   1][HEVC_MAX_NUM_PROFILE_IDC];
	unsigned char sub_layer_progressive_source_flag[HEVC_MAX_NUM_SUBLAYERS - 1];
	unsigned char sub_layer_interlaced_source_flag[HEVC_MAX_NUM_SUBLAYERS - 1];
	unsigned char sub_layer_non_packed_constraint_flag[HEVC_MAX_NUM_SUBLAYERS - 1];
	unsigned char sub_layer_frame_only_constraint_flag[HEVC_MAX_NUM_SUBLAYERS - 1];
	unsigned char sub_layer_max_12bit_constraint_flag[HEVC_MAX_NUM_SUBLAYERS - 1];
	unsigned char sub_layer_max_10bit_constraint_flag[HEVC_MAX_NUM_SUBLAYERS - 1];
	unsigned char sub_layer_max_8bit_constraint_flag[HEVC_MAX_NUM_SUBLAYERS - 1];
	unsigned char sub_layer_max_422chroma_constraint_flag[HEVC_MAX_NUM_SUBLAYERS - 1];
	unsigned char sub_layer_max_420chroma_constraint_flag[HEVC_MAX_NUM_SUBLAYERS - 1];
	unsigned char sub_layer_max_monochrome_constraint_flag[HEVC_MAX_NUM_SUBLAYERS - 1];
	unsigned char sub_layer_intra_constraint_flag[HEVC_MAX_NUM_SUBLAYERS - 1];
	unsigned char sub_layer_one_picture_only_constraint_flag[HEVC_MAX_NUM_SUBLAYERS - 1];
	unsigned char sub_layer_lower_bit_rate_constraint_flag[HEVC_MAX_NUM_SUBLAYERS - 1];
	unsigned char sub_layer_level_idc[HEVC_MAX_NUM_SUBLAYERS - 1];
};

/* HEVC sub layer HRD parameters */
struct bspp_hevc_sublayer_hrd_parameters {
	unsigned char bit_rate_value_minus1[HEVC_MAX_CPB_COUNT];
	unsigned char cpb_size_value_minus1[HEVC_MAX_CPB_COUNT];
	unsigned char cpb_size_du_value_minus1[HEVC_MAX_CPB_COUNT];
	unsigned char bit_rate_du_value_minus1[HEVC_MAX_CPB_COUNT];
	unsigned char cbr_flag[HEVC_MAX_CPB_COUNT];
};

/* HEVC HRD parameters */
struct bspp_hevc_hrd_parameters {
	unsigned char nal_hrd_parameters_present_flag;
	unsigned char vcl_hrd_parameters_present_flag;
	unsigned char sub_pic_hrd_params_present_flag;
	unsigned char tick_divisor_minus2;
	unsigned char du_cpb_removal_delay_increment_length_minus1;
	unsigned char sub_pic_cpb_params_in_pic_timing_sei_flag;
	unsigned char dpb_output_delay_du_length_minus1;
	unsigned char bit_rate_scale;
	unsigned char cpb_size_scale;
	unsigned char cpb_size_du_scale;
	unsigned char initial_cpb_removal_delay_length_minus1;
	unsigned char au_cpb_removal_delay_length_minus1;
	unsigned char dpb_output_delay_length_minus1;
	unsigned char fixed_pic_rate_general_flag[HEVC_MAX_NUM_SUBLAYERS];
	unsigned char fixed_pic_rate_within_cvs_flag[HEVC_MAX_NUM_SUBLAYERS];
	unsigned char elemental_duration_in_tc_minus1[HEVC_MAX_NUM_SUBLAYERS];
	unsigned char low_delay_hrd_flag[HEVC_MAX_NUM_SUBLAYERS];
	unsigned char cpb_cnt_minus1[HEVC_MAX_NUM_SUBLAYERS];
	struct bspp_hevc_sublayer_hrd_parameters sublayhrdparams[HEVC_MAX_NUM_SUBLAYERS];
};

/* HEVC video parameter set */
struct bspp_hevc_vps {
	unsigned char is_different;
	unsigned char is_sent;
	unsigned char is_available;
	unsigned char vps_video_parameter_set_id;
	unsigned char vps_reserved_three_2bits;
	unsigned char vps_max_layers_minus1;
	unsigned char vps_max_sub_layers_minus1;
	unsigned char vps_temporal_id_nesting_flag;
	unsigned short vps_reserved_0xffff_16bits;
	struct bspp_hevc_profile_tierlevel profiletierlevel;
	unsigned char vps_max_dec_pic_buffering_minus1[HEVC_MAX_NUM_SUBLAYERS];
	unsigned char vps_max_num_reorder_pics[HEVC_MAX_NUM_SUBLAYERS];
	unsigned char vps_max_latency_increase_plus1[HEVC_MAX_NUM_SUBLAYERS];
	unsigned char vps_sub_layer_ordering_info_present_flag;
	unsigned char vps_max_layer_id;
	unsigned char vps_num_layer_sets_minus1;
	unsigned char layer_id_included_flag[HEVC_MAX_VPS_OP_SETS_PLUS1]
		[HEVC_MAX_VPS_NUH_RESERVED_ZERO_LAYER_ID_PLUS1];
	unsigned char vps_timing_info_present_flag;
	unsigned int vps_num_units_in_tick;
	unsigned int vps_time_scale;
	unsigned char vps_poc_proportional_to_timing_flag;
	unsigned char vps_num_ticks_poc_diff_one_minus1;
	unsigned char vps_num_hrd_parameters;
	unsigned char *hrd_layer_set_idx;
	unsigned char *cprms_present_flag;
	unsigned char vps_extension_flag;
	unsigned char vps_extension_data_flag;
};

/* HEVC scaling lists */
struct bspp_hevc_scalinglist_data {
	unsigned char dccoeffs[HEVC_SCALING_LIST_NUM_SIZES - 2][HEVC_SCALING_LIST_NUM_MATRICES];
	unsigned char lists[HEVC_SCALING_LIST_NUM_SIZES][HEVC_SCALING_LIST_NUM_MATRICES]
		[HEVC_SCALING_LIST_MATRIX_SIZE];
};

/* HEVC short term reference picture set */
struct bspp_hevc_shortterm_refpicset {
	unsigned char num_negative_pics;
	unsigned char num_positive_pics;
	short delta_poc_s0[HEVC_MAX_NUM_REF_PICS];
	short delta_poc_s1[HEVC_MAX_NUM_REF_PICS];
	unsigned char used_bycurr_pic_s0[HEVC_MAX_NUM_REF_PICS];
	unsigned char used_bycurr_pic_s1[HEVC_MAX_NUM_REF_PICS];
	unsigned char num_delta_pocs;
};

/* HEVC video usability information */
struct bspp_hevc_vui_params {
	unsigned char aspect_ratio_info_present_flag;
	unsigned char aspect_ratio_idc;
	unsigned short sar_width;
	unsigned short sar_height;
	unsigned char overscan_info_present_flag;
	unsigned char overscan_appropriate_flag;
	unsigned char video_signal_type_present_flag;
	unsigned char video_format;
	unsigned char video_full_range_flag;
	unsigned char colour_description_present_flag;
	unsigned char colour_primaries;
	unsigned char transfer_characteristics;
	unsigned char matrix_coeffs;
	unsigned char chroma_loc_info_present_flag;
	unsigned char chroma_sample_loc_type_top_field;
	unsigned char chroma_sample_loc_type_bottom_field;
	unsigned char neutral_chroma_indication_flag;
	unsigned char field_seq_flag;
	unsigned char frame_field_info_present_flag;
	unsigned char default_display_window_flag;
	unsigned short def_disp_win_left_offset;
	unsigned short def_disp_win_right_offset;
	unsigned short def_disp_win_top_offset;
	unsigned short def_disp_win_bottom_offset;
	unsigned char vui_timing_info_present_flag;
	unsigned int vui_num_units_in_tick;
	unsigned int vui_time_scale;
	unsigned char vui_poc_proportional_to_timing_flag;
	unsigned int vui_num_ticks_poc_diff_one_minus1;
	unsigned char vui_hrd_parameters_present_flag;
	struct bspp_hevc_hrd_parameters vui_hrd_params;
	unsigned char bitstream_restriction_flag;
	unsigned char tiles_fixed_structure_flag;
	unsigned char motion_vectors_over_pic_boundaries_flag;
	unsigned char restricted_ref_pic_lists_flag;
	unsigned short min_spatial_segmentation_idc;
	unsigned char max_bytes_per_pic_denom;
	unsigned char max_bits_per_min_cu_denom;
	unsigned char log2_max_mv_length_horizontal;
	unsigned char log2_max_mv_length_vertical;
};

/* HEVC sps range extensions */
struct bspp_hevc_sps_range_exts {
	unsigned char transform_skip_rotation_enabled_flag;
	unsigned char transform_skip_context_enabled_flag;
	unsigned char implicit_rdpcm_enabled_flag;
	unsigned char explicit_rdpcm_enabled_flag;
	unsigned char extended_precision_processing_flag;
	unsigned char intra_smoothing_disabled_flag;
	unsigned char high_precision_offsets_enabled_flag;
	unsigned char persistent_rice_adaptation_enabled_flag;
	unsigned char cabac_bypass_alignment_enabled_flag;
};

/* HEVC sequence parameter set */
struct bspp_hevc_sps {
	unsigned char is_different;
	unsigned char is_sent;
	unsigned char is_available;
	unsigned char sps_video_parameter_set_id;
	unsigned char sps_max_sub_layers_minus1;
	unsigned char sps_temporal_id_nesting_flag;
	struct bspp_hevc_profile_tierlevel profile_tier_level;
	unsigned char sps_seq_parameter_set_id;
	unsigned char chroma_format_idc;
	unsigned char separate_colour_plane_flag;
	unsigned int pic_width_in_luma_samples;
	unsigned int pic_height_in_luma_samples;
	unsigned char conformance_window_flag;
	unsigned short conf_win_left_offset;
	unsigned short conf_win_right_offset;
	unsigned short conf_win_top_offset;
	unsigned short conf_win_bottom_offset;
	unsigned char bit_depth_luma_minus8;
	unsigned char bit_depth_chroma_minus8;
	unsigned char log2_max_pic_order_cnt_lsb_minus4;
	unsigned char sps_sub_layer_ordering_info_present_flag;
	unsigned char sps_max_dec_pic_buffering_minus1[HEVC_MAX_NUM_SUBLAYERS];
	unsigned char sps_max_num_reorder_pics[HEVC_MAX_NUM_SUBLAYERS];
	unsigned int sps_max_latency_increase_plus1[HEVC_MAX_NUM_SUBLAYERS];
	unsigned char log2_min_luma_coding_block_size_minus3;
	unsigned char log2_diff_max_min_luma_coding_block_size;
	unsigned char log2_min_transform_block_size_minus2;
	unsigned char log2_diff_max_min_transform_block_size;
	unsigned char max_transform_hierarchy_depth_inter;
	unsigned char max_transform_hierarchy_depth_intra;
	unsigned char scaling_list_enabled_flag;
	unsigned char sps_scaling_list_data_present_flag;
	struct bspp_hevc_scalinglist_data scalinglist_data;
	unsigned char amp_enabled_flag;
	unsigned char sample_adaptive_offset_enabled_flag;
	unsigned char pcm_enabled_flag;
	unsigned char pcm_sample_bit_depth_luma_minus1;
	unsigned char pcm_sample_bit_depth_chroma_minus1;
	unsigned char log2_min_pcm_luma_coding_block_size_minus3;
	unsigned char log2_diff_max_min_pcm_luma_coding_block_size;
	unsigned char pcm_loop_filter_disabled_flag;
	unsigned char num_short_term_ref_pic_sets;
	struct bspp_hevc_shortterm_refpicset rps_list[HEVC_MAX_NUM_ST_REF_PIC_SETS];
	unsigned char long_term_ref_pics_present_flag;
	unsigned char num_long_term_ref_pics_sps;
	unsigned short lt_ref_pic_poc_lsb_sps[HEVC_MAX_NUM_LT_REF_PICS];
	unsigned char used_by_curr_pic_lt_sps_flag[HEVC_MAX_NUM_LT_REF_PICS];
	unsigned char sps_temporal_mvp_enabled_flag;
	unsigned char strong_intra_smoothing_enabled_flag;
	unsigned char vui_parameters_present_flag;
	struct bspp_hevc_vui_params vui_params;
	unsigned char sps_extension_present_flag;
	unsigned char sps_range_extensions_flag;
	struct bspp_hevc_sps_range_exts range_exts;
	unsigned char sps_extension_7bits;
	unsigned char sps_extension_data_flag;
	/* derived elements */
	unsigned char sub_width_c;
	unsigned char sub_height_c;
	unsigned char ctb_log2size_y;
	unsigned char ctb_size_y;
	unsigned int pic_width_in_ctbs_y;
	unsigned int pic_height_in_ctbs_y;
	unsigned int pic_size_in_ctbs_y;
	int max_pic_order_cnt_lsb;
	unsigned int sps_max_latency_pictures[HEVC_MAX_NUM_SUBLAYERS];
	/* raw vui data as extracted from bitstream. */
	struct bspp_raw_bitstream_data *vui_raw_data;
};

/**
 * struct bspp_hevc_sequ_hdr_info - This structure contains HEVC sequence
 *					header information (VPS, SPS, VUI)
 *					contains everything parsed from the
 *					video/sequence header.
 * @vps: HEVC sequence header information
 * @sps:HEVC sequence header information
 */
struct bspp_hevc_sequ_hdr_info {
	struct bspp_hevc_vps vps;
	struct bspp_hevc_sps sps;
};

/* HEVC pps range extensions */
struct bspp_hevc_pps_range_exts {
	unsigned char log2_max_transform_skip_block_size_minus2;
	unsigned char cross_component_prediction_enabled_flag;
	unsigned char chroma_qp_offset_list_enabled_flag;
	unsigned char diff_cu_chroma_qp_offset_depth;
	unsigned char chroma_qp_offset_list_len_minus1;
	unsigned char cb_qp_offset_list[HEVC_MAX_CHROMA_QP];
	unsigned char cr_qp_offset_list[HEVC_MAX_CHROMA_QP];
	unsigned char log2_sao_offset_scale_luma;
	unsigned char log2_sao_offset_scale_chroma;
};

/* HEVC picture parameter set */
struct bspp_hevc_pps {
	unsigned char is_available;
	unsigned char is_param_copied;
	unsigned char pps_pic_parameter_set_id;
	unsigned char pps_seq_parameter_set_id;
	unsigned char dependent_slice_segments_enabled_flag;
	unsigned char output_flag_present_flag;
	unsigned char num_extra_slice_header_bits;
	unsigned char sign_data_hiding_enabled_flag;
	unsigned char cabac_init_present_flag;
	unsigned char num_ref_idx_l0_default_active_minus1;
	unsigned char num_ref_idx_l1_default_active_minus1;
	unsigned char init_qp_minus26;
	unsigned char constrained_intra_pred_flag;
	unsigned char transform_skip_enabled_flag;
	unsigned char cu_qp_delta_enabled_flag;
	unsigned char diff_cu_qp_delta_depth;
	int pps_cb_qp_offset;
	int pps_cr_qp_offset;
	unsigned char pps_slice_chroma_qp_offsets_present_flag;
	unsigned char weighted_pred_flag;
	unsigned char weighted_bipred_flag;
	unsigned char transquant_bypass_enabled_flag;
	unsigned char tiles_enabled_flag;
	unsigned char entropy_coding_sync_enabled_flag;
	unsigned char num_tile_columns_minus1;
	unsigned char num_tile_rows_minus1;
	unsigned char uniform_spacing_flag;
	unsigned char column_width_minus1[HEVC_MAX_TILE_COLS];
	unsigned char row_height_minus1[HEVC_MAX_TILE_ROWS];
	unsigned char loop_filter_across_tiles_enabled_flag;
	unsigned char pps_loop_filter_across_slices_enabled_flag;
	unsigned char deblocking_filter_control_present_flag;
	unsigned char deblocking_filter_override_enabled_flag;
	unsigned char pps_deblocking_filter_disabled_flag;
	unsigned char pps_beta_offset_div2;
	unsigned char pps_tc_offset_div2;
	unsigned char pps_scaling_list_data_present_flag;
	struct bspp_hevc_scalinglist_data scaling_list;
	unsigned char lists_modification_present_flag;
	unsigned char log2_parallel_merge_level_minus2;
	unsigned char slice_segment_header_extension_present_flag;
	unsigned char pps_extension_present_flag;
	unsigned char pps_range_extensions_flag;
	struct bspp_hevc_pps_range_exts range_exts;
	unsigned char pps_extension_7bits;
	unsigned char pps_extension_data_flag;
	/* derived elements */
	unsigned short col_bd[HEVC_MAX_TILE_COLS + 1];
	unsigned short row_bd[HEVC_MAX_TILE_ROWS + 1];
	/* PVDEC derived elements */
	unsigned int max_tile_height_in_ctbs_y;
};

/* HEVC slice segment header */
struct bspp_hevc_slice_segment_header {
	unsigned char bslice_is_idr;
	unsigned char first_slice_segment_in_pic_flag;
	unsigned char no_output_of_prior_pics_flag;
	unsigned char slice_pic_parameter_set_id;
	unsigned char dependent_slice_segment_flag;
	unsigned int slice_segment_address;
};

/*
 * @Function   bspp_hevc_set_parser_config
 * sets the parser configuration.
 */
int bspp_hevc_set_parser_config(enum vdec_bstr_format bstr_format,
				struct bspp_vid_std_features *pvidstd_features,
				struct bspp_swsr_ctx *pswsr_ctx,
				struct bspp_parser_callbacks *pparser_callbacks,
				struct bspp_inter_pict_data *pinterpict_data);

void bspp_hevc_determine_unittype(unsigned char bitstream_unittype,
				  int disable_mvc,
				  enum bspp_unit_type *bspp_unittype);

#endif /*__H264SECUREPARSER_H__ */
