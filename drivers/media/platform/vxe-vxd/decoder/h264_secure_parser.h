/* SPDX-License-Identifier: GPL-2.0 */
/*
 * h.264 secure data unit parsing API.
 *
 * Copyright (c) Imagination Technologies Ltd.
 * Copyright (c) 2021 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Authors:
 *	Lakshmi Sankar <lakshmisankar-t@ti.com>
 * Re-written for upstreming
 *	Prashanth Kumar Amai <prashanth.ka@pathpartnertech.com>
 *	Sidraya Jayagond <sidraya.bj@pathpartnertech.com>
 */
#ifndef __H264SECUREPARSER_H__
#define __H264SECUREPARSER_H__

#include "bspp_int.h"
#include "vdec_defs.h"

/*
 * enum h264_nalunittype
 * @Description Contains H264 NAL unit types
 */
enum h264_nalunittype {
	H264_NALTYPE_UNSPECIFIED                      = 0,
	H264_NALTYPE_SLICE                            = 1,
	H264_NALTYPE_SLICE_PARTITION_A                = 2,
	H264_NALTYPE_SLICE_PARTITION_B                = 3,
	H264_NALTYPE_SLICE_PARTITION_C                = 4,
	H264_NALTYPE_IDR_SLICE                        = 5,
	H264_NALTYPE_SUPPLEMENTAL_ENHANCEMENT_INFO    = 6,
	H264_NALTYPE_SEQUENCE_PARAMETER_SET           = 7,
	H264_NALTYPE_PICTURE_PARAMETER_SET            = 8,
	H264_NALTYPE_ACCESS_UNIT_DELIMITER            = 9,
	H264_NALTYPE_END_OF_SEQUENCE                  = 10,
	H264_NALTYPE_END_OF_STREAM                    = 11,
	H264_NALTYPE_FILLER_DATA                      = 12,
	H264_NALTYPE_SEQUENCE_PARAMETER_SET_EXTENSION = 13,
	H264_NALTYPE_SLICE_PREFIX                     = 14,
	H264_NALTYPE_SUBSET_SPS                       = 15,
	H264_NALTYPE_AUXILIARY_SLICE                  = 19,
	H264_NALTYPE_SLICE_SCALABLE                   = 20,
	H264_NALTYPE_SLICE_IDR_SCALABLE               = 21,
	H264_NALTYPE_MAX                              = 31,
	H264_NALTYPE_FORCE32BITS                      = 0x7FFFFFFFU
};

/*
 * struct bspp_h264_sps_info
 * @Description	H264 SPS parsed information
 */
struct bspp_h264_sps_info {
	unsigned int profile_idc;
	unsigned int constraint_set_flags;
	unsigned int level_idc;
	unsigned char seq_parameter_set_id;
	unsigned char chroma_format_idc;
	int separate_colour_plane_flag;
	unsigned int bit_depth_luma_minus8;
	unsigned int bit_depth_chroma_minus8;
	unsigned char qpprime_y_zero_transform_bypass_flag;
	int seq_scaling_matrix_present_flag;
	unsigned char seq_scaling_list_present_flag[12];
	unsigned int log2_max_frame_num_minus4;
	unsigned int pic_order_cnt_type;
	unsigned int log2_max_pic_order_cnt_lsb_minus4;
	int delta_pic_order_always_zero_flag;
	int offset_for_non_ref_pic;
	int offset_for_top_to_bottom_field;
	unsigned int num_ref_frames_in_pic_order_cnt_cycle;
	unsigned int *offset_for_ref_frame;
	unsigned int max_num_ref_frames;
	int gaps_in_frame_num_value_allowed_flag;
	unsigned int pic_width_in_mbs_minus1;
	unsigned int pic_height_in_map_units_minus1;
	int frame_mbs_only_flag;
	int mb_adaptive_frame_field_flag;
	int direct_8x8_inference_flag;
	int frame_cropping_flag;
	unsigned int frame_crop_left_offset;
	unsigned int frame_crop_right_offset;
	unsigned int frame_crop_top_offset;
	unsigned int frame_crop_bottom_offset;
	int vui_parameters_present_flag;
	/* mvc_vui_parameters_present_flag;   UNUSED */
	int bmvcvuiparameterpresentflag;
	/*
	 * scaling lists are derived from both SPS and PPS information
	 * but will change whenever the PPS changes
	 * The derived set of tables are associated here with the PPS
	 * NB: These are in H.264 order
	 */
	/* derived from SPS and PPS - 8 bit each */
	unsigned char *scllst4x4seq;
	/* derived from SPS and PPS - 8 bit each */
	unsigned char *scllst8x8seq;
	/* This is not direct parsed data, though it is extracted */
	unsigned char usedefaultscalingmatrixflag_seq[12];
};

struct bspp_h264_hrdparam_info {
	unsigned char cpb_cnt_minus1;
	unsigned char bit_rate_scale;
	unsigned char cpb_size_scale;
	unsigned int *bit_rate_value_minus1;
	unsigned int *cpb_size_value_minus1;
	unsigned char *cbr_flag;
	unsigned char initial_cpb_removal_delay_length_minus1;
	unsigned char cpb_removal_delay_length_minus1;
	unsigned char dpb_output_delay_length_minus1;
	unsigned char time_offset_length;
};

struct bspp_h264_vui_info {
	unsigned char aspect_ratio_info_present_flag;
	unsigned int aspect_ratio_idc;
	unsigned int sar_width;
	unsigned int sar_height;
	unsigned char overscan_info_present_flag;
	unsigned char overscan_appropriate_flag;
	unsigned char video_signal_type_present_flag;
	unsigned int video_format;
	unsigned char video_full_range_flag;
	unsigned char colour_description_present_flag;
	unsigned int colour_primaries;
	unsigned int transfer_characteristics;
	unsigned int matrix_coefficients;
	unsigned char chroma_location_info_present_flag;
	unsigned int chroma_sample_loc_type_top_field;
	unsigned int chroma_sample_loc_type_bottom_field;
	unsigned char timing_info_present_flag;
	unsigned int num_units_in_tick;
	unsigned int time_scale;
	unsigned char fixed_frame_rate_flag;
	unsigned char nal_hrd_parameters_present_flag;
	struct bspp_h264_hrdparam_info nal_hrd_parameters;
	unsigned char vcl_hrd_parameters_present_flag;
	struct bspp_h264_hrdparam_info vcl_hrd_parameters;
	unsigned char low_delay_hrd_flag;
	unsigned char pic_struct_present_flag;
	unsigned char bitstream_restriction_flag;
	unsigned char motion_vectors_over_pic_boundaries_flag;
	unsigned int max_bytes_per_pic_denom;
	unsigned int max_bits_per_mb_denom;
	unsigned int log2_max_mv_length_vertical;
	unsigned int log2_max_mv_length_horizontal;
	unsigned int num_reorder_frames;
	unsigned int max_dec_frame_buffering;
};

/*
 * struct bspp_h264_seq_hdr_info
 * @Description	Contains everything parsed from the Sequence Header.
 */
struct bspp_h264_seq_hdr_info {
	/* Video sequence header information */
	struct bspp_h264_sps_info sps_info;
	/* VUI sequence header information. */
	struct bspp_h264_vui_info vui_info;
};

/**
 * struct bspp_h264_ppssgm_info - This structure contains H264 PPS parse data.
 * @slice_group_id: slice_group_id
 * @slicegroupidnum: slicegroupidnum
 */
struct bspp_h264_ppssgm_info {
	unsigned char *slice_group_id;
	unsigned short slicegroupidnum;
};

/*
 * struct bspp_h264_pps_info
 * @Description	This structure contains H264 PPS parse data.
 */
struct bspp_h264_pps_info {
	/* pic_parameter_set_id: defines the PPS ID of the current PPS */
	int pps_id;
	/* seq_parameter_set_id: defines the SPS that current PPS points to */
	int seq_parameter_set_id;
	int entropy_coding_mode_flag;
	int pic_order_present_flag;
	unsigned char num_slice_groups_minus1;
	unsigned char slice_group_map_type;
	unsigned short run_length_minus1[8];
	unsigned short top_left[8];
	unsigned short bottom_right[8];
	int slice_group_change_direction_flag;
	unsigned short slice_group_change_rate_minus1;
	unsigned short pic_size_in_map_unit;
	struct bspp_h264_ppssgm_info h264_ppssgm_info;
	unsigned char num_ref_idx_lx_active_minus1[H264FW_MAX_REFPIC_LISTS];
	int weighted_pred_flag;
	unsigned char weighted_bipred_idc;
	int pic_init_qp_minus26;
	int pic_init_qs_minus26;
	int chroma_qp_index_offset;
	int deblocking_filter_control_present_flag;
	int constrained_intra_pred_flag;
	int redundant_pic_cnt_present_flag;
	int transform_8x8_mode_flag;
	int pic_scaling_matrix_present_flag;
	unsigned char pic_scaling_list_present_flag[12];
	int second_chroma_qp_index_offset;

	/*
	 * scaling lists are derived from both SPS and PPS information
	 * but will change whenever the PPS changes
	 * The derived set of tables are associated here with the PPS
	 * NB: These are in H.264 order
	 */
	/* derived from SPS and PPS - 8 bit each */
	unsigned char *scllst4x4pic;
	/* derived from SPS and PPS - 8 bit each */
	unsigned char *scllst8x8pic;
	/* This is not direct parsed data, though it is extracted */
	unsigned char usedefaultscalingmatrixflag_pic[12];
};

/*
 * enum bspp_h264_slice_type
 * @Description	contains H264 slice types
 */
enum bspp_h264_slice_type {
	P_SLICE = 0,
	B_SLICE,
	I_SLICE,
	SP_SLICE,
	SI_SLICE,
	SLICE_TYPE_FORCE32BITS = 0x7FFFFFFFU
};

/*
 * struct bspp_h264_slice_hdr_info
 * @Description This structure contains H264 slice header information
 */
struct bspp_h264_slice_hdr_info {
	unsigned short first_mb_in_slice;
	enum bspp_h264_slice_type slice_type;

	/* data to ID new picture */
	unsigned int pps_id;
	unsigned int frame_num;
	unsigned char colour_plane_id;
	unsigned char field_pic_flag;
	unsigned char bottom_field_flag;
	unsigned int idr_pic_id;
	unsigned int pic_order_cnt_lsb;
	int delta_pic_order_cnt_bottom;
	int delta_pic_order_cnt[2];
	unsigned int redundant_pic_cnt;

	/* Things we need to read out when doing In Secure */
	unsigned char num_ref_idx_active_override_flag;
	unsigned char num_ref_idx_lx_active_minus1[2];
	unsigned short slice_group_change_cycle;
};

/*
 * @Function	bspp_h264_set_parser_config
 * @Description	Sets the parser configuration
 */
int bspp_h264_set_parser_config(enum vdec_bstr_format bstr_format,
				struct bspp_vid_std_features *pvidstd_features,
				struct bspp_swsr_ctx *pswsr_ctx,
				struct bspp_parser_callbacks *pparser_callbacks,
				struct bspp_inter_pict_data *pinterpict_data);

/*
 * @Function	bspp_h264_determine_unittype
 * @Description	This function determines the BSPP unit type based on the
 *		provided bitstream (H264 specific) unit type
 */
void bspp_h264_determine_unittype(unsigned char bitstream_unittype,
				  int disable_mvc,
				  enum bspp_unit_type *pbsppunittype);

#endif /*__H264SECUREPARSER_H__ */
