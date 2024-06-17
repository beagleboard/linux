/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Public data structures for the h264 parser firmware module.
 *
 * Copyright (c) Imagination Technologies Ltd.
 * Copyright (c) 2021 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Authors:
 *	Amit Makani <amit.makani@ti.com>
 *
 * Re-written for upstreamimg
 *	Sidraya Jayagond <sidraya.bj@pathpartnertech.com>
 */

/* Include shared header version here to replace the standard version */
#include "h264fw_data_shared.h"

#ifndef _H264FW_DATA_H_
#define _H264FW_DATA_H_

#include "vdecfw_shared.h"

/* Maximum number of alternative CPB specifications in the stream */
#define H264_MAXIMUMVALUEOFCPB_CNT  32

/*
 * The maximum DPB size is related to the number of MVC views supported
 * The size is defined in H.10.2 for the H.264 spec.
 * If the number of views needs to be changed the DPB size should be too
 * The limits are as follows:
 * NumViews		1,  2,  4,  8, 16
 * MaxDpbFrames:	16, 16, 32, 48, 64
 */
#ifdef H264_ENABLE_MVC
#define H264FW_MAX_NUM_VIEWS            4
#define H264FW_MAX_DPB_SIZE             32
#define H264FW_MAX_NUM_MVC_REFS         16
#else
#define H264FW_MAX_NUM_VIEWS            1
#define H264FW_MAX_DPB_SIZE             16
#define H264FW_MAX_NUM_MVC_REFS         1
#endif

/* Maximum value for num_ref_frames_in_pic_order_cnt_cycle */
#define H264FW_MAX_CYCLE_REF_FRAMES     256

/* 4x4 scaling list size */
#define H264FW_4X4_SIZE                 16
/* 8x8 scaling list size */
#define H264FW_8X8_SIZE                 64
/* Number of 4x4 scaling lists */
#define H264FW_NUM_4X4_LISTS            6
/* Number of 8x8 scaling lists */
#define H264FW_NUM_8X8_LISTS            6

/* Number of reference picture lists */
#define H264FW_MAX_REFPIC_LISTS         2

/*
 * The maximum number of slice groups
 * remove if slice group map is prepared on the host
 */
#define H264FW_MAX_SLICE_GROUPS         8

/* The maximum number of planes for 4:4:4 separate color plane streams */
#define H264FW_MAX_PLANES               3

#define H264_MAX_SGM_SIZE               8196

#define IS_H264_HIGH_PROFILE(profile_idc, type) \
	({ \
		type __profile_idc = profile_idc;       \
		((__profile_idc) == H264_PROFILE_HIGH) ||      \
		((__profile_idc) == H264_PROFILE_HIGH10) ||      \
		((__profile_idc) == H264_PROFILE_HIGH422) ||      \
		((__profile_idc) == H264_PROFILE_HIGH444) ||      \
		((__profile_idc) == H264_PROFILE_CAVLC444) ||      \
		((__profile_idc) == H264_PROFILE_MVC_HIGH) ||      \
		((__profile_idc) == H264_PROFILE_MVC_STEREO); })        \

/*
 * This type describes the H.264 NAL unit types
 */
enum h264_enaltype {
	H264FW_NALTYPE_SLICE               = 1,
	H264FW_NALTYPE_IDRSLICE            = 5,
	H264FW_NALTYPE_SEI                 = 6,
	H264FW_NALTYPE_SPS                 = 7,
	H264FW_NALTYPE_PPS                 = 8,
	H264FW_NALTYPE_AUD                 = 9,
	H264FW_NALTYPE_EOSEQ               = 10,
	H264FW_NALTYPE_EOSTR               = 11,
	H264FW_NALTYPE_PREFIX              = 14,
	H264FW_NALTYPE_SUBSET_SPS          = 15,
	H264FW_NALTYPE_AUXILIARY_SLICE     = 19,
	H264FW_NALTYPE_EXTSLICE            = 20,
	H264FW_NALTYPE_EXTSLICE_DEPTH_VIEW = 21,
	H264FW_NALTYPE_FORCE32BITS         = 0x7FFFFFFFU
};

/*
 * AVC Profile IDC definitions
 */
enum h264_eprofileidc {
	/* YUV 4:4:4/14 "CAVLC 4:4:4 */
	H264_PROFILE_CAVLC444    =       44,
	/* YUV 4:2:0/8  "Baseline */
	H264_PROFILE_BASELINE    =       66,
	/* YUV 4:2:0/8  "Main */
	H264_PROFILE_MAIN        =       77,
	/* YUV 4:2:0/8  "Scalable" */
	H264_PROFILE_SCALABLE    =       83,
	/* YUV 4:2:0/8  "Extended" */
	H264_PROFILE_EXTENDED    =       88,
	/* YUV 4:2:0/8  "High" */
	H264_PROFILE_HIGH        =       100,
	/* YUV 4:2:0/10 "High 10" */
	H264_PROFILE_HIGH10      =       110,
	/* YUV 4:2:0/8  "Multiview High" */
	H264_PROFILE_MVC_HIGH    =       118,
	/* YUV 4:2:2/10 "High 4:2:2" */
	H264_PROFILE_HIGH422     =       122,
	/* YUV 4:2:0/8  "Stereo High" */
	H264_PROFILE_MVC_STEREO  =       128,
	/* YUV 4:4:4/14 "High 4:4:4" */
	H264_PROFILE_HIGH444     =       244,
	H264_PROFILE_FORCE32BITS = 0x7FFFFFFFU
};

/*
 * This type defines the constraint set flags
 */
enum h264fw_econstraint_flag {
	/* Compatible with Baseline profile */
	H264FW_CONSTRAINT_BASELINE_SHIFT   =       7,
	/* Compatible with Main profile */
	H264FW_CONSTRAINT_MAIN_SHIFT       =       6,
	/* Compatible with Extended profile */
	H264FW_CONSTRAINT_EXTENDED_SHIFT   =       5,
	/* Compatible with Intra profiles */
	H264FW_CONSTRAINT_INTRA_SHIFT      =       4,
	/* Compatible with Multiview High profile */
	H264FW_CONSTRAINT_MULTIHIGH_SHIFT  =       3,
	/* Compatible with Stereo High profile */
	H264FW_CONSTRAINT_STEREOHIGH_SHIFT =       2,
	/* Reserved flag */
	H264FW_CONSTRAINT_RESERVED6_SHIFT  =       1,
	/* Reserved flag */
	H264FW_CONSTRAINT_RESERVED7_SHIFT  =       0,
	H264FW_CONSTRAINT_FORCE32BITS      = 0x7FFFFFFFU
};

/*
 * This enum describes the reference status of an H.264 picture.
 * Unpaired fields should have all eRefStatusX set to the same value
 *
 * For Frame, Mbaff, and Pair types individual fields and frame ref status
 * should be set accordingly.
 *
 * eRefStatusFrame    eRefStatusTop   eRefStatusBottom
 * UNUSED             UNUSED          UNUSED
 * SHORTTERM          SHORTTERM       SHORTTERM
 * LONGTERM           LONGTERM        LONGTERM
 *
 * UNUSED             SHORT/LONGTERM  UNUSED
 * UNUSED             UNUSED          SHORT/LONGTERM
 *
 * SHORTTERM          LONGTERM        SHORTTERM
 * SHORTTERM          SHORTTERM       LONGTERM
 * NB: It is not clear from the spec if the Frame should be marked as short
 * or long term in this case
 */
enum h264fw_ereference {
	/* Picture is unused for reference */
	H264FW_REF_UNUSED = 0,
	/* used for short term reference */
	H264FW_REF_SHORTTERM,
	/* used for Long Term reference */
	H264FW_REF_LONGTERM,
	H264FW_REF_FORCE32BITS = 0x7FFFFFFFU
};

/*
 * This type defines the picture structure.
 */
enum h264fw_epicture_type {
	/* No valid picture */
	H264FW_TYPE_NONE = 0,
	/* Picture contains the top (even) lines of the frame */
	H264FW_TYPE_TOP,
	/* Picture contains the bottom (odd) lines of the frame */
	H264FW_TYPE_BOTTOM,
	/* Picture contains the entire frame */
	H264FW_TYPE_FRAME,
	/* Picture contains an MBAFF frame */
	H264FW_TYPE_MBAFF,
	/* Picture contains top and bottom lines of the frame */
	H264FW_TYPE_PAIR,
	H264FW_TYPE_FORCE32BITS = 0x7FFFFFFFU
};

/*
 * This describes the SPS header data required by the H264 firmware that should
 * be supplied by the Host.
 */
struct h264fw_sequence_ps {
	/* syntax elements from SPS */
	/* syntax element from bitstream - 8 bit */
	unsigned char profile_idc;
	/* syntax element from bitstream - 2 bit */
	unsigned char chroma_format_idc;
	/* syntax element from bitstream - 1 bit */
	unsigned char separate_colour_plane_flag;
	/* syntax element from bitstream - 3 bit */
	unsigned char bit_depth_luma_minus8;
	/* syntax element from bitstream - 3 bit */
	unsigned char bit_depth_chroma_minus8;
	/* syntax element from bitstream - 1 bit */
	unsigned char delta_pic_order_always_zero_flag;
	/* syntax element from bitstream - 4+ bit */
	unsigned char log2_max_pic_order_cnt_lsb;
	/* syntax element from bitstream - 5 bit */
	unsigned char max_num_ref_frames;
	/* syntax element from bitstream - 4+ bit */
	unsigned char log2_max_frame_num;
	/* syntax element from bitstream - 2 bit */
	unsigned char pic_order_cnt_type;
	/* syntax element from bitstream - 1 bit */
	unsigned char frame_mbs_only_flag;
	/* syntax element from bitstream - 1 bit */
	unsigned char gaps_in_frame_num_value_allowed_flag;

	/*
	 * set0--7 flags as they occur in the bitstream (including reserved
	 * values)
	 */
	unsigned char constraint_set_flags;
	/* syntax element from bitstream - 8 bit */
	unsigned char level_idc;
	/* syntax element from bitstream - 8 bit */
	unsigned char num_ref_frames_in_pic_order_cnt_cycle;
	/* syntax element from bitstream - 1 bit */
	unsigned char mb_adaptive_frame_field_flag;
	/* syntax element from bitstream - 32 bit */
	int offset_for_non_ref_pic;
	/* syntax element from bitstream - 32 bit */
	int offset_for_top_to_bottom_field;

	/* syntax element from bitstream */
	unsigned int pic_width_in_mbs_minus1;
	/* syntax element from bitstream */
	unsigned int pic_height_in_map_units_minus1;
	/* syntax element from bitstream - 1 bit */
	unsigned char direct_8x8_inference_flag;
	/* syntax element from bitstream */
	unsigned char qpprime_y_zero_transform_bypass_flag;

	/* syntax element from bitstream - 32 bit each */
	int offset_for_ref_frame[H264FW_MAX_CYCLE_REF_FRAMES];

	/* From VUI information */
	unsigned char num_reorder_frames;
	/*
	 * From VUI/MVC SEI, 0 indicates not set, any actual 0 value will be
	 * inferred by the firmware
	 */
	unsigned char max_dec_frame_buffering;

	/* From SPS MVC Extension - for the current view_id */
	/* Number of views in this stream */
	unsigned char num_views;
	/* a Map in order of VOIdx of view_id's */
	unsigned short view_ids[H264FW_MAX_NUM_VIEWS];

	/* Disable VDMC horizontal/vertical filtering */
	unsigned char disable_vdmc_filt;
	/* Disable CABAC 4:4:4 4x4 transform as not available */
	unsigned char transform4x4_mb_not_available;

	/* anchor reference list */
	unsigned short anchor_inter_view_reference_id_list[2]
	[H264FW_MAX_NUM_VIEWS][H264FW_MAX_NUM_MVC_REFS];
	/* nonanchor reference list */
	unsigned short non_anchor_inter_view_reference_id_list[2]
	[H264FW_MAX_NUM_VIEWS][H264FW_MAX_NUM_MVC_REFS];
	/* number of elements in aui16AnchorInterViewReferenceIndiciesLX[] */
	unsigned short num_anchor_refsx[2][H264FW_MAX_NUM_VIEWS];
	/* number of elements in aui16NonAnchorInterViewReferenceIndiciesLX[] */
	unsigned short num_non_anchor_refsx[2][H264FW_MAX_NUM_VIEWS];
};

/*
 * This structure represents HRD parameters.
 */
struct h264fw_hrd {
	/* cpb_cnt_minus1 */
	unsigned char cpb_cnt_minus1;
	/* bit_rate_scale */
	unsigned char bit_rate_scale;
	/* cpb_size_scale */
	unsigned char cpb_size_scale;
	/* bit_rate_value_minus1 */
	unsigned int bit_rate_value_minus1[H264_MAXIMUMVALUEOFCPB_CNT];
	/* cpb_size_value_minus1 */
	unsigned int cpb_size_value_minus1[H264_MAXIMUMVALUEOFCPB_CNT];
	/* cbr_flag */
	unsigned char cbr_flag[H264_MAXIMUMVALUEOFCPB_CNT];
	/* initial_cpb_removal_delay_length_minus1 */
	unsigned char initial_cpb_removal_delay_length_minus1;
	/* cpb_removal_delay_length_minus1 */
	unsigned char cpb_removal_delay_length_minus1;
	/* dpb_output_delay_length_minus1 */
	unsigned char dpb_output_delay_length_minus1;
	/* time_offset_length */
	unsigned char time_offset_length;
};

/*
 * This structure represents the VUI parameters data.
 */
struct h264fw_vui {
	int aspect_ratio_info_present_flag;
	unsigned char aspect_ratio_idc;
	unsigned short sar_width;
	unsigned short sar_height;
	int overscan_info_present_flag;
	int overscan_appropriate_flag;
	int video_signal_type_present_flag;
	unsigned char video_format;
	int video_full_range_flag;
	int colour_description_present_flag;
	unsigned char colour_primaries;
	unsigned char transfer_characteristics;
	unsigned char matrix_coefficients;
	int chroma_location_info_present_flag;
	unsigned int chroma_sample_loc_type_top_field;
	unsigned int chroma_sample_loc_type_bottom_field;
	int timing_info_present_flag;
	unsigned int num_units_in_tick;
	unsigned int time_scale;
	int fixed_frame_rate_flag;
	int nal_hrd_parameters_present_flag;
	struct h264fw_hrd nal_hrd_params;
	int vcl_hrd_parameters_present_flag;
	struct h264fw_hrd vcl_hrd_params;
	int low_delay_hrd_flag;
	int pic_struct_present_flag;
	int bitstream_restriction_flag;
	int motion_vectors_over_pic_boundaries_flag;
	unsigned int max_bytes_per_pic_denom;
	unsigned int max_bits_per_mb_denom;
	unsigned int log2_max_mv_length_vertical;
	unsigned int log2_max_mv_length_horizontal;
	unsigned int num_reorder_frames;
	unsigned int max_dec_frame_buffering;
};

/*
 * This describes the HW specific SPS header data required by the H264
 * firmware that should be supplied by the Host.
 */
struct h264fw_ddsequence_ps {
	/* pre-packed registers derived from SPS */
	/* Value for CR_VEC_ENTDEC_FE_CONTROL */
	unsigned int regentdec_control;

	/* NB: This register should contain the 4-bit SGM flag */
	/* Value for CR_VEC_H264_FE_SPS0 & CR_VEC_H264_BE_SPS0 combined */
	unsigned int reg_sps0;
	/* Value of CR_VEC_H264_BE_INTRA_8x8 */
	unsigned int reg_beintra;
	/* Value of CR_VEC_H264_FE_CABAC444 */
	unsigned int reg_fecaabac444;
	/* Treat CABAC 4:4:4 4x4 transform as not available */
	unsigned char transform4x4_mb_notavialbale;
	/* Disable VDMC horizontal/vertical filtering */
	unsigned char disable_vdmcfilt;
};

/*
 * This describes the PPS header data required by the H264 firmware that should
 * be supplied by the Host.
 */
struct h264fw_picture_ps {
	/* syntax elements from the PPS */
	/* syntax element from bitstream - 1 bit */
	unsigned char deblocking_filter_control_present_flag;
	/* syntax element from bitstream - 1 bit */
	unsigned char transform_8x8_mode_flag;
	/* syntax element from bitstream - 1 bit */
	unsigned char entropy_coding_mode_flag;
	/* syntax element from bitstream - 1 bit */
	unsigned char redundant_pic_cnt_present_flag;

	/* syntax element from bitstream - 2 bit */
	unsigned char weighted_bipred_idc;
	/* syntax element from bitstream - 1 bit */
	unsigned char weighted_pred_flag;
	/* syntax element from bitstream - 1 bit */
	unsigned char pic_order_present_flag;

	/* 26 + syntax element from bitstream - 7 bit */
	unsigned char pic_init_qp;
	/* syntax element from bitstream - 1 bit */
	unsigned char constrained_intra_pred_flag;
	/* syntax element from bitstream - 5 bit each */
	unsigned char num_ref_lx_active_minus1[H264FW_MAX_REFPIC_LISTS];

	/* syntax element from bitstream - 3 bit */
	unsigned char slice_group_map_type;
	/* syntax element from bitstream - 3 bit */
	unsigned char num_slice_groups_minus1;
	/* syntax element from bitstream - 13 bit */
	unsigned short slice_group_change_rate_minus1;

	/* syntax element from bitstream */
	unsigned int chroma_qp_index_offset;
	/* syntax element from bitstream */
	unsigned int second_chroma_qp_index_offset;

	/* scaling lists are derived from both SPS and PPS information */
	/* but will change whenever the PPS changes */
	/* The derived set of tables are associated here with the PPS */
	/* NB: These are in H.264 order */
	/* derived from SPS and PPS - 8 bit each */
	unsigned char scalinglist4x4[H264FW_NUM_4X4_LISTS][H264FW_4X4_SIZE];
	/* derived from SPS and PPS - 8 bit each */
	unsigned char scalinglist8x8[H264FW_NUM_8X8_LISTS][H264FW_8X8_SIZE];
};

/*
 * This describes the HW specific PPS header data required by the H264
 * firmware that should be supplied by the Host.
 */
struct h264fw_dd_picture_ps {
	/* values derived from the PPS */
	/* Value for MSVDX_CMDS_SLICE_PARAMS_MODE_CONFIG */
	unsigned char vdmc_mode_config;

	/* pre-packed registers derived from the PPS */
	/* Value for CR_VEC_H264_FE_PPS0 & CR_VEC_H264_BE_PPS0 combined */
	unsigned int reg_pps0;

	/*
	 * scaling lists are derived from both SPS and PPS information
	 * but will change whenever the PPS changes
	 * The derived set of tables are associated here with the PPS
	 * But this will become invalid if the SPS changes and will have to be
	 * recalculated
	 * These tables MUST be aligned on a 32-bit boundary
	 * NB: These are in MSVDX order
	 */
	/* derived from SPS and PPS - 8 bit each */
	unsigned char scalinglist4x4[H264FW_NUM_4X4_LISTS][H264FW_4X4_SIZE];
	/* derived from SPS and PPS - 8 bit each */
	unsigned char scalinglist8x8[H264FW_NUM_8X8_LISTS][H264FW_8X8_SIZE];
};

/*
 * This describes the H.264 parser component "Header data", shown in the
 * Firmware Memory Layout diagram. This data is required by the H264 firmware
 * and should be supplied by the Host.
 */
struct h264fw_header_data {
	/* Decode buffers and output control for the current picture */
	/* Primary decode buffer base addresses */
	struct vdecfw_image_buffer primary;
	/* buffer base addresses for alternate output */
	struct vdecfw_image_buffer alternate;
	/* Output control: rotation, scaling, oold, etc. */
	unsigned int pic_cmds[VDECFW_CMD_MAX];
	/* Macroblock parameters base address for the picture */
	unsigned int mbparams_base_address;

	unsigned int mbparams_size_per_plane;

	/* Buffers for context preload for colour plane switching (6.x.x) */
	unsigned int preload_buffer_base_address
	[H264FW_MAX_PLANES];

	/*
	 * slice group map should be calculated on Host
	 * (using some slice params) and base address provided here
	 */
	/* Base address of active slice group map */
	/* Base address of active slice group map */
	unsigned int slicegroupmap_base_address;

	/* H264 specific control */
	/* do second pass Intra Deblock on frame */
	unsigned int do_old __attribute__ (aligned(4));
	/* set to IMG_FALSE to disable second-pass deblock */
	unsigned int two_pass_flag __attribute__ (aligned(4));
	/* set to IMG_TRUE to disable MVC */
	unsigned int disable_mvc __attribute__ (aligned(4));
	/*
	 * Do we have second PPS in uipSecondPPSInfoSource provided for the
	 * second field
	 */
	unsigned int second_pps __attribute__ (aligned(4));
};

/*
 * This describes an H.264 picture. It is part of the Context data
 */
struct h264fw_picture {
	/* Primary (reconstructed) picture buffers */
	struct vdecfw_image_buffer primary;
	/* Secondary (alternative) picture buffers */
	struct vdecfw_image_buffer alternate;
	/* Macroblock parameters base address for the picture */
	unsigned int mbparams_base_address;

	/* Unique ID for this picture */
	unsigned int transaction_id;
	/* Picture type */
	enum h264fw_epicture_type pricture_type;

	/* Reference status of the picture */
	enum h264fw_ereference ref_status_bottom;
	/* Reference status of the picture */
	enum h264fw_ereference ref_status_top;
	/* Reference status of the picture */
	enum h264fw_ereference ref_status_frame;

	/* Frame Number */
	unsigned int frame_number;
	/* Short term reference info */
	int fame_number_wrap;
	/* long term reference number - should be 8-bit */
	unsigned int longterm_frame_idx;

	/* Top field order count for this picture */
	int top_field_order_count;
	/* Bottom field order count for this picture */
	int bottom_field_order_count;
	/* MVC view_id */
	unsigned short view_id;
	/*
	 * When picture is in the DPB Offset to use into the MSVDX DPB reg table
	 * when the current picture is the same view as this.
	 */
	unsigned char view_dpb_offset;
	/* Flags for this picture for the display process */
	unsigned char display_flags;

	/* IMG_FALSE if sent to display, or otherwise not needed for display */
	unsigned char needed_for_output;
};

/*
 * This structure describes frame data for POC calculation
 */
struct h264fw_poc_picture_data {
	/* type 0,1,2 */
	unsigned char mmco_5_flag;

	/* type 0 */
	unsigned char bottom_field_flag;
	unsigned short pic_order_cnt_lsb;
	int top_field_order_count;
	int pic_order_count_msb;

	/* type 1,2 */
	int16 frame_num;
	int frame_num_offset;

	/* output */
	int bottom_filed_order_count;
};

/*
 * This structure describes picture data for determining Complementary
 * Field Pairs
 */
struct h264fw_last_pic_data {
	/* Unique ID for this picture */
	unsigned int transaction_id;
	/* Picture type */
	enum h264fw_epicture_type picture_type;
	/* Reference status of the picture */
	enum h264fw_ereference ref_status_frame;
	/* Frame Number */
	unsigned int frame_number;

	unsigned int luma_recon;
	unsigned int chroma_recon;
	unsigned int chroma_2_recon;
	unsigned int luma_alter;
	unsigned int chroma_alter;
	unsigned int chroma_2_alter;
	struct vdecfw_image_buffer primary;
	struct vdecfw_image_buffer alternate;
	unsigned int mbparams_base_address;
	/* Top field order count for this picture */
	int top_field_order_count;
	/* Bottom field order count for this picture */
	int bottom_field_order_count;
};

/*
 * This describes the H.264 parser component "Context data", shown in the
 * Firmware Memory Layout diagram. This data is the state preserved across
 * pictures. It is loaded and saved by the Firmware, but requires the host to
 * provide buffer(s) for this.
 */
struct h264fw_context_data {
	/* Decoded Picture Buffer */
	struct h264fw_picture dpb[H264FW_MAX_DPB_SIZE];
	/*
	 * Inter-view reference components - also used as detail of the previous
	 * picture for any particular view, can be used to determine
	 * complemetary field pairs
	 */
	struct h264fw_picture interview_prediction_ref[H264FW_MAX_NUM_VIEWS];
	/* previous ref pic for type0, previous pic for type1&2 */
	struct h264fw_poc_picture_data prev_poc_pic_data[H264FW_MAX_NUM_VIEWS];
	/* previous picture information to detect complementary field pairs */
	struct h264fw_last_pic_data last_pic_data[H264FW_MAX_NUM_VIEWS];
	struct h264fw_last_pic_data last_displayed_pic_data
	[H264FW_MAX_NUM_VIEWS];

	/* previous reference frame number for each view */
	unsigned short prev_ref_frame_num[H264FW_MAX_NUM_VIEWS];
	/* Bitmap of used slots in each view DPB */
	unsigned short dpb_bitmap[H264FW_MAX_NUM_VIEWS];

	/* DPB size */
	unsigned int dpb_size;
	/* Number of pictures in DPB */
	unsigned int dpb_fullness;

	unsigned char prev_display_flags;
	int prev_display;
	int prev_release;
	/* Active parameter sets */
	/* Sequence Parameter Set data */
	struct h264fw_sequence_ps sps;
	/* Picture Parameter Set data */
	struct h264fw_picture_ps pps;
	/*
	 * Picture Parameter Set data for second field when in the same buffer
	 */
	struct h264fw_picture_ps second_pps;

	/* Set if stream is MVC */
	int mvc;
	/* DPB long term reference information */
	int max_longterm_frame_idx[H264FW_MAX_NUM_VIEWS];
};

#endif /* _H264FW_DATA_H_ */
