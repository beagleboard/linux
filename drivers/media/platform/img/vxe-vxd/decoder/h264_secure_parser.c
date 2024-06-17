// SPDX-License-Identifier: GPL-2.0
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

#include <linux/dma-mapping.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-mem2mem.h>

#include "bspp.h"
#include "bspp_int.h"
#include "h264_secure_parser.h"
#include "pixel_api.h"
#include "swsr.h"
#include "vdec_defs.h"

/*
 * Reduce DPB to 1 when no pic reordering.
 */
#define SL_MAX_REF_IDX          32
#define VUI_CPB_CNT_MAX         32
#define MAX_SPS_COUNT           32
#define MAX_PPS_COUNT           256
/* changed from 810 */
#define MAX_SLICE_GROUPMBS      65536
#define MAX_SPS_COUNT           32
#define MAX_PPS_COUNT           256
#define MAX_SLICEGROUP_COUNT    8
#define MAX_WIDTH_IN_MBS        256
#define MAX_HEIGHT_IN_MBS       256
#define MAX_COLOR_PLANE         4
#define H264_MAX_SGM_SIZE       8196

#define H264_MAX_CHROMA_QP_INDEX_OFFSET (12)
#define H264_MIN_CHROMA_QP_INDEX_OFFSET (-12)

/*
 * AVC Profile IDC definitions
 */
enum h264_profile_idc {
	h264_profile_cavlc444   = 44,   /*  YUV 4:4:4/14 "CAVLC 4:4:4" */
	h264_profile_baseline   = 66,   /* YUV 4:2:0/8  "Baseline" */
	h264_profile_main       = 77,   /* YUV 4:2:0/8  "Main" */
	h264_profile_scalable   = 83,   /* YUV 4:2:0/8  "Scalable" */
	h264_profile_extended   = 88,   /* YUV 4:2:0/8  "Extended" */
	h264_profile_high       = 100,  /* YUV 4:2:0/8  "High" */
	h264_profile_hig10     = 110,  /* YUV 4:2:0/10 "High 10" */
	h264_profile_mvc_high   = 118,  /* YUV 4:2:0/8  "Multiview High" */
	h264_profile_high422    = 122,  /* YUV 4:2:2/10 "High 4:2:2" */
	h264_profile_mvc_stereo = 128,  /* YUV 4:2:0/8  "Stereo High" */
	h264_profile_high444    = 244,  /* YUV 4:4:4/14 "High 4:4:4" */
	h264_profile_FORCE32BITS = 0x7FFFFFFFU
};

/*
 * Remap H.264 colour format into internal representation.
 */
static const enum pixel_fmt_idc pixel_format_idc[] = {
	PIXEL_FORMAT_MONO,
	PIXEL_FORMAT_420,
	PIXEL_FORMAT_422,
	PIXEL_FORMAT_444,
};

/*
 * Pixel Aspect Ratio
 */
static const unsigned short pixel_aspect[17][2] = {
	{ 0, 1 },
	{ 1, 1 },
	{ 12, 11 },
	{ 10, 11 },
	{ 16, 11 },
	{ 40, 33 },
	{ 24, 11 },
	{ 20, 11 },
	{ 32, 11 },
	{ 80, 33 },
	{ 18, 11 },
	{ 15, 11 },
	{ 64, 33 },
	{ 160, 99 },
	{ 4, 3 },
	{ 3, 2 },
	{ 2, 1 },
};

/*
 * Table 7-3, 7-4: Default Scaling lists
 */
static const unsigned char default_4x4_intra[16] = {
	6, 13, 13, 20,
	20, 20, 28, 28,
	28, 28, 32, 32,
	32, 37, 37, 42
};

static const unsigned char default_4x4_inter[16] = {
	10, 14, 14, 20,
	20, 20, 24, 24,
	24, 24, 27, 27,
	27, 30, 30, 34
};

static const unsigned char default_8x8_intra[64] = {
	6, 10, 10, 13, 11, 13, 16, 16,
	16, 16, 18, 18, 18, 18, 18, 23,
	23, 23, 23, 23, 23, 25, 25, 25,
	25, 25, 25, 25, 27, 27, 27, 27,
	27, 27, 27, 27, 29, 29, 29, 29,
	29, 29, 29, 31, 31, 31, 31, 31,
	31, 33, 33, 33, 33, 33, 36, 36,
	36, 36, 38, 38, 38, 40, 40, 42
};

static const unsigned char default_8x8_inter[64] = {
	9, 13, 13, 15, 13, 15, 17, 17,
	17, 17, 19, 19, 19, 19, 19, 21,
	21, 21, 21, 21, 21, 22, 22, 22,
	22, 22, 22, 22, 24, 24, 24, 24,
	24, 24, 24, 24, 25, 25, 25, 25,
	25, 25, 25, 27, 27, 27, 27, 27,
	27, 28, 28, 28, 28, 28, 30, 30,
	30, 30, 32, 32, 32, 33, 33, 35
};

/*
 * to be use if no q matrix is chosen
 */
static const unsigned char default_4x4_org[16] = {
	16, 16, 16, 16,
	16, 16, 16, 16,
	16, 16, 16, 16,
	16, 16, 16, 16
};

/*
 * to be use if no q matrix is chosen
 */
static const unsigned char default_8x8_org[64] = {
	16, 16, 16, 16, 16, 16, 16, 16,
	16, 16, 16, 16, 16, 16, 16, 16,
	16, 16, 16, 16, 16, 16, 16, 16,
	16, 16, 16, 16, 16, 16, 16, 16,
	16, 16, 16, 16, 16, 16, 16, 16,
	16, 16, 16, 16, 16, 16, 16, 16,
	16, 16, 16, 16, 16, 16, 16, 16,
	16, 16, 16, 16, 16, 16, 16, 16
};

/*
 * source: ITU-T H.264 2010/03, page 20 Table 6-1
 */
static const int bspp_subheightc[] = { -1, 2, 1, 1 };

/*
 * source: ITU-T H.264 2010/03, page 20 Table 6-1
 */
static const int bspp_subwidthc[] = { -1, 2, 2, 1 };

static inline int smin(int a, int b)
{
	return (((a) < (b)) ? (a) : (b));
}

static inline int smax(int a, int b)
{
	return (((a) > (b)) ? (a) : (b));
}

static void set_if_not_determined_yet(int *determined,
				      unsigned char condition,
				      int *target,
				      unsigned int value)
{
	if ((!(*determined)) && (condition)) {
		*target = value;
		*determined = 1;
	}
}

static int bspp_h264_get_subwidthc(int chroma_format_idc, int separate_colour_plane_flag)
{
	return bspp_subwidthc[chroma_format_idc];
}

static int bspp_h264_get_subheightc(int chroma_format_idc, int separate_colour_plane_flag)
{
	return bspp_subheightc[chroma_format_idc];
}

static unsigned int h264ceillog2(unsigned int value)
{
	unsigned int status = 0;

	value -= 1;
	while (value > 0) {
		value >>= 1;
		status++;
	}
	return status;
}

/*
 * @Function              bspp_h264_set_default_vui
 * @Description           Sets default values of the VUI info
 */
static void bspp_h264_set_default_vui(struct bspp_h264_vui_info *vui_info)
{
	unsigned int *nal_hrd_bitrate_valueminus1 = NULL;
	unsigned int *vcl_hrd_bitrate_valueminus1 = NULL;
	unsigned int *nal_hrd_cpbsize_valueminus1 = NULL;
	unsigned int *vcl_hrd_cpbsize_valueminus1 = NULL;
	unsigned char *nal_hrd_cbr_flag = NULL;
	unsigned char *vcl_hrd_cbr_flag = NULL;

	/* Saving pointers */
	nal_hrd_bitrate_valueminus1 = vui_info->nal_hrd_parameters.bit_rate_value_minus1;
	vcl_hrd_bitrate_valueminus1 = vui_info->vcl_hrd_parameters.bit_rate_value_minus1;

	nal_hrd_cpbsize_valueminus1 = vui_info->nal_hrd_parameters.cpb_size_value_minus1;
	vcl_hrd_cpbsize_valueminus1 = vui_info->vcl_hrd_parameters.cpb_size_value_minus1;

	nal_hrd_cbr_flag = vui_info->nal_hrd_parameters.cbr_flag;
	vcl_hrd_cbr_flag = vui_info->vcl_hrd_parameters.cbr_flag;

	/* Cleaning sVUIInfo */
	if (vui_info->nal_hrd_parameters.bit_rate_value_minus1)
		memset(vui_info->nal_hrd_parameters.bit_rate_value_minus1, 0x00,
		       VDEC_H264_MAXIMUMVALUEOFCPB_CNT * sizeof(unsigned int));

	if (vui_info->nal_hrd_parameters.cpb_size_value_minus1)
		memset(vui_info->nal_hrd_parameters.cpb_size_value_minus1, 0x00,
		       VDEC_H264_MAXIMUMVALUEOFCPB_CNT * sizeof(unsigned int));

	if (vui_info->vcl_hrd_parameters.cpb_size_value_minus1)
		memset(vui_info->vcl_hrd_parameters.cpb_size_value_minus1, 0x00,
		       VDEC_H264_MAXIMUMVALUEOFCPB_CNT * sizeof(unsigned int));

	if (vui_info->nal_hrd_parameters.cbr_flag)
		memset(vui_info->nal_hrd_parameters.cbr_flag, 0x00,
		       VDEC_H264_MAXIMUMVALUEOFCPB_CNT * sizeof(unsigned char));

	if (vui_info->vcl_hrd_parameters.cbr_flag)
		memset(vui_info->vcl_hrd_parameters.cbr_flag, 0x00,
		       VDEC_H264_MAXIMUMVALUEOFCPB_CNT * sizeof(unsigned char));

	/* Make sure you set default for everything */
	memset(vui_info, 0, sizeof(*vui_info));
	vui_info->video_format                            = 5;
	vui_info->colour_primaries                        = 2;
	vui_info->transfer_characteristics                = 2;
	vui_info->matrix_coefficients                     = 2;
	vui_info->motion_vectors_over_pic_boundaries_flag = 1;
	vui_info->max_bytes_per_pic_denom                 = 2;
	vui_info->max_bits_per_mb_denom                   = 1;
	vui_info->log2_max_mv_length_horizontal           = 16;
	vui_info->log2_max_mv_length_vertical             = 16;

#ifdef REDUCED_DPB_NO_PIC_REORDERING
	vui_info->max_dec_frame_buffering                 = 1;
	vui_info->num_reorder_frames                      = 0;
#else
	vui_info->max_dec_frame_buffering                 = 0;
	vui_info->num_reorder_frames                      = vui_info->max_dec_frame_buffering;
#endif

	/* Restoring pointers */
	vui_info->nal_hrd_parameters.bit_rate_value_minus1 = nal_hrd_bitrate_valueminus1;
	vui_info->vcl_hrd_parameters.bit_rate_value_minus1 = vcl_hrd_bitrate_valueminus1;

	vui_info->nal_hrd_parameters.cpb_size_value_minus1 = nal_hrd_cpbsize_valueminus1;
	vui_info->vcl_hrd_parameters.cpb_size_value_minus1 = vcl_hrd_cpbsize_valueminus1;

	vui_info->nal_hrd_parameters.cbr_flag = nal_hrd_cbr_flag;
	vui_info->vcl_hrd_parameters.cbr_flag = vcl_hrd_cbr_flag;
}

/*
 * @Function              bspp_h264_hrd_param_parser
 * @Description           Parse the HRD parameter
 */
static enum bspp_error_type bspp_h264_hrd_param_parser
					(void *swsr_context,
					 struct bspp_h264_hrdparam_info *h264_hrd_param_info)
{
	unsigned int sched_sel_idx;

	VDEC_ASSERT(swsr_context);
	h264_hrd_param_info->cpb_cnt_minus1 = swsr_read_unsigned_expgoulomb(swsr_context);

	if (h264_hrd_param_info->cpb_cnt_minus1 >= 32)
		pr_info("pb_cnt_minus1 is not within the range");

	h264_hrd_param_info->bit_rate_scale = swsr_read_bits(swsr_context, 4);
	h264_hrd_param_info->cpb_size_scale = swsr_read_bits(swsr_context, 4);

	if (!h264_hrd_param_info->bit_rate_value_minus1) {
		h264_hrd_param_info->bit_rate_value_minus1 = kcalloc
							(VDEC_H264_MAXIMUMVALUEOFCPB_CNT,
							 sizeof(unsigned int), GFP_KERNEL);
		VDEC_ASSERT(h264_hrd_param_info->bit_rate_value_minus1);
		if (!h264_hrd_param_info->bit_rate_value_minus1)
			return BSPP_ERROR_OUT_OF_MEMORY;
	}

	if (!h264_hrd_param_info->cpb_size_value_minus1) {
		h264_hrd_param_info->cpb_size_value_minus1 = kcalloc
							(VDEC_H264_MAXIMUMVALUEOFCPB_CNT,
							 sizeof(unsigned int),
							 GFP_KERNEL);
		VDEC_ASSERT(h264_hrd_param_info->cpb_size_value_minus1);
		if (!h264_hrd_param_info->cpb_size_value_minus1)
			return BSPP_ERROR_OUT_OF_MEMORY;
	}

	if (!h264_hrd_param_info->cbr_flag) {
		h264_hrd_param_info->cbr_flag =
			kcalloc(VDEC_H264_MAXIMUMVALUEOFCPB_CNT, sizeof(unsigned char), GFP_KERNEL);
		VDEC_ASSERT(h264_hrd_param_info->cbr_flag);
		if (!h264_hrd_param_info->cbr_flag)
			return BSPP_ERROR_OUT_OF_MEMORY;
	}

	for (sched_sel_idx = 0; sched_sel_idx <= h264_hrd_param_info->cpb_cnt_minus1;
		sched_sel_idx++) {
		h264_hrd_param_info->bit_rate_value_minus1[sched_sel_idx] =
			swsr_read_unsigned_expgoulomb(swsr_context);
		h264_hrd_param_info->cpb_size_value_minus1[sched_sel_idx] =
			swsr_read_unsigned_expgoulomb(swsr_context);

		if (h264_hrd_param_info->cpb_size_value_minus1[sched_sel_idx] == 0xffffffff)
			/* 65 bit pattern, 32 0's -1 - 32 0's then value should be 0 */
			h264_hrd_param_info->cpb_size_value_minus1[sched_sel_idx] = 0;

		h264_hrd_param_info->cbr_flag[sched_sel_idx] = swsr_read_bits(swsr_context, 1);
	}

	h264_hrd_param_info->initial_cpb_removal_delay_length_minus1 = swsr_read_bits(swsr_context,
										      5);
	h264_hrd_param_info->cpb_removal_delay_length_minus1 = swsr_read_bits(swsr_context, 5);
	h264_hrd_param_info->dpb_output_delay_length_minus1 = swsr_read_bits(swsr_context, 5);
	h264_hrd_param_info->time_offset_length = swsr_read_bits(swsr_context, 5);

	return BSPP_ERROR_NONE;
}

/*
 * @Function              bspp_h264_get_default_hrd_param
 * @Description           Get default value of the HRD parameter
 */
static void bspp_h264_get_default_hrd_param(struct bspp_h264_hrdparam_info *h264_hrd_param_info)
{
	/* other parameters already set to '0' */
	h264_hrd_param_info->initial_cpb_removal_delay_length_minus1 = 23;
	h264_hrd_param_info->cpb_removal_delay_length_minus1        = 23;
	h264_hrd_param_info->dpb_output_delay_length_minus1         = 23;
	h264_hrd_param_info->time_offset_length                   = 24;
}

/*
 * @Function              bspp_h264_vui_parser
 * @Description           Parse the VUI info
 */
static enum bspp_error_type bspp_h264_vui_parser(void *swsr_context,
						 struct bspp_h264_vui_info *vui_info,
						 struct bspp_h264_sps_info *sps_info)
{
	enum bspp_error_type vui_parser_error = BSPP_ERROR_NONE;

	vui_info->aspect_ratio_info_present_flag = swsr_read_bits(swsr_context, 1);
	if (vui_info->aspect_ratio_info_present_flag) {
		vui_info->aspect_ratio_idc = swsr_read_bits(swsr_context, 8);
		/* Extended SAR */
		if (vui_info->aspect_ratio_idc == 255) {
			vui_info->sar_width = swsr_read_bits(swsr_context, 16);
			vui_info->sar_height = swsr_read_bits(swsr_context, 16);
		} else if (vui_info->aspect_ratio_idc < 17) {
			vui_info->sar_width = pixel_aspect[vui_info->aspect_ratio_idc][0];
			vui_info->sar_height = pixel_aspect[vui_info->aspect_ratio_idc][1];
		} else {
			/* we can consider this error as a aux data error */
			vui_parser_error |= BSPP_ERROR_INVALID_VALUE;
		}
	}

	vui_info->overscan_info_present_flag = swsr_read_bits(swsr_context, 1);
	if (vui_info->overscan_info_present_flag)
		vui_info->overscan_appropriate_flag = swsr_read_bits(swsr_context, 1);

	vui_info->video_signal_type_present_flag = swsr_read_bits(swsr_context, 1);
	if (vui_info->video_signal_type_present_flag) {
		vui_info->video_format = swsr_read_bits(swsr_context, 3);
		vui_info->video_full_range_flag = swsr_read_bits(swsr_context, 1);
		vui_info->colour_description_present_flag = swsr_read_bits(swsr_context, 1);
		if (vui_info->colour_description_present_flag) {
			vui_info->colour_primaries = swsr_read_bits(swsr_context, 8);
			vui_info->transfer_characteristics = swsr_read_bits(swsr_context, 8);
			vui_info->matrix_coefficients = swsr_read_bits(swsr_context, 8);
		}
	}

	vui_info->chroma_location_info_present_flag = swsr_read_bits(swsr_context, 1);
	if (vui_info->chroma_location_info_present_flag) {
		vui_info->chroma_sample_loc_type_top_field = swsr_read_unsigned_expgoulomb
										(swsr_context);
		vui_info->chroma_sample_loc_type_bottom_field = swsr_read_unsigned_expgoulomb
										(swsr_context);
	}

	vui_info->timing_info_present_flag = swsr_read_bits(swsr_context, 1);
	if (vui_info->timing_info_present_flag) {
		vui_info->num_units_in_tick = swsr_read_bits(swsr_context, 16);
		vui_info->num_units_in_tick <<= 16;     /* SR can only do up to 31 bit reads */
		vui_info->num_units_in_tick |= swsr_read_bits(swsr_context, 16);
		vui_info->time_scale = swsr_read_bits(swsr_context, 16);
		vui_info->time_scale <<= 16;     /* SR can only do up to 31 bit reads */
		vui_info->time_scale |= swsr_read_bits(swsr_context, 16);
		if (!vui_info->num_units_in_tick || !vui_info->time_scale)
			vui_parser_error  |=  BSPP_ERROR_INVALID_VALUE;

		vui_info->fixed_frame_rate_flag = swsr_read_bits(swsr_context, 1);
	}

	/* no default values */
	vui_info->nal_hrd_parameters_present_flag = swsr_read_bits(swsr_context, 1);
	if (vui_info->nal_hrd_parameters_present_flag)
		vui_parser_error |= bspp_h264_hrd_param_parser(swsr_context,
				&vui_info->nal_hrd_parameters);
	else
		bspp_h264_get_default_hrd_param(&vui_info->nal_hrd_parameters);

	vui_info->vcl_hrd_parameters_present_flag = swsr_read_bits(swsr_context, 1);

	if (vui_info->vcl_hrd_parameters_present_flag)
		vui_parser_error |= bspp_h264_hrd_param_parser(swsr_context,
				&vui_info->vcl_hrd_parameters);
	else
		bspp_h264_get_default_hrd_param(&vui_info->vcl_hrd_parameters);

	if (vui_info->nal_hrd_parameters_present_flag || vui_info->vcl_hrd_parameters_present_flag)
		vui_info->low_delay_hrd_flag = swsr_read_bits(swsr_context, 1);

	vui_info->pic_struct_present_flag = swsr_read_bits(swsr_context, 1);
	vui_info->bitstream_restriction_flag = swsr_read_bits(swsr_context, 1);
	if (vui_info->bitstream_restriction_flag) {
		vui_info->motion_vectors_over_pic_boundaries_flag = swsr_read_bits(swsr_context, 1);
		vui_info->max_bytes_per_pic_denom = swsr_read_unsigned_expgoulomb(swsr_context);
		vui_info->max_bits_per_mb_denom = swsr_read_unsigned_expgoulomb(swsr_context);
		vui_info->log2_max_mv_length_horizontal =
			swsr_read_unsigned_expgoulomb(swsr_context);
		vui_info->log2_max_mv_length_vertical = swsr_read_unsigned_expgoulomb(swsr_context);
		vui_info->num_reorder_frames = swsr_read_unsigned_expgoulomb(swsr_context);
		vui_info->max_dec_frame_buffering = swsr_read_unsigned_expgoulomb(swsr_context);
	}

	if ((sps_info->profile_idc == h264_profile_baseline ||
	     sps_info->profile_idc == h264_profile_extended) &&
	    sps_info->max_num_ref_frames == 1) {
		vui_info->bitstream_restriction_flag = 1;
		vui_info->num_reorder_frames = 0;
		vui_info->max_dec_frame_buffering = 1;
	}

	if (vui_info->num_reorder_frames > 32)
		vui_parser_error |= BSPP_ERROR_UNSUPPORTED;

	return vui_parser_error;
}

/*
 * Parse scaling list
 */
static enum bspp_error_type bspp_h264_scl_listparser(void *swsr_context,
						     unsigned char *scaling_list,
						     unsigned char sizeof_scaling_list,
						     unsigned char *usedefaultscalingmatrixflag)
{
	enum bspp_error_type parse_error = BSPP_ERROR_NONE;
	int delta_scale;
	unsigned int lastscale = 8;
	unsigned int nextscale = 8;
	unsigned int j;

	VDEC_ASSERT(swsr_context);
	VDEC_ASSERT(scaling_list);
	VDEC_ASSERT(usedefaultscalingmatrixflag);

	if (!scaling_list || !swsr_context || !usedefaultscalingmatrixflag) {
		parse_error = BSPP_ERROR_UNRECOVERABLE;
		return parse_error;
	}

	/* 7.3.2.1.1 */
	for (j = 0; j < sizeof_scaling_list; j++) {
		if (nextscale != 0) {
			delta_scale = swsr_read_signed_expgoulomb(swsr_context);
			if ((-128 > delta_scale) || delta_scale > 127)
				parse_error |= BSPP_ERROR_INVALID_VALUE;
			nextscale = (lastscale + delta_scale + 256) & 0xff;
			*usedefaultscalingmatrixflag = (j == 0 && nextscale == 0);
		}
		scaling_list[j] = (nextscale == 0) ? lastscale : nextscale;
		lastscale = scaling_list[j];
	}
	return parse_error;
}

/*
 * Parse the SPS NAL unit
 */
static enum bspp_error_type bspp_h264_sps_parser(void *swsr_context,
						 void *str_res,
						 struct bspp_h264_seq_hdr_info *h264_seq_hdr_info)
{
	unsigned int i;
	unsigned char scaling_list_num;
	struct bspp_h264_sps_info *sps_info;
	struct bspp_h264_vui_info *vui_info;
	enum bspp_error_type sps_parser_error = BSPP_ERROR_NONE;
	enum bspp_error_type vui_parser_error = BSPP_ERROR_NONE;

	sps_info = &h264_seq_hdr_info->sps_info;
	vui_info = &h264_seq_hdr_info->vui_info;

	/* Set always the default VUI/MVCExt, their values
	 * may be used even if VUI/MVCExt not present
	 */
	bspp_h264_set_default_vui(vui_info);
#ifdef DEBUG_DECODER_DRIVER
	pr_info("Parsing Sequence Parameter Set");
#endif
	sps_info->profile_idc = swsr_read_bits(swsr_context, 8);
	if (sps_info->profile_idc != H264_PROFILE_BASELINE &&
	    sps_info->profile_idc != H264_PROFILE_MAIN &&
	    sps_info->profile_idc != H264_PROFILE_SCALABLE &&
	    sps_info->profile_idc != H264_PROFILE_EXTENDED &&
	    sps_info->profile_idc != H264_PROFILE_HIGH &&
	    sps_info->profile_idc != H264_PROFILE_HIGH10 &&
	    sps_info->profile_idc != H264_PROFILE_MVC_HIGH &&
	    sps_info->profile_idc != H264_PROFILE_HIGH422 &&
	    sps_info->profile_idc != H264_PROFILE_CAVLC444 &&
	    sps_info->profile_idc != H264_PROFILE_MVC_STEREO &&
	    sps_info->profile_idc != H264_PROFILE_HIGH444) {
		pr_err("Invalid Profile ID [%d],Parsed by BSPP", sps_info->profile_idc);
		return BSPP_ERROR_UNSUPPORTED;
	}
	sps_info->constraint_set_flags = swsr_read_bits(swsr_context, 8);
	sps_info->level_idc = swsr_read_bits(swsr_context, 8);

	/* sequence parameter set id */
	sps_info->seq_parameter_set_id = swsr_read_unsigned_expgoulomb(swsr_context);
	if (sps_info->seq_parameter_set_id >= MAX_SPS_COUNT) {
		pr_err("SPS ID [%d] goes beyond the limit", sps_info->seq_parameter_set_id);
		return BSPP_ERROR_UNSUPPORTED;
	}

	/* High profile settings */
	if (sps_info->profile_idc == H264_PROFILE_HIGH ||
	    sps_info->profile_idc == H264_PROFILE_HIGH10 ||
	    sps_info->profile_idc == H264_PROFILE_HIGH422 ||
	    sps_info->profile_idc == H264_PROFILE_HIGH444 ||
	    sps_info->profile_idc == H264_PROFILE_CAVLC444 ||
	    sps_info->profile_idc == H264_PROFILE_MVC_HIGH ||
	    sps_info->profile_idc == H264_PROFILE_MVC_STEREO) {
#ifdef DEBUG_DECODER_DRIVER
		pr_info("This is High Profile Bitstream");
#endif
		sps_info->chroma_format_idc = swsr_read_unsigned_expgoulomb(swsr_context);
		if (sps_info->chroma_format_idc > 3) {
			pr_err("chroma_format_idc[%d] is not within the range",
			       sps_info->chroma_format_idc);
			sps_parser_error |= BSPP_ERROR_INVALID_VALUE;
		}
		if (sps_info->chroma_format_idc == 3)
			sps_info->separate_colour_plane_flag = swsr_read_bits(swsr_context, 1);
		else
			sps_info->separate_colour_plane_flag = 0;

		sps_info->bit_depth_luma_minus8 = swsr_read_unsigned_expgoulomb(swsr_context);
		if (sps_info->bit_depth_luma_minus8 > 6)
			sps_parser_error |= BSPP_ERROR_INVALID_VALUE;

		sps_info->bit_depth_chroma_minus8 = swsr_read_unsigned_expgoulomb(swsr_context);
		if (sps_info->bit_depth_chroma_minus8 > 6)
			sps_parser_error |= BSPP_ERROR_INVALID_VALUE;

		sps_info->qpprime_y_zero_transform_bypass_flag = swsr_read_bits(swsr_context, 1);
		sps_info->seq_scaling_matrix_present_flag = swsr_read_bits(swsr_context, 1);
		if (sps_info->seq_scaling_matrix_present_flag) {
#ifdef DEBUG_DECODER_DRIVER
			pr_info("seq_scaling_matrix_present_flag is available");
#endif
			scaling_list_num = (sps_info->chroma_format_idc != 3) ? 8 : 12;

			if (!sps_info->scllst4x4seq) {
				sps_info->scllst4x4seq =
					kmalloc((sizeof(unsigned char[H264FW_NUM_4X4_LISTS]
							[H264FW_4X4_SIZE])), GFP_KERNEL);
				if (!sps_info->scllst4x4seq) {
					sps_parser_error |= BSPP_ERROR_OUT_OF_MEMORY;
				} else {
					VDEC_ASSERT(sps_info->scllst4x4seq);
					memset(sps_info->scllst4x4seq, 0x00,
					       sizeof(unsigned char[H264FW_NUM_4X4_LISTS]
					       [H264FW_4X4_SIZE]));
				}
			}
			if (!sps_info->scllst8x8seq) {
				sps_info->scllst8x8seq =
					kmalloc((sizeof(unsigned char[H264FW_NUM_8X8_LISTS]
							[H264FW_8X8_SIZE])), GFP_KERNEL);
				if (!sps_info->scllst8x8seq) {
					sps_parser_error |= BSPP_ERROR_OUT_OF_MEMORY;
				} else {
					VDEC_ASSERT(sps_info->scllst8x8seq);
					memset(sps_info->scllst8x8seq, 0x00,
					       sizeof(unsigned char[H264FW_NUM_8X8_LISTS]
						      [H264FW_8X8_SIZE]));
				}
			}

		{
			unsigned char(*scllst4x4seq)[H264FW_NUM_4X4_LISTS]
				[H264FW_4X4_SIZE] =
			(unsigned char (*)[H264FW_NUM_4X4_LISTS][H264FW_4X4_SIZE])
						sps_info->scllst4x4seq;
			unsigned char(*scllst8x8seq)[H264FW_NUM_8X8_LISTS]
				[H264FW_8X8_SIZE] =
				(unsigned char (*)[H264FW_NUM_8X8_LISTS]
				 [H264FW_8X8_SIZE])
				sps_info->scllst8x8seq;

			for (i = 0; i < scaling_list_num; i++) {
				unsigned char *ptr =
					&sps_info->usedefaultscalingmatrixflag_seq[i];

				sps_info->seq_scaling_list_present_flag[i] =
							swsr_read_bits(swsr_context, 1);
				if (sps_info->seq_scaling_list_present_flag[i]) {
					if (i < 6) {
						sps_parser_error |=
							bspp_h264_scl_listparser
							(swsr_context,
							 (*scllst4x4seq)[i], 16,
							 ptr);
					} else {
						sps_parser_error |=
							bspp_h264_scl_listparser
							(swsr_context,
							 (*scllst8x8seq)[i - 6], 64,
							 ptr);
					}
				}
			}
		}
		}
	} else {
		/* default values in here */
		sps_info->chroma_format_idc = 1;
		sps_info->bit_depth_luma_minus8 = 0;
		sps_info->bit_depth_chroma_minus8 = 0;
		sps_info->qpprime_y_zero_transform_bypass_flag = 0;
		sps_info->seq_scaling_matrix_present_flag = 0;
	}

	sps_info->log2_max_frame_num_minus4 = swsr_read_unsigned_expgoulomb(swsr_context);
	if (sps_info->log2_max_frame_num_minus4 > 12) {
		pr_err("log2_max_frame_num_minus4[%d] is not within range  [0 - 12]",
		       sps_info->log2_max_frame_num_minus4);
		sps_parser_error |= BSPP_ERROR_INVALID_VALUE;
	}

	sps_info->pic_order_cnt_type = swsr_read_unsigned_expgoulomb(swsr_context);
	if (sps_info->pic_order_cnt_type > 2) {
		pr_err("pic_order_cnt_type[%d] is not within range  [0 - 2]",
		       sps_info->pic_order_cnt_type);
		sps_parser_error |= BSPP_ERROR_INVALID_VALUE;
	}

	if (sps_info->pic_order_cnt_type == 0) {
		sps_info->log2_max_pic_order_cnt_lsb_minus4 = swsr_read_unsigned_expgoulomb
										(swsr_context);
		if (sps_info->log2_max_pic_order_cnt_lsb_minus4 > 12) {
			pr_err("log2_max_pic_order_cnt_lsb_minus4[%d] is not within range  [0 - 12]",
			       sps_info->log2_max_pic_order_cnt_lsb_minus4);
			sps_info->log2_max_pic_order_cnt_lsb_minus4 = 12;
			sps_parser_error |= BSPP_ERROR_CORRECTION_VALIDVALUE;
		}
	} else if (sps_info->pic_order_cnt_type == 1) {
		sps_info->delta_pic_order_always_zero_flag = swsr_read_bits(swsr_context, 1);
		sps_info->offset_for_non_ref_pic = swsr_read_signed_expgoulomb(swsr_context);
		sps_info->offset_for_top_to_bottom_field = swsr_read_signed_expgoulomb
										(swsr_context);
		sps_info->num_ref_frames_in_pic_order_cnt_cycle = swsr_read_unsigned_expgoulomb
										(swsr_context);
		if (sps_info->num_ref_frames_in_pic_order_cnt_cycle > 255) {
			pr_err("num_ref_frames_in_pic_order_cnt_cycle[%d] is not within range  [0 - 256]",
			       sps_info->num_ref_frames_in_pic_order_cnt_cycle);
			sps_parser_error |= BSPP_ERROR_INVALID_VALUE;
		}

		if (!sps_info->offset_for_ref_frame) {
			sps_info->offset_for_ref_frame =
				kmalloc((H264FW_MAX_CYCLE_REF_FRAMES * sizeof(unsigned int)),
					GFP_KERNEL);
			if (!sps_info->offset_for_ref_frame) {
				pr_err("out of memory");
				sps_parser_error |= BSPP_ERROR_OUT_OF_MEMORY;
			}
		}

		if (sps_info->offset_for_ref_frame) {
			VDEC_ASSERT(sps_info->num_ref_frames_in_pic_order_cnt_cycle <=
				    H264FW_MAX_CYCLE_REF_FRAMES);
			memset(sps_info->offset_for_ref_frame, 0x00,
			       (H264FW_MAX_CYCLE_REF_FRAMES * sizeof(unsigned int)));
			for (i = 0; i < sps_info->num_ref_frames_in_pic_order_cnt_cycle; i++) {
				/* check the max value and if it crosses then exit from the loop */
				sps_info->offset_for_ref_frame[i] = swsr_read_signed_expgoulomb
										(swsr_context);
			}
		}
	} else if (sps_info->pic_order_cnt_type != 2) {
		sps_parser_error |= BSPP_ERROR_INVALID_VALUE;
	}
	sps_info->max_num_ref_frames = swsr_read_unsigned_expgoulomb(swsr_context);

	if (sps_info->max_num_ref_frames > 16) {
		pr_err("num_ref_frames[%d] is not within range [0 - 16]",
		       sps_info->max_num_ref_frames);
		sps_parser_error |= BSPP_ERROR_INVALID_VALUE;
	}
	sps_info->gaps_in_frame_num_value_allowed_flag = swsr_read_bits(swsr_context, 1);
	sps_info->pic_width_in_mbs_minus1 = swsr_read_unsigned_expgoulomb(swsr_context);
	if (sps_info->pic_width_in_mbs_minus1 >= MAX_WIDTH_IN_MBS) {
		pr_err("pic_width_in_mbs_minus1[%d] is not within range",
		       sps_info->pic_width_in_mbs_minus1);
		sps_parser_error |= BSPP_ERROR_INVALID_VALUE;
	}
	sps_info->pic_height_in_map_units_minus1 = swsr_read_unsigned_expgoulomb(swsr_context);
	if (sps_info->pic_height_in_map_units_minus1 >= MAX_HEIGHT_IN_MBS) {
		pr_err("pic_height_in_map_units_minus1[%d] is not within range",
		       sps_info->pic_height_in_map_units_minus1);
		sps_parser_error |= BSPP_ERROR_INVALID_VALUE;
	}

	sps_info->frame_mbs_only_flag = swsr_read_bits(swsr_context, 1);
	if (!sps_info->frame_mbs_only_flag)
		sps_info->mb_adaptive_frame_field_flag = swsr_read_bits(swsr_context, 1);
	else
		sps_info->mb_adaptive_frame_field_flag = 0;

	sps_info->direct_8x8_inference_flag = swsr_read_bits(swsr_context, 1);

	sps_info->frame_cropping_flag = swsr_read_bits(swsr_context, 1);
	if (sps_info->frame_cropping_flag) {
		sps_info->frame_crop_left_offset = swsr_read_unsigned_expgoulomb(swsr_context);
		sps_info->frame_crop_right_offset = swsr_read_unsigned_expgoulomb(swsr_context);
		sps_info->frame_crop_top_offset = swsr_read_unsigned_expgoulomb(swsr_context);
		sps_info->frame_crop_bottom_offset = swsr_read_unsigned_expgoulomb(swsr_context);
	} else {
		sps_info->frame_crop_left_offset = 0;
		sps_info->frame_crop_right_offset = 0;
		sps_info->frame_crop_top_offset = 0;
		sps_info->frame_crop_bottom_offset = 0;
	}

	sps_info->vui_parameters_present_flag = swsr_read_bits(swsr_context, 1);
	/* initialise matrix_coefficients to 2 (unspecified) */
	vui_info->matrix_coefficients = 2;

	if (sps_info->vui_parameters_present_flag) {
#ifdef DEBUG_DECODER_DRIVER
		pr_info("vui_parameters_present_flag is available");
#endif
		/* save the SPS parse error in temp variable */
		vui_parser_error = bspp_h264_vui_parser(swsr_context, vui_info, sps_info);
		if (vui_parser_error != BSPP_ERROR_NONE)
			sps_parser_error  |= BSPP_ERROR_AUXDATA;

#ifdef REDUCED_DPB_NO_PIC_REORDERING
		vui_info->max_dec_frame_buffering = 1;
		vui_info->num_reorder_frames = 0;
#endif
	}

	if (sps_info->profile_idc == H264_PROFILE_MVC_HIGH ||
	    sps_info->profile_idc == H264_PROFILE_MVC_STEREO) {
		pr_err("No MVC Support for this version\n");
	}

	if (swsr_check_exception(swsr_context) != SWSR_EXCEPT_NO_EXCEPTION)
		sps_parser_error |= BSPP_ERROR_INSUFFICIENT_DATA;

	return sps_parser_error;
}

/*
 * Parse the PPS NAL unit
 */
static enum bspp_error_type bspp_h264_pps_parser(void *swsr_context,
						 void *str_res,
						 struct bspp_h264_pps_info *h264_pps_info)
{
	int i, group, chroma_format_idc;
	unsigned int number_bits_per_slicegroup_id;
	unsigned char n_scaling_list;
	unsigned char more_rbsp_data;
	unsigned int result;
	enum bspp_error_type pps_parse_error = BSPP_ERROR_NONE;

	VDEC_ASSERT(swsr_context);

	h264_pps_info->pps_id = swsr_read_unsigned_expgoulomb(swsr_context);
	if (h264_pps_info->pps_id >= MAX_PPS_COUNT) {
		pr_err("Picture Parameter Set(PPS) ID is not within the range");
		h264_pps_info->pps_id = (int)BSPP_INVALID;
		return BSPP_ERROR_UNSUPPORTED;
	}
	h264_pps_info->seq_parameter_set_id = swsr_read_unsigned_expgoulomb(swsr_context);
	if (h264_pps_info->seq_parameter_set_id >= MAX_SPS_COUNT) {
		pr_err("Sequence Parameter Set(SPS) ID is not within the range");
		h264_pps_info->seq_parameter_set_id = (int)BSPP_INVALID;
		return BSPP_ERROR_UNSUPPORTED;
	}

	{
		/*
		 * Get the chroma_format_idc from sps. Because of MVC sharing sps and subset sps ids
		 * (H.7.4.1.2.1).
		 * At this point is not clear if this pps refers to an sps or a subset sps.
		 * It should be finehowever for the case of chroma_format_idc to try and locate
		 * a subset sps if there isn't a normal one.
		 */
		struct bspp_h264_seq_hdr_info *h264_seq_hdr_info;
		struct bspp_sequence_hdr_info *seq_hdr_info;

		seq_hdr_info = bspp_get_sequ_hdr(str_res, h264_pps_info->seq_parameter_set_id);

		if (!seq_hdr_info) {
			seq_hdr_info = bspp_get_sequ_hdr(str_res,
							 h264_pps_info->seq_parameter_set_id + 32);
			if (!seq_hdr_info)
				return BSPP_ERROR_NO_SEQUENCE_HDR;
		}

		h264_seq_hdr_info =
			(struct bspp_h264_seq_hdr_info *)seq_hdr_info->secure_sequence_info;

		chroma_format_idc = h264_seq_hdr_info->sps_info.chroma_format_idc;
	}

	h264_pps_info->entropy_coding_mode_flag = swsr_read_bits(swsr_context, 1);
	h264_pps_info->pic_order_present_flag = swsr_read_bits(swsr_context, 1);
	h264_pps_info->num_slice_groups_minus1 = swsr_read_unsigned_expgoulomb(swsr_context);
	if ((h264_pps_info->num_slice_groups_minus1 + 1) >
		MAX_SLICEGROUP_COUNT) {
		h264_pps_info->num_slice_groups_minus1 =
			MAX_SLICEGROUP_COUNT - 1;
		pps_parse_error |= BSPP_ERROR_UNRECOVERABLE;
	}

	if (h264_pps_info->num_slice_groups_minus1 > 0) {
		h264_pps_info->slice_group_map_type = swsr_read_unsigned_expgoulomb(swsr_context);
		pr_err("slice_group_map_type is : %d, Parsed by BSPP",
		       h264_pps_info->slice_group_map_type);
		if (h264_pps_info->slice_group_map_type > 6) {
			pr_err("slice_group_map_type [%d] is not within the range [ 0- 6 ]",
			       h264_pps_info->slice_group_map_type);
			       pps_parse_error |= BSPP_ERROR_UNRECOVERABLE;
		}

		if (h264_pps_info->slice_group_map_type == 0) {
			for (group = 0; group <= h264_pps_info->num_slice_groups_minus1; group++) {
				h264_pps_info->run_length_minus1[group] =
					swsr_read_unsigned_expgoulomb(swsr_context);
			}
		} else if (h264_pps_info->slice_group_map_type == 2) {
			for (group = 0; group < h264_pps_info->num_slice_groups_minus1; group++) {
				h264_pps_info->top_left[group] = swsr_read_unsigned_expgoulomb
										(swsr_context);
				h264_pps_info->bottom_right[group] =
					swsr_read_unsigned_expgoulomb(swsr_context);
			}
		} else if (h264_pps_info->slice_group_map_type == 3 ||
			h264_pps_info->slice_group_map_type == 4 ||
			h264_pps_info->slice_group_map_type == 5) {
			h264_pps_info->slice_group_change_direction_flag = swsr_read_bits
									(swsr_context, 1);
			h264_pps_info->slice_group_change_rate_minus1 =
				swsr_read_unsigned_expgoulomb(swsr_context);
		} else if (h264_pps_info->slice_group_map_type == 6) {
			h264_pps_info->pic_size_in_map_unit = swsr_read_unsigned_expgoulomb
										(swsr_context);
			if (h264_pps_info->pic_size_in_map_unit >= H264_MAX_SGM_SIZE) {
				pr_err("pic_size_in_map_units_minus1 [%d] is not within the range",
				       h264_pps_info->pic_size_in_map_unit);
				pps_parse_error |= BSPP_ERROR_UNRECOVERABLE;
			}
			number_bits_per_slicegroup_id = h264ceillog2
				(h264_pps_info->num_slice_groups_minus1 + 1);

			if ((h264_pps_info->pic_size_in_map_unit + 1) >
				h264_pps_info->h264_ppssgm_info.slicegroupidnum) {
				unsigned char *slice_group_id =
					kmalloc(((h264_pps_info->pic_size_in_map_unit + 1) *
						sizeof(unsigned char)),
						GFP_KERNEL);
				if (!slice_group_id) {
					pr_err("out of memory");
					pps_parse_error |= BSPP_ERROR_OUT_OF_MEMORY;
				} else {
					pr_err("reallocating SGM info from size %lu bytes to size %lu bytes",
					       h264_pps_info->h264_ppssgm_info.slicegroupidnum *
					       sizeof(unsigned char),
					       (h264_pps_info->pic_size_in_map_unit + 1) *
					       sizeof(unsigned char));
					if (h264_pps_info->h264_ppssgm_info.slice_group_id) {
						memcpy
						(slice_group_id,
						 h264_pps_info->h264_ppssgm_info.slice_group_id,
						 h264_pps_info->h264_ppssgm_info.slicegroupidnum *
						 sizeof(unsigned char));
						kfree
						(h264_pps_info->h264_ppssgm_info.slice_group_id);
					}
					h264_pps_info->h264_ppssgm_info.slicegroupidnum =
						(h264_pps_info->pic_size_in_map_unit + 1);
					h264_pps_info->h264_ppssgm_info.slice_group_id =
						slice_group_id;
				}
			}

			VDEC_ASSERT((h264_pps_info->pic_size_in_map_unit + 1) <=
				h264_pps_info->h264_ppssgm_info.slicegroupidnum);
			for (i = 0; i <= h264_pps_info->pic_size_in_map_unit; i++)
				h264_pps_info->h264_ppssgm_info.slice_group_id[i] =
					swsr_read_bits(swsr_context, number_bits_per_slicegroup_id);
		}
	}

	for (i = 0; i < H264FW_MAX_REFPIC_LISTS; i++) {
		h264_pps_info->num_ref_idx_lx_active_minus1[i] = swsr_read_unsigned_expgoulomb
										(swsr_context);
		if (h264_pps_info->num_ref_idx_lx_active_minus1[i] >=
			SL_MAX_REF_IDX) {
			pr_err("num_ref_idx_lx_active_minus1[%d] [%d] is not within the range",
			       i, h264_pps_info->num_ref_idx_lx_active_minus1[i]);
			pps_parse_error |= BSPP_ERROR_UNRECOVERABLE;
		}
	}

	h264_pps_info->weighted_pred_flag = swsr_read_bits(swsr_context, 1);
	h264_pps_info->weighted_bipred_idc = swsr_read_bits(swsr_context, 2);
	h264_pps_info->pic_init_qp_minus26 = swsr_read_signed_expgoulomb(swsr_context);
	if (h264_pps_info->pic_init_qp_minus26 > 26)
		pr_err("pic_init_qp_minus26[%d] is not within the range [-25 , 26]",
		       h264_pps_info->pic_init_qp_minus26);

	h264_pps_info->pic_init_qs_minus26 = swsr_read_signed_expgoulomb(swsr_context);
	if (h264_pps_info->pic_init_qs_minus26 > 26)
		pr_err("pic_init_qs_minus26[%d] is not within the range [-25 , 26]",
		       h264_pps_info->pic_init_qs_minus26);

	h264_pps_info->chroma_qp_index_offset = swsr_read_signed_expgoulomb(swsr_context);
	if (h264_pps_info->chroma_qp_index_offset > H264_MAX_CHROMA_QP_INDEX_OFFSET)
		h264_pps_info->chroma_qp_index_offset = H264_MAX_CHROMA_QP_INDEX_OFFSET;

	else if (h264_pps_info->chroma_qp_index_offset < H264_MIN_CHROMA_QP_INDEX_OFFSET)
		h264_pps_info->chroma_qp_index_offset = H264_MIN_CHROMA_QP_INDEX_OFFSET;

	h264_pps_info->deblocking_filter_control_present_flag = swsr_read_bits(swsr_context, 1);
	h264_pps_info->constrained_intra_pred_flag = swsr_read_bits(swsr_context, 1);
	h264_pps_info->redundant_pic_cnt_present_flag = swsr_read_bits(swsr_context, 1);

	/* Check for more rbsp data. */
	result = swsr_check_more_rbsp_data(swsr_context, &more_rbsp_data);
	if (result == 0 && more_rbsp_data) {
#ifdef DEBUG_DECODER_DRIVER
		pr_info("More RBSP data is available");
#endif
		/* Fidelity Range Extensions Stuff */
		h264_pps_info->transform_8x8_mode_flag = swsr_read_bits(swsr_context, 1);
		h264_pps_info->pic_scaling_matrix_present_flag = swsr_read_bits(swsr_context, 1);
		if (h264_pps_info->pic_scaling_matrix_present_flag) {
			if (!h264_pps_info->scllst4x4pic) {
				h264_pps_info->scllst4x4pic =
					kmalloc((sizeof(unsigned char[H264FW_NUM_4X4_LISTS]
								[H264FW_4X4_SIZE])), GFP_KERNEL);
				if (!h264_pps_info->scllst4x4pic) {
					pps_parse_error |= BSPP_ERROR_OUT_OF_MEMORY;
				} else {
					VDEC_ASSERT(h264_pps_info->scllst4x4pic);
					memset(h264_pps_info->scllst4x4pic, 0x00,
					       sizeof(unsigned char[H264FW_NUM_4X4_LISTS]
					       [H264FW_4X4_SIZE]));
				}
			}
			if (!h264_pps_info->scllst8x8pic) {
				h264_pps_info->scllst8x8pic =
					kmalloc((sizeof(unsigned char[H264FW_NUM_8X8_LISTS]
								[H264FW_8X8_SIZE])), GFP_KERNEL);
				if (!h264_pps_info->scllst8x8pic) {
					pps_parse_error |= BSPP_ERROR_OUT_OF_MEMORY;
				} else {
					VDEC_ASSERT(h264_pps_info->scllst8x8pic);
					memset(h264_pps_info->scllst8x8pic, 0x00,
					       sizeof(unsigned char[H264FW_NUM_8X8_LISTS]
					       [H264FW_8X8_SIZE]));
				}
			}
		{
			unsigned char(*scllst4x4pic)[H264FW_NUM_4X4_LISTS][H264FW_4X4_SIZE] =
					(unsigned char (*)[H264FW_NUM_4X4_LISTS][H264FW_4X4_SIZE])
					h264_pps_info->scllst4x4pic;
			unsigned char(*scllst8x8pic)[H264FW_NUM_8X8_LISTS][H264FW_8X8_SIZE] =
					(unsigned char (*)[H264FW_NUM_8X8_LISTS][H264FW_8X8_SIZE])
					h264_pps_info->scllst8x8pic;

			/*
			 * For chroma_format =3 (YUV444) total list would be 12
			 * if transform_8x8_mode_flag is enabled else  6.
			 */
			n_scaling_list = 6 + (chroma_format_idc != 3 ? 2 : 6) *
				h264_pps_info->transform_8x8_mode_flag;
			if (n_scaling_list > 12)
				pps_parse_error |= BSPP_ERROR_UNRECOVERABLE;

			VDEC_ASSERT(h264_pps_info->scllst4x4pic);
			VDEC_ASSERT(h264_pps_info->scllst8x8pic);
			for (i = 0; i < n_scaling_list; i++) {
				unsigned char *ptr =
					&h264_pps_info->usedefaultscalingmatrixflag_pic[i];

				h264_pps_info->pic_scaling_list_present_flag[i] =
					swsr_read_bits(swsr_context, 1);
				if (h264_pps_info->pic_scaling_list_present_flag[i]) {
					if (i < 6)
						pps_parse_error |=
							bspp_h264_scl_listparser
							(swsr_context,
							 (*scllst4x4pic)[i], 16, ptr);
					else
						pps_parse_error |=
							bspp_h264_scl_listparser
							(swsr_context,
							(*scllst8x8pic)[i - 6], 64, ptr);
				}
			}
		}
		}
		h264_pps_info->second_chroma_qp_index_offset = swsr_read_signed_expgoulomb
										(swsr_context);

		if (h264_pps_info->second_chroma_qp_index_offset > H264_MAX_CHROMA_QP_INDEX_OFFSET)
			h264_pps_info->second_chroma_qp_index_offset =
				H264_MAX_CHROMA_QP_INDEX_OFFSET;
		else if (h264_pps_info->second_chroma_qp_index_offset <
			H264_MIN_CHROMA_QP_INDEX_OFFSET)
			h264_pps_info->second_chroma_qp_index_offset =
				H264_MIN_CHROMA_QP_INDEX_OFFSET;
	} else {
		h264_pps_info->second_chroma_qp_index_offset =
			h264_pps_info->chroma_qp_index_offset;
	}

	if (swsr_check_exception(swsr_context) != SWSR_EXCEPT_NO_EXCEPTION)
		pps_parse_error |= BSPP_ERROR_INSUFFICIENT_DATA;

	return pps_parse_error;
}

static int bspp_h264_release_sequ_hdr_info(void *str_alloc, void *secure_sps_info)
{
	struct bspp_h264_seq_hdr_info *h264_seq_hdr_info =
					(struct bspp_h264_seq_hdr_info *)secure_sps_info;

	if (!h264_seq_hdr_info)
		return IMG_ERROR_INVALID_PARAMETERS;

	return 0;
}

static int bspp_h264_reset_seq_hdr_info(void *secure_sps_info)
{
	struct bspp_h264_seq_hdr_info *h264_seq_hdr_info = NULL;
	unsigned int *nal_hrd_bitrate_valueminus1 = NULL;
	unsigned int *vcl_hrd_bitrate_valueminus1 = NULL;
	unsigned int *nal_hrd_cpbsize_valueminus1 = NULL;
	unsigned int *vcl_hrd_cpbsize_valueminus1 = NULL;
	unsigned char *nal_hrd_cbrflag = NULL;
	unsigned char *vcl_hrd_cbrflag = NULL;
	unsigned int *offset_for_ref_frame = NULL;
	unsigned char *scllst4x4seq = NULL;
	unsigned char *scllst8x8seq = NULL;

	if (!secure_sps_info)
		return IMG_ERROR_INVALID_PARAMETERS;

	h264_seq_hdr_info = (struct bspp_h264_seq_hdr_info *)secure_sps_info;

	offset_for_ref_frame = h264_seq_hdr_info->sps_info.offset_for_ref_frame;
	scllst4x4seq = h264_seq_hdr_info->sps_info.scllst4x4seq;
	scllst8x8seq = h264_seq_hdr_info->sps_info.scllst8x8seq;
	nal_hrd_bitrate_valueminus1 =
		h264_seq_hdr_info->vui_info.nal_hrd_parameters.bit_rate_value_minus1;
	vcl_hrd_bitrate_valueminus1 =
		h264_seq_hdr_info->vui_info.vcl_hrd_parameters.bit_rate_value_minus1;
	nal_hrd_cpbsize_valueminus1 =
		h264_seq_hdr_info->vui_info.nal_hrd_parameters.cpb_size_value_minus1;
	vcl_hrd_cpbsize_valueminus1 =
		h264_seq_hdr_info->vui_info.vcl_hrd_parameters.cpb_size_value_minus1;
	nal_hrd_cbrflag = h264_seq_hdr_info->vui_info.nal_hrd_parameters.cbr_flag;
	vcl_hrd_cbrflag = h264_seq_hdr_info->vui_info.vcl_hrd_parameters.cbr_flag;

	/* Cleaning vui_info */
	if (h264_seq_hdr_info->vui_info.nal_hrd_parameters.bit_rate_value_minus1)
		memset(h264_seq_hdr_info->vui_info.nal_hrd_parameters.bit_rate_value_minus1,
		       0x00, VDEC_H264_MAXIMUMVALUEOFCPB_CNT * sizeof(unsigned int));

	if (h264_seq_hdr_info->vui_info.nal_hrd_parameters.cpb_size_value_minus1)
		memset(h264_seq_hdr_info->vui_info.nal_hrd_parameters.cpb_size_value_minus1,
		       0x00, VDEC_H264_MAXIMUMVALUEOFCPB_CNT * sizeof(unsigned int));

	if (h264_seq_hdr_info->vui_info.vcl_hrd_parameters.cpb_size_value_minus1)
		memset(h264_seq_hdr_info->vui_info.vcl_hrd_parameters.cpb_size_value_minus1,
		       0x00, VDEC_H264_MAXIMUMVALUEOFCPB_CNT * sizeof(unsigned int));

	if (h264_seq_hdr_info->vui_info.nal_hrd_parameters.cbr_flag)
		memset(h264_seq_hdr_info->vui_info.nal_hrd_parameters.cbr_flag,
		       0x00, VDEC_H264_MAXIMUMVALUEOFCPB_CNT * sizeof(unsigned char));

	if (h264_seq_hdr_info->vui_info.vcl_hrd_parameters.cbr_flag)
		memset(h264_seq_hdr_info->vui_info.vcl_hrd_parameters.cbr_flag,
		       0x00, VDEC_H264_MAXIMUMVALUEOFCPB_CNT * sizeof(unsigned char));

	/* Cleaning sps_info */
	if (h264_seq_hdr_info->sps_info.offset_for_ref_frame)
		memset(h264_seq_hdr_info->sps_info.offset_for_ref_frame, 0x00,
		       H264FW_MAX_CYCLE_REF_FRAMES * sizeof(unsigned int));

	if (h264_seq_hdr_info->sps_info.scllst4x4seq)
		memset(h264_seq_hdr_info->sps_info.scllst4x4seq, 0x00,
		       sizeof(unsigned char[H264FW_NUM_4X4_LISTS][H264FW_4X4_SIZE]));

	if (h264_seq_hdr_info->sps_info.scllst8x8seq)
		memset(h264_seq_hdr_info->sps_info.scllst8x8seq, 0x00,
		       sizeof(unsigned char[H264FW_NUM_8X8_LISTS][H264FW_8X8_SIZE]));

	/* Erasing the structure */
	memset(h264_seq_hdr_info, 0, sizeof(*h264_seq_hdr_info));

	/* Restoring pointers */
	h264_seq_hdr_info->sps_info.offset_for_ref_frame = offset_for_ref_frame;
	h264_seq_hdr_info->sps_info.scllst4x4seq = scllst4x4seq;
	h264_seq_hdr_info->sps_info.scllst8x8seq = scllst8x8seq;

	h264_seq_hdr_info->vui_info.nal_hrd_parameters.bit_rate_value_minus1 =
		nal_hrd_bitrate_valueminus1;
	h264_seq_hdr_info->vui_info.vcl_hrd_parameters.bit_rate_value_minus1 =
		vcl_hrd_bitrate_valueminus1;

	h264_seq_hdr_info->vui_info.nal_hrd_parameters.cpb_size_value_minus1 =
		nal_hrd_cpbsize_valueminus1;
	h264_seq_hdr_info->vui_info.vcl_hrd_parameters.cpb_size_value_minus1 =
		vcl_hrd_cpbsize_valueminus1;

	h264_seq_hdr_info->vui_info.nal_hrd_parameters.cbr_flag = nal_hrd_cbrflag;
	h264_seq_hdr_info->vui_info.vcl_hrd_parameters.cbr_flag = vcl_hrd_cbrflag;

	return 0;
}

static int bspp_h264_reset_pps_info(void *secure_pps_info)
{
	struct bspp_h264_pps_info *h264_pps_info = NULL;
	unsigned short slicegroupidnum = 0;
	unsigned char *slice_group_id = NULL;
	unsigned char *scllst4x4pic = NULL;
	unsigned char *scllst8x8pic = NULL;

	if (!secure_pps_info)
		return IMG_ERROR_INVALID_PARAMETERS;

	h264_pps_info = (struct bspp_h264_pps_info *)secure_pps_info;

	/*
	 * Storing temp values (we want to leave the SGM structure
	 * it may be useful again instead of reallocating later
	 */
	slice_group_id = h264_pps_info->h264_ppssgm_info.slice_group_id;
	slicegroupidnum = h264_pps_info->h264_ppssgm_info.slicegroupidnum;
	scllst4x4pic = h264_pps_info->scllst4x4pic;
	scllst8x8pic = h264_pps_info->scllst8x8pic;

	if (h264_pps_info->h264_ppssgm_info.slice_group_id)
		memset(h264_pps_info->h264_ppssgm_info.slice_group_id, 0x00,
		       h264_pps_info->h264_ppssgm_info.slicegroupidnum * sizeof(unsigned char));

	if (h264_pps_info->scllst4x4pic)
		memset(h264_pps_info->scllst4x4pic, 0x00,
		       sizeof(unsigned char[H264FW_NUM_4X4_LISTS][H264FW_4X4_SIZE]));

	if (h264_pps_info->scllst8x8pic)
		memset(h264_pps_info->scllst8x8pic, 0x00,
		       sizeof(unsigned char[H264FW_NUM_8X8_LISTS][H264FW_8X8_SIZE]));

	/* Erasing the structure */
	memset(h264_pps_info, 0x00, sizeof(*h264_pps_info));

	/* Copy the temp variable back */
	h264_pps_info->h264_ppssgm_info.slicegroupidnum = slicegroupidnum;
	h264_pps_info->h264_ppssgm_info.slice_group_id = slice_group_id;
	h264_pps_info->scllst4x4pic = scllst4x4pic;
	h264_pps_info->scllst8x8pic = scllst8x8pic;

	return 0;
}

static enum bspp_error_type bspp_h264_pict_hdr_parser
					(void *swsr_context, void *str_res,
					 struct bspp_h264_slice_hdr_info *h264_slice_hdr_info,
					 struct bspp_pps_info **pps_info,
					 struct bspp_sequence_hdr_info **seq_hdr_info,
					 enum h264_nalunittype nal_unit_type,
					 unsigned char nal_ref_idc)
{
	enum bspp_error_type slice_parse_error = BSPP_ERROR_NONE;
	struct bspp_h264_pps_info *h264_pps_info;
	struct bspp_pps_info *pps_info_loc;
	struct bspp_h264_seq_hdr_info *h264_seq_hdr_info;
	struct bspp_sequence_hdr_info *seq_hdr_info_loc;
	int id_loc;

	VDEC_ASSERT(swsr_context);

	memset(h264_slice_hdr_info, 0, sizeof(*h264_slice_hdr_info));

	h264_slice_hdr_info->first_mb_in_slice = swsr_read_unsigned_expgoulomb(swsr_context);
	h264_slice_hdr_info->slice_type = (enum bspp_h264_slice_type)swsr_read_unsigned_expgoulomb
										(swsr_context);
	if ((unsigned int)h264_slice_hdr_info->slice_type > 9) {
		pr_err("Slice Type [%d] invalid, set to P", h264_slice_hdr_info->slice_type);
		h264_slice_hdr_info->slice_type = (enum bspp_h264_slice_type)0;
		slice_parse_error |= BSPP_ERROR_CORRECTION_VALIDVALUE;
	}
	h264_slice_hdr_info->slice_type =
		(enum bspp_h264_slice_type)(h264_slice_hdr_info->slice_type % 5);

	h264_slice_hdr_info->pps_id = swsr_read_unsigned_expgoulomb(swsr_context);
	if (h264_slice_hdr_info->pps_id >= MAX_PPS_COUNT) {
		pr_err("Picture Parameter ID [%d] invalid, set to 0", h264_slice_hdr_info->pps_id);
		h264_slice_hdr_info->pps_id = 0;
		slice_parse_error |= BSPP_ERROR_CORRECTION_VALIDVALUE;
	}

	/* Set relevant PPS and SPS */
	pps_info_loc = bspp_get_pps_hdr(str_res, h264_slice_hdr_info->pps_id);

	if (!pps_info_loc) {
		slice_parse_error |= BSPP_ERROR_NO_PPS;
		goto error;
	}
	h264_pps_info = (struct bspp_h264_pps_info *)pps_info_loc->secure_pps_info;
	if (!h264_pps_info) {
		slice_parse_error |= BSPP_ERROR_NO_PPS;
		goto error;
	}
	VDEC_ASSERT(h264_pps_info->pps_id == h264_slice_hdr_info->pps_id);
	*pps_info = pps_info_loc;

	/* seq_parameter_set_id is always in range 0-31,
	 * so we can add offset indicating subsequence header
	 */
	id_loc = h264_pps_info->seq_parameter_set_id;
	id_loc = (nal_unit_type == H264_NALTYPE_SLICE_SCALABLE ||
		nal_unit_type == H264_NALTYPE_SLICE_IDR_SCALABLE ||
		nal_unit_type == H264_NALTYPE_SUBSET_SPS) ? id_loc + 32 : id_loc;

	seq_hdr_info_loc = bspp_get_sequ_hdr(str_res, id_loc);

	if (!seq_hdr_info_loc) {
		slice_parse_error |= BSPP_ERROR_NO_SEQUENCE_HDR;
		goto error;
	}
	h264_seq_hdr_info = (struct bspp_h264_seq_hdr_info *)seq_hdr_info_loc->secure_sequence_info;
	VDEC_ASSERT((unsigned int)h264_seq_hdr_info->sps_info.seq_parameter_set_id ==
		h264_pps_info->seq_parameter_set_id);
	*seq_hdr_info = seq_hdr_info_loc;

	/*
	 * For MINIMAL parsing in secure mode, slice header parsing can stop
	 * here, may be problematic with field-coded streams and splitting
	 * fields
	 */
	if (h264_seq_hdr_info->sps_info.separate_colour_plane_flag)
		h264_slice_hdr_info->colour_plane_id = swsr_read_bits(swsr_context, 2);

	else
		h264_slice_hdr_info->colour_plane_id = 0;

	h264_slice_hdr_info->frame_num = swsr_read_bits
					(swsr_context,
					 h264_seq_hdr_info->sps_info.log2_max_frame_num_minus4
					 + 4);

	VDEC_ASSERT(h264_slice_hdr_info->frame_num <
		(1UL << (h264_seq_hdr_info->sps_info.log2_max_frame_num_minus4 + 4)));

	if (!h264_seq_hdr_info->sps_info.frame_mbs_only_flag) {
		if (h264_slice_hdr_info->slice_type == B_SLICE &&
		    !h264_seq_hdr_info->sps_info.direct_8x8_inference_flag)
			slice_parse_error |= BSPP_ERROR_INVALID_VALUE;

		h264_slice_hdr_info->field_pic_flag = swsr_read_bits(swsr_context, 1);
		if (h264_slice_hdr_info->field_pic_flag)
			h264_slice_hdr_info->bottom_field_flag = swsr_read_bits(swsr_context, 1);
		else
			h264_slice_hdr_info->bottom_field_flag = 0;
	} else {
		h264_slice_hdr_info->field_pic_flag = 0;
		h264_slice_hdr_info->bottom_field_flag = 0;
	}

	/*
	 * At this point we have everything we need, but we still lack all the
	 * conditions for detecting new pictures (needed for error cases)
	 */
	if (nal_unit_type == H264_NALTYPE_IDR_SLICE)
		h264_slice_hdr_info->idr_pic_id = swsr_read_unsigned_expgoulomb(swsr_context);

	if (h264_seq_hdr_info->sps_info.pic_order_cnt_type == 0) {
		h264_slice_hdr_info->pic_order_cnt_lsb = swsr_read_bits
				(swsr_context,
				 h264_seq_hdr_info->sps_info.log2_max_pic_order_cnt_lsb_minus4 + 4);
		if (h264_pps_info->pic_order_present_flag && !h264_slice_hdr_info->field_pic_flag)
			h264_slice_hdr_info->delta_pic_order_cnt_bottom =
							swsr_read_signed_expgoulomb(swsr_context);
	}

	if (h264_seq_hdr_info->sps_info.pic_order_cnt_type == 1 &&
	    !h264_seq_hdr_info->sps_info.delta_pic_order_always_zero_flag) {
		h264_slice_hdr_info->delta_pic_order_cnt[0] = swsr_read_signed_expgoulomb
									(swsr_context);
		if (h264_pps_info->pic_order_present_flag && !h264_slice_hdr_info->field_pic_flag)
			h264_slice_hdr_info->delta_pic_order_cnt[1] = swsr_read_signed_expgoulomb
										(swsr_context);
	}

	if (h264_pps_info->redundant_pic_cnt_present_flag)
		h264_slice_hdr_info->redundant_pic_cnt =
			swsr_read_unsigned_expgoulomb(swsr_context);

	/* For FMO streams, we need to go further */
	if (h264_pps_info->num_slice_groups_minus1 != 0 &&
	    h264_pps_info->slice_group_map_type >= 3 &&
	    h264_pps_info->slice_group_map_type <= 5) {
		if (h264_slice_hdr_info->slice_type == B_SLICE)
			swsr_read_bits(swsr_context, 1);

		if (h264_slice_hdr_info->slice_type == P_SLICE ||
		    h264_slice_hdr_info->slice_type == SP_SLICE ||
		    h264_slice_hdr_info->slice_type == B_SLICE) {
			h264_slice_hdr_info->num_ref_idx_active_override_flag =
				swsr_read_bits(swsr_context, 1);
			if (h264_slice_hdr_info->num_ref_idx_active_override_flag) {
				h264_slice_hdr_info->num_ref_idx_lx_active_minus1[0] =
					swsr_read_unsigned_expgoulomb(swsr_context);
				if (h264_slice_hdr_info->slice_type == B_SLICE)
					h264_slice_hdr_info->num_ref_idx_lx_active_minus1[1] =
						swsr_read_unsigned_expgoulomb(swsr_context);
			}
		}

		if (h264_slice_hdr_info->slice_type != SI_SLICE &&
		    h264_slice_hdr_info->slice_type != I_SLICE) {
			/* Reference picture list modification */
			/* parse reordering info and pack into commands */
			unsigned int i;
			unsigned int cmd_num, list_num;
			unsigned int command;

			i = (h264_slice_hdr_info->slice_type == B_SLICE) ? 2 : 1;

			for (list_num = 0; list_num < i; list_num++) {
				cmd_num = 0;
				if (swsr_read_bits(swsr_context, 1)) {
					do {
						command =
							swsr_read_unsigned_expgoulomb(swsr_context);
					if (command != 3) {
						swsr_read_unsigned_expgoulomb(swsr_context);
						cmd_num++;
					}
					} while (command != 3 && cmd_num <= SL_MAX_REF_IDX);
				}
			}
		}

		if ((h264_pps_info->weighted_pred_flag &&
		     h264_slice_hdr_info->slice_type == P_SLICE) ||
		    (h264_pps_info->weighted_bipred_idc &&
		     h264_slice_hdr_info->slice_type == B_SLICE)) {
			int mono_chrome;
			unsigned int list, i, j, k;

			mono_chrome = (!h264_seq_hdr_info->sps_info.chroma_format_idc) ? 1 : 0;

			swsr_read_unsigned_expgoulomb(swsr_context);
			if (!mono_chrome)
				swsr_read_unsigned_expgoulomb(swsr_context);

			k = (h264_slice_hdr_info->slice_type == B_SLICE) ? 2 : 1;

			for (list = 0; list < k; list++) {
				for (i = 0;
					i <=
					h264_slice_hdr_info->num_ref_idx_lx_active_minus1[list];
					i++) {
					if (swsr_read_bits(swsr_context, 1)) {
						swsr_read_signed_expgoulomb(swsr_context);
						swsr_read_signed_expgoulomb(swsr_context);
					}

				if (!mono_chrome && (swsr_read_bits(swsr_context, 1))) {
					for (j = 0; j < 2; j++) {
						swsr_read_signed_expgoulomb
								(swsr_context);
						swsr_read_signed_expgoulomb
								(swsr_context);
					}
				}
				}
			}
		}

		if (nal_ref_idc != 0) {
			unsigned int memmanop;

			if (nal_unit_type == H264_NALTYPE_IDR_SLICE) {
				swsr_read_bits(swsr_context, 1);
				swsr_read_bits(swsr_context, 1);
			}
			if (swsr_read_bits(swsr_context, 1)) {
				do {
					/* clamp 0--6 */
					memmanop = swsr_read_unsigned_expgoulomb
								(swsr_context);
				if (memmanop != 0 && memmanop != 5) {
					if (memmanop == 3) {
						swsr_read_unsigned_expgoulomb
							(swsr_context);
						swsr_read_unsigned_expgoulomb
							(swsr_context);
					} else {
						swsr_read_unsigned_expgoulomb
							(swsr_context);
					}
				}
				} while (memmanop != 0);
			}
		}

		if (h264_pps_info->entropy_coding_mode_flag &&
		    h264_slice_hdr_info->slice_type != I_SLICE)
			swsr_read_unsigned_expgoulomb(swsr_context);

		swsr_read_signed_expgoulomb(swsr_context);

		if (h264_slice_hdr_info->slice_type == SP_SLICE ||
		    h264_slice_hdr_info->slice_type == SI_SLICE) {
			if (h264_slice_hdr_info->slice_type == SP_SLICE)
				swsr_read_bits(swsr_context, 1);

			/* slice_qs_delta */
			swsr_read_signed_expgoulomb(swsr_context);
		}

		if (h264_pps_info->deblocking_filter_control_present_flag) {
			if (swsr_read_unsigned_expgoulomb(swsr_context) != 1) {
				swsr_read_signed_expgoulomb(swsr_context);
				swsr_read_signed_expgoulomb(swsr_context);
			}
		}

		if (h264_pps_info->slice_group_map_type >= 3 &&
		    h264_pps_info->slice_group_map_type <= 5) {
			unsigned int num_slice_group_map_units =
				(h264_seq_hdr_info->sps_info.pic_height_in_map_units_minus1 + 1) *
				(h264_seq_hdr_info->sps_info.pic_width_in_mbs_minus1 + 1);

			unsigned short slice_group_change_rate =
				(h264_pps_info->slice_group_change_rate_minus1 + 1);

			unsigned int width = h264ceillog2(num_slice_group_map_units /
					slice_group_change_rate +
					(num_slice_group_map_units % slice_group_change_rate ==
					 0 ? 0 : 1) + 1);                          /* (7-32) */
			h264_slice_hdr_info->slice_group_change_cycle = swsr_read_bits(swsr_context,
										       width);
		}
	}

error:
	return slice_parse_error;
}

static void bspp_h264_select_scaling_list(struct h264fw_picture_ps *h264fw_pps_info,
					  struct bspp_h264_pps_info *h264_pps_info,
					  struct bspp_h264_seq_hdr_info *h264_seq_hdr_info)
{
	unsigned int num8x8_lists;
	unsigned int i;
	const unsigned char *quant_matrix = NULL;
	unsigned char (*scllst4x4pic)[H264FW_NUM_4X4_LISTS][H264FW_4X4_SIZE] =
	(unsigned char (*)[H264FW_NUM_4X4_LISTS][H264FW_4X4_SIZE])h264_pps_info->scllst4x4pic;
	unsigned char (*scllst8x8pic)[H264FW_NUM_8X8_LISTS][H264FW_8X8_SIZE] =
	(unsigned char (*)[H264FW_NUM_8X8_LISTS][H264FW_8X8_SIZE])h264_pps_info->scllst8x8pic;

	unsigned char (*scllst4x4seq)[H264FW_NUM_4X4_LISTS][H264FW_4X4_SIZE] =
		(unsigned char (*)[H264FW_NUM_4X4_LISTS][H264FW_4X4_SIZE])
		h264_seq_hdr_info->sps_info.scllst4x4seq;
	unsigned char (*scllst8x8seq)[H264FW_NUM_8X8_LISTS][H264FW_8X8_SIZE] =
		(unsigned char (*)[H264FW_NUM_8X8_LISTS][H264FW_8X8_SIZE])
		h264_seq_hdr_info->sps_info.scllst8x8seq;

	if (h264_seq_hdr_info->sps_info.seq_scaling_matrix_present_flag) {
		VDEC_ASSERT(h264_seq_hdr_info->sps_info.scllst4x4seq);
		VDEC_ASSERT(h264_seq_hdr_info->sps_info.scllst8x8seq);
	}

	if (h264_pps_info->pic_scaling_matrix_present_flag) {
		for (i = 0; i < H264FW_NUM_4X4_LISTS; i++) {
			if (h264_pps_info->pic_scaling_list_present_flag[i]) {
				if (h264_pps_info->usedefaultscalingmatrixflag_pic[i])
					quant_matrix =
						(i > 2) ? default_4x4_inter : default_4x4_intra;
				else
					quant_matrix = (*scllst4x4pic)[i];

			} else {
				if (h264_seq_hdr_info->sps_info.seq_scaling_matrix_present_flag) {
					/* SPS matrix present - use fallback rule B */
					/* first 4x4 Intra list */
					if (i == 0) {
						if
				(h264_seq_hdr_info->sps_info.seq_scaling_list_present_flag[i] &&
				!h264_seq_hdr_info->sps_info.usedefaultscalingmatrixflag_seq[i]) {
							VDEC_ASSERT
							(h264_seq_hdr_info->sps_info.scllst4x4seq);
					if (scllst4x4seq)
						quant_matrix = (*scllst4x4seq)[i];
					} else {
						quant_matrix = default_4x4_intra;
					}
					}
					/* first 4x4 Inter list */
					else if (i == 3) {
						if
				(h264_seq_hdr_info->sps_info.seq_scaling_list_present_flag[i] &&
				!h264_seq_hdr_info->sps_info.usedefaultscalingmatrixflag_seq[i]) {
							VDEC_ASSERT
							(h264_seq_hdr_info->sps_info.scllst4x4seq);
					if (scllst4x4seq)
						quant_matrix = (*scllst4x4seq)[i];
					} else {
						quant_matrix = default_4x4_inter;
					}
					} else {
						quant_matrix =
							h264fw_pps_info->scalinglist4x4[i - 1];
					}
				} else {
					/* SPS matrix not present - use fallback rule A */
					/* first 4x4 Intra list */
					if (i == 0)
						quant_matrix = default_4x4_intra;
					/* first 4x4 Interlist */
					else if (i == 3)
						quant_matrix = default_4x4_inter;
					else
						quant_matrix =
							h264fw_pps_info->scalinglist4x4[i - 1];
				}
			}
			if (!quant_matrix) {
				VDEC_ASSERT(0);
				return;
			}
			/* copy correct 4x4 list to output - as selected by PPS */
			memcpy(h264fw_pps_info->scalinglist4x4[i], quant_matrix,
			       sizeof(h264fw_pps_info->scalinglist4x4[i]));
		}
	} else {
		/* PPS matrix not present, use SPS information */
		if (h264_seq_hdr_info->sps_info.seq_scaling_matrix_present_flag) {
			for (i = 0; i < H264FW_NUM_4X4_LISTS; i++) {
				if (h264_seq_hdr_info->sps_info.seq_scaling_list_present_flag[i]) {
					if
					(h264_seq_hdr_info->sps_info.usedefaultscalingmatrixflag_seq
											[i]) {
						quant_matrix = (i > 2) ? default_4x4_inter
							: default_4x4_intra;
					} else {
						VDEC_ASSERT
							(h264_seq_hdr_info->sps_info.scllst4x4seq);
					if (scllst4x4seq)
						quant_matrix = (*scllst4x4seq)[i];
					}
				} else {
					/* SPS list not present - use fallback rule A */
					/* first 4x4 Intra list */
					if (i == 0)
						quant_matrix = default_4x4_intra;
					else if (i == 3) /* first 4x4 Inter list */
						quant_matrix = default_4x4_inter;
					else
						quant_matrix =
							h264fw_pps_info->scalinglist4x4[i - 1];
				}
				if (quant_matrix) {
					/* copy correct 4x4 list to output - as selected by SPS */
					memcpy(h264fw_pps_info->scalinglist4x4[i], quant_matrix,
					       sizeof(h264fw_pps_info->scalinglist4x4[i]));
				}
			}
		} else {
			/* SPS matrix not present - use flat lists */
			quant_matrix = default_4x4_org;
			for (i = 0; i < H264FW_NUM_4X4_LISTS; i++)
				memcpy(h264fw_pps_info->scalinglist4x4[i], quant_matrix,
				       sizeof(h264fw_pps_info->scalinglist4x4[i]));
		}
	}

	/* 8x8 matrices */
	num8x8_lists = (h264_seq_hdr_info->sps_info.chroma_format_idc == 3) ? 6 : 2;
	if (h264_pps_info->transform_8x8_mode_flag) {
		unsigned char *seq_scllstflg =
			h264_seq_hdr_info->sps_info.seq_scaling_list_present_flag;
		unsigned char *def_sclmatflg_seq =
			h264_seq_hdr_info->sps_info.usedefaultscalingmatrixflag_seq;

		if (h264_pps_info->pic_scaling_matrix_present_flag) {
			for (i = 0; i < num8x8_lists; i++) {
				if (h264_pps_info->pic_scaling_list_present_flag[i +
					H264FW_NUM_4X4_LISTS]) {
					if (h264_pps_info->usedefaultscalingmatrixflag_pic[i +
							H264FW_NUM_4X4_LISTS]) {
						quant_matrix = (i & 0x1) ? default_8x8_inter
							: default_8x8_intra;
					} else {
						VDEC_ASSERT(h264_pps_info->scllst8x8pic);
					if (scllst8x8pic)
						quant_matrix = (*scllst8x8pic)[i];
					}
				} else {
					if
				(h264_seq_hdr_info->sps_info.seq_scaling_matrix_present_flag) {
						/* SPS matrix present - use fallback rule B */
						/* list 6 - first 8x8 Intra list */
				if (i == 0) {
					if (seq_scllstflg[i +
						H264FW_NUM_4X4_LISTS] &&
						!def_sclmatflg_seq[i +
						H264FW_NUM_4X4_LISTS]) {
						VDEC_ASSERT
						(h264_seq_hdr_info->sps_info.scllst8x8seq);
					if (scllst8x8seq)
						quant_matrix = (*scllst8x8seq)[i];
					} else {
						quant_matrix = default_8x8_intra;
						}
				/* list 7 - first 8x8 Inter list */
				} else if (i == 1) {
					if (seq_scllstflg[i +
							H264FW_NUM_4X4_LISTS] &&
							!def_sclmatflg_seq[i +
							H264FW_NUM_4X4_LISTS]) {
						VDEC_ASSERT
						(h264_seq_hdr_info->sps_info.scllst8x8seq);
					if (scllst8x8seq)
						quant_matrix = (*scllst8x8seq)[i];
					} else {
						quant_matrix = default_8x8_inter;
					}
					} else {
						quant_matrix =
							h264fw_pps_info->scalinglist8x8[i - 2];
					}
				} else {
						/* SPS matrix not present - use fallback rule A */
						/* list 6 - first 8x8 Intra list */
					if (i == 0)
						quant_matrix = default_8x8_intra;
					/* list 7 - first 8x8 Inter list */
					else if (i == 1)
						quant_matrix = default_8x8_inter;
					else
						quant_matrix =
							h264fw_pps_info->scalinglist8x8[i - 2];
				}
				}
				if (quant_matrix) {
					/* copy correct 8x8 list to output - as selected by PPS */
					memcpy(h264fw_pps_info->scalinglist8x8[i], quant_matrix,
					       sizeof(h264fw_pps_info->scalinglist8x8[i]));
				}
			}
		} else {
			/* PPS matrix not present, use SPS information */
			if (h264_seq_hdr_info->sps_info.seq_scaling_matrix_present_flag) {
				for (i = 0; i < num8x8_lists; i++) {
					if (seq_scllstflg[i + H264FW_NUM_4X4_LISTS] &&
					    def_sclmatflg_seq[i + H264FW_NUM_4X4_LISTS]) {
						quant_matrix =
							(i & 0x1) ? default_8x8_inter :
							default_8x8_intra;
					} else if ((seq_scllstflg[i + H264FW_NUM_4X4_LISTS]) &&
						   !(def_sclmatflg_seq[i + H264FW_NUM_4X4_LISTS])) {
						VDEC_ASSERT
							(h264_seq_hdr_info->sps_info.scllst8x8seq);
					if (scllst8x8seq)
						quant_matrix = (*scllst8x8seq)[i];
					} else if (!(seq_scllstflg[i + H264FW_NUM_4X4_LISTS]) &&
						   (i == 0)) {
						/* SPS list not present - use fallback rule A */
						/* list 6 - first 8x8 Intra list */
						quant_matrix = default_8x8_intra;
					} else if (!(seq_scllstflg[i + H264FW_NUM_4X4_LISTS]) &&
						   (i == 1)) {
						/* list 7 - first 8x8 Inter list */
						quant_matrix = default_8x8_inter;
					} else {
						quant_matrix =
							h264fw_pps_info->scalinglist8x8
							[i - 2];
					}
					if (quant_matrix) {
						/* copy correct 8x8 list to output -
						 * as selected by SPS
						 */
						memcpy(h264fw_pps_info->scalinglist8x8[i],
						       quant_matrix,
						       sizeof(h264fw_pps_info->scalinglist8x8[i]));
					}
				}
			} else {
				/* SPS matrix not present - use flat lists */
				quant_matrix = default_8x8_org;
				for (i = 0; i < num8x8_lists; i++)
					memcpy(h264fw_pps_info->scalinglist8x8[i], quant_matrix,
					       sizeof(h264fw_pps_info->scalinglist8x8[i]));
			}
		}
	}
}

static void bspp_h264_fwpps_populate(struct bspp_h264_pps_info *h264_pps_info,
				     struct h264fw_picture_ps *h264fw_pps_info)
{
	h264fw_pps_info->deblocking_filter_control_present_flag =
					h264_pps_info->deblocking_filter_control_present_flag;
	h264fw_pps_info->transform_8x8_mode_flag = h264_pps_info->transform_8x8_mode_flag;
	h264fw_pps_info->entropy_coding_mode_flag = h264_pps_info->entropy_coding_mode_flag;
	h264fw_pps_info->redundant_pic_cnt_present_flag =
						h264_pps_info->redundant_pic_cnt_present_flag;
	h264fw_pps_info->weighted_bipred_idc = h264_pps_info->weighted_bipred_idc;
	h264fw_pps_info->weighted_pred_flag = h264_pps_info->weighted_pred_flag;
	h264fw_pps_info->pic_order_present_flag = h264_pps_info->pic_order_present_flag;
	h264fw_pps_info->pic_init_qp = h264_pps_info->pic_init_qp_minus26 + 26;
	h264fw_pps_info->constrained_intra_pred_flag = h264_pps_info->constrained_intra_pred_flag;
	VDEC_ASSERT(sizeof(h264fw_pps_info->num_ref_lx_active_minus1) ==
		    sizeof(h264_pps_info->num_ref_idx_lx_active_minus1));
	VDEC_ASSERT(sizeof(h264fw_pps_info->num_ref_lx_active_minus1) ==
		    sizeof(unsigned char) * H264FW_MAX_REFPIC_LISTS);
	memcpy(h264fw_pps_info->num_ref_lx_active_minus1,
	       h264_pps_info->num_ref_idx_lx_active_minus1,
	       sizeof(h264fw_pps_info->num_ref_lx_active_minus1));
	h264fw_pps_info->slice_group_map_type = h264_pps_info->slice_group_map_type;
	h264fw_pps_info->num_slice_groups_minus1 = h264_pps_info->num_slice_groups_minus1;
	h264fw_pps_info->slice_group_change_rate_minus1 =
					h264_pps_info->slice_group_change_rate_minus1;
	h264fw_pps_info->chroma_qp_index_offset = h264_pps_info->chroma_qp_index_offset;
	h264fw_pps_info->second_chroma_qp_index_offset =
						h264_pps_info->second_chroma_qp_index_offset;
}

static void bspp_h264_fwseq_hdr_populate(struct bspp_h264_seq_hdr_info *h264_seq_hdr_info,
					 struct h264fw_sequence_ps *h264_fwseq_hdr_info)
{
	/* Basic SPS */
	h264_fwseq_hdr_info->profile_idc = h264_seq_hdr_info->sps_info.profile_idc;
	h264_fwseq_hdr_info->chroma_format_idc = h264_seq_hdr_info->sps_info.chroma_format_idc;
	h264_fwseq_hdr_info->separate_colour_plane_flag =
		h264_seq_hdr_info->sps_info.separate_colour_plane_flag;
	h264_fwseq_hdr_info->bit_depth_luma_minus8 =
		h264_seq_hdr_info->sps_info.bit_depth_luma_minus8;
	h264_fwseq_hdr_info->bit_depth_chroma_minus8 =
		h264_seq_hdr_info->sps_info.bit_depth_chroma_minus8;
	h264_fwseq_hdr_info->delta_pic_order_always_zero_flag =
		h264_seq_hdr_info->sps_info.delta_pic_order_always_zero_flag;
	h264_fwseq_hdr_info->log2_max_pic_order_cnt_lsb =
		h264_seq_hdr_info->sps_info.log2_max_pic_order_cnt_lsb_minus4 + 4;
	h264_fwseq_hdr_info->max_num_ref_frames = h264_seq_hdr_info->sps_info.max_num_ref_frames;
	h264_fwseq_hdr_info->log2_max_frame_num =
		h264_seq_hdr_info->sps_info.log2_max_frame_num_minus4 + 4;
	h264_fwseq_hdr_info->pic_order_cnt_type = h264_seq_hdr_info->sps_info.pic_order_cnt_type;
	h264_fwseq_hdr_info->frame_mbs_only_flag = h264_seq_hdr_info->sps_info.frame_mbs_only_flag;
	h264_fwseq_hdr_info->gaps_in_frame_num_value_allowed_flag =
		h264_seq_hdr_info->sps_info.gaps_in_frame_num_value_allowed_flag;
	h264_fwseq_hdr_info->constraint_set_flags =
		h264_seq_hdr_info->sps_info.constraint_set_flags;
	h264_fwseq_hdr_info->level_idc = h264_seq_hdr_info->sps_info.level_idc;
	h264_fwseq_hdr_info->num_ref_frames_in_pic_order_cnt_cycle =
		h264_seq_hdr_info->sps_info.num_ref_frames_in_pic_order_cnt_cycle;
	h264_fwseq_hdr_info->mb_adaptive_frame_field_flag =
		h264_seq_hdr_info->sps_info.mb_adaptive_frame_field_flag;
	h264_fwseq_hdr_info->offset_for_non_ref_pic =
		h264_seq_hdr_info->sps_info.offset_for_non_ref_pic;
	h264_fwseq_hdr_info->offset_for_top_to_bottom_field =
		h264_seq_hdr_info->sps_info.offset_for_top_to_bottom_field;
	h264_fwseq_hdr_info->pic_width_in_mbs_minus1 =
		h264_seq_hdr_info->sps_info.pic_width_in_mbs_minus1;
	h264_fwseq_hdr_info->pic_height_in_map_units_minus1 =
		h264_seq_hdr_info->sps_info.pic_height_in_map_units_minus1;
	h264_fwseq_hdr_info->direct_8x8_inference_flag =
		h264_seq_hdr_info->sps_info.direct_8x8_inference_flag;
	h264_fwseq_hdr_info->qpprime_y_zero_transform_bypass_flag =
		h264_seq_hdr_info->sps_info.qpprime_y_zero_transform_bypass_flag;

	if (h264_seq_hdr_info->sps_info.offset_for_ref_frame)
		memcpy(h264_fwseq_hdr_info->offset_for_ref_frame,
		       h264_seq_hdr_info->sps_info.offset_for_ref_frame,
		       sizeof(h264_fwseq_hdr_info->offset_for_ref_frame));
	else
		memset(h264_fwseq_hdr_info->offset_for_ref_frame, 0x00,
		       sizeof(h264_fwseq_hdr_info->offset_for_ref_frame));

	memset(h264_fwseq_hdr_info->anchor_inter_view_reference_id_list, 0x00,
	       sizeof(h264_fwseq_hdr_info->anchor_inter_view_reference_id_list));
	memset(h264_fwseq_hdr_info->non_anchor_inter_view_reference_id_list, 0x00,
	       sizeof(h264_fwseq_hdr_info->non_anchor_inter_view_reference_id_list));

#ifdef REDUCED_DPB_NO_PIC_REORDERING
	/* From VUI */
	h264_fwseq_hdr_info->max_dec_frame_buffering =
		h264_seq_hdr_info->vui_info.max_dec_frame_buffering;
	h264_fwseq_hdr_info->num_reorder_frames = h264_seq_hdr_info->vui_info.num_reorder_frames;
#else
	/* From VUI */
	if (h264_seq_hdr_info->vui_info.bitstream_restriction_flag) {
		VDEC_ASSERT(h264_seq_hdr_info->sps_info.vui_parameters_present_flag);
		h264_fwseq_hdr_info->max_dec_frame_buffering =
			h264_seq_hdr_info->vui_info.max_dec_frame_buffering;
		h264_fwseq_hdr_info->num_reorder_frames =
			h264_seq_hdr_info->vui_info.num_reorder_frames;
	} else {
		h264_fwseq_hdr_info->max_dec_frame_buffering = 1;
		h264_fwseq_hdr_info->num_reorder_frames = 16;
	}
#endif
}

static void bspp_h264_commonseq_hdr_populate(struct bspp_h264_seq_hdr_info *h264_seq_hdr_info,
					     struct vdec_comsequ_hdrinfo *comseq_hdr_info)
{
	struct bspp_h264_sps_info *sps_info = &h264_seq_hdr_info->sps_info;
	struct bspp_h264_vui_info *vui_info = &h264_seq_hdr_info->vui_info;

	comseq_hdr_info->codec_profile = sps_info->profile_idc;
	comseq_hdr_info->codec_level = sps_info->level_idc;

	if (sps_info->vui_parameters_present_flag && vui_info->timing_info_present_flag) {
		comseq_hdr_info->frame_rate_num = vui_info->time_scale;
		comseq_hdr_info->frame_rate_den = 2 * vui_info->num_units_in_tick;
		comseq_hdr_info->frame_rate = ((long)comseq_hdr_info->frame_rate_num) /
			((long)comseq_hdr_info->frame_rate_den);
	}

	/*
	 * ColorSpace Description was present in the VUI parameters.
	 * copy it in CommonSeqHdr info for use by application.
	 */
	if (vui_info->video_signal_type_present_flag & vui_info->colour_description_present_flag) {
		comseq_hdr_info->color_space_info.is_present = TRUE;
		comseq_hdr_info->color_space_info.color_primaries = vui_info->colour_primaries;
		comseq_hdr_info->color_space_info.transfer_characteristics =
			vui_info->transfer_characteristics;
		comseq_hdr_info->color_space_info.matrix_coefficients =
			vui_info->matrix_coefficients;
	}

	if (vui_info->aspect_ratio_info_present_flag) {
		comseq_hdr_info->aspect_ratio_num = vui_info->sar_width;
		comseq_hdr_info->aspect_ratio_den = vui_info->sar_height;
	}

	comseq_hdr_info->interlaced_frames = sps_info->frame_mbs_only_flag ? 0 : 1;

	/* pixel_info populate */
	VDEC_ASSERT(sps_info->chroma_format_idc < 4);
	comseq_hdr_info->pixel_info.chroma_fmt = (sps_info->chroma_format_idc == 0) ? 0 : 1;
	comseq_hdr_info->pixel_info.chroma_fmt_idc = pixel_format_idc[sps_info->chroma_format_idc];
	comseq_hdr_info->pixel_info.chroma_interleave =
		((sps_info->chroma_format_idc == 0) ||
		(sps_info->chroma_format_idc == 3 && sps_info->separate_colour_plane_flag)) ?
		PIXEL_INVALID_CI : PIXEL_UV_ORDER;
	comseq_hdr_info->pixel_info.num_planes =
		(sps_info->chroma_format_idc == 0) ? 1 :
		(sps_info->chroma_format_idc == 3 && sps_info->separate_colour_plane_flag) ? 3 : 2;
	comseq_hdr_info->pixel_info.bitdepth_y = sps_info->bit_depth_luma_minus8 + 8;
	comseq_hdr_info->pixel_info.bitdepth_c = sps_info->bit_depth_chroma_minus8 + 8;
	comseq_hdr_info->pixel_info.mem_pkg =
		(comseq_hdr_info->pixel_info.bitdepth_y > 8 ||
		comseq_hdr_info->pixel_info.bitdepth_c > 8) ?
		PIXEL_BIT10_MSB_MP : PIXEL_BIT8_MP;
	comseq_hdr_info->pixel_info.pixfmt =
		pixel_get_pixfmt(comseq_hdr_info->pixel_info.chroma_fmt_idc,
				 comseq_hdr_info->pixel_info.chroma_interleave,
				 comseq_hdr_info->pixel_info.mem_pkg,
				 comseq_hdr_info->pixel_info.bitdepth_y,
				 comseq_hdr_info->pixel_info.bitdepth_c,
				 comseq_hdr_info->pixel_info.num_planes);

	/* max_frame_size populate */
	comseq_hdr_info->max_frame_size.width = (sps_info->pic_width_in_mbs_minus1 + 1) * 16;
	/*
	 * H264 has always coded size MB aligned. For sequences which *may* have Field-Coded
	 * pictures, as described by the frame_mbs_only_flag, the pic_height_in_map_units_minus1
	 * refers to field height in MBs, so to find the actual Frame height we need to do
	 * Field_MBs_InHeight * 32
	 */
	comseq_hdr_info->max_frame_size.height = (sps_info->pic_height_in_map_units_minus1 + 1) *
						(sps_info->frame_mbs_only_flag ? 1 : 2) * 16;

	/* Passing 2*N to vxd_dec so that get_nbuffers can use formula N+3 for all codecs*/
	comseq_hdr_info->max_ref_frame_num  = 2 * sps_info->max_num_ref_frames;

	comseq_hdr_info->field_codec_mblocks = sps_info->mb_adaptive_frame_field_flag;
	comseq_hdr_info->min_pict_buf_num = vui_info->max_dec_frame_buffering;

	/* orig_display_region populate */
	if (sps_info->frame_cropping_flag) {
		int sub_width_c, sub_height_c, crop_unit_x, crop_unit_y;
		int frame_crop_left, frame_crop_right, frame_crop_top, frame_crop_bottom;

		sub_width_c = bspp_h264_get_subwidthc(sps_info->chroma_format_idc,
						      sps_info->separate_colour_plane_flag);

		sub_height_c = bspp_h264_get_subheightc(sps_info->chroma_format_idc,
							sps_info->separate_colour_plane_flag);

		/* equation source: ITU-T H.264 2010/03, page 77 */
		/* ChromaArrayType == 0 */
		if (sps_info->separate_colour_plane_flag || sps_info->chroma_format_idc == 0) {
			/* (7-18) */
			crop_unit_x = 1;
			/* (7-19) */
			crop_unit_y = 2 - sps_info->frame_mbs_only_flag;
			/* ChromaArrayType == chroma_format_idc */
		} else {
			/* (7-20) */
			crop_unit_x = sub_width_c;
			/* (7-21) */
			crop_unit_y = sub_height_c * (2 - sps_info->frame_mbs_only_flag);
		}

		VDEC_ASSERT(sps_info->frame_crop_left_offset <=
			(comseq_hdr_info->max_frame_size.width / crop_unit_x) -
			(sps_info->frame_crop_right_offset + 1));

		VDEC_ASSERT(sps_info->frame_crop_top_offset <=
			(comseq_hdr_info->max_frame_size.height / crop_unit_y) -
			(sps_info->frame_crop_bottom_offset + 1));
		frame_crop_left = crop_unit_x * sps_info->frame_crop_left_offset;
		frame_crop_right = comseq_hdr_info->max_frame_size.width -
			(crop_unit_x * sps_info->frame_crop_right_offset);
		frame_crop_top = crop_unit_y * sps_info->frame_crop_top_offset;
		frame_crop_bottom = comseq_hdr_info->max_frame_size.height -
			(crop_unit_y * sps_info->frame_crop_bottom_offset);
		comseq_hdr_info->orig_display_region.left_offset = (unsigned int)frame_crop_left;
		comseq_hdr_info->orig_display_region.top_offset = (unsigned int)frame_crop_top;
		comseq_hdr_info->orig_display_region.width = (frame_crop_right - frame_crop_left);
		comseq_hdr_info->orig_display_region.height = (frame_crop_bottom - frame_crop_top);
	} else {
		comseq_hdr_info->orig_display_region.left_offset = 0;
		comseq_hdr_info->orig_display_region.top_offset = 0;
		comseq_hdr_info->orig_display_region.width = comseq_hdr_info->max_frame_size.width;
		comseq_hdr_info->orig_display_region.height =
			comseq_hdr_info->max_frame_size.height;
	}

#ifdef REDUCED_DPB_NO_PIC_REORDERING
	comseq_hdr_info->max_reorder_picts = vui_info->max_dec_frame_buffering;
#else
	if (sps_info->vui_parameters_present_flag && vui_info->bitstream_restriction_flag)
		comseq_hdr_info->max_reorder_picts = vui_info->max_dec_frame_buffering;
	else
		comseq_hdr_info->max_reorder_picts = 0;
#endif
	comseq_hdr_info->separate_chroma_planes =
		h264_seq_hdr_info->sps_info.separate_colour_plane_flag ? 1 : 0;
}

static void bspp_h264_pict_hdr_populate(enum h264_nalunittype nal_unit_type,
					struct bspp_h264_slice_hdr_info *h264_slice_hdr_info,
					struct vdec_comsequ_hdrinfo *comseq_hdr_info,
					struct bspp_pict_hdr_info *pict_hdr_info)
{
	/*
	 * H264 has slice coding type, not picture. The bReference contrary to the rest of the
	 * standards is set explicitly from the NAL externally (see just below the call to
	 * bspp_h264_pict_hdr_populate) pict_hdr_info->bReference = ? (Set externally for H264)
	 */
	pict_hdr_info->intra_coded = (nal_unit_type == H264_NALTYPE_IDR_SLICE) ? 1 : 0;
	pict_hdr_info->field = h264_slice_hdr_info->field_pic_flag;

	pict_hdr_info->post_processing = 0;
	/* For H264 Maximum and Coded sizes are the same */
	pict_hdr_info->coded_frame_size.width = comseq_hdr_info->max_frame_size.width;
	/* For H264 Maximum and Coded sizes are the same */
	pict_hdr_info->coded_frame_size.height = comseq_hdr_info->max_frame_size.height;
	/*
	 * For H264 Encoded Display size has been precomputed as part of the
	 * common sequence info
	 */
	pict_hdr_info->disp_info.enc_disp_region = comseq_hdr_info->orig_display_region;
	/*
	 * For H264 there is no resampling, so encoded and actual display
	 * regions are the same
	 */
	pict_hdr_info->disp_info.disp_region = comseq_hdr_info->orig_display_region;
	/* H264 does not have that */
	pict_hdr_info->disp_info.num_pan_scan_windows = 0;
	memset(pict_hdr_info->disp_info.pan_scan_windows, 0,
	       sizeof(pict_hdr_info->disp_info.pan_scan_windows));
}

static int bspp_h264_destroy_seq_hdr_info(const void *secure_sps_info)
{
	struct bspp_h264_seq_hdr_info *h264_seq_hdr_info = NULL;

	if (!secure_sps_info)
		return IMG_ERROR_INVALID_PARAMETERS;

	h264_seq_hdr_info = (struct bspp_h264_seq_hdr_info *)secure_sps_info;

	/* Cleaning vui_info */
	kfree(h264_seq_hdr_info->vui_info.nal_hrd_parameters.bit_rate_value_minus1);
	kfree(h264_seq_hdr_info->vui_info.nal_hrd_parameters.cpb_size_value_minus1);
	kfree(h264_seq_hdr_info->vui_info.nal_hrd_parameters.cbr_flag);
	kfree(h264_seq_hdr_info->vui_info.vcl_hrd_parameters.bit_rate_value_minus1);
	kfree(h264_seq_hdr_info->vui_info.vcl_hrd_parameters.cpb_size_value_minus1);
	kfree(h264_seq_hdr_info->vui_info.vcl_hrd_parameters.cbr_flag);

	/* Cleaning sps_info */
	kfree(h264_seq_hdr_info->sps_info.offset_for_ref_frame);
	kfree(h264_seq_hdr_info->sps_info.scllst4x4seq);
	kfree(h264_seq_hdr_info->sps_info.scllst8x8seq);

	return 0;
}

static int bspp_h264_destroy_pps_info(const void *secure_pps_info)
{
	struct bspp_h264_pps_info *h264_pps_info = NULL;

	if (!secure_pps_info)
		return IMG_ERROR_INVALID_PARAMETERS;

	h264_pps_info = (struct bspp_h264_pps_info *)secure_pps_info;
	kfree(h264_pps_info->h264_ppssgm_info.slice_group_id);
	h264_pps_info->h264_ppssgm_info.slicegroupidnum = 0;
	kfree(h264_pps_info->scllst4x4pic);
	kfree(h264_pps_info->scllst8x8pic);

	return 0;
}

static int bspp_h264_destroy_data(enum bspp_unit_type data_type, void *data_handle)
{
	int result = 0;

	if (!data_handle)
		return IMG_ERROR_INVALID_PARAMETERS;

	switch (data_type) {
	case BSPP_UNIT_SEQUENCE:
		result = bspp_h264_destroy_seq_hdr_info(data_handle);
		break;
	case BSPP_UNIT_PPS:
		result = bspp_h264_destroy_pps_info(data_handle);
		break;
	default:
		break;
	}
	return result;
}

static void bspp_h264_generate_slice_groupmap(struct bspp_h264_slice_hdr_info *h264_slice_hdr_info,
					      struct bspp_h264_seq_hdr_info *h264_seq_hdr_info,
					      struct bspp_h264_pps_info *h264_pps_info,
					      unsigned char *map_unit_to_slice_groupmap,
					      unsigned int map_size)
{
	int group;
	unsigned int num_slice_group_mapunits;
	unsigned int i = 0, j, k = 0;
	unsigned char num_slice_groups = h264_pps_info->num_slice_groups_minus1 + 1;
	unsigned int pic_width_in_mbs = h264_seq_hdr_info->sps_info.pic_width_in_mbs_minus1 + 1;
	unsigned int pic_height_in_map_units =
		h264_seq_hdr_info->sps_info.pic_height_in_map_units_minus1 + 1;

	num_slice_group_mapunits = map_size;
	if (h264_pps_info->slice_group_map_type == 6) {
		if ((unsigned int)num_slice_groups != num_slice_group_mapunits) {
			VDEC_ASSERT
			("wrong pps->num_slice_group_map_units_minus1 for used SPS and FMO type 6"
					==
					NULL);
			if (num_slice_group_mapunits >
				h264_pps_info->h264_ppssgm_info.slicegroupidnum)
				num_slice_group_mapunits =
					h264_pps_info->h264_ppssgm_info.slicegroupidnum;
		}
	}

	/* only one slice group */
	if (h264_pps_info->num_slice_groups_minus1 == 0) {
		memset(map_unit_to_slice_groupmap, 0, map_size * sizeof(unsigned char));
		return;
	}
	if (h264_pps_info->num_slice_groups_minus1 >= MAX_SLICEGROUP_COUNT) {
		memset(map_unit_to_slice_groupmap, 0, map_size * sizeof(unsigned char));
		return;
	}
	if (h264_pps_info->slice_group_map_type == 0) {
		do {
			for (group =
				0;
				group <= h264_pps_info->num_slice_groups_minus1 &&
				i < num_slice_group_mapunits;
				i += h264_pps_info->run_length_minus1[group++] + 1) {
				for (j = 0;
					j <= h264_pps_info->run_length_minus1[group] &&
					i + j < num_slice_group_mapunits;
					j++)
					map_unit_to_slice_groupmap[i + j] = group;
			}
		} while (i < num_slice_group_mapunits);
	} else if (h264_pps_info->slice_group_map_type == 1) {
		for (i = 0; i < num_slice_group_mapunits; i++) {
			map_unit_to_slice_groupmap[i] = ((i % pic_width_in_mbs) +
				(((i / pic_width_in_mbs) *
				(h264_pps_info->num_slice_groups_minus1 + 1)) / 2)) %
				(h264_pps_info->num_slice_groups_minus1 + 1);
		}
	} else if (h264_pps_info->slice_group_map_type == 2) {
		unsigned int y_top_left, x_top_left, y_bottom_right, x_bottom_right, x, y;

		for (i = 0; i < num_slice_group_mapunits; i++)
			map_unit_to_slice_groupmap[i] = h264_pps_info->num_slice_groups_minus1;

		for (group = h264_pps_info->num_slice_groups_minus1 - 1; group >= 0; group--) {
			y_top_left = h264_pps_info->top_left[group] / pic_width_in_mbs;
			x_top_left = h264_pps_info->top_left[group] % pic_width_in_mbs;
			y_bottom_right = h264_pps_info->bottom_right[group] / pic_width_in_mbs;
			x_bottom_right = h264_pps_info->bottom_right[group] % pic_width_in_mbs;
			for (y = y_top_left; y <= y_bottom_right; y++)
				for (x = x_top_left; x <= x_bottom_right; x++) {
					if (h264_pps_info->top_left[group] >
						h264_pps_info->bottom_right[group] ||
						h264_pps_info->bottom_right[group] >=
						num_slice_group_mapunits)
						continue;
					map_unit_to_slice_groupmap[y * pic_width_in_mbs +
					x] = group;
				}
		}
	} else if (h264_pps_info->slice_group_map_type == 3) {
		int left_bound, top_bound, right_bound, bottom_bound;
		int x, y, x_dir, y_dir;
		int map_unit_vacant;

		unsigned int mapunits_in_slicegroup_0 =
			umin((unsigned int)((h264_pps_info->slice_group_change_rate_minus1 + 1) *
				h264_slice_hdr_info->slice_group_change_cycle),
				(unsigned int)num_slice_group_mapunits);

		for (i = 0; i < num_slice_group_mapunits; i++)
			map_unit_to_slice_groupmap[i] = 2;

		x = (pic_width_in_mbs - h264_pps_info->slice_group_change_direction_flag) / 2;
		y = (pic_height_in_map_units - h264_pps_info->slice_group_change_direction_flag) /
			2;

		left_bound = x;
		top_bound = y;
		right_bound = x;
		bottom_bound = y;

		x_dir = h264_pps_info->slice_group_change_direction_flag - 1;
		y_dir = h264_pps_info->slice_group_change_direction_flag;

		for (k = 0; k < num_slice_group_mapunits; k += map_unit_vacant) {
			map_unit_vacant =
				(map_unit_to_slice_groupmap[y * pic_width_in_mbs + x] ==
				2);
			if (map_unit_vacant)
				map_unit_to_slice_groupmap[y * pic_width_in_mbs + x] =
					(k >= mapunits_in_slicegroup_0);

			if (x_dir == -1 && x == left_bound) {
				left_bound = smax(left_bound - 1, 0);
				x = left_bound;
				x_dir = 0;
				y_dir = 2 * h264_pps_info->slice_group_change_direction_flag - 1;
			} else if (x_dir == 1 && x == right_bound) {
				right_bound = smin(right_bound + 1, (int)pic_width_in_mbs - 1);
				x = right_bound;
				x_dir = 0;
				y_dir = 1 - 2 * h264_pps_info->slice_group_change_direction_flag;
			} else if (y_dir == -1 && y == top_bound) {
				top_bound = smax(top_bound - 1, 0);
				y = top_bound;
				x_dir = 1 - 2 * h264_pps_info->slice_group_change_direction_flag;
				y_dir = 0;
			} else if (y_dir == 1 && y == bottom_bound) {
				bottom_bound = smin(bottom_bound + 1,
						    (int)pic_height_in_map_units - 1);
				y = bottom_bound;
				x_dir = 2 * h264_pps_info->slice_group_change_direction_flag - 1;
				y_dir = 0;
			} else {
				x = x + x_dir;
				y = y + y_dir;
			}
		}
	} else if (h264_pps_info->slice_group_map_type == 4) {
		unsigned int mapunits_in_slicegroup_0 =
			umin((unsigned int)((h264_pps_info->slice_group_change_rate_minus1 + 1) *
				h264_slice_hdr_info->slice_group_change_cycle),
				(unsigned int)num_slice_group_mapunits);
		unsigned int sizeof_upper_left_group =
			h264_pps_info->slice_group_change_direction_flag ?
			(num_slice_group_mapunits -
			mapunits_in_slicegroup_0) : mapunits_in_slicegroup_0;
		for (i = 0; i < num_slice_group_mapunits; i++) {
			if (i < sizeof_upper_left_group)
				map_unit_to_slice_groupmap[i] =
					h264_pps_info->slice_group_change_direction_flag;

			else
				map_unit_to_slice_groupmap[i] = 1 -
					h264_pps_info->slice_group_change_direction_flag;
		}
	} else if (h264_pps_info->slice_group_map_type == 5) {
		unsigned int mapunits_in_slicegroup_0 =
			umin((unsigned int)((h264_pps_info->slice_group_change_rate_minus1 + 1) *
				h264_slice_hdr_info->slice_group_change_cycle),
				(unsigned int)num_slice_group_mapunits);
		unsigned int sizeof_upper_left_group =
			h264_pps_info->slice_group_change_direction_flag ?
			(num_slice_group_mapunits -
			mapunits_in_slicegroup_0) : mapunits_in_slicegroup_0;

		for (j = 0; j < (unsigned int)pic_width_in_mbs; j++) {
			for (i = 0; i < (unsigned int)pic_height_in_map_units; i++) {
				if (k++ < sizeof_upper_left_group)
					map_unit_to_slice_groupmap[i * pic_width_in_mbs + j] =
						h264_pps_info->slice_group_change_direction_flag;
				else
					map_unit_to_slice_groupmap[i * pic_width_in_mbs + j] =
						1 -
						h264_pps_info->slice_group_change_direction_flag;
			}
		}
	} else if (h264_pps_info->slice_group_map_type == 6) {
		VDEC_ASSERT(num_slice_group_mapunits <=
			    h264_pps_info->h264_ppssgm_info.slicegroupidnum);
		for (i = 0; i < num_slice_group_mapunits; i++)
			map_unit_to_slice_groupmap[i] =
				h264_pps_info->h264_ppssgm_info.slice_group_id[i];
	}
}

static int bspp_h264_parse_mvc_slice_extension(void *swsr_context,
					       struct bspp_h264_inter_pict_ctx *inter_pict_ctx)
{
	if (!swsr_read_bits(swsr_context, 1)) {
		swsr_read_bits(swsr_context, 7);
		inter_pict_ctx->current_view_id = swsr_read_bits(swsr_context, 10);
		swsr_read_bits(swsr_context, 6);
		return 1;
	}

	return 0;
}

static int bspp_h264_unitparser_compile_sgmdata
			(struct bspp_h264_slice_hdr_info *h264_slice_hdr_info,
			 struct bspp_h264_seq_hdr_info *h264_seq_hdr_info,
			 struct bspp_h264_pps_info *h264_pps_info,
			 struct bspp_pict_hdr_info *pict_hdr_info)
{
	memset(&pict_hdr_info->pict_sgm_data, 0, sizeof(*&pict_hdr_info->pict_sgm_data));

	pict_hdr_info->pict_sgm_data.id = 1;

	/* Allocate memory for SGM. */
	pict_hdr_info->pict_sgm_data.size =
		(h264_seq_hdr_info->sps_info.pic_height_in_map_units_minus1 + 1) *
		(h264_seq_hdr_info->sps_info.pic_width_in_mbs_minus1 + 1);

	pict_hdr_info->pict_sgm_data.pic_data = kmalloc((pict_hdr_info->pict_sgm_data.size),
							GFP_KERNEL);
	VDEC_ASSERT(pict_hdr_info->pict_sgm_data.pic_data);
	if (!pict_hdr_info->pict_sgm_data.pic_data) {
		pict_hdr_info->pict_sgm_data.id = BSPP_INVALID;
		return IMG_ERROR_OUT_OF_MEMORY;
	}

	bspp_h264_generate_slice_groupmap(h264_slice_hdr_info, h264_seq_hdr_info, h264_pps_info,
					  pict_hdr_info->pict_sgm_data.pic_data,
					  pict_hdr_info->pict_sgm_data.size);

	/* check the discontinuous_mbs_flaginCurrFrame flag for FMO */
	/* NO FMO support */
	pict_hdr_info->discontinuous_mbs = 0;

	return 0;
}

static int bspp_h264_unit_parser(void *swsr_context, struct bspp_unit_data *unit_data)
{
	unsigned int result = 0;
	enum bspp_error_type parse_error = BSPP_ERROR_NONE;
	enum h264_nalunittype nal_unit_type;
	unsigned char nal_ref_idc;
	struct bspp_h264_inter_pict_ctx *interpicctx;
	struct bspp_sequence_hdr_info *out_seq_info;
	unsigned char id;

	interpicctx = &unit_data->parse_state->inter_pict_ctx->h264_ctx;
	out_seq_info = unit_data->out.sequ_hdr_info;

	/* At this point we should be EXACTLY at the NALTYPE byte */
	/* parse the nal header type */
	swsr_read_bits(swsr_context, 1);
	nal_ref_idc = swsr_read_bits(swsr_context, 2);
	nal_unit_type = (enum h264_nalunittype)swsr_read_bits(swsr_context, 5);

	switch (unit_data->unit_type) {
	case BSPP_UNIT_SEQUENCE:
		VDEC_ASSERT(nal_unit_type == H264_NALTYPE_SEQUENCE_PARAMETER_SET ||
			    nal_unit_type == H264_NALTYPE_SUBSET_SPS);
		{
			unsigned char id_loc;
			/* Parse SPS structure */
			struct bspp_h264_seq_hdr_info *h264_seq_hdr_info =
			(struct bspp_h264_seq_hdr_info *)(out_seq_info->secure_sequence_info);
			/* FW SPS Data structure */
			struct bspp_ddbuf_array_info *tmp = &out_seq_info->fw_sequence;
			struct h264fw_sequence_ps *h264_fwseq_hdr_info =
			(struct h264fw_sequence_ps *)((unsigned char *)tmp->ddbuf_info.cpu_virt_addr
				+ tmp->buf_offset);
			/* Common Sequence Header Info */
			struct vdec_comsequ_hdrinfo *comseq_hdr_info =
				&out_seq_info->sequ_hdr_info.com_sequ_hdr_info;

#ifdef DEBUG_DECODER_DRIVER
			pr_info("Unit Parser:Found SEQUENCE_PARAMETER_SET NAL unit");
#endif
			VDEC_ASSERT(h264_seq_hdr_info);
			VDEC_ASSERT(h264_fwseq_hdr_info);
			if (!h264_seq_hdr_info)
				return IMG_ERROR_ALREADY_COMPLETE;

			if (!h264_fwseq_hdr_info)
				return IMG_ERROR_ALREADY_COMPLETE;

			/* Call SPS parser to populate the "Parse SPS Structure" */
			unit_data->parse_error |=
				bspp_h264_sps_parser(swsr_context, unit_data->str_res_handle,
						     h264_seq_hdr_info);
			/* From "Parse SPS Structure" populate the "FW SPS Data Structure" */
			bspp_h264_fwseq_hdr_populate(h264_seq_hdr_info, h264_fwseq_hdr_info);
			/*
			 * From "Parse SPS Structure" populate the
			 * "Common Sequence Header Info"
			 */
			bspp_h264_commonseq_hdr_populate(h264_seq_hdr_info, comseq_hdr_info);
			/* Set the SPS ID */
			/*
			 * seq_parameter_set_id is always in range 0-31, so we can
			 * add offset indicating subsequence header
			 */
			id_loc = h264_seq_hdr_info->sps_info.seq_parameter_set_id;
			out_seq_info->sequ_hdr_info.sequ_hdr_id =
				(nal_unit_type == H264_NALTYPE_SLICE_SCALABLE ||
				nal_unit_type == H264_NALTYPE_SLICE_IDR_SCALABLE ||
				nal_unit_type == H264_NALTYPE_SUBSET_SPS) ? id_loc + 32 : id_loc;

			/*
			 * Set the first SPS ID as Active SPS ID for SEI parsing
			 * to cover the case of not having SeiBufferingPeriod to
			 * give us the SPS ID
			 */
			if (interpicctx->active_sps_for_sei_parsing == BSPP_INVALID)
				interpicctx->active_sps_for_sei_parsing =
					h264_seq_hdr_info->sps_info.seq_parameter_set_id;
		}
		break;

	case BSPP_UNIT_PPS:
		VDEC_ASSERT(nal_unit_type == H264_NALTYPE_PICTURE_PARAMETER_SET);
		{
			/* Parse PPS structure */
			struct bspp_h264_pps_info *h264_pps_info =
			(struct bspp_h264_pps_info *)(unit_data->out.pps_info->secure_pps_info);
			/* FW PPS Data structure */
			struct bspp_ddbuf_array_info *tmp = &unit_data->out.pps_info->fw_pps;
			struct h264fw_picture_ps *h264fw_pps_info =
				(struct h264fw_picture_ps *)((unsigned char *)
						tmp->ddbuf_info.cpu_virt_addr + tmp->buf_offset);

#ifdef DEBUG_DECODER_DRIVER
			pr_info("Unit Parser:Found PICTURE_PARAMETER_SET NAL unit");
#endif
			VDEC_ASSERT(h264_pps_info);
			VDEC_ASSERT(h264fw_pps_info);

			/* Call PPS parser to populate the "Parse PPS Structure" */
			unit_data->parse_error |=
				bspp_h264_pps_parser(swsr_context, unit_data->str_res_handle,
						     h264_pps_info);
			/* From "Parse PPS Structure" populate the "FW PPS Data Structure"
			 * - the scaling lists
			 */
			bspp_h264_fwpps_populate(h264_pps_info, h264fw_pps_info);
			/* Set the PPS ID */
			unit_data->out.pps_info->pps_id = h264_pps_info->pps_id;
		}
		break;

	case BSPP_UNIT_PICTURE:
		if (nal_unit_type == H264_NALTYPE_SLICE_PREFIX) {
			if (bspp_h264_parse_mvc_slice_extension(swsr_context, interpicctx))
				pr_err("%s: No MVC support\n", __func__);
		} else if (nal_unit_type == H264_NALTYPE_SLICE_SCALABLE ||
			nal_unit_type == H264_NALTYPE_SLICE_IDR_SCALABLE ||
			nal_unit_type == H264_NALTYPE_SLICE ||
			nal_unit_type == H264_NALTYPE_IDR_SLICE) {
			struct bspp_h264_slice_hdr_info h264_slice_hdr_info;
			struct bspp_h264_pps_info *h264_pps_info;
			struct bspp_pps_info *pps_info;
			struct h264fw_picture_ps *h264fw_pps_info;
			struct h264fw_sequence_ps *h264_fwseq_hdr_info;
			struct bspp_h264_seq_hdr_info *h264_seq_hdr_info;
			struct bspp_sequence_hdr_info *sequ_hdr_info;
			struct bspp_ddbuf_array_info *tmp1;
			struct bspp_ddbuf_array_info *tmp2;
			int current_pic_is_new = 0;
			int determined = 0;
			int id_loc;

#ifdef DEBUG_DECODER_DRIVER
			pr_info("Unit Parser:Found PICTURE DATA unit");
#endif

			unit_data->slice = 1;
			unit_data->ext_slice = 0;

			if (nal_unit_type == H264_NALTYPE_SLICE_SCALABLE ||
			    nal_unit_type == H264_NALTYPE_SLICE_IDR_SCALABLE) {
				pr_err("%s: No SVC support\n", __func__);
			}

			VDEC_ASSERT(unit_data->out.pict_hdr_info);
			if (!unit_data->out.pict_hdr_info)
				return IMG_ERROR_CANCELLED;

			/* Default */
			unit_data->out.pict_hdr_info->discontinuous_mbs = 0;

			/*
			 * Parse the Pic Header, return Parse SPS/PPS
			 * structures
			 */
			parse_error = bspp_h264_pict_hdr_parser(swsr_context,
								unit_data->str_res_handle,
								&h264_slice_hdr_info,
								&pps_info,
								&sequ_hdr_info,
								nal_unit_type,
								nal_ref_idc);

			if (parse_error) {
				unit_data->parse_error |= parse_error;
				return IMG_ERROR_CANCELLED;
			}

			/*
			 * We are signalling closed GOP at every I frame
			 * This does not conform 100% with the
			 * specification but insures that seeking always
			 * works.
			 */
			unit_data->new_closed_gop = h264_slice_hdr_info.slice_type ==
				I_SLICE ? 1 : 0;

			/*
			 * Now pps_info and sequ_hdr_info contain the
			 * PPS/SPS info related to this picture
			 */
			h264_pps_info = (struct bspp_h264_pps_info *)pps_info->secure_pps_info;
			h264_seq_hdr_info =
			(struct bspp_h264_seq_hdr_info *)sequ_hdr_info->secure_sequence_info;

			tmp1 = &pps_info->fw_pps;
			tmp2 = &sequ_hdr_info->fw_sequence;

			h264fw_pps_info = (struct h264fw_picture_ps *)((unsigned char *)
						tmp1->ddbuf_info.cpu_virt_addr + tmp1->buf_offset);
			h264_fwseq_hdr_info = (struct h264fw_sequence_ps *)((unsigned char *)
					tmp2->ddbuf_info.cpu_virt_addr + tmp2->buf_offset);
			VDEC_ASSERT(h264_slice_hdr_info.pps_id == h264_pps_info->pps_id);
			VDEC_ASSERT(h264_pps_info->seq_parameter_set_id ==
				(unsigned int)h264_seq_hdr_info->sps_info.seq_parameter_set_id);

			/*
			 * Update the decoding-related FW SPS info related to the current picture
			 * with the SEI data that were potentially received and also relate to
			 * the current info. Until we receive the picture we do not know which
			 * sequence to update with the SEI data.
			 * Setfrom last SEI, needed for decoding
			 */
			h264_fwseq_hdr_info->disable_vdmc_filt = interpicctx->disable_vdmc_filt;
			h264_fwseq_hdr_info->transform4x4_mb_not_available =
							interpicctx->b4x4transform_mb_unavailable;

			/*
			 * Determine if current slice is a new picture, and update the related
			 * params for future reference
			 * Order of checks is important
			 */
			{
				struct bspp_parse_state *state = unit_data->parse_state;

				set_if_not_determined_yet(&determined, state->new_view,
							  &current_pic_is_new, 1);
				set_if_not_determined_yet(&determined, state->next_pic_is_new,
							  &current_pic_is_new, 1);
				set_if_not_determined_yet
					(&determined,
					 (h264_slice_hdr_info.redundant_pic_cnt > 0),
					 &current_pic_is_new, 0);
				set_if_not_determined_yet
					(&determined,
					 (state->prev_frame_num !=
					  h264_slice_hdr_info.frame_num),
					 &current_pic_is_new, 1);
				set_if_not_determined_yet
					(&determined,
					 (state->prev_pps_id != h264_slice_hdr_info.pps_id),
					 &current_pic_is_new, 1);
				set_if_not_determined_yet
					(&determined,
					 (state->prev_field_pic_flag !=
					  h264_slice_hdr_info.field_pic_flag),
					 &current_pic_is_new, 1);
				set_if_not_determined_yet
					(&determined,
					 ((h264_slice_hdr_info.field_pic_flag) &&
					  (state->prev_bottom_pic_flag !=
					   h264_slice_hdr_info.bottom_field_flag)),
					 &current_pic_is_new, 1);
				set_if_not_determined_yet
					(&determined,
					 ((state->prev_nal_ref_idc == 0 || nal_ref_idc == 0) &&
					  (state->prev_nal_ref_idc != nal_ref_idc)),
					 &current_pic_is_new, 1);
				set_if_not_determined_yet
					(&determined,
					 ((h264_seq_hdr_info->sps_info.pic_order_cnt_type == 0) &&
					  ((state->prev_pic_order_cnt_lsb !=
					    h264_slice_hdr_info.pic_order_cnt_lsb) ||
					   (state->prev_delta_pic_order_cnt_bottom !=
					    h264_slice_hdr_info.delta_pic_order_cnt_bottom))),
					 &current_pic_is_new, 1);
				set_if_not_determined_yet
					(&determined,
					 ((h264_seq_hdr_info->sps_info.pic_order_cnt_type == 1) &&
					  ((state->prev_delta_pic_order_cnt[0] !=
					    h264_slice_hdr_info.delta_pic_order_cnt[0]) ||
					   (state->prev_delta_pic_order_cnt[1] !=
					    h264_slice_hdr_info.delta_pic_order_cnt[1]))),
					 &current_pic_is_new, 1);
				set_if_not_determined_yet
					(&determined,
					 ((state->prev_nal_unit_type ==
					   (int)H264_NALTYPE_IDR_SLICE ||
					   nal_unit_type == (int)H264_NALTYPE_IDR_SLICE) &&
					  (state->prev_nal_unit_type !=
					   (int)nal_unit_type)),
					 &current_pic_is_new, 1);
				set_if_not_determined_yet(&determined,
							  ((state->prev_nal_unit_type ==
							    (int)H264_NALTYPE_IDR_SLICE) &&
							   (state->prev_idr_pic_id !=
							    h264_slice_hdr_info.idr_pic_id)),
							  &current_pic_is_new, 1);

				/*
				 * Update whatever is not updated already in different places of
				 * the code or just needs to be updated here
				 */
				state->prev_frame_num = h264_slice_hdr_info.frame_num;
				state->prev_pps_id = h264_slice_hdr_info.pps_id;
				state->prev_field_pic_flag =
					h264_slice_hdr_info.field_pic_flag;
				state->prev_nal_ref_idc = nal_ref_idc;
				state->prev_pic_order_cnt_lsb =
					h264_slice_hdr_info.pic_order_cnt_lsb;
				state->prev_delta_pic_order_cnt_bottom =
					h264_slice_hdr_info.delta_pic_order_cnt_bottom;
				state->prev_delta_pic_order_cnt[0] =
					h264_slice_hdr_info.delta_pic_order_cnt[0];
				state->prev_delta_pic_order_cnt[1] =
					h264_slice_hdr_info.delta_pic_order_cnt[1];
				state->prev_nal_unit_type = (int)nal_unit_type;
				state->prev_idr_pic_id = h264_slice_hdr_info.idr_pic_id;
			}

			/* Detect second field and manage the prev_bottom_pic_flag flag */
			if (h264_slice_hdr_info.field_pic_flag && current_pic_is_new) {
				unit_data->parse_state->prev_bottom_pic_flag =
					h264_slice_hdr_info.bottom_field_flag;
			}

			/* Detect ASO    Just met new pic */
			id = h264_slice_hdr_info.colour_plane_id;
			if (current_pic_is_new) {
				unsigned int i;

				for (i = 0; i < MAX_COMPONENTS; i++)
					unit_data->parse_state->prev_first_mb_in_slice[i] = 0;
			} else if (unit_data->parse_state->prev_first_mb_in_slice[id] >
				h264_slice_hdr_info.first_mb_in_slice) {
				/* We just found ASO */
				unit_data->parse_state->discontinuous_mb = 1;
			}
			unit_data->parse_state->prev_first_mb_in_slice[id] =
				h264_slice_hdr_info.first_mb_in_slice;

			/* We may already knew we were DiscontinuousMB */
			if (unit_data->parse_state->discontinuous_mb)
				unit_data->out.pict_hdr_info->discontinuous_mbs =
					unit_data->parse_state->discontinuous_mb;

			/*
			 * We want to calculate the scaling lists only once per picture/field,
			 * not every slice We want to populate the VDEC Picture Header Info
			 * only once per picture/field, not every slice
			 */
			if (current_pic_is_new) {
				/* Common Sequence Header Info fetched */
				struct vdec_comsequ_hdrinfo *comseq_hdr_info =
					&sequ_hdr_info->sequ_hdr_info.com_sequ_hdr_info;
				struct bspp_pict_data *type_pict_aux_data;

				unit_data->parse_state->next_pic_is_new = 0;

				/* Generate SGM for this picture */
				if (h264_pps_info->num_slice_groups_minus1 != 0 &&
				    h264_pps_info->slice_group_map_type <= 6) {
					bspp_h264_unitparser_compile_sgmdata
								(&h264_slice_hdr_info,
								 h264_seq_hdr_info,
								 h264_pps_info,
								 unit_data->out.pict_hdr_info);
				} else {
					unit_data->out.pict_hdr_info->pict_sgm_data.pic_data = NULL;
					unit_data->out.pict_hdr_info->pict_sgm_data.bufmap_id = 0;
					unit_data->out.pict_hdr_info->pict_sgm_data.buf_offset = 0;
					unit_data->out.pict_hdr_info->pict_sgm_data.id =
						BSPP_INVALID;
					unit_data->out.pict_hdr_info->pict_sgm_data.size = 0;
				}

				unit_data->parse_state->discontinuous_mb =
					unit_data->out.pict_hdr_info->discontinuous_mbs;

				/*
				 * Select the scaling lists based on h264_pps_info and
				 * h264_seq_hdr_info and pass them to h264fw_pps_info
				 */
				bspp_h264_select_scaling_list(h264fw_pps_info,
							      h264_pps_info,
							      h264_seq_hdr_info);

				/*
				 * Uses the common sequence/SINGLE-slice info to populate the
				 * VDEC Picture Header Info
				 */
				bspp_h264_pict_hdr_populate(nal_unit_type, &h264_slice_hdr_info,
							    comseq_hdr_info,
							    unit_data->out.pict_hdr_info);

				/* Store some raw bitstream fields for output. */
				unit_data->out.pict_hdr_info->h264_pict_hdr_info.frame_num =
					h264_slice_hdr_info.frame_num;
				unit_data->out.pict_hdr_info->h264_pict_hdr_info.nal_ref_idc =
					nal_ref_idc;

				/*
				 * Update the display-related picture header information with
				 * the related SEI parsed data The display-related SEI is
				 * used only for the first picture after the SEI
				 */
				if (!interpicctx->sei_info_attached_to_pic) {
					interpicctx->sei_info_attached_to_pic = 1;
					if (interpicctx->active_sps_for_sei_parsing !=
						h264_seq_hdr_info->sps_info.seq_parameter_set_id) {
						/*
						 * We tried to guess the SPS ID that we should use
						 * to parse the SEI, but we guessed wrong
						 */
						pr_err("Parsed SEI with wrong SPS, data may be parsed wrong");
					}
					unit_data->out.pict_hdr_info->disp_info.repeat_first_fld =
						interpicctx->repeat_first_field;
					unit_data->out.pict_hdr_info->disp_info.max_frm_repeat =
						interpicctx->max_frm_repeat;
					/* SEI - Not supported */
				}

				/*
				 * For Idr slices update the Active
				 * Sequence ID for SEI parsing,
				 * error resilient
				 */
				if (nal_unit_type == H264_NALTYPE_IDR_SLICE)
					interpicctx->active_sps_for_sei_parsing =
						h264_seq_hdr_info->sps_info.seq_parameter_set_id;

				/*
				 * Choose the appropriate auxiliary data
				 * structure to populate.
				 */
				if (unit_data->parse_state->second_field_flag)
					type_pict_aux_data =
						&unit_data->out.pict_hdr_info->second_pict_aux_data;

				else
					type_pict_aux_data =
						&unit_data->out.pict_hdr_info->pict_aux_data;

				/*
				 * We have no container for the PPS that
				 * passes down to the kernel, for this
				 * reason the h264 secure parser needs
				 * to populate that info into the
				 * picture header (Second)PictAuxData.
				 */
				type_pict_aux_data->bufmap_id = pps_info->bufmap_id;
				type_pict_aux_data->buf_offset = pps_info->buf_offset;
				type_pict_aux_data->pic_data = (void *)h264fw_pps_info;
				type_pict_aux_data->id = h264_pps_info->pps_id;
				type_pict_aux_data->size = sizeof(struct h264fw_picture_ps);

				pps_info->ref_count++;

				/* This info comes from NAL directly */
				unit_data->out.pict_hdr_info->ref = (nal_ref_idc == 0) ? 0 : 1;
			}
			if (nal_unit_type == H264_NALTYPE_IDR_SLICE)
				unit_data->new_closed_gop = 1;

			/* Return the SPS ID */
			/*
			 * seq_parameter_set_id is always in range 0-31,
			 * so we can add offset indicating subsequence header
			 */
			id_loc = h264_pps_info->seq_parameter_set_id;
			unit_data->pict_sequ_hdr_id =
				(nal_unit_type == H264_NALTYPE_SLICE_SCALABLE ||
				nal_unit_type ==
				H264_NALTYPE_SLICE_IDR_SCALABLE) ? id_loc + 32 : id_loc;

		} else if (nal_unit_type == H264_NALTYPE_SLICE_PARTITION_A ||
			nal_unit_type == H264_NALTYPE_SLICE_PARTITION_B ||
			nal_unit_type == H264_NALTYPE_SLICE_PARTITION_C) {
			unit_data->slice = 1;

			pr_err("Unsupported Slice NAL type: %d", nal_unit_type);
			unit_data->parse_error = BSPP_ERROR_UNSUPPORTED;
		}
		break;

	case BSPP_UNIT_UNCLASSIFIED:
		if (nal_unit_type == H264_NALTYPE_ACCESS_UNIT_DELIMITER) {
			unit_data->parse_state->next_pic_is_new = 1;
		} else if (nal_unit_type == H264_NALTYPE_SLICE_PREFIX ||
			nal_unit_type == H264_NALTYPE_SUBSET_SPS) {
			/* if mvc disabled do nothing */
		} else {
			/* Should not have any other type of unclassified data. */
			pr_err("unclassified data detected!\n");
		}
		break;

	case BSPP_UNIT_NON_PICTURE:
		if (nal_unit_type == H264_NALTYPE_END_OF_SEQUENCE ||
		    nal_unit_type == H264_NALTYPE_END_OF_STREAM) {
			unit_data->parse_state->next_pic_is_new = 1;
		} else if (nal_unit_type == H264_NALTYPE_FILLER_DATA ||
			nal_unit_type == H264_NALTYPE_SEQUENCE_PARAMETER_SET_EXTENSION ||
			nal_unit_type == H264_NALTYPE_AUXILIARY_SLICE) {
		} else if (nal_unit_type == H264_NALTYPE_SLICE_SCALABLE ||
			nal_unit_type == H264_NALTYPE_SLICE_IDR_SCALABLE) {
			/* if mvc disabled do nothing */
		} else {
			/* Should not have any other type of non-picture data. */
			VDEC_ASSERT(0);
		}
		break;

	case BSPP_UNIT_UNSUPPORTED:
		pr_err("Unsupported NAL type: %d", nal_unit_type);
		unit_data->parse_error = BSPP_ERROR_UNKNOWN_DATAUNIT_DETECTED;
		break;

	default:
		VDEC_ASSERT(0);
		break;
	}

	return result;
}

static int bspp_h264releasedata(void *str_alloc, enum bspp_unit_type data_type, void *data_handle)
{
	int result = 0;

	if (!data_handle)
		return IMG_ERROR_INVALID_PARAMETERS;

	switch (data_type) {
	case BSPP_UNIT_SEQUENCE:
		result = bspp_h264_release_sequ_hdr_info(str_alloc, data_handle);
		break;
	default:
		break;
	}

	return result;
}

static int bspp_h264resetdata(enum bspp_unit_type data_type, void *data_handle)
{
	int result = 0;

	if (!data_handle)
		return IMG_ERROR_INVALID_PARAMETERS;

	switch (data_type) {
	case BSPP_UNIT_SEQUENCE:
		result = bspp_h264_reset_seq_hdr_info(data_handle);
		break;
	case BSPP_UNIT_PPS:
		result = bspp_h264_reset_pps_info(data_handle);
		break;
	default:
		break;
	}

	return result;
}

static void bspp_h264parse_codecconfig(void *swsr_ctx,
				       unsigned int *unitcount,
				       unsigned int *unit_arraycount,
				       unsigned int *delimlength,
				       unsigned int *size_delimlength)
{
	unsigned long long value = 6;

	/*
	 * Set the shift-register up to provide next 6 bytes
	 * without emulation prevention detection.
	 */
	swsr_consume_delim(swsr_ctx, SWSR_EMPREVENT_NONE, 0, &value);

	/*
	 * Codec config header must be read for size delimited data (H.264)
	 * to get to the start of each unit.
	 * This parsing follows section 5.2.4.1.1 of ISO/IEC 14496-15:2004(E).
	 */
	/* Configuration version. */
	swsr_read_bits(swsr_ctx, 8);
	/* AVC Profile Indication. */
	swsr_read_bits(swsr_ctx, 8);
	/* Profile compatibility. */
	swsr_read_bits(swsr_ctx, 8);
	/* AVC Level Indication. */
	swsr_read_bits(swsr_ctx, 8);
	*delimlength = ((swsr_read_bits(swsr_ctx, 8) & 0x3) + 1) * 8;
	*unitcount = swsr_read_bits(swsr_ctx, 8) & 0x1f;

	/* Size delimiter is only 2 bytes for H.264 codec configuration. */
	*size_delimlength = 2 * 8;
}

static void bspp_h264update_unitcounts(void *swsr_ctx,
				       unsigned int *unitcount,
				       unsigned int *unit_arraycount)
{
	if (*unitcount == 0) {
		unsigned long long value = 1;

		/*
		 * Set the shift-register up to provide next 1 byte without
		 * emulation prevention detection.
		 */
		swsr_consume_delim(swsr_ctx, SWSR_EMPREVENT_NONE, 0, &value);

		*unitcount = swsr_read_bits(swsr_ctx, 8);
	}

	(*unitcount)--;
}

/*
 * Sets the parser configuration
 */
int bspp_h264_set_parser_config(enum vdec_bstr_format bstr_format,
				struct bspp_vid_std_features *pvidstd_features,
				struct bspp_swsr_ctx *pswsr_ctx,
				struct bspp_parser_callbacks *pparser_callbacks,
				struct bspp_inter_pict_data *pinterpict_data)
{
	/* Set h.246 parser callbacks. */
	pparser_callbacks->parse_unit_cb = bspp_h264_unit_parser;
	pparser_callbacks->release_data_cb = bspp_h264releasedata;
	pparser_callbacks->reset_data_cb = bspp_h264resetdata;
	pparser_callbacks->destroy_data_cb = bspp_h264_destroy_data;
	pparser_callbacks->parse_codec_config_cb = bspp_h264parse_codecconfig;
	pparser_callbacks->update_unit_counts_cb = bspp_h264update_unitcounts;

	/* Set h.246 specific features. */
	pvidstd_features->seq_size = sizeof(struct bspp_h264_seq_hdr_info);
	pvidstd_features->uses_pps = 1;
	pvidstd_features->pps_size = sizeof(struct bspp_h264_pps_info);

	/* Set h.246 specific shift register config. */
	pswsr_ctx->emulation_prevention = SWSR_EMPREVENT_00000300;
	pinterpict_data->h264_ctx.active_sps_for_sei_parsing = BSPP_INVALID;

	if (bstr_format == VDEC_BSTRFORMAT_DEMUX_BYTESTREAM ||
	    bstr_format == VDEC_BSTRFORMAT_ELEMENTARY) {
		pswsr_ctx->sr_config.delim_type = SWSR_DELIM_SCP;
		pswsr_ctx->sr_config.delim_length = 3 * 8;
		pswsr_ctx->sr_config.scp_value = 0x000001;
	} else if (bstr_format == VDEC_BSTRFORMAT_DEMUX_SIZEDELIMITED) {
		pswsr_ctx->sr_config.delim_type = SWSR_DELIM_SIZE;
		/* Set the default size-delimiter number of bits */
		pswsr_ctx->sr_config.delim_length = 4 * 8;
	} else {
		VDEC_ASSERT(0);
		return IMG_ERROR_NOT_SUPPORTED;
	}

	return 0;
}

/*
 * This function determines the BSPP unit type based on the
 * provided bitstream (H264 specific) unit type
 */
void bspp_h264_determine_unittype(unsigned char bitstream_unittype,
				  int disable_mvc,
				  enum bspp_unit_type *bspp_unittype)
{
	unsigned char type = bitstream_unittype & 0x1f;

	switch (type) {
	case H264_NALTYPE_SLICE_PREFIX:
		*bspp_unittype = disable_mvc ? BSPP_UNIT_UNCLASSIFIED : BSPP_UNIT_PICTURE;
		break;
	case H264_NALTYPE_SUBSET_SPS:
		*bspp_unittype = disable_mvc ? BSPP_UNIT_UNCLASSIFIED : BSPP_UNIT_SEQUENCE;
		break;
	case H264_NALTYPE_SLICE_SCALABLE:
	case H264_NALTYPE_SLICE_IDR_SCALABLE:
		*bspp_unittype = disable_mvc ? BSPP_UNIT_NON_PICTURE : BSPP_UNIT_PICTURE;
		break;
	case H264_NALTYPE_SEQUENCE_PARAMETER_SET:
		*bspp_unittype = BSPP_UNIT_SEQUENCE;
		break;
	case H264_NALTYPE_PICTURE_PARAMETER_SET:
		*bspp_unittype = BSPP_UNIT_PPS;
		break;
	case H264_NALTYPE_SLICE:
	case H264_NALTYPE_SLICE_PARTITION_A:
	case H264_NALTYPE_SLICE_PARTITION_B:
	case H264_NALTYPE_SLICE_PARTITION_C:
	case H264_NALTYPE_IDR_SLICE:
		*bspp_unittype = BSPP_UNIT_PICTURE;
		break;
	case H264_NALTYPE_ACCESS_UNIT_DELIMITER:
	case H264_NALTYPE_SUPPLEMENTAL_ENHANCEMENT_INFO:
		/*
		 * Each of these NAL units should not change unit type if
		 * current is picture, since they can occur anywhere, any number
		 * of times
		 */
		*bspp_unittype = BSPP_UNIT_UNCLASSIFIED;
		break;
	case H264_NALTYPE_END_OF_SEQUENCE:
	case H264_NALTYPE_END_OF_STREAM:
	case H264_NALTYPE_FILLER_DATA:
	case H264_NALTYPE_SEQUENCE_PARAMETER_SET_EXTENSION:
	case H264_NALTYPE_AUXILIARY_SLICE:
		*bspp_unittype = BSPP_UNIT_NON_PICTURE;
		break;
	default:
		*bspp_unittype = BSPP_UNIT_UNSUPPORTED;
		break;
	}
}
