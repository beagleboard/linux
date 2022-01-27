// SPDX-License-Identifier: GPL-2.0
/*
 * encoder utility function implementations
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

#include <linux/string.h>
#include <linux/types.h>

#include "fw_headers/defs.h"
#include "img_errors.h"
#include "reg_headers/topazhp_core_regs.h"
#include "reg_headers/topaz_coreext_regs.h"
#include "reg_headers/topazhp_multicore_regs_old.h"
#include "topaz_api.h"

#define MV_OFFSET_IN_TABLE(distance, \
		position) ((distance) * MV_ROW_STRIDE + (position) * sizeof(struct img_mv_settings))
#define DEFAULT_MVCALC_CONFIG       ((0x00040303) | (MASK_TOPAZHP_CR_MVCALC_JITTER_POINTER_RST))

/*
 * Calculates the correct number of macroblocks per kick and kicks per BU
 */
void calculate_kick_and_bu_size(unsigned int width_in_mbs,
				unsigned int height_in_mbs,
				unsigned char is_interlaced,
				unsigned int max_bu_per_frame,
				unsigned int *kick_size,
				unsigned int *kicks_per_bu,
				unsigned int *min_slice_height)
{
	unsigned int kick_size_local, kicks_per_bu_local, bu_per_frame, min_slice_height_local;

	/*
	 * Basic unit is either an integer number of rows or an integer number of
	 * basic units fit in a row We calculate the ideal kick size first then decide
	 * how many kicks there will be for each basic unit
	 */

	/* Default to 1 kick per row */
	kick_size_local = width_in_mbs;
	kicks_per_bu_local = 1;
	min_slice_height_local = 1;

	/* See if we can use a smaller kick size */
	if (!(kick_size_local % 3) && kick_size_local > 30) {
		kick_size_local /= 3;
		kicks_per_bu_local = 3;
	} else if (!(kick_size_local % 2) && (kick_size_local > 20)) {
		kick_size_local /= 2;
		kicks_per_bu_local = 2;
	}

	IMG_DBG_ASSERT((kick_size_local < 256) && ("Kick Size can't be bigger than 255" != NULL));

	/* Now calculate how many kicks we do per BU */
	bu_per_frame = height_in_mbs * (is_interlaced ? 2 : 1);

	while (bu_per_frame > max_bu_per_frame) {
		/* we have too many BUs so double up the number
		 * of rows per BU so we can half the number of BUs
		 */
		kicks_per_bu_local *= 2;
		/* if we had an odd number of rows then the last BU will be half height */
		bu_per_frame = (bu_per_frame + 1) / 2;
		min_slice_height_local *= 2;
	}

	/* if we can afford to have 2 BUs per row then do it */
	if ((bu_per_frame < (max_bu_per_frame / 2)) && kicks_per_bu_local == 2) {
		kicks_per_bu_local = 1;
		bu_per_frame *= 2;
	}

	/* if we can afford to have 3 BUs per row then do it */
	if ((bu_per_frame < (max_bu_per_frame / 3)) && kicks_per_bu_local == 3) {
		kicks_per_bu_local = 1;
		bu_per_frame += 2;
	}

	*kick_size = kick_size_local;
	*kicks_per_bu = kicks_per_bu_local;
	*min_slice_height = min_slice_height_local;
}

/*
 * Calculates the stride based on the input format and width
 */
unsigned int calculate_stride(enum img_format format, ushort requested_stride_bytes, ushort width)
{
	ushort stride_bytes;

	if (requested_stride_bytes) {
		stride_bytes = requested_stride_bytes;
	} else {
		switch (format) {
		case IMG_CODEC_Y0UY1V_8888:
		case IMG_CODEC_Y0VY1U_8888:
		case IMG_CODEC_UY0VY1_8888:
		case IMG_CODEC_VY0UY1_8888:
			stride_bytes = width << 1;
			break;
		case IMG_CODEC_ABCX:
		case IMG_CODEC_XBCA:
			stride_bytes = width << 2;
			break;
		case IMG_CODEC_ABC565:
			stride_bytes = width << 1;
			break;
		default:
			stride_bytes = width;
			break;
		}
	}

	switch (format) {
	case IMG_CODEC_420_YUV:
	case IMG_CODEC_420_YV12:
	case IMG_CODEC_420_PL8:
	case IMG_CODEC_422_YUV:
	case IMG_CODEC_422_YV12:
	case IMG_CODEC_422_PL8:
	/* although luma stride is same as chroma stride,
	 * start address is half the stride. so we need 128-byte alignment
	 */
	case IMG_CODEC_420_IMC2:
	/* although luma stride is same as chroma stride,
	 * start address is half the stride. so we need 128-byte alignment
	 */
	case IMG_CODEC_422_IMC2:

		/*
		 * All strides need to be 64-byte aligned
		 * Chroma stride is half luma stride, so (luma) stride needs
		 * to be 64-byte aligned when divided by 2
		 */
		return ALIGN_128(stride_bytes);
	default:
		/* Stride needs to be 64-byte aligned */
		return ALIGN_64(stride_bytes);
	}
}

/*
 * Patch HW profile based on the profile specified by the user
 */
void patch_hw_profile(struct img_video_params *video_params, struct img_video_context *video)
{
	unsigned int ipe_control = 0;
	unsigned int pred_comb_control = 0;
	struct img_encode_features *enc_features = &video_params->enc_features;

	/* disable_intra4x4 */
	if (enc_features->disable_intra4x4)
		pred_comb_control |= F_ENCODE(1, TOPAZHP_CR_INTRA4X4_DISABLE);

	/* disable_intra8x8 */
	if (enc_features->disable_intra8x8)
		pred_comb_control |= F_ENCODE(1, TOPAZHP_CR_INTRA8X8_DISABLE);

	/* disable_intra16x16, check if at least one of the other Intra mode is enabled */
	if (enc_features->disable_intra16x16 &&
	    (!(enc_features->disable_intra8x8) || !(enc_features->disable_intra4x4)))
		pred_comb_control |= F_ENCODE(1, TOPAZHP_CR_INTRA16X16_DISABLE);

	if (video_params->mbps)
		video->mbps = video_params->mbps;

	if (enc_features->restrict_inter4x4)
		ipe_control |= F_ENCODE(1, TOPAZHP_CR_IPE_MV_NUMBER_RESTRICTION);

	if (enc_features->disable_inter8x8)
		pred_comb_control |= F_ENCODE(1, TOPAZHP_CR_INTER8X8_DISABLE);

	if (enc_features->disable_bpic_ref1)
		pred_comb_control |= F_ENCODE(1, TOPAZHP_CR_B_PIC1_DISABLE);
	else if (enc_features->disable_bpic_ref0)
		pred_comb_control |= F_ENCODE(1, TOPAZHP_CR_B_PIC0_DISABLE);

	/* save predictor combiner control in video encode parameter set */
	video->pred_comb_control = pred_comb_control;

	/* set blocksize */
	ipe_control |= F_ENCODE(enc_features->min_blk_sz, TOPAZHP_CR_IPE_BLOCKSIZE);

	if (enc_features->enable_8x16_mv_detect)
		ipe_control |= F_ENCODE(1, TOPAZHP_CR_IPE_8X16_ENABLE);

	if (enc_features->enable_16x8_mv_detect)
		ipe_control |= F_ENCODE(1, TOPAZHP_CR_IPE_16X8_ENABLE);

	if (enc_features->disable_bframes)
		video->rc_params.bframes = 0;

	if (enc_features->restricted_intra_pred)
		video->intra_pred_modes = 0xff0f;

	/* save IPE-control register */
	video->ipe_control = ipe_control;
}

/*
 * Set offsets and strides for YUV components of source picture
 */
int topaz_set_component_offsets(void *enc_ctx_handle, struct img_frame *frame)
{
	struct img_enc_context *enc;
	struct img_video_context *video;
	enum img_format format;
	ushort stride_bytes;
	ushort picture_height;

	if (!enc_ctx_handle)
		return IMG_ERROR_INVALID_CONTEXT;

	/* if source slot is NULL then it's just a next portion of slices */
	if (!frame)
		return IMG_ERROR_UNDEFINED;

	enc = (struct img_enc_context *)enc_ctx_handle;
	video = enc->video;

	format = video->format;
	picture_height = video->buffer_height >> (video->is_interlaced ? 1 : 0);
	stride_bytes = video->buffer_stride_bytes;

	/*
	 * 3 Components: Y, U, V
	 * Y component is always at the beginning
	 */
	frame->y_component_offset = 0;
	frame->src_y_stride_bytes = stride_bytes;

	/* Assume for now that field 0 comes first */
	frame->field0_y_offset = 0;
	frame->field0_u_offset = 0;
	frame->field0_v_offset = 0;

	switch (format) {
	case IMG_CODEC_420_YUV:
		frame->src_uv_stride_bytes = stride_bytes / 2;

		frame->u_component_offset = stride_bytes * picture_height;
		frame->v_component_offset = stride_bytes * picture_height + (stride_bytes / 2) *
			(picture_height / 2);
		break;

	case IMG_CODEC_420_PL8:
		frame->src_uv_stride_bytes = stride_bytes / 2;

		frame->u_component_offset = 0;
		frame->v_component_offset = 0;
		break;

	case IMG_CODEC_420_PL12:
	case IMG_CODEC_420_PL21:
		frame->src_uv_stride_bytes = stride_bytes;

		frame->u_component_offset = 0;
		frame->v_component_offset = 0;
		break;

	case IMG_CODEC_420_YV12:
		frame->src_uv_stride_bytes = stride_bytes / 2;
		frame->u_component_offset = stride_bytes * picture_height + (stride_bytes / 2) *
			(picture_height / 2);
		frame->v_component_offset = stride_bytes * picture_height;
		break;

	case IMG_CODEC_420_PL12_PACKED:
	case IMG_CODEC_420_PL21_PACKED:
		frame->src_uv_stride_bytes = stride_bytes;
		frame->u_component_offset = stride_bytes * picture_height;
		frame->v_component_offset = stride_bytes * picture_height;
		break;

	case IMG_CODEC_420_IMC2:        /* IMC2 */
		frame->src_uv_stride_bytes = stride_bytes;
		frame->u_component_offset = stride_bytes * picture_height + (stride_bytes / 2);
		frame->v_component_offset = stride_bytes * picture_height;
		break;

	case IMG_CODEC_422_YUV:
		frame->src_uv_stride_bytes = stride_bytes / 2;
		frame->u_component_offset = stride_bytes * picture_height;
		frame->v_component_offset = stride_bytes * picture_height + (stride_bytes / 2) *
			picture_height;
		break;

	case IMG_CODEC_422_YV12:        /* YV16 */
		frame->src_uv_stride_bytes = stride_bytes / 2;
		frame->u_component_offset = stride_bytes * picture_height + (stride_bytes / 2) *
			picture_height;
		frame->v_component_offset = stride_bytes * picture_height;
		break;

	case IMG_CODEC_422_PL8:
		frame->src_uv_stride_bytes = stride_bytes / 2;
		frame->u_component_offset = 0;
		frame->v_component_offset = 0;
		break;

	case IMG_CODEC_422_IMC2:        /* IMC2 */
		frame->src_uv_stride_bytes = stride_bytes;
		frame->u_component_offset = stride_bytes * picture_height + (stride_bytes / 2);
		frame->v_component_offset = stride_bytes * picture_height;
		break;

	case IMG_CODEC_422_PL12:
	case IMG_CODEC_422_PL21:
		frame->src_uv_stride_bytes = stride_bytes;
		frame->u_component_offset = 0;
		frame->v_component_offset = 0;
		break;

	case IMG_CODEC_444_YUV:
		frame->src_uv_stride_bytes = stride_bytes;
		frame->u_component_offset = stride_bytes * picture_height;
		frame->v_component_offset = stride_bytes * picture_height + stride_bytes *
			picture_height;
		break;

	case IMG_CODEC_444_YV12:        /* YV16 */
		frame->src_uv_stride_bytes = stride_bytes;
		frame->u_component_offset = stride_bytes * picture_height + stride_bytes *
			picture_height;
		frame->v_component_offset = stride_bytes * picture_height;
		break;

	case IMG_CODEC_444_PL8:
		frame->src_uv_stride_bytes = stride_bytes;
		frame->u_component_offset = 0;
		frame->v_component_offset = 0;
		break;

	case IMG_CODEC_444_IMC2:        /* IMC2 */
		frame->src_uv_stride_bytes = stride_bytes * 2;
		frame->u_component_offset = stride_bytes * picture_height + stride_bytes;
		frame->v_component_offset = stride_bytes * picture_height;
		break;

	case IMG_CODEC_444_PL12:
	case IMG_CODEC_444_PL21:
		frame->src_uv_stride_bytes = stride_bytes * 2;
		frame->u_component_offset = 0;
		frame->v_component_offset = 0;
		break;

	case IMG_CODEC_Y0UY1V_8888:
	case IMG_CODEC_Y0VY1U_8888:
	case IMG_CODEC_UY0VY1_8888:
	case IMG_CODEC_VY0UY1_8888:
	case IMG_CODEC_ABCX:
	case IMG_CODEC_XBCA:
	case IMG_CODEC_ABC565:
		frame->src_uv_stride_bytes = stride_bytes;
		frame->u_component_offset = 0;
		frame->v_component_offset = 0;
		break;

	default:
		break;
	}

	if (video->is_interlaced) {
		if (video->is_interleaved) {
			switch (format) {
			case IMG_CODEC_420_IMC2:
			case IMG_CODEC_422_IMC2:
				frame->v_component_offset *= 2;
				frame->u_component_offset = frame->v_component_offset +
					(stride_bytes / 2);
				break;
			case IMG_CODEC_444_IMC2:
				frame->v_component_offset *= 2;
				frame->u_component_offset = frame->v_component_offset +
					stride_bytes;
				break;

			default:
				frame->u_component_offset *= 2;
				frame->v_component_offset *= 2;
				break;
			}

			frame->field1_y_offset = frame->field0_y_offset + frame->src_y_stride_bytes;
			frame->field1_u_offset = frame->field0_u_offset +
				frame->src_uv_stride_bytes;
			frame->field1_v_offset = frame->field0_v_offset +
				frame->src_uv_stride_bytes;

			frame->src_y_stride_bytes *= 2;
			frame->src_uv_stride_bytes *= 2;
		} else {
			unsigned int y_field_size, c_field_size;

			switch (format) {
			case IMG_CODEC_420_YUV:
			case IMG_CODEC_420_YV12:
			case IMG_CODEC_420_IMC2:
			case IMG_CODEC_420_PL12_PACKED:
			case IMG_CODEC_420_PL21_PACKED:
				/* In Packed formats including PL12 packed the field offsets
				 * should be calculated in the following manner
				 */
				y_field_size = picture_height * stride_bytes * 3 / 2;
				c_field_size = y_field_size;
				break;
			case IMG_CODEC_420_PL8:
				y_field_size = picture_height * stride_bytes;
				c_field_size = picture_height * stride_bytes / 4;
				break;
			case IMG_CODEC_420_PL12:
			case IMG_CODEC_420_PL21:
				y_field_size = picture_height * stride_bytes;
				c_field_size = picture_height * stride_bytes / 2;
				break;
			case IMG_CODEC_422_YUV:
			case IMG_CODEC_422_YV12:
			case IMG_CODEC_422_IMC2:
				y_field_size = picture_height * stride_bytes * 2;
				c_field_size = y_field_size;
				break;
			case IMG_CODEC_422_PL8:
				y_field_size = picture_height * stride_bytes;
				c_field_size = picture_height * stride_bytes / 2;
				break;
			case IMG_CODEC_422_PL12:
			case IMG_CODEC_422_PL21:
				y_field_size = picture_height * stride_bytes;
				c_field_size = picture_height * stride_bytes;
				break;
			case IMG_CODEC_Y0UY1V_8888:
			case IMG_CODEC_UY0VY1_8888:
			case IMG_CODEC_Y0VY1U_8888:
			case IMG_CODEC_VY0UY1_8888:
				y_field_size = picture_height * stride_bytes;
				c_field_size = y_field_size;
				break;
			case IMG_CODEC_444_YUV:
			case IMG_CODEC_444_YV12:
			case IMG_CODEC_444_IMC2:
				y_field_size = picture_height * stride_bytes * 3;
				c_field_size = y_field_size;
				break;
			case IMG_CODEC_444_PL8:
				y_field_size = picture_height * stride_bytes;
				c_field_size = picture_height * stride_bytes;
				break;
			case IMG_CODEC_444_PL12:
			case IMG_CODEC_444_PL21:
				y_field_size = picture_height * stride_bytes;
				c_field_size = picture_height * stride_bytes * 2;
				break;
			case IMG_CODEC_ABCX:
			case IMG_CODEC_XBCA:
			case IMG_CODEC_ABC565:
				y_field_size = picture_height * stride_bytes;
				c_field_size = y_field_size;
				break;
			default:
				y_field_size = picture_height * stride_bytes * 3 / 2;
				c_field_size = y_field_size;
				break;
			}

			frame->field1_y_offset = y_field_size;
			frame->field1_u_offset = c_field_size;
			frame->field1_v_offset = c_field_size;
		}
	} else {
		frame->field1_y_offset = frame->field0_y_offset;
		frame->field1_u_offset = frame->field0_u_offset;
		frame->field1_v_offset = frame->field0_v_offset;
	}
	return IMG_SUCCESS;
}

void topaz_setup_input_csc(struct img_video_context *video,
			   struct img_vxe_scaler_setup *scaler_setup,
			   struct img_vxe_csc_setup *csc_setup,
			   enum img_csc_preset csc_preset)
{
#define CSC_MINUS_1_16(X) TOPAZHP_EXT_CR_CSC_SOURCE_MOD_0 ## X ## _MINUS_1_16
#define CSC_MINUS_1_2(X) TOPAZHP_EXT_CR_CSC_SOURCE_MOD_0 ## X ## _MINUS_1_2
#define CSC_UNSIGNED(X) TOPAZHP_EXT_CR_CSC_SOURCE_MOD_0 ## X ## _UNSIGNED

	if (csc_preset != IMG_CSC_NONE &&
	    (video->format == IMG_CODEC_ABCX ||
	     video->format == IMG_CODEC_XBCA || video->format == IMG_CODEC_ABC565)) {
		unsigned char source_mode[IMG_CSC_PRESETS][3] = {
			/* IMG_CSC_NONE - No colour-space conversion */
			{CSC_MINUS_1_16(0), CSC_MINUS_1_2(1), CSC_MINUS_1_16(2)},

			/* IMG_CSC_709_TO_601 - ITU BT.709 YUV to be converted to ITU BT.601 YUV */
			{CSC_MINUS_1_16(0), CSC_MINUS_1_2(1), CSC_MINUS_1_16(2)},

			/* IMG_CSC_601_TO_709 - ITU BT.601 YUV to be
			 * converted to ITU BT.709 YUV
			 */
			{CSC_MINUS_1_16(0), CSC_MINUS_1_2(1), CSC_MINUS_1_16(2)},

			/* IMG_CSC_RGB_TO_601_ANALOG - RGB to be
			 * converted to ITU BT.601 YUV
			 */
			{  CSC_UNSIGNED(0),  CSC_UNSIGNED(1),  CSC_UNSIGNED(2)},

			/* IMG_CSC_RGB_TO_601_DIGITAL - RGB to be
			 * converted to ITU BT.601 YCbCr RS
			 */
			{  CSC_UNSIGNED(0),  CSC_UNSIGNED(1),  CSC_UNSIGNED(2)},

			/* IMG_CSC_RGB_TO_601_DIGITAL_FS - RGB to be
			 * converted to ITU BT.601 YCbCr FS
			 */
			{  CSC_UNSIGNED(0),  CSC_UNSIGNED(1),  CSC_UNSIGNED(2)},

			/* IMG_CSC_RGB_TO_709 - RGB to be converted to ITU BT.709 YUV */
			{  CSC_UNSIGNED(0),  CSC_UNSIGNED(1),  CSC_UNSIGNED(2)},

			/* IMG_CSC_YIQ_TO_601 - YIQ to be converted to ITU BT.601 YUV */
			{CSC_MINUS_1_16(0), CSC_MINUS_1_2(1), CSC_MINUS_1_16(2)},

			/* IMG_CSC_YIQ_TO_709 - YIQ to be converted to ITU BT.709 YUV */
			{CSC_MINUS_1_16(0), CSC_MINUS_1_2(1), CSC_MINUS_1_16(2)},

			/* IMG_CSC_BRG_TO_601 - RGB to be converted to ITU BT.601 YUV */
			{0, 0, 0},

			/* IMG_CSC_RBG_TO_601 - RGB to be converted to ITU BT.709 YUV */
			{0, 0, 0},

			/* IMG_CSC_BGR_TO_601 - RGB to be converted to ITU BT.601 YUV */
			{0, 0, 0},

			/* IMG_CSC_UYV_TO_YUV - UYV to be converted to YUV */
			{CSC_MINUS_1_2(0), CSC_MINUS_1_16(1), CSC_MINUS_1_2(2)},
			/*{  CSC_UNSIGNED(0),  CSC_UNSIGNED(1),  CSC_UNSIGNED(2)}, */
		};

		int coeffs[IMG_CSC_PRESETS][3][3] = {
			/* IMG_CSC_NONE - No colour-space conversion */
			{
				{ 1024,    0,    0 },
				{    0, 1024,    0 },
				{    0,    0, 1024 }
			},

			/* IMG_CSC_709_TO_601 - ITU BT.709 YUV to be converted to ITU BT.601 YUV */
			{
				{ 1024, (int)(0.15941 * 1024), (int)(0.11649 * 1024) },
				{    0, (int)(-0.07844 * 1024), (int)(0.98985 * 1024) },
				{    0, (int)(0.9834  * 1024), (int)(-0.10219 * 1024) }
			},

			/* IMG_CSC_601_TO_709 - ITU BT.601 YUV to be converted to ITU BT.709 YUV */
			{
				{ 1024, (int)(-0.17292 * 1024), (int)(-0.13554 * 1024) },
				{    0, (int)(0.08125 * 1024), (int)(1.01864 * 1024) },
				{    0, (int)(1.02532 * 1024), (int)(0.10586 * 1024) }
			},

			/* IMG_CSC_RGB_TO_601_ANALOG - RGB to be converted to ITU BT.601 YUV */
			{ /*		R	G		B */
				{ (int)(219 *  0.299   * 4.0157),
				  (int)(219 *  0.587   * 4.0157),
				  (int)(219 *  0.114   * 4.0157) },
				{ (int)(224 * -0.14713 * 4.0157),
				  (int)(224 * -0.28886 * 4.0157),
				  (int)(224 *  0.446   * 4.0157) },
				{ (int)(224 *  0.615   * 4.0157),
				  (int)(224 * -0.51499 * 4.0157),
				  (int)(224 * -0.10001 * 4.0157) }
			}, /*		A	B	C */

			/* IMG_CSC_RGB_TO_601_DIGITAL - RGB to be
			 * converted to ITU BT.601 YCbCr reduced scale
			 */
			{ /*		R	G	B */
				{ (int)(219 *  0.299   * 4.0157),
				  (int)(219 *  0.587   * 4.0157),
				  (int)(219 *  0.114   * 4.0157) },
				{ (int)(224 * -0.172   * 4.0157),
				  (int)(224 * -0.339   * 4.0157),
				  (int)(224 *  0.511   * 4.0157) },
				{ (int)(224 *  0.511   * 4.0157),
				  (int)(224 * -0.428   * 4.0157),
				  (int)(224 * -0.083   * 4.0157) }
			}, /*	A		B		C */

			/* IMG_CSC_RGB_TO_601_DIGITAL_FS - RGB to be
			 * converted to ITU BT.601 YCbCr full scale
			 */
			{ /*		R	G	B */
				{ (int)(219 *  0.257   * 4.0157),
				  (int)(219 *  0.504   * 4.0157),
				  (int)(219 *  0.098   * 4.0157) },
				{ (int)(224 * -0.148   * 4.0157),
				  (int)(224 * -0.291   * 4.0157),
				  (int)(224 *  0.439   * 4.0157) },
				{ (int)(224 *  0.439   * 4.0157),
				  (int)(224 * -0.368   * 4.0157),
				  (int)(224 * -0.071   * 4.0157) }
			}, /*	A	B		C */

			/* IMG_CSC_RGB_TO_709 - RGB to be converted to ITU BT.709 YUV */
			{
				{ (int)(219 *  0.2215 * 4.0157), (int)(219 *  0.7154 * 4.0157),
				  (int)(219 *  0.0721 * 4.0157) },
				{ (int)(224 * -0.1145 * 4.0157), (int)(224 * -0.3855 * 4.0157),
				  (int)(224 *  0.5    * 4.0157) },
				{ (int)(224 *  0.5016 * 4.0157), (int)(224 * -0.4556 * 4.0157),
				  (int)(224 * -0.0459 * 4.0157) }
			},

			/* IMG_CSC_YIQ_TO_601 - YIQ to be converted to ITU BT.601 YUV */
			{
				{ 1024,                           0,                            0 },
				{    0, (int)(0.83885 * 1024), (int)(-0.54475 * 1024) },
				{    0, (int)(0.54484 * 1024), (int)(0.83896 * 1024) }
			},

			/* IMG_CSC_YIQ_TO_709 - YIQ to be converted to ITU BT.709 YUV */
			{
				{ 1024, (int)(-0.20792 * 1024), (int)(0.07122 * 1024) },
				{    0, (int)(0.89875 * 1024), (int)(-0.48675 * 1024) },
				{    0, (int)(0.64744 * 1024), (int)(0.80255 * 1024) }
			},

			/*
			 * IMG_CSC_BRG_TO_601 - RGB to be converted to ITU BT.601 YUV
			 * Entries have been reordered to provide support for xRGB format
			 */
			{ /*            B         R       G      */
				{ (int)(219 *  0.114   * 4.0157),
				  (int)(219 *  0.299   * 4.0157),
				  (int)(219 *  0.587   * 4.0157)},
				{ (int)(224 *  0.446   * 4.0157),
				  (int)(224 * -0.14713 * 4.0157),
				  (int)(224 * -0.28886 * 4.0157)},
				{ (int)(224 * -0.10001 * 4.0157),
				  (int)(224 *  0.615   * 4.0157),
				  (int)(224 * -0.51499 * 4.0157)}
			}, /*	A	B				C */

			/*
			 * IMG_CSC_RBG_TO_601 - RGB to be converted to ITU BT.601 YUV
			 * Entries have been reordered to provide support for xBGR format
			 */
			{ /*              R        B        G       */
				{ (int)(219 *  0.299   * 4.0157),
				  (int)(219 *  0.114   * 4.0157),
				  (int)(219 *  0.587   * 4.0157)},
				{ (int)(224 * -0.14713 * 4.0157),
				  (int)(224 *  0.446   * 4.0157),
				  (int)(224 * -0.28886 * 4.0157)},
				{ (int)(224 *  0.615   * 4.0157),
				  (int)(224 * -0.10001 * 4.0157),
				  (int)(224 * -0.51499 * 4.0157)}
			}, /*		   A		B	C */

			/*
			 * IMG_CSC_BGR_TO_601 - RGB to be converted to ITU BT.601 YUV
			 * Entries have been reordered to provide support for BGRx format
			 */
			{ /*              B       G          R */
				{ (int)(219 *  0.114   * 4.0157),
				  (int)(219 *  0.587   * 4.0157),
				  (int)(219 *  0.299   * 4.0157)},
				{ (int)(224 *  0.446   * 4.0157),
				  (int)(224 * -0.28886 * 4.0157),
				  (int)(224 * -0.14713 * 4.0157)},
				{ (int)(224 * -0.10001 * 4.0157),
				  (int)(224 * -0.51499 * 4.0157),
				  (int)(224 *  0.615   * 4.0157)},
			},	/*	A	B	C */

			/* IMG_CSC_UYV_TO_YUV - UYV to YUV */
			{
				{ 0, 1024, 0 },
				{ 1024, 0, 0 },
				{ 0, 0, 1024 }
			},
		};

		unsigned int index = csc_preset;

		IMG_DBG_ASSERT(index < IMG_CSC_PRESETS);

		if (index >= IMG_CSC_PRESETS)
			return;

#define SRC_MOD(X) TOPAZHP_EXT_CR_CSC_SOURCE_MOD_0 ## X
#define OUT_MOD(X) TOPAZHP_EXT_CR_CSC_OUTPUT_MOD_0 ## X

#define SOURCE_Y_ARRAY csc_setup->csc_source_y
#define SRC_Y_PARAM(X) TOPAZHP_EXT_CR_CSC_SOURCE_SRC_TO_Y

#define SOURCE_CBCR_ARRAY csc_setup->csc_source_cbcr
#define SRC_CB_PARAM(X) TOPAZHP_EXT_CR_CSC_SOURCE_SRC_TO_CB
#define SRC_CR_PARAM(X) TOPAZHP_EXT_CR_CSC_SOURCE_SRC_TO_CR

#define CLIP_VALUE 255

		scaler_setup->scaler_control |= F_ENCODE(1,
				TOPAZHP_EXT_CR_ENABLE_COLOUR_SPACE_CONVERSION);

		csc_setup->csc_output_clip[0] =
			F_ENCODE(TOPAZHP_EXT_CR_CSC_OUTPUT_MOD_00_ADD_1_16, OUT_MOD(0)) |
			F_ENCODE(CLIP_VALUE, TOPAZHP_EXT_CR_CSC_OUTPUT_MAX_CLIP) |
			F_ENCODE(0, TOPAZHP_EXT_CR_CSC_OUTPUT_MIN_CLIP);

		csc_setup->csc_output_clip[1] =
			F_ENCODE(TOPAZHP_EXT_CR_CSC_OUTPUT_MOD_01_ADD_1_2, OUT_MOD(1)) |
			F_ENCODE(CLIP_VALUE, TOPAZHP_EXT_CR_CSC_OUTPUT_MAX_CLIP) |
			F_ENCODE(0, TOPAZHP_EXT_CR_CSC_OUTPUT_MIN_CLIP);

		SOURCE_Y_ARRAY[0] = F_ENCODE(source_mode[index][0], SRC_MOD(0)) |
			F_ENCODE(coeffs[index][0][0], SRC_Y_PARAM(0));
		SOURCE_CBCR_ARRAY[0] = F_ENCODE(coeffs[index][1][0], SRC_CB_PARAM(0)) |
			F_ENCODE(coeffs[index][2][0], SRC_CR_PARAM(0));

		SOURCE_Y_ARRAY[1] = F_ENCODE(source_mode[index][1], SRC_MOD(1)) |
			F_ENCODE(coeffs[index][0][1], SRC_Y_PARAM(1));
		SOURCE_CBCR_ARRAY[1] = F_ENCODE(coeffs[index][1][1], SRC_CB_PARAM(1)) |
			F_ENCODE(coeffs[index][2][1], SRC_CR_PARAM(1));

		SOURCE_Y_ARRAY[2] = F_ENCODE(source_mode[index][2], SRC_MOD(2)) |
			F_ENCODE(coeffs[index][0][2], SRC_Y_PARAM(2));
		SOURCE_CBCR_ARRAY[2] = F_ENCODE(coeffs[index][1][2], SRC_CB_PARAM(2)) |
			F_ENCODE(coeffs[index][2][2], SRC_CR_PARAM(2));
	}
}

/*
 * Calculate buffer strides
 */
unsigned int topaz_get_packed_buffer_strides(ushort buffer_stride_bytes,
					     enum img_format format,
					     unsigned char enable_scaler,
					     unsigned char is_interlaced,
					     unsigned char is_interleaved)
{
	ushort src_y_stride_bytes;
	ushort src_uv_stride_bytes = 0;

	/* 3 Components: Y, U, V */
	src_y_stride_bytes = buffer_stride_bytes;

	switch (format) {
	case IMG_CODEC_420_YUV:
	case IMG_CODEC_420_PL8:
	case IMG_CODEC_420_YV12:
		src_uv_stride_bytes = src_y_stride_bytes / 2;
		break;

	case IMG_CODEC_422_YUV:         /* Odd-numbered chroma rows unused if scaler not present */
	case IMG_CODEC_422_YV12:        /* Odd-numbered chroma rows unused if scaler not present */
	case IMG_CODEC_422_PL8:         /* Odd-numbered chroma rows unused if scaler not present */
		if (!enable_scaler)
		/* Skip alternate lines of chroma for 4:2:2 if scaler disabled/not present */
			src_uv_stride_bytes = src_y_stride_bytes;
		else
			src_uv_stride_bytes = src_y_stride_bytes / 2;
		break;
	/* Interleaved chroma pixels (and unused odd-numbered chroma rows if scaler not present) */
	case IMG_CODEC_422_IMC2:
	/* Interleaved chroma rows (and unused odd-numbered chroma rows if scaler not present) */
	case IMG_CODEC_422_PL12:
	/* Interleaved chroma rows (and unused odd-numbered chroma rows if scaler not present) */
	case IMG_CODEC_422_PL21:
		if (!enable_scaler)
		/* Skip alternate lines of chroma for 4:2:2 if scaler disabled/not present */
			src_uv_stride_bytes = src_y_stride_bytes * 2;
		else
			src_uv_stride_bytes = src_y_stride_bytes;
		break;

	case IMG_CODEC_420_PL12:                /* Interleaved chroma pixels */
	case IMG_CODEC_420_PL21:

	case IMG_CODEC_420_PL12_PACKED: /* Interleaved chroma pixels */
	case IMG_CODEC_420_PL21_PACKED: /* Interleaved chroma pixels */
	case IMG_CODEC_420_IMC2:                /* Interleaved chroma rows */
	case IMG_CODEC_Y0UY1V_8888:     /* Interleaved luma and chroma pixels */
	case IMG_CODEC_Y0VY1U_8888:     /* Interleaved luma and chroma pixels */
	case IMG_CODEC_UY0VY1_8888:     /* Interleaved luma and chroma pixels */
	case IMG_CODEC_VY0UY1_8888:     /* Interleaved luma and chroma pixels */
	case IMG_CODEC_ABCX:            /* Interleaved pixels of unknown colour space */
	case IMG_CODEC_XBCA:            /* Interleaved pixels of unknown colour space */
	case IMG_CODEC_ABC565:          /* Packed pixels of unknown coloour space */
		src_uv_stride_bytes = src_y_stride_bytes;
		break;

	case IMG_CODEC_444_YUV:         /* Unusable if scaler not present */
	case IMG_CODEC_444_YV12:        /* Unusable if scaler not present */
	case IMG_CODEC_444_PL8:         /* Unusable if scaler not present */
		src_uv_stride_bytes = src_y_stride_bytes;
		break;

	/* Interleaved chroma pixels (unusable if scaler not present) */
	case IMG_CODEC_444_IMC2:
	/* Interleaved chroma rows (unusable if scaler not present) */
	case IMG_CODEC_444_PL12:
	/* Interleaved chroma rows (unusable if scaler not present) */
	case IMG_CODEC_444_PL21:
		src_uv_stride_bytes = src_y_stride_bytes * 2;
		break;

	default:
		break;
	}

	if (is_interlaced && is_interleaved) {
		src_y_stride_bytes *= 2;
		src_uv_stride_bytes *= 2;
	}
	return F_ENCODE(src_y_stride_bytes >> 6, MTX_MSG_PICMGMT_STRIDE_Y) |
		F_ENCODE(src_uv_stride_bytes >> 6, MTX_MSG_PICMGMT_STRIDE_UV);
}

/*
 * Setup the registers for scaling candidate motion vectors to take into account
 * how far away (temporally) the reference pictures are
 */
#define RESTRICT16x16_FLAGS                             (0x1)
#define RESTRICT8x8_FLAGS                               (0x2)

void update_driver_mv_scaling(unsigned int frame_num, unsigned int ref0_num, unsigned int ref1_num,
			      unsigned int pic_flags, unsigned int *mv_calc_below_handle,
			      unsigned int *mv_calc_colocated_handle,
			      unsigned int *mv_calc_config_handle)
{
	unsigned int mv_calc_config = 0;
	unsigned int mv_calc_colocated = F_ENCODE(0x10, TOPAZHP_CR_TEMPORAL_BLEND);
	unsigned int mv_calc_below = 0;

	/* If b picture calculate scaling factor for colocated motion vectors */
	if (pic_flags & ISINTERB_FLAGS) {
		int tb, td, tx;
		int dist_scale;

		/* calculation taken from H264 spec */
		tb = (frame_num * 2) - (ref1_num * 2);
		td = (ref0_num  * 2) - (ref1_num * 2);
		tx = (16384 + abs(td / 2)) / td;
		dist_scale = (tb * tx + 32) >> 6;
		if (dist_scale > 1023)
			dist_scale = 1023;

		if (dist_scale < -1024)
			dist_scale = -1024;

		mv_calc_colocated |= F_ENCODE(dist_scale, TOPAZHP_CR_COL_DIST_SCALE_FACT);

		/*
		 * We assume the below temporal mvs are from the latest reference frame
		 * rather then the most recently encoded B frame (as Bs aren't reference)
		 * Fwd temporal is same as colocated mv scale
		 */
		mv_calc_below |= F_ENCODE(dist_scale, TOPAZHP_CR_PIC0_DIST_SCALE_FACTOR);

		/* Bkwd temporal needs to be scaled by the recipricol
		 * amount in the other direction
		 */
		tb = (frame_num * 2) - (ref0_num * 2);
		td = (ref0_num  * 2) - (ref1_num * 2);
		tx = (16384 + abs(td / 2)) / td;
		dist_scale = (tb * tx + 32) >> 6;
		if (dist_scale > 1023)
			dist_scale = 1023;

		if (dist_scale < -1024)
			dist_scale = -1024;

		mv_calc_below |= F_ENCODE(dist_scale, TOPAZHP_CR_PIC1_DIST_SCALE_FACTOR);
	} else {
		/* Don't scale the temporal below mvs */
		mv_calc_below |= F_ENCODE(1 << 8, TOPAZHP_CR_PIC0_DIST_SCALE_FACTOR);

		if (ref0_num != ref1_num) {
			int ref0_dist, ref1_dist;
			int scale;

			/*
			 * Distance to second reference picture may be different when
			 * using multiple reference frames on P. Scale based on difference
			 * in temporal distance to ref pic 1 compared to distance to ref pic 0
			 */
			ref0_dist = (frame_num - ref0_num);
			ref1_dist = (frame_num - ref1_num);
			scale    = (ref1_dist << 8) / ref0_dist;

			if (scale > 1023)
				scale = 1023;
			if (scale < -1024)
				scale = -1024;

			mv_calc_below |= F_ENCODE(scale, TOPAZHP_CR_PIC1_DIST_SCALE_FACTOR);
		} else {
			mv_calc_below |= F_ENCODE(1 << 8, TOPAZHP_CR_PIC1_DIST_SCALE_FACTOR);
		}
	}

	if (frame_num > 0) {
		int ref0_distance, ref1_distance;
		int jitter0, jitter1;

		ref0_distance = abs((int)frame_num - (int)ref0_num);
		ref1_distance = abs((int)frame_num - (int)ref1_num);

		if (!(pic_flags & ISINTERB_FLAGS)) {
			jitter0 = ref0_distance * 1;
			jitter1 = jitter0 > 1 ? 1 : 2;
		} else {
			jitter0 = ref1_distance * 1;
			jitter1 = ref0_distance * 1;
		}

		/* Hardware can only cope with 1 - 4 jitter factors */
		jitter0 = (jitter0 > 4) ? 4 : (jitter0 < 1) ? 1 : jitter0;
		jitter1 = (jitter1 > 4) ? 4 : (jitter1 < 1) ? 1 : jitter1;

		/* Hardware can only cope with 1 - 4 jitter factors */
		IMG_DBG_ASSERT(jitter0 > 0 && jitter0 <= 4 && jitter1 > 0 && jitter1 <= 4);

		mv_calc_config |= F_ENCODE(jitter0 - 1, TOPAZHP_CR_MVCALC_IPE0_JITTER_FACTOR) |
			F_ENCODE(jitter1 - 1, TOPAZHP_CR_MVCALC_IPE1_JITTER_FACTOR);
	}

	mv_calc_config |= F_ENCODE(1, TOPAZHP_CR_MVCALC_DUP_VEC_MARGIN);
	mv_calc_config |= F_ENCODE(7, TOPAZHP_CR_MVCALC_GRID_MB_X_STEP);
	mv_calc_config |= F_ENCODE(13, TOPAZHP_CR_MVCALC_GRID_MB_Y_STEP);
	mv_calc_config |= F_ENCODE(3, TOPAZHP_CR_MVCALC_GRID_SUB_STEP);
	mv_calc_config |= F_ENCODE(1, TOPAZHP_CR_MVCALC_GRID_DISABLE);

	mv_calc_config |= F_ENCODE(1, TOPAZHP_CR_MVCALC_NO_PSEUDO_DUPLICATES);

	*mv_calc_below_handle = mv_calc_below;
	*mv_calc_colocated_handle = mv_calc_colocated;
	*mv_calc_config_handle = mv_calc_config;
}

void prepare_mv_estimates(struct img_enc_context *enc)
{
	struct img_video_context *vid_ctx = enc->video;
	unsigned int distance;
	unsigned int distance_b;
	unsigned int position;
	struct img_mv_settings *host_mv_settings_b_table;
	struct img_mv_settings *host_mv_settings_hierarchical;
	unsigned char hierarchical;

	/* IDR */
	vid_ctx->mv_settings_idr.mv_calc_config = DEFAULT_MVCALC_CONFIG; /* default based on TRM */
	vid_ctx->mv_settings_idr.mv_calc_colocated = 0x00100100; /* default based on TRM */
	vid_ctx->mv_settings_idr.mv_calc_below = 0x01000100;    /* default based on TRM */

	update_driver_mv_scaling(0, 0, 0, 0, &vid_ctx->mv_settings_idr.mv_calc_below,
				 &vid_ctx->mv_settings_idr.mv_calc_colocated,
				 &vid_ctx->mv_settings_idr.mv_calc_config);

	/* NonB (I or P) */
	for (distance = 1; distance <= MAX_BFRAMES + 1; distance++) {
		/* default based on TRM */
		vid_ctx->mv_settings_non_b[distance - 1].mv_calc_config = DEFAULT_MVCALC_CONFIG;
		/* default based on TRM */
		vid_ctx->mv_settings_non_b[distance - 1].mv_calc_colocated = 0x00100100;
		/* default based on TRM */
		vid_ctx->mv_settings_non_b[distance - 1].mv_calc_below = 0x01000100;

		update_driver_mv_scaling
				(distance, 0, 0, 0,
				 &vid_ctx->mv_settings_non_b[distance - 1].mv_calc_below,
				 &vid_ctx->mv_settings_non_b[distance - 1].mv_calc_colocated,
				 &vid_ctx->mv_settings_non_b[distance - 1].mv_calc_config);
	}

	hierarchical = (bool)(vid_ctx->mv_settings_hierarchical.cpu_virt);

	host_mv_settings_b_table = (struct img_mv_settings *)(vid_ctx->mv_settings_btable.cpu_virt);

	if (hierarchical)
		host_mv_settings_hierarchical =
			(struct img_mv_settings *)(vid_ctx->mv_settings_hierarchical.cpu_virt);

	for (distance_b = 0; distance_b < MAX_BFRAMES; distance_b++) {
		for (position = 1; position <= distance_b + 1; position++) {
			struct img_mv_settings *mv_element =
			(struct img_mv_settings *)((unsigned char *)host_mv_settings_b_table +
				MV_OFFSET_IN_TABLE(distance_b, position - 1));

			mv_element->mv_calc_config =
				/* default based on TRM */
				(DEFAULT_MVCALC_CONFIG | MASK_TOPAZHP_CR_MVCALC_GRID_DISABLE);

			mv_element->mv_calc_colocated = 0x00100100;/* default based on TRM */
			mv_element->mv_calc_below = 0x01000100; /* default based on TRM */

			update_driver_mv_scaling(position, distance_b + 2, 0, ISINTERB_FLAGS,
						 &mv_element->mv_calc_below,
						 &mv_element->mv_calc_colocated,
						 &mv_element->mv_calc_config);
		}
	}

	if (hierarchical) {
		for (distance_b = 0; distance_b < MAX_BFRAMES; distance_b++)
			memcpy(host_mv_settings_hierarchical + distance_b,
			       (unsigned char *)host_mv_settings_b_table +
			       MV_OFFSET_IN_TABLE(distance_b, distance_b >> 1),
			       sizeof(struct img_mv_settings));
	}
}

/*
 * Generates the video pic params template
 */
void adjust_pic_flags(struct img_enc_context *enc, struct img_rc_params *rc_params,
		      unsigned char first_pic, unsigned int *flags)
{
	unsigned int flags_local;
	struct pic_params *pic_params = &enc->video->pic_params;

	flags_local = pic_params->flags;

	if (!rc_params->rc_enable || !first_pic)
		flags_local = 0;

	*flags = flags_local;
}

/*
 * Sets up RC Data
 */
void setup_rc_data(struct img_video_context *video, struct pic_params *pic_params,
		   struct img_rc_params *rc_params)
{
	int tmp_qp = 0;
	int buffer_size_in_frames;
	short max_qp = MAX_QP_H264;
	short min_qp = 0;
	int mul_of_8mbits;
	int framerate, scale = 1;
	int l1, l2, l3, l4, l5, scaled_bpp;

	/* If Bit Rate and Basic Units are not specified then set to default values. */
	if (rc_params->bits_per_second == 0 && !video->enable_mvc)
		rc_params->bits_per_second = 640000; /* kbps */

	if (!rc_params->bu_size)
		/* BU = 1 Frame */
		rc_params->bu_size = (video->picture_height >> 4) * (video->width >> 4);

	if (!rc_params->frame_rate)
		rc_params->frame_rate = 30; /* fps */

	/* Calculate Bits per Pixel */
	if (video->width <= 176)
		framerate = 30;
	else
		framerate = rc_params->frame_rate;

	mul_of_8mbits = rc_params->bits_per_second / 8000000;

	if (mul_of_8mbits == 0)
		scale = 256;
	else if (mul_of_8mbits > 127)
		scale = 1;
	else
		scale = 128 / mul_of_8mbits;

	scaled_bpp = (scale * rc_params->bits_per_second) /
		(framerate * video->width * video->frame_height);

	pic_params->in_params.se_init_qp_i      = rc_params->initial_qp_i;

	pic_params->in_params.mb_per_row        = (video->width >> 4);
	pic_params->in_params.mb_per_bu         = rc_params->bu_size;
	pic_params->in_params.mb_per_frm                = ((unsigned int)(video->width >> 4)) *
		(video->frame_height >> 4);
	pic_params->in_params.bu_per_frm        = (pic_params->in_params.mb_per_frm) /
		rc_params->bu_size;

	pic_params->in_params.intra_period      = rc_params->intra_freq;
	pic_params->in_params.bframes           = rc_params->bframes;
	pic_params->in_params.bit_rate          = rc_params->bits_per_second;

	pic_params->in_params.frm_skip_disable = rc_params->disable_frame_skipping;

	pic_params->in_params.bits_per_frm      =
		(rc_params->bits_per_second + rc_params->frame_rate / 2) / rc_params->frame_rate;

	pic_params->in_params.bits_per_bu       = pic_params->in_params.bits_per_frm /
		(4 * pic_params->in_params.bu_per_frm);

	/*Disable Vcm Hardware*/
	pic_params->in_params.disable_vcm_hardware = rc_params->disable_vcm_hardware;
	/* Codec-dependent fields */
	if (video->standard == IMG_STANDARD_H264) {
		pic_params->in_params.mode.h264.transfer_rate  =
			(rc_params->transfer_bits_per_second + rc_params->frame_rate / 2) /
			rc_params->frame_rate;
		pic_params->in_params.mode.h264.hierarchical_mode =   rc_params->hierarchical;

		pic_params->in_params.mode.h264.enable_slice_bob =
			(unsigned char)rc_params->enable_slice_bob;
		pic_params->in_params.mode.h264.max_slice_bob =
			(unsigned char)rc_params->max_slice_bob;
		pic_params->in_params.mode.h264.slice_bob_qp =
			(unsigned char)rc_params->slice_bob_qp;
	}

	if (pic_params->in_params.bits_per_frm) {
		buffer_size_in_frames =
			(rc_params->buffer_size + (pic_params->in_params.bits_per_frm / 2)) /
			pic_params->in_params.bits_per_frm;
	} else {
		IMG_DBG_ASSERT(video->enable_mvc && ("Can happen only in MVC mode" != NULL));
		/* Asigning more or less `normal` value. To be overridden by MVC RC module */
		buffer_size_in_frames = 30;
	}

	/* select thresholds and initial Qps etc that are codec dependent */
	switch (video->standard) {
	case IMG_STANDARD_H264:
		/* Setup MAX and MIN Quant Values */
		pic_params->in_params.max_qp = (rc_params->max_qp > 0) &&
			(rc_params->max_qp < max_qp) ? rc_params->max_qp : max_qp;

		if (rc_params->min_qp == 0) {
			if (scaled_bpp >= (scale >> 1)) {
				tmp_qp = 4;
			} else if (scaled_bpp > ((scale << 1) / 15)) {
				tmp_qp = (22 * scale) - (40 * scaled_bpp);
				tmp_qp = tmp_qp / scale;
			} else {
				tmp_qp = (30 * scale) - (100 * scaled_bpp);
				tmp_qp = tmp_qp / scale;
			}

			/* Adjust minQp up for small buffer size and down for large buffer size */
			if (buffer_size_in_frames < 5) {
				tmp_qp += 2;
			} else if (buffer_size_in_frames > 40) {
				if (tmp_qp >= 1)
					tmp_qp -= 1;
			}
			/* for HD content allow a lower minQp as bitrate is
			 * more easily controlled in this case
			 */
			if (pic_params->in_params.mb_per_frm > 2000)
				tmp_qp -= 6;
		} else {
			tmp_qp = rc_params->min_qp;
		}

		min_qp = 2;

		if (tmp_qp < min_qp)
			pic_params->in_params.min_qp = min_qp;
		else
			pic_params->in_params.min_qp = tmp_qp;

		/* Calculate Initial QP if it has not been specified */
		tmp_qp = pic_params->in_params.se_init_qp_i;
		if (pic_params->in_params.se_init_qp_i == 0) {
			l1 = scale / 20;
			l2 = scale / 5;
			l3 = (scale * 2) / 5;
			l4 = (scale * 4) / 5;
			l5 = (scale * 1011) / 1000;

			tmp_qp = pic_params->in_params.min_qp;

			pic_params->in_params.se_init_qp_i = tmp_qp;
			if (scaled_bpp < l1)
				tmp_qp = (45 * scale) - (78 * scaled_bpp);
			else if (scaled_bpp < l2)
				tmp_qp = (44 * scale) - (73 * scaled_bpp);
			else if (scaled_bpp < l3)
				tmp_qp = (34 * scale) - (25 * scaled_bpp);
			else if (scaled_bpp < l4)
				tmp_qp = (32 * scale) - (20 * scaled_bpp);
			else if (scaled_bpp < l5)
				tmp_qp = (25 * scale) - (10 * scaled_bpp);
			else
				tmp_qp = (18 * scale) - (5 * scaled_bpp);

			/* Adjust ui8SeInitQP up for small buffer size or small fps */
			/* Adjust ui8SeInitQP up for small gop size */
			if (buffer_size_in_frames < 20 || rc_params->intra_freq < 20)
				tmp_qp += 2 * scale;

			/* for very small buffers increase initial Qp even more */
			if (buffer_size_in_frames < 5)
				tmp_qp += 8 * scale;

			/* start on a lower initial Qp for HD content
			 * as the coding is more efficient
			 */
			if (pic_params->in_params.mb_per_frm > 2000)
				tmp_qp -= 2 * scale;

			if (pic_params->in_params.intra_period == 1) {
				/* for very small GOPS start with a much higher initial Qp */
				tmp_qp += 12 * scale;
			} else if (pic_params->in_params.intra_period < 5) {
				tmp_qp += 6 * scale;
			}

			tmp_qp = tmp_qp / scale;
		}

		max_qp = 49;

		if (tmp_qp > max_qp)
			tmp_qp = max_qp;

		if (tmp_qp < pic_params->in_params.min_qp)
			tmp_qp = pic_params->in_params.min_qp;

		pic_params->in_params.se_init_qp_i = tmp_qp;

		if (scaled_bpp <= ((3 * scale) / 10))
			pic_params->flags |= ISRC_I16BIAS;
		break;

	default:
		/* the NO RC cases will fall here */
		break;
	}

	if (video->rc_params.rc_mode == IMG_RCMODE_VBR) {
		pic_params->in_params.mb_per_bu = pic_params->in_params.mb_per_frm;
		pic_params->in_params.bu_per_frm        = 1;

		/* Initialize the parameters of fluid flow traffic model. */
		pic_params->in_params.buffer_size = rc_params->buffer_size;

		/* VBR shouldn't skip frames */
		pic_params->in_params.frm_skip_disable = TRUE;

		/*
		 * These scale factor are used only for rate control to avoid overflow
		 * in fixed-point calculation these scale factors are decided by bit rate
		 */
		if (rc_params->bits_per_second < 640000)
			pic_params->in_params.scale_factor  = 2; /* related to complexity */
		else if (rc_params->bits_per_second < 2000000)    /* 2 Mbits */
			pic_params->in_params.scale_factor  = 4;
		else if (rc_params->bits_per_second < 8000000)     /* 8 Mbits */
			pic_params->in_params.scale_factor  = 6;
		else
			pic_params->in_params.scale_factor  = 8;
	} else {
		/* Set up Input Parameters that are mode dependent */
		switch (video->standard) {
		case IMG_STANDARD_H264:
			/*
			 * H264 CBR RC: Initialize the parameters of fluid flow traffic model.
			 */
			pic_params->in_params.buffer_size = rc_params->buffer_size;

			/* HRD consideration - These values are used by H.264 reference code. */
			if (rc_params->bits_per_second < 1000000)         /* 1 Mbits/s */
				pic_params->in_params.scale_factor = 0;
			else if (rc_params->bits_per_second < 2000000)    /* 2 Mbits/s */
				pic_params->in_params.scale_factor = 1;
			else if (rc_params->bits_per_second < 4000000)    /* 4 Mbits/s */
				pic_params->in_params.scale_factor = 2;
			else if (rc_params->bits_per_second < 8000000)    /* 8 Mbits/s */
				pic_params->in_params.scale_factor = 3;
			else
				pic_params->in_params.scale_factor = 4;

			if (video->rc_params.rc_mode == IMG_RCMODE_VCM)
				pic_params->in_params.buffer_size_frames = buffer_size_in_frames;
			break;

		default:
			break;
		}
	}

	if (rc_params->sc_detect_disable)
		pic_params->flags  |= ISSCENE_DISABLED;

	pic_params->in_params.initial_delay     = rc_params->initial_delay;
	pic_params->in_params.initial_level     = rc_params->initial_level;
	rc_params->initial_qp_i                 = pic_params->in_params.se_init_qp_i;

	/* The rate control uses this value to adjust
	 * the reaction rate to larger than expected frames
	 */
	if (video->standard == IMG_STANDARD_H264) {
		if (pic_params->in_params.bits_per_frm) {
			const int bits_per_gop =
				(rc_params->bits_per_second / rc_params->frame_rate) *
				rc_params->intra_freq;

			pic_params->in_params.mode.h264.rc_scale_factor = (bits_per_gop * 256) /
				(pic_params->in_params.buffer_size -
				pic_params->in_params.initial_level);
		} else {
			pic_params->in_params.mode.h264.rc_scale_factor = 0;
		}
	}
}

void topaz_setup_input_format(struct img_video_context *video,
			      struct img_vxe_scaler_setup *scaler_setup)
{
	const unsigned int scaler_coeff_regs_no_crop[] = {4261951490U, 4178589440U,
						    4078580480U, 4045614080U};

	if (video->enable_scaler) {
		unsigned int pitch_x, pitch_y;
		int phase;

		pitch_x = (((unsigned int)(video->source_width - video->crop_left -
			video->crop_right)) << 13) / video->unrounded_width;

		pitch_y = (((unsigned int)(video->source_frame_height - video->crop_top -
			video->crop_bottom)) << 13) / video->unrounded_frame_height;

		/* Input size */
		scaler_setup->scaler_input_size_reg =
			F_ENCODE(video->source_width - 1,
				 TOPAZHP_EXT_CR_SCALER_INPUT_WIDTH_MIN1) |
			F_ENCODE((video->source_frame_height >>
				 (video->is_interlaced ? 1 : 0)) - 1,
				  TOPAZHP_EXT_CR_SCALER_INPUT_HEIGHT_MIN1);

		scaler_setup->scaler_crop_reg = F_ENCODE(video->crop_left,
							 TOPAZHP_EXT_CR_SCALER_INPUT_CROP_HOR) |
						F_ENCODE(video->crop_top,
							 TOPAZHP_EXT_CR_SCALER_INPUT_CROP_VER);

		/* Scale factors */
		scaler_setup->scaler_pitch_reg = 0;

		if (pitch_x > 0x7FFF) {
			scaler_setup->scaler_pitch_reg |= F_ENCODE(1,
					TOPAZHP_EXT_CR_SCALER_HOR_BILINEAR_FILTER);
			pitch_x >>= 1;
		}

		if (pitch_x > 0x7FFF)
			pitch_x = 0x7FFF;

		if (pitch_y > 0x7FFF) {
			scaler_setup->scaler_pitch_reg |= F_ENCODE(1U,
					TOPAZHP_EXT_CR_SCALER_VER_BILINEAR_FILTER);
			pitch_y >>= 1;
		}

		if (pitch_y > 0x7FFF)
			pitch_y = 0x7FFF;

		scaler_setup->scaler_pitch_reg |=
			F_ENCODE(pitch_x, TOPAZHP_EXT_CR_SCALER_INPUT_HOR_PITCH) |
			F_ENCODE(pitch_y, TOPAZHP_EXT_CR_SCALER_INPUT_VER_PITCH);

		/*
		 * Coefficients
		 * With no crop, the coefficients remain the same.
		 * If crop is desired, new values will need to be calculated.
		 */
		for (phase = 0; phase < 4; phase++)
			scaler_setup->hor_scaler_coeff_regs[phase] =
				scaler_coeff_regs_no_crop[phase];

		for (phase = 0; phase < 4; phase++)
			scaler_setup->ver_scaler_coeff_regs[phase] =
				scaler_coeff_regs_no_crop[phase];

		scaler_setup->scaler_control = F_ENCODE(1, TOPAZHP_EXT_CR_SCALER_ENABLE);

		switch (video->format) {
		case IMG_CODEC_420_YUV:
		case IMG_CODEC_420_PL8:
		case IMG_CODEC_420_YV12:
		case IMG_CODEC_420_IMC2:
			scaler_setup->input_scaler_control =
				F_ENCODE(TOPAZHP_EXT_CR_INPUT_FRAME_STORE_FORMAT_420PL111YCBCR8,
					 TOPAZHP_EXT_CR_INPUT_FRAME_STORE_FORMAT);
			break;
		case IMG_CODEC_420_PL12:
		case IMG_CODEC_420_PL12_PACKED:
			scaler_setup->input_scaler_control =
				F_ENCODE(TOPAZHP_EXT_CR_INPUT_FRAME_STORE_FORMAT_420PL12YCBCR8,
					 TOPAZHP_EXT_CR_INPUT_FRAME_STORE_FORMAT);
			break;
		case IMG_CODEC_420_PL21:
		case IMG_CODEC_420_PL21_PACKED:
			scaler_setup->input_scaler_control =
				F_ENCODE(TOPAZHP_EXT_CR_INPUT_FRAME_STORE_FORMAT_420PL12YCRCB8,
					 TOPAZHP_EXT_CR_INPUT_FRAME_STORE_FORMAT);
			break;
		case IMG_CODEC_422_YUV:
		case IMG_CODEC_422_PL8:
		case IMG_CODEC_422_YV12:
		case IMG_CODEC_422_IMC2:
			scaler_setup->input_scaler_control =
				F_ENCODE(TOPAZHP_EXT_CR_INPUT_FRAME_STORE_FORMAT_422PL111YCBCR8,
					 TOPAZHP_EXT_CR_INPUT_FRAME_STORE_FORMAT);
			break;
		case IMG_CODEC_422_PL12:
			scaler_setup->input_scaler_control =
				F_ENCODE(TOPAZHP_EXT_CR_INPUT_FRAME_STORE_FORMAT_422PL12YCBCR8,
					 TOPAZHP_EXT_CR_INPUT_FRAME_STORE_FORMAT);
			break;
		case IMG_CODEC_422_PL21:
			scaler_setup->input_scaler_control =
				F_ENCODE(TOPAZHP_EXT_CR_INPUT_FRAME_STORE_FORMAT_422PL12YCRCB8,
					 TOPAZHP_EXT_CR_INPUT_FRAME_STORE_FORMAT);
			break;
		case IMG_CODEC_Y0UY1V_8888:
			scaler_setup->input_scaler_control =
				F_ENCODE(TOPAZHP_EXT_CR_INPUT_FRAME_STORE_FORMAT_422IL3YCBYCR8,
					 TOPAZHP_EXT_CR_INPUT_FRAME_STORE_FORMAT);
			break;
		case IMG_CODEC_Y0VY1U_8888:
			scaler_setup->input_scaler_control =
				F_ENCODE(TOPAZHP_EXT_CR_INPUT_FRAME_STORE_FORMAT_422IL3YCRYCB8,
					 TOPAZHP_EXT_CR_INPUT_FRAME_STORE_FORMAT);
			break;
		case IMG_CODEC_UY0VY1_8888:
			scaler_setup->input_scaler_control =
				F_ENCODE(TOPAZHP_EXT_CR_INPUT_FRAME_STORE_FORMAT_422IL3CBYCRY8,
					 TOPAZHP_EXT_CR_INPUT_FRAME_STORE_FORMAT);
			break;
		case IMG_CODEC_VY0UY1_8888:
			scaler_setup->input_scaler_control =
				F_ENCODE(TOPAZHP_EXT_CR_INPUT_FRAME_STORE_FORMAT_422IL3CRYCBY8,
					 TOPAZHP_EXT_CR_INPUT_FRAME_STORE_FORMAT);
			break;
		case IMG_CODEC_444_YUV:
		case IMG_CODEC_444_PL8:
		case IMG_CODEC_444_YV12:
		case IMG_CODEC_444_IMC2:
			scaler_setup->input_scaler_control =
				F_ENCODE(TOPAZHP_EXT_CR_INPUT_FRAME_STORE_FORMAT_444PL111YCBCR8,
					 TOPAZHP_EXT_CR_INPUT_FRAME_STORE_FORMAT);
			break;
		case IMG_CODEC_444_PL12:
			scaler_setup->input_scaler_control =
				F_ENCODE(TOPAZHP_EXT_CR_INPUT_FRAME_STORE_FORMAT_444PL12YCBCR8,
					 TOPAZHP_EXT_CR_INPUT_FRAME_STORE_FORMAT);
			break;
		case IMG_CODEC_444_PL21:
			scaler_setup->input_scaler_control =
				F_ENCODE(TOPAZHP_EXT_CR_INPUT_FRAME_STORE_FORMAT_444PL12YCRCB8,
					 TOPAZHP_EXT_CR_INPUT_FRAME_STORE_FORMAT);
			break;
		case IMG_CODEC_ABCX:
			scaler_setup->input_scaler_control =
				F_ENCODE(TOPAZHP_EXT_CR_INPUT_FRAME_STORE_FORMAT_444IL4ABCX8,
					 TOPAZHP_EXT_CR_INPUT_FRAME_STORE_FORMAT);
			break;
		case IMG_CODEC_XBCA:
			scaler_setup->input_scaler_control =
				F_ENCODE(TOPAZHP_EXT_CR_INPUT_FRAME_STORE_FORMAT_444IL4XBCA8,
					 TOPAZHP_EXT_CR_INPUT_FRAME_STORE_FORMAT);
			break;
		case IMG_CODEC_ABC565:
			scaler_setup->input_scaler_control =
				F_ENCODE(TOPAZHP_EXT_CR_INPUT_FRAME_STORE_FORMAT_444IL3RGB565,
					 TOPAZHP_EXT_CR_INPUT_FRAME_STORE_FORMAT);
			break;
		default:
			break;
		}
	} else {
		/* Disable Scaling */
		scaler_setup->scaler_control = 0;
	}
}
