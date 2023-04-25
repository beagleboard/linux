// SPDX-License-Identifier: GPL-2.0
/*
 * VXD DEC Common low level core interface component
 *
 * Copyright (c) Imagination Technologies Ltd.
 * Copyright (c) 2021 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Authors:
 *	Amit Makani <amit.makani@ti.com>
 *
 * Re-written for upstreamimg
 *	Sidraya Jayagond <sidraya.bj@pathpartnertech.com>
 *	Prashanth Kumar Amai <prashanth.ka@pathpartnertech.com>
 */

#include <linux/types.h>
#include <linux/dma-mapping.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-mem2mem.h>

#include "bspp.h"
#include "fw_interface.h"
#include "h264fw_data.h"
#include "img_errors.h"
#include "img_dec_common.h"
#include "img_pvdec_core_regs.h"
#include "img_pvdec_pixel_regs.h"
#include "img_pvdec_test_regs.h"
#include "img_vdec_fw_msg.h"
#include "img_video_bus4_mmu_regs.h"
#include "img_msvdx_core_regs.h"
#include "img_msvdx_cmds.h"
#include "reg_io2.h"
#include "scaler_setup.h"
#include "vdecdd_defs.h"
#include "vdecdd_utils.h"
#include "vdecfw_shared.h"
#include "vdec_defs.h"
#include "vxd_ext.h"
#include "vxd_int.h"
#include "vxd_props.h"

#define MSVDX_CACHE_REF_OFFSET_V100     (72L)
#define MSVDX_CACHE_ROW_OFFSET_V100     (4L)

#define MSVDX_CACHE_REF_OFFSET_V550     (144L)
#define MSVDX_CACHE_ROW_OFFSET_V550     (8L)

#define GET_BITS(v, lb, n)       (((v) >> (lb)) & ((1 << (n)) - 1))
#define IS_PVDEC_PIPELINE(std)   ((std) == VDEC_STD_HEVC ? 1 : 0)

static int amsvdx_codecmode[VDEC_STD_MAX] = {
	/* Invalid */
	-1,
	/* MPEG2 */
	3,
	/* MPEG4 */
	4,
	/* H263 */
	4,
	/* H264 */
	1,
	/* VC1 */
	2,
	/* AVS */
	5,
	/* RealVideo (8) */
	8,
	/* JPEG */
	0,
	/* On2 VP6 */
	10,
	/* On2 VP8 */
	11,
	/* Invalid */
#ifdef HAS_VP9
	/* On2 VP9 */
	13,
#endif
	/* Sorenson */
	4,
	/* HEVC */
	12,
};

struct msvdx_scaler_coeff_cmds {
	unsigned int acmd_horizluma_coeff[VDECFW_NUM_SCALE_COEFFS];
	unsigned int acmd_vertluma_coeff[VDECFW_NUM_SCALE_COEFFS];
	unsigned int acmd_horizchroma_coeff[VDECFW_NUM_SCALE_COEFFS];
	unsigned int acmd_vertchroma_coeff[VDECFW_NUM_SCALE_COEFFS];
};

static struct vxd_vidstd_props astd_props[] = {
	{ VDEC_STD_MPEG2, CORE_REVISION(7, 0, 0), 64, 16, 4096, 4096, 0, 8, 8,
	  PIXEL_FORMAT_420 },
	{ VDEC_STD_MPEG4, CORE_REVISION(7, 0, 0), 64, 16, 4096, 4096, 0, 8, 8,
	  PIXEL_FORMAT_420 },
	{ VDEC_STD_H263, CORE_REVISION(7, 0, 0), 64, 16, 4096, 4096, 0, 8, 8,
	  PIXEL_FORMAT_420 },
	{ VDEC_STD_H264, CORE_REVISION(7, 0, 0), 64, 16, 4096, 4096, 0x10000, 8,
	  8, PIXEL_FORMAT_420 },
	{ VDEC_STD_VC1, CORE_REVISION(7, 0, 0), 80, 16, 4096, 4096, 0, 8, 8,
	  PIXEL_FORMAT_420 },
	{ VDEC_STD_AVS, CORE_REVISION(7, 0, 0), 64, 16, 4096, 4096, 0, 8, 8,
	  PIXEL_FORMAT_420 },
	{ VDEC_STD_REAL, CORE_REVISION(7, 0, 0), 64, 16, 4096, 4096, 0, 8, 8,
	  PIXEL_FORMAT_420 },
	{ VDEC_STD_JPEG, CORE_REVISION(7, 0, 0), 64, 16, 32768, 32768, 0, 8, 8,
	  PIXEL_FORMAT_444 },
	{ VDEC_STD_VP6, CORE_REVISION(7, 0, 0), 64, 16, 4096, 4096, 0, 8, 8,
	  PIXEL_FORMAT_420 },
	{ VDEC_STD_VP8, CORE_REVISION(7, 0, 0), 64, 16, 4096, 4096, 0, 8, 8,
	  PIXEL_FORMAT_420 },
	{ VDEC_STD_SORENSON, CORE_REVISION(7, 0, 0), 64, 16, 4096, 4096, 0, 8,
	  8, PIXEL_FORMAT_420 },
	{ VDEC_STD_HEVC, CORE_REVISION(7, 0, 0), 64, 16, 8192, 8192, 0, 8, 8,
	  PIXEL_FORMAT_420 },
};

enum vdec_msvdx_async_mode {
	VDEC_MSVDX_ASYNC_NORMAL,
	VDEC_MSVDX_ASYNC_VDMC,
	VDEC_MSVDX_ASYNC_VDEB,
	VDEC_MSVDX_ASYNC_FORCE32BITS = 0x7FFFFFFFU
};

/* MSVDX row strides for video buffers. */
static const unsigned int amsvdx_64byte_row_stride[] = {
	384, 768, 1280, 1920, 512, 1024, 2048, 4096
};

/* MSVDX row strides for jpeg buffers. */
static const unsigned int amsvdx_jpeg_row_stride[] = {
	256, 384, 512, 768, 1024, 1536, 2048, 3072, 4096, 6144, 8192, 12288, 16384, 24576, 32768
};

/* VXD Core major revision. */
static unsigned int maj_rev;
/* VXD Core minor revision. */
static unsigned int min_rev;
/* VXD Core maintenance revision. */
static unsigned int maint_rev;

static int get_stride_code(enum vdec_vid_std vidstd, unsigned int row_stride)
{
	unsigned int i;

	if (vidstd == VDEC_STD_JPEG) {
		for (i = 0; i < (sizeof(amsvdx_jpeg_row_stride) /
			sizeof(amsvdx_jpeg_row_stride[0])); i++) {
			if (amsvdx_jpeg_row_stride[i] == row_stride)
				return i;
		}
	} else {
		for (i = 0; i < (sizeof(amsvdx_64byte_row_stride) /
			sizeof(amsvdx_64byte_row_stride[0])); i++) {
			if (amsvdx_64byte_row_stride[i] == row_stride)
				return i;
		}
	}

	return -1;
}

/* Obtains the hardware defined video profile. */
static unsigned int vxd_getprofile(enum vdec_vid_std vidstd, unsigned int std_profile)
{
	unsigned int profile = 0;

	switch (vidstd) {
	case VDEC_STD_H264:
		switch (std_profile) {
		case H264_PROFILE_BASELINE:
			profile = 0;
			break;

		/*
		 * Extended may be attempted as Baseline or
		 * Main depending on the constraint_set_flags
		 */
		case H264_PROFILE_EXTENDED:
		case H264_PROFILE_MAIN:
			profile = 1;
			break;

		case H264_PROFILE_HIGH:
		case H264_PROFILE_HIGH444:
		case H264_PROFILE_HIGH422:
		case H264_PROFILE_HIGH10:
		case H264_PROFILE_CAVLC444:
		case H264_PROFILE_MVC_HIGH:
		case H264_PROFILE_MVC_STEREO:
			profile = 2;
			break;
		default:
			profile = 2;
			break;
		}
		break;

	default:
		profile = 0;
		break;
	}

	return profile;
}

static int vxd_getcoreproperties(struct vxd_coreprops *coreprops,
				 unsigned int corerev,
				 unsigned int pvdec_coreid, unsigned int mmu_config0,
				 unsigned int mmu_config1, unsigned int *pixel_pipecfg,
				 unsigned int *pixel_misccfg, unsigned int max_framecfg)
{
	unsigned int group_id;
	unsigned int core_id;
	unsigned int core_config;
	unsigned int extended_address_range;
	unsigned char group_size = 0;
	unsigned char pipe_minus1 = 0;
	unsigned int max_h264_hw_chromaformat = 0;
	unsigned int max_hevc_hw_chromaformat = 0;
	unsigned int max_bitdepth_luma = 0;
	unsigned int i;

	struct pvdec_core_rev core_rev;

	if (!coreprops || !pixel_pipecfg || !pixel_misccfg)
		return IMG_ERROR_INVALID_PARAMETERS;

	/* PVDEC Core Revision Information */
	core_rev.maj_rev = REGIO_READ_FIELD(corerev, PVDEC_CORE, CR_PVDEC_CORE_REV,
					    CR_PVDEC_MAJOR_REV);
	core_rev.min_rev = REGIO_READ_FIELD(corerev, PVDEC_CORE, CR_PVDEC_CORE_REV,
					    CR_PVDEC_MINOR_REV);
	core_rev.maint_rev = REGIO_READ_FIELD(corerev, PVDEC_CORE, CR_PVDEC_CORE_REV,
					      CR_PVDEC_MAINT_REV);

	/* core id */
	group_id = REGIO_READ_FIELD(pvdec_coreid, PVDEC_CORE, CR_PVDEC_CORE_ID, CR_GROUP_ID);
	core_id = REGIO_READ_FIELD(pvdec_coreid, PVDEC_CORE, CR_PVDEC_CORE_ID, CR_CORE_ID);

	/* Ensure that the core is IMG Video Decoder (PVDEC). */
	if (group_id != 3 || core_id != 3)
		return IMG_ERROR_DEVICE_NOT_FOUND;

	core_config = REGIO_READ_FIELD(pvdec_coreid, PVDEC_CORE,
				       CR_PVDEC_CORE_ID, CR_PVDEC_CORE_CONFIG);

	memset(coreprops, 0, sizeof(*(coreprops)));

	/*  Construct core version name. */
	snprintf(coreprops->aversion, VER_STR_LEN, "%d.%d.%d",
		 core_rev.maj_rev, core_rev.min_rev, core_rev.maint_rev);

	coreprops->mmu_support_stride_per_context =
			REGIO_READ_FIELD(mmu_config1, IMG_VIDEO_BUS4_MMU,
					 MMU_CONFIG1,
					 SUPPORT_STRIDE_PER_CONTEXT) == 1 ? 1 : 0;

	coreprops->mmu_support_secure = REGIO_READ_FIELD(mmu_config1, IMG_VIDEO_BUS4_MMU,
							 MMU_CONFIG1, SUPPORT_SECURE) == 1 ? 1 : 0;

	extended_address_range = REGIO_READ_FIELD(mmu_config0, IMG_VIDEO_BUS4_MMU,
						  MMU_CONFIG0, EXTENDED_ADDR_RANGE);

	switch (extended_address_range) {
	case 0:
		coreprops->mmu_type = MMU_TYPE_32BIT;
		break;
	case 4:
		coreprops->mmu_type = MMU_TYPE_36BIT;
		break;
	case 8:
		coreprops->mmu_type = MMU_TYPE_40BIT;
		break;
	default:
		return IMG_ERROR_NOT_SUPPORTED;
	}

	group_size += REGIO_READ_FIELD(mmu_config0, IMG_VIDEO_BUS4_MMU,
			MMU_CONFIG0, GROUP_OVERRIDE_SIZE);

	coreprops->num_entropy_pipes = core_config & 0xF;
	coreprops->num_pixel_pipes = core_config >> 4 & 0xF;
#ifdef	DEBUG_DECODER_DRIVER
	pr_info("PVDEC revision %08x detected, id %08x.\n", corerev, core_id);
	pr_info("Found %d entropy pipe(s), %d pixel pipe(s), %d group size",
		coreprops->num_entropy_pipes, coreprops->num_pixel_pipes,
		group_size);
#endif

	/* Set global rev info variables used by macros */
	maj_rev = core_rev.maj_rev;
	min_rev = core_rev.min_rev;
	maint_rev = core_rev.maint_rev;

	/* Default settings */
	for (i = 0; i < ARRAY_SIZE(astd_props); i++) {
		struct vxd_vidstd_props *pvidstd_props =
			&coreprops->vidstd_props[astd_props[i].vidstd];
		/*
		 * Update video standard properties if the core is beyond
		 * specified version and the properties are for newer cores
		 * than the previous.
		 */
		if (FROM_REV(MAJOR_REVISION((int)astd_props[i].core_rev),
			     MINOR_REVISION((int)astd_props[i].core_rev),
			     MAINT_REVISION((int)astd_props[i].core_rev), int) &&
		    astd_props[i].core_rev >= pvidstd_props->core_rev) {
			*pvidstd_props = astd_props[i];

			if (pvidstd_props->vidstd != VDEC_STD_JPEG &&
			    (FROM_REV(8, 0, 0, int)) && (pvidstd_props->vidstd ==
			    VDEC_STD_HEVC ? 1 : 0)) {
				/*
				 * override default values with values
				 * specified in HW (register does not
				 * exist in previous cores)
				 */
				pvidstd_props->max_width =
					2 << REGIO_READ_FIELD(max_framecfg,
						PVDEC_PIXEL,
						CR_MAX_FRAME_CONFIG,
						CR_PVDEC_HOR_MSB);

				pvidstd_props->max_height =
					2 << REGIO_READ_FIELD(max_framecfg,
						PVDEC_PIXEL,
						CR_MAX_FRAME_CONFIG,
						CR_PVDEC_VER_MSB);
			} else if (pvidstd_props->vidstd != VDEC_STD_JPEG &&
				(FROM_REV(8, 0, 0, int))) {
				pvidstd_props->max_width =
					2 << REGIO_READ_FIELD(max_framecfg,
						PVDEC_PIXEL,
						CR_MAX_FRAME_CONFIG,
						CR_MSVDX_HOR_MSB);

				pvidstd_props->max_height =
					2 << REGIO_READ_FIELD(max_framecfg,
						PVDEC_PIXEL,
						CR_MAX_FRAME_CONFIG,
						CR_MSVDX_VER_MSB);
			}
		}
	}

	/* Populate the core properties. */
	if (GET_BITS(core_config, 11, 1))
		coreprops->hd_support = 1;

	for (pipe_minus1 = 0; pipe_minus1 < coreprops->num_pixel_pipes;
		pipe_minus1++) {
		unsigned int current_bitdepth =
			GET_BITS(pixel_misccfg[pipe_minus1], 4, 3) + 8;
		unsigned int current_h264_hw_chromaformat =
			GET_BITS(pixel_misccfg[pipe_minus1], 0, 2);
		unsigned int current_hevc_hw_chromaformat =
			GET_BITS(pixel_misccfg[pipe_minus1], 2, 2);
#ifdef DEBUG_DECODER_DRIVER
		pr_info("cur_bitdepth: %d  cur_h264_hw_chromaformat: %d",
			current_bitdepth, current_h264_hw_chromaformat);
		pr_info("cur_hevc_hw_chromaformat: %d  pipe_minus1: %d\n",
			current_hevc_hw_chromaformat, pipe_minus1);
#endif

		if (GET_BITS(pixel_misccfg[pipe_minus1], 8, 1))
			coreprops->rotation_support[pipe_minus1] = 1;

		if (GET_BITS(pixel_misccfg[pipe_minus1], 9, 1))
			coreprops->scaling_support[pipe_minus1] = 1;

		coreprops->num_streams[pipe_minus1] =
			GET_BITS(pixel_misccfg[pipe_minus1], 12, 2) + 1;

		/* Video standards. */
		coreprops->mpeg2[pipe_minus1] =
			GET_BITS(pixel_pipecfg[pipe_minus1], 0, 1) ? 1 : 0;
		coreprops->mpeg4[pipe_minus1] =
			GET_BITS(pixel_pipecfg[pipe_minus1], 1, 1) ? 1 : 0;
		coreprops->h264[pipe_minus1] =
			GET_BITS(pixel_pipecfg[pipe_minus1], 2, 1) ? 1 : 0;
		coreprops->vc1[pipe_minus1] =
			GET_BITS(pixel_pipecfg[pipe_minus1], 3, 1) ? 1 : 0;
		coreprops->jpeg[pipe_minus1] =
			GET_BITS(pixel_pipecfg[pipe_minus1], 5, 1) ? 1 : 0;
		coreprops->avs[pipe_minus1] =
			GET_BITS(pixel_pipecfg[pipe_minus1], 7, 1) ? 1 : 0;
		coreprops->real[pipe_minus1] =
			GET_BITS(pixel_pipecfg[pipe_minus1], 8, 1) ? 1 : 0;
		coreprops->vp6[pipe_minus1] =
			GET_BITS(pixel_pipecfg[pipe_minus1], 9, 1) ? 1 : 0;
		coreprops->vp8[pipe_minus1] =
			GET_BITS(pixel_pipecfg[pipe_minus1], 10, 1) ? 1 : 0;
		coreprops->hevc[pipe_minus1] =
			GET_BITS(pixel_pipecfg[pipe_minus1], 22, 1) ? 1 : 0;

		max_bitdepth_luma = (max_bitdepth_luma > current_bitdepth ?
			max_bitdepth_luma : current_bitdepth);
		max_h264_hw_chromaformat = (max_h264_hw_chromaformat >
			current_h264_hw_chromaformat ? max_h264_hw_chromaformat
			: current_h264_hw_chromaformat);
		max_hevc_hw_chromaformat = (max_hevc_hw_chromaformat >
			current_hevc_hw_chromaformat ? max_hevc_hw_chromaformat
			: current_hevc_hw_chromaformat);
	}

	/* Override default bit-depth with value signalled explicitly by core. */
	coreprops->vidstd_props[0].max_luma_bitdepth = max_bitdepth_luma;
	coreprops->vidstd_props[0].max_chroma_bitdepth =
		coreprops->vidstd_props[0].max_luma_bitdepth;

	for (i = 1; i < VDEC_STD_MAX; i++) {
		coreprops->vidstd_props[i].max_luma_bitdepth =
			coreprops->vidstd_props[0].max_luma_bitdepth;
		coreprops->vidstd_props[i].max_chroma_bitdepth =
			coreprops->vidstd_props[0].max_chroma_bitdepth;
	}

	switch (max_h264_hw_chromaformat) {
	case 1:
		coreprops->vidstd_props[VDEC_STD_H264].max_chroma_format =
			PIXEL_FORMAT_420;
		break;

	case 2:
		coreprops->vidstd_props[VDEC_STD_H264].max_chroma_format =
			PIXEL_FORMAT_422;
		break;

	case 3:
		coreprops->vidstd_props[VDEC_STD_H264].max_chroma_format =
			PIXEL_FORMAT_444;
		break;

	default:
		break;
	}

	switch (max_hevc_hw_chromaformat) {
	case 1:
		coreprops->vidstd_props[VDEC_STD_HEVC].max_chroma_format =
			PIXEL_FORMAT_420;
		break;

	case 2:
		coreprops->vidstd_props[VDEC_STD_HEVC].max_chroma_format =
			PIXEL_FORMAT_422;
		break;

	case 3:
		coreprops->vidstd_props[VDEC_STD_HEVC].max_chroma_format =
			PIXEL_FORMAT_444;
		break;

	default:
		break;
	}

	return 0;
}

static unsigned char vxd_is_supported_byatleast_onepipe(const unsigned char *features,
							unsigned int num_pipes)
{
	unsigned int i;

	VDEC_ASSERT(features);
	VDEC_ASSERT(num_pipes <= VDEC_MAX_PIXEL_PIPES);

	for (i = 0; i < num_pipes; i++) {
		if (features[i])
			return 1;
	}

	return 0;
}

void vxd_set_reconpictcmds(const struct vdecdd_str_unit *str_unit,
			   const struct vdec_str_configdata *str_configdata,
			   const struct vdec_str_opconfig *output_config,
			   const struct vxd_coreprops *coreprops,
			   const struct vxd_buffers *buffers,
			   unsigned int *pict_cmds)
{
	struct pixel_pixinfo  *pixel_info;
	unsigned int row_stride_code;
	unsigned char benable_auxline_buf = 1;

	unsigned int coded_height;
	unsigned int coded_width;
	unsigned int disp_height;
	unsigned int disp_width;
	unsigned int profile;
	unsigned char plane;
	unsigned int y_stride;
	unsigned int uv_stride;
	unsigned int v_stride;
	unsigned int cache_ref_offset;
	unsigned int cache_row_offset;

	if (str_configdata->vid_std == VDEC_STD_JPEG) {
		disp_height = 0;
		disp_width = 0;
		coded_height = 0;
		coded_width = 0;
	} else {
		coded_height = ALIGN(str_unit->pict_hdr_info->coded_frame_size.height,
				     (str_unit->pict_hdr_info->field) ?
				     2 * VDEC_MB_DIMENSION : VDEC_MB_DIMENSION);
				     /*  Hardware field is coded size - 1 */
				     coded_height -= 1;

		coded_width = ALIGN(str_unit->pict_hdr_info->coded_frame_size.width,
				    VDEC_MB_DIMENSION);
		/*  Hardware field is coded size - 1 */
		coded_width -= 1;

		disp_height = str_unit->pict_hdr_info->disp_info.enc_disp_region.height
			+ str_unit->pict_hdr_info->disp_info.enc_disp_region.left_offset - 1;
		disp_width = str_unit->pict_hdr_info->disp_info.enc_disp_region.width +
			str_unit->pict_hdr_info->disp_info.enc_disp_region.top_offset - 1;
	}
	/*
	 * Display picture size (DISPLAY_PICTURE)
	 * The display to be written is not the actual video size to be
	 * displayed but a number that has to differ from the coded pixel size
	 * by less than 1MB (coded_size-display_size <= 0x0F). Because H264 can
	 * have a different display size, we need to check and write
	 * the coded_size again in the display_size register if this condition
	 * is not fulfilled.
	 */
	if (str_configdata->vid_std != VDEC_STD_VC1 && ((coded_height - disp_height) > 0x0F)) {
		REGIO_WRITE_FIELD_LITE(pict_cmds[VDECFW_CMD_DISPLAY_PICTURE],
				       MSVDX_CMDS, DISPLAY_PICTURE_SIZE,
				       DISPLAY_PICTURE_HEIGHT,
				       coded_height, unsigned int);
	} else {
		REGIO_WRITE_FIELD_LITE(pict_cmds[VDECFW_CMD_DISPLAY_PICTURE],
				       MSVDX_CMDS, DISPLAY_PICTURE_SIZE,
				       DISPLAY_PICTURE_HEIGHT,
				       disp_height, unsigned int);
	}

	if (((coded_width - disp_width) > 0x0F)) {
		REGIO_WRITE_FIELD_LITE(pict_cmds[VDECFW_CMD_DISPLAY_PICTURE],
				       MSVDX_CMDS, DISPLAY_PICTURE_SIZE,
				       DISPLAY_PICTURE_WIDTH,
				       coded_width, unsigned int);
	} else {
		REGIO_WRITE_FIELD_LITE(pict_cmds[VDECFW_CMD_DISPLAY_PICTURE],
				       MSVDX_CMDS, DISPLAY_PICTURE_SIZE,
				       DISPLAY_PICTURE_WIDTH,
				       disp_width, unsigned int);
	}

	REGIO_WRITE_FIELD_LITE(pict_cmds[VDECFW_CMD_CODED_PICTURE],
			       MSVDX_CMDS, CODED_PICTURE_SIZE,
			       CODED_PICTURE_HEIGHT,
			       coded_height, unsigned int);
	REGIO_WRITE_FIELD_LITE(pict_cmds[VDECFW_CMD_CODED_PICTURE],
			       MSVDX_CMDS, CODED_PICTURE_SIZE,
			       CODED_PICTURE_WIDTH,
			       coded_width, unsigned int);

	/*
	 * For standards where dpb_diff != 1 and chroma format != 420
	 * cache_ref_offset has to be calculated in the F/W.
	 */
	if (str_configdata->vid_std != VDEC_STD_HEVC && str_configdata->vid_std != VDEC_STD_H264) {
		unsigned int log2_size, cache_size, luma_size;
		unsigned char is_hevc_supported, is_hevc444_supported = 0;

		is_hevc_supported =
			vxd_is_supported_byatleast_onepipe(coreprops->hevc,
							   coreprops->num_pixel_pipes);

		if (is_hevc_supported) {
			is_hevc444_supported =
				coreprops->vidstd_props[VDEC_STD_HEVC].max_chroma_format ==
				PIXEL_FORMAT_444 ? 1 : 0;
		}

		log2_size = 9 + (is_hevc_supported ? 1 : 0) + (is_hevc444_supported ? 1 : 0);
		cache_size = 3 << log2_size;
		luma_size = (cache_size * 2) / 3;
		cache_ref_offset = (luma_size * 15) / 32;
		cache_ref_offset = (cache_ref_offset + 7) & (~7);
		cache_row_offset = 0x0C;

		REGIO_WRITE_FIELD(pict_cmds[VDECFW_CMD_MC_CACHE_CONFIGURATION],
				  MSVDX_CMDS, MC_CACHE_CONFIGURATION,
				  CONFIG_REF_CHROMA_ADJUST, 1,
				  unsigned int, unsigned int);
		REGIO_WRITE_FIELD(pict_cmds[VDECFW_CMD_MC_CACHE_CONFIGURATION],
				  MSVDX_CMDS, MC_CACHE_CONFIGURATION,
				  CONFIG_REF_OFFSET, cache_ref_offset,
				  unsigned int, unsigned int);
		REGIO_WRITE_FIELD(pict_cmds[VDECFW_CMD_MC_CACHE_CONFIGURATION],
				  MSVDX_CMDS, MC_CACHE_CONFIGURATION,
				  CONFIG_ROW_OFFSET, cache_row_offset,
				  unsigned int, unsigned int);
	}

	REGIO_WRITE_FIELD(pict_cmds[VDECFW_CMD_OPERATING_MODE],
			  MSVDX_CMDS, OPERATING_MODE, CODEC_MODE,
			  amsvdx_codecmode[str_configdata->vid_std],
			  unsigned int, unsigned int);

	profile = str_unit->seq_hdr_info->com_sequ_hdr_info.codec_profile;
	REGIO_WRITE_FIELD(pict_cmds[VDECFW_CMD_OPERATING_MODE],
			  MSVDX_CMDS, OPERATING_MODE, CODEC_PROFILE,
			  vxd_getprofile(str_configdata->vid_std, profile),
			  unsigned int, unsigned int);

	plane = str_unit->seq_hdr_info->com_sequ_hdr_info.separate_chroma_planes;
	pixel_info = &str_unit->seq_hdr_info->com_sequ_hdr_info.pixel_info;
	REGIO_WRITE_FIELD(pict_cmds[VDECFW_CMD_OPERATING_MODE],
			  MSVDX_CMDS, OPERATING_MODE, CHROMA_FORMAT, plane ?
			  0 : pixel_info->chroma_fmt, unsigned int, int);

	if (str_configdata->vid_std != VDEC_STD_JPEG) {
		REGIO_WRITE_FIELD(pict_cmds[VDECFW_CMD_EXT_OP_MODE],
				  MSVDX_CMDS, EXT_OP_MODE, CHROMA_FORMAT_IDC, plane ?
				  0 : pixel_get_hw_chroma_format_idc
							(pixel_info->chroma_fmt_idc),
				  unsigned int, int);

		REGIO_WRITE_FIELD(pict_cmds[VDECFW_CMD_EXT_OP_MODE],
				  MSVDX_CMDS, EXT_OP_MODE, MEMORY_PACKING,
				  output_config->pixel_info.mem_pkg ==
				  PIXEL_BIT10_MP ? 1 : 0, unsigned int, int);

		REGIO_WRITE_FIELD(pict_cmds[VDECFW_CMD_EXT_OP_MODE],
				  MSVDX_CMDS, EXT_OP_MODE, BIT_DEPTH_LUMA_MINUS8,
				  pixel_info->bitdepth_y - 8,
				  unsigned int, unsigned int);

		if (pixel_info->chroma_fmt_idc == PIXEL_FORMAT_MONO) {
			/*
			 * For monochrome streams use the same bit depth for
			 * chroma and luma.
			 */
			REGIO_WRITE_FIELD(pict_cmds[VDECFW_CMD_EXT_OP_MODE],
					  MSVDX_CMDS, EXT_OP_MODE,
					  BIT_DEPTH_CHROMA_MINUS8,
					  pixel_info->bitdepth_y - 8,
					  unsigned int, unsigned int);
		} else {
			/*
			 * For normal streams use the appropriate bit depth for chroma.
			 */
			REGIO_WRITE_FIELD(pict_cmds[VDECFW_CMD_EXT_OP_MODE], MSVDX_CMDS,
					  EXT_OP_MODE, BIT_DEPTH_CHROMA_MINUS8,
					  pixel_info->bitdepth_c - 8,
					  unsigned int, unsigned int);
		}
	} else {
		pict_cmds[VDECFW_CMD_EXT_OP_MODE] = 0;
	}

	if (str_configdata->vid_std != VDEC_STD_JPEG) {
		REGIO_WRITE_FIELD(pict_cmds[VDECFW_CMD_OPERATING_MODE], MSVDX_CMDS,
				  OPERATING_MODE, CHROMA_INTERLEAVED,
				  PIXEL_GET_HW_CHROMA_INTERLEAVED
				  (output_config->pixel_info.chroma_interleave),
				  unsigned int, int);
	}

	if (str_configdata->vid_std == VDEC_STD_JPEG) {
		REGIO_WRITE_FIELD(pict_cmds[VDECFW_CMD_OPERATING_MODE],
				  MSVDX_CMDS, OPERATING_MODE, ASYNC_MODE,
				  VDEC_MSVDX_ASYNC_VDMC,
				  unsigned int, unsigned int);
	}

	if (str_configdata->vid_std == VDEC_STD_H264) {
		REGIO_WRITE_FIELD(pict_cmds[VDECFW_CMD_OPERATING_MODE], MSVDX_CMDS,
				  OPERATING_MODE, ASYNC_MODE,
				  str_unit->pict_hdr_info->discontinuous_mbs ?
				  VDEC_MSVDX_ASYNC_VDMC : VDEC_MSVDX_ASYNC_NORMAL,
				  unsigned int, int);
	}

	y_stride = buffers->recon_pict->rend_info.plane_info[VDEC_PLANE_VIDEO_Y].stride;
	uv_stride = buffers->recon_pict->rend_info.plane_info[VDEC_PLANE_VIDEO_UV].stride;
	v_stride = buffers->recon_pict->rend_info.plane_info[VDEC_PLANE_VIDEO_V].stride;

	if (((y_stride % (VDEC_VXD_EXT_STRIDE_ALIGNMENT_DEFAULT)) == 0) &&
	    ((uv_stride % (VDEC_VXD_EXT_STRIDE_ALIGNMENT_DEFAULT)) == 0) &&
	    ((v_stride % (VDEC_VXD_EXT_STRIDE_ALIGNMENT_DEFAULT)) == 0)) {
		REGIO_WRITE_FIELD(pict_cmds[VDECFW_CMD_OPERATING_MODE],
				  MSVDX_CMDS, OPERATING_MODE,
				  USE_EXT_ROW_STRIDE, 1, unsigned int, int);

		REGIO_WRITE_FIELD(pict_cmds[VDECFW_CMD_EXTENDED_ROW_STRIDE],
				  MSVDX_CMDS, EXTENDED_ROW_STRIDE,
				  EXT_ROW_STRIDE, y_stride >> 6, unsigned int, unsigned int);

		REGIO_WRITE_FIELD(pict_cmds[VDECFW_CMD_CHROMA_ROW_STRIDE],
				  MSVDX_CMDS, CHROMA_ROW_STRIDE,
				  CHROMA_ROW_STRIDE, uv_stride >> 6, unsigned int, unsigned int);
	} else {
		row_stride_code = get_stride_code(str_configdata->vid_std, y_stride);

		REGIO_WRITE_FIELD(pict_cmds[VDECFW_CMD_OPERATING_MODE],
				  MSVDX_CMDS, OPERATING_MODE, ROW_STRIDE,
				  row_stride_code & 0x7, unsigned int, unsigned int);

		if (str_configdata->vid_std == VDEC_STD_JPEG) {
			/*
			 * Use the unused chroma interleaved flag
			 * to hold MSB of row stride code
			 */
			IMG_ASSERT(row_stride_code < 16);
			REGIO_WRITE_FIELD(pict_cmds[VDECFW_CMD_OPERATING_MODE],
					  MSVDX_CMDS, OPERATING_MODE,
					  CHROMA_INTERLEAVED,
					  row_stride_code >> 3, unsigned int, unsigned int);
		} else {
			IMG_ASSERT(row_stride_code < 8);
		}
	}
	pict_cmds[VDECFW_CMD_LUMA_RECONSTRUCTED_PICTURE_BASE_ADDRESS] =
		(unsigned int)GET_HOST_ADDR(&buffers->recon_pict->pict_buf->ddbuf_info) +
		buffers->recon_pict->rend_info.plane_info[0].offset;

	pict_cmds[VDECFW_CMD_CHROMA_RECONSTRUCTED_PICTURE_BASE_ADDRESS] =
		(unsigned int)GET_HOST_ADDR(&buffers->recon_pict->pict_buf->ddbuf_info) +
		buffers->recon_pict->rend_info.plane_info[1].offset;

	pict_cmds[VDECFW_CMD_CHROMA2_RECONSTRUCTED_PICTURE_BASE_ADDRESS] =
		(unsigned int)GET_HOST_ADDR(&buffers->recon_pict->pict_buf->ddbuf_info) +
		buffers->recon_pict->rend_info.plane_info[2].offset;

	pict_cmds[VDECFW_CMD_LUMA_ERROR_PICTURE_BASE_ADDRESS] = 0;
	pict_cmds[VDECFW_CMD_CHROMA_ERROR_PICTURE_BASE_ADDRESS] = 0;

#ifdef ERROR_CONCEALMENT
	/* update error concealment frame info if available */
	if (buffers->err_pict_bufinfo) {
		pict_cmds[VDECFW_CMD_LUMA_ERROR_PICTURE_BASE_ADDRESS] =
			(unsigned int)GET_HOST_ADDR(buffers->err_pict_bufinfo) +
			buffers->recon_pict->rend_info.plane_info[0].offset;

		pict_cmds[VDECFW_CMD_CHROMA_ERROR_PICTURE_BASE_ADDRESS] =
			(unsigned int)GET_HOST_ADDR(buffers->err_pict_bufinfo) +
			buffers->recon_pict->rend_info.plane_info[1].offset;
	}
#endif

	pict_cmds[VDECFW_CMD_INTRA_BUFFER_BASE_ADDRESS] =
		(unsigned int)GET_HOST_ADDR(buffers->intra_bufinfo);
	pict_cmds[VDECFW_CMD_INTRA_BUFFER_PLANE_SIZE] =
		buffers->intra_bufsize_per_pipe / 3;
	pict_cmds[VDECFW_CMD_INTRA_BUFFER_SIZE_PER_PIPE] =
		buffers->intra_bufsize_per_pipe;
	pict_cmds[VDECFW_CMD_AUX_LINE_BUFFER_BASE_ADDRESS] =
		(unsigned int)GET_HOST_ADDR(buffers->auxline_bufinfo);
	pict_cmds[VDECFW_CMD_AUX_LINE_BUFFER_SIZE_PER_PIPE] =
		buffers->auxline_bufsize_per_pipe;

	/*
	 * for pvdec we need to set this registers even if we don't
	 * use alternative output
	 */
	REGIO_WRITE_FIELD(pict_cmds[VDECFW_CMD_ALTERNATIVE_OUTPUT_CONTROL],
			  MSVDX_CMDS, ALTERNATIVE_OUTPUT_CONTROL,
			  ALT_BIT_DEPTH_CHROMA_MINUS8,
			  output_config->pixel_info.bitdepth_c - 8, unsigned int, unsigned int);
	REGIO_WRITE_FIELD(pict_cmds[VDECFW_CMD_ALTERNATIVE_OUTPUT_CONTROL],
			  MSVDX_CMDS, ALTERNATIVE_OUTPUT_CONTROL,
			  ALT_BIT_DEPTH_LUMA_MINUS8,
			  output_config->pixel_info.bitdepth_y - 8, unsigned int, unsigned int);

	/*
	 * this is causing corruption in RV40 and VC1 streams with
	 * scaling/rotation enabled on Coral, so setting to 0
	 */
	benable_auxline_buf = benable_auxline_buf &&
		(str_configdata->vid_std != VDEC_STD_REAL) &&
		(str_configdata->vid_std != VDEC_STD_VC1);

	REGIO_WRITE_FIELD(pict_cmds[VDECFW_CMD_ALTERNATIVE_OUTPUT_PICTURE_ROTATION],
			  MSVDX_CMDS, ALTERNATIVE_OUTPUT_PICTURE_ROTATION,
			  USE_AUX_LINE_BUF, benable_auxline_buf ? 1 : 0, unsigned int, int);
}

void vxd_set_altpictcmds(const struct vdecdd_str_unit *str_unit,
			 const struct vdec_str_configdata *str_configdata,
			 const struct vdec_str_opconfig *output_config,
			 const struct vxd_coreprops *coreprops,
			 const struct vxd_buffers *buffers,
			 unsigned int *pict_cmds)
{
	unsigned int row_stride_code;
	unsigned int y_stride;
	unsigned int uv_stride;
	unsigned int v_stride;

	y_stride = buffers->alt_pict->rend_info.plane_info[VDEC_PLANE_VIDEO_Y].stride;
	uv_stride = buffers->alt_pict->rend_info.plane_info[VDEC_PLANE_VIDEO_UV].stride;
	v_stride = buffers->alt_pict->rend_info.plane_info[VDEC_PLANE_VIDEO_V].stride;

	if (((y_stride % (VDEC_VXD_EXT_STRIDE_ALIGNMENT_DEFAULT)) == 0) &&
	    ((uv_stride % (VDEC_VXD_EXT_STRIDE_ALIGNMENT_DEFAULT)) == 0) &&
	    ((v_stride % (VDEC_VXD_EXT_STRIDE_ALIGNMENT_DEFAULT)) == 0)) {
		REGIO_WRITE_FIELD(pict_cmds[VDECFW_CMD_ALTERNATIVE_OUTPUT_PICTURE_ROTATION],
				  MSVDX_CMDS, ALTERNATIVE_OUTPUT_PICTURE_ROTATION,
				  USE_EXT_ROT_ROW_STRIDE, 1, unsigned int, int);

		/* 64-byte (min) aligned luma stride value. */
		REGIO_WRITE_FIELD(pict_cmds[VDECFW_CMD_ALTERNATIVE_OUTPUT_PICTURE_ROTATION],
				  MSVDX_CMDS,
				  ALTERNATIVE_OUTPUT_PICTURE_ROTATION,
				  EXT_ROT_ROW_STRIDE, y_stride >> 6,
				  unsigned int, unsigned int);

		/* 64-byte (min) aligned chroma stride value. */
		REGIO_WRITE_FIELD(pict_cmds[VDECFW_CMD_CHROMA_ROW_STRIDE],
				  MSVDX_CMDS, CHROMA_ROW_STRIDE,
				  ALT_CHROMA_ROW_STRIDE, uv_stride >> 6,
				  unsigned int, unsigned int);
	} else {
		/*
		 * Obtain the code for buffer stride
		 * (must be less than 8, i.e. not JPEG strides)
		 */
		row_stride_code =
			get_stride_code(str_configdata->vid_std, y_stride);

		REGIO_WRITE_FIELD(pict_cmds[VDECFW_CMD_ALTERNATIVE_OUTPUT_PICTURE_ROTATION],
				  MSVDX_CMDS,
				  ALTERNATIVE_OUTPUT_PICTURE_ROTATION,
				  ROTATION_ROW_STRIDE, row_stride_code & 0x7,
				  unsigned int, unsigned int);
	}

	REGIO_WRITE_FIELD(pict_cmds[VDECFW_CMD_ALTERNATIVE_OUTPUT_PICTURE_ROTATION],
			  MSVDX_CMDS, ALTERNATIVE_OUTPUT_PICTURE_ROTATION,
			  SCALE_INPUT_SIZE_SEL,
			  ((output_config->pixel_info.chroma_fmt_idc !=
			  str_unit->seq_hdr_info->com_sequ_hdr_info.pixel_info.chroma_fmt_idc)) ?
			  1 : 0, unsigned int, int);

	REGIO_WRITE_FIELD(pict_cmds[VDECFW_CMD_ALTERNATIVE_OUTPUT_PICTURE_ROTATION],
			  MSVDX_CMDS, ALTERNATIVE_OUTPUT_PICTURE_ROTATION,
			  PACKED_422_OUTPUT,
			  (output_config->pixel_info.chroma_fmt_idc ==
			  PIXEL_FORMAT_422 &&
			  output_config->pixel_info.num_planes == 1) ? 1 : 0,
			  unsigned int, int);

	REGIO_WRITE_FIELD(pict_cmds[VDECFW_CMD_ALTERNATIVE_OUTPUT_CONTROL],
			  MSVDX_CMDS, ALTERNATIVE_OUTPUT_CONTROL,
			  ALT_OUTPUT_FORMAT,
			  str_unit->seq_hdr_info->com_sequ_hdr_info.separate_chroma_planes ?
			  0 : pixel_get_hw_chroma_format_idc
					(output_config->pixel_info.chroma_fmt_idc),
			  unsigned int, int);

	REGIO_WRITE_FIELD(pict_cmds[VDECFW_CMD_ALTERNATIVE_OUTPUT_CONTROL],
			  MSVDX_CMDS, ALTERNATIVE_OUTPUT_CONTROL,
			  ALT_BIT_DEPTH_CHROMA_MINUS8,
			  output_config->pixel_info.bitdepth_c - 8,
			  unsigned int, unsigned int);
	REGIO_WRITE_FIELD(pict_cmds[VDECFW_CMD_ALTERNATIVE_OUTPUT_CONTROL],
			  MSVDX_CMDS, ALTERNATIVE_OUTPUT_CONTROL,
			  ALT_BIT_DEPTH_LUMA_MINUS8,
			  output_config->pixel_info.bitdepth_y - 8,
			  unsigned int, unsigned int);
	REGIO_WRITE_FIELD(pict_cmds[VDECFW_CMD_ALTERNATIVE_OUTPUT_CONTROL],
			  MSVDX_CMDS, ALTERNATIVE_OUTPUT_CONTROL,
			  ALT_MEMORY_PACKING,
			  (output_config->pixel_info.mem_pkg ==
			  PIXEL_BIT10_MP) ? 1 : 0, unsigned int, int);

	pict_cmds[VDECFW_CMD_LUMA_ALTERNATIVE_PICTURE_BASE_ADDRESS] =
		(unsigned int)GET_HOST_ADDR(&buffers->alt_pict->pict_buf->ddbuf_info) +
		buffers->alt_pict->rend_info.plane_info[0].offset;

	pict_cmds[VDECFW_CMD_CHROMA_ALTERNATIVE_PICTURE_BASE_ADDRESS] =
		(unsigned int)GET_HOST_ADDR(&buffers->alt_pict->pict_buf->ddbuf_info) +
		buffers->alt_pict->rend_info.plane_info[1].offset;

	pict_cmds[VDECFW_CMD_CHROMA2_ALTERNATIVE_PICTURE_BASE_ADDRESS] =
		(unsigned int)GET_HOST_ADDR(&buffers->alt_pict->pict_buf->ddbuf_info) +
		buffers->alt_pict->rend_info.plane_info[2].offset;
}

int vxd_getscalercmds(const struct scaler_config *scaler_config,
		      const struct scaler_pitch *pitch,
		      const struct scaler_filter *filter,
		      const struct pixel_pixinfo *out_loop_pixel_info,
		      struct scaler_params *params,
		      unsigned int *pict_cmds)
{
	const struct vxd_coreprops *coreprops = scaler_config->coreprops;
	/*
	 * Indirectly detect decoder core type (if HEVC is supported, it has
	 * to be PVDEC core) and decide if to force luma re-sampling.
	 */
	unsigned char bforce_luma_resampling = coreprops->hevc[0];

	REGIO_WRITE_FIELD(pict_cmds[VDECFW_CMD_ALTERNATIVE_OUTPUT_CONTROL],
			  MSVDX_CMDS, ALTERNATIVE_OUTPUT_CONTROL,
			  ALT_OUTPUT_FORMAT,
			  scaler_config->bseparate_chroma_planes ? 0 :
			  pixel_get_hw_chroma_format_idc(out_loop_pixel_info->chroma_fmt_idc),
			  unsigned int, int);

	REGIO_WRITE_FIELD(pict_cmds[VDECFW_CMD_ALTERNATIVE_OUTPUT_CONTROL],
			  MSVDX_CMDS, ALTERNATIVE_OUTPUT_CONTROL,
			  SCALE_CHROMA_RESAMP_ONLY, bforce_luma_resampling ? 0 :
			  (pitch->horiz_luma == FIXED(1, HIGHP)) &&
			  (pitch->vert_luma == FIXED(1, HIGHP)), unsigned int, int);

	REGIO_WRITE_FIELD(pict_cmds[VDECFW_CMD_ALTERNATIVE_OUTPUT_CONTROL],
			  MSVDX_CMDS, ALTERNATIVE_OUTPUT_CONTROL, ALT_MEMORY_PACKING,
			  pixel_get_hw_memory_packing(out_loop_pixel_info->mem_pkg),
			  unsigned int, int);

	REGIO_WRITE_FIELD(pict_cmds[VDECFW_CMD_ALTERNATIVE_OUTPUT_CONTROL],
			  MSVDX_CMDS, ALTERNATIVE_OUTPUT_CONTROL,
			  ALT_BIT_DEPTH_LUMA_MINUS8,
			  out_loop_pixel_info->bitdepth_y - 8,
			  unsigned int, unsigned int);

	REGIO_WRITE_FIELD(pict_cmds[VDECFW_CMD_ALTERNATIVE_OUTPUT_CONTROL],
			  MSVDX_CMDS, ALTERNATIVE_OUTPUT_CONTROL,
			  ALT_BIT_DEPTH_CHROMA_MINUS8,
			  out_loop_pixel_info->bitdepth_c - 8,
			  unsigned int, unsigned int);

	/* Scale luma bifilter is always 0 for now */
	REGIO_WRITE_FIELD(pict_cmds[VDECFW_CMD_ALTERNATIVE_OUTPUT_CONTROL],
			  MSVDX_CMDS, ALTERNATIVE_OUTPUT_CONTROL,
			  SCALE_LUMA_BIFILTER_HORIZ,
			  0, unsigned int, int);

	REGIO_WRITE_FIELD(pict_cmds[VDECFW_CMD_ALTERNATIVE_OUTPUT_CONTROL],
			  MSVDX_CMDS, ALTERNATIVE_OUTPUT_CONTROL,
			  SCALE_LUMA_BIFILTER_VERT,
			  0, unsigned int, int);

	REGIO_WRITE_FIELD(pict_cmds[VDECFW_CMD_ALTERNATIVE_OUTPUT_CONTROL],
			  MSVDX_CMDS, ALTERNATIVE_OUTPUT_CONTROL,
			  SCALE_CHROMA_BIFILTER_HORIZ,
			  filter->bhoriz_bilinear ? 1 : 0,
			  unsigned int, int);

	REGIO_WRITE_FIELD(pict_cmds[VDECFW_CMD_ALTERNATIVE_OUTPUT_CONTROL],
			  MSVDX_CMDS, ALTERNATIVE_OUTPUT_CONTROL,
			  SCALE_CHROMA_BIFILTER_VERT,
			   filter->bvert_bilinear ? 1 : 0, unsigned int, int);

	/* for cores 7.x.x and more, precision 3.13 */
	params->fixed_point_shift = 13;

	/* Calculate the fixed-point versions for use by the hardware. */
	params->vert_pitch = (int)((pitch->vert_luma +
		(1 << (HIGHP - params->fixed_point_shift - 1))) >>
		(HIGHP - params->fixed_point_shift));
	params->vert_startpos = params->vert_pitch >> 1;
	params->vert_pitch_chroma = (int)((pitch->vert_chroma +
		(1 << (HIGHP - params->fixed_point_shift - 1))) >>
		(HIGHP - params->fixed_point_shift));
	params->vert_startpos_chroma = params->vert_pitch_chroma >> 1;
	params->horz_pitch = (int)(pitch->horiz_luma >>
		(HIGHP - params->fixed_point_shift));
	params->horz_startpos = params->horz_pitch >> 1;
	params->horz_pitch_chroma = (int)(pitch->horiz_chroma >>
		(HIGHP - params->fixed_point_shift));
	params->horz_startpos_chroma = params->horz_pitch_chroma >> 1;

#ifdef HAS_HEVC
	if (scaler_config->vidstd == VDEC_STD_HEVC) {
		REGIO_WRITE_FIELD(pict_cmds[VDECFW_CMD_SCALED_DISPLAY_SIZE],
				  MSVDX_CMDS, PVDEC_SCALED_DISPLAY_SIZE,
				  PVDEC_SCALE_DISPLAY_WIDTH,
				  scaler_config->recon_width - 1,
				  unsigned int, unsigned int);
		REGIO_WRITE_FIELD(pict_cmds[VDECFW_CMD_SCALED_DISPLAY_SIZE],
				  MSVDX_CMDS, PVDEC_SCALED_DISPLAY_SIZE,
				  PVDEC_SCALE_DISPLAY_HEIGHT,
				  scaler_config->recon_height - 1,
				  unsigned int, unsigned int);
	} else {
		REGIO_WRITE_FIELD(pict_cmds[VDECFW_CMD_SCALED_DISPLAY_SIZE],
				  MSVDX_CMDS, SCALED_DISPLAY_SIZE,
				  SCALE_DISPLAY_WIDTH,
				  scaler_config->recon_width - 1,
				  unsigned int, unsigned int);
		REGIO_WRITE_FIELD(pict_cmds[VDECFW_CMD_SCALED_DISPLAY_SIZE],
				  MSVDX_CMDS, SCALED_DISPLAY_SIZE,
				  SCALE_DISPLAY_HEIGHT,
				  scaler_config->recon_height - 1,
				  unsigned int, unsigned int);
	}
#else
	REGIO_WRITE_FIELD(pict_cmds[VDECFW_CMD_SCALED_DISPLAY_SIZE],
			  MSVDX_CMDS, SCALED_DISPLAY_SIZE,
			  SCALE_DISPLAY_WIDTH,
			  scaler_config->recon_width - 1,
			  unsigned int, unsigned int);
	REGIO_WRITE_FIELD(pict_cmds[VDECFW_CMD_SCALED_DISPLAY_SIZE],
			  MSVDX_CMDS, SCALED_DISPLAY_SIZE, SCALE_DISPLAY_HEIGHT,
			  scaler_config->recon_height - 1,
			  unsigned int, unsigned int);
#endif

	REGIO_WRITE_FIELD(pict_cmds[VDECFW_CMD_SCALE_OUTPUT_SIZE],
			  MSVDX_CMDS, SCALE_OUTPUT_SIZE,
			  SCALE_OUTPUT_WIDTH_MIN1,
			  scaler_config->scale_width - 1,
			  unsigned int, unsigned int);
	REGIO_WRITE_FIELD(pict_cmds[VDECFW_CMD_SCALE_OUTPUT_SIZE],
			  MSVDX_CMDS, SCALE_OUTPUT_SIZE,
			  SCALE_OUTPUT_HEIGHT_MIN1,
			  scaler_config->scale_height - 1,
			  unsigned int, unsigned int);

	REGIO_WRITE_FIELD(pict_cmds[VDECFW_CMD_HORIZONTAL_SCALE_CONTROL],
			  MSVDX_CMDS, HORIZONTAL_SCALE_CONTROL,
			  HORIZONTAL_SCALE_PITCH, params->horz_pitch,
			  unsigned int, unsigned int);
	REGIO_WRITE_FIELD(pict_cmds[VDECFW_CMD_HORIZONTAL_SCALE_CONTROL],
			  MSVDX_CMDS, HORIZONTAL_SCALE_CONTROL,
			  HORIZONTAL_INITIAL_POS, params->horz_startpos,
			  unsigned int, unsigned int);

	REGIO_WRITE_FIELD(pict_cmds[VDECFW_CMD_SCALE_HORIZONTAL_CHROMA],
			  MSVDX_CMDS, SCALE_HORIZONTAL_CHROMA,
			  CHROMA_HORIZONTAL_PITCH, params->horz_pitch_chroma,
			  unsigned int, unsigned int);
	REGIO_WRITE_FIELD(pict_cmds[VDECFW_CMD_SCALE_HORIZONTAL_CHROMA],
			  MSVDX_CMDS, SCALE_HORIZONTAL_CHROMA,
			  CHROMA_HORIZONTAL_INITIAL,
			  params->horz_startpos_chroma,
			  unsigned int, unsigned int);

	REGIO_WRITE_FIELD(pict_cmds[VDECFW_CMD_VERTICAL_SCALE_CONTROL],
			  MSVDX_CMDS, VERTICAL_SCALE_CONTROL,
			  VERTICAL_SCALE_PITCH, params->vert_pitch,
			  unsigned int, unsigned int);
	REGIO_WRITE_FIELD(pict_cmds[VDECFW_CMD_VERTICAL_SCALE_CONTROL],
			  MSVDX_CMDS, VERTICAL_SCALE_CONTROL,
			  VERTICAL_INITIAL_POS, params->vert_startpos,
			  unsigned int, unsigned int);

	REGIO_WRITE_FIELD(pict_cmds[VDECFW_CMD_SCALE_VERTICAL_CHROMA],
			  MSVDX_CMDS, SCALE_VERTICAL_CHROMA,
			  CHROMA_VERTICAL_PITCH, params->vert_pitch_chroma,
			  unsigned int, unsigned int);
	REGIO_WRITE_FIELD(pict_cmds[VDECFW_CMD_SCALE_VERTICAL_CHROMA],
			  MSVDX_CMDS, SCALE_VERTICAL_CHROMA,
			  CHROMA_VERTICAL_INITIAL,
			  params->vert_startpos_chroma,
			  unsigned int, unsigned int);
	return 0;
}

unsigned int vxd_get_codedpicsize(unsigned short width_min1, unsigned short height_min1)
{
	unsigned int reg = 0;

	REGIO_WRITE_FIELD_LITE(reg, MSVDX_CMDS, CODED_PICTURE_SIZE,
			       CODED_PICTURE_WIDTH, width_min1,
			       unsigned short);
	REGIO_WRITE_FIELD_LITE(reg, MSVDX_CMDS, CODED_PICTURE_SIZE,
			       CODED_PICTURE_HEIGHT, height_min1,
			       unsigned short);

	return reg;
}

unsigned char vxd_get_codedmode(enum vdec_vid_std vidstd)
{
	return (unsigned char)amsvdx_codecmode[vidstd];
}

void vxd_get_coreproperties(void *hndl_coreproperties,
			    struct vxd_coreprops *vxd_coreprops)
{
	struct vxd_core_props *props =
		(struct vxd_core_props *)hndl_coreproperties;

	vxd_getcoreproperties(vxd_coreprops, props->core_rev,
			      props->pvdec_core_id,
			      props->mmu_config0,
			      props->mmu_config1,
			      props->pixel_pipe_cfg,
			      props->pixel_misc_cfg,
			      props->pixel_max_frame_cfg);
}

int vxd_get_pictattrs(unsigned int flags, struct vxd_pict_attrs *pict_attrs)
{
	if (flags & (VXD_FW_MSG_FLAG_DWR | VXD_FW_MSG_FLAG_FATAL))
		pict_attrs->dwrfired = 1;
	if (flags & VXD_FW_MSG_FLAG_MMU_FAULT)
		pict_attrs->mmufault = 1;
	if (flags & VXD_FW_MSG_FLAG_DEV_ERR)
		pict_attrs->deverror = 1;

	return 0;
}

int vxd_get_msgerrattr(unsigned int flags, enum vxd_msg_attr *msg_attr)
{
	if ((flags & ~VXD_FW_MSG_FLAG_CANCELED))
		*msg_attr = VXD_MSG_ATTR_FATAL;
	else if ((flags & VXD_FW_MSG_FLAG_CANCELED))
		*msg_attr = VXD_MSG_ATTR_CANCELED;
	else
		*msg_attr = VXD_MSG_ATTR_NONE;

	return 0;
}

int vxd_set_msgflag(enum vxd_msg_flag input_flag, unsigned int *flags)
{
	switch (input_flag) {
	case VXD_MSG_FLAG_DROP:
		*flags |= VXD_FW_MSG_FLAG_DROP;
		break;
	case VXD_MSG_FLAG_EXCL:
		*flags |= VXD_FW_MSG_FLAG_EXCL;
		break;
	default:
		return IMG_ERROR_FATAL;
	}

	return 0;
}
