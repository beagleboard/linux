// SPDX-License-Identifier: GPL-2.0
/*
 * VXD Decoder device driver buffer utility functions implementation
 *
 * Copyright (c) Imagination Technologies Ltd.
 * Copyright (c) 2021 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Authors:
 *	Sunita Nadampalli <sunitan@ti.com>
 *
 * Re-written for upstream
 *	Sidraya Jayagond <sidraya.bj@pathpartnertech.com>
 *	Prashanth Kumar Amai <prashanth.ka@pathpartnertech.com>
 */

#include <linux/dma-mapping.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-mem2mem.h>

#include "img_profiles_levels.h"
#include "pixel_api.h"
#include "vdecdd_utils.h"

/*
 * Tests if chroma offset (immediately after size of luma) is exactly
 * aligned to buffer alignment constraint.
 */
static inline unsigned char is_packedbuf_chroma_aligned(unsigned int offset,
							unsigned int color_plane,
							unsigned int align)
{
	return(color_plane != VDEC_PLANE_VIDEO_Y ? TRUE :
	       (offset == ALIGN(offset, align) ? TRUE : FALSE));
}

/*
 * < h.264 MaxDpbMbs values per profile (see Table A-1 of Rec. ITU-T H.264
 * (03/2010)).
 *   NOTE: Level 1b will be treated as 1.1 in case of Baseline,
 * Constrained Baseline, Main, and Extended profiles as the value of the
 * constraint_set3_flag is not available in #VDEC_sComSequHdrInfo structure.
 */
static unsigned int h264_max_dpb_mbs[H264_LEVEL_MAJOR_NUM][H264_LEVEL_MINOR_NUM] = {
	/* level: n/a     n/a     n/a     1.0b */
	{ 396,    396,    396,    396  },
	/* level: 1.0     1.1     1.2     1.3 */
	{ 396,    900,    2376,   2376 },
	/* level: 2.0     2.1     2.2     n/a */
	{ 2376,   4752,   8100,   8100 },
	/* level: 3.0     3.1     3.2     n/a */
	{ 8100,   18000,  20480,  20480},
	/* level: 4.0     4.1     4.2     n/a */
	{ 32768,  32768,  34816,  34816},
	/* level: 5.0     5.1     5.2     n/a */
	{ 110400, 184320, 184320, 184320}
};

typedef int (*fn_ref_pic_get_max_num)(const struct vdec_comsequ_hdrinfo
	*comseq_hdrinfo, unsigned int *max_ref_pic_num);

void vdecddutils_buf_vxd_adjust_size(unsigned int *buf_size)
{
	/* Align the buffer size to VXD page size. */
	*buf_size = ALIGN(*buf_size, VDEC_VXD_BUF_ALIGNMENT);
}

static int vdecddutils_ref_pic_h264_get_maxnum
	(const struct vdec_comsequ_hdrinfo *comseq_hdrinfo,
	unsigned int *max_ref_pic_num)
{
	unsigned int pic_width_mb;
	unsigned int pic_height_mb;
	unsigned int lvl_major = 0;
	unsigned int lvl_minor = 0;

	/* Pre-validate level. */
	if (comseq_hdrinfo->codec_level < H264_LEVEL_MIN ||
	    comseq_hdrinfo->codec_level > H264_LEVEL_MAX) {
		pr_warn("Wrong H264 level value: %u",
			comseq_hdrinfo->codec_level);
	}

	if (comseq_hdrinfo->max_reorder_picts) {
		*max_ref_pic_num = comseq_hdrinfo->max_reorder_picts;
	} else {
		/* Calculate level major and minor. */
		lvl_major = comseq_hdrinfo->codec_level / 10;
		lvl_minor = comseq_hdrinfo->codec_level % 10;

		/* Calculate picture sizes in MBs. */
		pic_width_mb = (comseq_hdrinfo->max_frame_size.width +
			(VDEC_MB_DIMENSION - 1)) / VDEC_MB_DIMENSION;
		pic_height_mb = (comseq_hdrinfo->max_frame_size.height +
			(VDEC_MB_DIMENSION - 1)) / VDEC_MB_DIMENSION;

		/* Validate lvl_minor */
		if (lvl_minor > 3) {
			pr_warn("Wrong H264 lvl_minor level value: %u, overriding with 3",
				lvl_minor);
			lvl_minor = 3;
		}
		/* Validate lvl_major */
		if (lvl_major > 5) {
			pr_warn("Wrong H264 lvl_major level value: %u, overriding with 5",
				lvl_major);
			lvl_major = 5;
		}

		/*
		 * Calculate the maximum number of reference pictures
		 * required based on level.
		 */
		*max_ref_pic_num = h264_max_dpb_mbs[lvl_major][lvl_minor] /
			(pic_width_mb * pic_height_mb);
		if (*max_ref_pic_num > 16)
			*max_ref_pic_num = 16;
	}

	/* Return success. */
	return IMG_SUCCESS;
}

#ifdef HAS_HEVC
/*
 * @Function              vdecddutils_ref_pic_hevc_get_maxnum
 */
int vdecddutils_ref_pic_hevc_get_maxnum(const struct vdec_comsequ_hdrinfo *comseq_hdrinfo,
					unsigned int *max_ref_picnum)
{
	static const unsigned int HEVC_LEVEL_IDC_MIN = 30;
	static const unsigned int HEVC_LEVEL_IDC_MAX = 186;

	static const unsigned int
		max_luma_ps_list[HEVC_LEVEL_MAJOR_NUM][HEVC_LEVEL_MINOR_NUM] = {
		/* level: 1.0       1.1       1.2       */
		{ 36864,    0,        0,        },
		/* level: 2.0       2.1       2.2       */
		{ 122880,   245760,   0,        },
		/* level: 3.0       3.1       3.2       */
		{ 552960,   983040,   0,        },
		/* level: 4.0       4.1       4.2       */
		{ 2228224,  2228224,  0,        },
		/* level: 5.0       5.1       5.2       */
		{ 8912896,  8912896,  8912896,  },
		/* level: 6.0       6.1       6.2       */
		{ 35651584, 35651584, 35651584, }
	};

	/* ITU-T H.265 04/2013 A.4.1 */

	const unsigned int max_dpb_picbuf = 6;

	/* this is rounded to whole Ctbs */
	unsigned int pic_size_in_samples_Y = comseq_hdrinfo->frame_size.height *
		comseq_hdrinfo->frame_size.width;

	signed char level_maj, level_min;
	unsigned int max_luma_ps;

	/* some error resilience */
	if (comseq_hdrinfo->codec_level > HEVC_LEVEL_IDC_MAX ||
	    comseq_hdrinfo->codec_level < HEVC_LEVEL_IDC_MIN) {
		pr_warn("HEVC Codec level out of range: %u, falling back to %u",
			comseq_hdrinfo->codec_level,
			comseq_hdrinfo->min_pict_buf_num);

		*max_ref_picnum = comseq_hdrinfo->min_pict_buf_num;
		return IMG_SUCCESS;
	}

	level_maj = comseq_hdrinfo->codec_level / 30;
	level_min = (comseq_hdrinfo->codec_level % 30) / 3;

	if (level_maj > 0 && level_maj <= HEVC_LEVEL_MAJOR_NUM &&
	    level_min >= 0 && level_min < HEVC_LEVEL_MINOR_NUM) {
		max_luma_ps = max_luma_ps_list[level_maj - 1][level_min];
	} else {
		pr_err("%s: Invalid parameters\n", __func__);
		return IMG_ERROR_INVALID_PARAMETERS;
	}

	if (max_luma_ps == 0) {
		pr_err("Wrong HEVC level value: %u.%u (general_level_idc: %u)",
		       level_maj, level_min, comseq_hdrinfo->codec_level);

		return IMG_ERROR_VALUE_OUT_OF_RANGE;
	}

	if (max_luma_ps < pic_size_in_samples_Y)
		pr_warn("HEVC PicSizeInSamplesY too large for level (%u > %u)",
			pic_size_in_samples_Y, max_luma_ps);

	if (pic_size_in_samples_Y <= (max_luma_ps >> 2))
		*max_ref_picnum = vdec_size_min(4 * max_dpb_picbuf, 16);
	else if (pic_size_in_samples_Y <= (max_luma_ps >> 1))
		*max_ref_picnum = vdec_size_min(2 * max_dpb_picbuf, 16);
	else if (pic_size_in_samples_Y <= ((3 * max_luma_ps) >> 2))
		*max_ref_picnum = vdec_size_min((4 * max_dpb_picbuf) / 3, 16);
	else
		*max_ref_picnum = max_dpb_picbuf;

	/* Return success. */
	return IMG_SUCCESS;
}
#endif

#ifdef HAS_JPEG
static int vdecddutils_ref_pic_jpeg_get_maxnum(const struct vdec_comsequ_hdrinfo *comseq_hdrinfo,
					       unsigned int *max_ref_picnum)
{
	/* No reference frames for JPEG. */
	*max_ref_picnum = 0;

	/* Return success. */
	return IMG_SUCCESS;
}
#endif

/*
 * The array of pointers to functions calculating the maximum number
 * of reference pictures required for each supported video standard.
 * NOTE: The table is indexed by #VDEC_eVidStd enum values.
 */
static fn_ref_pic_get_max_num ref_pic_get_maxnum[VDEC_STD_MAX - 1] = {
	NULL,
	NULL,
	NULL,
	vdecddutils_ref_pic_h264_get_maxnum,
	NULL,
	NULL,
	NULL,
#ifdef HAS_JPEG
	vdecddutils_ref_pic_jpeg_get_maxnum,
#else
	NULL,
#endif
	NULL,
	NULL,
	NULL,
#ifdef HAS_HEVC
	vdecddutils_ref_pic_hevc_get_maxnum
#else
	NULL
#endif
};

int
vdecddutils_ref_pict_get_maxnum(const struct vdec_str_configdata *str_cfg_data,
				const struct vdec_comsequ_hdrinfo *comseq_hdr_info,
				unsigned int *num_picts)
{
	int ret = IMG_SUCCESS;

	/* Validate input params. */
	if (str_cfg_data->vid_std == VDEC_STD_UNDEFINED || str_cfg_data->vid_std >= VDEC_STD_MAX)
		return IMG_ERROR_VALUE_OUT_OF_RANGE;

	/* Call the function related to the provided video standard. */
	ret = ref_pic_get_maxnum[str_cfg_data->vid_std - 1](comseq_hdr_info,
			num_picts);
	if (ret != IMG_SUCCESS)
		pr_warn("[USERSID=0x%08X] Failed to get number of reference pictures",
			str_cfg_data->user_str_id);

	/*
	 * For non-conformant stream use the
	 * max(*pui32NumPicts,comseq_hdrinfo->ui32MinPicBufNum)
	 */
	if (*num_picts < comseq_hdr_info->min_pict_buf_num)
		*num_picts = comseq_hdr_info->min_pict_buf_num;

	/*
	 * Increase for MVC: mvcScaleFactor = 2 (H.10.2) and additional pictures
	 * for a StoreInterViewOnlyRef case (C.4.5.2)
	 */
	if (comseq_hdr_info->num_views > 1) {
		*num_picts *= 2;
		*num_picts += comseq_hdr_info->num_views - 1;
	}

	return ret;
}

static void vdecddutils_update_rend_pictsize(struct vdec_pict_size pict_size,
					     struct vdec_pict_size *rend_pict_size)
{
	if (rend_pict_size->width == 0) {
		rend_pict_size->width = pict_size.width;
	} else {
		/* Take the smallest resolution supported by all the planes */
		rend_pict_size->width = (pict_size.width <
			rend_pict_size->width) ?
			pict_size.width :
			rend_pict_size->width;
	}
	if (rend_pict_size->height == 0) {
		rend_pict_size->height = pict_size.height;
	} else {
		/* Take the smallest resolution supported by all the planes. */
		rend_pict_size->height = (pict_size.height <
			rend_pict_size->height) ?
			pict_size.height :
			rend_pict_size->height;
	}
}

int vdecddutils_convert_buffer_config(const struct vdec_str_configdata *str_cfg_data,
				      const struct vdec_pict_bufconfig *pict_bufcfg,
				      struct vdec_pict_rendinfo *pict_rend_info)
{
	const struct pixel_pixinfo      *pix_info;
	struct  img_pixfmt_desc pixfmt;
	unsigned int i;
	unsigned int total_vert_samples = 0;
	unsigned int vert_samples[IMG_MAX_NUM_PLANES];
	unsigned int plane_size = 0;
	unsigned int plane_offset = 0;
	struct vdec_pict_size pict_size;

	/* Validate inputs. */
	VDEC_ASSERT(str_cfg_data);
	VDEC_ASSERT(pict_bufcfg);
	VDEC_ASSERT(pict_rend_info);

	/* Reset picture buffer allocation data. */
	memset(pict_rend_info, 0x0, sizeof(*pict_rend_info));

	pr_debug("%s picture buffer pixel_fmt = %d\n", __func__, pict_bufcfg->pixel_fmt);
	/* Get pixel format info for regular pixel formats... */
	if (pict_bufcfg->pixel_fmt < IMG_PIXFMT_ARBPLANAR8) {
		pix_info = pixel_get_pixinfo(pict_bufcfg->pixel_fmt);
		pixel_yuv_get_desc((struct pixel_pixinfo *)pix_info, &pixfmt);
	} else {
		pixel_get_fmt_desc(pict_bufcfg->pixel_fmt, &pixfmt);
	}

	/*
	 * Construct the render region information from the picture
	 * buffer configuration.
	 */
	for (i = 0; i < IMG_MAX_NUM_PLANES; i++) {
		if (pixfmt.planes[i]) {
			unsigned int plane_align = VDEC_VXD_PICTBUF_ALIGNMENT;

			/*
			 * Determine the offset (in bytes) to this plane.
			 * This is zero for the first (luma) plane and at the
			 * end of the previous plane for all subsequent planes.
			 */
			plane_offset = plane_offset + plane_size;

			/*
			 * Calculate the minimum number of vertical samples
			 * for this plane.
			 */
			vert_samples[i] =
				((pict_bufcfg->coded_height +
				pixfmt.v_denom - 1) / pixfmt.v_denom) *
				pixfmt.v_numer[i];

			/*
			 * Calculate the mimimum plane size from the stride and
			 * decode picture height. Packed buffers have the luma
			 * and chroma exactly adjacent and consequently the
			 * chroma plane offset is equal to this plane size.
			 */
			plane_size = pict_bufcfg->stride[i] * vert_samples[i];
			plane_size = ALIGN(plane_size, plane_align);

			if (!pict_bufcfg->packed && pict_bufcfg->chroma_offset[i]) {
				unsigned int max_plane_size;

				max_plane_size =
					pict_bufcfg->chroma_offset[i] - plane_offset;

				if (plane_size > max_plane_size) {
					pr_err("Chroma offset [%d bytes] is not large enough to fit minimum plane data [%d bytes] at offset [%d]",
					       pict_bufcfg->chroma_offset[i],
					       plane_size, plane_offset);
					return IMG_ERROR_INVALID_PARAMETERS;
				}

				plane_size = max_plane_size;

				vert_samples[i] = plane_size /
					pict_bufcfg->stride[i];
			} else {
				if (pict_bufcfg->chroma_offset[i] && (plane_offset + plane_size) !=
				    pict_bufcfg->chroma_offset[i]) {
					pr_err("Chroma offset specified [%d bytes] should match that required for plane size calculated from stride and height [%d bytes]",
					       pict_bufcfg->chroma_offset[i],
					       plane_offset + plane_size);
					return IMG_ERROR_INVALID_PARAMETERS;
				}
			}

			pict_rend_info->plane_info[i].offset = plane_offset;
			pict_rend_info->plane_info[i].stride =
				pict_bufcfg->stride[i];
			pict_rend_info->plane_info[i].size = plane_size;

#ifdef DEBUG_DECODER_DRIVER
			pr_info("VDECDDUTILS_ConvertBufferConfig() plane %d stride %u size %u offset %u",
				i, pict_rend_info->plane_info[i].stride,
				pict_rend_info->plane_info[i].size,
				pict_rend_info->plane_info[i].offset);
#endif

			pict_rend_info->rendered_size +=
				pict_rend_info->plane_info[i].size;

			total_vert_samples += vert_samples[i];

			/* Calculate the render region maximum picture size. */
			pict_size.width = (pict_rend_info->plane_info[i].stride *
					   pixfmt.bop_denom) / pixfmt.bop_numer[i];
			pict_size.height = (vert_samples[i] * pixfmt.v_denom) / pixfmt.v_numer[i];
			vdecddutils_update_rend_pictsize(pict_size,
							 &pict_rend_info->rend_pict_size);
		}
	}
#ifdef DEBUG_DECODER_DRIVER
	pr_info("VDECDDUTILS_ConvertBufferConfig() total required %u (inc. alignment for addressing/tiling) vs. buffer %u",
		pict_rend_info->rendered_size, pict_bufcfg->buf_size);
#endif

	/* Ensure that the buffer size is large enough to hold the data */
	if (pict_bufcfg->buf_size < pict_rend_info->rendered_size) {
		pr_err("Buffer size [%d bytes] should be at least as large as rendered data (inc. any enforced gap between planes) [%d bytes]",
		       pict_bufcfg->buf_size,
		       pict_rend_info->rendered_size);
		return IMG_ERROR_INVALID_PARAMETERS;
	}

	/* Whole buffer should be marked as rendered region */
	pict_rend_info->rendered_size = pict_bufcfg->buf_size;
	/* Use the actual stride alignment */
	pict_rend_info->stride_alignment = pict_bufcfg->stride_alignment;

	return IMG_SUCCESS;
}

static unsigned char vdecddutils_is_secondary_op_required(const struct vdec_comsequ_hdrinfo
	*comseq_hdr_info,
	const struct vdec_str_opconfig
	*op_cfg)
{
	unsigned char result = TRUE;

	if (!op_cfg->force_oold &&
	    !comseq_hdr_info->post_processing &&
	    comseq_hdr_info->pixel_info.chroma_fmt_idc ==
	    op_cfg->pixel_info.chroma_fmt_idc &&
	    comseq_hdr_info->pixel_info.bitdepth_y ==
	    op_cfg->pixel_info.bitdepth_y &&
	    comseq_hdr_info->pixel_info.bitdepth_c ==
	    op_cfg->pixel_info.bitdepth_c)
		/*
		 * The secondary output is not required (if we have it we will
		 * not use it for transformation (e.g. scaling. rotating or
		 * up/down-sampling).
		 */
		result = FALSE;

	return result;
}

int vdecddutils_get_minrequired_numpicts(const struct vdec_str_configdata *str_cfg_data,
					 const struct vdec_comsequ_hdrinfo *comseq_hdr_info,
					 const struct vdec_str_opconfig *op_cfg,
					 unsigned int *num_picts)
{
	int ret;
	unsigned int max_held_picnum;

	/* If any operation requiring internal buffers is to be applied... */
	if (vdecddutils_is_secondary_op_required(comseq_hdr_info, op_cfg)) {
		/*
		 * Reference picture buffers will be allocated internally,
		 * but there may be a number of picture buffers to which
		 * out-of-display-order pictures will be decoded. These
		 * buffers need to be allocated externally, so there's a
		 * need to calculate the number of out-of-(display)-order
		 * pictures required for the provided video standard.
		 */
		ret = vdecddutils_ref_pict_get_maxnum(str_cfg_data, comseq_hdr_info,
						      &max_held_picnum);
		if (ret != IMG_SUCCESS)
			return ret;
	} else {
		/*
		 * All the reference picture buffers have to be allocated
		 * externally, so there's a need to calculate the number of
		 * reference picture buffers required for the provided video
		 * standard.
		 */
		ret = vdecddutils_ref_pict_get_maxnum(str_cfg_data, comseq_hdr_info,
						      &max_held_picnum);
		if (ret != IMG_SUCCESS)
			return ret;
	}

	/*
	 * Calculate the number of picture buffers required as the maximum
	 * number of picture buffers to be held onto by the driver plus the
	 * current picture buffer.
	 */
	*num_picts = max_held_picnum +
		(comseq_hdr_info->interlaced_frames ? 2 : 1);

	return IMG_SUCCESS;
}

static void vdecddutils_get_codedsize(const struct vdec_pict_rend_config *pict_rend_cfg,
				      struct vdec_pict_size *decoded_pict_size)
{
	decoded_pict_size->width = pict_rend_cfg->coded_pict_size.width;
	decoded_pict_size->height = pict_rend_cfg->coded_pict_size.height;
}

static unsigned char vdecddutils_is_packed(const struct vdec_pict_rendinfo *pict_rend_info,
					   const struct vdec_pict_rend_config *pict_rend_cfg)
{
	unsigned char packed = TRUE;
	unsigned int pict_buf_align;

	/* Validate inputs. */
	VDEC_ASSERT(pict_rend_info);
	VDEC_ASSERT(pict_rend_cfg);

	pict_buf_align = VDEC_VXD_PICTBUF_ALIGNMENT;

	if (pict_rend_info->plane_info[VDEC_PLANE_VIDEO_Y].size !=
		pict_rend_info->plane_info[VDEC_PLANE_VIDEO_UV].offset) {
		/* Planes that are not adjacent cannot be packed */
		packed = FALSE;
	} else if (!is_packedbuf_chroma_aligned(pict_rend_info->plane_info
				[VDEC_PLANE_VIDEO_UV].offset,
				VDEC_PLANE_VIDEO_Y,
				pict_buf_align)) {
		/* Chroma plane must be aligned for packed buffers. */
		VDEC_ASSERT(pict_rend_info->plane_info[VDEC_PLANE_VIDEO_Y].size ==
			pict_rend_info->plane_info[VDEC_PLANE_VIDEO_UV].offset);
		packed = FALSE;
	}

	return packed;
}

static int vdecddutils_get_stride
	(const struct vdec_str_configdata *str_cfg_data,
	const struct vdec_pict_rend_config *pict_rend_cfg,
	unsigned int vert_samples, unsigned int *h_stride,
	enum vdec_color_planes color_planes)
{
	unsigned int hw_h_stride = *h_stride;

	/*
	 * If extended strides are to be used or indexed strides failed,
	 * make extended stride alignment.
	 */
	hw_h_stride = ALIGN(hw_h_stride,
			    pict_rend_cfg->stride_alignment > 0 ?
			    pict_rend_cfg->stride_alignment :
			    VDEC_VXD_EXT_STRIDE_ALIGNMENT_DEFAULT);

	/* A zero-value indicates unsupported stride */
	if (hw_h_stride == 0)
		/* No valid stride found */
		return IMG_ERROR_NOT_SUPPORTED;

	*h_stride = hw_h_stride;

	return IMG_SUCCESS;
}

static int vdecddutils_get_render_info(const struct vdec_str_configdata *str_cfg_data,
				       const struct vdec_pict_rend_config *pict_rend_cfg,
				       const struct pixel_pixinfo *pix_info,
				       struct vdec_pict_rendinfo *pict_rend_info)
{
	unsigned int i;
	struct img_pixfmt_desc pixfmt;
	struct vdec_pict_size coded_pict_size;
	unsigned char single_stride = FALSE;
	unsigned int vert_sample[IMG_MAX_NUM_PLANES] = {0};
	unsigned int total_vert_samples;
	unsigned int largest_stride;
	unsigned int result;

	/* Reset the output structure. */
	memset(pict_rend_info, 0, sizeof(*pict_rend_info));

	/* Ensure that the coded sizes are in whole macroblocks. */
	if ((pict_rend_cfg->coded_pict_size.width  &
		(VDEC_MB_DIMENSION - 1)) != 0 ||
		(pict_rend_cfg->coded_pict_size.height &
		(VDEC_MB_DIMENSION - 1)) != 0) {
		pr_err("Invalid render configuration coded picture size [%d x %d]. It should be a whole number of MBs in each dimension",
		       pict_rend_cfg->coded_pict_size.width,
		       pict_rend_cfg->coded_pict_size.height);
		return IMG_ERROR_INVALID_PARAMETERS;
	}

	/* Check if the stride alignment is multiple of default. */
	if ((pict_rend_cfg->stride_alignment &
		(VDEC_VXD_EXT_STRIDE_ALIGNMENT_DEFAULT - 1)) != 0) {
		pr_err("Invalid stride alignment %d used. It should be multiple of %d.",
		       pict_rend_cfg->stride_alignment,
		       VDEC_VXD_EXT_STRIDE_ALIGNMENT_DEFAULT);
		return IMG_ERROR_INVALID_PARAMETERS;
	}

	/* Get pixel format info for regular pixel formats... */
	if (pix_info->pixfmt < IMG_PIXFMT_ARBPLANAR8)
		pixel_yuv_get_desc((struct pixel_pixinfo *)pix_info, &pixfmt);
	else
		pixel_get_fmt_desc(pix_info->pixfmt, &pixfmt);

	/* Get the coded size for the appropriate orientation */
	vdecddutils_get_codedsize(pict_rend_cfg, &coded_pict_size);

	/*
	 * Calculate the hardware (inc. constraints) strides and
	 * number of vertical samples for each plane.
	 */
	total_vert_samples = 0;
	largest_stride = 0;
	for (i = 0; i < IMG_MAX_NUM_PLANES; i++) {
		if (pixfmt.planes[i]) {
			unsigned int h_stride;

			/* Horizontal stride must be for a multiple of BOPs. */
			h_stride = ((coded_pict_size.width +
				pixfmt.bop_denom - 1) /
				pixfmt.bop_denom) * pixfmt.bop_numer[i];

			/*
			 * Vertical only has to satisfy whole pixel of
			 * samples.
			 */
			vert_sample[i] = ((coded_pict_size.height +
				pixfmt.v_denom - 1) /
				pixfmt.v_denom) * pixfmt.v_numer[i];

			/*
			 * Obtain a horizontal stride supported by the hardware
			 * (inc. constraints).
			 */
			result = vdecddutils_get_stride(str_cfg_data, pict_rend_cfg, vert_sample[i],
							&h_stride, (enum vdec_color_planes)i);
			if (result != IMG_SUCCESS) {
				VDEC_ASSERT(0);
				pr_err("No valid VXD stride found for picture with decoded dimensions [%d x %d] and min stride [%d]",
				       coded_pict_size.width, coded_pict_size.height, h_stride);
				return result;
			}

			pict_rend_info->plane_info[i].stride = h_stride;
			if (i == VDEC_PLANE_VIDEO_UV && (str_cfg_data->vid_std == VDEC_STD_H264 ||
							 str_cfg_data->vid_std == VDEC_STD_HEVC)) {
				struct pixel_pixinfo *info =
					pixel_get_pixinfo(pix_info->pixfmt);
				VDEC_ASSERT(PIXEL_FORMAT_INVALID !=
					info->chroma_fmt_idc);
			}

			total_vert_samples += vert_sample[i];
			if (h_stride > largest_stride)
				largest_stride = h_stride;
		}
	}
	pict_rend_info->stride_alignment =
		pict_rend_cfg->stride_alignment > 0 ?
		pict_rend_cfg->stride_alignment :
		VDEC_VXD_EXT_STRIDE_ALIGNMENT_DEFAULT;

	if (pict_rend_cfg->packed)
		single_stride = TRUE;

#ifdef HAS_JPEG
	/* JPEG hardware uses a single (luma) stride for all planes. */
	if (str_cfg_data->vid_std == VDEC_STD_JPEG) {
		single_stride = true;

		/* Luma should be largest for this to be used for all planes. */
		VDEC_ASSERT(largest_stride ==
			pict_rend_info->plane_info[VDEC_PLANE_VIDEO_Y].stride);
	}
#endif

	/* Calculate plane sizes. */
	for (i = 0; i < IMG_MAX_NUM_PLANES; i++) {
		if (pixfmt.planes[i]) {
			struct vdec_pict_size pict_size;
			unsigned int vert_samples = vert_sample[i];
			unsigned int plane_align = VDEC_VXD_PICTBUF_ALIGNMENT;

			if (single_stride)
				pict_rend_info->plane_info[i].stride =
					largest_stride;

			pict_rend_info->plane_info[i].size =
				pict_rend_info->plane_info[i].stride *
				vert_samples;
			pict_rend_info->plane_info[i].size =
				ALIGN(pict_rend_info->plane_info[i].size, plane_align);
			/*
			 * Ensure that the total buffer rendered size is
			 * rounded-up to the picture buffer alignment so that
			 * this plane (within this single buffer) can be
			 * correctly addressed by the hardware at this byte
			 * offset.
			 */
			if (i == 1 && pict_rend_cfg->packed)
				/*
				 * Packed buffers must have chroma plane
				 * already aligned since this was factored
				 * into the stride/size calculation.
				 */
				VDEC_ASSERT(pict_rend_info->rendered_size ==
					    ALIGN(pict_rend_info->rendered_size, plane_align));

			pict_rend_info->plane_info[i].offset = pict_rend_info->rendered_size;

			/* Update the total buffer size (inc. this plane). */
			pict_rend_info->rendered_size +=
				pict_rend_info->plane_info[i].size;

			/*
			 * Update the maximum render picture size supported
			 * by all planes of this buffer.
			 */
			pict_size.width = (pict_rend_info->plane_info[i].stride *
				pixfmt.bop_denom) / pixfmt.bop_numer[i];

			pict_size.height = (vert_sample[i] * pixfmt.v_denom) / pixfmt.v_numer[i];

			vdecddutils_update_rend_pictsize(pict_size,
							 &pict_rend_info->rend_pict_size);

#ifdef DEBUG_DECODER_DRIVER
			pr_info("vdecddutils_GetRenderInfo() plane %d stride %u size %u offset %u",
				i, pict_rend_info->plane_info[i].stride,
				pict_rend_info->plane_info[i].size,
				pict_rend_info->plane_info[i].offset);
#endif
		}
	}

#ifdef DEBUG_DECODER_DRIVER
	pr_info("vdecddutils_GetRenderInfo() total %u (inc. alignment for addressing/tiling)",
		pict_rend_info->rendered_size);
#endif

	return IMG_SUCCESS;
}

int vdecddutils_pictbuf_getconfig(const struct vdec_str_configdata *str_cfg_data,
				  const struct vdec_pict_rend_config *pict_rend_cfg,
				  const struct vdec_str_opconfig *str_opcfg,
				  struct vdec_pict_bufconfig *pict_bufcfg)
{
	struct vdec_pict_rendinfo disp_pict_rendinfo;
	struct vdec_pict_size coded_pict_size;
	unsigned int ret, i;
	unsigned int size0, size1;

	/* Validate inputs. */
	VDEC_ASSERT(str_cfg_data);
	VDEC_ASSERT(pict_rend_cfg);
	VDEC_ASSERT(str_opcfg);
	VDEC_ASSERT(pict_bufcfg);

	/* Clear the picture buffer config before populating */
	memset(pict_bufcfg, 0, sizeof(struct vdec_pict_bufconfig));

	/* Determine the rounded-up coded sizes (compatible with hardware) */
	ret = vdecddutils_get_render_info(str_cfg_data,
					  pict_rend_cfg,
					  &str_opcfg->pixel_info,
					  &disp_pict_rendinfo);
	if (ret != IMG_SUCCESS)
		return ret;

	/* Get the coded size for the appropriate orientation */
	vdecddutils_get_codedsize(pict_rend_cfg, &coded_pict_size);

	pict_bufcfg->coded_width = coded_pict_size.width;
	pict_bufcfg->coded_height = coded_pict_size.height;

	/*
	 * Use the luma stride for all planes in buffer.
	 * Additional chroma stride may be needed for other pixel formats.
	 */
	for (i = 0; i < VDEC_PLANE_MAX; i++)
		pict_bufcfg->stride[i] = disp_pict_rendinfo.plane_info[i].stride;

	/*
	 * Pixel information is taken from that
	 * specified for display.
	 */
	pict_bufcfg->pixel_fmt = str_opcfg->pixel_info.pixfmt;
	pr_debug("picture buffer pixel_fmt = %d\n", pict_bufcfg->pixel_fmt);

	/* Tiling scheme is taken from render configuration */
	pict_bufcfg->byte_interleave = pict_rend_cfg->byte_interleave;
	pr_debug("picture buffer byte_interleave = %d\n", pict_bufcfg->byte_interleave);
	/* Stride alignment */
	pict_bufcfg->stride_alignment = pict_rend_cfg->stride_alignment > 0 ?
		pict_rend_cfg->stride_alignment : VDEC_VXD_EXT_STRIDE_ALIGNMENT_DEFAULT;

	pr_debug("picture buffer stride_alignment = %d\n", pict_bufcfg->stride_alignment);
	/* Chroma offset taken as calculated for render configuration. */
	pict_bufcfg->chroma_offset[0] = disp_pict_rendinfo.plane_info[VDEC_PLANE_VIDEO_UV].offset;
	pict_bufcfg->chroma_offset[1] = disp_pict_rendinfo.plane_info[VDEC_PLANE_VIDEO_V].offset;

	if (pict_rend_cfg->packed && str_opcfg->pixel_info.num_planes > 1) {
		pict_bufcfg->packed = vdecddutils_is_packed(&disp_pict_rendinfo, pict_rend_cfg);
		if (!pict_bufcfg->packed) {
			/* Report if unable to meet request to pack. */
			pr_err("Request for packed buffer could not be met");
			return IMG_ERROR_NOT_SUPPORTED;
		}

		size0 = ALIGN(pict_bufcfg->chroma_offset[0], VDEC_VXD_PICTBUF_ALIGNMENT);
		size1 = ALIGN(pict_bufcfg->chroma_offset[1], VDEC_VXD_PICTBUF_ALIGNMENT);

		if (pict_bufcfg->chroma_offset[0] != size0 ||
		    pict_bufcfg->chroma_offset[1] != size1)  {
			pr_err("Chroma plane could not be located on a %d byte boundary (investigate stride calculations)",
			       VDEC_VXD_PICTBUF_ALIGNMENT);
			return IMG_ERROR_NOT_SUPPORTED;
		}
	} else {
		pict_bufcfg->packed = FALSE;
	}

	pict_bufcfg->buf_size = disp_pict_rendinfo.rendered_size;

	/* Return success */
	return IMG_SUCCESS;
}

int vdecddutils_get_display_region(const struct vdec_pict_size *coded_size,
				   const struct vdec_rect   *orig_disp_region,
				   struct vdec_rect *disp_region)
{
	int ret = IMG_SUCCESS;

	/* Validate inputs. */
	VDEC_ASSERT(coded_size);
	VDEC_ASSERT(orig_disp_region);
	VDEC_ASSERT(disp_region);
	if (!coded_size || !orig_disp_region || !disp_region)
		return IMG_ERROR_INVALID_PARAMETERS;

	/*
	 * In the simplest case the display region is the same as
	 * that defined in the bitstream.
	 */
	*disp_region = *orig_disp_region;

	if (orig_disp_region->height == 0 || orig_disp_region->width == 0 ||
	    coded_size->height == 0 || coded_size->width == 0) {
		pr_err("Invalid params to calculate display region:");
		pr_err("Display Size: [%d,%d]", orig_disp_region->width, orig_disp_region->height);
		pr_err("Coded Size  : [%d,%d]", coded_size->width, coded_size->height);
		return IMG_ERROR_INVALID_PARAMETERS;
	}

	return ret;
}

int vdecddutils_pictbuf_getinfo(const struct vdec_str_configdata *str_cfg_data,
				const struct vdec_pict_rend_config *pict_rend_cfg,
				const struct vdec_str_opconfig *str_op_cfg,
				struct vdec_pict_rendinfo *pict_rend_info)
{
	unsigned int ret;

	/* Validate inputs. */
	VDEC_ASSERT(str_cfg_data);
	VDEC_ASSERT(pict_rend_cfg);
	VDEC_ASSERT(str_op_cfg);
	VDEC_ASSERT(pict_rend_info);

	ret = vdecddutils_get_render_info(str_cfg_data, pict_rend_cfg,
					  &str_op_cfg->pixel_info,
					  pict_rend_info);
	VDEC_ASSERT(ret == IMG_SUCCESS);
	if (ret != IMG_SUCCESS)
		return ret;

	return IMG_SUCCESS;
}
