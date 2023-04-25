/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Low-level VXD interface component
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

#ifndef _VXD_PROPS_H
#define _VXD_PROPS_H

#include "vdec_defs.h"
#include "imgmmu.h"

#define VDEC_MAX_PIXEL_PIPES 2

#define VXD_MAX_CORES    1
#define VER_STR_LEN      64

#define CORE_REVISION(maj, min, maint) \
	((((maj) & 0xff) << 16) | (((min) & 0xff) << 8) | (((maint) & 0xff)))
#define MAJOR_REVISION(rev)     (((rev) >> 16) & 0xff)
#define MINOR_REVISION(rev)     (((rev) >> 8) & 0xff)
#define MAINT_REVISION(rev)     ((rev) & 0xff)

#define FROM_REV(maj, min, maint, type) \
	({ \
		type __maj = maj; \
		type __min = min; \
		(((maj_rev) > (__maj)) || \
		 (((maj_rev) == (__maj)) && ((min_rev) > (__min))) || \
		 (((maj_rev) == (__maj)) && ((min_rev) == (__min)) && \
		  ((int)(maint_rev) >= (maint)))); })

struct vxd_vidstd_props {
	enum vdec_vid_std vidstd;
	unsigned int core_rev;
	unsigned int min_width;
	unsigned int min_height;
	unsigned int max_width;
	unsigned int max_height;
	unsigned int max_macroblocks;
	unsigned int max_luma_bitdepth;
	unsigned int max_chroma_bitdepth;
	enum pixel_fmt_idc max_chroma_format;
};

struct vxd_coreprops {
	unsigned char aversion[VER_STR_LEN];
	unsigned char mpeg2[VDEC_MAX_PIXEL_PIPES];
	unsigned char mpeg4[VDEC_MAX_PIXEL_PIPES];
	unsigned char h264[VDEC_MAX_PIXEL_PIPES];
	unsigned char vc1[VDEC_MAX_PIXEL_PIPES];
	unsigned char avs[VDEC_MAX_PIXEL_PIPES];
	unsigned char real[VDEC_MAX_PIXEL_PIPES];
	unsigned char jpeg[VDEC_MAX_PIXEL_PIPES];
	unsigned char vp6[VDEC_MAX_PIXEL_PIPES];
	unsigned char vp8[VDEC_MAX_PIXEL_PIPES];
	unsigned char hevc[VDEC_MAX_PIXEL_PIPES];
	unsigned char rotation_support[VDEC_MAX_PIXEL_PIPES];
	unsigned char scaling_support[VDEC_MAX_PIXEL_PIPES];
	unsigned char hd_support;
	unsigned int num_streams[VDEC_MAX_PIXEL_PIPES];
	unsigned int num_entropy_pipes;
	unsigned int num_pixel_pipes;
	struct vxd_vidstd_props vidstd_props[VDEC_STD_MAX];
	enum mmu_etype mmu_type;
	unsigned char mmu_support_stride_per_context;
	unsigned char mmu_support_secure;
	/* Range extensions supported by hw -> used only by hevc */
	unsigned char hevc_range_ext[VDEC_MAX_PIXEL_PIPES];
};

#endif /* _VXD_PROPS_H */
