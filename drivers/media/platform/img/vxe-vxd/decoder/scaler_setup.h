/* SPDX-License-Identifier: GPL-2.0 */
/*
 * VXD DEC constants calculation and scalling coefficients
 *
 * Copyright (c) Imagination Technologies Ltd.
 * Copyright (c) 2021 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Authors:
 *	Amit Makani <amit.makani@ti.com>
 */

#ifndef _SCALER_SETUP_H
#define _SCALER_SETUP_H

#define LOWP                            11
#define HIGHP                           14

#define FIXED(a, digits)                ((int)((a) * (1 << (digits))))

struct scaler_params {
	unsigned int vert_pitch;
	unsigned int vert_startpos;
	unsigned int vert_pitch_chroma;
	unsigned int vert_startpos_chroma;
	unsigned int horz_pitch;
	unsigned int horz_startpos;
	unsigned int horz_pitch_chroma;
	unsigned int horz_startpos_chroma;
	unsigned char fixed_point_shift;
};

struct scaler_filter {
	unsigned char bhoriz_bilinear;
	unsigned char bvert_bilinear;
};

struct scaler_pitch {
	int horiz_luma;
	int vert_luma;
	int horiz_chroma;
	int vert_chroma;
};

struct scaler_config {
	enum vdec_vid_std vidstd;
	const struct vxd_coreprops *coreprops;
	struct pixel_pixinfo *in_pixel_info;
	const struct pixel_pixinfo *out_pixel_info;
	unsigned char bfield_coded;
	unsigned char bseparate_chroma_planes;
	unsigned int recon_width;
	unsigned int recon_height;
	unsigned int mb_width;
	unsigned int mb_height;
	unsigned int scale_width;
	unsigned int scale_height;
};

#endif /* _SCALER_SETUP_H */
