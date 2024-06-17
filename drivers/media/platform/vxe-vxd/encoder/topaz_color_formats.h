/* SPDX-License-Identifier: GPL-2.0 */
/*
 * buffer sizes calculation
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

#include "topaz_api.h"
#include "fw_headers/defs.h"

void plane_size(enum img_format color_format, unsigned int stride,
		unsigned int height, unsigned int *y_size, unsigned int *u_size,
		unsigned int *v_size)
{
	*y_size = *u_size = *v_size = 0;

	switch (color_format) {
	case IMG_CODEC_420_PL8:
		/* allocate frame for 4:2:0 planar format */
		*y_size = stride * height;
		*u_size = stride * height / 4;
		*v_size = stride * height / 4;
		break;
	case IMG_CODEC_420_PL12:
		/* allocate frame for 4:2:0 planar format (chroma interleaved) */
		*y_size = stride * height;
		*u_size = stride * height / 2;
		break;
	case IMG_CODEC_422_YUV:
	case IMG_CODEC_422_YV12:
	case IMG_CODEC_422_IMC2:
		/* allocate frame for 4:2:2 format */
		*y_size = stride * height * 2;
		break;
	case IMG_CODEC_422_PL8:
		/* allocate frame for 4:2:2 planar format */
		*y_size = stride * height;
		*u_size = stride * height / 2;
		*v_size = stride * height / 2;
		break;
	case IMG_CODEC_422_PL12:
		/* allocate frame for 4:2:2 planar format (chroma interleaved) */
		*y_size = stride * height;
		*u_size = stride * height;
		break;
	case IMG_CODEC_Y0UY1V_8888:
	case IMG_CODEC_UY0VY1_8888:
	case IMG_CODEC_Y0VY1U_8888:
	case IMG_CODEC_VY0UY1_8888:
		/* allocate frame for 4:2:2 format */
		*y_size = stride * height;
		break;
	case IMG_CODEC_444_YUV:
	case IMG_CODEC_444_YV12:
	case IMG_CODEC_444_IMC2:
		/* allocate frame for 4:2:2 format */
		*y_size = stride * height * 3;
		break;
	case IMG_CODEC_444_PL8:
		/* allocate frame for 4:2:2 planar format */
		*y_size = stride * height;
		*u_size = stride * height;
		*v_size = stride * height;
		break;
	case IMG_CODEC_444_PL12:
		/* allocate frame for 4:2:2 planar format (chroma interleaved) */
		*y_size = stride * height;
		*u_size = stride * height * 2;
		break;
	case IMG_CODEC_ABCX:
	case IMG_CODEC_XBCA:
	case IMG_CODEC_ABC565:
		/* allocate frame for RGB interleaved format */
		*y_size = stride * height;
		break;
	case IMG_CODEC_420_YUV:
	case IMG_CODEC_420_YV12:
	case IMG_CODEC_420_IMC2:
	case IMG_CODEC_420_PL12_PACKED:
	case IMG_CODEC_420_PL21_PACKED:
		/* allocate frame for 4:2:0 format */
		*y_size = stride * height * 3 / 2;
		break;
	default:
		*y_size = 0;
		*u_size = 0;
		*v_size = 0;
		break;
	}
}
