/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Pixel processing functions header
 *
 * Copyright (c) Imagination Technologies Ltd.
 * Copyright (c) 2021 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Authors:
 *	Sunita Nadampalli <sunitan@ti.com>
 *
 * Re-written for upstream
 *	Sidraya Jayagond <sidraya.bj@pathpartnertech.com>
 */

#ifndef __PIXEL_API_H__
#define __PIXEL_API_H__

#include <linux/types.h>

#include "img_errors.h"
#include "img_pixfmts.h"

#define PIXEL_MULTICHROME   TRUE
#define PIXEL_MONOCHROME    FALSE
#define IMG_MAX_NUM_PLANES  4
#define PIXEL_INVALID_BDC   8

extern unsigned char pix_fmt_idc_names[6][16];

struct img_pixfmt_desc {
	unsigned char planes[IMG_MAX_NUM_PLANES];
	unsigned int bop_denom;
	unsigned int bop_numer[IMG_MAX_NUM_PLANES];
	unsigned int h_denom;
	unsigned int v_denom;
	unsigned int h_numer[IMG_MAX_NUM_PLANES];
	unsigned int v_numer[IMG_MAX_NUM_PLANES];
};

/*
 * @brief This type defines memory chroma interleaved order
 */
enum pixel_chroma_interleaved {
	PIXEL_INVALID_CI        = 0,
	PIXEL_UV_ORDER          = 1,
	PIXEL_VU_ORDER          = 2,
	PIXEL_YAYB_ORDER        = 4,
	PIXEL_AYBY_ORDER        = 8,
	PIXEL_ORDER_FORCE32BITS = 0x7FFFFFFFU
};

/*
 * @brief This macro translates enum pixel_chroma_interleaved values into
 * value that can be used to write HW registers directly.
 */
#define PIXEL_GET_HW_CHROMA_INTERLEAVED(value) \
	((value) & PIXEL_VU_ORDER ? TRUE : FALSE)

/*
 * @brief This type defines memory packing types
 */
enum pixel_mem_packing {
	PIXEL_BIT8_MP             = 0,
	PIXEL_BIT10_MSB_MP        = 1,
	PIXEL_BIT10_LSB_MP        = 2,
	PIXEL_BIT10_MP            = 3,
	PIXEL_DEFAULT_MP          = 0xff,
	PIXEL_DEFAULT_FORCE32BITS = 0x7FFFFFFFU
};

static inline unsigned char pixel_get_hw_memory_packing(enum pixel_mem_packing value)
{
	return value == PIXEL_BIT8_MP ? FALSE :
	       value == PIXEL_BIT10_MSB_MP ? FALSE :
	       value == PIXEL_BIT10_LSB_MP ? FALSE :
	       value == PIXEL_BIT10_MP ? TRUE : FALSE;
}

/*
 * @brief This type defines chroma formats
 */
enum pixel_fmt_idc {
	PIXEL_FORMAT_MONO        = 0,
	PIXEL_FORMAT_411         = 1,
	PIXEL_FORMAT_420         = 2,
	PIXEL_FORMAT_422         = 3,
	PIXEL_FORMAT_444         = 4,
	PIXEL_FORMAT_INVALID     = 0xFF,
	PIXEL_FORMAT_FORCE32BITS = 0x7FFFFFFFU
};

static inline int pixel_get_hw_chroma_format_idc(enum pixel_fmt_idc value)
{
	return value == PIXEL_FORMAT_MONO ? 0 :
	       value == PIXEL_FORMAT_420 ? 1 :
	       value == PIXEL_FORMAT_422 ? 2 :
	       value == PIXEL_FORMAT_444 ? 3 :
	       PIXEL_FORMAT_INVALID;
}

/*
 * @brief This structure contains information about the pixel formats
 */
struct pixel_pixinfo {
	enum img_pixfmt pixfmt;
	enum pixel_chroma_interleaved chroma_interleave;
	unsigned char chroma_fmt;
	enum pixel_mem_packing mem_pkg;
	enum pixel_fmt_idc chroma_fmt_idc;
	unsigned int bitdepth_y;
	unsigned int bitdepth_c;
	unsigned int num_planes;
};

/*
 * @brief This type defines the image in memory
 */
struct pixel_info {
	unsigned int pixels_in_bop;
	unsigned int ybytes_in_bop;
	unsigned int uvbytes_in_bop;
	unsigned int vbytes_in_bop;
	unsigned int alphabytes_in_bop;
	unsigned char is_planar;
	unsigned char uv_height_halved;
	unsigned int uv_stride_ratio_times4;
	unsigned char has_alpha;
};

struct pixel_pixinfo_table {
	enum img_pixfmt pix_color_fmt;
	struct pixel_info info;
};

struct pixel_pixinfo *pixel_get_pixinfo(const enum img_pixfmt pixfmt);

enum img_pixfmt pixel_get_pixfmt(enum pixel_fmt_idc chroma_fmt_idc,
				 enum pixel_chroma_interleaved
				 chroma_interleaved,
				 enum pixel_mem_packing mem_packing,
				 unsigned int bitdepth_y, unsigned int bitdepth_c,
				 unsigned int num_planes);

int pixel_yuv_get_desc(struct pixel_pixinfo *pix_info,
		       struct img_pixfmt_desc *desc);

int pixel_get_fmt_desc(enum img_pixfmt pixfmt,
		       struct img_pixfmt_desc *fmt_desc);

int pixel_gen_pixfmt(enum img_pixfmt *pix_fmt, struct img_pixfmt_desc *pix_desc);

#endif
