// SPDX-License-Identifier: GPL-2.0
/*
 * Pixel processing function implementations
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

#include <linux/dma-mapping.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-mem2mem.h>

#include "img_errors.h"
#include "img_pixfmts.h"
#include "pixel_api.h"
#include "vdec_defs.h"

#define NUM_OF_FORMATS 17
#define PIXNAME(x) /* Pixel name support not enabled */
#define FACT_SPEC_FORMAT_NUM_PLANES 4
#define FACT_SPEC_FORMAT_PLANE_UNUSED 0xf
#define FACT_SPEC_FORMAT_PLANE_CODE_BITS 4
#define FACT_SPEC_FORMAT_PLANE_CODE_MASK 3
#define FACT_SPEC_FORMAT_MIN_FACT_VAL 1

/*
 * @brief Pointer to the default format in the asPixelFormats array
 * default format is an invalid format
 * @note pointer set by initSearch()
 * This pointer is also used to know if the arrays were sorted
 */
static struct pixel_pixinfo *def_fmt;

/*
 * @brief Actual array storing the pixel formats information.
 */
static struct pixel_pixinfo pix_fmts[NUM_OF_FORMATS] = {
	{
		IMG_PIXFMT_420PL12YUV8,
		PIXEL_UV_ORDER,
		PIXEL_MULTICHROME,
		PIXEL_BIT8_MP,
		PIXEL_FORMAT_420,
		8,
		8,
		2
	},

	{
		IMG_PIXFMT_420PL12YVU8,
		PIXEL_VU_ORDER,
		PIXEL_MULTICHROME,
		PIXEL_BIT8_MP,
		PIXEL_FORMAT_420,
		8,
		8,
		2
	},

	{
		IMG_PIXFMT_420PL12YUV10,
		PIXEL_UV_ORDER,
		PIXEL_MULTICHROME,
		PIXEL_BIT10_MP,
		PIXEL_FORMAT_420,
		10,
		10,
		2
	},

	{
		IMG_PIXFMT_420PL12YVU10,
		PIXEL_VU_ORDER,
		PIXEL_MULTICHROME,
		PIXEL_BIT10_MP,
		PIXEL_FORMAT_420,
		10,
		10,
		2
	},

	{
		IMG_PIXFMT_420PL12YUV10_MSB,
		PIXEL_UV_ORDER,
		PIXEL_MULTICHROME,
		PIXEL_BIT10_MSB_MP,
		PIXEL_FORMAT_420,
		10,
		10,
		2
	},

	{
		IMG_PIXFMT_420PL12YVU10_MSB,
		PIXEL_VU_ORDER,
		PIXEL_MULTICHROME,
		PIXEL_BIT10_MSB_MP,
		PIXEL_FORMAT_420,
		10,
		10,
		2
	},

	{
		IMG_PIXFMT_420PL12YUV10_LSB,
		PIXEL_UV_ORDER,
		PIXEL_MULTICHROME,
		PIXEL_BIT10_LSB_MP,
		PIXEL_FORMAT_420,
		10,
		10,
		2
	},

	{
		IMG_PIXFMT_420PL12YVU10_LSB,
		PIXEL_VU_ORDER,
		PIXEL_MULTICHROME,
		PIXEL_BIT10_LSB_MP,
		PIXEL_FORMAT_420,
		10,
		10,
		2
	},

	{
		IMG_PIXFMT_422PL12YUV8,
		PIXEL_UV_ORDER,
		PIXEL_MULTICHROME,
		PIXEL_BIT8_MP,
		PIXEL_FORMAT_422,
		8,
		8,
		2
	},

	{
		IMG_PIXFMT_422PL12YVU8,
		PIXEL_VU_ORDER,
		PIXEL_MULTICHROME,
		PIXEL_BIT8_MP,
		PIXEL_FORMAT_422,
		8,
		8,
		2
	},

	{
		IMG_PIXFMT_422PL12YUV10,
		PIXEL_UV_ORDER,
		PIXEL_MULTICHROME,
		PIXEL_BIT10_MP,
		PIXEL_FORMAT_422,
		10,
		10,
		2
	},

	{
		IMG_PIXFMT_422PL12YVU10,
		PIXEL_VU_ORDER,
		PIXEL_MULTICHROME,
		PIXEL_BIT10_MP,
		PIXEL_FORMAT_422,
		10,
		10,
		2
	},

	{
		IMG_PIXFMT_422PL12YUV10_MSB,
		PIXEL_UV_ORDER,
		PIXEL_MULTICHROME,
		PIXEL_BIT10_MSB_MP,
		PIXEL_FORMAT_422,
		10,
		10,
		2
	},

	{
		IMG_PIXFMT_422PL12YVU10_MSB,
		PIXEL_VU_ORDER,
		PIXEL_MULTICHROME,
		PIXEL_BIT10_MSB_MP,
		PIXEL_FORMAT_422,
		10,
		10,
		2
	},

	{
		IMG_PIXFMT_422PL12YUV10_LSB,
		PIXEL_UV_ORDER,
		PIXEL_MULTICHROME,
		PIXEL_BIT10_LSB_MP,
		PIXEL_FORMAT_422,
		10,
		10,
		2
	},

	{
		IMG_PIXFMT_422PL12YVU10_LSB,
		PIXEL_VU_ORDER,
		PIXEL_MULTICHROME,
		PIXEL_BIT10_LSB_MP,
		PIXEL_FORMAT_422,
		10,
		10,
		2
	},

	{
		IMG_PIXFMT_UNDEFINED,
		PIXEL_INVALID_CI,
		0,
		(enum pixel_mem_packing)0,
		PIXEL_FORMAT_INVALID,
		0,
		0,
		0
	}
};

static struct pixel_pixinfo_table pixinfo_table[] = {
	{
		IMG_PIXFMT_420PL12YUV8_A8,
		{
			PIXNAME(IMG_PIXFMT_420PL12YUV8_A8)
			16,
			16,
			16,
			0,
			16,
			TRUE,
			TRUE,
			4,
			TRUE
		}
	},

	{
		IMG_PIXFMT_422PL12YUV8_A8,
		{
			PIXNAME(IMG_PIXFMT_422PL12YUV8_A8)
			16,
			16,
			16,
			0,
			16,
			TRUE,
			FALSE,
			4,
			TRUE
		}
	},

	{
		IMG_PIXFMT_420PL12YUV8,
		{
			PIXNAME(IMG_PIXFMT_420PL12YUV8)
			16,
			16,
			16,
			0,
			0,
			TRUE,
			TRUE,
			4,
			FALSE
		}
	},

	{
		IMG_PIXFMT_420PL12YVU8,
		{
			PIXNAME(IMG_PIXFMT_420PL12YVU8)
			16,
			16,
			16,
			0,
			0,
			TRUE,
			TRUE,
			4,
			FALSE
		}
	},

	{
		IMG_PIXFMT_420PL12YUV10,
		{
			PIXNAME(IMG_PIXFMT_420PL12YUV10)
			12,
			16,
			16,
			0,
			0,
			TRUE,
			TRUE,
			4,
			FALSE
		}
	},

	{
		IMG_PIXFMT_420PL12YVU10,
		{
			PIXNAME(IMG_PIXFMT_420PL12YVU10)
			12,
			16,
			16,
			0,
			0,
			TRUE,
			TRUE,
			4,
			FALSE
		}
	},

	{
		IMG_PIXFMT_420PL12YUV10_MSB,
		{
			PIXNAME(IMG_PIXFMT_420PL12YUV10_MSB)
			8,
			16,
			16,
			0,
			0,
			TRUE,
			TRUE,
			4,
			FALSE
		}
	},

	{
		IMG_PIXFMT_420PL12YVU10_MSB,
		{
			PIXNAME(IMG_PIXFMT_420PL12YVU10_MSB)
			8,
			16,
			16,
			0,
			0,
			TRUE,
			TRUE,
			4,
			FALSE
		}
	},

	{
		IMG_PIXFMT_422PL12YUV8,
		{
			PIXNAME(IMG_PIXFMT_422PL12YUV8)
			16,
			16,
			16,
			0,
			0,
			TRUE,
			FALSE,
			4,
			FALSE
		}
	},

	{
		IMG_PIXFMT_422PL12YVU8,
		{
			PIXNAME(IMG_PIXFMT_422PL12YVU8)
			16,
			16,
			16,
			0,
			0,
			TRUE,
			FALSE,
			4,
			FALSE
		}
	},

	{
		IMG_PIXFMT_422PL12YUV10,
		{
			PIXNAME(IMG_PIXFMT_422PL12YUV10)
			12,
			16,
			16,
			0,
			0,
			TRUE,
			FALSE,
			4,
			FALSE
		}
	},

	{
		IMG_PIXFMT_422PL12YVU10,
		{
			PIXNAME(IMG_PIXFMT_422PL12YVU10)
			12,
			16,
			16,
			0,
			0,
			TRUE,
			FALSE,
			4,
			FALSE
		}
	},

	{
		IMG_PIXFMT_422PL12YUV10_MSB,
		{
			PIXNAME(IMG_PIXFMT_422PL12YUV10_MSB)
			8,
			16,
			16,
			0,
			0,
			TRUE,
			FALSE,
			4,
			FALSE
		}
	},

	{
		IMG_PIXFMT_422PL12YVU10_MSB,
		{
			PIXNAME(IMG_PIXFMT_422PL12YVU10_MSB)
			8,
			16,
			16,
			0,
			0,
			TRUE,
			FALSE,
			4,
			FALSE
		}
	},
};

static struct pixel_pixinfo_table*
pixel_get_pixelinfo_from_pixfmt(enum img_pixfmt pix_fmt)
{
	unsigned int i;
	unsigned char found = FALSE;
	struct pixel_pixinfo_table *this_pixinfo_table_entry = NULL;

	for (i = 0;
		i < (sizeof(pixinfo_table) / sizeof(struct pixel_pixinfo_table));
		i++) {
		if (pix_fmt ==  pixinfo_table[i].pix_color_fmt) {
			/*
			 * There must only be one entry per pixel colour format
			 * in the table
			 */
			VDEC_ASSERT(!found);
			found = TRUE;
			this_pixinfo_table_entry = &pixinfo_table[i];

			/*
			 * We deliberately do NOT break here - scan rest of
			 * table to ensure there are not duplicate entries
			 */
		}
	}
	return this_pixinfo_table_entry;
}

/*
 * @brief Array containing string lookup of pixel format IDC.
 * @warning this must be kept in step with PIXEL_FormatIdc.
 */
unsigned char pix_fmt_idc_names[6][16] = {
	"Monochrome",
	"4:1:1",
	"4:2:0",
	"4:2:2",
	"4:4:4",
	"Invalid",
};

static int pixel_compare_pixfmts(const void *a, const void *b)
{
	return ((struct pixel_pixinfo *)a)->pixfmt -
	       ((struct pixel_pixinfo *)b)->pixfmt;
}

static struct pixel_info*
pixel_get_bufinfo_from_pixfmt(enum img_pixfmt pix_fmt)
{
	struct pixel_pixinfo_table *pixinfo_table_entry = NULL;
	struct pixel_info *pix_info = NULL;

	pixinfo_table_entry = pixel_get_pixelinfo_from_pixfmt(pix_fmt);
	VDEC_ASSERT(pixinfo_table_entry);
	if (pixinfo_table_entry)
		pix_info = &pixinfo_table_entry->info;

	return pix_info;
}

/*
 * @brief Search a pixel format based on its attributes rather than its format
 * enum.
 * @warning use PIXEL_Comparpix_fmts to search by enum
 */
static int pixel_compare_pixinfo(const void *a, const void *b)
{
	int result = 0;
	const struct pixel_pixinfo *fmt_a = (struct pixel_pixinfo *)a;
	const struct pixel_pixinfo *fmt_b = (struct pixel_pixinfo *)b;

	result = fmt_a->chroma_fmt_idc - fmt_b->chroma_fmt_idc;
	if (result != 0)
		return result;

	result = fmt_a->mem_pkg - fmt_b->mem_pkg;
	if (result != 0)
		return result;

	result = fmt_a->chroma_interleave - fmt_b->chroma_interleave;
	if (result != 0)
		return result;

	result = fmt_a->bitdepth_y - fmt_b->bitdepth_y;
	if (result != 0)
		return result;

	result = fmt_a->bitdepth_c - fmt_b->bitdepth_c;
	if (result != 0)
		return result;

	result = fmt_a->num_planes - fmt_b->num_planes;
	if (result != 0)
		return result;

	return result;
}

static void pixel_init_search(void)
{
	static unsigned int search_inited;

	search_inited++;
	if (search_inited == 1) {
		if (!def_fmt) {
			int i = 0;

			i = NUM_OF_FORMATS - 1;
			while (i >= 0) {
				if (IMG_PIXFMT_UNDEFINED ==
					pix_fmts[i].pixfmt) {
					def_fmt = &pix_fmts[i];
					break;
				}
			}
			VDEC_ASSERT(def_fmt);
		}
	} else {
		search_inited--;
	}
}

static struct pixel_pixinfo *pixel_search_fmt(const struct pixel_pixinfo *key,
					      unsigned char enum_only)
{
	struct pixel_pixinfo *fmt_found = NULL;
	int (*compar)(const void *pixfmt1, const void *pixfmt2);

	if (enum_only)
		compar = &pixel_compare_pixfmts;
	else
		compar = &pixel_compare_pixinfo;

	{
		unsigned int i;

		for (i = 0; i < NUM_OF_FORMATS; i++) {
			if (compar(key, &pix_fmts[i]) == 0) {
				fmt_found = &pix_fmts[i];
				break;
			}
		}
	}
	return fmt_found;
}

/*
 * @brief Set a pixel format info structure to the default.
 * @warning This MODIDIFES the pointer therefore you shouldn't
 * call it on pointer you got from the library!
 */
static void pixel_pixinfo_defaults(struct pixel_pixinfo *to_def)
{
	if (!def_fmt)
		pixel_init_search();

	memcpy(to_def, def_fmt, sizeof(struct pixel_pixinfo));
}

enum img_pixfmt pixel_get_pixfmt(enum pixel_fmt_idc chroma_fmt_idc,
				 enum pixel_chroma_interleaved
				 chroma_interleaved,
				 enum pixel_mem_packing mem_pkg,
				 unsigned int bitdepth_y, unsigned int bitdepth_c,
				 unsigned int num_planes)
{
	unsigned int internal_num_planes = (num_planes == 0 || num_planes > 4) ? 2 :
		num_planes;
	struct pixel_pixinfo key;
	struct pixel_pixinfo *fmt_found = NULL;

	if (chroma_fmt_idc != PIXEL_FORMAT_MONO &&
	    chroma_fmt_idc != PIXEL_FORMAT_411 &&
	    chroma_fmt_idc != PIXEL_FORMAT_420 &&
	    chroma_fmt_idc != PIXEL_FORMAT_422 &&
	    chroma_fmt_idc != PIXEL_FORMAT_444)
		return IMG_PIXFMT_UNDEFINED;

	/* valid bit depth 8, 9, 10, or 16/0 for 422 */
	if (bitdepth_y < 8 || bitdepth_y > 10)
		return IMG_PIXFMT_UNDEFINED;

	/* valid bit depth 8, 9, 10, or 16/0 for 422 */
	if (bitdepth_c < 8 || bitdepth_c > 10)
		return IMG_PIXFMT_UNDEFINED;

	key.pixfmt = IMG_PIXFMT_UNDEFINED;
	key.chroma_fmt_idc = chroma_fmt_idc;
	key.chroma_interleave = chroma_interleaved;
	key.mem_pkg = mem_pkg;
	key.bitdepth_y = bitdepth_y;
	key.bitdepth_c = bitdepth_c;
	key.num_planes = internal_num_planes;

	/*
	 * 9 and 10 bits formats are handled in the same way, and there is only
	 * one entry in the PixelFormat table
	 */
	if (key.bitdepth_y == 9)
		key.bitdepth_y = 10;

	/*
	 * 9 and 10 bits formats are handled in the same way, and there is only
	 * one entry in the PixelFormat table
	 */
	if (key.bitdepth_c == 9)
		key.bitdepth_c = 10;

	pixel_init_search();

	/* do not search by format */
	fmt_found = pixel_search_fmt(&key, FALSE);
	if (!fmt_found)
		return IMG_PIXFMT_UNDEFINED;

	return fmt_found->pixfmt;
}

static void pixel_get_internal_pixelinfo(struct pixel_pixinfo *pixinfo,
					 struct pixel_info *pix_bufinfo)
{
	if (pixinfo->bitdepth_y == 8 && pixinfo->bitdepth_c == 8)
		pix_bufinfo->pixels_in_bop = 16;
	else if (pixinfo->mem_pkg == PIXEL_BIT10_MP)
		pix_bufinfo->pixels_in_bop = 12;
	else
		pix_bufinfo->pixels_in_bop = 8;

	if (pixinfo->bitdepth_y == 8)
		pix_bufinfo->ybytes_in_bop = pix_bufinfo->pixels_in_bop;
	else
		pix_bufinfo->ybytes_in_bop = 16;

	if (pixinfo->chroma_fmt_idc == PIXEL_FORMAT_MONO) {
		pix_bufinfo->uvbytes_in_bop = 0;
	} else if (pixinfo->bitdepth_c == 8) {
		pix_bufinfo->uvbytes_in_bop = pix_bufinfo->pixels_in_bop;
		if (pixinfo->chroma_fmt_idc == PIXEL_FORMAT_422 && pixinfo->num_planes == 1) {
			pix_bufinfo->uvbytes_in_bop = 0;
			pix_bufinfo->pixels_in_bop = 8;
		}
	} else {
		pix_bufinfo->uvbytes_in_bop = 16;
	}

	if (pixinfo->chroma_fmt_idc == PIXEL_FORMAT_444)
		pix_bufinfo->uvbytes_in_bop *= 2;

	if (pixinfo->chroma_interleave == PIXEL_INVALID_CI) {
		pix_bufinfo->uvbytes_in_bop /= 2;
		pix_bufinfo->vbytes_in_bop = pix_bufinfo->uvbytes_in_bop;
	} else {
		pix_bufinfo->vbytes_in_bop = 0;
	}

	pix_bufinfo->alphabytes_in_bop = 0;

	if (pixinfo->num_planes == 1)
		pix_bufinfo->is_planar = FALSE;
	else
		pix_bufinfo->is_planar = TRUE;

	if (pixinfo->chroma_fmt_idc == PIXEL_FORMAT_420)
		pix_bufinfo->uv_height_halved = TRUE;
	else
		pix_bufinfo->uv_height_halved = FALSE;

	if (pixinfo->chroma_fmt_idc == PIXEL_FORMAT_444)
		pix_bufinfo->uv_stride_ratio_times4 = 8;
	else
		pix_bufinfo->uv_stride_ratio_times4 = 4;

	if (pixinfo->chroma_interleave == PIXEL_INVALID_CI)
		pix_bufinfo->uv_stride_ratio_times4 /= 2;

	pix_bufinfo->has_alpha = FALSE;
}

static void pixel_yuv_get_descriptor_int(struct pixel_info *pixinfo,
					 struct img_pixfmt_desc  *pix_desc)
{
	pix_desc->bop_denom = pixinfo->pixels_in_bop;
	pix_desc->h_denom = (pixinfo->uv_stride_ratio_times4 == 2 ||
		!pixinfo->is_planar) ? 2 : 1;
	pix_desc->v_denom = (pixinfo->uv_height_halved || !pixinfo->is_planar)
		? 2 : 1;

	pix_desc->planes[0] = TRUE;
	pix_desc->bop_numer[0] = pixinfo->ybytes_in_bop;
	pix_desc->h_numer[0] = pix_desc->h_denom;
	pix_desc->v_numer[0] = pix_desc->v_denom;

	pix_desc->planes[1] = pixinfo->is_planar;
	pix_desc->bop_numer[1] = pixinfo->uvbytes_in_bop;
	pix_desc->h_numer[1] = (pix_desc->h_denom * pixinfo->uv_stride_ratio_times4) / 4;
	pix_desc->v_numer[1] = 1;

	pix_desc->planes[2] = (pixinfo->vbytes_in_bop > 0) ? TRUE : FALSE;
	pix_desc->bop_numer[2] = pixinfo->vbytes_in_bop;
	pix_desc->h_numer[2] = (pixinfo->vbytes_in_bop > 0) ? 1 : 0;
	pix_desc->v_numer[2] = (pixinfo->vbytes_in_bop > 0) ? 1 : 0;

	pix_desc->planes[3] = pixinfo->has_alpha;
	pix_desc->bop_numer[3] = pixinfo->alphabytes_in_bop;
	pix_desc->h_numer[3] = pix_desc->h_denom;
	pix_desc->v_numer[3] = pix_desc->v_denom;
}

int pixel_yuv_get_desc(struct pixel_pixinfo *pix_info, struct img_pixfmt_desc *pix_desc)
{
	struct pixel_info int_pix_info;

	struct pixel_info *int_pix_info_old = NULL;
	enum img_pixfmt pix_fmt = pixel_get_pixfmt(pix_info->chroma_fmt_idc,
			pix_info->chroma_interleave,
			pix_info->mem_pkg,
			pix_info->bitdepth_y,
			pix_info->bitdepth_c,
			pix_info->num_planes);

	/* Validate the output from new function. */
	if (pix_fmt != IMG_PIXFMT_UNDEFINED)
		int_pix_info_old = pixel_get_bufinfo_from_pixfmt(pix_fmt);

	pixel_get_internal_pixelinfo(pix_info, &int_pix_info);

	if (int_pix_info_old) {
		VDEC_ASSERT(int_pix_info_old->has_alpha ==
			int_pix_info.has_alpha);
		VDEC_ASSERT(int_pix_info_old->is_planar ==
			int_pix_info.is_planar);
		VDEC_ASSERT(int_pix_info_old->uv_height_halved ==
			int_pix_info.uv_height_halved);
		VDEC_ASSERT(int_pix_info_old->alphabytes_in_bop ==
			int_pix_info.alphabytes_in_bop);
		VDEC_ASSERT(int_pix_info_old->pixels_in_bop ==
			int_pix_info.pixels_in_bop);
		VDEC_ASSERT(int_pix_info_old->uvbytes_in_bop ==
			int_pix_info.uvbytes_in_bop);
		VDEC_ASSERT(int_pix_info_old->uv_stride_ratio_times4 ==
			int_pix_info.uv_stride_ratio_times4);
		VDEC_ASSERT(int_pix_info_old->vbytes_in_bop ==
			int_pix_info.vbytes_in_bop);
		VDEC_ASSERT(int_pix_info_old->ybytes_in_bop ==
			int_pix_info.ybytes_in_bop);
	}

	pixel_yuv_get_descriptor_int(&int_pix_info, pix_desc);

	return IMG_SUCCESS;
}

struct pixel_pixinfo *pixel_get_pixinfo(const enum img_pixfmt pix_fmt)
{
	struct pixel_pixinfo key;
	struct pixel_pixinfo *fmt_found = NULL;

	pixel_init_search();
	pixel_pixinfo_defaults(&key);
	key.pixfmt = pix_fmt;

	fmt_found = pixel_search_fmt(&key, TRUE);
	if (!fmt_found)
		return def_fmt;
	return fmt_found;
}

int pixel_get_fmt_desc(enum img_pixfmt pix_fmt, struct img_pixfmt_desc *pix_desc)
{
	if (pix_fmt >= IMG_PIXFMT_ARBPLANAR8 && pix_fmt <= IMG_PIXFMT_ARBPLANAR8_LAST) {
		unsigned int i;
		unsigned short spec;

		pix_desc->bop_denom = 1;
		pix_desc->h_denom = 1;
		pix_desc->v_denom = 1;

		spec = (pix_fmt - IMG_PIXFMT_ARBPLANAR8) & 0xffff;
		for (i = 0; i < FACT_SPEC_FORMAT_NUM_PLANES; i++) {
			unsigned char code = (spec >> FACT_SPEC_FORMAT_PLANE_CODE_BITS *
				(FACT_SPEC_FORMAT_NUM_PLANES - 1 - i)) & 0xf;
			pix_desc->bop_numer[i] = 1;
			pix_desc->h_numer[i] = ((code >> 2) & FACT_SPEC_FORMAT_PLANE_CODE_MASK) +
						FACT_SPEC_FORMAT_MIN_FACT_VAL;
			pix_desc->v_numer[i] = (code & FACT_SPEC_FORMAT_PLANE_CODE_MASK) +
						FACT_SPEC_FORMAT_MIN_FACT_VAL;
			if (i == 0 || code != FACT_SPEC_FORMAT_PLANE_UNUSED) {
				pix_desc->planes[i] = TRUE;

				pix_desc->h_denom =
					pix_desc->h_denom > pix_desc->h_numer[i] ?
					pix_desc->h_denom : pix_desc->h_numer[i];

				pix_desc->v_denom =
					pix_desc->v_denom > pix_desc->v_numer[i] ?
					pix_desc->v_denom : pix_desc->v_numer[i];
			} else {
				pix_desc->planes[i] = FALSE;
			}
		}
	} else {
		struct pixel_info *info =
			pixel_get_bufinfo_from_pixfmt(pix_fmt);
		if (!info) {
			VDEC_ASSERT(0);
			return -EINVAL;
		}

		pixel_yuv_get_descriptor_int(info, pix_desc);
	}

	return IMG_SUCCESS;
}

int pixel_gen_pixfmt(enum img_pixfmt *pix_fmt, struct img_pixfmt_desc *pix_desc)
{
	unsigned short spec = 0, i;
	unsigned char code;

	for (i = 0; i < FACT_SPEC_FORMAT_NUM_PLANES; i++) {
		if (pix_desc->planes[i] != 1) {
			code = FACT_SPEC_FORMAT_PLANE_UNUSED;
		} else {
			code = (((pix_desc->h_numer[i] - FACT_SPEC_FORMAT_MIN_FACT_VAL) &
				FACT_SPEC_FORMAT_PLANE_CODE_MASK) << 2) |
				((pix_desc->v_numer[i] - FACT_SPEC_FORMAT_MIN_FACT_VAL) &
				FACT_SPEC_FORMAT_PLANE_CODE_MASK);
		}
		spec |= (code << FACT_SPEC_FORMAT_PLANE_CODE_BITS *
			(FACT_SPEC_FORMAT_NUM_PLANES - 1 - i));
	}

	*pix_fmt = (enum img_pixfmt)(IMG_PIXFMT_ARBPLANAR8 | spec);

	return 0;
}
