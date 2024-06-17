/* SPDX-License-Identifier: GPL-2.0 */
/*
 * JPEG secure data unit parsing API.
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
#ifndef __JPEGSECUREPARSER_H__
#define __JPEGSECUREPARSER_H__

#include "bspp_int.h"

/**
 * struct bspp_jpeg_sequ_hdr_info - bspp_jpeg_sequ_hdr_info dummu structure
 * @dummy: dummy structure
 */
struct bspp_jpeg_sequ_hdr_info {
	unsigned int dummy;
};

int bspp_jpeg_setparser_config(enum vdec_bstr_format bstr_format,
			       struct bspp_vid_std_features *pvidstd_features,
			       struct bspp_swsr_ctx *pswsr_ctx,
			       struct bspp_parser_callbacks *pparser_callbacks,
			       struct bspp_inter_pict_data *pinterpict_data);

void bspp_jpeg_determine_unit_type(unsigned char bitstream_unittype,
				   int disable_mvc,
				   enum bspp_unit_type *bspp_unittype);

#endif /*__JPEGSECUREPARSER_H__ */
