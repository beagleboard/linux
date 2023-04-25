/* SPDX-License-Identifier: GPL-2.0 */
/*
 * VXD Decoder resource allocation and destroy Interface header
 *
 * Copyright (c) Imagination Technologies Ltd.
 * Copyright (c) 2021 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Authors:
 *	Sunita Nadampalli <sunitan@ti.com>
 *
 * Re-written for upstream
 *      Sidraya Jayagond <sidraya.bj@pathpartnertech.com>
 */

#ifndef _DEC_RESOURCES_H_
#define _DEC_RESOURCES_H_

#include "decoder.h"
#include "lst.h"

/*
 * This structure contains the core resources.
 * @brief  Decoder Core Resources
 */
struct dec_res_ctx {
	struct vidio_ddbufinfo intra_bufinfo;
	struct vidio_ddbufinfo auxline_bufinfo;
	struct vidio_ddbufinfo start_code_bufinfo;
	struct vidio_ddbufinfo vlc_tables_bufinfo[VDEC_STD_MAX];
	struct vidio_ddbufinfo vlc_idxtables_bufinfo[VDEC_STD_MAX];
	void *res_pool[DECODER_RESTYPE_MAX];
	struct lst_t pool_data_list[DECODER_RESTYPE_MAX];
};

int dec_res_picture_detach(void **res_ctx, struct dec_decpict *dec_pict);

int dec_res_picture_attach(void **res_ctx, enum vdec_vid_std vid_std,
			   struct dec_decpict *dec_pict);

int dec_res_create(void *mmudev_handle,
		   struct vxd_coreprops *core_props, unsigned int num_dec_slots,
		   unsigned int mem_heap_id, void **resources);

int dec_res_destroy(void *mmudev_handle, void *res_ctx);

#endif
