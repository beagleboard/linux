/* SPDX-License-Identifier: GPL-2.0 */
/*
 * VXD Decoder CORE and V4L2 Node Interface header
 *
 * Copyright (c) Imagination Technologies Ltd.
 * Copyright (c) 2021 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Authors:
 *	Sunita Nadampalli <sunitan@ti.com>
 *
 * Re-written for upstream
 *      Sidraya Jayagond <sidraya.bj@pathpartnertech.com>
 *      Prashanth Kumar Amai <prashanth.ka@pathpartnertech.com>
 */

#ifndef __CORE_H__
#define __CORE_H__

#include <linux/types.h>
#include "decoder.h"

int core_initialise(void *dev_handle, unsigned int internal_heap_id,
		    void *cb);

/**
 * core_deinitialise - deinitialise core
 */
int core_deinitialise(void);

int core_supported_features(struct vdec_features *features);

int core_stream_create(void *vxd_dec_ctx_arg,
		       const struct vdec_str_configdata *str_cfgdata,
		       unsigned int *res_str_id);

int core_stream_destroy(unsigned int res_str_id);

int core_stream_play(unsigned int res_str_id);

int core_stream_stop(unsigned int res_str_id);

int core_stream_map_buf(unsigned int res_str_id, enum vdec_buf_type buf_type,
			struct vdec_buf_info *buf_info, unsigned int *buf_map_id);

int core_stream_map_buf_sg(unsigned int res_str_id,
			   enum vdec_buf_type buf_type,
			   struct vdec_buf_info *buf_info,
			   void *sgt, unsigned int *buf_map_id);

int core_stream_unmap_buf(unsigned int buf_map_id);

int core_stream_unmap_buf_sg(unsigned int buf_map_id);

int core_stream_submit_unit(unsigned int res_str_id,
			    struct vdecdd_str_unit *str_unit);

int core_stream_fill_pictbuf(unsigned int buf_map_id);

/* This function to be called before stream play */
int core_stream_set_output_config(unsigned int res_str_id,
				  struct vdec_str_opconfig *str_opcfg,
				  struct vdec_pict_bufconfig *pict_bufcg);

int core_stream_flush(unsigned int res_str_id, unsigned char discard_refs);

int core_stream_release_bufs(unsigned int res_str_id,
			     enum vdec_buf_type buf_type);

int core_stream_get_status(unsigned int res_str_id,
			   struct vdecdd_decstr_status *str_status);

#endif
