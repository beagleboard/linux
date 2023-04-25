/* SPDX-License-Identifier: GPL-2.0 */
/*
 * VXD DEC Hardware control implementation
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

#ifndef _HW_CONTROL_H
#define _HW_CONTROL_H

#include "bspp.h"
#include "decoder.h"
#include "fw_interface.h"
#include "img_dec_common.h"
#include "img_errors.h"
#include "lst.h"
#include "mem_io.h"
#include "vdecdd_defs.h"
#include "vdecfw_shared.h"
#include "vid_buf.h"
#include "vxd_ext.h"
#include "vxd_props.h"

/* Size of additional buffers needed for each HEVC picture */
#ifdef HAS_HEVC

/* Empirically defined */
#define MEM_TO_REG_BUF_SIZE 0x2000

/*
 * Max. no. of slices found in stream db: approx. 2200,
 * set MAX_SLICES to 2368 to get buffer size page aligned
 */
#define MAX_SLICES 2368
#define SLICE_PARAMS_SIZE 64
#define SLICE_PARAMS_BUF_SIZE (MAX_SLICES * SLICE_PARAMS_SIZE)

/*
 * Size of buffer for "above params" structure, sufficient for stream of width 8192
 * 192 * (8192/64) == 0x6000, see "above_param_size" in TRM
 */
#define ABOVE_PARAMS_BUF_SIZE 0x6000
#endif

enum hwctrl_msgid {
	HWCTRL_MSGID_BATCH     = 0,
	HWCTRL_MSGID_FRAGMENT  = 1,
	CORE_MSGID_MAX,
	CORE_MSGID_FORCE32BITS = 0x7FFFFFFFU
};

struct hwctrl_to_kernel_msg {
	unsigned int msg_size;
	unsigned int km_str_id;
	unsigned int flags;
	unsigned char *msg_hdr;
};

struct hwctrl_batch_msgdata {
	struct vidio_ddbufinfo *batchmsg_bufinfo;
	struct vidio_ddbufinfo *pvdec_fwctx;
	unsigned int ctrl_alloc_bytes;
	unsigned int operating_mode;
	unsigned int transaction_id;
	unsigned int tile_cfg;
	unsigned int genc_id;
	unsigned int mb_load;
	unsigned int size_delimited_mode;
};

struct hwctrl_fragment_msgdata {
	struct vidio_ddbufinfo *batchmsg_bufinfo;
	unsigned int ctrl_alloc_offset;
	unsigned int ctrl_alloc_bytes;
};

struct hwctrl_msgdata {
	unsigned int km_str_id;
	struct hwctrl_batch_msgdata batch_msgdata;
	struct hwctrl_fragment_msgdata fragment_msgdata;
};

/*
 * This structure contains MSVDX Message information.
 */
struct hwctrl_msgstatus {
	unsigned char control_fence_id[VDECFW_MSGID_CONTROL_TYPES];
	unsigned char decode_fence_id[VDECFW_MSGID_DECODE_TYPES];
	unsigned char completion_fence_id[VDECFW_MSGID_COMPLETION_TYPES];
};

/*
 * this structure contains the HWCTRL Core state.
 */
struct hwctrl_state {
	struct vxd_states core_state;
	struct hwctrl_msgstatus fwmsg_status;
	struct hwctrl_msgstatus hostmsg_status;
};

int hwctrl_picture_submit_fragment(void *hndl_hwctx,
				   struct dec_pict_fragment *pict_fragment,
				   struct dec_decpict *decpict,
				   void *vxd_dec_ctx);

int hwctrl_process_msg(void *hndl_hwct, unsigned int msg_flags, unsigned int *msg,
		       struct dec_decpict **decpict);

int hwctrl_getcore_cached_status(void *hndl_hwctx, struct hwctrl_state *state);

int hwctrl_get_core_status(void *hndl_hwctx, struct hwctrl_state *state);

int hwctrl_is_on_seq_replay(void *hndl_hwctx);

int hwctrl_picture_submitbatch(void *hndl_hwctx, struct dec_decpict *decpict,
			       void *vxd_dec_ctx);

int hwctrl_getpicpend_pictlist(void *hndl_hwctx, unsigned int transaction_id,
			       struct dec_decpict **decpict);

int hwctrl_peekheadpiclist(void *hndl_hwctx, struct dec_decpict **decpict);

int hwctrl_getdecodedpicture(void *hndl_hwctx, struct dec_decpict **decpict);

void hwctrl_removefrom_piclist(void *hndl_hwctx, struct dec_decpict *decpict);

int hwctrl_getregsoffset(void *hndl_hwctx,
			 struct decoder_regsoffsets *regs_offsets);

int hwctrl_initialise(void *dec_core, void *comp_int_userdata,
		      const struct vdecdd_dd_devconfig *dd_devconfig,
		      struct vxd_coreprops *core_props, void **hndl_hwctx);

int hwctrl_deinitialise(void *hndl_hwctx);

#endif /* _HW_CONTROL_H */
