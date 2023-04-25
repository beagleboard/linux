/* SPDX-License-Identifier: GPL-2.0 */
/*
 * VXD DEC Low-level device interface component
 *
 * Copyright (c) Imagination Technologies Ltd.
 * Copyright (c) 2021 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Authors:
 *	Amit Makani <amit.makani@ti.com>
 */

#ifndef _VXD_EXT_H
#define _VXD_EXT_H

#define VLR_COMPLETION_COMMS_AREA_SIZE             476

/* Word Size of buffer used to pass messages between LISR and HISR */
#define VXD_SIZE_MSG_BUFFER   (1 * 1024)

/* This structure describes macroblock coordinates. */
struct vxd_mb_coords {
	unsigned int x;
	unsigned int y;
};

/* This structure contains firmware and decoding pipe state information. */
struct vxd_pipestate {
	unsigned char is_pipe_present;
	unsigned char cur_codec;
	unsigned int acheck_point[VDECFW_CHECKPOINT_MAX];
	unsigned int firmware_action;
	unsigned int fe_slices;
	unsigned int be_slices;
	unsigned int fe_errored_slices;
	unsigned int be_errored_slices;
	unsigned int be_mbs_dropped;
	unsigned int be_mbs_recovered;
	struct vxd_mb_coords fe_mb;
	struct vxd_mb_coords be_mb;
};

/* This structure contains firmware and decoder core state information. */
struct vxd_firmware_state {
	unsigned int fw_step;
	struct vxd_pipestate pipe_state[VDECFW_MAX_DP];
};

/* This structure contains the video decoder device state. */
struct vxd_states {
	struct vxd_firmware_state fw_state;
};

struct vxd_pict_attrs {
	unsigned int dwrfired;
	unsigned int mmufault;
	unsigned int deverror;
};

/* This type defines the message attributes. */
enum vxd_msg_attr {
	VXD_MSG_ATTR_NONE        = 0,
	VXD_MSG_ATTR_DECODED     = 1,
	VXD_MSG_ATTR_FATAL       = 2,
	VXD_MSG_ATTR_CANCELED    = 3,
	VXD_MSG_ATTR_FORCE32BITS = 0x7FFFFFFFU
};

enum vxd_msg_flag {
	VXD_MSG_FLAG_DROP        = 0,
	VXD_MSG_FLAG_EXCL        = 1,
	VXD_MSG_FLAG_FORCE32BITS = 0x7FFFFFFFU
};

#endif /* VXD_EXT_H */
