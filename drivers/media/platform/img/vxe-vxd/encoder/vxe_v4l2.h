/* SPDX-License-Identifier: GPL-2.0 */
/*
 * V4L2 interface header
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

#ifndef _VXE_V4L2_H
#define _VXE_V4L2_H

#include <linux/dma-mapping.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-mem2mem.h>

/*
 * struct vxe_ctrl - contains info for each supported v4l2 control
 */
struct vxe_ctrl {
	unsigned int cid;
	enum v4l2_ctrl_type type;
	unsigned char name[32];
	int minimum;
	int maximum;
	int step;
	int default_value;
	unsigned char compound;
};

extern struct mem_space topaz_mem_space[];

#endif
