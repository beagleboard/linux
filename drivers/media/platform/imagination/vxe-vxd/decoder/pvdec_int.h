/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Low-level PVDEC interface component.
 *
 * Copyright (c) Imagination Technologies Ltd.
 * Copyright (c) 2021 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Authors:
 *	Amit Makani <amit.makani@ti.com>
 *
 * Re-written for upstreamimg
 *	Sidraya Jayagond <sidraya.bj@pathpartnertech.com>
 */
#ifndef __PVDEC_INT_H__
#define __PVDEC_INT_H__

#include "hw_control.h"
#include "vxd_ext.h"
#include "vxd_props.h"

/* How many VLC IDX addresses fits in single address register */
#define PVDECIO_VLC_IDX_ADDR_PARTS 2

/* How many VLC IDX initial fits in single width register */
#define PVDECIO_VLC_IDX_WIDTH_PARTS 10

/* How many VLC IDX initial opcodes fits in single opcode register */
#define PVDECIO_VLC_IDX_OPCODE_PARTS 16

/*
 * Length (shift) of VLC IDX opcode field. We're taking [0][1] here, as it
 * corresponds to shift of one element
 */
#define PVDECIO_VLC_IDX_ADDR_ID 2

/*
 * Mask for VLC IDX address field. We're taking [0][0] here, as it corresponds
 * to unshifted mask
 */
#define PVDECIO_VLC_IDX_ADDR_MASK	MSVDX_VEC_CR_VEC_VLC_TABLE_ADDR0_VLC_TABLE_ADDR0_MASK

/*
 * Length (shift) of VLC IDX address field. We're taking [0][1] here, as it
 * corresponds to shift of one element
 */
#define PVDECIO_VLC_IDX_ADDR_SHIFT	MSVDX_VEC_CR_VEC_VLC_TABLE_ADDR0_VLC_TABLE_ADDR1_SHIFT
#define PVDECIO_VLC_IDX_WIDTH_ID	1

/*
 * Mask for VLC IDX width field. We're taking [0][0] here, as it corresponds
 * to unshifted mask
 */
#define PVDECIO_VLC_IDX_WIDTH_MASK      \
	MSVDX_VEC_CR_VEC_VLC_TABLE_INITIAL_WIDTH0_VLC_TABLE_INITIAL_WIDTH0_MASK

/*
 * Length (shift) of VLC IDX width field. We're taking [0][1] here, as it
 * corresponds to shift of one element
 */
#define PVDECIO_VLC_IDX_WIDTH_SHIFT     \
	MSVDX_VEC_CR_VEC_VLC_TABLE_INITIAL_WIDTH0_VLC_TABLE_INITIAL_WIDTH1_SHIFT

#define PVDECIO_VLC_IDX_OPCODE_ID	0

/*
 * Length (shift) of VLC IDX opcode field. We're taking [0][1] here, as it
 * corresponds to shift of one element
 */
#define PVDECIO_VLC_IDX_OPCODE_SHIFT    \
	MSVDX_VEC_CR_VEC_VLC_TABLE_INITIAL_OPCODE0_VLC_TABLE_INITIAL_OPCODE1_SHIFT

/* This comes from DEVA PVDEC FW */
#define CTRL_ALLOC_MAX_SEGMENT_SIZE 1024

/*
 * Mask for VLC IDX opcode field. We're taking [0][0] here, as it corresponds
 * to unshifted mask
 */
#define PVDECIO_VLC_IDX_OPCODE_MASK     \
	MSVDX_VEC_CR_VEC_VLC_TABLE_INITIAL_OPCODE0_VLC_TABLE_INITIAL_OPCODE0_MASK

#endif /* __PVDEC_INT_H__ */
