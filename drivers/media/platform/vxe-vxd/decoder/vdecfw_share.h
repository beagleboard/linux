/* SPDX-License-Identifier: GPL-2.0 */
/*
 * VXD DEC SYSDEV and UI Interface header
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
#ifndef _VDECFW_SHARE_H_
#define _VDECFW_SHARE_H_

/*
 * This macro sets alignment for a field structure.
 * Parameters :
 * a - alignment value
 * t - field type
 * n - field name
 */
#define IMG_ALIGN_FIELD(a, t, n) t n  __aligned(a)

/* END of vdecfw_share_macros.h */

/*
 * Field alignments in shared data structures
 */
/* Default field alignment */
#define VDECFW_SHARE_DEFAULT_ALIGNMENT  4
/* Pointer field alignment */
#define VDECFW_SHARE_PTR_ALIGNMENT      4

#endif /* _VDECFW_SHARE_H_ */
