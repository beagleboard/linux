/* SPDX-License-Identifier: GPL-2.0 */
/*
 * firmware header
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

#if !defined DEFS_H_
#define DEFS_H_

#include <linux/types.h>

/*
 * MACROS to insert values into fields within a word. The basename of the
 * field must have MASK_BASENAME and SHIFT_BASENAME constants.
 */
#define F_MASK(basename)  (MASK_##basename)
#define F_SHIFT(basename) (SHIFT_##basename)
/*
 * Extract a value from an instruction word.
 */
#define F_EXTRACT(val, basename) (((val) & (F_MASK(basename))) >> (F_SHIFT(basename)))

/*
 * Mask and shift a value to the position of a particular field.
 */
#define F_ENCODE(val, basename)  (((val) << (F_SHIFT(basename))) & (F_MASK(basename)))
#define F_DECODE(val, basename)  (((val) & (F_MASK(basename))) >> (F_SHIFT(basename)))

/*
 * Insert a value into a word.
 */
#define F_INSERT(word, val, basename) (((word) & ~(F_MASK(basename))) | (F_ENCODE((val), basename)))

#endif
