/* SPDX-License-Identifier: GPL-2.0 */
/*
 * IMG PVDEC pixel Registers
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

#ifndef _MEM_IO_H
#define _MEM_IO_H

#include <linux/types.h>

#include "reg_io2.h"

#define MEMIO_CHECK_ALIGNMENT(vpmem)        \
	IMG_ASSERT((vpmem))

#define MEMIO_READ_FIELD(vpmem, field) \
	((((*((field ## _TYPE *)(((unsigned long)(vpmem)) + field ## _OFFSET))) & \
	   field ## _MASK) >> field ## _SHIFT))

#define MEMIO_WRITE_FIELD(vpmem, field, value, type) \
	do { \
		type __vpmem = vpmem; \
		MEMIO_CHECK_ALIGNMENT(__vpmem); \
		(*((field ## _TYPE *)(((unsigned long)(__vpmem)) + \
				      field ## _OFFSET))) = \
			(field ## _TYPE)(((*((field ## _TYPE *)(((unsigned long)(__vpmem)) + \
								field ## _OFFSET))) & \
					  ~(field ## _TYPE)field ## _MASK) | \
					 (field ## _TYPE)(((value) << field ## _SHIFT) & \
							  field ## _MASK)); \
	} while (0) \

#endif /* _MEM_IO_H */
