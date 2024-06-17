/* SPDX-License-Identifier: GPL-2.0 */
/*
 * VXD DEC Common low level core interface component
 *
 * Copyright (c) Imagination Technologies Ltd.
 * Copyright (c) 2021 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Authors:
 *	Angela Stegmaier <angelabaker@ti.com>
 *
 * Re-written for upstreming
 *	Sidraya Jayagond <sidraya.bj@pathpartnertech.com>
 */

#ifndef __PVDEC_VEC_BE_REGS_H__
#define __PVDEC_VEC_BE_REGS_H__

#define PVDEC_VEC_BE_CR_GENC_BUFFER_SIZE_OFFSET         (0x0040)

/*
 * PVDEC_VEC_BE, CR_GENC_BUFFER_SIZE, GENC_BUFFER_SIZE
 */
#define PVDEC_VEC_BE_CR_GENC_BUFFER_BASE_ADDRESS_OFFSET         (0x0050)

/*
 * PVDEC_VEC_BE, CR_MEM_TO_REG_CONTROL, MEM_TO_REG_NUM_PAIRS
 */
#define PVDEC_VEC_BE_CR_GENC_FRAGMENT_BASE_ADDRESS_OFFSET               (0x0030)

/*
 * PVDEC_VEC_BE, CR_GENC_CONTEXT1, GENC_CONTEXT1_1
 */
#define PVDEC_VEC_BE_CR_ABOVE_PARAM_BASE_ADDRESS_OFFSET         (0x00C0)

#endif
