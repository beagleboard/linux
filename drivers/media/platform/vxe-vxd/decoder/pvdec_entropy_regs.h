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

#ifndef __PVDEC_ENTROPY_REGS_H__
#define __PVDEC_ENTROPY_REGS_H__

/*
 * PVDEC_ENTROPY, CR_ENTROPY_SHIFTREG_CONTROL, SR_SW_RESET
 */
#define PVDEC_ENTROPY_CR_GENC_BUFFER_SIZE_OFFSET                (0x0100)

/*
 * PVDEC_ENTROPY, CR_GENC_BUFFER_SIZE, GENC_BUFFER_SIZE
 */
#define PVDEC_ENTROPY_CR_GENC_BUFFER_BASE_ADDRESS_OFFSET                (0x0110)

/*
 * PVDEC_ENTROPY, CR_ENTROPY_SLICE_PARAMETER_SIZE, SLICE_PARAMETER_SIZE
 */
#define PVDEC_ENTROPY_CR_GENC_FRAGMENT_BASE_ADDRESS_OFFSET              (0x0098)

#endif
