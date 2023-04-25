/* SPDX-License-Identifier: GPL-2.0 */
/*
 * IMG MSVDX VDMC Registers
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

#ifndef _IMG_MSVDX_VDMC_REGS_H
#define _IMG_MSVDX_VDMC_REGS_H

/* MSVDX_VDMC, CR_VDMC_MACROBLOCK_NUMBER, CR_VDMC_MACROBLOCK_X_OFFSET */
#define MSVDX_VDMC_CR_VDMC_MACROBLOCK_NUMBER_CR_VDMC_MACROBLOCK_X_OFFSET_MASK           (0x0000FFFF)
#define MSVDX_VDMC_CR_VDMC_MACROBLOCK_NUMBER_CR_VDMC_MACROBLOCK_X_OFFSET_SHIFT          (0)

/* MSVDX_VDMC, CR_VDMC_MACROBLOCK_NUMBER, CR_VDMC_MACROBLOCK_Y_OFFSET */
#define MSVDX_VDMC_CR_VDMC_MACROBLOCK_NUMBER_CR_VDMC_MACROBLOCK_Y_OFFSET_MASK           (0xFFFF0000)
#define MSVDX_VDMC_CR_VDMC_MACROBLOCK_NUMBER_CR_VDMC_MACROBLOCK_Y_OFFSET_SHIFT          (16)

#endif /* _IMG_MSVDX_VDMC_REGS_H */
