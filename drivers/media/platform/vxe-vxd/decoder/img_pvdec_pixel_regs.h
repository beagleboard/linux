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

#ifndef _IMG_PVDEC_PIXEL_REGS_H
#define _IMG_PVDEC_PIXEL_REGS_H

/* PVDEC_PIXEL, CR_MAX_FRAME_CONFIG, CR_PVDEC_HOR_MSB */
#define PVDEC_PIXEL_CR_MAX_FRAME_CONFIG_CR_PVDEC_HOR_MSB_MASK           (0x001F0000)

#define PVDEC_PIXEL_CR_MAX_FRAME_CONFIG_CR_PVDEC_HOR_MSB_SHIFT          (16)

/* PVDEC_PIXEL, CR_MAX_FRAME_CONFIG, CR_PVDEC_VER_MSB */
#define PVDEC_PIXEL_CR_MAX_FRAME_CONFIG_CR_PVDEC_VER_MSB_MASK           (0x1F000000)
#define PVDEC_PIXEL_CR_MAX_FRAME_CONFIG_CR_PVDEC_VER_MSB_SHIFT          (24)

/* PVDEC_PIXEL, CR_MAX_FRAME_CONFIG, CR_MSVDX_HOR_MSB */
#define PVDEC_PIXEL_CR_MAX_FRAME_CONFIG_CR_MSVDX_HOR_MSB_MASK           (0x0000001F)
#define PVDEC_PIXEL_CR_MAX_FRAME_CONFIG_CR_MSVDX_HOR_MSB_SHIFT          (0)

/* PVDEC_PIXEL, CR_MAX_FRAME_CONFIG, CR_MSVDX_VER_MSB */
#define PVDEC_PIXEL_CR_MAX_FRAME_CONFIG_CR_MSVDX_VER_MSB_MASK           (0x00001F00)
#define PVDEC_PIXEL_CR_MAX_FRAME_CONFIG_CR_MSVDX_VER_MSB_SHIFT          (8)

#endif /* _IMG_PVDEC_PIXEL_REGS_H */
