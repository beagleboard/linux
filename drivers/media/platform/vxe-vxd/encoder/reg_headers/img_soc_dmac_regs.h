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

#ifndef _REGCONV_H_img_soc_dmac_regs_h
#define _REGCONV_H_img_soc_dmac_regs_h

/* Register DMAC_COUNT */
#define IMG_SOC_DMAC_COUNT(X)       (0x0004 + (32 * (X)))
#define MASK_IMG_SOC_BSWAP          0x40000000
#define SHIFT_IMG_SOC_BSWAP         30
#define SHIFT_IMG_SOC_PW            27
#define MASK_IMG_SOC_PW             0x18000000
#define MASK_IMG_SOC_DIR            0x04000000
#define SHIFT_IMG_SOC_DIR           26

/* Register DMAC_COUNT */
#define MASK_IMG_SOC_EN             0x00010000
#define MASK_IMG_SOC_LIST_EN        0x00040000

/* Register DMAC_COUNT */
#define MASK_IMG_SOC_PI             0x03000000
#define SHIFT_IMG_SOC_PI            24
#define MASK_IMG_SOC_CNT            0x0000FFFF
#define SHIFT_IMG_SOC_CNT           0
#define MASK_IMG_SOC_TRANSFER_IEN   0x20000000

/* Register DMAC_IRQ_STAT */
#define IMG_SOC_DMAC_IRQ_STAT(X)    (0x000C + (32 * (X)))
#define MASK_IMG_SOC_TRANSFER_FIN   0x00020000
#define SHIFT_IMG_SOC_TRANSFER_FIN  17

/* Register DMAC_PER_HOLD */
#define IMG_SOC_DMAC_PER_HOLD(X)    (0x0018 + (32 * (X)))

/* Register DMAC_SETUP */
#define IMG_SOC_DMAC_SETUP(X)       (0x0000 + (32 * (X)))

/* Register DMAC_PERIPH */
#define IMG_SOC_DMAC_PERIPH(X)      (0x0008 + (32 * (X)))
#define MASK_IMG_SOC_ACC_DEL        0xE0000000
#define SHIFT_IMG_SOC_ACC_DEL       29
#define MASK_IMG_SOC_INCR           0x08000000
#define SHIFT_IMG_SOC_INCR          27
#define MASK_IMG_SOC_BURST          0x07000000
#define SHIFT_IMG_SOC_BURST         24

/* Register DMAC_PERIPHERAL_ADDR */
#define IMG_SOC_DMAC_PERIPHERAL_ADDR(X) (0x0014 + (32 * (X)))

#endif

