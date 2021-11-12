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

#ifndef _REGCONV_H_mtx_regs_h
#define _REGCONV_H_mtx_regs_h

/* Register CR_MTX_ENABLE */
#define MTX_CR_MTX_ENABLE           0x0000
#define MASK_MTX_MTX_ENABLE         0x00000001
#define MASK_MTX_MTX_TOFF           0x00000002

/* Register CR_MTX_KICK */
#define MTX_CR_MTX_KICK             0x0080

/* Register CR_MTX_REGISTER_READ_WRITE_DATA */
#define MTX_CR_MTX_REGISTER_READ_WRITE_DATA 0x00F8

/* Register CR_MTX_REGISTER_READ_WRITE_REQUEST */
#define MTX_CR_MTX_REGISTER_READ_WRITE_REQUEST 0x00FC
#define MASK_MTX_MTX_RNW            0x00010000
#define MASK_MTX_MTX_DREADY         0x80000000

/* Register CR_MTX_RAM_ACCESS_DATA_TRANSFER */
#define MTX_CR_MTX_RAM_ACCESS_DATA_TRANSFER 0x0104

/* Register CR_MTX_RAM_ACCESS_CONTROL */
#define MTX_CR_MTX_RAM_ACCESS_CONTROL 0x0108
#define MASK_MTX_MTX_MCMR           0x00000001
#define MASK_MTX_MTX_MCMAI          0x00000002
#define SHIFT_MTX_MTX_MCMAI         1
#define MASK_MTX_MTX_MCM_ADDR       0x000FFFFC
#define SHIFT_MTX_MTX_MCM_ADDR      2
#define MASK_MTX_MTX_MCMID          0x0FF00000
#define SHIFT_MTX_MTX_MCMID         20

/* Register CR_MTX_RAM_ACCESS_STATUS */
#define MTX_CR_MTX_RAM_ACCESS_STATUS 0x010C
#define MASK_MTX_MTX_MTX_MCM_STAT   0x00000001

/* Register CR_MTX_SOFT_RESET */
#define MTX_CR_MTX_SOFT_RESET       0x0200
#define MASK_MTX_MTX_RESET          0x00000001

/* Register CR_MTX_SYSC_CDMAC */
#define MTX_CR_MTX_SYSC_CDMAC       0x0340
#define MASK_MTX_LENGTH             0x0000FFFF
#define SHIFT_MTX_LENGTH            0
#define MASK_MTX_ENABLE             0x00010000
#define SHIFT_MTX_ENABLE            16
#define MASK_MTX_RNW                0x00020000
#define SHIFT_MTX_RNW               17
#define MASK_MTX_BURSTSIZE          0x07000000
#define SHIFT_MTX_BURSTSIZE         24

/* Register CR_MTX_SYSC_CDMAA */
#define MTX_CR_MTX_SYSC_CDMAA       0x0344

/* Register CR_MTX_SYSC_CDMAT */
#define MTX_CR_MTX_SYSC_CDMAT       0x0350

#endif
