#ifndef __ARCH_ASM_MACH_OMAP2_PRCM_COMMON_H
#define __ARCH_ASM_MACH_OMAP2_PRCM_COMMON_H

/*
 * OMAP2 PRCM base and module definitions
 *
 * Copyright (C) 2007 Texas Instruments, Inc.
 * Copyright (C) 2007 Nokia Corporation
 *
 * Written by Paul Walmsley
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


/* Module offsets from both CM_BASE & PRM_BASE */

/* Offsets that are the same on 24xx and 34xx */
/* Technically OCP_MOD is 34xx only, and PLL_MOD is CCR_MOD on 3430 */
#define OCP_MOD						0x000
#define MPU_MOD						0x100
#define CORE_MOD					0x200
#define GFX_MOD						0x300
#define WKUP_MOD					0x400
#define PLL_MOD						0x500


/* Chip-specific module offsets */
#define OMAP24XX_DSP_MOD				0x800

#define OMAP2430_MDM_MOD				0xc00


#endif

