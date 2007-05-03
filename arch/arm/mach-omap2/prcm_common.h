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


/* 24XX register bits shared between CM & PRM registers */

/* CM_FCLKEN1_CORE, CM_ICLKEN1_CORE, PM_WKEN1_CORE shared bits */
#define OMAP2420_EN_MMC_SHIFT				26
#define OMAP2420_EN_MMC					(1 << 26)
#define OMAP24XX_EN_UART2_SHIFT				22
#define OMAP24XX_EN_UART2				(1 << 22)
#define OMAP24XX_EN_UART1_SHIFT				21
#define OMAP24XX_EN_UART1				(1 << 21)
#define OMAP24XX_EN_MCSPI2_SHIFT			18
#define OMAP24XX_EN_MCSPI2				(1 << 18)
#define OMAP24XX_EN_MCSPI1_SHIFT			17
#define OMAP24XX_EN_MCSPI1				(1 << 17)
#define OMAP24XX_EN_MCBSP2_SHIFT			16
#define OMAP24XX_EN_MCBSP2				(1 << 16)
#define OMAP24XX_EN_MCBSP1_SHIFT			15
#define OMAP24XX_EN_MCBSP1				(1 << 15)
#define OMAP24XX_EN_GPT12_SHIFT				14
#define OMAP24XX_EN_GPT12				(1 << 14)
#define OMAP24XX_EN_GPT11_SHIFT				13
#define OMAP24XX_EN_GPT11				(1 << 13)
#define OMAP24XX_EN_GPT10_SHIFT				12
#define OMAP24XX_EN_GPT10				(1 << 12)
#define OMAP24XX_EN_GPT9_SHIFT				11
#define OMAP24XX_EN_GPT9				(1 << 11)
#define OMAP24XX_EN_GPT8_SHIFT				10
#define OMAP24XX_EN_GPT8				(1 << 10)
#define OMAP24XX_EN_GPT7_SHIFT				9
#define OMAP24XX_EN_GPT7				(1 << 9)
#define OMAP24XX_EN_GPT6_SHIFT				8
#define OMAP24XX_EN_GPT6				(1 << 8)
#define OMAP24XX_EN_GPT5_SHIFT				7
#define OMAP24XX_EN_GPT5				(1 << 7)
#define OMAP24XX_EN_GPT4_SHIFT				6
#define OMAP24XX_EN_GPT4				(1 << 6)
#define OMAP24XX_EN_GPT3_SHIFT				5
#define OMAP24XX_EN_GPT3				(1 << 5)
#define OMAP24XX_EN_GPT2_SHIFT				4
#define OMAP24XX_EN_GPT2				(1 << 4)
#define OMAP2420_EN_VLYNQ_SHIFT				3
#define OMAP2420_EN_VLYNQ				(1 << 3)

/* CM_FCLKEN2_CORE, CM_ICLKEN2_CORE, PM_WKEN2_CORE shared bits */
#define OMAP2430_EN_GPIO5_SHIFT				10
#define OMAP2430_EN_GPIO5				(1 << 10)
#define OMAP2430_EN_MCSPI3_SHIFT			9
#define OMAP2430_EN_MCSPI3				(1 << 9)
#define OMAP2430_EN_MMCHS2_SHIFT			8
#define OMAP2430_EN_MMCHS2				(1 << 8)
#define OMAP2430_EN_MMCHS1_SHIFT			7
#define OMAP2430_EN_MMCHS1				(1 << 7)
#define OMAP24XX_EN_UART3_SHIFT				2
#define OMAP24XX_EN_UART3				(1 << 2)
#define OMAP24XX_EN_USB_SHIFT				0
#define OMAP24XX_EN_USB					(1 << 0)

/* CM_ICLKEN2_CORE, PM_WKEN2_CORE shared bits */
#define OMAP2430_EN_MDM_INTC_SHIFT			11
#define OMAP2430_EN_MDM_INTC				(1 << 11)
#define OMAP2430_EN_USBHS_SHIFT				6
#define OMAP2430_EN_USBHS				(1 << 6)

/* CM_IDLEST1_CORE, PM_WKST1_CORE shared bits */
#define OMAP2420_ST_MMC					(1 << 26)
#define OMAP24XX_ST_UART2				(1 << 22)
#define OMAP24XX_ST_UART1				(1 << 21)
#define OMAP24XX_ST_MCSPI2				(1 << 18)
#define OMAP24XX_ST_MCSPI1				(1 << 17)
#define OMAP24XX_ST_GPT12				(1 << 14)
#define OMAP24XX_ST_GPT11				(1 << 13)
#define OMAP24XX_ST_GPT10				(1 << 12)
#define OMAP24XX_ST_GPT9				(1 << 11)
#define OMAP24XX_ST_GPT8				(1 << 10)
#define OMAP24XX_ST_GPT7				(1 << 9)
#define OMAP24XX_ST_GPT6				(1 << 8)
#define OMAP24XX_ST_GPT5				(1 << 7)
#define OMAP24XX_ST_GPT4				(1 << 6)
#define OMAP24XX_ST_GPT3				(1 << 5)
#define OMAP24XX_ST_GPT2				(1 << 4)
#define OMAP2420_ST_VLYNQ				(1 << 3)

/* CM_IDLEST2_CORE, PM_WKST2_CORE shared bits */
#define OMAP2430_ST_MDM_INTC				(1 << 11)
#define OMAP2430_ST_GPIO5				(1 << 10)
#define OMAP2430_ST_MCSPI3				(1 << 9)
#define OMAP2430_ST_MMCHS2				(1 << 8)
#define OMAP2430_ST_MMCHS1				(1 << 7)
#define OMAP2430_ST_USBHS				(1 << 6)
#define OMAP24XX_ST_UART3				(1 << 2)
#define OMAP24XX_ST_USB					(1 << 0)

/* CM_FCLKEN_WKUP, CM_ICLKEN_WKUP, PM_WKEN_WKUP shared bits */
#define OMAP24XX_EN_GPIOS_SHIFT				2
#define OMAP24XX_EN_GPIOS				(1 << 2)
#define OMAP24XX_EN_GPT1_SHIFT				0
#define OMAP24XX_EN_GPT1				(1 << 0)

/* PM_WKST_WKUP, CM_IDLEST_WKUP shared bits */
#define OMAP24XX_ST_GPIOS				(1 << 2)
#define OMAP24XX_ST_GPT1				(1 << 0)

/* CM_IDLEST_MDM and PM_WKST_MDM shared bits */
#define OMAP2430_ST_MDM					(1 << 0)


#endif

