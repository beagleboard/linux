/*
 * This file is part of OMAP DSP driver (DSP Gateway version 3.3.1)
 *
 * Copyright (C) 2006 Nokia Corporation. All rights reserved.
 *
 * Contact: Toshihiro Kobayashi <toshihiro.kobayashi@nokia.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#ifndef __OMAP_DSP_OMAP2_DSP_H
#define __OMAP_DSP_OMAP2_DSP_H

#ifdef CONFIG_ARCH_OMAP24XX
#define OMAP24XX_DARAM_BASE	(DSP_MEM_24XX_VIRT + 0x0)
#define OMAP24XX_DARAM_SIZE	0x10000
#define OMAP24XX_SARAM_BASE	(DSP_MEM_24XX_VIRT + 0x10000)
#define OMAP24XX_SARAM_SIZE	0x18000
#endif

#include <mach/hardware.h>

/*
 * DSP IPI registers: mapped to 0xe1000000 -- use readX(), writeX()
 */
#ifdef CONFIG_ARCH_OMAP24XX
#define DSP_IPI_BASE			DSP_IPI_24XX_VIRT
#endif

#ifdef CONFIG_ARCH_OMAP34XX
#define DSP_IPI_BASE			DSP_IPI_34XX_VIRT
#endif

#define DSP_IPI_REVISION		(DSP_IPI_BASE + 0x00)
#define DSP_IPI_SYSCONFIG		(DSP_IPI_BASE + 0x10)
#define DSP_IPI_INDEX			(DSP_IPI_BASE + 0x40)
#define DSP_IPI_ENTRY			(DSP_IPI_BASE + 0x44)
#define DSP_IPI_ENABLE			(DSP_IPI_BASE + 0x48)
#define DSP_IPI_IOMAP			(DSP_IPI_BASE + 0x4c)
#define DSP_IPI_DSPBOOTCONFIG		(DSP_IPI_BASE + 0x50)

#define DSP_IPI_ENTRY_ELMSIZEVALUE_MASK	0x00000003
#define DSP_IPI_ENTRY_ELMSIZEVALUE_8	0x00000000
#define DSP_IPI_ENTRY_ELMSIZEVALUE_16	0x00000001
#define DSP_IPI_ENTRY_ELMSIZEVALUE_32	0x00000002

#define DSP_BOOT_CONFIG_DIRECT		0x00000000
#define DSP_BOOT_CONFIG_PSD_DIRECT	0x00000001
#define DSP_BOOT_CONFIG_IDLE		0x00000002
#define DSP_BOOT_CONFIG_DL16		0x00000003
#define DSP_BOOT_CONFIG_DL32		0x00000004
#define DSP_BOOT_CONFIG_API		0x00000005
#define DSP_BOOT_CONFIG_INTERNAL	0x00000006

/*
 * DSP boot mode
 *   direct:        0xffff00
 *   pseudo direct: 0x080000
 *   API:           branch 0x010000
 *   internel:      branch 0x024000
 */
#define DSP_BOOT_ADR_DIRECT		0xffff00
#define DSP_BOOT_ADR_PSD_DIRECT		0x080000
#define DSP_BOOT_ADR_API		0x010000
#define DSP_BOOT_ADR_INTERNAL		0x024000

/*
 * DSP ICR
 */
#define DSPREG_ICR_RESERVED_BITS	0xfc00
#define DSPREG_ICR_HWA			0x0200
#define DSPREG_ICR_IPORT		0x0100
#define DSPREG_ICR_MPORT		0x0080
#define DSPREG_ICR_XPORT		0x0040
#define DSPREG_ICR_DPORT		0x0020
#define DSPREG_ICR_DPLL			0x0010
#define DSPREG_ICR_PER			0x0008
#define DSPREG_ICR_CACHE		0x0004
#define DSPREG_ICR_DMA			0x0002
#define DSPREG_ICR_CPU			0x0001

#endif /* __OMAP_DSP_OMAP2_DSP_H */
