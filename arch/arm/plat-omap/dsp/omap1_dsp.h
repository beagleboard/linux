/*
 * This file is part of OMAP DSP driver (DSP Gateway version 3.3.1)
 *
 * Copyright (C) 2002-2006 Nokia Corporation. All rights reserved.
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

#ifndef __OMAP_DSP_OMAP1_DSP_H
#define __OMAP_DSP_OMAP1_DSP_H

#ifdef CONFIG_ARCH_OMAP15XX
#define OMAP1510_DARAM_BASE	(OMAP1510_DSP_BASE + 0x0)
#define OMAP1510_DARAM_SIZE	0x10000
#define OMAP1510_SARAM_BASE	(OMAP1510_DSP_BASE + 0x10000)
#define OMAP1510_SARAM_SIZE	0x18000
#endif

#ifdef CONFIG_ARCH_OMAP16XX
#define OMAP16XX_DARAM_BASE	(OMAP16XX_DSP_BASE + 0x0)
#define OMAP16XX_DARAM_SIZE	0x10000
#define OMAP16XX_SARAM_BASE	(OMAP16XX_DSP_BASE + 0x10000)
#define OMAP16XX_SARAM_SIZE	0x18000
#endif

/*
 * Reset Control
 */
#define ARM_RSTCT1_SW_RST		0x0008
#define ARM_RSTCT1_DSP_RST		0x0004
#define ARM_RSTCT1_DSP_EN		0x0002
#define ARM_RSTCT1_ARM_RST		0x0001

/*
 * MPUI
 */
#define MPUI_CTRL_WORDSWAP_MASK		0x00600000
#define MPUI_CTRL_WORDSWAP_ALL		0x00000000
#define MPUI_CTRL_WORDSWAP_NONAPI	0x00200000
#define MPUI_CTRL_WORDSWAP_API		0x00400000
#define MPUI_CTRL_WORDSWAP_NONE		0x00600000
#define MPUI_CTRL_AP_MASK		0x001c0000
#define MPUI_CTRL_AP_MDH		0x00000000
#define MPUI_CTRL_AP_MHD		0x00040000
#define MPUI_CTRL_AP_DMH		0x00080000
#define MPUI_CTRL_AP_HMD		0x000c0000
#define MPUI_CTRL_AP_DHM		0x00100000
#define MPUI_CTRL_AP_HDM		0x00140000
#define MPUI_CTRL_BYTESWAP_MASK		0x00030000
#define MPUI_CTRL_BYTESWAP_NONE		0x00000000
#define MPUI_CTRL_BYTESWAP_NONAPI	0x00010000
#define MPUI_CTRL_BYTESWAP_ALL		0x00020000
#define MPUI_CTRL_BYTESWAP_API		0x00030000
#define MPUI_CTRL_TIMEOUT_MASK		0x0000ff00
#define MPUI_CTRL_APIF_HNSTB_DIV_MASK	0x000000f0
#define MPUI_CTRL_S_NABORT_GL		0x00000008
#define MPUI_CTRL_S_NABORT_32BIT	0x00000004
#define MPUI_CTRL_EN_TIMEOUT		0x00000002
#define MPUI_CTRL_HF_MCUCLK		0x00000001
#define DSP_BOOT_CONFIG_DIRECT		0x00000000
#define DSP_BOOT_CONFIG_PSD_DIRECT	0x00000001
#define DSP_BOOT_CONFIG_IDLE		0x00000002
#define DSP_BOOT_CONFIG_DL16		0x00000003
#define DSP_BOOT_CONFIG_DL32		0x00000004
#define DSP_BOOT_CONFIG_MPUI		0x00000005
#define DSP_BOOT_CONFIG_INTERNAL	0x00000006

/*
 * DSP boot mode
 *   direct:        0xffff00
 *   pseudo direct: 0x080000
 *   MPUI:          branch 0x010000
 *   internel:      branch 0x024000
 */
#define DSP_BOOT_ADR_DIRECT		0xffff00
#define DSP_BOOT_ADR_PSD_DIRECT		0x080000
#define DSP_BOOT_ADR_MPUI		0x010000
#define DSP_BOOT_ADR_INTERNAL		0x024000

/*
 * TC
 */
#define TC_ENDIANISM_SWAP		0x00000002
#define TC_ENDIANISM_SWAP_WORD		0x00000002
#define TC_ENDIANISM_SWAP_BYTE		0x00000000
#define TC_ENDIANISM_EN			0x00000001

/*
 * DSP ICR
 */
#define DSPREG_ICR_RESERVED_BITS	0xffc0
#define DSPREG_ICR_EMIF			0x0020
#define DSPREG_ICR_DPLL			0x0010
#define DSPREG_ICR_PER			0x0008
#define DSPREG_ICR_CACHE		0x0004
#define DSPREG_ICR_DMA			0x0002
#define DSPREG_ICR_CPU			0x0001

#endif /* __OMAP_DSP_OMAP1_DSP_H */
