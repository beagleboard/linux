/*
 * TI AM437x Image Sensor Interface Registers
 *
 * Copyright (C) 2013 - 2014 Texas Instruments, Inc.
 *
 * Benoit Parrot <bparrot@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _AM437X_ISIF_REGS_H
#define _AM437X_ISIF_REGS_H

/**************************************************************************\
* Register OFFSET Definitions
\**************************************************************************/
#define ISIF_PID				0x0
#define ISIF_PCR				0x4
#define ISIF_SYN_MODE				0x8
#define ISIF_HD_VD_WID				0xc
#define ISIF_PIX_LINES				0x10
#define ISIF_HORZ_INFO				0x14
#define ISIF_VERT_START				0x18
#define ISIF_VERT_LINES				0x1c
#define ISIF_CULLING				0x20
#define ISIF_HSIZE_OFF				0x24
#define ISIF_SDOFST				0x28
#define ISIF_SDR_ADDR				0x2c
#define ISIF_CLAMP				0x30
#define ISIF_DCSUB				0x34
#define ISIF_COLPTN				0x38
#define ISIF_BLKCMP				0x3c
#define ISIF_VDINT				0x48
#define ISIF_ALAW				0x4c
#define ISIF_REC656IF				0x50
#define ISIF_CCDCFG				0x54
#define ISIF_DMA_CNTL				0x98
#define ISIF_VPFE_SYSCONFIG			0x104
#define ISIF_VPFE_CONFIG			0x108
#define ISIF_VPFE_IRQ_EOI			0x110
#define ISIF_VPFE_IRQ_STATUS_RAW		0x114
#define ISIF_VPFE_IRQ_STATUS			0x118
#define ISIF_VPFE_IRQ_ENABLE_SET		0x11c
#define ISIF_VPFE_IRQ_ENABLE_CLR		0x120
#define ISIF_REG_END				0x124

/***************************************************************
*	Define for various register bit mask and shifts for ISIF
****************************************************************/
#define ISIF_FID_POL_MASK			1
#define ISIF_FID_POL_SHIFT			4
#define ISIF_HD_POL_MASK			1
#define ISIF_HD_POL_SHIFT			3
#define ISIF_VD_POL_MASK			1
#define ISIF_VD_POL_SHIFT			2
#define ISIF_HSIZE_OFF_MASK			0xffffffe0
#define ISIF_32BYTE_ALIGN_VAL			31
#define ISIF_FRM_FMT_MASK			0x1
#define ISIF_FRM_FMT_SHIFT			7
#define ISIF_DATA_SZ_MASK			7
#define ISIF_DATA_SZ_SHIFT			8
#define ISIF_PIX_FMT_MASK			3
#define ISIF_PIX_FMT_SHIFT			12
#define ISIF_VP2SDR_DISABLE			0xFFFBFFFF
#define ISIF_WEN_ENABLE				(1 << 17)
#define ISIF_SDR2RSZ_DISABLE			0xFFF7FFFF
#define ISIF_VDHDEN_ENABLE			(1 << 16)
#define ISIF_LPF_ENABLE				(1 << 14)
#define ISIF_ALAW_ENABLE			(1 << 3)
#define ISIF_ALAW_GAMMA_WD_MASK			7
#define ISIF_BLK_CLAMP_ENABLE			(1 << 31)
#define ISIF_BLK_SGAIN_MASK			0x1F
#define ISIF_BLK_ST_PXL_MASK			0x7FFF
#define ISIF_BLK_ST_PXL_SHIFT			10
#define ISIF_BLK_SAMPLE_LN_MASK			7
#define ISIF_BLK_SAMPLE_LN_SHIFT		28
#define ISIF_BLK_SAMPLE_LINE_MASK		7
#define ISIF_BLK_SAMPLE_LINE_SHIFT		25
#define ISIF_BLK_DC_SUB_MASK			0x03FFF
#define ISIF_BLK_COMP_MASK			0xFF
#define ISIF_BLK_COMP_GB_COMP_SHIFT		8
#define ISIF_BLK_COMP_GR_COMP_SHIFT		16
#define ISIF_BLK_COMP_R_COMP_SHIFT		24
#define ISIF_LATCH_ON_VSYNC_DISABLE		(1 << 15)
#define ISIF_DATA_PACK_ENABLE			(1 << 11)
#define ISIF_HORZ_INFO_SPH_SHIFT		16
#define ISIF_VERT_START_SLV0_SHIFT		16
#define ISIF_VDINT_VDINT0_SHIFT			16
#define ISIF_VDINT_VDINT1_MASK			0xFFFF
#define ISIF_PPC_RAW				1
#define ISIF_DCSUB_DEFAULT_VAL			0
#define ISIF_CLAMP_DEFAULT_VAL			0
#define ISIF_COLPTN_VAL				0xBB11BB11
#define ISIF_TWO_BYTES_PER_PIXEL		2
#define ISIF_INTERLACED_IMAGE_INVERT		0x4B6D
#define ISIF_INTERLACED_NO_IMAGE_INVERT		0x0249
#define ISIF_PROGRESSIVE_IMAGE_INVERT		0x4000
#define ISIF_PROGRESSIVE_NO_IMAGE_INVERT	0
#define ISIF_INTERLACED_HEIGHT_SHIFT		1
#define ISIF_SYN_MODE_INPMOD_SHIFT		12
#define ISIF_SYN_MODE_INPMOD_MASK		3
#define ISIF_SYN_MODE_8BITS			(7 << 8)
#define ISIF_SYN_MODE_10BITS			(6 << 8)
#define ISIF_SYN_MODE_11BITS			(5 << 8)
#define ISIF_SYN_MODE_12BITS			(4 << 8)
#define ISIF_SYN_MODE_13BITS			(3 << 8)
#define ISIF_SYN_MODE_14BITS			(2 << 8)
#define ISIF_SYN_MODE_15BITS			(1 << 8)
#define ISIF_SYN_MODE_16BITS			(0 << 8)
#define ISIF_SYN_FLDMODE_MASK			1
#define ISIF_SYN_FLDMODE_SHIFT			7
#define ISIF_REC656IF_BT656_EN			3
#define ISIF_SYN_MODE_VD_POL_NEGATIVE		(1 << 2)
#define ISIF_CCDCFG_Y8POS_SHIFT			11
#define ISIF_CCDCFG_BW656_10BIT			(1 << 5)
#define ISIF_SDOFST_FIELD_INTERLEAVED		0x249
#define ISIF_NO_CULLING				0xffff00ff
#define ISIF_VPFE_VDINT0			(1 << 0)
#define ISIF_VPFE_VDINT1			(1 << 1)
#define ISIF_VPFE_VDINT2			(1 << 2)
#define ISIF_DMA_CNTL_OVERFLOW			(1 << 31)

#define ISIF_VPFE_CONFIG_PCLK_INV_SHIFT		0
#define ISIF_VPFE_CONFIG_PCLK_INV_MASK		1
#define ISIF_VPFE_CONFIG_PCLK_INV_NOT_INV	0
#define ISIF_VPFE_CONFIG_PCLK_INV_INV		1
#define ISIF_VPFE_CONFIG_VPFE_EN_SHIFT		1
#define ISIF_VPFE_CONFIG_VPFE_EN_MASK		2
#define ISIF_VPFE_CONFIG_VPFE_EN_DISABLE	0
#define ISIF_VPFE_CONFIG_VPFE_EN_ENABLE		1
#define ISIF_VPFE_CONFIG_VPFE_ST_SHIFT		2
#define ISIF_VPFE_CONFIG_VPFE_ST_MASK		4
#define ISIF_VPFE_CONFIG_VPFE_ST_OCP_ACTIVE	0
#define ISIF_VPFE_CONFIG_VPFE_ST_OCP_STANDBY	1


#endif
