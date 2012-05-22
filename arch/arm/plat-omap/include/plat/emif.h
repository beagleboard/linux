/*
 * EMIF register definitions for TI81xx and AM33xx
 *
 * Copyright (C) 2011 Texas Instruments, Inc. - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __EMIF_H
#define __EMIF_H

#define EMIF_MOD_ID_REV			(0x0)
#define EMIF4_0_SDRAM_STATUS		(0x04)
#define EMIF4_0_SDRAM_CONFIG		(0x08)
#define EMIF4_0_SDRAM_CONFIG2		(0x0C)
#define EMIF4_0_SDRAM_REF_CTRL		(0x10)
#define EMIF4_0_SDRAM_REF_CTRL_SHADOW	(0x14)
#define EMIF4_0_SDRAM_TIM_1		(0x18)
#define EMIF4_0_SDRAM_TIM_1_SHADOW	(0x1C)
#define EMIF4_0_SDRAM_TIM_2		(0x20)
#define EMIF4_0_SDRAM_TIM_2_SHADOW	(0x24)
#define EMIF4_0_SDRAM_TIM_3		(0x28)
#define EMIF4_0_SDRAM_TIM_3_SHADOW	(0x2C)
#define EMIF4_0_SDRAM_MGMT_CTRL		(0x38)
#define EMIF4_0_SDRAM_MGMT_CTRL_SHADOW	(0x3C)
#define EMIF4_0_IODFT_TLGC		(0x60)
#define EMIF4_0_ZQ_CONFIG		(0xC8)
#define EMIF4_0_DDR_PHY_CTRL_1		(0xE4)
#define EMIF4_0_DDR_PHY_CTRL_1_SHADOW	(0xE8)

#define SELF_REFRESH_ENABLE(m)		(0x2 << 8 | (m << 4))
#define SELF_REFRESH_DISABLE		(0x0 << 8)

#endif /* __EMIF_H */
