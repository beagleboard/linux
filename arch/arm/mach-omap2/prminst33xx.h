/*
 * AM33XX Power/Reset Management (PRM) function prototypes
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
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

#ifndef __ARCH_ASM_MACH_OMAP2_PRMINST33XX_H
#define __ARCH_ASM_MACH_OMAP2_PRMINST33XX_H

extern u32 am33xx_prminst_read_inst_reg(s16 inst, u16 idx);
extern void am33xx_prminst_write_inst_reg(u32 val, s16 inst, u16 idx);
extern u32 am33xx_prminst_rmw_inst_reg_bits(u32 mask, u32 bits,
					   s16 inst, s16 idx);
extern u32 am33xx_prminst_is_hardreset_asserted(s16 domain, s16 idx, u32 mask);
extern int am33xx_prminst_assert_hardreset(s16 prm_mod, u8 shift);
extern int am33xx_prminst_deassert_hardreset(s16 prm_mod, u8 rst_shift,
					u8 st_shift);
extern void am33xx_prm_global_warm_sw_reset(void);

#endif
