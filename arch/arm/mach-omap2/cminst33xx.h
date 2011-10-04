/*
 * am33xx Clock Management (CM) function prototypes
 *
 * Copyright (C) 2010 Nokia Corporation
 * Paul Walmsley
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef __ARCH_ASM_MACH_OMAP2_CMINST33XX_H
#define __ARCH_ASM_MACH_OMAP2_CMINST33XX_H

extern bool am33xx_cminst_is_clkdm_in_hwsup(s16 inst, u16 cdoffs);
extern void am33xx_cminst_clkdm_enable_hwsup(s16 inst, u16 cdoffs);
extern void am33xx_cminst_clkdm_disable_hwsup(s16 inst, u16 cdoffs);
extern void am33xx_cminst_clkdm_force_sleep(s16 inst, u16 cdoffs);
extern void am33xx_cminst_clkdm_force_wakeup(s16 inst, u16 cdoffs);

extern int am33xx_cminst_wait_module_ready(u16 inst, s16 cdoffs, u16 clkctrl_offs);

#ifdef CONFIG_SOC_OMAPAM33XX
extern int am33xx_cminst_wait_module_idle(u16 inst, s16 cdoffs,
					 u16 clkctrl_offs);

extern void am33xx_cminst_module_enable(u8 mode, u16 inst, s16 cdoffs,
				       u16 clkctrl_offs);
extern void am33xx_cminst_module_disable(u16 inst, s16 cdoffs,
					u16 clkctrl_offs);

#else

static inline int am33xx_cminst_wait_module_idle(u16 inst, s16 cdoffs,
					u16 clkctrl_offs)
{
	return 0;
}

static inline void am33xx_cminst_module_enable(u8 mode, u16 inst,
				s16 cdoffs, u16 clkctrl_offs)
{
}

static inline void am33xx_cminst_module_disable(u16 inst, s16 cdoffs,
				 u16 clkctrl_offs)
{
}

#endif

/*
 * In an ideal world, we would not export these low-level functions,
 * but this will probably take some time to fix properly
 */
extern u32 am33xx_cminst_read_inst_reg(s16 inst, u16 idx);
extern void am33xx_cminst_write_inst_reg(u32 val, s16 inst, u16 idx);
extern u32 am33xx_cminst_rmw_inst_reg_bits(u32 mask, u32 bits,
					   s16 inst, s16 idx);
extern u32 am33xx_cminst_set_inst_reg_bits(u32 bits, s16 inst, s16 idx);
extern u32 am33xx_cminst_clear_inst_reg_bits(u32 bits, s16 inst, s16 idx);
extern u32 am33xx_cminst_read_inst_reg_bits(u16 inst, s16 idx, u32 mask);

#endif
