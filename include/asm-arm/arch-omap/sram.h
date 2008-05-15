/*
 * linux/include/asm-arm/arch-omap/sram.h
 *
 * Interface for functions that need to be run in internal SRAM
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ARCH_ARM_OMAP_SRAM_H
#define __ARCH_ARM_OMAP_SRAM_H

#include <linux/poison.h>    /* for SRAM_VA_MAGIC */

extern int __init omap_sram_init(void);
extern void * omap_sram_push(void * start, unsigned long size);
extern int omap_sram_patch_va(void *srcfn, void *srcd, void *sramfn, void __iomem *d);
extern void omap_sram_reprogram_clock(u32 dpllctl, u32 ckctl);

extern void omap2_sram_ddr_init(u32 *slow_dll_ctrl, u32 fast_dll_ctrl,
				u32 base_cs, u32 force_unlock);
extern void omap2_sram_reprogram_sdrc(u32 perf_level, u32 dll_val,
				      u32 mem_type);
extern u32 omap2_set_prcm(u32 dpll_ctrl_val, u32 sdrc_rfr_val, int bypass);

/* Do not use these */
extern void omap1_sram_reprogram_clock(u32 ckctl, u32 dpllctl);
extern unsigned long omap1_sram_reprogram_clock_sz;

extern void omap24xx_sram_reprogram_clock(u32 ckctl, u32 dpllctl);
extern unsigned long omap24xx_sram_reprogram_clock_sz;

extern void omap24xx_sram_ddr_init(u32 *slow_dll_ctrl, u32 fast_dll_ctrl,
						u32 base_cs, u32 force_unlock);
extern unsigned long omap24xx_sram_ddr_init_sz;

extern u32 omap24xx_sram_set_prcm(u32 dpll_ctrl_val, u32 sdrc_rfr_val,
						int bypass);
extern unsigned long omap24xx_sram_set_prcm_sz;

extern void omap24xx_sram_reprogram_sdrc(u32 perf_level, u32 dll_val,
						u32 mem_type);
extern unsigned long omap24xx_sram_reprogram_sdrc_sz;


extern void omap34xx_sram_ddr_init(u32 *slow_dll_ctrl, u32 fast_dll_ctrl,
						u32 base_cs, u32 force_unlock);
extern unsigned long omap34xx_sram_ddr_init_sz;

extern void omap34xx_sram_reprogram_sdrc(u32 perf_level, u32 dll_val,
						u32 mem_type);
extern unsigned long omap34xx_sram_reprogram_sdrc_sz;

extern u32 omap34xx_sram_set_prcm(u32 dpll_ctrl_val, u32 sdrc_rfr_val,
						int bypass);
extern unsigned long omap34xx_sram_set_prcm_sz;

extern u32 omap34xx_sram_reprogram_gpmc(u32 perf_level);
extern unsigned long omap34xx_sram_reprogram_gpmc_sz;

extern u32 omap34xx_sram_configure_core_dpll(u32 m, u32 n, u32 freqsel, u32 m2);
extern unsigned long omap34xx_sram_configure_core_dpll_sz;

#endif
