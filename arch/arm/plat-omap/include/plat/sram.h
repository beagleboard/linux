/*
 * arch/arm/plat-omap/include/mach/sram.h
 *
 * Interface for functions that need to be run in internal SRAM
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ARCH_ARM_OMAP_SRAM_H
#define __ARCH_ARM_OMAP_SRAM_H

#ifndef __ASSEMBLY__
#include <linux/slab.h>
#include <linux/genalloc.h>
#include <asm/fncpy.h>

extern struct gen_pool *omap_gen_pool;

/*
 * Note that fncpy requires the SRAM address to be aligned to an 8-byte
 * boundary, so the min_alloc_order for the pool is set appropriately.
 */
#define omap_sram_push(funcp, size) ({					\
	typeof(&(funcp)) _res;						\
	size_t _sz = size;						\
	void *_sram = (void *) gen_pool_alloc(omap_gen_pool, _sz);	\
	_res = (_sram ? fncpy(_sram, &(funcp), _sz) : NULL);		\
	if (!_res)							\
		pr_err("Not enough space in SRAM\n");			\
	_res;								\
})

extern void omap_sram_reprogram_clock(u32 dpllctl, u32 ckctl);

extern void omap2_sram_ddr_init(u32 *slow_dll_ctrl, u32 fast_dll_ctrl,
				u32 base_cs, u32 force_unlock);
extern void omap2_sram_reprogram_sdrc(u32 perf_level, u32 dll_val,
				      u32 mem_type);
extern u32 omap2_set_prcm(u32 dpll_ctrl_val, u32 sdrc_rfr_val, int bypass);

extern u32 omap3_configure_core_dpll(
			u32 m2, u32 unlock_dll, u32 f, u32 inc,
			u32 sdrc_rfr_ctrl_0, u32 sdrc_actim_ctrl_a_0,
			u32 sdrc_actim_ctrl_b_0, u32 sdrc_mr_0,
			u32 sdrc_rfr_ctrl_1, u32 sdrc_actim_ctrl_a_1,
			u32 sdrc_actim_ctrl_b_1, u32 sdrc_mr_1);
extern void omap3_sram_restore_context(void);

/* Do not use these */
extern void omap1_sram_reprogram_clock(u32 ckctl, u32 dpllctl);
extern unsigned long omap1_sram_reprogram_clock_sz;

extern void omap24xx_sram_reprogram_clock(u32 ckctl, u32 dpllctl);
extern unsigned long omap24xx_sram_reprogram_clock_sz;

extern void omap242x_sram_ddr_init(u32 *slow_dll_ctrl, u32 fast_dll_ctrl,
						u32 base_cs, u32 force_unlock);
extern unsigned long omap242x_sram_ddr_init_sz;

extern u32 omap242x_sram_set_prcm(u32 dpll_ctrl_val, u32 sdrc_rfr_val,
						int bypass);
extern unsigned long omap242x_sram_set_prcm_sz;

extern void omap242x_sram_reprogram_sdrc(u32 perf_level, u32 dll_val,
						u32 mem_type);
extern unsigned long omap242x_sram_reprogram_sdrc_sz;


extern void omap243x_sram_ddr_init(u32 *slow_dll_ctrl, u32 fast_dll_ctrl,
						u32 base_cs, u32 force_unlock);
extern unsigned long omap243x_sram_ddr_init_sz;

extern u32 omap243x_sram_set_prcm(u32 dpll_ctrl_val, u32 sdrc_rfr_val,
						int bypass);
extern unsigned long omap243x_sram_set_prcm_sz;

extern void omap243x_sram_reprogram_sdrc(u32 perf_level, u32 dll_val,
						u32 mem_type);
extern unsigned long omap243x_sram_reprogram_sdrc_sz;

extern u32 omap3_sram_configure_core_dpll(
			u32 m2, u32 unlock_dll, u32 f, u32 inc,
			u32 sdrc_rfr_ctrl_0, u32 sdrc_actim_ctrl_a_0,
			u32 sdrc_actim_ctrl_b_0, u32 sdrc_mr_0,
			u32 sdrc_rfr_ctrl_1, u32 sdrc_actim_ctrl_a_1,
			u32 sdrc_actim_ctrl_b_1, u32 sdrc_mr_1);
extern unsigned long omap3_sram_configure_core_dpll_sz;

#ifdef CONFIG_PM
extern void am33xx_push_sram_idle(void);
extern void omap_push_sram_idle(void);
#else
static inline void am33xx_push_sram_idle(void) {}
static inline void omap_push_sram_idle(void) {}
#endif /* CONFIG_PM */

#endif /* __ASSEMBLY__ */

/*
 * OMAP2+: define the SRAM PA addresses.
 * Used by the SRAM management code and the idle sleep code.
 */
#define OMAP2_SRAM_PA		0x40200000
#define OMAP3_SRAM_PA           0x40200000
#ifdef CONFIG_OMAP4_ERRATA_I688
#define OMAP4_SRAM_PA		0x40304000
#define OMAP4_SRAM_VA		0xfe404000
#else
#define OMAP4_SRAM_PA		0x40300000
#endif
#define AM33XX_SRAM_PA		0x40300000
#endif
