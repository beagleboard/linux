/*
 * OMAP2/3 Power Management Routines
 *
 * Copyright (C) 2008 Nokia Corporation
 * Jouni Hogander
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef __ARCH_ARM_MACH_OMAP2_PM_H
#define __ARCH_ARM_MACH_OMAP2_PM_H

#include <linux/err.h>

#include "powerdomain.h"

#ifdef CONFIG_CPU_IDLE
extern int am33xx_idle_init(bool ddr3, void (*do_idle)(u32 wfi_flags));
extern int am437x_idle_init(void);
extern int __init omap3_idle_init(void);
extern int __init omap4_idle_init(void);
#else
static inline int am33xx_idle_init(bool ddr3, void (*do_sram_cpuidle)
				   (u32 wfi_flags))
{
	return 0;
}

static inline int am437x_idle_init(void)
{
	return 0;
}

static inline int omap3_idle_init(void)
{
	return 0;
}

static inline int omap4_idle_init(void)
{
	return 0;
}
#endif

extern void *omap3_secure_ram_storage;
extern void omap3_pm_off_mode_enable(int);
extern void omap_sram_idle(void);
extern int omap_pm_clkdms_setup(struct clockdomain *clkdm, void *unused);

#if defined(CONFIG_PM_OPP)
extern int omap3_opp_init(void);
extern int omap4_opp_init(void);
extern int am33xx_opp_init(void);
extern int am43xx_opp_init(void);
extern int dra7xx_opp_init(void);
#else
static inline int omap3_opp_init(void)
{
	return -EINVAL;
}
static inline int omap4_opp_init(void)
{
	return -EINVAL;
}
static inline int am33xx_opp_init(void)
{
	return -EINVAL;
}
static inline int am43xx_opp_init(void)
{
	return -EINVAL;
}
static inline int dra7xx_opp_init(void)
{
	return -EINVAL;
}
#endif

extern int omap3_pm_get_suspend_state(struct powerdomain *pwrdm);
extern int omap3_pm_set_suspend_state(struct powerdomain *pwrdm, int state);

#ifdef CONFIG_PM_DEBUG
extern u32 enable_off_mode;
#else
#define enable_off_mode 0
#endif

#if defined(CONFIG_PM_DEBUG) && defined(CONFIG_DEBUG_FS)
extern void pm_dbg_update_time(struct powerdomain *pwrdm, int prev);
#else
#define pm_dbg_update_time(pwrdm, prev) do {} while (0);
#endif /* CONFIG_PM_DEBUG */

/* 24xx */
extern void omap24xx_idle_loop_suspend(void);
extern unsigned int omap24xx_idle_loop_suspend_sz;

extern void omap24xx_cpu_suspend(u32 dll_ctrl, void __iomem *sdrc_dlla_ctrl,
					void __iomem *sdrc_power);
extern unsigned int omap24xx_cpu_suspend_sz;

/* 3xxx */
extern void omap34xx_cpu_suspend(int save_state);

/* omap3_do_wfi function pointer and size, for copy to SRAM */
extern void omap3_do_wfi(void);
extern unsigned int omap3_do_wfi_sz;
/* ... and its pointer from SRAM after copy */
extern void (*omap3_do_wfi_sram)(void);

/* for sharing core pm ops with amx3 pm modules */
struct am33xx_pm_ops {
	int	(*init)(void (*do_sram_cpuidle)(u32 wfi_flags));
	int	(*soc_suspend)(unsigned int state, int (*fn)(unsigned long),
			       unsigned long args);
	int	(*cpu_suspend)(int (*fn)(unsigned long), unsigned long args);
	void (*save_context)(void);
	void (*restore_context)(void);
	void (*prepare_rtc_suspend)(void);
	void (*prepare_rtc_resume)(void);
	int (*check_off_mode_enable)(void);
	void __iomem *(*get_rtc_base_addr)(void);
};

/* for sharing asm function addrs with amx3 pm modules */
struct am33xx_pm_sram_addr {
	void (*do_wfi)(void);
	unsigned long *do_wfi_sz;
	unsigned long *resume_offset;
	unsigned long *emif_sram_table;
	unsigned long *rtc_base_virt;
	phys_addr_t rtc_resume_phys_addr;
};

struct am33xx_pm_ops *amx3_get_pm_ops(void);
struct am33xx_pm_sram_addr *amx3_get_sram_addrs(void);

#define WFI_FLAG_SELF_REFRESH		(1 << 2)
#define WFI_FLAG_SAVE_EMIF		(1 << 3)
#define WFI_FLAG_WAKE_M3		(1 << 4)
#define WFI_FLAG_DISABLE_EMIF		(1 << 7)
#define WFI_FLAG_RTC_ONLY		(1 << 8)

extern struct am33xx_pm_sram_addr am33xx_pm_sram;
extern struct am33xx_pm_sram_addr am43xx_pm_sram;

/* save_secure_ram_context function pointer and size, for copy to SRAM */
extern int save_secure_ram_context(u32 *addr);
extern unsigned int save_secure_ram_context_sz;

extern void omap3_save_scratchpad_contents(void);

#define PM_RTA_ERRATUM_i608		(1 << 0)
#define PM_SDRC_WAKEUP_ERRATUM_i583	(1 << 1)
#define PM_PER_MEMORIES_ERRATUM_i582	(1 << 2)

#if defined(CONFIG_PM) && defined(CONFIG_ARCH_OMAP3)
extern u16 pm34xx_errata;
#define IS_PM34XX_ERRATUM(id)		(pm34xx_errata & (id))
extern void enable_omap3630_toggle_l2_on_restore(void);
#else
#define IS_PM34XX_ERRATUM(id)		0
static inline void enable_omap3630_toggle_l2_on_restore(void) { }
#endif		/* defined(CONFIG_PM) && defined(CONFIG_ARCH_OMAP3) */

#define PM_OMAP4_ROM_SMP_BOOT_ERRATUM_GICD	(1 << 0)
#define PM_OMAP4_CPU_OSWR_DISABLE		(1 << 1)

#if defined(CONFIG_PM) && (defined(CONFIG_ARCH_OMAP4) ||\
	   defined(CONFIG_SOC_OMAP5) || defined (CONFIG_SOC_DRA7XX))
extern u16 pm44xx_errata;
#define IS_PM44XX_ERRATUM(id)		(pm44xx_errata & (id))
#else
#define IS_PM44XX_ERRATUM(id)		0
#endif

#ifdef CONFIG_POWER_AVS_OMAP
extern int omap_devinit_smartreflex(void);
extern void omap_enable_smartreflex_on_init(void);
#else
static inline int omap_devinit_smartreflex(void)
{
	return -EINVAL;
}

static inline void omap_enable_smartreflex_on_init(void) {}
#endif

#ifdef CONFIG_TWL4030_CORE
extern int omap3_twl_init(void);
extern int omap4_twl_init(void);
extern int omap3_twl_set_sr_bit(bool enable);
#else
static inline int omap3_twl_init(void)
{
	return -EINVAL;
}
static inline int omap4_twl_init(void)
{
	return -EINVAL;
}
#endif

#ifdef CONFIG_PM
extern void omap_pm_setup_oscillator(u32 tstart, u32 tshut);
extern void omap_pm_get_oscillator(u32 *tstart, u32 *tshut);
extern void omap_pm_setup_sr_i2c_pcb_length(u32 mm);
void amx3_common_pm_init(void);
#else
static inline void omap_pm_setup_oscillator(u32 tstart, u32 tshut) { }
static inline void omap_pm_get_oscillator(u32 *tstart, u32 *tshut) { *tstart = *tshut = 0; }
static inline void omap_pm_setup_sr_i2c_pcb_length(u32 mm) { }
static inline void amx3_common_pm_init(void) { }
#endif

#ifdef CONFIG_SUSPEND
void omap_common_suspend_init(void *pm_suspend);
#else
static inline void omap_common_suspend_init(void *pm_suspend)
{
}
#endif /* CONFIG_SUSPEND */
#endif
