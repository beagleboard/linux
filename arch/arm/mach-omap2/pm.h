#ifndef __ARCH_ARM_MACH_OMAP2_PM_H
#define __ARCH_ARM_MACH_OMAP2_PM_H
/*
 * linux/arch/arm/mach-omap2/pm.h
 *
 * OMAP Power Management Routines
 *
 * Copyright (C) 2008 Nokia Corporation
 * Jouni Hogander
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

extern int omap2_pm_init(void);
extern int omap3_pm_init(void);

extern unsigned short enable_dyn_sleep;
extern atomic_t sleep_block;

#ifdef CONFIG_PM_DEBUG
extern u32 omap2_read_32k_sync_counter(void);
extern void omap2_pm_dump(int mode, int resume, unsigned int us);
extern void serial_console_fclk_mask(u32 *f1, u32 *f2);
extern void pm_init_serial_console(void);
extern void serial_console_sleep(int enable);
extern int omap2_pm_debug;
#else
#define omap2_read_32k_sync_counter() 0;
#define serial_console_sleep(enable) do; while(0)
#define pm_init_serial_console() do; while(0)
#define omap2_pm_dump(mode,resume,us) do; while(0)
#define serial_console_fclk_mask(f1,f2)  do; while(0)
#define omap2_pm_debug 0
#endif /* CONFIG_PM_DEBUG */
#endif
