/*
 * TI pm33xx platform data
 *
 * Copyright (C) 2016 Texas Instruments, Inc.
 *
 * Dave Gerlach <d-gerlach@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _LINUX_PLATFORM_DATA_PM33XX_H
#define _LINUX_PLATFORM_DATA_PM33XX_H

#define WFI_FLAG_RTC_ONLY	BIT(8)
#ifndef __ASSEMBLER__
struct am33xx_pm_sram_addr {
	void (*do_wfi)(void);
	unsigned long *do_wfi_sz;
	unsigned long *resume_offset;
	unsigned long *emif_sram_table;
	unsigned long *rtc_base_virt;
	phys_addr_t rtc_resume_phys_addr;
};

struct am33xx_pm_platform_data {
	int	(*init)(void);
	int	(*soc_suspend)(unsigned int state, int (*fn)(unsigned long),
			       unsigned long args);
	struct  am33xx_pm_sram_addr *pm_sram_addr;
	void (*save_context)(void);
	void (*restore_context)(void);
	void (*prepare_rtc_suspend)(void);
	void (*prepare_rtc_resume)(void);
	int (*check_off_mode_enable)(void);
	void __iomem *(*get_rtc_base_addr)(void);
};
#endif /* __ASSEMBLER__ */
#endif /* _LINUX_PLATFORM_DATA_PM33XX_H */
