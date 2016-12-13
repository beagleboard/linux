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

#include <linux/kbuild.h>

#ifndef __ASSEMBLER__
struct am33xx_pm_sram_addr {
	void (*do_wfi)(void);
	unsigned long *do_wfi_sz;
	unsigned long *resume_offset;
	unsigned long *emif_sram_table;
	unsigned long *ro_sram_data;
};

struct am33xx_pm_platform_data {
	int	(*init)(void);
	int	(*soc_suspend)(unsigned int state, int (*fn)(unsigned long));
	struct  am33xx_pm_sram_addr *(*get_sram_addrs)(void);
};

struct am33xx_pm_sram_data {
	u32 wfi_flags;
	u32 l2_aux_ctrl_val;
	u32 l2_prefetch_ctrl_val;
};

struct am33xx_pm_ro_sram_data {
	u32 amx3_pm_sram_data_virt;
	u32 amx3_pm_sram_data_phys;
};

extern inline void amx3_pm_asm_offsets(void)
{
	DEFINE(AMX3_PM_WFI_FLAGS_OFFSET,
	       offsetof(struct am33xx_pm_sram_data, wfi_flags));
	DEFINE(AMX3_PM_L2_AUX_CTRL_VAL_OFFSET,
	       offsetof(struct am33xx_pm_sram_data, l2_aux_ctrl_val));
	DEFINE(AMX3_PM_L2_PREFETCH_CTRL_VAL_OFFSET,
	       offsetof(struct am33xx_pm_sram_data, l2_prefetch_ctrl_val));
	DEFINE(AMX3_PM_SRAM_DATA_SIZE, sizeof(struct am33xx_pm_sram_data));

	BLANK();

	DEFINE(AMX3_PM_RO_SRAM_DATA_VIRT_OFFSET,
	       offsetof(struct am33xx_pm_ro_sram_data, amx3_pm_sram_data_virt));
	DEFINE(AMX3_PM_RO_SRAM_DATA_PHYS_OFFSET,
	       offsetof(struct am33xx_pm_ro_sram_data, amx3_pm_sram_data_phys));
	DEFINE(AMX3_PM_RO_SRAM_DATA_SIZE,
	       sizeof(struct am33xx_pm_ro_sram_data));
}

#endif /* __ASSEMBLER__ */
#endif /* _LINUX_PLATFORM_DATA_PM33XX_H */
