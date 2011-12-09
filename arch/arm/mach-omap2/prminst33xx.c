/*
 * AM33XX PRM instance functions
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

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/io.h>

#include "common.h"

#include "prm33xx.h"
#include "prminst33xx.h"
#include "prm-regbits-33xx.h"

#define AM33XX_PRM_MOD_SIZE	0x100
#define AM33XX_PRM_MOD_START	AM33XX_PRM_PER_MOD
#define PRM_REG_SZ		0x4

/*
 * PRM Offsets are screwed up, and they are not consistent across modules.
 * Below are the offsets for PWRSTCTRL and PWRSTST for respective modules.
 */
static u16 off_fixup[][4] = {
	/* PRM_PER_MOD: 0x0C, 0x08, 0x00, 0x04 */
	{
		AM33XX_PM_PER_PWRSTCTRL_OFFSET,
		AM33XX_PM_PER_PWRSTST_OFFSET,
		AM33XX_RM_PER_RSTCTRL_OFFSET,
		AM33XX_RM_PER_RSTST_OFFSET,
	},
	/* PRM_WKUP_MOD: 0x04, 0x08, 0x00, 0x0C */
	{
		AM33XX_PM_WKUP_PWRSTCTRL_OFFSET,
		AM33XX_PM_WKUP_PWRSTST_OFFSET,
		AM33XX_RM_WKUP_RSTCTRL_OFFSET,
		AM33XX_RM_WKUP_RSTST_OFFSET,
	},
	/* AM33XX_PRM_MPU_MOD: 0x00, 0x04, 0x08, NA */
	{
		AM33XX_PM_MPU_PWRSTCTRL_OFFSET,
		AM33XX_PM_MPU_PWRSTST_OFFSET,
		AM33XX_RM_MPU_RSTST_OFFSET,
		AM33XX_RM_MPU_RSTST_OFFSET,
	},
	/* PRM_DEVICE_MOD: NA, NA, 0x00, 0x08 */
	{
		0x0,
		0x0,
		AM33XX_PRM_RSTCTRL_OFFSET,
		AM33XX_PRM_RSTST_OFFSET,
	},
	/* PRM_RTC_MOD: 0x00, 0x04, NA, NA */
	{
		AM33XX_PM_RTC_PWRSTCTRL_OFFSET,
		AM33XX_PM_RTC_PWRSTST_OFFSET,
		0x0,
		0x0,
	},
	/* PRM_GFX_MOD: 0x00, 0x10, 0x04, 0x14 */
	{
		AM33XX_PM_GFX_PWRSTCTRL_OFFSET,
		AM33XX_PM_GFX_PWRSTST_OFFSET,
		AM33XX_RM_GFX_RSTCTRL_OFFSET,
		AM33XX_RM_GFX_RSTST_OFFSET,
	},
	/* PRM_CEFUSE_MOD: 0x00, 0x04, NA, NA */
	{
		AM33XX_PM_CEFUSE_PWRSTCTRL_OFFSET,
		AM33XX_PM_CEFUSE_PWRSTST_OFFSET,
		0x0,
		0x0,
	},
};

/* Read a register in a PRM instance */
u32 am33xx_prminst_read_inst_reg(s16 inst, u16 idx)
{
	int i = (inst - AM33XX_PRM_MOD_START) / AM33XX_PRM_MOD_SIZE;

	return __raw_readl(prm_base + inst + off_fixup[i][idx / PRM_REG_SZ]);
}

/* Write into a register in a PRM instance */
void am33xx_prminst_write_inst_reg(u32 val, s16 inst, u16 idx)
{
	int i = (inst - AM33XX_PRM_MOD_START) / AM33XX_PRM_MOD_SIZE;

	__raw_writel(val, prm_base + inst + off_fixup[i][idx / PRM_REG_SZ]);
}

/* Read-modify-write a register in PRM. Caller must lock */
u32 am33xx_prminst_rmw_inst_reg_bits(u32 mask, u32 bits, s16 inst, s16 idx)
{
	u32 v;

	v = am33xx_prminst_read_inst_reg(inst, idx);
	v &= ~mask;
	v |= bits;
	am33xx_prminst_write_inst_reg(v, inst, idx);

	return v;
}

/**
 * am33xx_prminst_is_hardreset_asserted - read the HW reset line state of
 * submodules contained in the hwmod module
 * @prm_mod: PRM submodule base (e.g. CORE_MOD)
 * @idx: register bit offset for calculating correct offset
 * @mask: register bit mask
 *
 * Returns 1 if the (sub)module hardreset line is currently asserted,
 * 0 if the (sub)module hardreset line is not currently asserted, or
 * -EINVAL if called while running on a non-OMAP2/3 chip.
 */

u32 am33xx_prminst_is_hardreset_asserted(s16 prm_mod, s16 idx, u32 mask)
{
	u32 v;

	v = am33xx_prminst_read_inst_reg(prm_mod, idx);
	v &= mask;
	v >>= __ffs(mask);

	return v;
}

/**
 * am33xx_prminst_assert_hardreset - assert the HW reset line of a submodule
 * @prm_mod: PRM submodule base (e.g. GFX_MOD)
 * @shift: register bit shift corresponding to the reset line to assert
 *
 * Some IPs like iva or PRUSS contain processors that require an HW
 * reset line to be asserted / deasserted in order to fully enable the
 * IP.  These modules may have multiple hard-reset lines that reset
 * different 'submodules' inside the IP block.  This function will
 * place the submodule into reset.  Returns 0 upon success or -EINVAL
 * upon an argument error.
 */

int am33xx_prminst_assert_hardreset(s16 prm_mod, u8 shift)
{
	u32 mask;

	if (!cpu_is_am33xx())
		return -EINVAL;

	mask = 1 << shift;
	/* assert the reset control line */
	am33xx_prminst_rmw_inst_reg_bits(mask, mask, prm_mod, AM33XX_PM_RSTCTRL);

	return 0;
}

/**
 * am33xx_prminst_deassert_hardreset - deassert a submodule hardreset line and
 * wait
 * @prm_mod: PRM submodule base (e.g. CORE_MOD)
 * @rst_shift: register bit shift corresponding to the reset line to deassert
 * @st_shift: register bit shift for the status of the deasserted submodule
 *
 * Some IPs like SGX, PRUSS and M3 contain processors that require an HW
 * reset line to be asserted / deasserted in order to fully enable the
 * IP.
 * This function will take the submodule out of reset and wait until the
 * PRCM indicates that the reset has completed before returning.
 * Returns 0 upon success or -EINVAL upon an argument error,
 * -EEXIST if the submodule was already out of reset, or -EBUSY if the
 * submodule did not exit reset promptly.
 */
int am33xx_prminst_deassert_hardreset(s16 prm_mod, u8 rst_shift, u8 st_shift)
{
	u32 rst, st;

	if (!cpu_is_am33xx())
		return -EINVAL;

	rst = 1 << rst_shift;
	st = 1 << st_shift;

	/* Clear the reset status by writing 1 to the status bit */
	am33xx_prminst_rmw_inst_reg_bits(0xffffffff, st, prm_mod,
							AM33XX_PM_RSTST);
	/* de-assert the reset control line */
	am33xx_prminst_rmw_inst_reg_bits(rst, 0, prm_mod, AM33XX_PM_RSTCTRL);
	/* TODO: wait the status to be set */

	return 0;
}
