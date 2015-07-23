/*
 * DRA7XX OPP table definitions.
 *
 * Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/io.h>
#include <linux/module.h>

#include "control.h"
#include "omap_opp_data.h"
#include "pm.h"
#include "soc.h"

/* From DRA7x TRM, SPRUHI2Q */
#define DRA7XX_STD_FUSE_DIE_ID_2		0x4AE0C20C

#define SPEED_VAL_MASK				(0x1F << 19)
#define SPEED_VAL_SHFT				19

/*
 * These values indicate the highest available OPP so if the value
 * is found we want that OPP and all below it
 */
#define EFUSE_HAS_OD_MPU_OPP		11
#define EFUSE_HAS_HIGH_MPU_OPP		15
#define EFUSE_HAS_ALL_MPU_OPP		23

/*
 * We only include OPP_OD and OPP_HIGH here, OPP_NOM is common to all
 * variants and left to be supplied by DT.
 */
static struct omap_opp_def dra7xx_es1_0_opp_list[] __initdata = {
	/* MPU OPP - OPP_OD */
	OPP_INITIALIZER("mpu", false, 1176000000, 1160000),
	/* MPU OPP - OPP_HIGH */
	OPP_INITIALIZER("mpu", false, 1500000000, 1210000),
};

/**
 * dra7xx_opp_init() - initialize dra7xx opp table
 */
int __init dra7xx_opp_init(void)
{
	int r = -ENODEV;
	u32 rev, val, max_freq;
	void __iomem *speed_reg_addr;

	if (WARN(!soc_is_dra7xx(), "Cannot init OPPs: unsupported SoC.\n"))
		return r;

	speed_reg_addr = ioremap(DRA7XX_STD_FUSE_DIE_ID_2, SZ_1);

	rev = omap_rev();

	switch (rev) {
	case DRA752_REV_ES1_0:
	case DRA752_REV_ES1_1:
	case DRA722_REV_ES1_0:
	default:
		/*
		 * First read speed reg to detect supported frequency
		 */
		val = readl(speed_reg_addr);

		max_freq = val & SPEED_VAL_MASK;
		max_freq >>= SPEED_VAL_SHFT;

		switch (max_freq) {
		case EFUSE_HAS_ALL_MPU_OPP:
		case EFUSE_HAS_HIGH_MPU_OPP:
			opp_def_list_enable_opp(dra7xx_es1_0_opp_list,
				ARRAY_SIZE(dra7xx_es1_0_opp_list),
				"mpu", 1500000000, true);
		/* FALLTHROUGH */
		case EFUSE_HAS_OD_MPU_OPP:
			opp_def_list_enable_opp(dra7xx_es1_0_opp_list,
				ARRAY_SIZE(dra7xx_es1_0_opp_list),
				"mpu", 1176000000, true);
		};

		r = omap_init_opp_table(dra7xx_es1_0_opp_list,
					ARRAY_SIZE(dra7xx_es1_0_opp_list));
		break;
	}

	return r;
}
