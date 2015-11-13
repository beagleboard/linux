/*
 * AM43XX OPP table definitions.
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

/* From AM437x TRM, SPRUHL7 */
#define AM43XX_DEV_ATTR_OFFSET	0x610

/*
 * Bits [5:0] are OPP Disabled bits,
 * 1 = OPP is disabled and not available,
 * 0 = OPP available.
 */
#define MAX_FREQ_MASK		0x3f
#define MAX_FREQ_SHFT		0

#define EFUSE_OPP_50_300MHZ_BIT		(0x1 << 0)
#define EFUSE_OPP_100_600MHZ_BIT		(0x1 << 2)
#define EFUSE_OPP_120_720MHZ_BIT		(0x1 << 3)
#define EFUSE_OPP_TURBO_800MHZ_BIT		(0x1 << 4)
#define EFUSE_OPP_NITRO_1GHZ_BIT		(0x1 << 5)

static struct omap_opp_def am43xx_es1_0_opp_list[] __initdata = {
	/* MPU OPP1 - OPP50 */
	OPP_INITIALIZER("mpu", false, 300000000, 950000),
	/* MPU OPP2 - OPP100 */
	OPP_INITIALIZER("mpu", false, 600000000, 1100000),
	/* MPU OPP3 - OPP120 */
	OPP_INITIALIZER("mpu", false, 720000000, 1200000),
	/* MPU OPP3 - OPPTurbo */
	OPP_INITIALIZER("mpu", false, 800000000, 1260000),
	/* MPU OPP4 - OPPNitro */
	OPP_INITIALIZER("mpu", false, 1000000000, 1325000),
};

/**
 * am43xx_opp_init() - initialize am43xx opp table
 */
int __init am43xx_opp_init(void)
{
	int r = -ENODEV;
	u32 rev, val, max_freq;

	if (WARN(!soc_is_am43xx(), "Cannot init OPPs: unsupported SoC.\n"))
		return r;

	rev = omap_rev();

	switch (rev) {
	case AM437X_REV_ES1_0:
	case AM437X_REV_ES1_1:
	default:
		/*
		 * First read dev attr reg to detect supported frequency
		 */
		val = omap_ctrl_readl(AM43XX_DEV_ATTR_OFFSET);

		/*
		 * 1 = OPP is disabled and not available,
		 * 0 = OPP available.
		 */
		max_freq = ~val & MAX_FREQ_MASK;

		opp_def_list_enable_opp(am43xx_es1_0_opp_list,
			ARRAY_SIZE(am43xx_es1_0_opp_list),
			"mpu", 300000000,
			(max_freq & EFUSE_OPP_50_300MHZ_BIT) ? true : false);

		opp_def_list_enable_opp(am43xx_es1_0_opp_list,
			ARRAY_SIZE(am43xx_es1_0_opp_list),
			"mpu", 600000000,
			(max_freq & EFUSE_OPP_100_600MHZ_BIT) ? true : false);

		opp_def_list_enable_opp(am43xx_es1_0_opp_list,
			ARRAY_SIZE(am43xx_es1_0_opp_list),
			"mpu", 720000000,
			(max_freq & EFUSE_OPP_120_720MHZ_BIT) ? true : false);

		opp_def_list_enable_opp(am43xx_es1_0_opp_list,
			ARRAY_SIZE(am43xx_es1_0_opp_list),
			"mpu", 800000000,
			(max_freq & EFUSE_OPP_TURBO_800MHZ_BIT) ? true : false);

		opp_def_list_enable_opp(am43xx_es1_0_opp_list,
			ARRAY_SIZE(am43xx_es1_0_opp_list),
			"mpu", 1000000000,
			(max_freq & EFUSE_OPP_NITRO_1GHZ_BIT) ? true : false);

		r = omap_init_opp_table(am43xx_es1_0_opp_list,
					ARRAY_SIZE(am43xx_es1_0_opp_list));
		break;
	}

	return r;
}
