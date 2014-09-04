/*
 * AM33XX OPP table definitions.
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

/*
 * Errata 1.0.15: OPP50 Operation on MPU Domain is Not Supported.
 *
 * To minimize power consumption, the ARM Cortex-A8 may be operated at
 * the lower frequency defined by OPP50, but the respective voltage
 * domain VDD_MPU must be operated as defined by OPP100. So MPU OPP50
 * definition is modified to 275MHz, 1.1V.
 */
static struct omap_opp_def am33xx_es1_0_opp_list[] __initdata = {
	/* MPU OPP1 - OPP50 */
	OPP_INITIALIZER("mpu", true, 275000000, 1100000),
	/* MPU OPP2 - OPP100 */
	OPP_INITIALIZER("mpu", true, 500000000, 1100000),
	/* MPU OPP3 - OPP120 */
	OPP_INITIALIZER("mpu", true, 600000000, 1200000),
	/* MPU OPP4 - OPPTurbo */
	OPP_INITIALIZER("mpu", true, 720000000, 1260000),
};

static struct omap_opp_def am33xx_es2_x_opp_list[] __initdata = {
	/* MPU OPP1 - OPP50 or OPP100 */
	OPP_INITIALIZER("mpu", true, 300000000, 950000),
	/* MPU OPP2 - OPP100 */
	OPP_INITIALIZER("mpu", true, 600000000, 1100000),
	/* MPU OPP3 - OPP120 */
	OPP_INITIALIZER("mpu", true, 720000000, 1200000),
	/* MPU OPP4 - OPPTurbo */
	OPP_INITIALIZER("mpu", true, 800000000, 1260000),
	/* MPU OPP5 - OPPNitro */
	OPP_INITIALIZER("mpu", true, 1000000000, 1325000),
};

/* From AM335x TRM, SPRUH73H, Section 9.3.50 */
#define AM33XX_EFUSE_SMA_OFFSET	0x7fc

/*
 * Bits [12:0] are OPP Disabled bits,
 * 1 = OPP is disabled and not available,
 * 0 = OPP available.
 */
#define MAX_FREQ_MASK		0x1fff
#define MAX_FREQ_SHFT		0

/* ES 2.1 eFuse Values for enabling/diasbling specific OPPs */
#define EFUSE_OPP_50_300MHZ_BIT		(0x1 << 4)
#define EFUSE_OPP_100_300MHZ_BIT	(0x1 << 5)
#define EFUSE_OPP_100_600MHZ_BIT	(0x1 << 6)
#define EFUSE_OPP_120_720MHZ_BIT	(0x1 << 7)
#define EFUSE_OPP_TURBO_800MHZ_BIT	(0x1 << 8)
#define EFUSE_OPP_NITRO_1GHZ_BIT	(0x1 << 9)

/**
 * am33xx_opp_init() - initialize am33xx opp table
 */
int __init am33xx_opp_init(void)
{
	int r = -ENODEV;
	u32 rev, val, max_freq;

	if (WARN(!soc_is_am33xx(), "Cannot init OPPs: unsupported SoC.\n"))
		return r;

	rev = omap_rev();

	switch (rev) {
	case AM335X_REV_ES1_0:
		r = omap_init_opp_table(am33xx_es1_0_opp_list,
			ARRAY_SIZE(am33xx_es1_0_opp_list));
		break;

	case AM335X_REV_ES2_1:
		/*
		 * First read efuse sma reg to detect package type and
		 * supported frequency
		 */
		val = omap_ctrl_readl(AM33XX_EFUSE_SMA_OFFSET);

		if (!(val & MAX_FREQ_MASK)) {
			/*
			* if mpu max freq is not populated, fall back to
			* PG 2.0 OPP settings.
			*/
			r =
			omap_init_opp_table(am33xx_es2_x_opp_list,
					ARRAY_SIZE(am33xx_es2_x_opp_list));
			break;
		}

		/*
		 * 1 = OPP is disabled and not available,
		 * 0 = OPP available.
		 */
		max_freq = ~val & MAX_FREQ_MASK;

		/*
		 * 300 MHz OPP is special because one of two different u_volt
		 * values so check and update voltage value to OPP 100 level
		 * if efuse is set.
		 */
		if (max_freq & EFUSE_OPP_100_300MHZ_BIT) {
			opp_def_list_update_opp_voltage(am33xx_es2_x_opp_list,
				ARRAY_SIZE(am33xx_es2_x_opp_list),
				"mpu", 300000000, 1100000);
			opp_def_list_enable_opp(am33xx_es2_x_opp_list,
				ARRAY_SIZE(am33xx_es2_x_opp_list),
				"mpu", 300000000, true);
		} else if (max_freq & EFUSE_OPP_50_300MHZ_BIT) {
			opp_def_list_enable_opp(am33xx_es2_x_opp_list,
				ARRAY_SIZE(am33xx_es2_x_opp_list),
				"mpu", 300000000, true);
		} else {
			opp_def_list_enable_opp(am33xx_es2_x_opp_list,
				ARRAY_SIZE(am33xx_es2_x_opp_list),
				"mpu", 300000000, false);
		}

		opp_def_list_enable_opp(am33xx_es2_x_opp_list,
			ARRAY_SIZE(am33xx_es2_x_opp_list),
			"mpu", 600000000,
			(max_freq & EFUSE_OPP_100_600MHZ_BIT) ? true : false);

		opp_def_list_enable_opp(am33xx_es2_x_opp_list,
			ARRAY_SIZE(am33xx_es2_x_opp_list),
			"mpu", 720000000,
			(max_freq & EFUSE_OPP_120_720MHZ_BIT) ? true : false);

		opp_def_list_enable_opp(am33xx_es2_x_opp_list,
			ARRAY_SIZE(am33xx_es2_x_opp_list),
			"mpu", 800000000,
			(max_freq & EFUSE_OPP_TURBO_800MHZ_BIT) ? true : false);

		opp_def_list_enable_opp(am33xx_es2_x_opp_list,
			ARRAY_SIZE(am33xx_es2_x_opp_list),
			"mpu", 1000000000,
			(max_freq & EFUSE_OPP_NITRO_1GHZ_BIT) ? true : false);

		r = omap_init_opp_table(am33xx_es2_x_opp_list,
			ARRAY_SIZE(am33xx_es2_x_opp_list));
		break;

	case AM335X_REV_ES2_0:
	/* FALLTHROUGH */
	default:
		r = omap_init_opp_table(am33xx_es2_x_opp_list,
			ARRAY_SIZE(am33xx_es2_x_opp_list));
	}

	return r;
}
