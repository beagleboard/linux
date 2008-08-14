/*
 * linux/arch/arm/mach-omap2/bci.c
 *
 * TWL4030 BCI platform device setup/initialization
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <mach/bci.h>

#if defined(CONFIG_TWL4030_BCI_BATTERY) || \
	defined(CONFIG_TWL4030_BCI_BATTERY_MODULE)
/*
 * Thermistor Calibration for Current Source and MADC
 * Tolerance (for THS05-3H103F)
 */
static int sdp3430_batt_table[] = {
/* 0 C*/
30800, 29500, 28300, 27100,
26000, 24900, 23900, 22900, 22000, 21100, 20300, 19400, 18700, 17900,
17200, 16500, 15900, 15300, 14700, 14100, 13600, 13100, 12600, 12100,
11600, 11200, 10800, 10400, 10000, 9630,   9280,   8950,   8620,   8310,
8020,   7730,   7460,   7200,   6950,   6710,   6470,   6250,   6040,   5830,
5640,   5450,   5260,   5090,   4920,   4760,   4600,   4450,   4310,   4170,
4040,   3910,   3790,   3670,   3550
};

static struct twl4030_bci_platform_data sdp3430_bci_data = {
      .battery_tmp_tbl = sdp3430_batt_table,
      .tblsize = ARRAY_SIZE(sdp3430_batt_table),
};

static struct platform_device twl4030_bci_battery_device = {
	.name           = "twl4030-bci-battery",
	.id             = -1,
	.dev            = {
		.platform_data  = &sdp3430_bci_data,
	},
	.num_resources  = 0,
};

void __init twl4030_bci_battery_init(void)
{
	(void) platform_device_register(&twl4030_bci_battery_device);
}
#else
void __init twl4030_bci_battery_init(void)
{
}
#endif
