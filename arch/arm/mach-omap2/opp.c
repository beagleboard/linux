/*
 * OMAP SoC specific OPP wrapper function
 *
 * Copyright (C) 2009-2010 Texas Instruments Incorporated - http://www.ti.com/
 *	Nishanth Menon
 *	Kevin Hilman
 * Copyright (C) 2010 Nokia Corporation.
 *      Eduardo Valentin
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
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pm_opp.h>
#include <linux/cpu.h>

#include "omap_device.h"

#include "omap_opp_data.h"

/* Temp variable to allow multiple calls */
static u8 __initdata omap_table_init;

/*
 * opp_def_list_enable_opp() - enable opp by hwmod_name and frequency
 * @list:	opp default list for this silicon
 * @size:	number of opp entries for this silicon
 * @hwmod_name: hwmod for which the provided opp_freq exists
 * @opp_freq:	frequency for which the state should be updated
 * @state:	new state to set for opp availability
 */
int __init opp_def_list_enable_opp(struct omap_opp_def *list,
				   unsigned int size,
				   const char *hwmod_name,
				   unsigned long opp_freq, bool state)
{
	int i;

	for (i = 0; i < size; i++) {
		struct omap_opp_def *entry = &list[i];

		if (entry->hwmod_name == hwmod_name &&
		    entry->freq == opp_freq) {
			entry->default_available = state;
			return 0;
		}
	}
	WARN(1, "Unable to find opp for %s, frequency %ld\n",
	     hwmod_name, opp_freq);
	return -EINVAL;
}

/**
 * opp_def_list_update_voltage_opp() - update opp u_volt entry by hwmod_name
 *				       and frequency
 * @list:	opp default list for this silicon
 * @size:	number of opp entries for this silicon
 * @hwmod_name: hwmod for which the provided opp_freq exists
 * @opp_freq:	frequency for which the voltage should be updated
 * @u_volt:	new voltage value for the provided opp
 */
int __init opp_def_list_update_opp_voltage(struct omap_opp_def *list,
					   unsigned int size,
					   const char *hwmod_name,
					   unsigned long opp_freq,
					   unsigned long u_volt)
{
	int i;

	for (i = 0; i < size; i++) {
		struct omap_opp_def *entry = &list[i];

		if (entry->hwmod_name == hwmod_name &&
		    entry->freq == opp_freq) {
			entry->u_volt = u_volt;
			return 0;
		}
	}
	WARN(1, "Unable to find opp for %s, frequency %ld\n",
	     hwmod_name, opp_freq);
	return -EINVAL;
}

/**
 * omap_init_opp_table() - Initialize opp table as per the CPU type
 * @opp_def:		opp default list for this silicon
 * @opp_def_size:	number of opp entries for this silicon
 *
 * Register the initial OPP table with the OPP library based on the CPU
 * type. This is meant to be used only by SoC specific registration.
 */
int __init omap_init_opp_table(struct omap_opp_def *opp_def,
		u32 opp_def_size)
{
	int i, r;

	if (!opp_def || !opp_def_size) {
		pr_err("%s: invalid params!\n", __func__);
		return -EINVAL;
	}

	/*
	 * Initialize only if not already initialized even if the previous
	 * call failed, because, no reason we'd succeed again.
	 */
	if (omap_table_init)
		return -EEXIST;
	omap_table_init = 1;

	/* Lets now register with OPP library */
	for (i = 0; i < opp_def_size; i++, opp_def++) {
		struct omap_hwmod *oh;
		struct device *dev;

		if (!opp_def->hwmod_name) {
			pr_err("%s: NULL name of omap_hwmod, failing [%d].\n",
				__func__, i);
			return -EINVAL;
		}

		if (!strncmp(opp_def->hwmod_name, "mpu", 3)) {
			/* 
			 * All current OMAPs share voltage rail and
			 * clock source, so CPU0 is used to represent
			 * the MPU-SS.
			 */
			dev = get_cpu_device(0);
		} else {
			oh = omap_hwmod_lookup(opp_def->hwmod_name);
			if (!oh || !oh->od) {
				pr_debug("%s: no hwmod or odev for %s, [%d] cannot add OPPs.\n",
					 __func__, opp_def->hwmod_name, i);
				continue;
			}
			dev = &oh->od->pdev->dev;
		}

		r = dev_pm_opp_add(dev, opp_def->freq, opp_def->u_volt);
		if (r) {
			dev_err(dev, "%s: add OPP %ld failed for %s [%d] result=%d\n",
				__func__, opp_def->freq,
				opp_def->hwmod_name, i, r);
		} else {
			if (!opp_def->default_available)
				r = dev_pm_opp_disable(dev, opp_def->freq);
			if (r)
				dev_err(dev, "%s: disable %ld failed for %s [%d] result=%d\n",
					__func__, opp_def->freq,
					opp_def->hwmod_name, i, r);
		}
	}

	return 0;
}
