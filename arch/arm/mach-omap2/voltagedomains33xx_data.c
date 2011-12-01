/*
 * AM33XX voltage domain data
 *
 * Copyright (C) 2011 Texas Instruments, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/kernel.h>
#include <linux/init.h>

#include "voltage.h"

static struct voltagedomain am33xx_voltdm_mpu = {
	.name		= "mpu",
	.scalable	= false,
};

static struct voltagedomain am33xx_voltdm_core = {
	.name		= "core",
	.scalable	= false,
};

static struct voltagedomain am33xx_voltdm_rtc = {
	.name		= "rtc",
	.scalable	= false,
};

static struct voltagedomain *voltagedomains_am33xx[] __initdata = {
	&am33xx_voltdm_mpu,
	&am33xx_voltdm_core,
	&am33xx_voltdm_rtc,
	NULL,
};

void __init am33xx_voltagedomains_init(void)
{
	voltdm_init(voltagedomains_am33xx);
}
