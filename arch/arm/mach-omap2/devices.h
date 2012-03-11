/*
 * arch/arm/mach-omap2/devices.h
 *
 * OMAP2 platform device setup/initialization
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __ARCH_ARM_MACH_OMAP_DEVICES_H
#define __ARCH_ARM_MACH_OMAP_DEVICES_H

struct isp_platform_data;

int omap3_init_camera(struct isp_platform_data *pdata);

int __init am335x_register_mcasp(struct snd_platform_data *pdata, int ctrl_nr);
extern int __init am33xx_register_tsc(struct tsc_data *pdata);
extern int __init am33xx_register_ecap(int id,
		struct pwmss_platform_data *pdata);
extern int __init am33xx_register_ehrpwm(int id,
		struct pwmss_platform_data *pdata);
extern int __init omap_init_elm(void);

#endif
